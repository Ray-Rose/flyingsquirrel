//! End-to-end MAVLink loopback test.
//!
//! Spawns a fake autopilot that emits MAVLink GPS_RAW_INT + HIGHRES_IMU with
//! a sudden-jump spoof, runs the FlyingSquirrel detector against it via UDP,
//! and asserts that the autopilot receives `MAV_CMD_NAV_RETURN_TO_LAUNCH`
//! and a `PARAM_SET GPS_TYPE=0` within the bounded test duration. The fake
//! autopilot ACKs the RTL (→ `ActionAcked` read-back) and echoes `PARAM_VALUE`
//! for the sever PARAM_SET (→ the typed/causal/multi-echo sever read-back
//! CONFIRMS, so no CRITICAL `ActionFailed`).
//!
//! Uses fixed UDP ports on 127.0.0.1 — if you run this concurrently with
//! something else on those ports it'll fail. The detector dwell is
//! shortened so the test completes in well under 20s of wall clock.

use flyingsquirrel::action::FlightController;
use flyingsquirrel::detect::DetectConfig;
use flyingsquirrel::fusion::FusionConfig;
use flyingsquirrel::ingest::{GpsSource, ImuSource};
use flyingsquirrel::mav::controller::MavFlightController;
use flyingsquirrel::mav::monitor::{VehicleProfile, ARDUCOPTER_MODE_RTL};
use flyingsquirrel::mav::source::{MavGpsSource, MavImuSource};
use flyingsquirrel::mav::MavLink;
use flyingsquirrel::nav::R_EARTH_M;
use flyingsquirrel::runtime::spawn_all;
use flyingsquirrel::types::SpoofKind;
use mavlink::common::{
    GpsFixType, HighresImuUpdatedFlags, MavAutopilot, MavCmd, MavMessage, MavModeFlag, MavResult,
    MavState, MavType as MmMavType, COMMAND_ACK_DATA, GPS_RAW_INT_DATA, HEARTBEAT_DATA,
    HIGHRES_IMU_DATA, PARAM_VALUE_DATA,
};
use mavlink::peek_reader::PeekReader;
use mavlink::{MavHeader, MavlinkVersion};
use std::io::Cursor;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;
use std::sync::Mutex as StdMutex;
use std::time::Duration;
use tokio::net::UdpSocket;
use tokio::sync::Mutex;
use tokio::time::Instant;

const MODE_GUIDED: u32 = 4;

const G: f64 = 9.806_65;
const AUTOPILOT_ADDR: &str = "127.0.0.1:24600";
const DETECTOR_ADDR: &str = "127.0.0.1:24601";

#[derive(Debug, Default)]
struct AutopilotReactions {
    rtl_count: u32,
    sever_gps_count: u32,
}

#[tokio::test]
async fn mavlink_loopback_detects_jump_and_commands_rtl() {
    let autopilot_addr: SocketAddr = AUTOPILOT_ADDR.parse().unwrap();
    let detector_addr: SocketAddr = DETECTOR_ADDR.parse().unwrap();

    // --- Fake autopilot side ---
    let autopilot_socket = Arc::new(UdpSocket::bind(autopilot_addr).await.unwrap());
    let reactions = Arc::new(StdMutex::new(AutopilotReactions::default()));
    // Shared autopilot mode: starts GUIDED, transitions to RTL when we ACK
    // the detector's MAV_CMD_NAV_RETURN_TO_LAUNCH. The HEARTBEAT emitter
    // reads this; the listener writes it.
    let autopilot_mode = Arc::new(AtomicU32::new(MODE_GUIDED));
    let ap_seq = Arc::new(Mutex::new(0u8));

    // HEARTBEAT emitter from the fake autopilot side. 4 Hz to make the
    // mode-transition observable within the verification window without
    // padding test duration.
    let hb_socket = autopilot_socket.clone();
    let hb_seq = ap_seq.clone();
    let hb_mode = autopilot_mode.clone();
    let hb_handle = tokio::spawn(async move {
        let mut ticker = tokio::time::interval(Duration::from_millis(250));
        loop {
            ticker.tick().await;
            let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                custom_mode: hb_mode.load(Ordering::Acquire),
                mavtype: MmMavType::MAV_TYPE_QUADROTOR,
                autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                    | MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED,
                system_status: MavState::MAV_STATE_ACTIVE,
                mavlink_version: 3,
            });
            send_one(&hb_socket, detector_addr, &msg, &hb_seq).await;
        }
    });

    // Listener: capture commands from the detector + ACK them + flip mode
    // to RTL when MAV_CMD_NAV_RETURN_TO_LAUNCH arrives.
    let ap_listen_socket = autopilot_socket.clone();
    let ap_reactions = reactions.clone();
    let ap_mode = autopilot_mode.clone();
    let ap_listen_seq = ap_seq.clone();
    let ap_listen = tokio::spawn(async move {
        let mut buf = vec![0u8; 2048];
        loop {
            let (n, _src) = match ap_listen_socket.recv_from(&mut buf).await {
                Ok(v) => v,
                Err(_) => break,
            };
            let cur = Cursor::new(buf[..n].to_vec());
            let mut reader = PeekReader::new(cur);
            if let Ok((_hdr, msg)) =
                mavlink::read_versioned_msg::<MavMessage, _>(&mut reader, MavlinkVersion::V2)
            {
                match msg {
                    MavMessage::COMMAND_LONG(c)
                        if c.command == MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH =>
                    {
                        {
                            let mut r = ap_reactions.lock().unwrap();
                            r.rtl_count += 1;
                        }
                        // Transition mode so the detector's monitor observes RTL,
                        // and ACK so the detector knows we got the command.
                        ap_mode.store(ARDUCOPTER_MODE_RTL, Ordering::Release);
                        let ack = MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
                            command: c.command,
                            result: MavResult::MAV_RESULT_ACCEPTED,
                        });
                        send_one(&ap_listen_socket, detector_addr, &ack, &ap_listen_seq).await;
                    }
                    MavMessage::PARAM_SET(p) => {
                        let name = std::str::from_utf8(&p.param_id)
                            .unwrap_or("")
                            .trim_end_matches('\0');
                        if name == "GPS_TYPE" && p.param_value == 0.0 {
                            let mut r = ap_reactions.lock().unwrap();
                            r.sever_gps_count += 1;
                        }
                        // Echo PARAM_VALUE back, exactly as a real autopilot (and
                        // mavsim) does. This is what lets the detector's
                        // closed-loop sever read-back CONFIRM the GPS-disable
                        // param was applied (typed, causal, multi-echo dwell);
                        // each of the controller's 3 PARAM_SET retries gets an
                        // echo, so the dwell of 2 is met. Without these echoes
                        // the read-back correctly stays unconfirmed (CRITICAL).
                        let pv = MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
                            param_id: p.param_id,
                            param_value: p.param_value,
                            param_type: p.param_type,
                            param_count: 1,
                            param_index: 0,
                        });
                        send_one(&ap_listen_socket, detector_addr, &pv, &ap_listen_seq).await;
                    }
                    _ => {}
                }
            }
        }
    });

    // Emitter: send GPS+IMU at high rate (faster than real-time) with sudden-jump
    // spoof at t=1.5s scenario time. We pace IMU at 1ms wall = effectively 100x.
    let ap_emit_socket = autopilot_socket.clone();
    let seq = Arc::new(Mutex::new(0u8));
    let emit_handle = tokio::spawn(async move {
        let origin = (40.0_f64, -100.0_f64, 0.0_f64);
        let speed_mps = 8.0_f64;
        let imu_dt = 0.01_f64;
        let gps_dt = 1.0_f64;
        let spoof_at_s = 1.5_f64;
        let spoof_east_m = 200.0_f64;
        let start = Instant::now();
        // Real-time pacing (1× wall = 1× sim) keeps the residual buffer
        // window math correct. The detector's dwell is shortened in
        // FusionConfig below to keep total wall time small.
        let mut t_imu = 0.0_f64;
        let mut t_gps = 0.5_f64;
        let deadline = 12.0_f64;
        while t_imu <= deadline + 1e-9 {
            tokio::time::sleep_until(start + Duration::from_secs_f64(t_imu)).await;
            // IMU: constant velocity, level body -> specific force = (0, 0, -g),
            // as a real ArduPilot/PX4 HIGHRES_IMU emits at rest/level.
            let imu_msg = MavMessage::HIGHRES_IMU(HIGHRES_IMU_DATA {
                time_usec: (t_imu * 1e6) as u64,
                xacc: 0.0,
                yacc: 0.0,
                zacc: -G as f32,
                xgyro: 0.0,
                ygyro: 0.0,
                zgyro: 0.0,
                xmag: 0.0,
                ymag: 0.0,
                zmag: 0.0,
                abs_pressure: 0.0,
                diff_pressure: 0.0,
                pressure_alt: 0.0,
                temperature: 25.0,
                fields_updated: HighresImuUpdatedFlags::DEFAULT,
            });
            send_one(&ap_emit_socket, detector_addr, &imu_msg, &seq).await;

            if t_imu + 1e-9 >= t_gps && t_gps <= deadline + 1e-9 {
                let pos_n = speed_mps * t_gps;
                let mut pos_e = 0.0_f64;
                if t_gps >= spoof_at_s {
                    pos_e += spoof_east_m;
                }
                let lat0 = origin.0;
                let lon0 = origin.1;
                let dlat_deg = (pos_n / R_EARTH_M).to_degrees();
                let dlon_deg = (pos_e / (R_EARTH_M * lat0.to_radians().cos())).to_degrees();
                let lat_e7 = ((lat0 + dlat_deg) * 1e7) as i32;
                let lon_e7 = ((lon0 + dlon_deg) * 1e7) as i32;
                let course_deg = 0.0_f64; // pointing +N
                let gps_msg = MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
                    time_usec: (t_gps * 1e6) as u64,
                    lat: lat_e7,
                    lon: lon_e7,
                    alt: 0,
                    eph: 90,
                    epv: 130,
                    vel: (speed_mps * 100.0) as u16,
                    cog: (course_deg * 100.0) as u16,
                    fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
                    satellites_visible: 12,
                });
                send_one(&ap_emit_socket, detector_addr, &gps_msg, &seq).await;
                t_gps += gps_dt;
            }
            t_imu += imu_dt;
        }
    });

    // --- Detector side ---
    let link = Arc::new(MavLink::bind(detector_addr, autopilot_addr).await.unwrap());
    let (det_listener, gps_rx, imu_rx, _other_rx) = link.spawn_listener();
    let controller: Arc<dyn FlightController> = Arc::new(
        MavFlightController::new(link.clone(), None, 1, 1, VehicleProfile::ArduCopter)
            .await
            .unwrap(),
    );
    let gps: Box<dyn GpsSource> = Box::new(MavGpsSource { rx: gps_rx });
    let imu: Box<dyn ImuSource> = Box::new(MavImuSource { rx: imu_rx });

    // Tighten the detector dwell so the test completes faster.
    let cfg = FusionConfig {
        detect: DetectConfig {
            suspicious_to_spoofed_dwell_s: 3.0,
            ..Default::default()
        },
        ..Default::default()
    };
    let mav_preflight = Some(flyingsquirrel::runtime::MavPreflight {
        monitor: link.monitor.clone(),
        expected_autopilot: flyingsquirrel::mav::monitor::expected_autopilot_byte(
            VehicleProfile::ArduCopter,
        ),
    });
    let handles = spawn_all(gps, imu, controller, cfg, mav_preflight, None).unwrap();

    // Spawn the heartbeat watchdog so LinkDown/LinkRestored events flow
    // through the same broadcast bus.
    let watchdog = link.spawn_heartbeat_watchdog(
        handles.events_tx.clone(),
        handles.boot,
        Duration::from_secs(2),
        Duration::from_millis(200),
    );

    // Subscribe to events to assert closed-loop behavior.
    let mut events_rx = handles.events_tx.subscribe();
    let observed = Arc::new(StdMutex::new(Vec::<SpoofKind>::new()));
    let observed_for_task = observed.clone();
    let events_handle = tokio::spawn(async move {
        while let Ok(ev) = events_rx.recv().await {
            observed_for_task.lock().unwrap().push(ev.kind);
        }
    });

    // Wait for the autopilot to receive both reactions, with a generous timeout.
    // We expect repeated sends (controller retries x3 at 100ms spacing), so
    // wait for at least 2 of each to land — that proves both initial send AND
    // retry path actually fired.
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut got_both = false;
    while Instant::now() < deadline {
        tokio::time::sleep(Duration::from_millis(200)).await;
        let r = reactions.lock().unwrap();
        if r.rtl_count >= 2 && r.sever_gps_count >= 2 {
            got_both = true;
            break;
        }
    }
    // Grace period long enough for verify_rtb_engaged to:
    //   (a) observe the ACK matching MAV_CMD_NAV_RETURN_TO_LAUNCH,
    //   (b) see the autopilot's mode change AFTER sent_at,
    //   (c) accumulate the required dwell (3 × 200ms poll).
    // Real verification completes ~800ms after engage_rtb returns; pad to 2s.
    if got_both {
        tokio::time::sleep(Duration::from_millis(2000)).await;
    }

    let final_r = {
        let g = reactions.lock().unwrap();
        format!("rtl={}, sever_gps={}", g.rtl_count, g.sever_gps_count)
    };

    // Cleanup.
    handles.gps_ingest.abort();
    handles.imu_ingest.abort();
    handles.action.abort();
    det_listener.abort();
    emit_handle.abort();
    ap_listen.abort();
    hb_handle.abort();
    watchdog.abort();
    events_handle.abort();

    assert!(
        got_both,
        "autopilot never received both reactions ({})",
        final_r
    );

    // UDP-loss-resilience guarantee: the MavFlightController repeats each
    // safety-critical command N=3 times. Asserting we saw at least 2 copies
    // of each proves retry is actually firing, not just the first send.
    let r = reactions.lock().unwrap();
    assert!(
        r.rtl_count >= 2,
        "expected MavFlightController to repeat RTL (got {})",
        r.rtl_count
    );
    assert!(
        r.sever_gps_count >= 2,
        "expected MavFlightController to repeat PARAM_SET (got {})",
        r.sever_gps_count
    );

    // Closed-loop verification: the detector should have observed the
    // autopilot's HEARTBEAT (LinkRestored = first-heartbeat event) and
    // confirmed the post-RTL mode transition (ActionAcked).
    let kinds = observed.lock().unwrap();
    let saw_link_up = kinds.contains(&SpoofKind::LinkRestored);
    let saw_action_acked = kinds.contains(&SpoofKind::ActionAcked);
    let saw_action_unconfirmed = kinds.contains(&SpoofKind::ActionUnconfirmed);
    let saw_action_failed = kinds.contains(&SpoofKind::ActionFailed);
    assert!(
        saw_link_up,
        "expected LinkRestored event (heartbeat detected) — saw: {:?}",
        *kinds
    );
    assert!(
        saw_action_acked,
        "expected ActionAcked event (autopilot mode -> RTL) — saw: {:?}",
        *kinds
    );
    assert!(
        !saw_action_unconfirmed,
        "did not expect ActionUnconfirmed — saw: {:?}",
        *kinds
    );
    // Closed-loop SEVER read-back: now that the fake autopilot echoes
    // PARAM_VALUE for each PARAM_SET, the detector's typed/causal/multi-echo
    // dwell must CONFIRM the GPS-disable param took effect — i.e. NO CRITICAL
    // ActionFailed event (which is what an unconfirmed sever emits).
    assert!(
        !saw_action_failed,
        "expected sever read-back to confirm (no CRITICAL ActionFailed) — saw: {:?}",
        *kinds
    );
}

async fn send_one(
    socket: &Arc<UdpSocket>,
    target: SocketAddr,
    msg: &MavMessage,
    seq: &Arc<Mutex<u8>>,
) {
    let mut buf = Vec::with_capacity(280);
    let header = {
        let mut g = seq.lock().await;
        let h = MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: *g,
        };
        *g = g.wrapping_add(1);
        h
    };
    let _ = mavlink::write_versioned_msg(&mut buf, MavlinkVersion::V2, header, msg);
    let _ = socket.send_to(&buf, target).await;
}
