//! mavsim — fake autopilot for testing the FlyingSquirrel anti-spoof
//! detector without a real ArduPilot/PX4 SITL install.
//!
//! Emits MAVLink GPS_RAW_INT + HIGHRES_IMU derived from a ground-truth
//! trajectory + optional spoof attack overlay. Listens for incoming
//! COMMAND_LONG (e.g. MAV_CMD_NAV_RETURN_TO_LAUNCH) and PARAM_SET messages
//! and prints them to stderr — that is the autopilot "reacting".

use clap::{Parser, ValueEnum};
use flyingsquirrel::error::FsError;
use flyingsquirrel::mav::monitor::ARDUCOPTER_MODE_RTL;
use flyingsquirrel::nav::R_EARTH_M;
use flyingsquirrel::sim::trajectory::{LinearNorth, TrajectoryGenerator};
use mavlink::common::{
    GpsFixType, HighresImuUpdatedFlags, MavAutopilot, MavCmd, MavMessage, MavModeFlag, MavResult,
    MavState, MavType as MmMavType, COMMAND_ACK_DATA, GPS_RAW_INT_DATA, HEARTBEAT_DATA,
    HIGHRES_IMU_DATA, PARAM_VALUE_DATA,
};
use mavlink::peek_reader::PeekReader;
use mavlink::{MavHeader, MavlinkVersion};
use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;
use std::io::Cursor;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::net::UdpSocket;
use tokio::sync::Mutex;
use tracing_subscriber::EnvFilter;

/// ArduCopter GUIDED mode — what we start in.
const MODE_GUIDED: u32 = 4;

const G: f64 = 9.806_65;
const DEFAULT_SEED: u64 = 0xF1F5_5184_0000_0000;

#[derive(Debug, Clone, Copy, ValueEnum)]
enum Scenario {
    Clean,
    SuddenJump,
    GradualDrift,
}

#[derive(Debug, Parser)]
#[command(name = "mavsim", about = "Fake autopilot emitting MAVLink GPS/IMU.")]
struct Cli {
    #[arg(long, value_enum, default_value_t = Scenario::GradualDrift)]
    scenario: Scenario,
    #[arg(long, default_value_t = 60)]
    duration: u32,
    /// Address to bind to (we receive autopilot-bound commands here).
    /// AUDIT I-02 fix: default to loopback. mavsim emits ATTACK trajectories
    /// (SuddenJump, GradualDrift) by design — binding to all interfaces made
    /// a test fixture an open MAVLink endpoint on the LAN. Use an explicit
    /// `--bind 0.0.0.0:14550` only for deliberate cross-host SITL setups.
    #[arg(long, default_value = "127.0.0.1:14550")]
    bind: SocketAddr,
    /// Address of the detector (we send GPS/IMU here).
    #[arg(long, default_value = "127.0.0.1:14551")]
    target: SocketAddr,
    #[arg(long, default_value_t = DEFAULT_SEED)]
    seed: u64,
    #[arg(long, default_value = "info")]
    log_level: String,
}

#[tokio::main]
async fn main() -> Result<(), FsError> {
    let cli = Cli::parse();

    let filter = EnvFilter::try_from_default_env()
        .or_else(|_| EnvFilter::try_new(&cli.log_level))
        .unwrap_or_else(|_| EnvFilter::new("info"));
    tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_writer(std::io::stderr)
        .init();

    let socket = Arc::new(UdpSocket::bind(cli.bind).await?);
    tracing::info!(local = %cli.bind, target = %cli.target, "mavsim: bound");

    let seq = Arc::new(Mutex::new(0u8));
    // Shared autopilot state: starts in GUIDED, transitions to RTL when the
    // detector sends MAV_CMD_NAV_RETURN_TO_LAUNCH. The HEARTBEAT emitter reads
    // this; the command listener writes it.
    let current_mode = Arc::new(AtomicU32::new(MODE_GUIDED));

    // HEARTBEAT emitter task — 1 Hz, the standard rate.
    let hb_socket = socket.clone();
    let hb_seq = seq.clone();
    let hb_mode = current_mode.clone();
    let hb_target = cli.target;
    tokio::spawn(async move {
        let mut ticker = tokio::time::interval(Duration::from_secs(1));
        loop {
            ticker.tick().await;
            let mode = hb_mode.load(Ordering::Acquire);
            let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                custom_mode: mode,
                mavtype: MmMavType::MAV_TYPE_QUADROTOR,
                autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                    | MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED,
                system_status: MavState::MAV_STATE_ACTIVE,
                mavlink_version: 3,
            });
            send_msg(&hb_socket, hb_target, &msg, &hb_seq).await;
        }
    });

    // Listener task: print incoming commands, ack RTL/PARAM_SET, and update
    // our local mode so the detector's HEARTBEAT watcher sees the transition.
    let recv_socket = socket.clone();
    let recv_seq = seq.clone();
    let recv_mode = current_mode.clone();
    let recv_target = cli.target;
    tokio::spawn(async move {
        let mut buf = vec![0u8; 2048];
        loop {
            let (n, src) = match recv_socket.recv_from(&mut buf).await {
                Ok(v) => v,
                Err(e) => {
                    // Windows reports ICMP unreachable as a UDP recv error; ignore.
                    if matches!(e.raw_os_error(), Some(10054 | 10052 | 10040))
                        || matches!(
                            e.kind(),
                            std::io::ErrorKind::ConnectionReset
                                | std::io::ErrorKind::ConnectionRefused
                                | std::io::ErrorKind::HostUnreachable
                        )
                    {
                        tracing::debug!(error = %e, "mavsim transient recv error, continuing");
                        continue;
                    }
                    tracing::warn!(error = %e, "mavsim fatal recv error");
                    break;
                }
            };
            let cur = Cursor::new(buf[..n].to_vec());
            let mut reader = PeekReader::new(cur);
            match mavlink::read_versioned_msg::<MavMessage, _>(&mut reader, MavlinkVersion::V2) {
                Ok((_hdr, msg)) => {
                    handle_incoming(src, &msg, &recv_socket, recv_target, &recv_seq, &recv_mode)
                        .await
                }
                Err(e) => tracing::debug!(error = ?e, "mavsim parse failed"),
            }
        }
    });

    // Build trajectory + spoof.
    let origin = (40.0_f64, -100.0_f64, 0.0_f64);
    let traj = LinearNorth { speed_mps: 8.0 };
    let (spoof_at_s, spoof_n_rate, spoof_e_rate, spoof_n_jump, spoof_e_jump) = match cli.scenario {
        Scenario::Clean => (f64::INFINITY, 0.0, 0.0, 0.0, 0.0),
        Scenario::SuddenJump => (30.0, 0.0, 0.0, 0.0, 200.0),
        Scenario::GradualDrift => (20.0, 0.0, 1.0, 0.0, 0.0),
    };

    let mut rng = ChaCha8Rng::seed_from_u64(cli.seed);
    let imu_dt = 0.01_f64; // 100 Hz
    let gps_dt = 1.0_f64;
    let mut t_imu = 0.0_f64;
    let mut t_gps = 0.5_f64; // first GPS at 0.5s
    let deadline = cli.duration as f64;

    let start_wall = tokio::time::Instant::now();
    while t_imu <= deadline + 1e-9 {
        // Sleep until t_imu wall-clock.
        let target_wall = start_wall + Duration::from_secs_f64(t_imu);
        tokio::time::sleep_until(target_wall).await;

        // Emit one IMU sample.
        let gt = traj.sample(t_imu);
        // Specific force f = R(q)ᵀ·(a_ned − g_ned), g_ned = [0,0,+g]; identity
        // attitude here. At rest → [0,0,-g], matching a real ArduPilot/PX4
        // SCALED_IMU/HIGHRES_IMU stream (this fixture stands in for one).
        let body_accel = [gt.acc_n as f32, gt.acc_e as f32, (gt.acc_d - G) as f32];
        let gyro = [gt.gyro_n as f32, gt.gyro_e as f32, gt.gyro_d as f32];

        let imu_msg = MavMessage::HIGHRES_IMU(HIGHRES_IMU_DATA {
            time_usec: (t_imu * 1e6) as u64,
            xacc: body_accel[0] + gauss(&mut rng) as f32 * 0.001,
            yacc: body_accel[1] + gauss(&mut rng) as f32 * 0.001,
            zacc: body_accel[2] + gauss(&mut rng) as f32 * 0.001,
            xgyro: gyro[0] + gauss(&mut rng) as f32 * 0.0001,
            ygyro: gyro[1] + gauss(&mut rng) as f32 * 0.0001,
            zgyro: gyro[2] + gauss(&mut rng) as f32 * 0.0001,
            xmag: 0.0,
            ymag: 0.0,
            zmag: 0.0,
            abs_pressure: 0.0,
            diff_pressure: 0.0,
            pressure_alt: 0.0,
            temperature: 25.0,
            fields_updated: HighresImuUpdatedFlags::DEFAULT,
        });
        send_msg(&socket, cli.target, &imu_msg, &seq).await;

        // Emit GPS at gps_dt cadence (after start_after_s).
        if t_imu + 1e-9 >= t_gps && t_gps <= deadline + 1e-9 {
            let mut pos_n = gt.pos_n;
            let mut pos_e = gt.pos_e;
            if t_gps >= spoof_at_s {
                pos_n += spoof_n_jump + spoof_n_rate * (t_gps - spoof_at_s);
                pos_e += spoof_e_jump + spoof_e_rate * (t_gps - spoof_at_s);
            }
            // Add tiny GPS noise.
            pos_n += gauss(&mut rng) * 0.3;
            pos_e += gauss(&mut rng) * 0.3;

            let lat0 = origin.0;
            let lon0 = origin.1;
            let dlat_deg = (pos_n / R_EARTH_M).to_degrees();
            let dlon_deg = (pos_e / (R_EARTH_M * lat0.to_radians().cos())).to_degrees();
            let lat_e7 = ((lat0 + dlat_deg) * 1e7) as i32;
            let lon_e7 = ((lon0 + dlon_deg) * 1e7) as i32;
            let alt_mm = ((origin.2 - gt.pos_d) * 1e3) as i32;
            let speed = ((gt.vel_n * gt.vel_n + gt.vel_e * gt.vel_e).sqrt() * 100.0) as u16;
            let course_deg = ((gt.vel_e.atan2(gt.vel_n).to_degrees() + 360.0) % 360.0) * 100.0;

            let gps_msg = MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
                time_usec: (t_gps * 1e6) as u64,
                lat: lat_e7,
                lon: lon_e7,
                alt: alt_mm,
                eph: 90, // HDOP = 0.9
                epv: 130,
                vel: speed,
                cog: course_deg as u16,
                fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
                satellites_visible: 12,
            });
            send_msg(&socket, cli.target, &gps_msg, &seq).await;

            t_gps += gps_dt;
        }

        t_imu += imu_dt;
    }
    tracing::info!("mavsim: scenario complete");
    // Linger for a second so trailing commands have a chance to arrive.
    tokio::time::sleep(Duration::from_secs(1)).await;
    Ok(())
}

async fn handle_incoming(
    src: SocketAddr,
    msg: &MavMessage,
    socket: &Arc<UdpSocket>,
    target: SocketAddr,
    seq: &Arc<Mutex<u8>>,
    mode: &Arc<AtomicU32>,
) {
    match msg {
        MavMessage::COMMAND_LONG(c) => {
            if c.command == MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH {
                tracing::warn!(from = %src, "AUTOPILOT REACTION: RTL command received -> engaging Return-to-Launch");
                // Transition mode so the detector's HEARTBEAT-derived monitor
                // observes the autopilot moving into RTL.
                mode.store(ARDUCOPTER_MODE_RTL, Ordering::Release);
                // ACK so the detector knows the autopilot received it.
                let ack = MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
                    command: c.command,
                    result: MavResult::MAV_RESULT_ACCEPTED,
                });
                send_msg(socket, target, &ack, seq).await;
            } else {
                tracing::info!(from = %src, command = ?c.command, "command_long received");
                // Generic ACK for any other command so the detector can match.
                let ack = MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
                    command: c.command,
                    result: MavResult::MAV_RESULT_ACCEPTED,
                });
                send_msg(socket, target, &ack, seq).await;
            }
        }
        MavMessage::PARAM_SET(p) => {
            let name = std::str::from_utf8(&p.param_id)
                .unwrap_or("?")
                .trim_end_matches('\0');
            tracing::warn!(
                from = %src,
                param = name,
                value = p.param_value,
                "AUTOPILOT REACTION: PARAM_SET received -> disabling parameter"
            );
            // Echo PARAM_VALUE with the applied value, exactly as a real
            // autopilot does. This is what lets the detector CONFIRM the
            // GPS-disable param actually took effect (closed-loop sever
            // read-back) instead of trusting best-effort UDP.
            let pv = MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
                param_id: p.param_id,
                param_value: p.param_value,
                param_type: p.param_type,
                param_count: 1,
                param_index: 0,
            });
            send_msg(socket, target, &pv, seq).await;
        }
        _ => {}
    }
}

async fn send_msg(
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
    if let Err(e) = mavlink::write_versioned_msg(&mut buf, MavlinkVersion::V2, header, msg) {
        tracing::warn!(error = ?e, "mavsim encode failed");
        return;
    }
    if let Err(e) = socket.send_to(&buf, target).await {
        tracing::warn!(error = %e, "mavsim send failed");
    }
}

fn gauss(rng: &mut ChaCha8Rng) -> f64 {
    let u1: f64 = rng.gen_range(1e-9..1.0);
    let u2: f64 = rng.gen_range(0.0..1.0);
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}
