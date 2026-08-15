//! Fuzz the MAVLink datagram decode chain: arbitrary UDP payload → version
//! sniff → frame parse → typed conversion → plausibility gate → clock align.
//!
//! This mirrors the per-datagram work the listener does in `mav::spawn_listener`
//! (see the `read_versioned_msg` call site), composed from the SAME public
//! functions it calls — not a reimplementation. What is deliberately excluded
//! is the loop's stateful side-effect layer (source-IP/sysid filtering, the
//! source-port lock, monitor bookkeeping): those need real sockets and a
//! `MavMonitor`, and their logic is covered by the integration suites.
//!
//! The threat this covers: everything upstream of the plausibility gate runs
//! on bytes from the network. The source-IP and sysid filters are the first
//! line, but both are spoofable on a shared link — which is precisely why the
//! decode path must survive hostile input on its own.
//!
//! Properties asserted:
//!
//! 1. No panic anywhere in the chain, for any byte string.
//! 2. Whatever the plausibility gates ACCEPT is finite (the contract the
//!    detector's residual arithmetic depends on).
//! 3. Clock alignment stays bounded even when the frame's timestamp field is
//!    hostile (the `ClockAligner` overflow class, reached here through a real
//!    parsed frame rather than a synthetic call).

#![no_main]

use libfuzzer_sys::fuzz_target;
use mavlink::common::MavMessage;
use mavlink::peek_reader::PeekReader;
use std::io::Cursor;
use std::time::Duration;

const MAX_SKEW: Duration = Duration::from_secs(2);

fuzz_target!(|datagrams: Vec<Vec<u8>>| {
    // Per-stream aligners, exactly as the listener holds them: GPS and IMU
    // clocks may use different bases, so they never share an anchor.
    let mut gps_aligner = flyingsquirrel::mav::ClockAligner::default();
    let mut imu_aligner = flyingsquirrel::mav::ClockAligner::default();
    let base = tokio::time::Instant::now();
    let mut arrival_us: u64 = 0;

    for datagram in datagrams {
        if datagram.is_empty() {
            continue;
        }
        arrival_us = arrival_us.saturating_add(1000);
        let arrival = base + Duration::from_micros(arrival_us);

        // Version is sniffed from the leading magic byte (audit V-MAVVER:
        // hard-coding V2 made the detector deaf on ArduPilot's default v1).
        let version = flyingsquirrel::mav::mavlink_version_of(datagram[0]);
        let mut reader = PeekReader::new(Cursor::new(&datagram[..]));
        let Ok((_hdr, msg)) = mavlink::read_versioned_msg::<MavMessage, _>(&mut reader, version)
        else {
            continue;
        };

        if let Some(fix) = flyingsquirrel::mav::gps_from_msg(&msg) {
            if flyingsquirrel::mav::gps_fix_is_plausible(&fix) {
                assert!(
                    fix.lat_deg.is_finite() && fix.lon_deg.is_finite() && fix.alt_m.is_finite(),
                    "gate accepted a non-finite GPS fix: {fix:?}"
                );
            }
            let aligned = gps_aligner.align(flyingsquirrel::mav::gps_sensor_us(&msg), arrival);
            assert!(
                aligned.saturating_duration_since(arrival) <= MAX_SKEW,
                "GPS stamp ran ahead of arrival by {:?}",
                aligned.saturating_duration_since(arrival)
            );
        }

        if let Some(sample) = flyingsquirrel::mav::imu_from_msg(&msg) {
            if flyingsquirrel::mav::imu_sample_is_plausible(&sample) {
                assert!(
                    sample.accel_mps2.iter().all(|v| v.is_finite())
                        && sample.gyro_rps.iter().all(|v| v.is_finite()),
                    "gate accepted a non-finite IMU sample: {sample:?}"
                );
            }
            let aligned = imu_aligner.align(flyingsquirrel::mav::imu_sensor_us(&msg), arrival);
            assert!(
                aligned.saturating_duration_since(arrival) <= MAX_SKEW,
                "IMU stamp ran ahead of arrival by {:?}",
                aligned.saturating_duration_since(arrival)
            );
        }

        // HEARTBEAT custom_mode decode: the PX4 split had a real byte-order
        // bug (main/sub reversed) that self-consistent round-trip unit tests
        // could not see. Exercise it on hostile bit patterns too.
        if let MavMessage::HEARTBEAT(hb) = &msg {
            let _ = flyingsquirrel::mav::monitor::px4_split_mode(hb.custom_mode);
        }
    }
});
