//! Realistic false-alarm characterization (audit S-09/S-10 + the nav-review
//! "long/dynamic flight false-fires" finding).
//!
//! The other scenarios fly `LinearNorth` with ~30x-below-realistic IMU noise
//! (accel σ=0.001, gyro σ=0.0001) — a detector-LOGIC test, not a false-alarm-
//! rate test. These run CLEAN flights (no spoof injected, so any detection is a
//! FALSE ALARM) and sweep the IMU error model.
//!
//! Established empirically by the tests below:
//!   * Logic is sound at low noise — `low_noise_clean_flight_holds` PASSES.
//!   * **Realistic IMU noise alone false-fires** (`realistic_*`, `#[ignore]`d):
//!     at consumer-MEMS σ (accel 0.03 m/s², gyro 0.002 rad/s), the dead-reckoned
//!     position random-walks ~10 m over 90 s. Because the residual is GPS minus
//!     the cumulative-since-anchor DR position, that wander is a PERSISTENT
//!     offset (not zero-mean), so the per-axis CUSUM ramps and latches Spoofed —
//!     i.e. it would sever GPS and RTL a perfectly healthy drone. A persistent
//!     accel bias and a sustained coordinated turn make it worse/faster.
//!
//! Root cause + fix: the cumulative-residual-since-anchor design treats honest
//! DR drift the same as a slow spoof. The fix is a sliding-window residual
//! (compare GPS-vs-DR DISPLACEMENT over a short window, which differences out
//! the accumulated DR error) and/or in-flight accel-bias estimation + GPS-aided
//! attitude for the turn case. This is a deliberate, separately-validated
//! redesign — see docs/threats.md. These `#[ignore]`d tests stay executable so
//! that fix can be validated by flipping their assertions. Run the
//! characterizations with: `cargo test --test scenario_realistic -- --ignored`.

mod common;

use common::TestController;
use flyingsquirrel::fusion::FusionConfig;
use flyingsquirrel::ingest::{GpsSource, ImuSource};
use flyingsquirrel::runtime::spawn_all;
use flyingsquirrel::sim::trajectory::{CircleHorizontal, LinearNorth, TrajectoryGenerator};
use flyingsquirrel::sim::{SyntheticGps, SyntheticImu};
use flyingsquirrel::types::{NavStateKind, SpoofKind, SpoofingEvent};
use std::sync::Arc;
use std::time::Duration;

#[allow(clippy::too_many_arguments)]
async fn run_clean_flight<T>(
    traj: T,
    duration_s: u32,
    gps_pos_sigma_m: f32,
    accel_sigma: f32,
    gyro_sigma: f32,
    accel_bias: [f32; 3],
    gyro_bias: [f32; 3],
    seed: u64,
) -> Vec<SpoofingEvent>
where
    T: TrajectoryGenerator + Send + Sync + Copy + 'static,
{
    let origin = (40.0, -100.0, 0.0);
    let imu = SyntheticImu {
        traj,
        rate_hz: 100.0,
        accel_sigma,
        gyro_sigma,
        gyro_bias,
        accel_bias,
        seed: seed.wrapping_add(1),
        duration_s: duration_s as f32 + 2.0,
    };
    let gps = SyntheticGps {
        traj,
        rate_hz: 5.0,
        pos_sigma_m: gps_pos_sigma_m,
        vel_sigma_mps: 0.1,
        origin,
        seed,
        duration_s: duration_s as f32,
        start_after_s: 0.5,
    };

    let controller = Arc::new(TestController::new());
    let events_handle = controller.events.clone();
    let gps_boxed: Box<dyn GpsSource> = Box::new(gps);
    let imu_boxed: Box<dyn ImuSource> = Box::new(imu);
    let handles = spawn_all(
        gps_boxed,
        imu_boxed,
        controller,
        FusionConfig::default(),
        None,
        None,
    )
    .unwrap();

    let ticks = (duration_s + 2) * 10;
    for _ in 0..ticks {
        tokio::time::advance(Duration::from_millis(100)).await;
        tokio::task::yield_now().await;
    }
    let _ = flyingsquirrel::runtime::drain_pending_bg(
        handles.pending_bg.clone(),
        Duration::from_secs(5),
    )
    .await;
    for _ in 0..5 {
        tokio::task::yield_now().await;
    }
    let evs = events_handle.lock().unwrap().clone();
    evs
}

fn spoofed(tag: &str, events: &[SpoofingEvent]) -> bool {
    let drift = events.iter().filter(|e| e.kind == SpoofKind::Drift).count();
    let jump = events.iter().filter(|e| e.kind == SpoofKind::Jump).count();
    let latched = events.iter().any(|e| e.state == NavStateKind::Spoofed);
    let max_resid = events.iter().map(|e| e.residual_m).fold(0.0f32, f32::max);
    eprintln!(
        "[{tag}] events={} drift={drift} jump={jump} spoofed={latched} max_residual_m={max_resid:.1}",
        events.len()
    );
    latched
}

/// Baseline: at the low IMU noise the other scenarios use, a clean flight does
/// NOT false-fire. Confirms the detection LOGIC is sound — the limitation
/// characterized below is robustness to realistic sensor error, not a logic bug.
#[tokio::test(start_paused = true)]
async fn low_noise_clean_flight_holds() {
    let events = run_clean_flight(
        LinearNorth { speed_mps: 8.0 },
        90,
        0.3, // GPS σ at the regime the existing integration tests validate
        0.001,
        0.0001,
        [0.0; 3],
        [0.0; 3],
        0x5EED_1234_5EED_1234,
    )
    .await;
    assert!(
        !spoofed("low-noise", &events),
        "clean flight at low IMU noise must not false-latch Spoofed"
    );
}

/// KNOWN LIMITATION: realistic consumer-MEMS NOISE alone (no bias) false-fires.
#[tokio::test(start_paused = true)]
#[ignore = "documents an unfixed limitation: realistic IMU noise false-fires (needs sliding-window residual)"]
async fn realistic_noise_false_fires_known_limitation() {
    let events = run_clean_flight(
        LinearNorth { speed_mps: 8.0 },
        90,
        2.5, // realistic GPS σ — by itself enough to ramp the fixed-k per-axis CUSUM
        0.03,
        0.002,
        [0.0; 3],
        [0.0; 3],
        0x5EED_1234_5EED_1234,
    )
    .await;
    assert!(
        spoofed("realistic-noise", &events),
        "EXPECTED-FAIL: if realistic IMU noise no longer false-fires, the robustness \
         limitation is fixed — update docs/threats.md and promote this test."
    );
}

/// KNOWN LIMITATION: a persistent accel + gyro bias makes the false-fire faster.
#[tokio::test(start_paused = true)]
#[ignore = "documents an unfixed limitation: realistic IMU bias false-fires (needs bias est / sliding-window residual)"]
async fn realistic_bias_false_fires_known_limitation() {
    let events = run_clean_flight(
        LinearNorth { speed_mps: 8.0 },
        90,
        2.5,
        0.03,
        0.002,
        [0.05, -0.04, 0.03],
        [0.001, -0.001, 0.0015],
        0x5EED_1234_5EED_1234,
    )
    .await;
    assert!(
        spoofed("realistic-bias", &events),
        "EXPECTED-FAIL: see docs/threats.md (cumulative-residual drift limitation)."
    );
}

/// KNOWN LIMITATION: a sustained coordinated turn false-fires worst — the
/// centripetal specific force tilts the Madgwick attitude (~6°), coupling
/// through gravity compensation into a large DR residual (~60 m).
#[tokio::test(start_paused = true)]
#[ignore = "documents an unfixed limitation: sustained turns false-fire (needs GPS-aided attitude)"]
async fn sustained_turn_false_fires_known_limitation() {
    let events = run_clean_flight(
        CircleHorizontal {
            speed_mps: 8.0,
            radius_m: 60.0,
        },
        120,
        2.5,
        0.03,
        0.002,
        [0.05, -0.04, 0.03],
        [0.001, -0.001, 0.0015],
        0x5EED_1234_5EED_1234,
    )
    .await;
    assert!(
        spoofed("turn", &events),
        "EXPECTED-FAIL: see docs/threats.md (centripetal attitude-coupling limitation)."
    );
}
