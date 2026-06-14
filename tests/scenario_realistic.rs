//! Realistic false-alarm characterization (audit S-09/S-10 + the nav-review
//! "long/dynamic flight false-fires" finding).
//!
//! The other scenarios fly `LinearNorth` with ~30x-below-realistic IMU noise
//! (accel σ=0.001, gyro σ=0.0001) — a detector-LOGIC test, not a false-alarm-
//! rate test. These run CLEAN flights (no spoof injected, so any detection is a
//! FALSE ALARM) and sweep the IMU error model.
//!
//! Status (after the adaptive per-axis CUSUM — step 1 of the false-latch fix):
//!   * Logic is sound at low noise — `low_noise_clean_flight_holds` PASSES.
//!   * **Realistic noise / bias on LINEAR flight no longer false-fire**
//!     (`realistic_noise_does_not_false_fire`, `realistic_bias_does_not_false_fire`
//!     — promoted from the `#[ignore]`d characterizations). The OLD fixed per-axis
//!     k (0.4σ at σ=2.5 m) ramped on pure GPS noise plus the bounded DR
//!     random-walk; the adaptive per-axis `eff_k` (≈2σ, learned only while the
//!     axis is quiescent — see drift.rs) tolerates both. They now produce the
//!     SAME clean result as the low-noise baseline (events=3, no drift/jump).
//!   * **A sustained coordinated turn STILL false-fires**
//!     (`sustained_turn_false_fires_known_limitation`, still `#[ignore]`d): the
//!     centripetal specific force tilts the Madgwick attitude (~6°), coupling
//!     through gravity compensation into a ~90 m DR residual that no adaptive
//!     threshold should (or does) absorb. The fix is GPS-aided attitude (subtract
//!     the GPS-derived centripetal) — step 3, a separate validated pass. See
//!     docs/threats.md. Run the remaining characterization with:
//!     `cargo test --test scenario_realistic -- --ignored`.

mod common;

use common::TestController;
use flyingsquirrel::fusion::FusionConfig;
use flyingsquirrel::ingest::{GpsSource, ImuSource};
use flyingsquirrel::runtime::spawn_all;
use flyingsquirrel::sim::spoof::{SpoofInjector, SpoofPattern};
use flyingsquirrel::sim::trajectory::{CircleHorizontal, LinearNorth, TrajectoryGenerator};
use flyingsquirrel::sim::{SyntheticGps, SyntheticImu};
use flyingsquirrel::types::{NavStateKind, SpoofKind, SpoofingEvent};
use std::sync::Arc;
use std::time::Duration;

/// Clean flight (no spoof injected). Thin wrapper over `run_flight`.
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
    run_flight(
        traj,
        duration_s,
        gps_pos_sigma_m,
        accel_sigma,
        gyro_sigma,
        accel_bias,
        gyro_bias,
        seed,
        SpoofPattern::Clean,
        None,
    )
    .await
}

/// Run a flight, optionally overlaying a GPS spoof `pattern`, and collect the
/// emitted events. `Clean` = no attack (any detection is a FALSE ALARM); other
/// patterns inject a real attack (detection is REQUIRED).
#[allow(clippy::too_many_arguments)]
async fn run_flight<T>(
    traj: T,
    duration_s: u32,
    gps_pos_sigma_m: f32,
    accel_sigma: f32,
    gyro_sigma: f32,
    accel_bias: [f32; 3],
    gyro_bias: [f32; 3],
    seed: u64,
    pattern: SpoofPattern,
    also: Option<SpoofPattern>,
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
    let gps_boxed: Box<dyn GpsSource> = match also {
        // Compose two patterns (e.g. DropDoppler then Frozen) by nesting injectors.
        Some(outer) => Box::new(SpoofInjector::new(SpoofInjector::new(gps, pattern), outer)),
        None => Box::new(SpoofInjector::new(gps, pattern)),
    };
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

/// REGRESSION (promoted from a known limitation, fixed by the adaptive per-axis
/// CUSUM): realistic consumer-MEMS NOISE alone (no bias) on a linear flight must
/// NOT false-fire. The old fixed per-axis k (0.4σ at σ=2.5 m) ramped on pure GPS
/// noise plus the bounded DR random-walk; the adaptive `eff_k` (≈2σ, learned
/// only while quiescent) tolerates both. Same clean result as the low-noise
/// baseline.
#[tokio::test(start_paused = true)]
async fn realistic_noise_does_not_false_fire() {
    let events = run_clean_flight(
        LinearNorth { speed_mps: 8.0 },
        90,
        2.5, // realistic GPS σ — enough to ramp the OLD fixed-k per-axis CUSUM
        0.03,
        0.002,
        [0.0; 3],
        [0.0; 3],
        0x5EED_1234_5EED_1234,
    )
    .await;
    assert!(
        !spoofed("realistic-noise", &events),
        "realistic IMU noise on a linear flight must NOT false-latch Spoofed \
         (adaptive per-axis CUSUM, step 1)"
    );
}

/// REGRESSION (promoted): a persistent accel + gyro bias on a linear flight must
/// also NOT false-fire. The adaptive per-axis `eff_k` absorbs the bias-induced
/// per-fix residual scale the same way it absorbs noise; the bounded cumulative
/// excursion over 90 s no longer ramps the CUSUM.
#[tokio::test(start_paused = true)]
async fn realistic_bias_does_not_false_fire() {
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
        !spoofed("realistic-bias", &events),
        "realistic IMU bias on a linear flight must NOT false-latch Spoofed \
         (adaptive per-axis CUSUM, step 1)"
    );
}

/// REGRESSION (promoted — fixed by inertial centripetal attitude compensation,
/// step 3): a sustained coordinated turn must NOT false-fire. The centripetal
/// specific force used to tilt the Madgwick attitude ~6° and couple a ~90 m DR
/// residual; subtracting the inertial ω×v estimate (gyro × dead-reckoned
/// velocity) before the gravity correction keeps the attitude level. Now
/// produces the same clean result as the low-noise baseline.
#[tokio::test(start_paused = true)]
async fn sustained_turn_does_not_false_fire() {
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
        !spoofed("turn", &events),
        "a sustained coordinated turn must NOT false-latch Spoofed \
         (inertial centripetal attitude compensation, step 3)"
    );
}

/// EXPLORATORY (step-2 characterization): does a LONG clean flight false-latch?
/// Steps 1+3 bound the residual over 90–120 s, but cumulative DR drift grows
/// without bound, so a 10-minute flight is the real test of whether the
/// cumulative-since-anchor residual still needs the sliding-window fix.
/// REGRESSION (step 2): a long clean flight WITH Doppler does not false-latch —
/// the complementary velocity blend bounds DR drift indefinitely (verified to
/// 20 min). This is why the long-flight cumulative-drift false-latch the
/// sliding-window residual was scoped for never materializes in normal flight.
#[tokio::test(start_paused = true)]
async fn long_doppler_flight_does_not_false_fire() {
    let events = run_clean_flight(
        LinearNorth { speed_mps: 8.0 },
        600,
        2.5,
        0.03,
        0.002,
        [0.05, -0.04, 0.03],
        [0.001, -0.001, 0.0015],
        0x5EED_1234_5EED_1234,
    )
    .await;
    assert!(
        !spoofed("doppler-600s", &events),
        "a 10-min Doppler flight must not false-latch"
    );
}

/// REGRESSION (step 2): a clean flight whose GPS LOSES Doppler mid-flight must
/// not false-latch. Without velocity the blend can't bound DR drift, so the
/// cumulative residual grows unbounded and used to false-latch a healthy drone
/// in ~2 min. The no-Doppler policy gates the cumulative-residual detectors
/// (drift CUSUM + hard-residual jump) once Doppler has been absent long enough,
/// relying on the Doppler-independent frozen-GPS / vertical-rate detectors.
#[tokio::test(start_paused = true)]
async fn no_doppler_flight_does_not_false_fire() {
    let events = run_flight(
        LinearNorth { speed_mps: 8.0 },
        300,
        2.5,
        0.03,
        0.002,
        [0.05, -0.04, 0.03],
        [0.001, -0.001, 0.0015],
        0x5EED_1234_5EED_1234,
        SpoofPattern::DropDoppler { after_s: 20.0 },
        None,
    )
    .await;
    assert!(
        !spoofed("no-doppler", &events),
        "a clean flight that loses Doppler must NOT false-latch (cumulative-residual \
         detectors gated; Doppler-independent detectors retained)"
    );
}

/// MISSED-DETECTION GUARD (step 2): gating the cumulative-residual detectors off
/// in no-Doppler must NOT blind the Doppler-INDEPENDENT frozen-GPS detector. A
/// stuck/replayed GPS (pinned lat/lon) after Doppler is lost must STILL latch
/// Spoofed — frozen-GPS compares GPS to IMU displacement directly, and DR stays
/// un-aided (no GPS-position coupling), so the IMU still shows the real motion.
/// Doppler is present for the first 20 s so DR velocity is established before the
/// dropout, then the GPS freezes at 40 s.
#[tokio::test(start_paused = true)]
async fn frozen_gps_during_no_doppler_is_still_detected() {
    let events = run_flight(
        LinearNorth { speed_mps: 8.0 },
        120,
        2.5,
        0.03,
        0.002,
        [0.05, -0.04, 0.03],
        [0.001, -0.001, 0.0015],
        0x5EED_1234_5EED_1234,
        SpoofPattern::DropDoppler { after_s: 20.0 },
        Some(SpoofPattern::Frozen { freeze_at_s: 40.0 }),
    )
    .await;
    assert!(
        spoofed("frozen-no-doppler", &events),
        "a frozen/replayed GPS with no Doppler MUST still be detected — frozen-GPS is \
         Doppler-independent and must survive the no-Doppler detector gating"
    );
}

/// MISSED-DETECTION GUARD for step 3: the inertial centripetal compensation
/// must not HIDE a real spoof during a turn. Same realistic-noise coordinated
/// turn, but a 2 m/s GPS position drift is injected partway through — the
/// detector MUST still latch Spoofed. The compensation only sharpens DR
/// accuracy and consumes no GPS input, so detection is preserved, not weakened.
#[tokio::test(start_paused = true)]
async fn spoof_during_turn_is_still_detected() {
    let events = run_flight(
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
        SpoofPattern::GradualDrift {
            apply_at_s: 30.0,
            drift_north_mps: 2.0,
            drift_east_mps: 0.0,
        },
        None,
    )
    .await;
    assert!(
        spoofed("turn-spoof", &events),
        "a real GPS drift during a turn MUST still be detected — the centripetal \
         compensation must not hide a spoof"
    );
}
