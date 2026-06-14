//! SITL Phase 2: the SMART / EKF-laundered spoofer and the velocity-aiding lane.
//!
//! `GradualDrift` (scenario_drift) is a NAIVE spoofer — position ramps but the
//! reported GPS velocity stays honest, so the position residual diverges and the
//! detector catches even 1 m/s.
//!
//! `ConsistentDrift` reports a velocity that MATCHES the position ramp — the
//! signature of (a) a spoofer that fakes Doppler and (b) the param-mode SITL case
//! where ArduPilot's EKF has FUSED a slow ramp (its output velocity then tracks
//! the ramp). Here `|v_gps − v_imu|` equals only the (EKF-fusable) drift rate, far
//! under the 15 m/s velocity-mismatch threshold, and the complementary velocity
//! blend (tau=3 s) is fed the spoofed velocity — which pulls dead-reckoning onto
//! the spoofed track and BOUNDS the position residual. The position + post-blend
//! velocity lanes are therefore structurally blind to it.
//!
//! The VELOCITY-AIDING lane (src/detect/drift.rs) closes that gap: it accumulates
//! the FREE-INERTIAL velocity residual `mag_vel_free` (= v_gps − v_free_inertial,
//! reconstructed from the cumulative blend correction), which retains the masked
//! velocity bias ≈ the spoof rate. These tests pin its coverage: a self-consistent
//! walk-off ≥1 m/s now latches Spoofed (it previously evaded entirely below 2 m/s),
//! while honest flight and the pre-attack baseline never false-latch. The residual
//! bound — a walk-off below ~0.5 m/s (near the free-inertial velocity-bias floor),
//! or one confined to turns (the lane is suspended while maneuvering) — is
//! documented in docs/threats.md.

mod common;

use common::{reached_state, run_scenario, Scenario};
use flyingsquirrel::sim::spoof::SpoofPattern;
use flyingsquirrel::types::NavStateKind;

/// Sweep consistent-velocity (smart) vs honest-velocity (naive) drift across
/// rates. Asserts the coverage we rely on AND prints the table for inspection
/// (`--nocapture`): the naive spoofer is caught at every rate by the position
/// lanes; the smart spoofer is caught at ≥1 m/s by the velocity-aiding lane.
#[tokio::test(start_paused = true)]
async fn velocity_aiding_closes_the_smart_walkoff_gap() {
    eprintln!("rate(m/s east)  naive:spoof   smart:spoof");
    for &rate in &[0.5f32, 1.0, 2.0, 3.0, 5.0, 8.0] {
        let naive = run_scenario(Scenario {
            duration_s: 120,
            speed_mps: 8.0,
            pattern: SpoofPattern::GradualDrift {
                apply_at_s: 20.0,
                drift_north_mps: 0.0,
                drift_east_mps: rate,
            },
            seed: 0xC0_5157,
            ..Scenario::default()
        })
        .await;
        let smart = run_scenario(Scenario {
            duration_s: 120,
            speed_mps: 8.0,
            pattern: SpoofPattern::ConsistentDrift {
                apply_at_s: 20.0,
                drift_north_mps: 0.0,
                drift_east_mps: rate,
            },
            seed: 0xC0_5157,
            ..Scenario::default()
        })
        .await;
        let naive_spoofed = reached_state(&naive.events, NavStateKind::Spoofed);
        let smart_spoofed = reached_state(&smart.events, NavStateKind::Spoofed);
        eprintln!(
            "  {:>4}          {}            {}",
            rate, naive_spoofed as u8, smart_spoofed as u8
        );

        // The naive spoofer is position-observable at every rate.
        assert!(
            naive_spoofed,
            "naive (honest-velocity) drift at {rate} m/s must latch Spoofed (position lane)"
        );
        // The smart spoofer is caught by the velocity-aiding lane at ≥1 m/s.
        // 0.5 m/s sits at/under the free-inertial velocity-bias floor — the
        // documented residual bound, not asserted as caught.
        if rate >= 1.0 {
            assert!(
                smart_spoofed,
                "smart (consistent-velocity) walk-off at {rate} m/s must latch Spoofed \
                 (velocity-aiding lane)"
            );
        }
    }
}

/// Focused regression: the canonical 1 m/s self-consistent walk-off — the rate
/// that completely evaded the detector before the velocity-aiding lane — must
/// latch Spoofed.
#[tokio::test(start_paused = true)]
async fn smart_one_mps_walkoff_latches_spoofed() {
    let outcome = run_scenario(Scenario {
        duration_s: 120,
        speed_mps: 8.0,
        pattern: SpoofPattern::ConsistentDrift {
            apply_at_s: 20.0,
            drift_north_mps: 0.0,
            drift_east_mps: 1.0,
        },
        seed: 0xC0_5157,
        ..Scenario::default()
    })
    .await;
    assert!(
        reached_state(&outcome.events, NavStateKind::Spoofed),
        "a 1 m/s self-consistent (faked-Doppler) GPS walk-off must latch Spoofed"
    );
}

/// A clean baseline: the consistent-velocity spoofer must NOT cause a false
/// latch before it is even applied (sanity that the pattern is inert pre-attack).
#[tokio::test(start_paused = true)]
async fn consistent_drift_clean_before_apply_does_not_false_fire() {
    let outcome = run_scenario(Scenario {
        duration_s: 30,
        speed_mps: 8.0,
        // Applied late (t=25s) so the first 25s are honest; 30s total.
        pattern: SpoofPattern::ConsistentDrift {
            apply_at_s: 25.0,
            drift_north_mps: 0.0,
            drift_east_mps: 3.0,
        },
        seed: 0xC0_5157,
        ..Scenario::default()
    })
    .await;
    assert!(
        !reached_state(&outcome.events, NavStateKind::Spoofed),
        "consistent-drift spoofer false-latched before/at the moment of application"
    );
}
