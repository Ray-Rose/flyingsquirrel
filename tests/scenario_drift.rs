//! 1 m/s gradual drift starting at t=20s — Drift event fires, SPOOFED latches.

mod common;

use common::{count_kind, reached_state, run_scenario, Scenario};
use flyingsquirrel::sim::spoof::SpoofPattern;
use flyingsquirrel::types::{NavStateKind, SpoofKind};

#[tokio::test(start_paused = true)]
async fn gradual_drift_is_detected_and_latches_spoofed() {
    let outcome = run_scenario(Scenario {
        duration_s: 80,
        speed_mps: 8.0,
        pattern: SpoofPattern::GradualDrift {
            apply_at_s: 20.0,
            drift_north_mps: 0.0,
            drift_east_mps: 1.0,
        },
        seed: 0xD12F_D12F,
        ..Scenario::default()
    })
    .await;

    let drifts = count_kind(&outcome.events, SpoofKind::Drift);
    assert!(drifts > 0, "expected >=1 Drift event, got {}", drifts);
    assert!(
        reached_state(&outcome.events, NavStateKind::Spoofed),
        "state machine never reached SPOOFED"
    );
    assert_eq!(outcome.sever_count, 1);
    assert_eq!(outcome.rtb_count, 1);
}
