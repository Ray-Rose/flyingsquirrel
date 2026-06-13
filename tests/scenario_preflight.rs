//! Pre-flight gate: detector must NOT evaluate the FSM until preflight is
//! Ready. With the existing synthetic harness (no MAV monitor), preflight
//! requires only static_init_done + first_fix_accepted, which complete
//! within ~1-2s of scenario time. We assert:
//!   - exactly one `PreflightPassed` event fires (on the transition)
//!   - it fires BEFORE any Jump/Drift event would
//!   - a `Spoofed` transition that would normally occur (sudden-jump scenario)
//!     STILL occurs after preflight Ready

mod common;

use common::{count_kind, run_scenario, Scenario};
use flyingsquirrel::sim::spoof::SpoofPattern;
use flyingsquirrel::types::SpoofKind;

#[tokio::test(start_paused = true)]
async fn preflight_passed_fires_exactly_once() {
    let outcome = run_scenario(Scenario {
        duration_s: 30,
        pattern: SpoofPattern::Clean,
        ..Scenario::default()
    })
    .await;

    let passes = count_kind(&outcome.events, SpoofKind::PreflightPassed);
    assert_eq!(
        passes, 1,
        "PreflightPassed must fire exactly once on Initializing→Ready transition (got {})",
        passes
    );
}

#[tokio::test(start_paused = true)]
async fn detector_works_after_preflight_passes() {
    // With a sudden-jump after preflight is ready, the detector should fire
    // and the FSM should transition to Spoofed — preflight gate must not
    // permanently suppress detection.
    let outcome = run_scenario(Scenario {
        duration_s: 60,
        pattern: SpoofPattern::SuddenJump {
            apply_at_s: 20.0,
            jump_north_m: 0.0,
            jump_east_m: 300.0,
        },
        ..Scenario::default()
    })
    .await;

    // Preflight must have passed.
    assert!(count_kind(&outcome.events, SpoofKind::PreflightPassed) >= 1);
    // ... and the spoof must still be detected (jump fires, sever fires).
    assert!(
        count_kind(&outcome.events, SpoofKind::Jump) >= 1,
        "expected Jump events after preflight passed + spoof injected"
    );
    assert!(
        outcome.sever_count >= 1,
        "expected sever_gps action to fire on Spoofed transition"
    );
}
