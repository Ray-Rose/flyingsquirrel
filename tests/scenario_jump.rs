//! 200m teleport at t=30s — Jump event must fire promptly, SPOOFED within dwell.

mod common;

use common::{count_kind, reached_state, run_scenario, Scenario};
use flyingsquirrel::sim::spoof::SpoofPattern;
use flyingsquirrel::types::{NavStateKind, SpoofKind};

#[tokio::test(start_paused = true)]
async fn sudden_jump_is_detected_and_latches_spoofed() {
    let outcome = run_scenario(Scenario {
        duration_s: 50,
        speed_mps: 8.0,
        pattern: SpoofPattern::SuddenJump {
            apply_at_s: 30.0,
            jump_north_m: 0.0,
            jump_east_m: 200.0,
        },
        seed: 0x5DDD_EE50,
        ..Scenario::default()
    })
    .await;

    let jumps = count_kind(&outcome.events, SpoofKind::Jump);
    assert!(jumps > 0, "expected >=1 Jump event, got {}", jumps);

    assert!(
        reached_state(&outcome.events, NavStateKind::Spoofed),
        "state machine never reached SPOOFED"
    );
    assert_eq!(
        outcome.sever_count, 1,
        "sever_gps should be called exactly once"
    );
    assert_eq!(
        outcome.rtb_count, 1,
        "engage_rtb should be called exactly once"
    );
}
