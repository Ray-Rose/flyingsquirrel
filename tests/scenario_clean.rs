//! 60s clean flight — no spoof events should fire.

mod common;

use common::{count_kind, run_scenario, Scenario};
use flyingsquirrel::sim::spoof::SpoofPattern;
use flyingsquirrel::types::SpoofKind;

#[tokio::test(start_paused = true)]
async fn clean_flight_emits_no_spoof_events() {
    let outcome = run_scenario(Scenario {
        duration_s: 60,
        speed_mps: 8.0,
        pattern: SpoofPattern::Clean,
        seed: 0xC1EA_C1EA_C1EA_C1EA,
        ..Scenario::default()
    })
    .await;

    let jumps = count_kind(&outcome.events, SpoofKind::Jump);
    let drifts = count_kind(&outcome.events, SpoofKind::Drift);

    assert_eq!(jumps, 0, "clean flight produced {} Jump events", jumps);
    assert_eq!(drifts, 0, "clean flight produced {} Drift events", drifts);
    assert_eq!(outcome.sever_count, 0, "GPS was severed on a clean flight");
    assert_eq!(outcome.rtb_count, 0, "RTB engaged on a clean flight");
}
