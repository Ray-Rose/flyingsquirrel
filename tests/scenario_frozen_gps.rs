//! Frozen-fix detector: when GPS reports identical lat/lon for multiple
//! consecutive fixes WHILE the IMU shows motion, the detector must fire
//! a `FrozenGps` event AND treat it as a detector firing (FSM enters
//! Suspicious, then Spoofed if persistent).

mod common;

use common::{count_kind, run_scenario, Scenario};
use flyingsquirrel::sim::spoof::SpoofPattern;
use flyingsquirrel::types::SpoofKind;

#[tokio::test(start_paused = true)]
async fn frozen_gps_fires_when_imu_shows_motion() {
    // Vehicle moves at 8 m/s. GPS freezes at scenario t=10s. After 3
    // consecutive frozen fixes (~t=13s), the FrozenGps detector should
    // fire. We run for 60s to give the FSM time to escalate to Spoofed.
    let outcome = run_scenario(Scenario {
        duration_s: 60,
        speed_mps: 8.0,
        pattern: SpoofPattern::Frozen { freeze_at_s: 10.0 },
        ..Scenario::default()
    })
    .await;

    let frozen_events = count_kind(&outcome.events, SpoofKind::FrozenGps);
    assert!(
        frozen_events >= 1,
        "expected at least one FrozenGps event (vehicle moving + GPS frozen for 50s), got {}",
        frozen_events
    );

    // The frozen-fix detector calls note_external_anomaly which feeds the
    // FSM as if a detector fired. With 50s of frozen fixes, we should
    // transition to Spoofed.
    let sever = outcome.sever_count;
    let rtb = outcome.rtb_count;
    assert!(
        sever >= 1 && rtb >= 1,
        "expected Spoofed actions (sever={} rtb={}) after sustained FrozenGps",
        sever,
        rtb
    );
}
