//! Boot-anchor cross-check: first GPS fix far from operator's declared
//! expected_home must be rejected with `BootAnchorRejected`, and the
//! detector must NOT enter NORMAL detection state. Defends against
//! meaconing-at-boot.

mod common;

use common::{count_kind, run_scenario, Scenario};
use flyingsquirrel::sim::spoof::SpoofPattern;
use flyingsquirrel::types::SpoofKind;

#[tokio::test(start_paused = true)]
async fn first_fix_far_from_expected_home_is_rejected() {
    // SyntheticGps reports lat/lon around (40, -100). Operator says the
    // drone is supposed to be at (50, -50) — a different continent. The
    // first fix must be REJECTED.
    let outcome = run_scenario(Scenario {
        duration_s: 15,
        pattern: SpoofPattern::Clean,
        gps_origin: Some((40.0, -100.0)),
        expected_home: Some((50.0, -50.0)),
        expected_home_radius_m: 1000.0, // tight; real distance is ~thousands of km
        ..Scenario::default()
    })
    .await;

    let rejections = count_kind(&outcome.events, SpoofKind::BootAnchorRejected);
    assert!(
        rejections >= 1,
        "expected at least one BootAnchorRejected event, got {}",
        rejections
    );

    // Preflight should NOT have passed (anchor never accepted → first_fix_accepted stays false).
    let preflight_passes = count_kind(&outcome.events, SpoofKind::PreflightPassed);
    assert_eq!(
        preflight_passes, 0,
        "preflight must not pass when boot-anchor rejection prevents first fix"
    );

    // No spurious Jump/Drift should fire (we never anchored, so we never compute residuals).
    assert_eq!(count_kind(&outcome.events, SpoofKind::Jump), 0);
    assert_eq!(count_kind(&outcome.events, SpoofKind::Drift), 0);
    // The controller actions must NOT have fired.
    assert_eq!(outcome.sever_count, 0);
    assert_eq!(outcome.rtb_count, 0);
}

#[tokio::test(start_paused = true)]
async fn first_fix_near_expected_home_is_accepted() {
    // Sanity: same coords for GPS and expected_home → no rejection.
    let outcome = run_scenario(Scenario {
        duration_s: 15,
        pattern: SpoofPattern::Clean,
        gps_origin: Some((40.0, -100.0)),
        expected_home: Some((40.0, -100.0)),
        expected_home_radius_m: 500.0,
        ..Scenario::default()
    })
    .await;

    let rejections = count_kind(&outcome.events, SpoofKind::BootAnchorRejected);
    assert_eq!(
        rejections, 0,
        "expected no BootAnchorRejected when GPS matches expected_home, got {}",
        rejections
    );
}
