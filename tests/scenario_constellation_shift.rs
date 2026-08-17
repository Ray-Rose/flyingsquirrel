//! Constellation-quality lane, end to end.
//!
//! Two properties, and the second matters as much as the first:
//!
//! 1. **It sees the takeover.** A spoofer capturing the receiver swaps in a
//!    simulated constellation. At that instant it has not moved the vehicle at
//!    all, so every residual-based lane is looking at a ~0 residual and
//!    correctly stays silent. The metadata step is the only observable, and
//!    this lane must catch it.
//!
//! 2. **It cannot sever GPS by itself.** The FSM counts an external anomaly as
//!    "a detector fired", so anything able to fire continuously across the
//!    suspicious→spoofed dwell can reach Spoofed and cut the GPS. A metadata
//!    step is corroboration, not proof — the position data here is perfectly
//!    good — so a run where NOTHING but the constellation changed must never
//!    end in a sever. `ConstellationHealth` guarantees that by re-baselining on
//!    every report and rate-limiting itself; this test is what holds that
//!    guarantee honest at the system level.

mod common;

use common::{count_kind, run_scenario, Scenario};
use flyingsquirrel::sim::spoof::SpoofPattern;
use flyingsquirrel::types::SpoofKind;

#[tokio::test(start_paused = true)]
async fn constellation_swap_is_seen_but_never_severs_on_its_own() {
    // 12 sats / HDOP 0.9 for the first 20 s, then a swap to 6 sats / HDOP 0.3
    // — fewer satellites AND implausibly perfect geometry, the ordinary shape
    // of a synthesized constellation. lat/lon/velocity remain honest for the
    // whole run.
    let outcome = run_scenario(Scenario {
        duration_s: 120,
        speed_mps: 8.0,
        pattern: SpoofPattern::ConstellationSwap {
            apply_at_s: 20.0,
            sats_after: 6,
            hdop_after: 0.3,
        },
        ..Scenario::default()
    })
    .await;

    // 1. The takeover is seen.
    let shifts = count_kind(&outcome.events, SpoofKind::ConstellationShift);
    assert_eq!(
        shifts, 1,
        "expected exactly one ConstellationShift — the lane must report the step \
         once and then re-baseline onto the new sky, not re-report it forever \
         (a sustained firing is what would let it sever GPS alone); got {shifts}"
    );

    // 2. Nothing else could possibly have seen it — the position never moved.
    //    This is what makes the lane orthogonal rather than redundant.
    let jumps = count_kind(&outcome.events, SpoofKind::Jump);
    let drifts = count_kind(&outcome.events, SpoofKind::Drift);
    assert_eq!(
        (jumps, drifts),
        (0, 0),
        "position and velocity stayed honest, so the residual lanes must stay \
         silent — if they fired, this test is no longer isolating the takeover \
         signal (jumps={jumps} drifts={drifts})"
    );

    // 3. THE SAFETY BOUND: metadata alone never cuts the GPS.
    assert_eq!(
        (outcome.sever_count, outcome.rtb_count),
        (0, 0),
        "a constellation step is corroboration, not proof — the position data in \
         this run is good, and severing GPS + forcing RTL on an aircraft with a \
         healthy position solution would be a self-inflicted denial of service \
         (sever={} rtb={})",
        outcome.sever_count,
        outcome.rtb_count
    );
}

#[tokio::test(start_paused = true)]
async fn steady_sky_produces_no_constellation_events() {
    // The false-positive guard at system level: the sim's ordinary GPS reports
    // a constant 12 sats / 0.9 HDOP, and a clean flight must never produce a
    // shift report.
    let outcome = run_scenario(Scenario {
        duration_s: 120,
        speed_mps: 8.0,
        pattern: SpoofPattern::Clean,
        ..Scenario::default()
    })
    .await;

    assert_eq!(
        count_kind(&outcome.events, SpoofKind::ConstellationShift),
        0,
        "a steady constellation must never be reported as a shift"
    );
    assert_eq!(
        (outcome.sever_count, outcome.rtb_count),
        (0, 0),
        "clean flight must not act"
    );
}
