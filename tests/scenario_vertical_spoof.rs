//! Vertical-spoof detection — end-to-end coverage (audit B-42).
//!
//! Every residual-based detector operates on the HORIZONTAL plane, so a pure
//! altitude teleport (lat/lon honest, alt jumped) was previously invisible.
//! Fusion now runs a GPS-only vertical-rate sanity check: if the reported
//! altitude changes faster than `max_vertical_rate_mps` (default 30 m/s)
//! between consecutive fixes, it emits a `Jump` event with reason
//! `VerticalRate` and registers an external anomaly with the FSM.
//!
//! This scenario teleports altitude by +200 m in a single 1 Hz fix at t=20s
//! (apparent rate 200 m/s, far above the 30 m/s bound) while leaving lat/lon
//! and velocity honest. We assert a `Jump` with reason `VerticalRate` fires.

mod common;

use common::{count_kind, run_scenario, Scenario};
use flyingsquirrel::sim::spoof::SpoofPattern;
use flyingsquirrel::types::SpoofKind;

#[tokio::test(start_paused = true)]
async fn altitude_teleport_fires_vertical_rate_jump() {
    let outcome = run_scenario(Scenario {
        duration_s: 40,
        speed_mps: 8.0,
        pattern: SpoofPattern::AltitudeJump {
            apply_at_s: 20.0,
            jump_m: 200.0,
        },
        seed: 0xA177_0001,
        ..Scenario::default()
    })
    .await;

    let jumps = count_kind(&outcome.events, SpoofKind::Jump);
    assert!(
        jumps > 0,
        "expected >=1 Jump event from altitude teleport, got {}",
        jumps
    );

    let vertical_jumps = outcome
        .events
        .iter()
        .filter(|e| e.kind == SpoofKind::Jump)
        .filter(|e| {
            e.detail
                .get("reason")
                .and_then(|r| r.as_str())
                .map(|s| s == "VerticalRate")
                .unwrap_or(false)
        })
        .count();
    assert!(
        vertical_jumps > 0,
        "expected at least one Jump with reason=VerticalRate; vertical-spoof check not firing. \
         Jump reasons seen: {:?}",
        outcome
            .events
            .iter()
            .filter(|e| e.kind == SpoofKind::Jump)
            .filter_map(|e| e.detail.get("reason").and_then(|r| r.as_str()))
            .collect::<Vec<_>>()
    );
}

#[tokio::test(start_paused = true)]
async fn clean_flight_emits_no_vertical_rate_jump() {
    // A clean flight (constant altitude) must NOT trip the vertical-rate
    // check — confirms the 30 m/s bound doesn't false-fire on honest data.
    let outcome = run_scenario(Scenario {
        duration_s: 40,
        speed_mps: 8.0,
        pattern: SpoofPattern::Clean,
        seed: 0xA177_0002,
        ..Scenario::default()
    })
    .await;

    let vertical_jumps = outcome
        .events
        .iter()
        .filter(|e| e.kind == SpoofKind::Jump)
        .filter(|e| {
            e.detail
                .get("reason")
                .and_then(|r| r.as_str())
                .map(|s| s == "VerticalRate")
                .unwrap_or(false)
        })
        .count();
    assert_eq!(
        vertical_jumps, 0,
        "clean constant-altitude flight must not trip the vertical-rate check"
    );
}
