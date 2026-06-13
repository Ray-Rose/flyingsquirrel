//! Velocity-mismatch detection — end-to-end coverage (audit S-02 / S-03).
//!
//! The position-based scenarios (sudden_jump, drift) fire `JumpReason::
//! HardResidual` purely on position, so the velocity-mismatch branch of the
//! jump detector — including the B-37 consecutive-fire persistence gate —
//! had NO integration coverage. A regression that disabled velocity-mismatch
//! detection in the wired pipeline would leave every other test green.
//!
//! This scenario uses the `VelocityInconsistent` spoof pattern: it adds a
//! large offset to the reported GPS Doppler speed while leaving position
//! honest. That produces |v_gps − v_imu| well above the 15 m/s threshold
//! WITHOUT moving position past the 50 m hard-residual threshold — isolating
//! the velocity-mismatch path. We assert that a `Jump` event fires AND that
//! at least one Jump event's `reason` is `VelocityMismatch` (not just
//! `HardResidual`), proving the velocity path is actually wired and working.

mod common;

use common::{count_kind, run_scenario, Scenario};
use flyingsquirrel::sim::spoof::SpoofPattern;
use flyingsquirrel::types::SpoofKind;

#[tokio::test(start_paused = true)]
async fn velocity_inconsistency_fires_velocity_mismatch_jump() {
    let outcome = run_scenario(Scenario {
        duration_s: 50,
        speed_mps: 8.0,
        // +40 m/s added to reported speed starting at t=20s. True IMU velocity
        // is ~8 m/s north; reported GPS speed becomes ~48 m/s → mismatch ≈ 40
        // m/s, far above the 15 m/s threshold. Position stays honest so the
        // hard-residual (50 m) path does NOT fire — isolating velocity.
        pattern: SpoofPattern::VelocityInconsistent {
            apply_at_s: 20.0,
            extra_speed_mps: 40.0,
        },
        seed: 0x5E10_C17F,
        ..Scenario::default()
    })
    .await;

    let jumps = count_kind(&outcome.events, SpoofKind::Jump);
    assert!(
        jumps > 0,
        "expected >=1 Jump event from velocity mismatch, got {}",
        jumps
    );

    // The decisive assertion: at least one Jump must carry reason
    // "VelocityMismatch". If the velocity-mismatch detector were broken or
    // unwired, we'd see zero Jumps here (position is honest, so HardResidual
    // can't fire).
    let velocity_mismatch_jumps = outcome
        .events
        .iter()
        .filter(|e| e.kind == SpoofKind::Jump)
        .filter(|e| {
            e.detail
                .get("reason")
                .and_then(|r| r.as_str())
                .map(|s| s.contains("VelocityMismatch"))
                .unwrap_or(false)
        })
        .count();
    assert!(
        velocity_mismatch_jumps > 0,
        "expected at least one Jump with reason=VelocityMismatch; the velocity-mismatch \
         detector path is not firing end-to-end. Jump reasons seen: {:?}",
        outcome
            .events
            .iter()
            .filter(|e| e.kind == SpoofKind::Jump)
            .filter_map(|e| e.detail.get("reason").and_then(|r| r.as_str()))
            .collect::<Vec<_>>()
    );
}
