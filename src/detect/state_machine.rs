//! Detector state machine: NORMAL -> SUSPICIOUS -> SPOOFED (latched).

use super::drift::{DriftDetector, DriftReason};
use super::jump::{JumpDetector, JumpReason};
use super::residual::Residual;
use super::DetectConfig;
use crate::types::NavStateKind;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Event {
    Jump(JumpReason),
    Drift(DriftReason),
    Transition {
        from: NavStateKind,
        to: NavStateKind,
    },
}

pub struct StateMachine {
    cfg: DetectConfig,
    state: NavStateKind,
    jump: JumpDetector,
    drift: DriftDetector,
    suspicious_since: Option<f64>,
    last_clean_since: Option<f64>,
    /// Set by `note_dropout_recovery()` once per fix. When true, the next
    /// `evaluate()` skips dwell-time accumulation and CUSUM accrual — the
    /// fix is on the far side of a GPS dropout and we can't trust the
    /// dwell timer (it would otherwise tick across the gap and false-fire).
    dropout_recovery: bool,
    /// Set by `note_external_anomaly()` to indicate an out-of-band detector
    /// (e.g. frozen-fix) fired. The next `evaluate()` treats this like
    /// jump_or_drift_firing for state-transition purposes.
    external_anomaly: bool,
    /// Most recent t_secs at which we recorded an "ignore" tick (so we can
    /// compute how long a single dropout pause lasted).
    last_dropout_t: Option<f64>,
    /// Total count of fixes processed with the dropout-recovery flag set.
    /// Exported for tests / operator dashboards.
    pub dropout_recovery_count: u64,
    /// Current streak of consecutive dropout-recovery fixes. Capped at
    /// `max_consecutive_dropout_recoveries`; past that the FSM stops pausing
    /// the dwell timer so an attacker can't stall escalation indefinitely
    /// by throttling GPS to >gps_dropout_freeze_s.
    consecutive_dropout: u8,
    /// CUMULATIVE seconds of dwell-pause absorbed during the CURRENT
    /// Suspicious episode. The consecutive-streak cap alone is bypassable by
    /// cadence modulation (N slow fixes + 1 fast fix resets the streak AND
    /// `last_dropout_t`, so the fast fix's dwell progress is re-absorbed by
    /// the next pause — net dwell progress per cycle is zero and Spoofed
    /// never latches). This budget bounds the TOTAL stall an attacker can
    /// buy per episode regardless of cadence pattern. Reset on genuine
    /// episode end (fresh Suspicious entry, committed clear, operator
    /// reset); preserved across re-anchor vetoes (same episode).
    paused_cumulative_s: f64,
    /// Whether the cumulative-budget DwellPauseExceeded notice fired for
    /// this episode (one event per episode, not per fix).
    pause_budget_warned: bool,
    /// Set when the consecutive cap or the cumulative budget is exceeded;
    /// consumed by Fusion to emit a `DwellPauseExceeded` event.
    pub dwell_pause_exceeded_pending: bool,
    /// Whether the CUMULATIVE-position-residual detectors (drift CUSUM +
    /// hard-residual jump) are active. Fusion disables them during sustained
    /// no-Doppler (step 2): without GPS velocity, an accumulated DR-drift offset
    /// is indistinguishable from a spoof (both make GPS sit far from the
    /// dead-reckoned position), so these only false-latch a healthy drone. The
    /// Doppler-INDEPENDENT detectors (frozen-GPS, vertical-rate) — which compare
    /// GPS to the IMU directly, not to the accumulated residual — stay active.
    /// Re-enabled the instant Doppler returns.
    pos_detectors_enabled: bool,
}

// NB: an earlier `dwell_paused_s` accumulator was removed (audit C5). The
// dwell-pause mechanic is implemented by walking `suspicious_since` forward
// inside `evaluate()`; the redundant counter was write-only.

/// Once `consecutive_dropout` reaches this value, we stop pausing the FSM
/// dwell. At 1Hz GPS this is ~5 seconds of consecutive dropouts. Any longer
/// and we conclude the GPS link is structurally degraded (or being throttled
/// adversarially), not just glitching.
const MAX_CONSECUTIVE_DROPOUT: u8 = 5;

/// The position-clean threshold for clearing Suspicious is at least this
/// multiple of the drift detector's learned magnitude noise floor. ~3× the
/// Rayleigh mean (≈3.7σ) so clean-noise spikes rarely break the continuous-clean
/// streak that `normal_clear_dwell_s` requires — otherwise a noisy-but-honest
/// flight (σ≈2.5 m ⇒ ~3 m residual) could never satisfy the sub-noise base
/// `cusum_k_m` (1 m) and the FSM would get STUCK in Suspicious after a transient.
const CLEAR_NOISE_FACTOR: f32 = 3.0;
/// Clearing also requires every drift CUSUM to be below this fraction of the
/// firing threshold `cusum_h_m`. A slow drift accumulating between the detection
/// floor and the (generous) clean threshold keeps a CUSUM elevated, so this
/// bars it from being classified "clean" and cleared away (which would reset the
/// CUSUM each cycle — a missed detection).
const CLEAR_QUIESCENT_FRACTION: f32 = 0.25;

impl StateMachine {
    pub fn new(cfg: DetectConfig) -> Self {
        Self {
            cfg,
            state: NavStateKind::Normal,
            jump: JumpDetector::new(cfg),
            drift: DriftDetector::new(cfg),
            suspicious_since: None,
            last_clean_since: None,
            dropout_recovery: false,
            external_anomaly: false,
            last_dropout_t: None,
            dropout_recovery_count: 0,
            consecutive_dropout: 0,
            paused_cumulative_s: 0.0,
            pause_budget_warned: false,
            dwell_pause_exceeded_pending: false,
            pos_detectors_enabled: true,
        }
    }

    /// Enable/disable the cumulative-residual detectors (drift CUSUM +
    /// hard-residual jump) — step 2 no-Doppler policy. Disabling resets both so
    /// stale accumulation can't fire when Doppler returns. The velocity-mismatch
    /// jump is already inert without Doppler; frozen-GPS / vertical-rate are
    /// unaffected (they drive the FSM via `note_external_anomaly`).
    pub fn set_pos_detectors_enabled(&mut self, on: bool) {
        if !on && self.pos_detectors_enabled {
            self.drift.reset();
            self.jump.reset();
        }
        self.pos_detectors_enabled = on;
    }

    /// Called by `Fusion` before `evaluate()` when the current fix is the
    /// first one after a GPS dropout exceeded `gps_dropout_freeze_s`. The
    /// flag is consumed by the next `evaluate()`.
    pub fn note_dropout_recovery(&mut self) {
        self.dropout_recovery = true;
    }

    /// Called by `Fusion` when an EXTERNAL detector (e.g. frozen-fix /
    /// sensor-stuck) has fired for the current fix. The next `evaluate()`
    /// treats this as a detector firing — escalating Normal→Suspicious or
    /// suppressing a clearing trigger. Consumed by the next evaluate.
    pub fn note_external_anomaly(&mut self) {
        self.external_anomaly = true;
    }

    /// Current streak of consecutive dropout-recovery fixes. Exposed for
    /// the DwellPauseExceeded event payload.
    pub fn consecutive_dropout_count(&self) -> u8 {
        self.consecutive_dropout
    }

    /// Cumulative dwell-pause seconds consumed in the current Suspicious
    /// episode. Exposed for the DwellPauseExceeded event payload.
    pub fn paused_cumulative_secs(&self) -> f64 {
        self.paused_cumulative_s
    }

    pub fn state(&self) -> NavStateKind {
        self.state
    }

    /// Monotonic timestamp at which the FSM most recently entered Suspicious.
    /// `None` in any other state. Fusion captures this BEFORE `evaluate()`
    /// so that, if `evaluate` transitions Susp→Normal and fusion later vetoes,
    /// the original entry can be restored via `force_back_to_suspicious` —
    /// cumulative dwell accrues across veto cycles rather than resetting.
    pub fn suspicious_since(&self) -> Option<f64> {
        self.suspicious_since
    }

    pub fn drift_state(&self) -> super::drift::CusumState {
        self.drift.state()
    }

    /// One fix of evaluation. `t_secs` is the monotonic timestamp of the GPS fix.
    /// Returns the events generated this tick.
    pub fn evaluate(
        &mut self,
        r: &Residual,
        t_secs: f64,
        hdop: Option<f32>,
        sats: Option<u8>,
    ) -> Vec<Event> {
        let mut out = Vec::new();

        // Drain the per-fix flags BEFORE the Spoofed early-return. Otherwise,
        // a `note_dropout_recovery()` / `note_external_anomaly()` call that
        // races with Spoofed would leave the flag set forever — and
        // `manual_reset_to_normal()` would then carry the stale flag into the
        // first post-reset fix, producing a phantom dropout pause or external
        // anomaly with no current evidence.
        let in_dropout_recovery = std::mem::replace(&mut self.dropout_recovery, false);
        let pending_external = std::mem::replace(&mut self.external_anomaly, false);

        if self.state == NavStateKind::Spoofed {
            // The drained flags were valid at the moment they were set but the
            // Spoofed latch supersedes them — Spoofed is a one-way street.
            // Letting them silently expire keeps `manual_reset_to_normal()`
            // recovery clean. (Holds them in locals so the compiler doesn't
            // grumble about the unused bind.)
            let _ = pending_external;
            return out;
        }

        // GPS-dropout handling: don't let the suspicious->spoofed dwell timer
        // advance across the gap. The detector still runs (operators want to
        // see Jump/Drift events even during a recovery fix), but the dwell
        // bookkeeping pushes `suspicious_since` forward by the gap duration so
        // the timer math comes out the same as if those seconds hadn't passed.
        // Defends against jam-then-spoof: attacker jams GPS for N seconds
        // while we're SUSPICIOUS, then drops the spoof — without this freeze,
        // dwell timer keeps ticking and SPOOFED fires on the resumption fix.
        if in_dropout_recovery {
            self.dropout_recovery_count = self.dropout_recovery_count.saturating_add(1);
            self.consecutive_dropout = self.consecutive_dropout.saturating_add(1);
            // TWO caps gate the pause, and BOTH must hold:
            //
            //   1. Consecutive-streak cap (MAX_CONSECUTIVE_DROPOUT): a
            //      sustained jam stops pausing after ~5 fixes so the operator
            //      sees a real decision rather than infinite Suspicious.
            //
            //   2. Cumulative per-episode budget (cfg.max_dwell_pause_s): the
            //      streak cap alone is bypassable by cadence modulation — an
            //      attacker emitting 5 slow fixes + 1 fast fix per cycle
            //      resets the streak each cycle, and because `last_dropout_t`
            //      is nulled by the fast fix, the next pause re-absorbs the
            //      fast fix's dwell progress too: net dwell progress per
            //      cycle was ZERO and Spoofed never latched (no RTL, ever).
            //      The cumulative budget bounds total stalling per Suspicious
            //      episode regardless of the cadence pattern.
            let budget_ok = self.paused_cumulative_s < self.cfg.max_dwell_pause_s as f64;
            if self.consecutive_dropout <= MAX_CONSECUTIVE_DROPOUT && budget_ok {
                if self.state == NavStateKind::Suspicious {
                    if let Some(susp_t) = self.suspicious_since {
                        let prev_t = self.last_dropout_t.unwrap_or(susp_t);
                        let elapsed_during_dropout = (t_secs - prev_t).max(0.0);
                        self.suspicious_since = Some(susp_t + elapsed_during_dropout);
                        self.paused_cumulative_s += elapsed_during_dropout;
                    }
                }
            } else if self.state == NavStateKind::Suspicious
                && (self.consecutive_dropout == MAX_CONSECUTIVE_DROPOUT + 1
                    || (!budget_ok && !self.pause_budget_warned))
            {
                // First fix past either cap — set the flag so Fusion emits an
                // event. The streak arm fires once per streak; the budget arm
                // once per episode. Gated on Suspicious: dwell-pause only
                // exists there, and counting in Normal produced spurious
                // operator events.
                if !budget_ok {
                    self.pause_budget_warned = true;
                }
                self.dwell_pause_exceeded_pending = true;
            }
            self.last_dropout_t = Some(t_secs);
            // Detectors continue below — operators should still see the
            // anomaly events, just without the dwell escalation when under
            // the cap.
        } else {
            self.last_dropout_t = None;
            self.consecutive_dropout = 0;
        }

        // Cumulative-residual detectors, gated off during sustained no-Doppler
        // (step 2). Frozen-GPS / vertical-rate still drive the FSM via the
        // `pending_external` flag below, so they remain effective.
        let (jump, drift) = if self.pos_detectors_enabled {
            (
                self.jump.evaluate(r, hdop, sats),
                self.drift.evaluate(r, hdop, sats),
            )
        } else {
            (None, None)
        };

        // Use the external-anomaly flag drained at the top.
        let detector_firing = jump.is_some() || drift.is_some() || pending_external;

        // Sample-level cleanliness — decides whether to CLEAR back to Normal.
        //
        // Base position threshold, with the no-Doppler tightening preserved: when
        // GPS lacks Doppler the velocity-mismatch detector is skipped and
        // `vel_clean` is forced true, so we TIGHTEN the position gate by half to
        // deny an attacker a free pass to walk at just under `cusum_k_m`.
        let base_pos_thresh = if r.dvel_known {
            self.cfg.cusum_k_m
        } else {
            self.cfg.cusum_k_m * 0.5
        };
        // Make the clean threshold NOISE-AWARE: the base `cusum_k_m` (1 m) sits
        // below the realistic GPS residual (~3 m at σ=2.5 m), so a clean-but-noisy
        // flight could never look "clean" and the FSM got STUCK in Suspicious
        // after a transient anomaly. Scale by the drift detector's learned
        // magnitude noise floor (the step-1 adaptive floor), so clearing tracks
        // the actual noise environment instead of a fixed sub-noise constant.
        let pos_thresh = base_pos_thresh.max(self.drift.mag_noise_floor() * CLEAR_NOISE_FACTOR);
        let pos_clean = (r.mag_pos as f32) < pos_thresh;
        let vel_clean =
            !r.dvel_known || (r.mag_vel as f32) < self.cfg.max_velocity_mismatch_mps * 0.25;
        // Two safety gates so the generous noise-aware threshold can never clear
        // an ACTUAL attack: (1) no detector may be firing this fix — an active
        // jump/drift/external anomaly is never "clean"; (2) the drift CUSUMs must
        // be QUIESCENT — a slow drift sitting between the detection floor and
        // `pos_thresh` doesn't fire yet but keeps a CUSUM elevated, so requiring
        // it near zero stops that drift from being cleared away (which would
        // reset the CUSUM each cycle and let the attack never escalate).
        let st = self.drift.state();
        let cusum_peak = st
            .s_pos_n
            .max(st.s_neg_n)
            .max(st.s_pos_e)
            .max(st.s_neg_e)
            .max(st.s_mag);
        let drift_quiescent = cusum_peak < self.cfg.cusum_h_m * CLEAR_QUIESCENT_FRACTION;
        let sample_clean = pos_clean && vel_clean && drift_quiescent && !detector_firing;

        if let Some(j) = jump {
            out.push(Event::Jump(j));
        }
        if let Some(d) = drift {
            out.push(Event::Drift(d));
        }

        match self.state {
            NavStateKind::Normal => {
                if detector_firing {
                    self.suspicious_since = Some(t_secs);
                    self.last_clean_since = None;
                    // Fresh Suspicious episode — the dwell-pause budget renews.
                    self.paused_cumulative_s = 0.0;
                    self.pause_budget_warned = false;
                    self.state = NavStateKind::Suspicious;
                    out.push(Event::Transition {
                        from: NavStateKind::Normal,
                        to: NavStateKind::Suspicious,
                    });
                }
            }
            NavStateKind::Suspicious => {
                let entered = self.suspicious_since.unwrap_or(t_secs);
                // Decision tree (sample_clean wins over latched CUSUM):
                //   sample_clean        => count toward CLEAR
                //   !sample_clean & fire => count toward SPOOFED
                //   !sample_clean & !fire => stagnate
                if sample_clean {
                    let clean_since = *self.last_clean_since.get_or_insert(t_secs);
                    if t_secs - clean_since >= self.cfg.normal_clear_dwell_s as f64 {
                        self.state = NavStateKind::Normal;
                        self.suspicious_since = None;
                        self.last_clean_since = None;
                        // Intentionally DO NOT call drift.reset() / jump.reset()
                        // here. Fusion's `Susp→Normal` event handler validates
                        // the re-anchor plausibility, and on failure calls
                        // `force_back_to_suspicious()` — at which point we want
                        // the cumulative CUSUM evidence intact. Fusion must call
                        // `commit_susp_to_normal()` AFTER veto passes to reset
                        // the detectors.
                        out.push(Event::Transition {
                            from: NavStateKind::Suspicious,
                            to: NavStateKind::Normal,
                        });
                    }
                } else if detector_firing {
                    self.last_clean_since = None;
                    let dwell = t_secs - entered;
                    if dwell >= self.cfg.suspicious_to_spoofed_dwell_s as f64 {
                        self.state = NavStateKind::Spoofed;
                        self.suspicious_since = None;
                        out.push(Event::Transition {
                            from: NavStateKind::Suspicious,
                            to: NavStateKind::Spoofed,
                        });
                    }
                } else {
                    self.last_clean_since = None;
                }
            }
            NavStateKind::Spoofed => unreachable!(),
        }

        out
    }

    /// Operator-driven reset. Clears latched SPOOFED.
    pub fn manual_reset_to_normal(&mut self) {
        self.state = NavStateKind::Normal;
        self.suspicious_since = None;
        self.last_clean_since = None;
        self.drift.reset();
        self.jump.reset();
        // Drain any stale per-fix flags so the next evaluate() starts fresh.
        // Without this, a `note_dropout_recovery()` / `note_external_anomaly()`
        // call between Spoofed-latch and operator-reset leaves the flag set,
        // and the first post-reset fix would consume it as if the operator
        // had just seen a dropout / frozen-fix.
        self.dropout_recovery = false;
        self.external_anomaly = false;
        self.consecutive_dropout = 0;
        self.paused_cumulative_s = 0.0;
        self.pause_budget_warned = false;
        self.dwell_pause_exceeded_pending = false;
        self.last_dropout_t = None;
    }

    /// Push the FSM back to Suspicious after fusion vetoed a Susp→Normal
    /// transition (e.g. re-anchor plausibility failed). Drift detector state
    /// is preserved automatically because `evaluate()` no longer resets the
    /// detectors on Susp→Normal — fusion must explicitly commit via
    /// `commit_susp_to_normal()` when the transition is accepted.
    ///
    /// AUDIT B-26 fix: `original_suspicious_since` is the timestamp at which
    /// the FSM ENTERED Suspicious for this attack cycle. Previously this
    /// function set `suspicious_since = Some(t_secs)` (the current time),
    /// which RESET the dwell counter — an attacker could cycle veto→reset
    /// indefinitely with a fresh 10s budget each time, never escalating to
    /// Spoofed. Now we preserve the ORIGINAL entry timestamp so cumulative
    /// dwell accrues across vetoes; after enough total veto-cycle time, the
    /// FSM escalates as intended.
    pub fn force_back_to_suspicious(
        &mut self,
        t_secs: f64,
        original_suspicious_since: Option<f64>,
    ) {
        if self.state != NavStateKind::Normal {
            return;
        }
        self.state = NavStateKind::Suspicious;
        // Use the original entry timestamp if known; fall back to now (which
        // preserves prior behavior for safety).
        self.suspicious_since = original_suspicious_since.or(Some(t_secs));
        self.last_clean_since = None;
    }

    /// Called by fusion AFTER it has validated a Susp→Normal transition
    /// (re-anchor plausibility check passed). Resets the detector state so
    /// the next Suspicious entry starts from a clean baseline. Without this
    /// commit step, cumulative CUSUM values from the prior attack would
    /// stick around.
    pub fn commit_susp_to_normal(&mut self) {
        self.drift.reset();
        self.jump.reset();
        // The Suspicious episode genuinely ended — renew the dwell-pause
        // budget. (A VETOED clear never reaches here: force_back_to_suspicious
        // preserves both the original dwell and the consumed pause budget, so
        // veto-cycling can't refresh an attacker's stall allowance.)
        self.paused_cumulative_s = 0.0;
        self.pause_budget_warned = false;
    }

    /// Notify the state machine that a GPS fix was anchored (re-anchor in NORMAL).
    /// Resets CUSUMs because the residual is now measured against a new origin.
    pub fn on_clean_anchor(&mut self) {
        if self.state == NavStateKind::Normal {
            self.drift.reset();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{NedPos, NedVel};

    fn clean() -> Residual {
        Residual {
            dpos: NedPos::default(),
            dvel: NedVel::default(),
            mag_pos: 0.0,
            mag_vel: 0.0,
            dvel_known: true,
            dvel_free: NedVel::default(),
            mag_vel_free: 0.0,
            maneuvering: false,
        }
    }

    fn big_jump() -> Residual {
        Residual {
            dpos: NedPos {
                n: 100.0,
                e: 0.0,
                d: 0.0,
            },
            dvel: NedVel::default(),
            mag_pos: 100.0,
            mag_vel: 0.0,
            dvel_known: true,
            dvel_free: NedVel::default(),
            mag_vel_free: 0.0,
            maneuvering: false,
        }
    }

    /// Residual with explicit N/E components (for noise/drift sequences).
    fn r(n: f64, e: f64) -> Residual {
        Residual {
            dpos: NedPos { n, e, d: 0.0 },
            dvel: NedVel::default(),
            mag_pos: (n * n + e * e).sqrt(),
            mag_vel: 0.0,
            dvel_known: true,
            dvel_free: NedVel::default(),
            mag_vel_free: 0.0,
            maneuvering: false,
        }
    }

    /// Deterministic Gaussian pair (SplitMix64 + Box-Muller); reproducible per
    /// seed, no `rand` dependency.
    fn gauss_pair(state: &mut u64) -> (f64, f64) {
        let mut next = || {
            *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut z = *state;
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            z ^= z >> 31;
            ((z >> 11) as f64 + 1.0) / ((1u64 << 53) as f64 + 1.0)
        };
        let u1 = next();
        let u2 = next();
        let rr = (-2.0 * u1.ln()).sqrt();
        (
            rr * (std::f64::consts::TAU * u2).cos(),
            rr * (std::f64::consts::TAU * u2).sin(),
        )
    }

    /// Feed `n` fixes of clean σ=2.5 m GPS noise so the drift detector learns a
    /// realistic noise floor (and the FSM stays Normal — step 1).
    fn warm_noise_floor(sm: &mut StateMachine, seed: &mut u64, n: usize) {
        for _ in 0..n {
            let (gn, ge) = gauss_pair(seed);
            sm.evaluate(&r(gn * 2.5, ge * 2.5), 0.0, None, None);
        }
    }

    #[test]
    fn jump_takes_us_to_suspicious_then_spoofed() {
        let mut sm = StateMachine::new(DetectConfig::default());
        let evs = sm.evaluate(&big_jump(), 0.0, None, None);
        assert!(evs.iter().any(|e| matches!(
            e,
            Event::Transition {
                to: NavStateKind::Suspicious,
                ..
            }
        )));
        // Run for 12s of firing jumps to cross the 10s dwell.
        for s in 1..=12 {
            sm.evaluate(&big_jump(), s as f64, None, None);
        }
        assert_eq!(sm.state(), NavStateKind::Spoofed);
    }

    #[test]
    fn clearing_clean_returns_to_normal() {
        let mut sm = StateMachine::new(DetectConfig::default());
        sm.evaluate(&big_jump(), 0.0, None, None);
        assert_eq!(sm.state(), NavStateKind::Suspicious);
        // 6 seconds clean -> back to NORMAL.
        for s in 1..=6 {
            sm.evaluate(&clean(), s as f64, None, None);
        }
        assert_eq!(sm.state(), NavStateKind::Normal);
    }

    #[test]
    fn spoofed_latches() {
        let mut sm = StateMachine::new(DetectConfig::default());
        for s in 0..=15 {
            sm.evaluate(&big_jump(), s as f64, None, None);
        }
        assert_eq!(sm.state(), NavStateKind::Spoofed);
        // Even with clean residuals, stay SPOOFED.
        for s in 16..=100 {
            sm.evaluate(&clean(), s as f64, None, None);
        }
        assert_eq!(sm.state(), NavStateKind::Spoofed);
    }

    #[test]
    fn force_back_to_suspicious_preserves_drift_state() {
        let mut sm = StateMachine::new(DetectConfig::default());
        // Drive into Suspicious then clear.
        sm.evaluate(&big_jump(), 0.0, None, None);
        assert_eq!(sm.state(), NavStateKind::Suspicious);
        for s in 1..=6 {
            sm.evaluate(&clean(), s as f64, None, None);
        }
        assert_eq!(sm.state(), NavStateKind::Normal);
        // Operator (or fusion's re-anchor veto) forces back. With None as
        // original-since, falls back to using t_secs as in the pre-audit
        // behavior (safe default).
        sm.force_back_to_suspicious(7.0, None);
        assert_eq!(sm.state(), NavStateKind::Suspicious);
        // No-op when called in Suspicious (already there).
        sm.force_back_to_suspicious(8.0, None);
        assert_eq!(sm.state(), NavStateKind::Suspicious);
        // No-op from Spoofed (latched).
        let mut sm2 = StateMachine::new(DetectConfig::default());
        for s in 0..=15 {
            sm2.evaluate(&big_jump(), s as f64, None, None);
        }
        assert_eq!(sm2.state(), NavStateKind::Spoofed);
        sm2.force_back_to_suspicious(20.0, None);
        assert_eq!(
            sm2.state(),
            NavStateKind::Spoofed,
            "force_back_to_suspicious must not unlatch Spoofed"
        );
    }

    #[test]
    fn force_back_to_suspicious_preserves_original_dwell() {
        // Audit B-26: a vetoed Susp→Normal must NOT reset the dwell counter.
        // Drive into Suspicious at t=0; accumulate firing fixes up to t=9
        // (just under the 10s spoofed dwell — at t=9 the dwell calculation
        // sees 9-0=9 < 10, no Spoofed). Then clear to Normal across 6 fixes
        // (mirrors the `clearing_clean_returns_to_normal` test). Then force
        // back to Suspicious carrying the ORIGINAL since=0.0. A single firing
        // fix at t=16 should escalate to Spoofed because cumulative dwell
        // (16-0=16) is well over the 10s threshold.
        let mut sm = StateMachine::new(DetectConfig::default());
        sm.evaluate(&big_jump(), 0.0, None, None);
        assert_eq!(sm.state(), NavStateKind::Suspicious);
        let original = sm.suspicious_since();
        assert_eq!(original, Some(0.0));
        for s in 1..=9 {
            sm.evaluate(&big_jump(), s as f64, None, None);
        }
        assert_eq!(sm.state(), NavStateKind::Suspicious);
        // 6 clean fixes at t=10..=15 — dwell threshold (5s) crossed at t=15.
        for s in 10..=15 {
            sm.evaluate(&clean(), s as f64, None, None);
        }
        assert_eq!(sm.state(), NavStateKind::Normal);
        // Fusion's veto fires, forcing back with original since.
        sm.force_back_to_suspicious(16.0, original);
        assert_eq!(sm.state(), NavStateKind::Suspicious);
        // Firing fix at t=16 — cumulative dwell from t=0 = 16s, well over 10s.
        sm.evaluate(&big_jump(), 16.0, None, None);
        assert_eq!(
            sm.state(),
            NavStateKind::Spoofed,
            "vetoed re-anchor must NOT grant a fresh 10s dwell window — \
             cumulative dwell must escalate"
        );
    }

    #[test]
    fn force_back_to_suspicious_falls_back_to_now_when_no_original() {
        // Backwards-compat: passing None preserves the prior behavior
        // (sets suspicious_since to the supplied t_secs). Other tests rely
        // on this for non-veto callsites.
        let mut sm = StateMachine::new(DetectConfig::default());
        sm.evaluate(&big_jump(), 0.0, None, None);
        for s in 1..=6 {
            sm.evaluate(&clean(), s as f64, None, None);
        }
        assert_eq!(sm.state(), NavStateKind::Normal);
        sm.force_back_to_suspicious(7.0, None);
        assert_eq!(sm.state(), NavStateKind::Suspicious);
        assert_eq!(sm.suspicious_since(), Some(7.0));
    }

    #[test]
    fn dropout_recovery_does_not_advance_dwell() {
        let mut sm = StateMachine::new(DetectConfig::default());
        // First firing fix at t=0 takes us to SUSPICIOUS.
        sm.evaluate(&big_jump(), 0.0, None, None);
        assert_eq!(sm.state(), NavStateKind::Suspicious);
        // Without dropout marker, 11s of firing fixes => SPOOFED.
        // With dropout marker on EACH intervening fix, dwell timer pauses.
        for s in 1..=11 {
            sm.note_dropout_recovery();
            sm.evaluate(&big_jump(), s as f64, None, None);
        }
        assert_eq!(
            sm.state(),
            NavStateKind::Suspicious,
            "11s of dropout-recovery fixes must NOT advance to SPOOFED"
        );
        // A normal (non-dropout) firing fix should still progress the dwell:
        // after another suspicious_to_spoofed_dwell_s seconds without dropout,
        // we expect SPOOFED.
        for s in 12..=23 {
            sm.evaluate(&big_jump(), s as f64, None, None);
        }
        assert_eq!(sm.state(), NavStateKind::Spoofed);
        assert!(sm.dropout_recovery_count >= 11);
    }

    #[test]
    fn cadence_modulation_cannot_freeze_dwell_forever() {
        // The B-03-class evasion: 5 slow (dropout-flagged) firing fixes, then
        // one fast (unflagged) firing fix, repeating. The fast fix resets the
        // consecutive-dropout streak AND nulls last_dropout_t, so the next
        // pause re-absorbed its dwell progress — net dwell progress per cycle
        // was zero and the FSM sat in Suspicious FOREVER (no Spoofed, no RTL).
        // The cumulative per-episode pause budget (default 20 s) bounds the
        // total stall: once exhausted, dwell accrues and Spoofed latches.
        let mut sm = StateMachine::new(DetectConfig::default());
        sm.evaluate(&big_jump(), 0.0, None, None);
        assert_eq!(sm.state(), NavStateKind::Suspicious);

        let mut t = 0.0_f64;
        let mut fired_pause_exceeded = false;
        // Drive up to 60 s of the modulation pattern. Pre-fix behavior:
        // Suspicious forever. Post-fix: Spoofed well before t=60.
        'outer: for _cycle in 0..10 {
            for _ in 0..5 {
                t += 3.0; // gap > 2.5 s ⇒ fusion flags dropout recovery
                sm.note_dropout_recovery();
                sm.evaluate(&big_jump(), t, None, None);
                fired_pause_exceeded |= sm.dwell_pause_exceeded_pending;
                if sm.state() == NavStateKind::Spoofed {
                    break 'outer;
                }
            }
            t += 1.0; // fast fix, no dropout flag ⇒ streak resets
            sm.evaluate(&big_jump(), t, None, None);
            fired_pause_exceeded |= sm.dwell_pause_exceeded_pending;
            if sm.state() == NavStateKind::Spoofed {
                break 'outer;
            }
        }
        assert_eq!(
            sm.state(),
            NavStateKind::Spoofed,
            "cadence-modulated dropouts must not stall escalation indefinitely \
             (cumulative pause budget must force the dwell to resume)"
        );
        assert!(
            fired_pause_exceeded,
            "exceeding the cumulative pause budget must raise the \
             DwellPauseExceeded flag for operator visibility"
        );
        assert!(
            sm.paused_cumulative_secs() >= DetectConfig::default().max_dwell_pause_s as f64,
            "budget must actually have been consumed (got {})",
            sm.paused_cumulative_secs()
        );
    }

    #[test]
    fn pause_budget_renews_on_committed_clear_but_not_on_veto() {
        let cfg = DetectConfig::default();
        let mut sm = StateMachine::new(cfg);
        // Enter Suspicious and burn most of the pause budget.
        sm.evaluate(&big_jump(), 0.0, None, None);
        let mut t = 0.0;
        for _ in 0..6 {
            t += 3.0;
            sm.note_dropout_recovery();
            sm.evaluate(&big_jump(), t, None, None);
        }
        let consumed = sm.paused_cumulative_secs();
        assert!(consumed > 0.0);

        // Vetoed clear: force_back preserves the consumed budget.
        let original = sm.suspicious_since();
        for k in 1..=6 {
            sm.evaluate(&clean(), t + k as f64, None, None);
        }
        assert_eq!(sm.state(), NavStateKind::Normal);
        sm.force_back_to_suspicious(t + 7.0, original);
        assert_eq!(sm.state(), NavStateKind::Suspicious);
        assert_eq!(
            sm.paused_cumulative_secs(),
            consumed,
            "a vetoed re-anchor must NOT refresh the attacker's stall budget"
        );

        // Committed clear: budget renews for the next genuine episode.
        sm.commit_susp_to_normal();
        assert_eq!(sm.paused_cumulative_secs(), 0.0);
    }

    #[test]
    fn dropouts_in_normal_do_not_raise_dwell_pause_event() {
        // The dwell-pause concept only exists in Suspicious; a dropout streak
        // while Normal used to set the pending flag and emit a spurious
        // operator event.
        let mut sm = StateMachine::new(DetectConfig::default());
        let mut t = 0.0;
        for _ in 0..20 {
            t += 3.0;
            sm.note_dropout_recovery();
            sm.evaluate(&clean(), t, None, None);
            assert_eq!(sm.state(), NavStateKind::Normal);
            assert!(
                !sm.dwell_pause_exceeded_pending,
                "dropout streak in Normal must not raise DwellPauseExceeded"
            );
        }
    }

    #[test]
    fn pos_detectors_gate_blocks_drift_jump_but_not_external() {
        // Step 2 no-Doppler policy: with the cumulative-residual detectors gated
        // off, a large position residual must NOT escalate (drift/jump suppressed)
        // — but a frozen-GPS / vertical-rate anomaly (the `note_external_anomaly`
        // path) MUST still escalate.
        let mut sm = StateMachine::new(DetectConfig::default());
        sm.set_pos_detectors_enabled(false);
        for s in 0..=15 {
            sm.evaluate(&big_jump(), s as f64, None, None);
        }
        assert_eq!(
            sm.state(),
            NavStateKind::Normal,
            "gated cumulative-residual detectors must not latch on a big residual"
        );
        sm.note_external_anomaly();
        let evs = sm.evaluate(&big_jump(), 16.0, None, None);
        assert!(
            evs.iter().any(|e| matches!(
                e,
                Event::Transition {
                    to: NavStateKind::Suspicious,
                    ..
                }
            )),
            "an external (frozen/vertical) anomaly must still escalate while gated"
        );

        // Re-enabling restores jump/drift detection.
        let mut sm2 = StateMachine::new(DetectConfig::default());
        sm2.set_pos_detectors_enabled(false);
        sm2.set_pos_detectors_enabled(true);
        sm2.evaluate(&big_jump(), 0.0, None, None);
        assert_eq!(
            sm2.state(),
            NavStateKind::Suspicious,
            "re-enabled detectors must fire again"
        );
    }

    #[test]
    fn manual_reset_clears_spoofed_and_resumes_detection() {
        let mut sm = StateMachine::new(DetectConfig::default());
        // Drive into SPOOFED.
        for s in 0..=15 {
            sm.evaluate(&big_jump(), s as f64, None, None);
        }
        assert_eq!(sm.state(), NavStateKind::Spoofed);
        // Operator clears the latch.
        sm.manual_reset_to_normal();
        assert_eq!(sm.state(), NavStateKind::Normal);
        // Detector continues to work after reset.
        sm.evaluate(&big_jump(), 200.0, None, None);
        assert_eq!(sm.state(), NavStateKind::Suspicious);
    }

    #[test]
    fn noisy_clean_flight_clears_from_suspicious() {
        // Clear-gate fix: at realistic GPS noise (σ=2.5 m ⇒ ~3 m residual), a
        // clean flight must be able to CLEAR back to Normal after a transient
        // anomaly. With the old sub-noise pos_thresh (cusum_k_m = 1 m) the
        // instantaneous residual was essentially never < 1 m, so the FSM got
        // STUCK in Suspicious. The noise-aware threshold tracks the learned floor.
        let mut sm = StateMachine::new(DetectConfig::default());
        let mut seed = 0x1234_5678_9ABC_DEF0u64;
        warm_noise_floor(&mut sm, &mut seed, 60);
        assert_eq!(
            sm.state(),
            NavStateKind::Normal,
            "noise alone must not fire"
        );

        // A transient external anomaly (e.g. a brief vertical-rate blip) → Suspicious.
        sm.note_external_anomaly();
        sm.evaluate(&r(0.0, 0.0), 100.0, None, None);
        assert_eq!(sm.state(), NavStateKind::Suspicious);

        // Realistic-noise clean fixes past normal_clear_dwell_s must CLEAR.
        let mut t = 100.0;
        for _ in 0..60 {
            t += 1.0;
            let (gn, ge) = gauss_pair(&mut seed);
            sm.evaluate(&r(gn * 2.5, ge * 2.5), t, None, None);
            if sm.state() == NavStateKind::Normal {
                break;
            }
        }
        assert_eq!(
            sm.state(),
            NavStateKind::Normal,
            "a clean noisy flight must clear from Suspicious (noise-aware clear gate)"
        );
    }

    #[test]
    fn accumulating_drift_is_not_cleared_away() {
        // Clear-gate SAFETY: a drift above the detection floor but below the
        // generous noise-aware clean threshold keeps a CUSUM elevated, so it must
        // NOT be classified clean and cleared (which would reset the CUSUM each
        // cycle and let the attack never escalate). It must instead escalate.
        let mut sm = StateMachine::new(DetectConfig::default());
        let mut seed = 0x0F0F_0F0F_0F0F_0F0Fu64;
        warm_noise_floor(&mut sm, &mut seed, 60);
        sm.note_external_anomaly();
        sm.evaluate(&r(0.0, 0.0), 100.0, None, None);
        assert_eq!(sm.state(), NavStateKind::Suspicious);

        // Sustained ~7 m north drift: above eff_k (~5 m) so it accumulates the
        // per-axis CUSUM, below pos_thresh (~9 m) so a naive threshold would call
        // it "clean". The quiescent gate must prevent a clear; it escalates.
        let mut t = 100.0;
        for _ in 0..40 {
            t += 1.0;
            sm.evaluate(&r(7.0, 0.0), t, None, None);
        }
        assert_ne!(
            sm.state(),
            NavStateKind::Normal,
            "an accumulating drift must not be cleared away"
        );
    }
}
