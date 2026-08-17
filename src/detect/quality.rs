//! Constellation-quality discontinuity detector — the takeover-moment lane.
//!
//! # Why this exists
//!
//! Every other detector in this crate is RESIDUAL-based: it needs the spoofer
//! to have already moved the vehicle's apparent position away from truth by
//! enough to clear a noise floor. That is a real and deliberate design, but it
//! has one structural consequence, and `docs/threats.md` states it plainly as
//! the accepted bound of the velocity-aiding lane: a competent spoofer takes
//! over ALIGNED (its first fix matches truth) and walks off slowly enough to
//! stay under the floor. During that alignment window the residual is ~0 and
//! nothing in the residual family can fire, because there is nothing to see.
//!
//! But the takeover itself is not free. To capture a receiver, a spoofer must
//! out-power the live signals and hand the receiver a SIMULATED constellation.
//! That simulated constellation is almost never a byte-perfect continuation of
//! the real one: the satellite count steps, and the dilution-of-precision steps
//! with it — very often *downward*, because a synthesized geometry is cleaner
//! than anything a real sky with real obstructions produces.
//!
//! So this lane watches the metadata rather than the position, and it is
//! strongest exactly where the residual lanes are weakest — at t=0 of the
//! attack, before any drift has accumulated. The two families are orthogonal:
//! one sees the spoof's EFFECT, this one sees its ONSET.
//!
//! # What it deliberately does NOT claim
//!
//! **A spoofer that controls the receiver's output can forge these fields.**
//! Everything here is metadata the attacker's own signal generator produces, so
//! a careful adversary holds the satellite count and HDOP steady across the
//! handover and this lane sees nothing. That is not a reason to skip it — it
//! raises the cost of a clean takeover from "out-power the sky" to "out-power
//! the sky AND model the victim's current constellation well enough to fake
//! continuity into it" — but it IS a reason never to describe this as coverage.
//! It is corroboration, not proof, and it is listed that way in the threat
//! model.
//!
//! It is also blind to **meaconing** (rebroadcast of the genuine signals),
//! which by construction preserves the real constellation. That case is covered
//! elsewhere: at boot by the home-anchor check, and in flight by the position
//! jump the rebroadcast delay produces.
//!
//! # Why it cannot sever GPS on its own
//!
//! The FSM treats an external anomaly as "a detector fired", and a SUSTAINED
//! firing across the whole suspicious→spoofed dwell would reach Spoofed and cut
//! the GPS. A quality discontinuity is by definition a transient, so this
//! detector is built to be **one-shot per step**: it fires on the transition,
//! immediately re-baselines onto the new level, and goes quiet. A genuine
//! takeover therefore escalates to Suspicious — loud, visible, dwell running —
//! and then needs a residual lane to corroborate before anything is severed.
//!
//! [`REFRACTORY_S`] closes the remaining hole: a receiver (or an attacker)
//! oscillating the reported quality would otherwise fire on every step and
//! sustain the latch through repetition. Bounding the report rate means this
//! lane can raise the alarm but can never, by itself, talk the FSM into
//! severing GPS on an aircraft whose position data is fine.

use crate::types::GpsFix;

/// Fixes with usable quality data required before any baseline is trusted.
/// A cold receiver climbs from 0 satellites to its working count over the first
/// seconds of lock; treating that ramp as a discontinuity would fire on every
/// boot.
pub const WARMUP_FIXES: u32 = 12;

/// Consecutive fixes a deviation must hold before it counts. One-sample sat
/// dropouts are ordinary (a satellite clipped by an airframe leg, a momentary
/// multipath null); a takeover persists.
pub const PERSIST_FIXES: u32 = 2;

/// Seconds of silence enforced after a report. Bounds how much a flapping
/// receiver — or an attacker deliberately oscillating the metadata — can
/// contribute toward the spoofed dwell.
pub const REFRACTORY_S: f64 = 30.0;

/// EWMA weight for the baselines. Deliberately slow: real constellation
/// changes (a satellite rising or setting) take minutes, so the baseline should
/// represent "the sky we have been flying under", not the last few fixes.
pub const BASELINE_ALPHA: f64 = 0.05;

/// What kind of discontinuity was seen. Both directions matter, which is the
/// point of naming them separately in the event payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShiftKind {
    /// Satellite count stepped (either way).
    SatCount,
    /// HDOP stepped. An abrupt IMPROVEMENT is as suspicious as a degradation:
    /// synthesized geometry tends to be implausibly good.
    Dop,
    /// Both stepped on the same fix — the strongest form, and the ordinary
    /// signature of a constellation swap.
    Both,
}

/// A reported discontinuity, carrying the evidence for the operator log.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QualityShift {
    pub kind: ShiftKind,
    pub sats_baseline: Option<f64>,
    pub sats_now: Option<u8>,
    pub hdop_baseline: Option<f64>,
    pub hdop_now: Option<f32>,
}

/// Tunables. Defaults are sized against ordinary receiver behaviour, not
/// against any particular spoofer.
#[derive(Debug, Clone, Copy)]
pub struct QualityConfig {
    /// Satellite-count deviation from baseline that counts as a step. A real
    /// sky loses or gains satellites one at a time; 4 at once is a different
    /// sky.
    pub sats_step: f64,
    /// HDOP deviation from baseline that counts as a step.
    pub hdop_step: f64,
    /// Set false to disable the lane entirely (operators flying receivers with
    /// erratic quality reporting).
    pub enabled: bool,
}

impl Default for QualityConfig {
    fn default() -> Self {
        Self {
            sats_step: 4.0,
            hdop_step: 0.8,
            enabled: true,
        }
    }
}

/// Tracks the constellation we have been flying under and reports steps away
/// from it.
#[derive(Debug, Default)]
pub struct ConstellationHealth {
    sats_baseline: Option<f64>,
    hdop_baseline: Option<f64>,
    warmed: u32,
    deviating_streak: u32,
    last_report_t: Option<f64>,
}

impl ConstellationHealth {
    pub fn new() -> Self {
        Self::default()
    }

    /// Current baselines, for event payloads and tests.
    pub fn baselines(&self) -> (Option<f64>, Option<f64>) {
        (self.sats_baseline, self.hdop_baseline)
    }

    /// Feed one fix. Returns `Some` exactly on the fix where a discontinuity is
    /// confirmed; the baseline is re-anchored onto the new level at that point,
    /// so a persisting new level does NOT keep reporting.
    pub fn observe(
        &mut self,
        fix: &GpsFix,
        t_secs: f64,
        cfg: &QualityConfig,
    ) -> Option<QualityShift> {
        if !cfg.enabled {
            return None;
        }
        // No quality data at all → nothing to say. Notably we do NOT treat
        // "the receiver stopped reporting quality" as a discontinuity: the
        // NMEA path legitimately omits these fields on some sentence mixes,
        // and firing on that would punish a receiver for being terse.
        let (sats, hdop) = (fix.sats, fix.hdop);
        if sats.is_none() && hdop.is_none() {
            return None;
        }

        let sats_dev = match (sats, self.sats_baseline) {
            (Some(s), Some(b)) => (s as f64 - b).abs() >= cfg.sats_step,
            _ => false,
        };
        let hdop_dev = match (hdop, self.hdop_baseline) {
            (Some(h), Some(b)) => (h as f64 - b).abs() >= cfg.hdop_step,
            _ => false,
        };
        // Classify up front so "something deviated" and "what deviated" are the
        // same fact. Carrying them as two values would leave an impossible
        // (false, false) case to handle at the report site — and a flight loop
        // is the wrong place to answer that with a panic.
        let kind = match (sats_dev, hdop_dev) {
            (true, true) => Some(ShiftKind::Both),
            (true, false) => Some(ShiftKind::SatCount),
            (false, true) => Some(ShiftKind::Dop),
            (false, false) => None,
        };
        let deviating = kind.is_some();

        // Warmup counts only fixes we could actually learn from.
        if self.warmed < WARMUP_FIXES {
            self.warmed += 1;
            self.absorb(sats, hdop);
            return None;
        }

        if !deviating {
            // Quiet fix: this is the only place the baseline moves. Freezing
            // the learner while a deviation is in progress is what stops the
            // baseline from chasing the step and erasing the very signal we are
            // looking for — the same guard the drift lane uses on its noise
            // floor.
            self.deviating_streak = 0;
            self.absorb(sats, hdop);
            return None;
        }

        self.deviating_streak += 1;
        // `deviating` is true, so `kind` is Some by construction.
        let kind = kind?;
        if self.deviating_streak < PERSIST_FIXES {
            return None;
        }

        // Confirmed step. Re-anchor onto the new level FIRST so that whatever
        // happens next, this lane is quiet again — it must not be able to hold
        // the FSM's dwell open by itself.
        let report = QualityShift {
            kind,
            sats_baseline: self.sats_baseline,
            sats_now: sats,
            hdop_baseline: self.hdop_baseline,
            hdop_now: hdop,
        };
        self.sats_baseline = sats.map(|s| s as f64).or(self.sats_baseline);
        self.hdop_baseline = hdop.map(|h| h as f64).or(self.hdop_baseline);
        self.deviating_streak = 0;

        if let Some(last) = self.last_report_t {
            if t_secs - last < REFRACTORY_S {
                // Re-baselined (so we stay quiet), but deliberately silent: a
                // metadata oscillation must not accumulate dwell.
                return None;
            }
        }
        self.last_report_t = Some(t_secs);
        Some(report)
    }

    /// Blend a quiet fix into the baselines.
    fn absorb(&mut self, sats: Option<u8>, hdop: Option<f32>) {
        if let Some(s) = sats {
            let s = s as f64;
            self.sats_baseline = Some(match self.sats_baseline {
                Some(b) => b + BASELINE_ALPHA * (s - b),
                None => s,
            });
        }
        if let Some(h) = hdop {
            let h = h as f64;
            self.hdop_baseline = Some(match self.hdop_baseline {
                Some(b) => b + BASELINE_ALPHA * (h - b),
                None => h,
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Timestamp;

    fn fix(sats: Option<u8>, hdop: Option<f32>) -> GpsFix {
        GpsFix {
            t: Timestamp::now_mono(),
            lat_deg: 47.0,
            lon_deg: 8.0,
            alt_m: 500.0,
            speed_mps: Some(5.0),
            course_deg: Some(90.0),
            hdop,
            sats,
        }
    }

    /// Drive `n` steady fixes starting at `t0`, one per second.
    fn steady(h: &mut ConstellationHealth, n: u32, t0: f64, sats: u8, hdop: f32) -> f64 {
        let cfg = QualityConfig::default();
        let mut t = t0;
        for _ in 0..n {
            assert_eq!(
                h.observe(&fix(Some(sats), Some(hdop)), t, &cfg),
                None,
                "steady sky must stay quiet at t={t}"
            );
            t += 1.0;
        }
        t
    }

    #[test]
    fn steady_constellation_never_reports() {
        let mut h = ConstellationHealth::new();
        steady(&mut h, 600, 0.0, 11, 0.9);
    }

    #[test]
    fn warmup_suppresses_reports_while_the_baseline_is_forming() {
        // A cold receiver climbing 4 -> 12 satellites is a normal boot, not a
        // takeover. Nothing may fire before the baseline is trusted.
        let mut h = ConstellationHealth::new();
        let cfg = QualityConfig::default();
        for (i, s) in (4u8..=15).enumerate() {
            assert_eq!(
                h.observe(&fix(Some(s), Some(2.0)), i as f64, &cfg),
                None,
                "no report during warmup (sats={s})"
            );
        }
    }

    #[test]
    fn satellite_count_step_is_reported_once_then_the_lane_goes_quiet() {
        // THE core property. A sustained firing would let this lane alone
        // drive Suspicious -> Spoofed and sever a healthy GPS.
        let mut h = ConstellationHealth::new();
        let cfg = QualityConfig::default();
        let mut t = steady(&mut h, 30, 0.0, 12, 0.9);

        // Takeover: 12 -> 6 satellites, and it persists.
        assert_eq!(
            h.observe(&fix(Some(6), Some(0.9)), t, &cfg),
            None,
            "first deviating fix only arms the streak"
        );
        t += 1.0;
        let shift = h
            .observe(&fix(Some(6), Some(0.9)), t, &cfg)
            .expect("a persisting sat-count step must report");
        assert_eq!(shift.kind, ShiftKind::SatCount);
        assert_eq!(shift.sats_now, Some(6));

        // The new level persists — and the lane must NOT keep firing.
        for _ in 0..200 {
            t += 1.0;
            assert_eq!(
                h.observe(&fix(Some(6), Some(0.9)), t, &cfg),
                None,
                "a persisting new level must not re-report"
            );
        }
    }

    #[test]
    fn single_fix_glitch_is_ignored() {
        // One satellite briefly clipped by an airframe leg is not an attack.
        let mut h = ConstellationHealth::new();
        let cfg = QualityConfig::default();
        let t = steady(&mut h, 30, 0.0, 12, 0.9);
        assert_eq!(h.observe(&fix(Some(5), Some(0.9)), t, &cfg), None);
        // Recovers on the very next fix → streak broken, nothing reported.
        assert_eq!(h.observe(&fix(Some(12), Some(0.9)), t + 1.0, &cfg), None);
        steady(&mut h, 30, t + 2.0, 12, 0.9);
    }

    #[test]
    fn suspiciously_perfect_geometry_is_reported() {
        // An abrupt HDOP IMPROVEMENT is a spoof signature, not good news:
        // synthesized constellations are cleaner than real skies.
        let mut h = ConstellationHealth::new();
        let cfg = QualityConfig::default();
        let mut t = steady(&mut h, 30, 0.0, 10, 1.6);
        h.observe(&fix(Some(10), Some(0.4)), t, &cfg);
        t += 1.0;
        let shift = h
            .observe(&fix(Some(10), Some(0.4)), t, &cfg)
            .expect("an abrupt DOP improvement must report");
        assert_eq!(shift.kind, ShiftKind::Dop);
    }

    #[test]
    fn simultaneous_steps_report_as_both() {
        let mut h = ConstellationHealth::new();
        let cfg = QualityConfig::default();
        let mut t = steady(&mut h, 30, 0.0, 12, 1.8);
        h.observe(&fix(Some(6), Some(0.5)), t, &cfg);
        t += 1.0;
        let shift = h.observe(&fix(Some(6), Some(0.5)), t, &cfg).unwrap();
        assert_eq!(
            shift.kind,
            ShiftKind::Both,
            "a constellation swap moves both"
        );
    }

    #[test]
    fn oscillating_metadata_cannot_sustain_the_dwell() {
        // The safety property behind REFRACTORY_S: an attacker (or a sick
        // receiver) flapping the sat count must not be able to fire every few
        // fixes and hold the FSM's suspicious->spoofed dwell open.
        let mut h = ConstellationHealth::new();
        let cfg = QualityConfig::default();
        let mut t = steady(&mut h, 30, 0.0, 12, 0.9);
        let mut reports = 0;
        for cycle in 0..40 {
            let sats = if cycle % 2 == 0 { 5 } else { 13 };
            for _ in 0..PERSIST_FIXES {
                if h.observe(&fix(Some(sats), Some(0.9)), t, &cfg).is_some() {
                    reports += 1;
                }
                t += 1.0;
            }
        }
        let elapsed = t - 30.0;
        let ceiling = (elapsed / REFRACTORY_S).ceil() as usize + 1;
        assert!(
            reports <= ceiling,
            "reports ({reports}) must be bounded by the refractory rate (<= {ceiling}) over {elapsed}s"
        );
    }

    #[test]
    fn absent_quality_data_is_not_a_discontinuity() {
        // A receiver that simply stops reporting HDOP/sats is terse, not
        // hostile — and the NMEA path legitimately does this on some sentence
        // mixes.
        let mut h = ConstellationHealth::new();
        let cfg = QualityConfig::default();
        let mut t = steady(&mut h, 30, 0.0, 12, 0.9);
        for _ in 0..50 {
            assert_eq!(h.observe(&fix(None, None), t, &cfg), None);
            t += 1.0;
        }
        // And the baseline survived the gap, so the sky we return to is still
        // recognised as the same one.
        steady(&mut h, 10, t, 12, 0.9);
    }

    #[test]
    fn slow_natural_drift_is_absorbed_not_reported() {
        // Satellites rise and set over minutes. The baseline must follow that
        // without ever calling it a step.
        let mut h = ConstellationHealth::new();
        let cfg = QualityConfig::default();
        let mut t = steady(&mut h, 30, 0.0, 14, 1.0);
        for i in 0..8 {
            // One satellite lost every 40 fixes: 14 -> 6 over ~5 minutes.
            let sats = 14 - i;
            for _ in 0..40 {
                assert_eq!(
                    h.observe(&fix(Some(sats), Some(1.0)), t, &cfg),
                    None,
                    "gradual constellation change must be absorbed (sats={sats})"
                );
                t += 1.0;
            }
        }
    }

    #[test]
    fn disabled_lane_is_silent() {
        let mut h = ConstellationHealth::new();
        let cfg = QualityConfig {
            enabled: false,
            ..Default::default()
        };
        for i in 0..100 {
            let sats = if i < 50 { 12 } else { 4 };
            assert_eq!(h.observe(&fix(Some(sats), Some(0.9)), i as f64, &cfg), None);
        }
    }
}
