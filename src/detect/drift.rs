//! Drift detector: two-sided CUSUM per horizontal axis.

use super::residual::Residual;
use super::DetectConfig;

#[derive(Debug, Clone, Copy)]
pub struct CusumState {
    pub s_pos_n: f32,
    pub s_neg_n: f32,
    pub s_pos_e: f32,
    pub s_neg_e: f32,
    /// Magnitude (one-sided) CUSUM accumulator. Catches circular/spiral
    /// drift where the per-axis CUSUMs oscillate around zero.
    pub s_mag: f32,
}

impl Default for CusumState {
    fn default() -> Self {
        Self {
            s_pos_n: 0.0,
            s_neg_n: 0.0,
            s_pos_e: 0.0,
            s_neg_e: 0.0,
            s_mag: 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DriftReason {
    NorthPositive,
    NorthNegative,
    EastPositive,
    EastNegative,
    /// Magnitude CUSUM exceeded threshold — used when the per-axis sums
    /// stay near zero but `|r|` accumulates (circular / spiral drift).
    Magnitude,
}

/// EWMA smoothing for the online magnitude-noise floor (audit B-02). A small
/// alpha tracks the slowly-varying GPS noise environment without chasing
/// per-fix spikes.
const MAG_NOISE_ALPHA: f32 = 0.05;
/// Effective `k_mag` is at least this multiple of the observed residual-
/// magnitude noise floor. `|r|` is Rayleigh-distributed with mean ≈ 1.25·σ,
/// so a factor of 1.5 keeps the per-fix CUSUM increment `|r| − k_mag`
/// negative on average under H0 (no attack), preventing false accumulation.
const MAG_NOISE_FACTOR: f32 = 1.5;
/// EWMA smoothing for the per-axis noise floors. Same rationale + value as
/// `MAG_NOISE_ALPHA`.
const AXIS_NOISE_ALPHA: f32 = 0.05;
/// Effective per-axis `k` is at least this multiple of the observed per-axis
/// noise floor. The floor tracks `E|rn| = σ·√(2/π) ≈ 0.80σ` (mean absolute
/// deviation of a zero-mean Gaussian), so a factor of 2.5 puts `eff_k ≈ 2σ` —
/// comparable to the magnitude lane's `1.5 × 1.25σ ≈ 1.875σ` gate, keeping the
/// per-fix increment `rn − eff_k` negative on average under H0 (no attack).
/// THE PER-AXIS FIX for cause #1 of the realistic false-latch: a fixed k of
/// 0.4σ (the old `cusum_k_m = 1.0` at σ=2.5 m) false-fired on pure GPS noise
/// (~29 fires / 30k fixes). The factor must stay BELOW (smallest drift we must
/// still catch ÷ noise MAD): an 8 m/fix drift at a 0.80σ≈2 m floor tolerates up
/// to 4.0, so 2.5 keeps clear margin (see `per_axis_real_drift_still_fires…`).
const AXIS_NOISE_FACTOR: f32 = 2.5;
/// Number of fixes the noise floors estimate before the CUSUMs are armed.
/// Prevents a bootstrap false-fire while the per-axis and magnitude floors
/// converge from their zero initial values on a noisy real-GPS link. During
/// warmup every CUSUM is held at 0; a spoof in this short window is still
/// caught by the jump detector.
const WARMUP_FIXES: u32 = 20;

pub struct DriftDetector {
    cfg: DetectConfig,
    state: CusumState,
    /// Online EWMA of the residual magnitude `|r|`, updated ONLY while the
    /// magnitude CUSUM is quiescent (`s_mag == 0`) so an in-progress attack
    /// can't inflate it. Drives the adaptive `k_mag` floor (audit B-02):
    /// without it, the fixed `k_mag=1.0` sat below the Rayleigh mean of
    /// real-GPS noise (~3 m at σ=2.5 m) and the magnitude CUSUM false-fired
    /// every ~14 s on real hardware.
    mag_noise_ewma: f32,
    /// Online EWMA of `|rn|` / `|re|` per axis, updated ONLY while THAT axis's
    /// CUSUMs are quiescent (`s_pos == s_neg == 0`). Drives the adaptive
    /// per-axis `eff_k` — the fix for cause #1 of the realistic false-latch
    /// (fixed per-axis k = 0.4σ false-fired on σ=2.5 m GPS noise). Quiescent-
    /// only updating is what prevents a steady or ramping drift from inflating
    /// the floor and absorbing its own signal (a missed detection).
    axis_noise_n: f32,
    axis_noise_e: f32,
    /// Counts fixes until the CUSUMs are armed (see `WARMUP_FIXES`). Shared by
    /// the per-axis and magnitude floors — they learn the same GPS-noise
    /// environment over the same window.
    warmup: u32,
}

impl DriftDetector {
    pub fn new(cfg: DetectConfig) -> Self {
        Self {
            cfg,
            state: CusumState::default(),
            mag_noise_ewma: 0.0,
            axis_noise_n: 0.0,
            axis_noise_e: 0.0,
            warmup: 0,
        }
    }

    pub fn state(&self) -> CusumState {
        self.state
    }

    /// Current adaptive magnitude-noise floor estimate (for observability/tests).
    pub fn mag_noise_floor(&self) -> f32 {
        self.mag_noise_ewma
    }

    pub fn reset(&mut self) {
        // Reset only the CUSUM accumulators. The noise-floor estimates (both
        // magnitude and per-axis) and the warmup counter are properties of the
        // GPS environment, not the attack, so they persist across re-anchors —
        // re-learning them every clear would re-open the bootstrap false-fire
        // window.
        self.state = CusumState::default();
    }

    pub fn evaluate(
        &mut self,
        r: &Residual,
        hdop: Option<f32>,
        sats: Option<u8>,
    ) -> Option<DriftReason> {
        let k = self.cfg.cusum_k_m;
        let mut h = self.cfg.cusum_h_m;
        let k_mag = self.cfg.mag_cusum_k_m;
        let mut h_mag = self.cfg.mag_cusum_h_m;
        // AUDIT B-23 fix: same as jump detector's scale() — require BOTH a
        // bad HDOP AND a low sat count before widening, so an attacker who
        // spoofs ONLY one of them can't reduce drift sensitivity. Both must
        // be present (not the None sentinel for unknown).
        let hdop_bad = matches!(hdop, Some(x) if x > self.cfg.hdop_multipath_threshold);
        let sats_bad = matches!(sats, Some(s) if s < self.cfg.sats_low_threshold);
        if hdop_bad && sats_bad {
            h *= 1.5;
            h_mag *= 1.5;
        }

        let rn = r.dpos.n as f32;
        let re = r.dpos.e as f32;
        let rmag = r.mag_pos as f32;

        // Defense in depth: a single non-finite residual must not touch the
        // accumulators. `(s + NaN - k).max(0.0)` evaluates to 0.0 (f32::max
        // discards NaN), so one NaN would silently WIPE every per-axis sum
        // and s_mag — an evidence-erasure primitive. Worse, NaN folded into
        // the warmup mean made `mag_noise_ewma` permanently NaN, reverting
        // the adaptive magnitude lane to the fixed-k behavior B-02 fixed.
        // Inputs are gated at ingest and nav re-checks finiteness, so this
        // is unreachable today — but the detector is the last line and must
        // not trust upstream.
        if !(rn.is_finite() && re.is_finite() && rmag.is_finite()) {
            tracing::warn!(
                rn,
                re,
                rmag,
                "drift detector: non-finite residual — skipping fix (accumulators preserved)"
            );
            return None;
        }

        // WARMUP: learn ALL noise floors UNCONDITIONALLY (Welford running
        // MEAN) and hold every CUSUM pinned at 0. Two-phase learning avoids a
        // chicken-and-egg problem — if we only learned a floor while its CUSUM
        // was quiescent, the first noisy fix would push the CUSUM positive
        // (floor starts at 0, eff_k at the small base) and freeze the estimate
        // before it reflected reality. The detector is not armed during warmup;
        // a spoof in this short window is still caught by the jump detector. The
        // Welford mean converges to the TRUE mean over the window, unlike a
        // lagging EWMA from zero (audit B-02).
        if self.warmup < WARMUP_FIXES {
            self.warmup += 1;
            let w = self.warmup as f32;
            self.axis_noise_n += (rn.abs() - self.axis_noise_n) / w;
            self.axis_noise_e += (re.abs() - self.axis_noise_e) / w;
            self.mag_noise_ewma += (rmag - self.mag_noise_ewma) / w;
            self.state = CusumState::default();
            return None;
        }

        // ARMED. Update each noise floor by EWMA, but ONLY while its lane is
        // quiescent (CUSUM == 0). An in-progress drift makes a CUSUM positive,
        // which FREEZES that lane's floor — otherwise a steady (constant-offset)
        // or ramping (constant-rate) attack would inflate the floor and absorb
        // its own signal, a missed detection. `|r|` is Rayleigh and `|rn|`/`|re|`
        // are folded-normal, all strictly positive, so a fixed k below their
        // mean accumulates on pure noise; the adaptive `floor × factor` tracks
        // them. Update the floors BEFORE computing eff_k (matches the magnitude
        // lane's ordering).
        if self.state.s_pos_n == 0.0 && self.state.s_neg_n == 0.0 {
            self.axis_noise_n =
                (1.0 - AXIS_NOISE_ALPHA) * self.axis_noise_n + AXIS_NOISE_ALPHA * rn.abs();
        }
        if self.state.s_pos_e == 0.0 && self.state.s_neg_e == 0.0 {
            self.axis_noise_e =
                (1.0 - AXIS_NOISE_ALPHA) * self.axis_noise_e + AXIS_NOISE_ALPHA * re.abs();
        }
        if self.state.s_mag <= 0.0 {
            self.mag_noise_ewma =
                (1.0 - MAG_NOISE_ALPHA) * self.mag_noise_ewma + MAG_NOISE_ALPHA * rmag;
        }

        let eff_k_n = k.max(self.axis_noise_n * AXIS_NOISE_FACTOR);
        let eff_k_e = k.max(self.axis_noise_e * AXIS_NOISE_FACTOR);
        let eff_k_mag = k_mag.max(self.mag_noise_ewma * MAG_NOISE_FACTOR);

        self.state.s_pos_n = (self.state.s_pos_n + rn - eff_k_n).max(0.0);
        self.state.s_neg_n = (self.state.s_neg_n - rn - eff_k_n).max(0.0);
        self.state.s_pos_e = (self.state.s_pos_e + re - eff_k_e).max(0.0);
        self.state.s_neg_e = (self.state.s_neg_e - re - eff_k_e).max(0.0);
        self.state.s_mag = (self.state.s_mag + rmag - eff_k_mag).max(0.0);

        // Per-axis sums dominate when an attack is linear: report those first.
        // (Past warmup here, so all CUSUMs are armed.)
        if self.state.s_pos_n >= h {
            return Some(DriftReason::NorthPositive);
        }
        if self.state.s_neg_n >= h {
            return Some(DriftReason::NorthNegative);
        }
        if self.state.s_pos_e >= h {
            return Some(DriftReason::EastPositive);
        }
        if self.state.s_neg_e >= h {
            return Some(DriftReason::EastNegative);
        }
        // Magnitude catches circular / spiral attacks that the per-axis sums
        // miss because their signed values cancel.
        if self.state.s_mag >= h_mag {
            return Some(DriftReason::Magnitude);
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{NedPos, NedVel};

    fn r(n: f64, e: f64) -> Residual {
        Residual {
            dpos: NedPos { n, e, d: 0.0 },
            dvel: NedVel::default(),
            mag_pos: (n * n + e * e).sqrt(),
            mag_vel: 0.0,
            dvel_known: true,
        }
    }

    #[test]
    fn quiet_residuals_dont_fire() {
        let mut d = DriftDetector::new(DetectConfig::default());
        for _ in 0..1000 {
            assert!(d.evaluate(&r(0.5, 0.3), None, None).is_none());
        }
    }

    /// Feed `WARMUP_FIXES` clean (zero) residuals so the adaptive per-axis and
    /// magnitude floors settle near zero (`eff_k` ≈ base `cusum_k_m`). Without
    /// a clean baseline, warmup learns the attack itself as "noise" (Trap A)
    /// and the lane can never fire — the same reason B-02 restructured the
    /// magnitude tests. Tests that inject an attack from fix 1 must warm first.
    fn warm(d: &mut DriftDetector) {
        for _ in 0..WARMUP_FIXES {
            assert!(d.evaluate(&r(0.0, 0.0), None, None).is_none());
        }
    }

    #[test]
    fn persistent_north_drift_fires() {
        let mut d = DriftDetector::new(DetectConfig::default());
        warm(&mut d);
        // Residual of 2 m/sample beyond eff_k≈1.0 => ~1 m accumulation per
        // sample, need ~25 samples to cross h=25.
        for _ in 0..24 {
            assert!(d.evaluate(&r(2.0, 0.0), None, None).is_none());
        }
        // One more should trip it.
        assert_eq!(
            d.evaluate(&r(2.0, 0.0), None, None),
            Some(DriftReason::NorthPositive)
        );
    }

    #[test]
    fn reset_clears_cusum() {
        let mut d = DriftDetector::new(DetectConfig::default());
        for _ in 0..30 {
            d.evaluate(&r(2.0, 0.0), None, None);
        }
        d.reset();
        assert_eq!(d.state().s_pos_n, 0.0);
        assert_eq!(d.state().s_mag, 0.0);
    }

    #[test]
    fn non_finite_residual_preserves_accumulated_evidence() {
        // A NaN/Inf residual must be a no-op: pre-guard, `(s + NaN - k).max(0)`
        // zeroed every accumulator (evidence wipe) and NaN poisoned the warmup
        // mean permanently. Accumulate 20 m of north evidence, inject NaN and
        // Inf, and confirm the evidence survives and the detector still fires
        // on schedule afterwards.
        let mut d = DriftDetector::new(DetectConfig::default());
        // Clean warmup so the per-axis floor settles near zero (eff_k≈1.0); see
        // `warm` / Trap A. Then accumulate ~20 m of north evidence at ~1 m/fix.
        warm(&mut d);
        for _ in 0..20 {
            assert!(d.evaluate(&r(2.0, 0.0), None, None).is_none());
        }
        let s_before = d.state().s_pos_n;
        let floor_before = d.mag_noise_floor();
        assert!(s_before > 0.0);

        assert!(d.evaluate(&r(f64::NAN, 0.0), None, None).is_none());
        assert!(d.evaluate(&r(f64::INFINITY, 2.0), None, None).is_none());

        assert_eq!(
            d.state().s_pos_n,
            s_before,
            "non-finite residual must not wipe per-axis CUSUM evidence"
        );
        assert!(
            d.mag_noise_floor().is_finite() && d.mag_noise_floor() == floor_before,
            "non-finite residual must not poison the magnitude noise floor"
        );

        // 5 more honest drift fixes complete the 25 m accumulation — the
        // detector picks up exactly where it left off.
        let mut fired = false;
        for _ in 0..5 {
            if d.evaluate(&r(2.0, 0.0), None, None) == Some(DriftReason::NorthPositive) {
                fired = true;
            }
        }
        assert!(fired, "evidence must keep accumulating after the NaN no-op");
    }

    #[test]
    fn circular_drift_caught_by_magnitude_cusum_only() {
        let mut d = DriftDetector::new(DetectConfig::default());
        // AUDIT B-02: realistic scenario is clean flight THEN attack. Feed a
        // clean baseline first so the adaptive noise floor learns the low
        // clean residual and the warmup completes; then the circular attack
        // (5 m magnitude) accumulates against that low floor and fires.
        for i in 0..40 {
            // Small clean noise, ±0.4 m, mean-reverting so per-axis stays calm.
            let s = if i % 2 == 0 { 0.4 } else { -0.4 };
            assert!(
                d.evaluate(&r(s, -s), None, None).is_none(),
                "clean baseline must not fire"
            );
        }
        // Circular drift: per-axis residuals oscillate ±5 m around zero so
        // signed CUSUMs cancel; magnitude is always ~5 m.
        let mut fired_kind = None;
        for i in 0..50 {
            let angle = (i as f32) * 0.7; // radians
            let n = 5.0 * angle.cos() as f64;
            let e = 5.0 * angle.sin() as f64;
            if let Some(reason) = d.evaluate(&r(n, e), None, None) {
                fired_kind = Some(reason);
                break;
            }
        }
        match fired_kind {
            Some(DriftReason::Magnitude) => {} // expected
            other => panic!(
                "expected DriftReason::Magnitude on circular drift after clean baseline, got {:?}",
                other
            ),
        }
    }

    /// Deterministic Gaussian pair via Box-Muller over a tiny LCG. No `rand`
    /// dependency needed in the lib; the stream is reproducible per seed.
    fn gauss_pair(state: &mut u64) -> (f64, f64) {
        let mut next = || {
            // SplitMix64-style step → uniform in (0,1).
            *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut z = *state;
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            z ^= z >> 31;
            // Map to (0,1), avoiding exactly 0 for the ln().
            ((z >> 11) as f64 + 1.0) / ((1u64 << 53) as f64 + 1.0)
        };
        let u1 = next();
        let u2 = next();
        let r = (-2.0 * u1.ln()).sqrt();
        (
            r * (std::f64::consts::TAU * u2).cos(),
            r * (std::f64::consts::TAU * u2).sin(),
        )
    }

    #[test]
    fn magnitude_cusum_no_false_alarm_on_realistic_gps_noise() {
        // AUDIT B-02 regression: 600 s (1 Hz) of clean flight with realistic
        // consumer-GPS noise (σ = 2.5 m per axis → Rayleigh |r| mean ≈ 3.1 m).
        // With the OLD fixed k_mag = 1.0 this false-fired the magnitude CUSUM
        // in ~14 fixes. The adaptive floor must prevent ANY drift fire.
        let mut d = DriftDetector::new(DetectConfig::default());
        let mut seed = 0xC0FF_EE12_3456_789Au64;
        let sigma = 2.5_f64;
        let mut fires = 0;
        for _ in 0..600 {
            let (gn, ge) = gauss_pair(&mut seed);
            let n = gn * sigma;
            let e = ge * sigma;
            if d.evaluate(&r(n, e), None, None).is_some() {
                fires += 1;
            }
        }
        assert_eq!(
            fires,
            0,
            "drift detector false-fired {} time(s) on 600 s of clean σ=2.5 m GPS noise; \
             magnitude noise floor settled at {:.2} m",
            fires,
            d.mag_noise_floor()
        );
        // Sanity: the noise floor should have learned roughly the Rayleigh mean.
        let floor = d.mag_noise_floor();
        assert!(
            floor > 2.0 && floor < 5.0,
            "noise floor estimate off: {floor}"
        );
    }

    #[test]
    fn real_attack_still_fires_after_noise_floor_learned_on_noisy_gps() {
        // Complement to the false-alarm test: after learning a σ=2.5 m noise
        // floor, a genuine sustained NE drift well above the noise must still
        // escalate. Proves the adaptive floor didn't simply desensitize the
        // detector into uselessness.
        let mut d = DriftDetector::new(DetectConfig::default());
        let mut seed = 0x1234_5678_9ABC_DEF0u64;
        let sigma = 2.5_f64;
        // 100 s clean to establish the floor.
        for _ in 0..100 {
            let (gn, ge) = gauss_pair(&mut seed);
            let _ = d.evaluate(&r(gn * sigma, ge * sigma), None, None);
        }
        // Now a 6 m/fix north drift on top of noise — far above the ~3 m floor.
        let mut fired = false;
        for _ in 0..100 {
            let (gn, ge) = gauss_pair(&mut seed);
            let n = 6.0 + gn * sigma;
            let e = ge * sigma;
            if d.evaluate(&r(n, e), None, None).is_some() {
                fired = true;
                break;
            }
        }
        assert!(
            fired,
            "a 6 m/fix drift must still fire even after learning a 2.5 m noise floor"
        );
    }

    #[test]
    fn per_axis_no_false_alarm_on_realistic_gps_noise() {
        // Cause #1 of the realistic false-latch: the PER-AXIS CUSUM k was
        // fixed at cusum_k_m (1.0 m) while only the magnitude lane adapted.
        // At a fixed k of 0.4σ (σ=2.5 m consumer GPS) the per-axis CUSUM is a
        // reflected random walk with too-weak negative drift; its mean run
        // length to h=25 is ~9000 fixes per lane, so over a long flight it
        // WILL false-fire on pure zero-mean GPS noise — severing GPS and
        // RTL'ing a healthy drone. The adaptive per-axis k (eff_k ≈ 2σ) makes
        // that run length astronomically long: zero fires over a multi-hour-
        // equivalent flight. (Perfect-IMU equivalent: the residual is pure GPS
        // measurement noise, isolating cause #1 from the DR-drift term.)
        let mut d = DriftDetector::new(DetectConfig::default());
        let mut seed = 0xA11C_EF00_D123_4567u64;
        let sigma = 2.5_f64;
        let mut fires = 0;
        for _ in 0..30_000 {
            let (gn, ge) = gauss_pair(&mut seed);
            if d.evaluate(&r(gn * sigma, ge * sigma), None, None).is_some() {
                fires += 1;
            }
        }
        assert_eq!(
            fires, 0,
            "per-axis drift lane false-fired {fires} time(s) on 30k fixes of clean \
             σ=2.5 m GPS noise (perfect-IMU equivalent)"
        );
    }

    #[test]
    fn per_axis_real_drift_still_fires_after_learning_noisy_floor() {
        // Anti-desensitization guard (the missed-detection risk): after the
        // adaptive per-axis k learns a σ=2.5 m noise floor, a genuine drift
        // WELL above the noise must still escalate. Proves the adaptive k did
        // not desensitize the lane into uselessness.
        let mut d = DriftDetector::new(DetectConfig::default());
        let mut seed = 0xD1CE_5EED_4321_0000u64;
        let sigma = 2.5_f64;
        for _ in 0..120 {
            let (gn, ge) = gauss_pair(&mut seed);
            let _ = d.evaluate(&r(gn * sigma, ge * sigma), None, None);
        }
        // 8 m/fix north drift on top of noise — well above eff_k (~2σ = 5 m).
        let mut fired = false;
        for _ in 0..100 {
            let (gn, ge) = gauss_pair(&mut seed);
            let n = 8.0 + gn * sigma;
            let e = ge * sigma;
            if d.evaluate(&r(n, e), None, None).is_some() {
                fired = true;
                break;
            }
        }
        assert!(
            fired,
            "an 8 m/fix drift must still fire after learning a 2.5 m noise floor"
        );
    }

    #[test]
    fn linear_drift_still_caught_by_per_axis_first() {
        // Regression: per-axis detection must come first on a clear linear
        // attack so operator gets the more-specific axis reason rather than
        // the generic Magnitude. Warm up on a clean baseline first (Trap A).
        let mut d = DriftDetector::new(DetectConfig::default());
        warm(&mut d);
        let mut reason = None;
        for _ in 0..50 {
            if let Some(rsn) = d.evaluate(&r(2.0, 0.0), None, None) {
                reason = Some(rsn);
                break;
            }
        }
        assert_eq!(reason, Some(DriftReason::NorthPositive));
    }
}
