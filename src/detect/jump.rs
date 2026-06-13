//! Jump detector: velocity-mismatch primary + hard-residual fallback.

use super::residual::Residual;
use super::DetectConfig;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum JumpReason {
    VelocityMismatch,
    HardResidual,
}

pub struct JumpDetector {
    cfg: DetectConfig,
    consecutive_vmismatch: u8,
}

impl JumpDetector {
    pub fn new(cfg: DetectConfig) -> Self {
        Self {
            cfg,
            consecutive_vmismatch: 0,
        }
    }

    /// Apply per-fix sensitivity scaling for HDOP / low-sat conditions.
    ///
    /// AUDIT B-24 fix: previously this used an OR — high HDOP alone (or low
    /// sats alone) widened the detector gates. An attacker who spoofs
    /// `hdop > 4` on every fix gets a free 2× tolerance, defeating jump
    /// detection of 100 m teleports. We now require BOTH a degraded HDOP
    /// AND a low sat count, AND both fields must be present (not the
    /// `None` sentinel from `gps_from_msg` when the autopilot reports
    /// "unknown"). This is still spoofable in principle, but the attacker
    /// now has to forge two correlated quality fields rather than one — and
    /// inconsistent (high HDOP but reported plenty of sats) values now
    /// don't trigger the widening at all, surfacing the inconsistency to
    /// the detector instead of hiding it.
    fn scale(&self, hdop: Option<f32>, sats: Option<u8>) -> (f32, f32) {
        let mut jump = self.cfg.max_jump_m;
        let mut vmismatch = self.cfg.max_velocity_mismatch_mps;
        let hdop_bad = matches!(hdop, Some(h) if h > self.cfg.hdop_multipath_threshold);
        let sats_bad = matches!(sats, Some(s) if s < self.cfg.sats_low_threshold);
        if hdop_bad && sats_bad {
            jump *= 2.0;
            vmismatch *= 1.5;
        }
        (jump, vmismatch)
    }

    pub fn evaluate(
        &mut self,
        r: &Residual,
        hdop: Option<f32>,
        sats: Option<u8>,
    ) -> Option<JumpReason> {
        let (jump_thresh, vmismatch_thresh) = self.scale(hdop, sats);

        // Velocity-mismatch test is skipped entirely when GPS lacked speed/course.
        // Treating absent v_gps as zero would produce a guaranteed false positive
        // on any moving vehicle whose GPS lost Doppler.
        //
        // AUDIT B-37 fix: previously the counter was UNTOUCHED when
        // `dvel_known == false` — an attacker could fire vmismatch on fix N,
        // insert a no-Doppler fix on N+1 (counter unchanged at 1), then fire
        // again on N+2 (counter→2 → fires) — defeating the
        // `jump_persist_fixes=2` "two CONSECUTIVE fires" requirement. We now
        // DECREMENT (saturating to 0) on no-Doppler fixes so the persistence
        // gate truly requires consecutive firing fixes; the attacker can't
        // bridge gaps with no-evidence fixes.
        if r.dvel_known {
            if r.mag_vel as f32 > vmismatch_thresh {
                self.consecutive_vmismatch = self.consecutive_vmismatch.saturating_add(1);
                if self.consecutive_vmismatch >= self.cfg.jump_persist_fixes {
                    return Some(JumpReason::VelocityMismatch);
                }
            } else {
                self.consecutive_vmismatch = 0;
            }
        } else {
            self.consecutive_vmismatch = self.consecutive_vmismatch.saturating_sub(1);
        }
        // Hard-residual test always runs — it's based on position alone.

        if r.mag_pos as f32 > jump_thresh {
            return Some(JumpReason::HardResidual);
        }

        None
    }

    pub fn reset(&mut self) {
        self.consecutive_vmismatch = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{NedPos, NedVel};

    fn r(n: f64, e: f64, vn: f64, ve: f64) -> Residual {
        let mag_pos = (n * n + e * e).sqrt();
        let mag_vel = (vn * vn + ve * ve).sqrt();
        Residual {
            dpos: NedPos { n, e, d: 0.0 },
            dvel: NedVel {
                n: vn,
                e: ve,
                d: 0.0,
            },
            mag_pos,
            mag_vel,
            dvel_known: true,
        }
    }

    fn r_no_vel(n: f64, e: f64) -> Residual {
        let mag_pos = (n * n + e * e).sqrt();
        Residual {
            dpos: NedPos { n, e, d: 0.0 },
            dvel: NedVel::default(),
            mag_pos,
            mag_vel: 0.0,
            dvel_known: false,
        }
    }

    #[test]
    fn missing_gps_velocity_does_not_fire_vmismatch() {
        let mut d = JumpDetector::new(DetectConfig::default());
        // Position residual is small, no v_gps — must not trigger vmismatch even
        // across many consecutive fixes.
        for _ in 0..20 {
            assert_eq!(d.evaluate(&r_no_vel(2.0, 0.5), None, None), None);
        }
    }

    #[test]
    fn fires_on_hard_residual() {
        let mut d = JumpDetector::new(DetectConfig::default());
        assert_eq!(
            d.evaluate(&r(100.0, 0.0, 0.0, 0.0), None, None),
            Some(JumpReason::HardResidual)
        );
    }

    #[test]
    fn vmismatch_requires_persistence() {
        let mut d = JumpDetector::new(DetectConfig::default());
        // One fix above threshold should NOT trigger (persist=2 default).
        assert_eq!(d.evaluate(&r(0.0, 0.0, 20.0, 0.0), None, None), None);
        // Second fix should.
        assert_eq!(
            d.evaluate(&r(0.0, 0.0, 20.0, 0.0), None, None),
            Some(JumpReason::VelocityMismatch)
        );
    }

    #[test]
    fn quiet_doesnt_fire() {
        let mut d = JumpDetector::new(DetectConfig::default());
        for _ in 0..10 {
            assert!(d.evaluate(&r(1.0, 0.5, 0.5, 0.2), None, None).is_none());
        }
    }

    #[test]
    fn high_hdop_alone_does_not_widen_thresholds() {
        // Audit B-24: attacker-controllable HDOP must NOT widen by itself.
        // 80 m residual is over default 50; with attacker-spoofed hdop=10
        // (alone, sats not reported) we expect FIRE — old code would have
        // widened to 100 and missed this.
        let mut d = JumpDetector::new(DetectConfig::default());
        assert!(
            d.evaluate(&r(80.0, 0.0, 0.0, 0.0), Some(10.0), None)
                .is_some(),
            "attacker spoofing hdop alone must NOT desensitize the jump detector"
        );
    }

    #[test]
    fn high_hdop_alone_with_good_sats_does_not_widen() {
        // Attacker spoofs HDOP but the autopilot honestly reports plenty of
        // sats — this inconsistency must be treated as bogus quality info,
        // NOT as multipath.
        let mut d = JumpDetector::new(DetectConfig::default());
        assert!(
            d.evaluate(&r(80.0, 0.0, 0.0, 0.0), Some(10.0), Some(12))
                .is_some(),
            "inconsistent quality (high HDOP + plenty of sats) must not widen"
        );
    }

    #[test]
    fn both_quality_indicators_bad_widens_thresholds() {
        // BOTH hdop > threshold AND sats < threshold → genuine multipath
        // conditions → widen to 100 m threshold. 80 m residual now under.
        let mut d = JumpDetector::new(DetectConfig::default());
        assert!(
            d.evaluate(&r(80.0, 0.0, 0.0, 0.0), Some(10.0), Some(4))
                .is_none(),
            "genuine multipath (both indicators bad) should widen detector gates"
        );
    }

    #[test]
    fn no_doppler_gap_decays_persistence_counter() {
        // Audit B-37: an attacker who inserts a no-Doppler fix between two
        // vmismatch fires must NOT cross the persist=2 threshold. After the
        // first fire (counter=1), the no-Doppler decay drops it to 0, so the
        // next firing fix takes it to 1 — still under threshold.
        let mut d = JumpDetector::new(DetectConfig::default());
        // Fire 1: counter=1, no fire.
        assert_eq!(d.evaluate(&r(0.0, 0.0, 20.0, 0.0), None, None), None);
        // No-Doppler fix: counter decays to 0.
        assert_eq!(d.evaluate(&r_no_vel(0.0, 0.0), None, None), None);
        // Fire 2 after gap: counter=1, still no fire.
        assert_eq!(
            d.evaluate(&r(0.0, 0.0, 20.0, 0.0), None, None),
            None,
            "no-Doppler gap must DECAY the persistence counter, so this fire \
             starts a fresh sequence rather than completing the prior one"
        );
        // Fire 3: counter=2 → fires.
        assert_eq!(
            d.evaluate(&r(0.0, 0.0, 20.0, 0.0), None, None),
            Some(JumpReason::VelocityMismatch)
        );
    }

    #[test]
    fn unknown_quality_does_not_widen() {
        // Audit MAV-01 / B-24 interaction: when gps_from_msg translates
        // UINT16_MAX sentinels to None, the detector should treat that as
        // "no quality info" — neither widen NOR refuse to detect.
        let mut d = JumpDetector::new(DetectConfig::default());
        assert!(
            d.evaluate(&r(80.0, 0.0, 0.0, 0.0), None, None).is_some(),
            "unknown quality (None/None) must fall back to default thresholds"
        );
    }
}
