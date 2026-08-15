//! MPU-6050 register-level decode + read-health escalation policy.
//!
//! Everything here is pure logic — no bus I/O — split out of the Linux-only
//! `i2c_imu` driver so it is unit-tested on every platform (same rationale as
//! [`crate::hw::axis_map`]). The driver owns the fd and the tick loop; this
//! module owns WHAT the bytes mean and WHEN a read pattern is trouble.
//!
//! Three hardware failure modes drive the design (audit: hardware-readiness
//! findings; none of these is observable from the samples' VALUES alone):
//!
//! 1. **Wrong chip behind the address** (WHO_AM_I). An MPU-9250/ICM class
//!    part on 0x68 speaks the same protocol but different full-scale
//!    factors — every sample would be silently mis-scaled and the
//!    GPS↔IMU cross-check desensitized. Refuse at init.
//! 2. **Read errors** (cable, pull-ups, EMI, driver unbind). Escalation
//!    ladder: warn once at [`DEGRADED_THRESHOLD`], attempt an in-process
//!    re-open + re-init every [`REINIT_INTERVAL`], give up and end the
//!    stream at [`BUS_DOWN_THRESHOLD`] (systemd restart is the backstop —
//!    but an in-process recovery preserves detector FSM/CUSUM state, which
//!    a process restart destroys).
//! 3. **Frozen output.** A brown-out resets the MPU-6050: PWR_MGMT_1 comes
//!    back with the SLEEP bit set, I2C reads keep SUCCEEDING, but the
//!    sensor registers stop updating. The samples are bit-identical,
//!    individually plausible, and arrive at full rate — the dead-reckoner
//!    would integrate a stale specific-force vector with nothing firing.
//!    (The "DEAD-RECKONING STALLED" escalation only catches MISSING
//!    samples.) Detect bit-identical raw frames and route them into the
//!    same ladder; the re-init step re-writes PWR_MGMT_1 and wakes the
//!    chip. The raw 14-byte frame includes the temperature register, whose
//!    LSB (1/340 °C) dithers continuously on an awake die, so a genuine
//!    [`FROZEN_OUTPUT_THRESHOLD`]-long run of identical frames does not
//!    occur on a healthy chip even when the vehicle is perfectly still.

use crate::hw::axis_map::AxisMap;
use crate::types::{ImuSample, Timestamp};

/// I2C address with AD0 low (0x69 with AD0 high — operator-configurable).
pub const MPU6050_DEFAULT_ADDR: u8 = 0x68;

/// Register addresses (MPU-6050 Register Map RM-MPU-6000A v4.2).
pub const REG_PWR_MGMT_1: u8 = 0x6B;
pub const REG_GYRO_CONFIG: u8 = 0x1B;
pub const REG_ACCEL_CONFIG: u8 = 0x1C;
pub const REG_ACCEL_XOUT_H: u8 = 0x3B;
pub const REG_WHO_AM_I: u8 = 0x75;

/// WHO_AM_I for the MPU-6000/6050 die (both report 0x68, independent of the
/// AD0 pin / bus address).
pub const WHO_AM_I_MPU6050: u8 = 0x68;

/// Scale factors at default ranges (±2g, ±250 °/s).
pub const G_PER_LSB: f32 = 1.0 / 16384.0;
pub const DEG_S_PER_LSB: f32 = 1.0 / 131.0;
pub const G_TO_MPS2: f32 = 9.806_65;

/// One burst read: AX/AY/AZ/TEMP/GX/GY/GZ as 16-bit big-endian.
///
/// Register-level atomicity: the MPU-60X0 latches sensor registers from its
/// internal shadow set only while the serial interface is idle, so a burst
/// read is guaranteed coherent (no mid-frame tearing) — on a GENUINE part.
/// That guarantee is one more reason the WHO_AM_I gate is load-bearing.
pub const RAW_FRAME_LEN: usize = 14;

/// Verify the chip identity byte read from [`REG_WHO_AM_I`].
///
/// Strict on purpose: "same protocol, different scales" substitutes (MPU-6500,
/// MPU-9250, ICM class) would mis-scale every sample by a constant factor the
/// plausibility gate cannot see. For a device whose job is a quantitative
/// GPS↔IMU cross-check, delivering mis-scaled IMU data is worse than refusing
/// to start.
pub fn check_who_am_i(read: u8) -> Result<(), String> {
    if read == WHO_AM_I_MPU6050 {
        Ok(())
    } else {
        Err(format!(
            "IMU WHO_AM_I mismatch: chip at the configured address reports {read:#04x}, \
             expected 0x68 (MPU-6050/6000). Common values: 0x70=MPU-6500, 0x71=MPU-9250, \
             0x73=MPU-9255, 0x12=ICM-20602. A substituted chip has different full-scale \
             factors, so its samples would be silently mis-scaled — refusing to feed the \
             detector. Fix the wiring/address, or port the driver (register map + scale \
             constants) to the actual chip."
        ))
    }
}

/// Decode one raw burst frame into an SI-unit, body-frame [`ImuSample`]
/// stamped with the current monotonic time.
pub fn sample_from_raw(raw: &[u8; RAW_FRAME_LEN], axis_map: &AxisMap) -> ImuSample {
    let ax = i16::from_be_bytes([raw[0], raw[1]]);
    let ay = i16::from_be_bytes([raw[2], raw[3]]);
    let az = i16::from_be_bytes([raw[4], raw[5]]);
    // raw[6..8] = TEMP: not part of the sample, but it participates in the
    // frozen-output comparison in `ReadHealth` (an awake die dithers it).
    let gx = i16::from_be_bytes([raw[8], raw[9]]);
    let gy = i16::from_be_bytes([raw[10], raw[11]]);
    let gz = i16::from_be_bytes([raw[12], raw[13]]);

    // Scale to SI in the chip frame, then remap chip axes → FRD body frame for
    // the mounting orientation. The remap is a proper rotation, so the same map
    // applies to accel and gyro. Identity map = pass-through (default).
    let accel = axis_map.apply([
        ax as f32 * G_PER_LSB * G_TO_MPS2,
        ay as f32 * G_PER_LSB * G_TO_MPS2,
        az as f32 * G_PER_LSB * G_TO_MPS2,
    ]);
    let gyro = axis_map.apply([
        (gx as f32 * DEG_S_PER_LSB).to_radians(),
        (gy as f32 * DEG_S_PER_LSB).to_radians(),
        (gz as f32 * DEG_S_PER_LSB).to_radians(),
    ]);

    ImuSample {
        t: Timestamp::now_mono(),
        accel_mps2: accel,
        gyro_rps: gyro,
    }
}

/// AUDIT E-14: after this many consecutive bad reads, emit a single loud
/// warning so a dying bus is visible on operator dashboards. ~1 s at 100 Hz.
pub const DEGRADED_THRESHOLD: u32 = 100;
/// Attempt an in-process re-open + re-init at every multiple of this many
/// consecutive bad reads (~2 s cadence at 100 Hz: attempts at 200/400/600/800
/// before the bus-down budget runs out). Recovering in-process preserves the
/// detector's FSM/CUSUM/dead-reckoning state; the systemd restart path loses
/// all of it plus 2–5 s of coverage.
pub const REINIT_INTERVAL: u32 = 200;
/// After this many consecutive bad reads (~10 s at 100 Hz), give up and end
/// the stream so systemd restarts the process and re-initializes from clean.
/// The detector must NOT keep running GPS-only past this point: the
/// cross-check defense REQUIRES IMU samples to be meaningful.
pub const BUS_DOWN_THRESHOLD: u32 = 1000;
/// Consecutive bit-identical raw frames before the stream is declared frozen
/// (~1 s at 100 Hz). The 14-byte frame includes the temperature register,
/// whose LSB dithers continuously on an awake die, so a healthy chip does not
/// produce runs anywhere near this long even at perfect standstill.
pub const FROZEN_OUTPUT_THRESHOLD: u32 = 100;

/// What went wrong with a read (for log wording).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TroubleKind {
    /// The bus transaction itself failed.
    ReadError,
    /// The transaction succeeded but returned a bit-identical frame for
    /// [`FROZEN_OUTPUT_THRESHOLD`] consecutive reads — sensor registers are
    /// not updating (asleep after brown-out, or wedged).
    FrozenOutput,
}

/// Escalation decision for one bad read. Field order mirrors the ladder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Trouble {
    pub kind: TroubleKind,
    /// Length of the current bad-read streak, this read included.
    pub consecutive_errors: u32,
    /// Fires exactly once per streak, at [`DEGRADED_THRESHOLD`].
    pub warn_degraded: bool,
    /// Caller should drop the fd and re-run open+init (wakes a slept chip).
    pub reinit: bool,
    /// Error budget exhausted — end the stream (systemd takes over).
    pub end_stream: bool,
}

/// Verdict for a read that returned bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// Sample is fresh — decode and yield it. `recovered_after` carries the
    /// length of the bad-read streak this read ended, if any.
    Fresh { recovered_after: Option<u32> },
    /// Frozen-output streak at/past threshold: do NOT yield (the frame is
    /// stale data, not a measurement) and act on the ladder.
    Frozen(Trouble),
}

/// Pure decision core for the driver's read loop: tracks error streaks and
/// frozen-output runs, and tells the caller when to warn / re-init / give up.
#[derive(Debug, Default)]
pub struct ReadHealth {
    consecutive_errors: u32,
    last_raw: Option<[u8; RAW_FRAME_LEN]>,
    frozen_run: u32,
}

impl ReadHealth {
    pub fn new() -> Self {
        Self::default()
    }

    /// A read returned a frame. Detect frozen output; otherwise reset the
    /// ladder and report recovery.
    pub fn on_read_ok(&mut self, raw: &[u8; RAW_FRAME_LEN]) -> Verdict {
        if self.last_raw.as_ref() == Some(raw) {
            self.frozen_run = self.frozen_run.saturating_add(1);
            if self.frozen_run >= FROZEN_OUTPUT_THRESHOLD {
                return Verdict::Frozen(self.escalate(TroubleKind::FrozenOutput));
            }
        } else {
            self.last_raw = Some(*raw);
            self.frozen_run = 0;
        }
        let recovered_after = (self.consecutive_errors > 0).then_some(self.consecutive_errors);
        self.consecutive_errors = 0;
        Verdict::Fresh { recovered_after }
    }

    /// The bus transaction failed. NB: `last_raw` is deliberately kept — an
    /// error gap between two identical frames does not prove freshness, so a
    /// frozen run continues across interleaved read errors.
    pub fn on_read_err(&mut self) -> Trouble {
        self.escalate(TroubleKind::ReadError)
    }

    fn escalate(&mut self, kind: TroubleKind) -> Trouble {
        self.consecutive_errors = self.consecutive_errors.saturating_add(1);
        let end_stream = self.consecutive_errors >= BUS_DOWN_THRESHOLD;
        Trouble {
            kind,
            consecutive_errors: self.consecutive_errors,
            // `==` fires exactly once per streak: the counter is monotone
            // until a fresh read resets it.
            warn_degraded: self.consecutive_errors == DEGRADED_THRESHOLD,
            reinit: !end_stream && self.consecutive_errors.is_multiple_of(REINIT_INTERVAL),
            end_stream,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(seed: u8) -> [u8; RAW_FRAME_LEN] {
        let mut f = [0u8; RAW_FRAME_LEN];
        f[0] = seed;
        f[7] = seed.wrapping_add(1); // temp low byte — the natural ditherer
        f
    }

    #[test]
    fn who_am_i_accepts_mpu6050() {
        assert!(check_who_am_i(0x68).is_ok());
    }

    #[test]
    fn who_am_i_rejects_substitutes_with_actionable_message() {
        let err = check_who_am_i(0x71).unwrap_err();
        assert!(err.contains("0x71"), "message names the value read: {err}");
        assert!(
            err.contains("MPU-9250"),
            "message identifies the likely chip: {err}"
        );
        assert!(check_who_am_i(0x70).is_err());
        assert!(check_who_am_i(0x12).is_err());
        assert!(check_who_am_i(0x00).is_err());
    }

    #[test]
    fn sample_decode_known_bytes() {
        // AX = 16384 LSB = 1 g on X; GZ = 131 LSB = 1 °/s on Z; rest zero.
        let mut raw = [0u8; RAW_FRAME_LEN];
        raw[0..2].copy_from_slice(&16384i16.to_be_bytes());
        raw[12..14].copy_from_slice(&131i16.to_be_bytes());
        let s = sample_from_raw(&raw, &AxisMap::IDENTITY);
        assert!((s.accel_mps2[0] - G_TO_MPS2).abs() < 1e-4);
        assert_eq!(s.accel_mps2[1], 0.0);
        assert_eq!(s.accel_mps2[2], 0.0);
        assert!((s.gyro_rps[2] - 1f32.to_radians()).abs() < 1e-6);
    }

    #[test]
    fn fresh_reads_stay_quiet() {
        let mut h = ReadHealth::new();
        for i in 0..1000u32 {
            match h.on_read_ok(&frame(i as u8)) {
                Verdict::Fresh {
                    recovered_after: None,
                } => {}
                v => panic!("unexpected verdict on fresh read {i}: {v:?}"),
            }
        }
    }

    #[test]
    fn error_streak_warns_once_at_degraded() {
        let mut h = ReadHealth::new();
        for i in 1..DEGRADED_THRESHOLD {
            let t = h.on_read_err();
            assert!(!t.warn_degraded, "no warn at {i}");
            assert!(!t.end_stream);
        }
        let t = h.on_read_err();
        assert!(t.warn_degraded, "warn exactly at {DEGRADED_THRESHOLD}");
        let t = h.on_read_err();
        assert!(!t.warn_degraded, "warn does not repeat within a streak");
    }

    #[test]
    fn reinit_requested_on_interval_but_not_at_bus_down() {
        let mut h = ReadHealth::new();
        let mut reinits = Vec::new();
        for i in 1..=BUS_DOWN_THRESHOLD {
            let t = h.on_read_err();
            if t.reinit {
                reinits.push(i);
            }
            assert_eq!(t.end_stream, i >= BUS_DOWN_THRESHOLD, "end only at budget");
        }
        assert_eq!(reinits, vec![200, 400, 600, 800]);
    }

    #[test]
    fn recovery_resets_the_ladder_and_reports_streak() {
        let mut h = ReadHealth::new();
        for _ in 0..50 {
            h.on_read_err();
        }
        match h.on_read_ok(&frame(1)) {
            Verdict::Fresh {
                recovered_after: Some(50),
            } => {}
            v => panic!("expected recovery report, got {v:?}"),
        }
        // Ladder restarted: the next streak warns at the threshold again.
        for i in 1..DEGRADED_THRESHOLD {
            assert!(!h.on_read_err().warn_degraded, "no early warn at {i}");
        }
        assert!(h.on_read_err().warn_degraded);
    }

    #[test]
    fn frozen_output_detected_at_threshold_and_not_yielded() {
        let mut h = ReadHealth::new();
        let f = frame(7);
        // First sight of the frame is fresh; identical repeats below the
        // threshold still pass (bounded stale exposure by design).
        assert!(matches!(h.on_read_ok(&f), Verdict::Fresh { .. }));
        for _ in 1..FROZEN_OUTPUT_THRESHOLD {
            assert!(matches!(h.on_read_ok(&f), Verdict::Fresh { .. }));
        }
        // Threshold reached: dropped + escalated, and it keeps escalating
        // while the freeze persists.
        match h.on_read_ok(&f) {
            Verdict::Frozen(t) => {
                assert_eq!(t.kind, TroubleKind::FrozenOutput);
                assert_eq!(t.consecutive_errors, 1);
            }
            v => panic!("expected frozen verdict, got {v:?}"),
        }
        for _ in 0..(REINIT_INTERVAL - 1) {
            match h.on_read_ok(&f) {
                Verdict::Frozen(_) => {}
                v => panic!("freeze persists: {v:?}"),
            }
        }
        // A differing frame ends the freeze and reports the streak.
        match h.on_read_ok(&frame(8)) {
            Verdict::Fresh {
                recovered_after: Some(n),
            } => assert_eq!(n, REINIT_INTERVAL),
            v => panic!("expected recovery, got {v:?}"),
        }
    }

    #[test]
    fn frozen_run_survives_interleaved_read_errors() {
        let mut h = ReadHealth::new();
        let f = frame(3);
        h.on_read_ok(&f);
        for _ in 1..FROZEN_OUTPUT_THRESHOLD {
            h.on_read_ok(&f);
        }
        // An error gap does not prove freshness — the same frame after the
        // gap is still frozen, and both kinds accumulate one shared ladder.
        let t = h.on_read_err();
        assert_eq!(t.consecutive_errors, 1);
        match h.on_read_ok(&f) {
            Verdict::Frozen(t) => assert_eq!(t.consecutive_errors, 2),
            v => panic!("expected frozen after error gap, got {v:?}"),
        }
    }

    #[test]
    fn frozen_and_errors_reach_reinit_together() {
        let mut h = ReadHealth::new();
        let f = frame(9);
        h.on_read_ok(&f);
        for _ in 1..FROZEN_OUTPUT_THRESHOLD {
            h.on_read_ok(&f);
        }
        let mut saw_reinit = false;
        for _ in 0..REINIT_INTERVAL {
            let trouble = match h.on_read_ok(&f) {
                Verdict::Frozen(t) => t,
                v => panic!("expected frozen, got {v:?}"),
            };
            if trouble.reinit {
                saw_reinit = true;
                assert_eq!(trouble.consecutive_errors, REINIT_INTERVAL);
            }
        }
        assert!(saw_reinit, "a persistent freeze must request re-init");
    }

    #[test]
    fn frozen_streak_eventually_ends_stream() {
        let mut h = ReadHealth::new();
        let f = frame(4);
        h.on_read_ok(&f);
        for _ in 1..FROZEN_OUTPUT_THRESHOLD {
            h.on_read_ok(&f);
        }
        let mut ended = false;
        for _ in 0..BUS_DOWN_THRESHOLD {
            if let Verdict::Frozen(t) = h.on_read_ok(&f) {
                if t.end_stream {
                    ended = true;
                    break;
                }
            }
        }
        assert!(ended, "a re-init-immune freeze must exhaust the budget");
    }
}
