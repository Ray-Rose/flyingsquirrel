//! Inertial dead-reckoning.
//!
//! Pure math layer — no async, no I/O. `DeadReckoner` consumes IMU samples
//! at the IMU rate and maintains an attitude+velocity+position estimate in
//! a local NED tangent plane whose origin is re-anchored on every clean GPS
//! fix while the system is in `NavStateKind::Normal`.

pub mod attitude;
pub mod strapdown;

use crate::error::NavError;
use crate::types::{GpsFix, ImuSample, NavStateKind, NedPos, NedVel};
use attitude::Madgwick;
use strapdown::{BiasEstimator, Strapdown};
use tokio::time::Instant;

/// WGS-84 mean Earth radius (meters). Equirectangular tangent-plane projection only.
pub const R_EARTH_M: f64 = 6_378_137.0;
/// Standard gravity, m/s².
pub const G: f64 = 9.806_65;
/// Default upper bound on the IMU integration step. Past this, we assume the
/// IMU stalled (e.g. channel backpressure, runtime starvation) and skip the
/// integration rather than amplify a stale-bias error into meters of false
/// position drift. This default suits a ~50–100 Hz IMU (10–20 ms period).
///
/// AUDIT (rate-gate): this MUST be sized from the *actual* IMU rate. A real
/// ArduPilot/PX4 link streams `SCALED_IMU` at SR2_RAW_SENS (the README
/// recommends 4 Hz = 250 ms; SITL ran ~10 Hz = 100 ms). At those periods a
/// fixed 100 ms bound skips EVERY integration step — dead-reckoning never runs,
/// no residual is ever computed, and the detector fails *open* (silently blind).
/// So `NavConfig.max_imu_dt_s` is derived from the configured IMU rate at
/// startup (see the binary's `--imu-rate` handling); this constant is only the
/// fallback default for callers that don't override it.
const DEFAULT_MAX_IMU_DT_S: f32 = 0.10;

/// Multiplier on the nominal IMU inter-sample period used to size the
/// integration-gap gate (`max_imu_dt_s`). ~2.5 sample periods of slack absorbs
/// normal link jitter without admitting a meters-scale false position step.
/// Mirrors the binary's `--imu-rate` startup sizing so the auto-derive below and
/// the configured value agree when the operator set `--imu-rate` correctly.
const IMU_GATE_PERIOD_FACTOR: f32 = 2.5;

/// Hard cap on the gate when auto-derived from the observed stream. A real IMU /
/// autopilot raw-sensor stream is >= ~4 Hz; a stream slow enough to need a gate
/// beyond this (~2.5 Hz effective) is treated as broken or manipulated and left
/// to the `DEAD-RECKONING STALLED` warning rather than silently accommodated —
/// this bounds how far a boot-time cadence manipulation could widen the runtime
/// stall gate.
const AUTO_IMU_GATE_MAX_S: f32 = 1.0;

/// Number of consecutive integration-gap skips after which dead-reckoning is
/// effectively not running and we escalate from a per-step debug line to a
/// single loud warning. ~1 s of a 25 Hz stream, but capped low so a genuinely
/// mis-sized gate (e.g. `--imu-rate 100` against a real 4 Hz link) is surfaced
/// within a second rather than silently failing open.
const IMU_STARVATION_SKIP_THRESHOLD: u32 = 25;

/// `recent_gyro_mag` EWMA gains — asymmetric: rise fast (engage the maneuvering
/// gate within ~0.1 s of a turn starting, before the velocity-aiding lane can
/// accumulate a false fire), fall slow (~1 s, so the gate persists through the
/// post-turn velocity-settling transient).
const MANEUVER_GYRO_ALPHA_RISE: f32 = 0.1;
const MANEUVER_GYRO_ALPHA_FALL: f32 = 0.01;
/// Bias-corrected gyro magnitude (rad/s) above which the vehicle is treated as
/// maneuvering for the velocity-aiding gate. ~2.3°/s — well above straight-flight
/// gyro noise (σ≈0.002 rad/s) yet far below a normal coordinated turn (a 60 m,
/// 8 m/s circle is ω≈0.13 rad/s). Straight legs (where a consistent-velocity
/// walk-off is observable) stay below it; turns sit far above.
const MANEUVER_GYRO_THRESHOLD_RPS: f32 = 0.04;

#[derive(Debug, Clone, Copy)]
pub struct NavConfig {
    pub madgwick_beta: f32,
    pub imu_bias_window_s: f32,
    pub blend_tau_s: f32,
    pub dead_reckon_freeze_on_suspicious: bool,
    pub bias_ema_alpha: f32,
    pub static_init_s: f32,
    pub yaw_from_gps_speed_min_mps: f32,
    /// Operator-supplied expected location of the drone at boot, as
    /// `(lat_deg, lon_deg)`. If set, the first GPS fix must be within
    /// `max_home_distance_m` of this point or we emit `BootAnchorRejected`
    /// and refuse to anchor. Defends against the meaconing-at-boot attack
    /// where an attacker is already broadcasting a spoofed location when
    /// the drone powers up — without this check, we'd silently anchor to
    /// their grid.
    pub expected_home: Option<(f64, f64)>,
    /// Plausibility radius around `expected_home` (meters). Sized to cover
    /// initial GPS uncertainty plus typical home-position imprecision.
    pub max_home_distance_m: f64,
    /// Upper bound (seconds) on a single IMU integration step before it's
    /// treated as a stall and skipped. MUST be sized from the real IMU rate:
    /// too small (relative to the inter-sample period) and every step is
    /// skipped, so dead-reckoning never runs and the detector fails open. The
    /// binary derives this from `--imu-rate`; see `DEFAULT_MAX_IMU_DT_S`.
    pub max_imu_dt_s: f32,
}

impl Default for NavConfig {
    fn default() -> Self {
        Self {
            madgwick_beta: 0.05,
            imu_bias_window_s: 5.0,
            blend_tau_s: 3.0,
            dead_reckon_freeze_on_suspicious: true,
            bias_ema_alpha: 0.01,
            static_init_s: 1.0,
            yaw_from_gps_speed_min_mps: 2.0,
            expected_home: None,
            max_home_distance_m: 1000.0,
            max_imu_dt_s: DEFAULT_MAX_IMU_DT_S,
        }
    }
}

/// Snapshot used by the residual ring buffer.
#[derive(Debug, Clone, Copy)]
pub struct NavStateSnapshot {
    pub mono_secs: f64,
    pub pos: NedPos,
    pub vel: NedVel,
}

pub struct DeadReckoner {
    cfg: NavConfig,
    attitude: Madgwick,
    strapdown: Strapdown,
    bias: BiasEstimator,
    origin: Option<(f64, f64, f64)>,
    /// Shared time anchor — must match fusion's `boot`. All `mono_secs` values
    /// (in snapshots, in buffer lookups) are seconds since this point.
    boot: Instant,
    last_imu_mono: Option<f64>,
    initialized: bool,
    static_init_accum: usize,
    static_init_target: usize,
    static_init_accel_sum: [f64; 3],
    /// Monotonic-seconds of the FIRST static-init IMU sample. With the last
    /// init sample (`last_imu_mono` at completion) and the sample count, this
    /// yields the OBSERVED stream cadence, used to auto-size `max_imu_dt_s` so a
    /// mis-set `--imu-rate` can't silently fail dead-reckoning open.
    static_init_first_mono: Option<f64>,
    state: NavStateKind,
    yaw_seeded_from_gps: bool,
    /// Monotonic-seconds of the last GPS fix, for computing the actual
    /// inter-fix interval used by the complementary-blend gain (audit N-02).
    last_gps_mono: Option<f64>,
    /// True once the first GPS fix arrived AFTER static-init completed. Before
    /// this point, every clean GPS fix re-anchors origin (to absorb the lag
    /// between when origin was first set and when IMU integration actually
    /// started). After this point, the anchor is locked — re-anchoring would
    /// destroy the cumulative drift signal the CUSUM detector relies on.
    anchor_locked: bool,
    /// Count of consecutive IMU samples skipped because their inter-sample gap
    /// exceeded `cfg.max_imu_dt_s`. Reset to 0 on any successful integration.
    /// Used to escalate a mis-sized gate (which silently disables dead-reckoning)
    /// from per-step debug spam to a single loud warning.
    consecutive_imu_skips: u32,
    /// Latches true once the starvation warning has fired for the current run
    /// of skips, so we warn ONCE per streak rather than every sample. Cleared on
    /// the next successful integration.
    imu_starvation_warned: bool,
    /// Cumulative velocity (per NED axis) imported into the dead-reckoned
    /// velocity by the complementary blend SINCE the anchor — i.e. exactly
    /// `v_blended − v_free_inertial` (both tracks start equal at the anchor and
    /// integrate the same accel, so their running difference is just the sum of
    /// blend corrections). This is the GPS-velocity-INDEPENDENT signal the
    /// velocity-aiding lane needs: a smart / EKF-laundered consistent-velocity
    /// walk-off drives the position residual AND the post-blend velocity residual
    /// to ~0 (the blend tracks the spoofed velocity), but the velocity it had to
    /// import to do so persists here as a bias ≈ the spoof rate. Reset to 0 only
    /// at an anchor reset (where both tracks coincide).
    aiding_n: f64,
    aiding_e: f64,
    /// Asymmetric EWMA of the bias-corrected gyro magnitude (rad/s) — fast to
    /// rise, slow to fall. Drives `is_maneuvering()`, which gates the
    /// velocity-aiding detection lane. A sustained coordinated turn makes the
    /// FREE-INERTIAL velocity track diverge from GPS velocity by ~2 m/s (legit
    /// attitude/centripetal error the blend masks for position); the
    /// velocity-aiding lane would read that as a walk-off. The pure-inertial
    /// velocity reference is only trustworthy in benign (near-straight,
    /// un-accelerated) flight, so the lane is suspended while maneuvering.
    recent_gyro_mag: f32,
}

impl DeadReckoner {
    pub fn new(cfg: NavConfig, boot: Instant) -> Self {
        Self {
            cfg,
            attitude: Madgwick::new(cfg.madgwick_beta),
            strapdown: Strapdown::new(),
            bias: BiasEstimator::new(cfg.bias_ema_alpha, cfg.imu_bias_window_s),
            origin: None,
            boot,
            last_imu_mono: None,
            initialized: false,
            static_init_accum: 0,
            static_init_target: 0,
            static_init_accel_sum: [0.0; 3],
            static_init_first_mono: None,
            state: NavStateKind::Normal,
            yaw_seeded_from_gps: false,
            last_gps_mono: None,
            anchor_locked: false,
            consecutive_imu_skips: 0,
            imu_starvation_warned: false,
            aiding_n: 0.0,
            aiding_e: 0.0,
            recent_gyro_mag: 0.0,
        }
    }

    pub fn set_state(&mut self, state: NavStateKind) {
        self.state = state;
    }

    pub fn state(&self) -> NavStateKind {
        self.state
    }

    /// Process one IMU sample. Returns a snapshot iff the reckoner has finished
    /// static-init AND has a GPS-anchored NED origin.
    pub fn step(&mut self, sample: &ImuSample) -> Result<Option<NavStateSnapshot>, NavError> {
        // Defense in depth: even though `mav::imu_sample_is_plausible` should have
        // already gated this, a synth or hardware path could still produce
        // non-finite values. NaN propagation through the strapdown integration
        // would silently disable downstream detection.
        for v in &sample.accel_mps2 {
            if !v.is_finite() {
                return Err(NavError::NonFinite("accel"));
            }
        }
        for v in &sample.gyro_rps {
            if !v.is_finite() {
                return Err(NavError::NonFinite("gyro"));
            }
        }

        let mono_secs: f64 = self.mono_secs(sample.t.mono);

        if !self.initialized {
            if self.static_init_target == 0 {
                self.static_init_target =
                    (self.cfg.static_init_s as f64 * 100.0).max(10.0) as usize;
            }
            if self.static_init_first_mono.is_none() {
                self.static_init_first_mono = Some(mono_secs);
            }
            for k in 0..3 {
                self.static_init_accel_sum[k] += sample.accel_mps2[k] as f64;
            }
            self.static_init_accum += 1;
            self.last_imu_mono = Some(mono_secs);

            if self.static_init_accum >= self.static_init_target {
                let n = self.static_init_accum as f64;
                let avg = [
                    (self.static_init_accel_sum[0] / n) as f32,
                    (self.static_init_accel_sum[1] / n) as f32,
                    (self.static_init_accel_sum[2] / n) as f32,
                ];
                // AUDIT N-10 fix: sanity-check the boot gravity vector before
                // trusting it to seed attitude. A disconnected IMU (reads ~0),
                // a mis-scaled IMU (wrong LSB→m/s² factor), or one that's in
                // motion / freefall at boot would otherwise initialize the
                // attitude filter to garbage with NO operator-visible signal,
                // silently disabling dead-reckoning and therefore the entire
                // cross-check. We require the static-average magnitude to look
                // like Earth gravity (1 g ± ~8%); outside that, we still
                // initialize (best-effort) but emit a loud warning so the
                // operator knows the IMU calibration / mounting is suspect.
                let mag = ((avg[0] * avg[0] + avg[1] * avg[1] + avg[2] * avg[2]) as f64).sqrt();
                if !(9.0..=10.6).contains(&mag) {
                    tracing::warn!(
                        gravity_mag = mag,
                        ax = avg[0], ay = avg[1], az = avg[2],
                        "BOOT IMU SANITY: static-average accel magnitude is {:.2} m/s², not ~9.81. \
                         IMU may be disconnected, mis-scaled, or moving at boot. Dead-reckoning \
                         (and the GPS cross-check) will be unreliable until this is corrected.",
                        mag
                    );
                }
                // AUDIT N-01 fix: the nav stack now uses the physical
                // SPECIFIC-FORCE convention — a level, stationary sensor reads
                // −g on the body Down axis ([0,0,-9.81]), as real MEMS IMUs and
                // ArduPilot/PX4 SCALED_IMU/HIGHRES_IMU emit. A strongly POSITIVE
                // Down-axis average at rest means the IMU is reporting the
                // gravity-reaction sign (the opposite); `init_from_accel` would
                // then seed ~180° roll and dead-reckoning would be mirrored
                // during maneuvers. Warn loudly so the operator sign-normalizes
                // their driver/mounting (the I2C path in particular passes raw
                // chip axes through — see hw/i2c_imu.rs).
                if avg[2] > 5.0 {
                    tracing::warn!(
                        az = avg[2],
                        "BOOT IMU SANITY: Down-axis accel is POSITIVE at rest ({:.2} m/s²). \
                         The nav stack expects the specific-force convention (−g down), as \
                         real IMUs and ArduPilot/PX4 emit. A gravity-reaction IMU needs its \
                         Z (and horizontal) axes sign-normalized before ingest, or \
                         dead-reckoning will be wrong during maneuvers. See docs/threats.md §6.",
                        avg[2]
                    );
                }
                self.attitude.init_from_accel(avg);
                self.initialized = true;

                // Auto-size the IMU integration-gap gate from the OBSERVED
                // static-init cadence. The binary sizes `max_imu_dt_s` from
                // `--imu-rate` at startup, but if the operator leaves the 100 Hz
                // default against a real ~10 Hz autopilot stream the gate is far
                // too tight: every ~100 ms sample is skipped, dead-reckoning
                // never runs, and the detector fails OPEN (silently blind). Now
                // the actual cadence measured during static-init refines the
                // gate. We only ever WIDEN (never tighten — tightening could
                // false-trip the stall gate on jitter), and only up to a cap
                // (an implausibly slow stream is left to the STALLED warning, so
                // a boot-time cadence manipulation can't widen the runtime stall
                // gate without bound). This runs once, before any post-init
                // integration, so runtime stall detection stays sharp.
                if let (Some(first), Some(last)) = (self.static_init_first_mono, self.last_imu_mono)
                {
                    let span = (last - first) as f32;
                    let intervals = self.static_init_accum.saturating_sub(1);
                    if span > 0.0 && intervals > 0 {
                        let observed_period = span / intervals as f32;
                        let observed_hz = 1.0 / observed_period;
                        let needed_gate = (IMU_GATE_PERIOD_FACTOR * observed_period)
                            .clamp(DEFAULT_MAX_IMU_DT_S, AUTO_IMU_GATE_MAX_S);
                        if needed_gate > self.cfg.max_imu_dt_s {
                            tracing::warn!(
                                observed_hz,
                                observed_period_ms = observed_period * 1000.0,
                                old_gate_ms = self.cfg.max_imu_dt_s * 1000.0,
                                new_gate_ms = needed_gate * 1000.0,
                                "IMU rate auto-derived from the observed stream (~{:.1} Hz): the \
                                 configured integration-gap gate ({:.0} ms) was too tight and \
                                 would skip nearly every sample, so dead-reckoning (and the GPS \
                                 cross-check) would fail OPEN. Widened the gate to {:.0} ms. Set \
                                 --imu-rate ~{:.0} to match the real stream and silence this.",
                                observed_hz,
                                self.cfg.max_imu_dt_s * 1000.0,
                                needed_gate * 1000.0,
                                observed_hz,
                            );
                            self.cfg.max_imu_dt_s = needed_gate;
                        } else {
                            tracing::info!(
                                observed_hz,
                                gate_ms = self.cfg.max_imu_dt_s * 1000.0,
                                "IMU init: observed stream ~{:.1} Hz; integration-gap gate \
                                 ({:.0} ms) is adequate",
                                observed_hz,
                                self.cfg.max_imu_dt_s * 1000.0,
                            );
                        }
                    }
                }
            }
            return Ok(None);
        }

        // dt must be bounded on BOTH sides: a stalled IMU producing a huge gap
        // would propagate as a meters-scale false position step under trapezoidal
        // integration. Also reject monotonic regressions defensively — Instant
        // is supposed to be monotonic but some platforms have surprised us.
        let dt = match self.last_imu_mono {
            Some(prev) => {
                let raw = (mono_secs - prev) as f32;
                if !raw.is_finite() || raw <= 0.0 {
                    // Time stood still or went backwards — skip integration this step,
                    // keep nav state stable, advance the last-time marker.
                    self.last_imu_mono = Some(mono_secs);
                    return Ok(None);
                }
                if raw > self.cfg.max_imu_dt_s {
                    self.consecutive_imu_skips = self.consecutive_imu_skips.saturating_add(1);
                    // Escalate from per-step debug to ONE loud warning once a run
                    // of skips means dead-reckoning has effectively stopped — the
                    // signature of a gate sized too small for the real IMU rate,
                    // which otherwise fails the detector open in silence.
                    if self.consecutive_imu_skips >= IMU_STARVATION_SKIP_THRESHOLD
                        && !self.imu_starvation_warned
                    {
                        self.imu_starvation_warned = true;
                        tracing::warn!(
                            consecutive_skips = self.consecutive_imu_skips,
                            dt_ms = raw * 1000.0,
                            max_imu_dt_ms = self.cfg.max_imu_dt_s * 1000.0,
                            "DEAD-RECKONING STALLED: {} consecutive IMU samples skipped — the \
                             inter-sample gap ({:.0} ms) exceeds the integration limit ({:.0} ms), \
                             so the GPS cross-check is NOT running. If this persists, the configured \
                             IMU rate (--imu-rate) is higher than the actual stream rate; set it to \
                             match the real IMU / autopilot raw-sensor rate.",
                            self.consecutive_imu_skips,
                            raw * 1000.0,
                            self.cfg.max_imu_dt_s * 1000.0,
                        );
                    } else {
                        tracing::debug!(
                            dt_ms = raw * 1000.0,
                            max_imu_dt_ms = self.cfg.max_imu_dt_s * 1000.0,
                            "imu gap exceeded max_imu_dt_s; skipping integration step"
                        );
                    }
                    self.last_imu_mono = Some(mono_secs);
                    return Ok(None);
                }
                raw.max(1e-6)
            }
            None => 0.01,
        };
        self.last_imu_mono = Some(mono_secs);
        // A within-bounds dt means the IMU is streaming at a rate the gate
        // accepts — clear any starvation tracking accrued during a prior gap.
        self.consecutive_imu_skips = 0;
        self.imu_starvation_warned = false;

        let allow_bias = self.state == NavStateKind::Normal;
        self.bias
            .observe(&sample.accel_mps2, &sample.gyro_rps, dt, allow_bias);

        let gyro_corr = [
            sample.gyro_rps[0] - self.bias.gyro()[0],
            sample.gyro_rps[1] - self.bias.gyro()[1],
            sample.gyro_rps[2] - self.bias.gyro()[2],
        ];

        // Track the bias-corrected gyro magnitude (asymmetric EWMA) so the
        // velocity-aiding detection lane can be suspended while maneuvering —
        // see `recent_gyro_mag` / `is_maneuvering`.
        let gmag = (gyro_corr[0] * gyro_corr[0]
            + gyro_corr[1] * gyro_corr[1]
            + gyro_corr[2] * gyro_corr[2])
            .sqrt();
        let a = if gmag > self.recent_gyro_mag {
            MANEUVER_GYRO_ALPHA_RISE
        } else {
            MANEUVER_GYRO_ALPHA_FALL
        };
        self.recent_gyro_mag = (1.0 - a) * self.recent_gyro_mag + a * gmag;

        // GPS-INDEPENDENT centripetal compensation (cause #3 fix). The
        // accelerometer measures specific force f = a_kinematic − g, and in a
        // sustained coordinated turn a_kinematic is the centripetal ω × v. Left
        // in, that ~1 m/s² horizontal term tilts the Madgwick "down" ~6° and
        // couples through gravity-comp (`R·f + g` below) into a large DR drift
        // that false-latches Spoofed on a perfectly healthy drone. We estimate
        // it from the bias-corrected gyro and the DEAD-RECKONED velocity — NOT
        // GPS: using the attacker-controlled GPS velocity here would let a
        // spoofed velocity bias the attitude and pull DR toward the attacker's
        // track, defeating the whole GPS-vs-inertial cross-check. v_dr freezes
        // GPS-independent in Suspicious/Spoofed (blend off), so the correction
        // stays honest exactly when it matters. Bounded so a transiently bad
        // gyro/velocity can't swing the gravity reference.
        let v_ned = self.strapdown.vel();
        let v_body = attitude::rotate_ned_to_body(
            &self.attitude.q(),
            &[v_ned.n as f32, v_ned.e as f32, v_ned.d as f32],
        );
        let a_cent_body = bound_centripetal([
            gyro_corr[1] * v_body[2] - gyro_corr[2] * v_body[1],
            gyro_corr[2] * v_body[0] - gyro_corr[0] * v_body[2],
            gyro_corr[0] * v_body[1] - gyro_corr[1] * v_body[0],
        ]);
        self.attitude
            .update_with_lin_accel(&sample.accel_mps2, &gyro_corr, dt, &a_cent_body);

        let q = self.attitude.q();
        // Rotate body-frame specific force into NED, then recover linear accel.
        // Specific force f = Rᵀ(a − g) ⇒ a = R·f + g, with g = [0,0,+G] (Down).
        // So a_lin = (R·f) + [0,0,+G]: ADD gravity on the Down axis. (The old
        // gravity-reaction convention subtracted it; that required a physically
        // impossible [0,0,+g]-at-rest IMU and mirrored real-hardware DR.)
        let a_ned = attitude::rotate_body_to_ned(&q, &sample.accel_mps2);
        let a_lin = [a_ned[0] as f64, a_ned[1] as f64, a_ned[2] as f64 + G];

        if !self.is_initialized() {
            // No origin yet — don't integrate position.
            return Ok(None);
        }

        // Subtract estimated NED-frame accel bias before integrating, and
        // update the bias EMA when quiet. Catches the failure mode the
        // previous audit flagged: without this, accel-bias-induced drift
        // accumulates to meters over a long flight and false-fires the
        // drift detector. The quiet-window gate inside `correct_and_update`
        // makes this safe during maneuvers.
        let a_lin_corr = self.bias.correct_and_update_accel_ned(a_lin, allow_bias);

        self.strapdown.integrate(&a_lin_corr, dt);

        // `q` is still used above for attitude→NED rotation but is not
        // carried into the snapshot — the buffer only needs pos/vel for the
        // residual time-interpolation. A future forensic dump can recompute
        // attitude on demand or capture it separately.
        let _ = q;
        Ok(Some(NavStateSnapshot {
            mono_secs,
            pos: self.strapdown.pos(),
            vel: self.strapdown.vel(),
        }))
    }

    /// Process a GPS fix.
    ///
    /// Before `anchor_locked`: re-anchor origin and reset pos/vel on every clean
    /// fix. This burns through the boot-init transient cleanly.
    ///
    /// After `anchor_locked`: position is NEVER re-anchored — that preserves
    /// the cumulative drift signal the CUSUM detector relies on. Velocity gets
    /// a soft complementary blend.
    ///
    /// On SUSPICIOUS/SPOOFED: freeze blend, never anchor.
    pub fn on_gps_fix(&mut self, fix: &GpsFix) {
        let gps_vel = gps_velocity(fix);

        // Compute the ACTUAL inter-fix interval (audit N-02). The blend gain
        // depends on it; using a hardcoded 1.0 s at 10 Hz GPS made the blend
        // 10× too aggressive, over-trusting GPS and letting a slow spoof pull
        // the predictor toward the spoofed track faster than the τ design.
        let now_mono = self.mono_secs(fix.t.mono);
        let dt_blend = match self.last_gps_mono {
            Some(prev) => {
                let d = (now_mono - prev) as f32;
                // Clamp to a sane band: reject regressions/zero, cap at 5 s so
                // a long dropout doesn't produce a near-1.0 alpha that snaps
                // the predictor onto the resumption fix.
                if d.is_finite() && d > 0.0 {
                    d.min(5.0)
                } else {
                    1.0
                }
            }
            None => 1.0,
        };
        self.last_gps_mono = Some(now_mono);

        // Seed yaw from GPS course once the vehicle is moving fast enough for
        // course to be meaningful. Audit N-05: this used to be gated behind
        // `!anchor_locked`, so a vehicle stationary at boot (no course) that
        // locked its anchor and only THEN started moving would never get yaw
        // seeded — leaving heading at the static-init default and corrupting
        // dead-reckoned horizontal position. Now it can fire at any time until
        // it succeeds once.
        if !self.yaw_seeded_from_gps {
            if let (Some(course), Some(speed)) = (fix.course_deg, fix.speed_mps) {
                if speed as f32 >= self.cfg.yaw_from_gps_speed_min_mps {
                    self.attitude.seed_yaw_deg(course as f32);
                    self.yaw_seeded_from_gps = true;
                }
            }
        }

        // Re-anchor freely until the first GPS fix that arrives after
        // static-init has completed. That fix locks the anchor.
        if !self.anchor_locked {
            self.origin = Some((fix.lat_deg, fix.lon_deg, fix.alt_m));
            self.strapdown.reset();
            if let Some(v) = gps_vel {
                self.strapdown.set_vel(v);
            }
            // Both the blended and the free-inertial velocity tracks coincide at
            // an anchor reset — zero the cumulative aiding so the velocity-aiding
            // lane measures only post-anchor divergence.
            self.aiding_n = 0.0;
            self.aiding_e = 0.0;
            if self.initialized {
                self.anchor_locked = true;
            }
            return;
        }

        // Complementary blend: pull dead-reckoned velocity toward GPS.
        // Skipped on SUSPICIOUS/SPOOFED unless the operator opted out of
        // freezing — that prevents a smart spoofer from gradually walking
        // the predictor toward the spoofed track.
        let do_blend =
            self.state == NavStateKind::Normal || !self.cfg.dead_reckon_freeze_on_suspicious;
        if do_blend {
            let tau = self.cfg.blend_tau_s.max(0.05);
            let alpha = (dt_blend / tau).clamp(0.0, 1.0) as f64;
            if let Some(vg) = gps_vel {
                let v = self.strapdown.vel();
                let cn = alpha * (vg.n - v.n);
                let ce = alpha * (vg.e - v.e);
                // Accumulate the velocity the blend imports from GPS this fix.
                // Summed since the anchor, this is `v_blended − v_free_inertial`
                // (see `aiding_n/e`) — the velocity-aiding lane's input.
                self.aiding_n += cn;
                self.aiding_e += ce;
                self.strapdown.set_vel(NedVel {
                    n: v.n + cn,
                    e: v.e + ce,
                    d: v.d + alpha * (vg.d - v.d),
                });
            }
        }
    }

    /// Manually re-anchor position to zero (called on SUSPICIOUS -> NORMAL transition).
    pub fn re_anchor_after_clearing(&mut self, fix: &GpsFix) {
        self.origin = Some((fix.lat_deg, fix.lon_deg, fix.alt_m));
        self.strapdown.set_pos(NedPos::default());
        // A confirmed-clean re-anchor coincides the two velocity tracks again:
        // drop stale aiding accumulated during the (cleared) suspicious episode.
        self.aiding_n = 0.0;
        self.aiding_e = 0.0;
    }

    /// Cumulative velocity imported from GPS by the complementary blend since the
    /// anchor, i.e. `v_blended − v_free_inertial`. GPS-velocity-independent: a
    /// consistent-velocity walk-off that the blend has absorbed (so the position
    /// and post-blend velocity residuals read ~0) still shows here as a sustained
    /// bias ≈ the spoof rate. Drives the velocity-aiding detection lane.
    pub fn aiding_vel(&self) -> NedVel {
        NedVel {
            n: self.aiding_n,
            e: self.aiding_e,
            d: 0.0,
        }
    }

    /// True when the recent (bias-corrected) gyro magnitude indicates a turn or
    /// other rotational maneuver, during which the free-inertial velocity track
    /// is unreliable and the velocity-aiding lane must be suspended.
    pub fn is_maneuvering(&self) -> bool {
        self.recent_gyro_mag > MANEUVER_GYRO_THRESHOLD_RPS
    }

    pub fn gps_in_ned(&self, fix: &GpsFix) -> Option<NedPos> {
        let (lat0, lon0, _) = self.origin?;
        Some(lla_to_ned(fix.lat_deg, fix.lon_deg, fix.alt_m, lat0, lon0))
    }

    pub fn is_initialized(&self) -> bool {
        self.initialized && self.origin.is_some()
    }

    pub fn origin(&self) -> Option<(f64, f64, f64)> {
        self.origin
    }

    pub fn anchor_locked(&self) -> bool {
        self.anchor_locked
    }

    /// Plausibility check for the FIRST GPS fix only. Returns `Err(distance)`
    /// if `cfg.expected_home` is set and this fix is further than
    /// `cfg.max_home_distance_m` from that point — caller (fusion) emits
    /// `BootAnchorRejected` and refuses to anchor. No-op when no expected
    /// home is configured (operator opted out of the check).
    pub fn is_first_fix_plausible(&self, fix: &GpsFix) -> Result<(), f64> {
        let Some((home_lat, home_lon)) = self.cfg.expected_home else {
            return Ok(());
        };
        if self.anchor_locked {
            // Only check the first-ever fix.
            return Ok(());
        }
        let ned = lla_to_ned(fix.lat_deg, fix.lon_deg, 0.0, home_lat, home_lon);
        let d = ned.horizontal_norm();
        if d <= self.cfg.max_home_distance_m {
            Ok(())
        } else {
            Err(d)
        }
    }

    fn mono_secs(&self, t: Instant) -> f64 {
        t.saturating_duration_since(self.boot).as_secs_f64()
    }
}

fn gps_velocity(fix: &GpsFix) -> Option<NedVel> {
    let speed = fix.speed_mps?;
    let course = fix.course_deg?.to_radians();
    Some(NedVel {
        n: speed * course.cos(),
        e: speed * course.sin(),
        d: 0.0,
    })
}

/// Upper bound (m/s²) on the centripetal compensation magnitude. A real drone's
/// sustained horizontal acceleration is ≲ 2 g; beyond ~3 g the gyro × velocity
/// product is almost certainly a transient fault, so we clamp the magnitude
/// (preserving direction) to keep a bad estimate from swinging the attitude
/// gravity reference. The Madgwick gradient is already normalized (per-step
/// effect bounded by β), so this clamp is defense-in-depth.
const MAX_CENTRIPETAL_MPS2: f32 = 30.0;

/// Clamp the centripetal-compensation vector's magnitude to
/// `MAX_CENTRIPETAL_MPS2`, preserving direction.
fn bound_centripetal(a: [f32; 3]) -> [f32; 3] {
    let mag = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
    // Defense in depth: a non-finite estimate (e.g. NaN from a poisoned DR
    // velocity) would slip through `mag > MAX` (NaN comparisons are false) and
    // NaN the quaternion in `update_with_lin_accel`. Drop to zero compensation,
    // the same as no maneuver. Inputs are finite-gated upstream (nav::step), so
    // this is unreachable today — but the attitude filter must not trust them.
    if !mag.is_finite() {
        return [0.0; 3];
    }
    if mag > MAX_CENTRIPETAL_MPS2 {
        let s = MAX_CENTRIPETAL_MPS2 / mag;
        [a[0] * s, a[1] * s, a[2] * s]
    } else {
        a
    }
}

pub fn lla_to_ned(lat_deg: f64, lon_deg: f64, alt_m: f64, lat0: f64, lon0: f64) -> NedPos {
    let lat0_rad = lat0.to_radians();
    let dlat = (lat_deg - lat0).to_radians();
    let dlon = (lon_deg - lon0).to_radians();
    NedPos {
        n: dlat * R_EARTH_M,
        e: dlon * R_EARTH_M * lat0_rad.cos(),
        d: -alt_m,
    }
}

/// Validate a fix's NED position against a reference NED position with a max
/// horizontal distance. Used for both:
///   - boot-anchor cross-check (reference = expected_home in NED)
///   - re-anchor plausibility on Susp→Normal (reference = dead-reckoned position)
///
/// Returns `Err(distance)` if the fix is farther than `max_m` from `reference`.
pub fn validate_fix_against_reference(
    fix_ned: &NedPos,
    reference: &NedPos,
    max_m: f64,
) -> Result<(), f64> {
    let dn = fix_ned.n - reference.n;
    let de = fix_ned.e - reference.e;
    let d = (dn * dn + de * de).sqrt();
    if d <= max_m {
        Ok(())
    } else {
        Err(d)
    }
}

/// Access the dead-reckoned position (relative to the locked anchor). Used by
/// the re-anchor plausibility check on Susp→Normal.
impl DeadReckoner {
    pub fn dr_pos(&self) -> NedPos {
        self.strapdown.pos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lla_to_ned_zero_at_origin() {
        let p = lla_to_ned(40.0, -100.0, 0.0, 40.0, -100.0);
        assert!(p.n.abs() < 1e-9 && p.e.abs() < 1e-9);
    }

    #[test]
    fn lla_to_ned_one_degree_north_is_circa_111km() {
        let p = lla_to_ned(41.0, -100.0, 0.0, 40.0, -100.0);
        assert!((p.n - 111_319.49).abs() < 10.0, "got n={}", p.n);
        assert!(p.e.abs() < 1.0);
    }

    #[test]
    fn boot_anchor_rejects_far_first_fix() {
        use crate::types::Timestamp;
        use tokio::time::Instant;
        let cfg = NavConfig {
            expected_home: Some((40.0, -100.0)),
            max_home_distance_m: 500.0,
            ..NavConfig::default()
        };
        let nav = DeadReckoner::new(cfg, Instant::now());
        // 1km north of home — outside the 500m radius.
        let far_fix = GpsFix {
            t: Timestamp::now_mono(),
            lat_deg: 40.01, // ≈ 1.1 km north
            lon_deg: -100.0,
            alt_m: 0.0,
            speed_mps: Some(0.0),
            course_deg: Some(0.0),
            hdop: Some(1.0),
            sats: Some(10),
        };
        match nav.is_first_fix_plausible(&far_fix) {
            Err(d) => assert!(d > 500.0, "expected distance > 500m, got {d}"),
            Ok(()) => panic!("expected rejection for fix far from home"),
        }
    }

    #[test]
    fn boot_anchor_accepts_nearby_first_fix() {
        use crate::types::Timestamp;
        use tokio::time::Instant;
        let cfg = NavConfig {
            expected_home: Some((40.0, -100.0)),
            max_home_distance_m: 500.0,
            ..NavConfig::default()
        };
        let nav = DeadReckoner::new(cfg, Instant::now());
        // Same coords as home: well within 500m.
        let near_fix = GpsFix {
            t: Timestamp::now_mono(),
            lat_deg: 40.0,
            lon_deg: -100.0,
            alt_m: 0.0,
            speed_mps: Some(0.0),
            course_deg: Some(0.0),
            hdop: Some(1.0),
            sats: Some(10),
        };
        assert!(nav.is_first_fix_plausible(&near_fix).is_ok());
    }

    #[test]
    fn validate_fix_against_reference_basic() {
        let near = NedPos {
            n: 10.0,
            e: 5.0,
            d: 0.0,
        };
        let ref_pos = NedPos {
            n: 0.0,
            e: 0.0,
            d: 0.0,
        };
        // sqrt(100+25) ≈ 11.18m — within 50m
        assert!(validate_fix_against_reference(&near, &ref_pos, 50.0).is_ok());
        // Far fix
        let far = NedPos {
            n: 100.0,
            e: 0.0,
            d: 0.0,
        };
        match validate_fix_against_reference(&far, &ref_pos, 50.0) {
            Err(d) => assert!((d - 100.0).abs() < 1e-6, "got {d}"),
            Ok(()) => panic!("expected rejection"),
        }
    }

    #[test]
    fn boot_anchor_check_skipped_when_no_expected_home() {
        use crate::types::Timestamp;
        use tokio::time::Instant;
        let nav = DeadReckoner::new(NavConfig::default(), Instant::now());
        let any_fix = GpsFix {
            t: Timestamp::now_mono(),
            lat_deg: 89.99,
            lon_deg: 179.99,
            alt_m: 0.0,
            speed_mps: None,
            course_deg: None,
            hdop: None,
            sats: None,
        };
        // No expected_home configured -> always plausible (operator opted out).
        assert!(nav.is_first_fix_plausible(&any_fix).is_ok());
    }

    /// Drive a `DeadReckoner` through static-init with `samples` level, at-rest
    /// IMU samples spaced `dt` seconds apart, and return it post-init so the
    /// auto-derived `max_imu_dt_s` can be asserted. `static_init_target` is
    /// `static_init_s * 100` (= 100 at the default), so 100 samples complete it.
    fn drive_static_init(dt: f64) -> DeadReckoner {
        use crate::types::{ImuSample, Timestamp};
        use std::time::Duration;
        use tokio::time::Instant;
        let boot = Instant::now();
        let mut dr = DeadReckoner::new(NavConfig::default(), boot);
        for i in 0..100 {
            let s = ImuSample {
                t: Timestamp {
                    mono: boot + Duration::from_secs_f64(i as f64 * dt),
                    utc: None,
                },
                // Level, at rest: specific force is −g on Down (what real IMUs and
                // ArduPilot/PX4 emit), passes the boot gravity sanity gate.
                accel_mps2: [0.0, 0.0, -9.806_65],
                gyro_rps: [0.0, 0.0, 0.0],
            };
            let _ = dr.step(&s);
        }
        assert!(
            dr.initialized,
            "static-init should complete after 100 samples"
        );
        dr
    }

    #[test]
    fn imu_gate_auto_widens_on_slow_stream() {
        // The footgun: configured gate is the 100 ms default (--imu-rate left at
        // 100) but the real stream is ~9 Hz (110 ms). Without the auto-derive
        // every sample's gap exceeds the gate -> dead-reckoning never runs and
        // the detector fails OPEN. The observed cadence must widen the gate.
        assert!((NavConfig::default().max_imu_dt_s - 0.10).abs() < 1e-6);
        let dr = drive_static_init(0.110);
        assert!(
            dr.cfg.max_imu_dt_s > 0.10,
            "gate must widen for a slow stream, got {} ms",
            dr.cfg.max_imu_dt_s * 1000.0
        );
        // ~2.5 × 110 ms = 275 ms.
        assert!(
            (dr.cfg.max_imu_dt_s - 0.275).abs() < 0.03,
            "gate should be ~275 ms, got {} ms",
            dr.cfg.max_imu_dt_s * 1000.0
        );
    }

    #[test]
    fn imu_gate_not_widened_on_fast_stream() {
        // A 100 Hz stream (10 ms): 2.5 × 10 ms = 25 ms is below the 100 ms floor,
        // so the gate stays at the default — no spurious widening.
        let dr = drive_static_init(0.010);
        assert!(
            (dr.cfg.max_imu_dt_s - 0.10).abs() < 1e-6,
            "gate should stay at the 100 ms default for a fast stream, got {} ms",
            dr.cfg.max_imu_dt_s * 1000.0
        );
    }

    #[test]
    fn imu_gate_widen_is_capped_for_implausibly_slow_stream() {
        // ~1.4 Hz (700 ms): 2.5 × 700 ms = 1.75 s would widen without bound; the
        // cap holds it at AUTO_IMU_GATE_MAX_S so a boot-time cadence manipulation
        // can't open the runtime stall gate arbitrarily.
        let dr = drive_static_init(0.700);
        assert!(
            (dr.cfg.max_imu_dt_s - AUTO_IMU_GATE_MAX_S).abs() < 1e-6,
            "gate should be capped at {} s, got {} s",
            AUTO_IMU_GATE_MAX_S,
            dr.cfg.max_imu_dt_s
        );
    }
}
