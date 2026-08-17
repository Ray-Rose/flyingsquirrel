//! Spoofing detection.
//!
//! Two layered detectors over the (GPS - dead-reckoned) residual:
//! - `jump`: instantaneous velocity-mismatch + hard-residual fallback.
//! - `drift`: per-axis CUSUM for slow walk-off.
//!
//! A simple state machine wraps both: NORMAL -> SUSPICIOUS -> SPOOFED (latched).

pub mod drift;
pub mod jump;
/// Constellation-quality discontinuity — the only lane that watches the GPS
/// METADATA rather than the residual, so it can fire at the takeover moment,
/// before any drift has accumulated. Corroboration, not coverage: a spoofer
/// controls these fields and can forge continuity. See the module docs.
pub mod quality;
pub mod residual;
pub mod state_machine;

#[derive(Debug, Clone, Copy)]
pub struct DetectConfig {
    pub max_jump_m: f32,
    pub max_velocity_mismatch_mps: f32,
    pub jump_persist_fixes: u8,
    pub cusum_k_m: f32,
    pub cusum_h_m: f32,
    /// Magnitude-CUSUM reference value. The magnitude `|r|` of a 2D
    /// Gaussian residual follows a Rayleigh distribution with mean ≈ 1.25σ,
    /// so its noise floor is HIGHER than a single-axis floor. We use
    /// `k_mag = cusum_k_m` (equal to per-axis k, not smaller) which gives a
    /// comparable safety margin above the magnitude mean. Catches
    /// circular/spiral drift where the per-axis signed sums oscillate to
    /// near zero.
    pub mag_cusum_k_m: f32,
    /// Magnitude-CUSUM threshold. Set ABOVE the per-axis `cusum_h_m` so
    /// per-axis fires first on a clear linear drift — operators get the
    /// more specific axis reason. Magnitude is reserved for the cases the
    /// per-axis sums truly cannot catch (circular / spiral / dithered).
    pub mag_cusum_h_m: f32,
    pub suspicious_to_spoofed_dwell_s: f32,
    pub normal_clear_dwell_s: f32,
    pub hdop_multipath_threshold: f32,
    pub sats_low_threshold: u8,
    pub gps_dropout_freeze_s: f32,
    /// On a Suspicious→Normal FSM transition, if the new GPS fix is farther
    /// than this from the dead-reckoned position, REFUSE the transition and
    /// stay Suspicious. Sized for legitimate DR drift over a long Suspicious
    /// dwell (~30s × max ground speed). Defends against the "wait out the
    /// dropout pause then re-anchor on a spoofed coordinate" attack.
    pub re_anchor_max_distance_m: f32,
    /// Maximum plausible vertical speed (climb/descent) in m/s. A GPS
    /// altitude that changes faster than this between consecutive fixes is a
    /// vertical-spoof indicator. Audit B-42: all other detectors operate on
    /// the HORIZONTAL residual, so a pure altitude spoof was previously
    /// invisible. This is a GPS-only sanity check (no dependency on the
    /// notoriously-unreliable vertical dead-reckoning) — it catches SUDDEN
    /// altitude teleports. Gradual altitude drift remains a known gap (GPS
    /// altitude is inherently weak; see docs/threats.md). Default 30 m/s is
    /// far above any real multirotor/VTOL climb or descent rate (~10-15 m/s),
    /// so honest flight never trips it.
    pub max_vertical_rate_mps: f32,
    /// CUMULATIVE cap (seconds) on how much suspicious→spoofed dwell time the
    /// GPS-dropout pause may absorb within a single Suspicious episode. The
    /// per-streak `MAX_CONSECUTIVE_DROPOUT` cap alone is bypassable: an
    /// attacker who modulates GPS cadence (a few slow fixes, then one fast
    /// fix, repeating) resets the consecutive counter every cycle and freezes
    /// the dwell timer FOREVER — Spoofed never latches and RTL never fires
    /// (audit B-03 class). Once the cumulative paused time in an episode
    /// exceeds this budget, the FSM stops pausing regardless of the streak
    /// pattern and emits `DwellPauseExceeded`. Default = 2× the default
    /// spoofed dwell: legitimate links rarely lose more than that mid-episode,
    /// while an attacker gets a hard upper bound on stalling. Resets when a
    /// Suspicious episode genuinely ends (commit to Normal / fresh entry /
    /// operator reset) — NOT on a vetoed re-anchor (same episode).
    pub max_dwell_pause_s: f32,
    /// Velocity-aiding lane — catches the SMART consistent-velocity walk-off
    /// (RQ-170-style slow spoof) the position + post-blend velocity lanes miss.
    /// A CUSUM on the FREE-INERTIAL velocity-residual magnitude (`mag_vel_free` =
    /// |v_gps − v_free_inertial|): a spoofer that fakes Doppler so GPS position
    /// and velocity stay mutually consistent drives `dpos`/`dvel` to ~0 via the
    /// blend, but the velocity it imported persists in `mag_vel_free` ≈ the spoof
    /// rate. `k` (m/s) is the reference value (effective k floors on a learned
    /// noise estimate, like the drift lane, so honest IMU-velocity-bias jitter +
    /// GPS Doppler noise don't accumulate); `h` (m/s·fix) is the firing
    /// threshold. KNOWN BOUND: the free-inertial reference drifts with IMU bias,
    /// so a sufficiently slow ramp-on can be tracked by the adaptive floor —
    /// see docs/threats.md.
    pub vel_aiding_cusum_k_mps: f32,
    pub vel_aiding_cusum_h: f32,
    /// Constellation-quality discontinuity lane (see [`quality`]). Watches the
    /// satellite count and HDOP for a STEP away from the sky we have been
    /// flying under — the signature of a receiver being captured by a
    /// simulated constellation. Orthogonal to every other lane: it fires at the
    /// takeover, where the residual is still ~0 and nothing else can see
    /// anything. Evadable by a spoofer that forges metadata continuity, so it
    /// escalates to Suspicious and is deliberately built so it can never
    /// sustain a firing and sever GPS by itself.
    pub quality: quality::QualityConfig,
}

impl Default for DetectConfig {
    fn default() -> Self {
        Self {
            max_jump_m: 50.0,
            max_velocity_mismatch_mps: 15.0,
            jump_persist_fixes: 2,
            cusum_k_m: 1.0,
            cusum_h_m: 25.0,
            mag_cusum_k_m: 1.0,
            mag_cusum_h_m: 30.0,
            suspicious_to_spoofed_dwell_s: 10.0,
            normal_clear_dwell_s: 5.0,
            hdop_multipath_threshold: 4.0,
            sats_low_threshold: 6,
            // 1Hz GPS normally produces 1.0s gaps with ~100ms jitter — set
            // the threshold safely above that so honest 1Hz GPS never tips it.
            // Operators on 5–10 Hz GPS should override down to 0.5s.
            gps_dropout_freeze_s: 2.5,
            // 500m default = roughly (15 m/s max ground speed) × (suspicious dwell
            // + dropout pause + observation latency). Tight enough to catch
            // teleport-after-jam, loose enough to absorb legitimate DR drift.
            re_anchor_max_distance_m: 500.0,
            // 30 m/s ≈ 2× the fastest real multirotor descent; honest flight
            // never reaches it, but a vertical teleport spoof does.
            max_vertical_rate_mps: 30.0,
            // 2× the default suspicious_to_spoofed_dwell_s (10 s). Operators
            // who raise the dwell should raise this proportionally.
            max_dwell_pause_s: 20.0,
            // Velocity-aiding lane. Base k = 0.55 m/s sits just above the honest
            // free-inertial velocity-residual ceiling measured on a 600 s
            // realistic-noise+bias clean flight (mean 0.125, max 0.454 m/s, never
            // ≥0.5); a consistent-velocity walk-off's masked bias is ~its drift
            // rate (≈0.8–1.1 m/s for a 1 m/s spoof) — a clean separation. h = 8.0
            // (m/s·fix) fires on that ~1 m/s bias in ~16–25 fixes, slow enough to
            // ignore transients. A CUSUM needs SUSTAINED exceedance, so isolated
            // honest spikes near the floor never reach it. Tuned against the
            // realistic-noise + consistent-drift sims.
            vel_aiding_cusum_k_mps: 0.55,
            vel_aiding_cusum_h: 8.0,
            quality: quality::QualityConfig::default(),
        }
    }
}
