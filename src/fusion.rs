//! Fusion task: consumes the GPS+IMU channels, runs dead-reckoning and the
//! detector, emits `SpoofingEvent`s on a broadcast channel, and calls the
//! controller's sever_gps / engage_rtb on the transition into SPOOFED.

use crate::action::FlightController;
use crate::detect::residual::{Residual, ResidualBuffer};
use crate::detect::state_machine::{Event, StateMachine};
use crate::detect::DetectConfig;
use crate::nav::{DeadReckoner, NavConfig};
use crate::types::{
    GpsFix, ImuSample, NavStateKind, NedPos, NedVel, SpoofKind, SpoofingEvent, Timestamp,
};
use std::sync::Arc;
use tokio::sync::{broadcast, mpsc};
use tokio::time::Instant;
use tracing::{debug, warn};

#[derive(Debug, Clone, Copy)]
pub struct FusionConfig {
    pub nav: NavConfig,
    pub detect: DetectConfig,
    pub residual_window_s: f64,
    pub max_imu_rate_hz: f64,
}

impl Default for FusionConfig {
    fn default() -> Self {
        Self {
            nav: NavConfig::default(),
            detect: DetectConfig::default(),
            residual_window_s: 2.0,
            max_imu_rate_hz: 100.0,
        }
    }
}

impl FusionConfig {
    /// Reject degenerate parameter combinations that would silently break the
    /// algorithm: zero thresholds, inverted CUSUM bounds, zero dwell, etc.
    /// Called at `spawn_all` so misconfigurations fail at startup, not in
    /// flight.
    pub fn validate(&self) -> Result<(), String> {
        if !(self.residual_window_s > 0.0 && self.residual_window_s.is_finite()) {
            return Err(format!(
                "residual_window_s must be > 0 (got {})",
                self.residual_window_s
            ));
        }
        if !(self.max_imu_rate_hz > 0.0 && self.max_imu_rate_hz.is_finite()) {
            return Err(format!(
                "max_imu_rate_hz must be > 0 (got {})",
                self.max_imu_rate_hz
            ));
        }
        let d = &self.detect;
        if !(d.max_jump_m > 0.0 && d.max_jump_m.is_finite()) {
            return Err(format!(
                "detect.max_jump_m must be > 0 (got {})",
                d.max_jump_m
            ));
        }
        if !(d.max_velocity_mismatch_mps > 0.0 && d.max_velocity_mismatch_mps.is_finite()) {
            return Err(format!(
                "detect.max_velocity_mismatch_mps must be > 0 (got {})",
                d.max_velocity_mismatch_mps
            ));
        }
        if d.jump_persist_fixes == 0 {
            return Err("detect.jump_persist_fixes must be >= 1".into());
        }
        if !(d.cusum_k_m > 0.0 && d.cusum_h_m > 0.0) {
            return Err(format!(
                "detect.cusum_k_m and cusum_h_m must both be > 0 (got k={}, h={})",
                d.cusum_k_m, d.cusum_h_m
            ));
        }
        if d.cusum_k_m >= d.cusum_h_m {
            return Err(format!(
                "detect.cusum_k_m ({}) must be < cusum_h_m ({}); otherwise drift detector fires \
                 on every sample",
                d.cusum_k_m, d.cusum_h_m
            ));
        }
        if !(d.mag_cusum_k_m > 0.0 && d.mag_cusum_h_m > 0.0) {
            return Err(format!(
                "detect.mag_cusum_k_m and mag_cusum_h_m must both be > 0 (got k={}, h={})",
                d.mag_cusum_k_m, d.mag_cusum_h_m
            ));
        }
        if d.mag_cusum_k_m >= d.mag_cusum_h_m {
            return Err(format!(
                "detect.mag_cusum_k_m ({}) must be < mag_cusum_h_m ({})",
                d.mag_cusum_k_m, d.mag_cusum_h_m
            ));
        }
        if !(d.suspicious_to_spoofed_dwell_s > 0.0 && d.normal_clear_dwell_s > 0.0) {
            return Err(format!(
                "detect dwell times must be > 0 (got susp->spoofed={}, normal_clear={})",
                d.suspicious_to_spoofed_dwell_s, d.normal_clear_dwell_s
            ));
        }
        if !(d.re_anchor_max_distance_m > 0.0 && d.re_anchor_max_distance_m.is_finite()) {
            return Err(format!(
                "detect.re_anchor_max_distance_m must be > 0 (got {})",
                d.re_anchor_max_distance_m
            ));
        }
        if !(d.max_vertical_rate_mps > 0.0 && d.max_vertical_rate_mps.is_finite()) {
            return Err(format!(
                "detect.max_vertical_rate_mps must be > 0 (got {})",
                d.max_vertical_rate_mps
            ));
        }
        if !(d.max_dwell_pause_s > 0.0 && d.max_dwell_pause_s.is_finite()) {
            return Err(format!(
                "detect.max_dwell_pause_s must be > 0 and finite (got {}); an unbounded \
                 dwell-pause budget lets an attacker stall Spoofed escalation forever by \
                 throttling GPS",
                d.max_dwell_pause_s
            ));
        }
        let n = &self.nav;
        if !(n.madgwick_beta > 0.0 && n.madgwick_beta.is_finite()) {
            return Err(format!(
                "nav.madgwick_beta must be > 0 (got {})",
                n.madgwick_beta
            ));
        }
        if !(n.blend_tau_s > 0.0 && n.blend_tau_s.is_finite()) {
            return Err(format!(
                "nav.blend_tau_s must be > 0 (got {})",
                n.blend_tau_s
            ));
        }
        if !(n.bias_ema_alpha > 0.0 && n.bias_ema_alpha <= 1.0) {
            return Err(format!(
                "nav.bias_ema_alpha must be in (0, 1] (got {})",
                n.bias_ema_alpha
            ));
        }
        if !(n.static_init_s > 0.0 && n.static_init_s.is_finite()) {
            return Err(format!(
                "nav.static_init_s must be > 0 (got {})",
                n.static_init_s
            ));
        }
        Ok(())
    }
}

/// Pre-flight readiness state. Orthogonal to `NavStateKind` deliberately:
/// keeping it out of the serialized event enum minimizes blast radius and
/// avoids forcing every event consumer to handle a new state. Until this
/// reaches `Ready`, `Fusion` records sensor input but does NOT evaluate the
/// detector — operator hasn't been told the system is armed yet, so a
/// false positive (or missed detection) here is more harmful than a quiet
/// startup window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PreflightState {
    Initializing,
    Ready,
}

/// What the pre-flight self-test is waiting on. Reported in
/// `PreflightFailed` events so operators can see WHY the system isn't
/// armed yet.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
pub struct PreflightChecklist {
    pub static_init_done: bool,
    pub first_fix_accepted: bool,
    /// Only required when mav source/controller is in use.
    pub heartbeat_seen: bool,
    /// Only required when --vehicle was set AND a HEARTBEAT has been seen.
    /// True if the autopilot's MAV_AUTOPILOT byte matches the operator's
    /// declared vehicle profile.
    pub autopilot_matches: bool,
}

pub struct Fusion {
    nav: DeadReckoner,
    fsm: StateMachine,
    buffer: ResidualBuffer,
    boot: Instant,
    /// Monotonic-seconds-since-boot of the last GPS fix we processed
    /// (regardless of whether it produced a residual). Used to detect
    /// inter-fix gaps and freeze the SUSPICIOUS dwell during GPS dropout
    /// — defends against the "jam GPS, wait for our timer to expire,
    /// then spoof" attack pattern.
    last_gps_t_secs: Option<f64>,
    /// Cached config copy for the dropout threshold.
    gps_dropout_freeze_s: f32,
    /// Cached config copy for the re-anchor plausibility threshold.
    re_anchor_max_distance_m: f32,
    /// Streak count of consecutive fixes that arrived without GPS velocity.
    /// When persistent (>=10), we emit a SyncWarning so the operator can
    /// see that velocity-mismatch detection is silently inactive.
    no_doppler_streak: u16,
    /// Frozen-fix detector state: the ANCHOR fix of the current frozen
    /// streak (lat/lon, degrees). Fixes are compared against this anchor —
    /// NOT the immediately-previous fix — so slow honest flight on a
    /// high-rate receiver (where consecutive fixes are millimeters apart but
    /// the vehicle steadily walks away from any fixed point) cannot build a
    /// streak. Catches stuck GPS modules AND replay attacks: the attacker
    /// re-injects an old GPS_RAW_INT with fresh timestamp, keeping lat/lon
    /// pinned while the vehicle physically moves.
    frozen_anchor_lla: Option<(f64, f64)>,
    frozen_fix_streak: u8,
    /// Cumulative IMU-implied displacement (m) accrued while GPS has stayed
    /// within `FROZEN_FIX_RADIUS_M` of the anchor. This — not a fix count —
    /// is the firing evidence, which makes the detector GPS-rate-independent:
    /// "GPS says we moved <0.5 m total while inertial says we moved ≥5 m"
    /// is the same strength of evidence at 1 Hz or 10 Hz.
    frozen_imu_displacement_m: f64,
    /// Previous GPS altitude (m) + its mono-seconds timestamp, for the
    /// vertical-rate sanity check (audit B-42). `None` until the first fix.
    last_gps_alt: Option<(f64, f64)>,
    /// Cached config copy for the vertical-rate threshold.
    max_vertical_rate_mps: f32,
    /// Pre-flight readiness — see `PreflightState` doc.
    preflight: PreflightState,
    checklist: PreflightChecklist,
    /// Optional HEARTBEAT-watcher: when set, preflight waits for this monitor
    /// to report a fresh heartbeat AND an autopilot byte that matches
    /// `expected_autopilot`. Constructed by callers that use MAV sources.
    mav_monitor: Option<std::sync::Arc<crate::mav::monitor::MavMonitor>>,
    /// Operator-declared autopilot byte (`expected_autopilot_byte(profile)`).
    /// `None` = no MAV in use OR vehicle profile not declared.
    expected_autopilot: Option<u8>,
    /// Last monotonic-seconds at which we emitted a `PreflightFailed` event.
    /// Rate-limited to one every 5 seconds — operators don't need a storm of
    /// "still waiting" notices.
    last_preflight_failed_emit_s: f64,
    /// Optional forensic ring buffer. `None` = forensic disabled (no
    /// --forensic-dir). When `Some`, every on_gps / on_imu pushes into it,
    /// and the first Spoofed transition triggers a dump to disk.
    forensic: Option<crate::forensic::ForensicBuffer>,
    /// Directory for forensic dumps. Cloned into spawn_blocking when the
    /// write fires.
    forensic_dir: Option<std::path::PathBuf>,
    /// Registry of in-flight background tasks (forensic dump + RTL verify).
    /// Audit E-10 fix: the main shutdown path drains this with a bounded
    /// timeout so forensic data isn't lost on `systemctl stop`. `None`
    /// means tasks are still spawned bare via tokio::spawn (back-compat for
    /// callers that don't care about drain semantics — most tests).
    pending_bg: Option<crate::runtime::PendingBgTasks>,
}

/// How close a fix must stay to the frozen-streak ANCHOR fix (horizontal
/// meters) to count as "frozen." 0.5m is well below GPS noise floor (~2.5m 1σ).
const FROZEN_FIX_RADIUS_M: f64 = 0.5;
/// Cumulative IMU-implied displacement (meters) while GPS stays pinned to the
/// anchor before `FrozenGps` fires. Displacement — not a fix count — keeps the
/// detector GPS-rate-independent: the old "3 consecutive fixes within 0.5 m"
/// rule was really a per-fix-displacement threshold tuned for the 1 Hz sim,
/// and false-fired within ~0.5 s on a real 5–10 Hz receiver during slow
/// flight (1–2.5 m/s moves <0.5 m per fix while the IMU shows >1 m/s).
const FROZEN_MIN_IMU_DISPLACEMENT_M: f64 = 5.0;
/// IMU-derived velocity above this magnitude counts as "moving" for
/// frozen-fix detection (gates the displacement accumulation). Below this,
/// the vehicle may legitimately be stationary — identical fixes are normal,
/// and DR velocity noise on a parked vehicle must not slowly accumulate
/// phantom displacement.
const FROZEN_FIX_MIN_IMU_VEL_MPS: f64 = 1.0;

impl Fusion {
    pub fn new(cfg: FusionConfig, boot: Instant) -> Self {
        let nav = DeadReckoner::new(cfg.nav, boot);
        let fsm = StateMachine::new(cfg.detect);
        let buffer = ResidualBuffer::new(cfg.residual_window_s, cfg.max_imu_rate_hz);
        let mav_required = false;
        Self {
            nav,
            fsm,
            buffer,
            boot,
            last_gps_t_secs: None,
            gps_dropout_freeze_s: cfg.detect.gps_dropout_freeze_s,
            re_anchor_max_distance_m: cfg.detect.re_anchor_max_distance_m,
            forensic: None,
            forensic_dir: None,
            no_doppler_streak: 0,
            frozen_anchor_lla: None,
            frozen_fix_streak: 0,
            frozen_imu_displacement_m: 0.0,
            last_gps_alt: None,
            max_vertical_rate_mps: cfg.detect.max_vertical_rate_mps,
            preflight: PreflightState::Initializing,
            checklist: PreflightChecklist {
                static_init_done: false,
                first_fix_accepted: false,
                // Pre-mark "satisfied" when we don't expect mav at all — the
                // mav monitor wire-up below toggles this back to false when
                // applicable.
                heartbeat_seen: !mav_required,
                autopilot_matches: !mav_required,
            },
            mav_monitor: None,
            expected_autopilot: None,
            last_preflight_failed_emit_s: f64::NEG_INFINITY,
            pending_bg: None,
        }
    }

    /// Wire a shared `PendingBgTasks` registry so the main shutdown path can
    /// drain in-flight forensic dumps and RTL verifies before exit. Without
    /// this, the spawned tasks are bare `tokio::spawn`s and get aborted when
    /// the runtime drops — audit E-10.
    pub fn with_pending_bg(mut self, pending: crate::runtime::PendingBgTasks) -> Self {
        self.pending_bg = Some(pending);
        self
    }

    /// Wire a MAV monitor + expected autopilot byte into preflight. Without
    /// this, preflight treats MAV-related checks as "not applicable" and
    /// passes them by default. Caller (the binary's main) must call this
    /// whenever it uses MAV sources or the MAV controller.
    pub fn with_mav_monitor(
        mut self,
        monitor: std::sync::Arc<crate::mav::monitor::MavMonitor>,
        expected_autopilot: u8,
    ) -> Self {
        self.mav_monitor = Some(monitor);
        self.expected_autopilot = Some(expected_autopilot);
        self.checklist.heartbeat_seen = false;
        self.checklist.autopilot_matches = false;
        self
    }

    pub fn preflight_state(&self) -> PreflightState {
        self.preflight
    }

    pub fn checklist(&self) -> PreflightChecklist {
        self.checklist
    }

    /// Wire up the forensic ring buffer. `dir` is where dumps land; `window_s`
    /// is how much history to retain. Without this call, forensic capture is
    /// disabled and Spoofed transitions emit no dump.
    pub fn with_forensic(
        mut self,
        dir: std::path::PathBuf,
        window_s: f64,
        max_imu_rate_hz: f64,
    ) -> Self {
        self.forensic = Some(crate::forensic::ForensicBuffer::new(
            window_s,
            max_imu_rate_hz,
        ));
        self.forensic_dir = Some(dir);
        self
    }

    /// Run the fusion loop until both upstream channels close. Emits events
    /// on `events_tx`; calls controller actions on transition into SPOOFED.
    /// `reset_rx` allows an operator to clear a latched SPOOFED state
    /// (e.g. via a signal handler in main, or a future MAVLink command).
    pub async fn run(
        mut self,
        mut gps_rx: mpsc::Receiver<GpsFix>,
        mut imu_rx: mpsc::Receiver<ImuSample>,
        mut reset_rx: mpsc::Receiver<()>,
        events_tx: broadcast::Sender<SpoofingEvent>,
        controller: Arc<dyn FlightController>,
    ) {
        let mut imu_open = true;
        let mut gps_open = true;

        while imu_open || gps_open {
            tokio::select! {
                biased; // service IMU first so the buffer is always fresh when a GPS fix arrives
                maybe_imu = imu_rx.recv(), if imu_open => {
                    match maybe_imu {
                        Some(sample) => self.on_imu(sample),
                        None => imu_open = false,
                    }
                }
                maybe_gps = gps_rx.recv(), if gps_open => {
                    match maybe_gps {
                        Some(fix) => self.on_gps(fix, &events_tx, &controller).await,
                        None => gps_open = false,
                    }
                }
                _ = reset_rx.recv() => {
                    warn!("OPERATOR RESET: clearing SPOOFED state and resuming detection");
                    let prev = self.fsm.state();
                    self.fsm.manual_reset_to_normal();
                    self.nav.set_state(crate::types::NavStateKind::Normal);
                    // AUDIT FUS-01 fix: clear all Fusion-level ephemeral state
                    // that survives an FSM-only reset. Previously these fields
                    // persisted across operator reset, causing spurious events
                    // on the first post-reset fix:
                    //   * `last_gps_t_secs` → first fix flagged as dropout-recovery
                    //   * `frozen_anchor_lla` / `frozen_fix_streak` /
                    //     `frozen_imu_displacement_m` → first fix compared as
                    //     frozen against the pre-reset anchor, could fire
                    //     FrozenGps spuriously
                    //   * `no_doppler_streak` → could tip the SyncWarning emission
                    //     on the next no-Doppler fix
                    // Note: forensic.dump_fired flag is INTENTIONALLY preserved —
                    // "once per process" is documented behavior; restart re-arms.
                    // ResidualBuffer is also preserved — IMU snapshots remain valid.
                    self.last_gps_t_secs = None;
                    self.frozen_anchor_lla = None;
                    self.frozen_fix_streak = 0;
                    self.frozen_imu_displacement_m = 0.0;
                    self.no_doppler_streak = 0;
                    self.last_gps_alt = None;
                    // Clear controller-side action latches too. Without this,
                    // the next Spoofed transition produces AlreadyEngaged
                    // errors and silently fails to re-engage RTL — composite
                    // attack: local SIGHUP after first action = silent disarm.
                    if let Err(e) = controller.reset().await {
                        warn!(error = %e, "controller reset failed");
                    }
                    let ev = SpoofingEvent::new(
                        Timestamp::now_mono(),
                        SpoofKind::StateTransition,
                        crate::types::NavStateKind::Normal,
                        0.0,
                        serde_json::json!({
                            "from": format!("{:?}", prev),
                            "to": "Normal",
                            "reason": "operator_reset",
                        }),
                        self.boot,
                    );
                    let _ = events_tx.send(ev);
                }
            }
        }
        debug!("fusion: both upstream channels closed; exiting");
    }

    fn on_imu(&mut self, sample: ImuSample) {
        // Push to forensic buffer FIRST (before integration may panic etc.).
        // mono_secs from the sample's timestamp — same time base as fusion.
        if let Some(f) = self.forensic.as_mut() {
            let t_secs = sample
                .t
                .mono
                .saturating_duration_since(self.boot)
                .as_secs_f64();
            f.push_imu(t_secs, &sample);
        }
        match self.nav.step(&sample) {
            Ok(Some(snap)) => self.buffer.push(snap),
            Ok(None) => {} // still initializing or no anchor
            Err(e) => warn!(error = %e, "nav step error"),
        }
        // Preflight progresses on each IMU sample if static-init has finished.
        // Side effect of nav.step is updating its internal `initialized` flag.
        if !self.checklist.static_init_done && self.nav.is_initialized() {
            self.checklist.static_init_done = true;
        }
    }

    fn fsm_cfg_re_anchor_max(&self) -> f32 {
        self.re_anchor_max_distance_m
    }

    /// On a Spoofed transition, capture the forensic ring buffer to disk if
    /// forensic mode is configured AND we haven't already dumped this
    /// process. Spawned async — fusion's main loop must keep moving so the
    /// detector continues evaluating (a real attack may include follow-up
    /// spoof patterns that the forensic dump shouldn't block detection of).
    fn maybe_trigger_forensic_dump(
        &self,
        from: NavStateKind,
        r: &crate::detect::residual::Residual,
        ev: &crate::detect::state_machine::Event,
        events_tx: &broadcast::Sender<SpoofingEvent>,
    ) {
        let (Some(forensic), Some(dir)) = (self.forensic.as_ref(), self.forensic_dir.as_ref())
        else {
            return; // forensic not configured
        };
        let timestamp = Timestamp::now_mono();
        if forensic.already_dumped() {
            let dup_ev = SpoofingEvent::new(
                timestamp,
                SpoofKind::ForensicDumpSuppressed,
                NavStateKind::Spoofed,
                r.mag_pos as f32,
                serde_json::json!({
                    "reason": "forensic dump already written for this process; restart to re-arm",
                }),
                self.boot,
            );
            let _ = events_tx.send(dup_ev);
            return;
        }
        // Build the snapshot in the synchronous path (we own &self here).
        // The async write happens off-thread.
        let attestation = crate::forensic::AttestationSummary {
            crate_version: env!("CARGO_PKG_VERSION").to_string(),
            binary_sha256: crate::attestation::Attestation::capture().binary_sha256,
            host_os: std::env::consts::OS.to_string(),
            host_arch: std::env::consts::ARCH.to_string(),
        };
        let trigger = crate::forensic::ForensicTrigger::SpoofedTransition {
            from,
            to: NavStateKind::Spoofed,
            residual_m: r.mag_pos as f32,
            event_detail: serde_json::json!({"event": format!("{:?}", ev)}),
        };
        let snap = forensic.snapshot(
            timestamp
                .mono
                .saturating_duration_since(self.boot)
                .as_nanos(),
            trigger,
            attestation,
        );
        let dir = dir.clone();
        let tx = events_tx.clone();
        let boot = self.boot;
        let fired_flag = forensic.dump_fired_flag();
        let task = async move {
            match crate::forensic::dump_to_disk(&snap, &dir).await {
                Ok(path) => {
                    // Mark fired ONLY on success — a transient write failure
                    // shouldn't suppress retry on the next Spoofed transition.
                    fired_flag.store(true, std::sync::atomic::Ordering::Release);
                    let ev = SpoofingEvent::new(
                        Timestamp::now_mono(),
                        SpoofKind::ForensicDumpWritten,
                        NavStateKind::Spoofed,
                        0.0,
                        serde_json::json!({
                            "path": path.display().to_string(),
                            "bytes_estimated": serde_json::to_vec(&snap)
                                .map(|v| v.len())
                                .unwrap_or(0),
                        }),
                        boot,
                    );
                    let _ = tx.send(ev);
                    tracing::warn!(path = %path.display(), "FORENSIC DUMP written");
                }
                Err(e) => {
                    let ev = SpoofingEvent::new(
                        Timestamp::now_mono(),
                        SpoofKind::ForensicDumpFailed,
                        NavStateKind::Spoofed,
                        0.0,
                        serde_json::json!({"error": format!("{}", e)}),
                        boot,
                    );
                    let _ = tx.send(ev);
                    tracing::error!(error = %e, "FORENSIC DUMP failed — Spoofed event still fired but post-mortem data is missing");
                }
            }
        };
        // Route into the shared JoinSet when available so shutdown can drain
        // pending writes. Without it, fall back to a bare spawn (back-compat).
        self.spawn_bg(task);
    }

    /// Helper: route a future into the shared `pending_bg` JoinSet if
    /// available, else fall back to bare `tokio::spawn`. The fall-back
    /// preserves prior behavior for tests / callers that don't wire
    /// `with_pending_bg`. Audit E-10.
    ///
    /// `JoinSet::spawn` is synchronous (it spawns onto the current tokio
    /// runtime and returns immediately), so we can register under a sync
    /// Mutex without holding the lock across an await. Drain takes
    /// ownership of the JoinSet via `mem::take` so the drain await never
    /// holds the registration lock.
    fn spawn_bg<F>(&self, fut: F)
    where
        F: std::future::Future<Output = ()> + Send + 'static,
    {
        if let Some(pending) = self.pending_bg.as_ref() {
            let mut guard = pending.lock().unwrap_or_else(|e| e.into_inner());
            guard.spawn(fut);
        } else {
            tokio::spawn(fut);
        }
    }

    /// Re-evaluate preflight and emit events on transitions.
    /// Returns `true` if preflight just transitioned to `Ready`.
    fn refresh_preflight(
        &mut self,
        t_secs: f64,
        events_tx: &broadcast::Sender<SpoofingEvent>,
        fix_for_timestamp: Option<&GpsFix>,
    ) -> bool {
        if self.preflight == PreflightState::Ready {
            return false;
        }
        // Pull HEARTBEAT-derived facts from the monitor if connected.
        if let Some(m) = &self.mav_monitor {
            let hb_fresh = m.heartbeat_fresh(
                Instant::now(),
                self.boot,
                std::time::Duration::from_secs(3).as_nanos() as u64,
            );
            self.checklist.heartbeat_seen = hb_fresh;
            self.checklist.autopilot_matches = match self.expected_autopilot {
                Some(want) => {
                    let seen = m.current_autopilot();
                    hb_fresh && seen == want
                }
                None => true,
            };
        }

        let all_ok = self.checklist.static_init_done
            && self.checklist.first_fix_accepted
            && self.checklist.heartbeat_seen
            && self.checklist.autopilot_matches;

        let timestamp = fix_for_timestamp
            .map(|f| f.t)
            .unwrap_or_else(Timestamp::now_mono);

        if all_ok {
            self.preflight = PreflightState::Ready;
            let ev = SpoofingEvent::new(
                timestamp,
                SpoofKind::PreflightPassed,
                self.fsm.state(),
                0.0,
                serde_json::json!({"checklist": self.checklist}),
                self.boot,
            );
            let _ = events_tx.send(ev);
            warn!(checklist = ?self.checklist, "PREFLIGHT: passed — detector arming");
            true
        } else {
            // Rate-limit: at most one PreflightFailed event every 5 seconds.
            if t_secs - self.last_preflight_failed_emit_s >= 5.0 {
                self.last_preflight_failed_emit_s = t_secs;
                let ev = SpoofingEvent::new(
                    timestamp,
                    SpoofKind::PreflightFailed,
                    self.fsm.state(),
                    0.0,
                    serde_json::json!({"checklist": self.checklist}),
                    self.boot,
                );
                let _ = events_tx.send(ev);
            }
            false
        }
    }

    async fn on_gps(
        &mut self,
        fix: GpsFix,
        events_tx: &broadcast::Sender<SpoofingEvent>,
        controller: &Arc<dyn FlightController>,
    ) {
        let gps_ned = match self.nav.gps_in_ned(&fix) {
            Some(p) => p,
            None => {
                // First-ever fix: plausibility-check vs expected home before
                // anchoring. If the fix is implausibly far from where the
                // drone is supposed to be, REFUSE to anchor — meaconing-at-
                // boot defense. Operator sees `BootAnchorRejected` and the
                // detector waits for a sane fix instead of silently anchoring
                // to a spoofed origin.
                if let Err(distance_m) = self.nav.is_first_fix_plausible(&fix) {
                    let ev = SpoofingEvent::new(
                        fix.t,
                        SpoofKind::BootAnchorRejected,
                        self.fsm.state(),
                        distance_m as f32,
                        serde_json::json!({
                            "lat_deg": fix.lat_deg,
                            "lon_deg": fix.lon_deg,
                            "distance_from_expected_home_m": distance_m,
                        }),
                        self.boot,
                    );
                    let _ = events_tx.send(ev);
                    warn!(
                        distance_m,
                        "BOOT: first fix rejected — too far from expected home (meaconing-at-boot suspected)"
                    );
                    return;
                }
                self.nav.on_gps_fix(&fix);
                // First fix accepted — preflight gate makes progress.
                self.checklist.first_fix_accepted = true;
                let t_secs = fix
                    .t
                    .mono
                    .saturating_duration_since(self.boot)
                    .as_secs_f64();
                self.refresh_preflight(t_secs, events_tx, Some(&fix));
                return;
            }
        };

        // Preflight gate: until Ready, record state but DO NOT run the FSM.
        // (We've already done first-fix anchor + IMU buffering — those are
        // necessary preconditions for preflight itself.)
        //
        // During preflight, if the IMU buffer cannot serve a residual lookup
        // (empty, or t_gps outside its window), re-anchor with on_gps_fix.
        // This mirrors the post-Ready SyncWarning empty-buffer recovery path
        // and keeps the IMU and GPS roughly co-located through startup. Once
        // the IMU has caught up enough that interpolate would succeed, we
        // STOP re-anchoring — preserving the residual signal so that a spoof
        // arriving right at preflight Ready is still detected at the first
        // post-Ready fix.
        if self.preflight != PreflightState::Ready {
            let t_secs = fix
                .t
                .mono
                .saturating_duration_since(self.boot)
                .as_secs_f64();
            let _ = self.refresh_preflight(t_secs, events_tx, Some(&fix));
            if self.buffer.is_empty() || self.buffer.interpolate(t_secs).is_err() {
                self.nav.on_gps_fix(&fix);
                self.fsm.on_clean_anchor();
            }
            return;
        }

        // Time-align IMU prediction to GPS timestamp.
        let t_gps_secs = fix
            .t
            .mono
            .saturating_duration_since(self.boot)
            .as_secs_f64();

        // GPS-dropout detection. If the inter-fix gap exceeds the configured
        // threshold, tell the FSM to PAUSE its suspicious->spoofed dwell
        // timer for this fix. Rationale: an attacker can jam GPS for N
        // seconds while we're SUSPICIOUS — without this freeze, the dwell
        // timer keeps ticking (it's wall-clock-based), and SPOOFED fires on
        // the next valid fix even if that fix happens to be honest. Real GPS
        // dropouts during sharp maneuvers / tunnels would also false-fire.
        let dropout_seconds = match self.last_gps_t_secs {
            Some(prev) => (t_gps_secs - prev) as f32,
            None => 0.0,
        };
        self.last_gps_t_secs = Some(t_gps_secs);
        let in_dropout_recovery = dropout_seconds > self.gps_dropout_freeze_s;
        if in_dropout_recovery {
            // Debug-level: this is normal during real flight (sharp turns,
            // brief obstructions). Only a sustained sequence is alarming —
            // operator dashboards can track `dropout_recovery_count`.
            debug!(
                dropout_s = dropout_seconds,
                threshold_s = self.gps_dropout_freeze_s,
                "GPS gap exceeded freeze threshold — pausing FSM dwell for this fix"
            );
            self.fsm.note_dropout_recovery();
        }
        // If the FSM tripped the consecutive-dropout cap on a recent
        // evaluate(), it set a flag we consume here. The flag fires once per
        // streak so it doesn't spam.
        if self.fsm.dwell_pause_exceeded_pending {
            self.fsm.dwell_pause_exceeded_pending = false;
            let ev = SpoofingEvent::new(
                fix.t,
                SpoofKind::DwellPauseExceeded,
                self.fsm.state(),
                0.0,
                serde_json::json!({
                    "consecutive_dropouts": self.fsm.consecutive_dropout_count(),
                    "paused_cumulative_s": self.fsm.paused_cumulative_secs(),
                    "reason": "GPS dropout pause cap exceeded (consecutive streak or \
                               cumulative per-episode budget); dwell timer resuming",
                }),
                self.boot,
            );
            let _ = events_tx.send(ev);
            warn!(
                "GPS dropout streak cap exceeded — possible adversarial throttling; \
                 resuming dwell escalation"
            );
        }

        // Frozen-fix detector: compare lat/lon to the STREAK ANCHOR before any
        // residual math. While fixes stay within FROZEN_FIX_RADIUS_M of the
        // anchor, IMU-implied displacement accrues (below, once v_imu is
        // known); when GPS claims <0.5 m total motion while the IMU says the
        // vehicle covered ≥FROZEN_MIN_IMU_DISPLACEMENT_M, the GPS is stuck or
        // being replayed. Anchor-based comparison (not consecutive-fix) is
        // what makes slow honest flight innocent: the vehicle steadily exits
        // the 0.5 m radius and the streak resets, regardless of GPS rate.
        let frozen_candidate = match self.frozen_anchor_lla {
            Some((anchor_lat, anchor_lon)) => {
                let ned =
                    crate::nav::lla_to_ned(fix.lat_deg, fix.lon_deg, 0.0, anchor_lat, anchor_lon);
                ned.horizontal_norm() < FROZEN_FIX_RADIUS_M
            }
            None => false,
        };
        if !frozen_candidate {
            // Moved away from the anchor (or first fix): restart the streak
            // anchored at the current fix.
            self.frozen_anchor_lla = Some((fix.lat_deg, fix.lon_deg));
            self.frozen_fix_streak = 0;
            self.frozen_imu_displacement_m = 0.0;
        }

        let (p_imu, v_imu) = match self.buffer.interpolate(t_gps_secs) {
            Ok(p) => p,
            Err(e) => {
                let ev = SpoofingEvent::new(
                    fix.t,
                    SpoofKind::SyncWarning,
                    self.fsm.state(),
                    0.0,
                    serde_json::json!({"reason": format!("{}", e)}),
                    self.boot,
                );
                let _ = events_tx.send(ev);
                // Skip residual for this fix.
                self.nav.on_gps_fix(&fix);
                // AUDIT B-20 / B-09 fix: do NOT call `self.fsm.on_clean_anchor()`
                // here. Previously this reset the drift CUSUM whenever
                // interpolation failed post-Ready. An attacker who can stall
                // the IMU stream (compromised MAV peer, hardware fault) can
                // engineer interpolate failures at will → CUSUM resets on
                // every gap → a slow-drift attack never accumulates. The
                // CUSUM reset only makes sense after a genuine re-anchor
                // (which post-Ready never does — anchor is locked), not
                // after a transient buffer miss. The detector should keep
                // its accumulated evidence across these gaps.
                return;
            }
        };

        let dpos = NedPos {
            n: gps_ned.n - p_imu.n,
            e: gps_ned.e - p_imu.e,
            d: gps_ned.d - p_imu.d,
        };
        // GPS velocity is ONLY usable when both speed and course are present.
        // Missing either field => v_gps is unknown — never default to zero.
        let v_gps_opt: Option<NedVel> = fix.speed_mps.zip(fix.course_deg).map(|(s, c)| {
            let cr = c.to_radians();
            NedVel {
                n: s * cr.cos(),
                e: s * cr.sin(),
                d: 0.0,
            }
        });
        let (dvel, mag_vel, dvel_known) = match v_gps_opt {
            Some(v_gps) => {
                let dv = NedVel {
                    n: v_gps.n - v_imu.n,
                    e: v_gps.e - v_imu.e,
                    d: v_gps.d - v_imu.d,
                };
                (dv, dv.horizontal_norm(), true)
            }
            None => (NedVel::default(), 0.0, false),
        };
        let r = Residual {
            dpos,
            dvel,
            mag_pos: dpos.horizontal_norm(),
            mag_vel,
            dvel_known,
        };

        // Forensic capture of this GPS fix + computed residual + current FSM
        // state. Done before detector evaluation so the recorded state
        // matches what the FSM SAW when it decided.
        if let Some(fb) = self.forensic.as_mut() {
            fb.push_gps(t_gps_secs, &fix);
            fb.push_residual(t_gps_secs, &r, self.fsm.state());
        }

        // Frozen-fix decision: accrue IMU-implied displacement over the frozen
        // interval; fire + register an external anomaly with the FSM once the
        // inertial evidence says the vehicle moved ≥FROZEN_MIN_IMU_DISPLACEMENT_M
        // while GPS stayed pinned. (Streak reset for a non-candidate fix
        // already happened above, before the interpolate call.)
        if frozen_candidate {
            self.frozen_fix_streak = self.frozen_fix_streak.saturating_add(1);
            let v_h = v_imu.horizontal_norm();
            if v_h > FROZEN_FIX_MIN_IMU_VEL_MPS {
                // Inter-fix interval, clamped to the dropout threshold so a
                // long GPS outage can't accrue a huge phantom displacement in
                // one step (consistent with the dwell-freeze treatment of
                // dropout gaps).
                let dt = (dropout_seconds as f64).clamp(0.0, self.gps_dropout_freeze_s as f64);
                self.frozen_imu_displacement_m += v_h * dt;
            }
            if self.frozen_imu_displacement_m >= FROZEN_MIN_IMU_DISPLACEMENT_M {
                self.fsm.note_external_anomaly();
                let ev = SpoofingEvent::new(
                    fix.t,
                    SpoofKind::FrozenGps,
                    self.fsm.state(),
                    r.mag_pos as f32,
                    serde_json::json!({
                        "streak": self.frozen_fix_streak,
                        "imu_displacement_m": self.frozen_imu_displacement_m,
                        "v_imu_horizontal_mps": v_h,
                        "lat_deg": fix.lat_deg,
                        "lon_deg": fix.lon_deg,
                        "reason": "GPS pinned within 0.5 m of a fixed point while inertial \
                                   dead-reckoning shows the vehicle moving (stuck module or replay)",
                    }),
                    self.boot,
                );
                let _ = events_tx.send(ev);
                warn!(
                    streak = self.frozen_fix_streak,
                    imu_displacement_m = self.frozen_imu_displacement_m,
                    v_imu = v_h,
                    "FROZEN GPS detected — stuck-module or replay attack"
                );
            }
        }

        // AUDIT B-42: vertical-spoof sanity check. Every other detector uses
        // the HORIZONTAL residual, so a pure altitude spoof (lat/lon honest,
        // alt teleported) was invisible. This is a GPS-only check — it does
        // NOT use the vertical dead-reckoned position (double-integrating
        // accel-Z bias makes vertical DR drift fast and unreliable). Instead
        // it bounds the GPS-reported climb/descent RATE between consecutive
        // fixes. A drone physically can't climb/descend at >30 m/s; an
        // altitude teleport produces a far higher apparent rate. Catches
        // sudden vertical spoofs; gradual altitude drift remains a known gap
        // (GPS altitude is inherently weak — see docs/threats.md §3.1).
        if let Some((prev_alt, prev_t)) = self.last_gps_alt {
            let dt_alt = (t_gps_secs - prev_t) as f32;
            // AUDIT U-02: only assess the instantaneous rate over a NORMAL
            // inter-fix interval. Across a GPS dropout (gap > freeze
            // threshold) the |Δalt|/dt rate is meaningless — a legitimate
            // sustained descent over a 10 s outage would look like a fast
            // "jump" and false-fire, while an attacker could dilute a real
            // teleport below the bound by stretching dt. We skip the check
            // across dropouts (consistent with the horizontal dwell freeze).
            // Residual: a pure-altitude teleport that lands exactly on a
            // dropout boundary is not caught here — documented in
            // docs/threats.md §3.1; it's a narrow attack and GPS altitude is
            // a weak spoof target to begin with.
            let normal_interval = dt_alt > 1e-3 && dt_alt <= self.gps_dropout_freeze_s;
            if normal_interval {
                let rate = ((fix.alt_m - prev_alt).abs() as f32) / dt_alt;
                if rate > self.max_vertical_rate_mps {
                    self.fsm.note_external_anomaly();
                    let ev = SpoofingEvent::new(
                        fix.t,
                        SpoofKind::Jump,
                        self.fsm.state(),
                        r.mag_pos as f32,
                        serde_json::json!({
                            "reason": "VerticalRate",
                            "alt_rate_mps": rate,
                            "max_vertical_rate_mps": self.max_vertical_rate_mps,
                            "alt_m": fix.alt_m,
                            "prev_alt_m": prev_alt,
                            "detail": "GPS altitude changed faster than physically plausible \
                                       — vertical-spoof indicator (horizontal residual unaffected)",
                        }),
                        self.boot,
                    );
                    let _ = events_tx.send(ev);
                    warn!(
                        alt_rate_mps = rate,
                        max = self.max_vertical_rate_mps,
                        "VERTICAL RATE anomaly — possible altitude spoof"
                    );
                }
            }
        }
        self.last_gps_alt = Some((fix.alt_m, t_gps_secs));

        // Observability for an attacker (or broken GPS) hiding behind missing
        // Doppler. Velocity-mismatch detection is skipped while this is true;
        // operator should know.
        if dvel_known {
            self.no_doppler_streak = 0;
        } else {
            self.no_doppler_streak = self.no_doppler_streak.saturating_add(1);
            if self.no_doppler_streak == 10 {
                let ev = SpoofingEvent::new(
                    fix.t,
                    SpoofKind::SyncWarning,
                    self.fsm.state(),
                    r.mag_pos as f32,
                    serde_json::json!({
                        "reason": "GPS missing Doppler (speed/course) for 10+ consecutive fixes — \
                                   velocity-mismatch detection inactive; using tightened position-clean gate",
                    }),
                    self.boot,
                );
                let _ = events_tx.send(ev);
                warn!("GPS Doppler missing for 10+ fixes — velocity-mismatch detector inactive");
            }
        }

        // Capture the FSM's `suspicious_since` BEFORE evaluate() so that, if
        // evaluate transitions Susp→Normal and the re-anchor veto fires below,
        // we can restore the ORIGINAL entry timestamp (audit B-26). Without
        // this, a vetoed transition resets the dwell clock and the attacker
        // can cycle veto→reset indefinitely without escalating to Spoofed.
        let pre_eval_suspicious_since = self.fsm.suspicious_since();

        // Hand the residual to the state machine.
        let evs = self.fsm.evaluate(&r, t_gps_secs, fix.hdop, fix.sats);
        for ev in &evs {
            // Susp→Normal transition is validated BEFORE emission. If the
            // post-jam fix is implausibly far from the dead-reckoned position,
            // refuse the transition (force_back_to_suspicious) and emit
            // BootAnchorRejected instead of the normal Transition event.
            // Defends against the wait-out-then-spoof attack: attacker holds
            // GPS dropout/Suspicious-state, then drops a clean-looking fix at
            // attacker-chosen coordinates → without this veto we anchor to
            // their grid and clear the alarm.
            if let Event::Transition {
                from: NavStateKind::Suspicious,
                to: NavStateKind::Normal,
            } = ev
            {
                let dr_pos = self.nav.dr_pos();
                let max_m = self.fsm_cfg_re_anchor_max();
                if let Err(distance) =
                    crate::nav::validate_fix_against_reference(&gps_ned, &dr_pos, max_m as f64)
                {
                    self.fsm
                        .force_back_to_suspicious(t_gps_secs, pre_eval_suspicious_since);
                    let ev_reject = SpoofingEvent::new(
                        fix.t,
                        SpoofKind::BootAnchorRejected,
                        self.fsm.state(),
                        distance as f32,
                        serde_json::json!({
                            "phase": "susp_to_normal_re_anchor",
                            "distance_from_dr_m": distance,
                            "max_m": max_m,
                            "reason": "post-jam fix too far from dead-reckoned position",
                        }),
                        self.boot,
                    );
                    let _ = events_tx.send(ev_reject);
                    warn!(
                        distance_m = distance,
                        max_m, "SUSP→NORMAL re-anchor REJECTED — staying Suspicious"
                    );
                    // Skip emitting the original Transition event (it didn't happen).
                    continue;
                }
            }
            self.emit(*ev, &fix, &r, events_tx);
            if let Event::Transition { from, to } = ev {
                self.nav.set_state(*to);
                match (*from, *to) {
                    (from, NavStateKind::Spoofed) => {
                        // Forensic capture FIRST — before any action latency,
                        // so the dump reflects the moment-of-decision state
                        // not the post-action state. Spawned async so it
                        // doesn't block the fusion loop while writing.
                        self.maybe_trigger_forensic_dump(from, &r, ev, events_tx);
                        // Action fire-and-observe: failures get a structured event so
                        // operators see them. The MavFlightController retries internally.
                        //
                        // AUDIT B-13 fix: sever_gps and engage_rtb are run sequentially.
                        // If sever_gps fails (autopilot's GPS_TYPE param didn't get
                        // zeroed), engage_rtb fires RTL but the autopilot is STILL
                        // consuming the spoofed GPS — it'll fly to the attacker's
                        // "home." We can't refuse RTL (the autopilot is still safer
                        // in some RTL mode than continuing GUIDED with bad GPS), but
                        // we MUST loudly flag this state. A separate CRITICAL event
                        // tells the operator to take manual control before the
                        // autopilot navigates anywhere.
                        let sever_ok = self
                            .try_action_result(controller, events_tx, &fix, "sever_gps")
                            .await;
                        if !sever_ok {
                            let ev = SpoofingEvent::new(
                                fix.t,
                                SpoofKind::ActionFailed,
                                self.fsm.state(),
                                r.mag_pos as f32,
                                serde_json::json!({
                                    "action": "sever_gps",
                                    "severity": "CRITICAL",
                                    "consequence": "engage_rtb will fire next but the autopilot \
                                                    is still using the (spoofed) GPS to navigate. \
                                                    Operator should assume RTL is compromised and \
                                                    take manual control.",
                                }),
                                self.boot,
                            );
                            let _ = events_tx.send(ev);
                            tracing::error!(
                                "CRITICAL: sever_gps FAILED — autopilot is still using spoofed GPS. \
                                 RTL about to fire but cannot be trusted. OPERATOR INTERVENTION REQUIRED."
                            );
                        }
                        self.try_action(controller, events_tx, &fix, "engage_rtb")
                            .await;
                        // Closed-loop SEVER read-back (parallel to RTL verify, so it
                        // never delays the RTL command). The synchronous `sever_ok`
                        // above only confirms a datagram left the host; this confirms
                        // the autopilot ECHOED the GPS-disable param as applied. If
                        // not, the drone may still be navigating on the spoofed GPS
                        // even though RTL fired — the most dangerous silent failure in
                        // the action path, so it gets its own CRITICAL event.
                        {
                            let ctrl_s = controller.clone();
                            let tx_s = events_tx.clone();
                            let t_s = fix.t;
                            let state_s = self.fsm.state();
                            let boot_s = self.boot;
                            self.spawn_bg(async move {
                                let confirmed = match ctrl_s.verify_sever_engaged().await {
                                    Ok(v) => v,
                                    Err(e) => {
                                        warn!(error = %e, "sever read-back verify errored");
                                        false
                                    }
                                };
                                if !confirmed {
                                    let ev = SpoofingEvent::new(
                                        t_s,
                                        SpoofKind::ActionFailed,
                                        state_s,
                                        0.0,
                                        serde_json::json!({
                                            "action": "sever_gps",
                                            "phase": "readback",
                                            "severity": "CRITICAL",
                                            "consequence": "autopilot did NOT confirm the GPS-disable \
                                                parameter (no causal PARAM_VALUE=0 echo). The drone may \
                                                still be navigating on the spoofed GPS while in RTL. \
                                                Take manual control.",
                                        }),
                                        boot_s,
                                    );
                                    let _ = tx_s.send(ev);
                                    tracing::error!(
                                        "CRITICAL: sever_gps read-back UNCONFIRMED — autopilot may \
                                         still be using spoofed GPS despite RTL. OPERATOR INTERVENTION."
                                    );
                                }
                            });
                        }
                        // Closed-loop verification: did the autopilot actually engage RTL?
                        // Routed through `spawn_bg` so the shutdown drain can wait
                        // for the verify result before exit (audit E-10).
                        let ctrl = controller.clone();
                        let tx = events_tx.clone();
                        let t = fix.t;
                        let state = self.fsm.state();
                        let boot = self.boot;
                        self.spawn_bg(async move {
                            match ctrl.verify_rtb_engaged().await {
                                Ok(true) => {
                                    let ev = SpoofingEvent::new(
                                        t,
                                        SpoofKind::ActionAcked,
                                        state,
                                        0.0,
                                        serde_json::json!({"action": "engage_rtb", "confirmed": true}),
                                        boot,
                                    );
                                    let _ = tx.send(ev);
                                }
                                Ok(false) => {
                                    let ev = SpoofingEvent::new(
                                        t,
                                        SpoofKind::ActionUnconfirmed,
                                        state,
                                        0.0,
                                        serde_json::json!({
                                            "action": "engage_rtb",
                                            "reason": "autopilot mode did not transition to RTL within verification window",
                                        }),
                                        boot,
                                    );
                                    let _ = tx.send(ev);
                                }
                                Err(e) => {
                                    let ev = SpoofingEvent::new(
                                        t,
                                        SpoofKind::ActionUnconfirmed,
                                        state,
                                        0.0,
                                        serde_json::json!({
                                            "action": "engage_rtb",
                                            "verify_error": format!("{}", e),
                                        }),
                                        boot,
                                    );
                                    let _ = tx.send(ev);
                                }
                            }
                        });
                    }
                    (NavStateKind::Suspicious, NavStateKind::Normal) => {
                        // Re-anchor with this clean fix so accumulated dead-reckoned
                        // drift doesn't immediately re-trigger SUSPICIOUS on the next
                        // fix (otherwise: ping-pong false positives).
                        self.nav.re_anchor_after_clearing(&fix);
                        // CRITICAL: the buffer still holds snapshots stamped relative
                        // to the OLD origin. Without clearing, the next GPS fix's
                        // interpolate() would bracket pre-anchor and post-anchor
                        // snapshots, producing a garbage residual that could flip the
                        // FSM right back into Suspicious or trip the hard-residual
                        // jump detector. Drop them.
                        self.buffer.clear();
                        // Commit: reset detector state now that the transition has
                        // BOTH passed the plausibility veto (above) AND been accepted
                        // by fusion (re-anchor + buffer clear done). The FSM no
                        // longer resets detectors in evaluate() — we own that here.
                        self.fsm.commit_susp_to_normal();
                    }
                    _ => {}
                }
            }
        }

        // Forward the fix to nav for the (velocity-only) blend. Position is
        // anchored once at first fix — cumulative drift signal is preserved by
        // NEVER re-anchoring during NORMAL. The state machine resets CUSUM on
        // its own SUSPICIOUS -> NORMAL transition; we don't reset every fix.
        if self.fsm.state() == NavStateKind::Normal {
            self.nav.on_gps_fix(&fix);
        }
    }

    /// Invoke one action method by name; on failure, emit a SyncWarning-class
    /// event carrying the error so it shows up in the broadcast + JSON log
    /// instead of being silently swallowed.
    async fn try_action(
        &self,
        controller: &Arc<dyn FlightController>,
        events_tx: &broadcast::Sender<SpoofingEvent>,
        fix: &GpsFix,
        which: &'static str,
    ) {
        let _ = self
            .try_action_result(controller, events_tx, fix, which)
            .await;
    }

    /// Same as `try_action` but returns `true` on success and `false` on
    /// failure. Used by the Spoofed-transition handler so it can detect a
    /// failed `sever_gps` and emit a CRITICAL warning BEFORE firing RTL
    /// (audit B-13: otherwise the autopilot RTLs on spoofed GPS without
    /// the operator knowing).
    async fn try_action_result(
        &self,
        controller: &Arc<dyn FlightController>,
        events_tx: &broadcast::Sender<SpoofingEvent>,
        fix: &GpsFix,
        which: &'static str,
    ) -> bool {
        let result = match which {
            "sever_gps" => controller.sever_gps().await,
            "engage_rtb" => controller.engage_rtb().await,
            // AUDIT B-31 fix: previously silent on unknown action names. Now
            // logs at warn! and returns false so callers can react.
            _ => {
                warn!(action = which, "try_action: unknown action name");
                return false;
            }
        };
        match result {
            Ok(()) => true,
            Err(e) => {
                warn!(action = which, error = %e, "FlightController action failed");
                let ev = SpoofingEvent::new(
                    fix.t,
                    SpoofKind::ActionFailed,
                    self.fsm.state(),
                    0.0,
                    serde_json::json!({
                        "action": which,
                        "error": format!("{}", e),
                    }),
                    self.boot,
                );
                let _ = events_tx.send(ev);
                false
            }
        }
    }

    fn emit(&self, ev: Event, fix: &GpsFix, r: &Residual, tx: &broadcast::Sender<SpoofingEvent>) {
        let (kind, detail) = match ev {
            Event::Jump(j) => (
                SpoofKind::Jump,
                serde_json::json!({
                    "reason": format!("{:?}", j),
                    "dpos_n_m": r.dpos.n,
                    "dpos_e_m": r.dpos.e,
                    "dvel_n_mps": r.dvel.n,
                    "dvel_e_mps": r.dvel.e,
                }),
            ),
            Event::Drift(d) => (
                SpoofKind::Drift,
                serde_json::json!({
                    "reason": format!("{:?}", d),
                    "dpos_n_m": r.dpos.n,
                    "dpos_e_m": r.dpos.e,
                    "cusum": cusum_to_json(self.fsm.drift_state()),
                }),
            ),
            Event::Transition { from, to } => (
                SpoofKind::StateTransition,
                serde_json::json!({
                    "from": format!("{:?}", from),
                    "to": format!("{:?}", to),
                }),
            ),
        };
        let spoof_event = SpoofingEvent::new(
            self.timestamp_from_gps(fix),
            kind,
            self.fsm.state(),
            r.mag_pos as f32,
            detail,
            self.boot,
        );
        let _ = tx.send(spoof_event);
    }

    fn timestamp_from_gps(&self, fix: &GpsFix) -> Timestamp {
        Timestamp {
            mono: fix.t.mono,
            utc: fix.t.utc,
        }
    }
}

fn cusum_to_json(s: crate::detect::drift::CusumState) -> serde_json::Value {
    serde_json::json!({
        "s_pos_n": s.s_pos_n,
        "s_neg_n": s.s_neg_n,
        "s_pos_e": s.s_pos_e,
        "s_neg_e": s.s_neg_e,
    })
}

#[cfg(test)]
mod validation_tests {
    use super::*;

    #[test]
    fn default_validates_ok() {
        FusionConfig::default()
            .validate()
            .expect("defaults must validate");
    }

    #[test]
    fn rejects_zero_residual_window() {
        let c = FusionConfig {
            residual_window_s: 0.0,
            ..FusionConfig::default()
        };
        assert!(c.validate().is_err());
    }

    #[test]
    fn rejects_inverted_cusum_bounds() {
        let mut c = FusionConfig::default();
        // h <= k: drift detector would fire on every sample.
        c.detect = DetectConfig {
            cusum_k_m: 10.0,
            cusum_h_m: 5.0,
            ..c.detect
        };
        assert!(c.validate().is_err());
    }

    #[test]
    fn rejects_zero_jump_persist() {
        let mut c = FusionConfig::default();
        c.detect = DetectConfig {
            jump_persist_fixes: 0,
            ..c.detect
        };
        assert!(c.validate().is_err());
    }

    #[test]
    fn rejects_nan_alpha() {
        let mut c = FusionConfig::default();
        c.nav = NavConfig {
            bias_ema_alpha: f32::NAN,
            ..c.nav
        };
        assert!(c.validate().is_err());
    }

    #[test]
    fn rejects_zero_dwell() {
        let mut c = FusionConfig::default();
        c.detect = DetectConfig {
            suspicious_to_spoofed_dwell_s: 0.0,
            ..c.detect
        };
        assert!(c.validate().is_err());
    }
}
