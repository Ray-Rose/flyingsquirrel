//! Task wiring & lifecycle. The only place channels are constructed.

use crate::action::FlightController;
use crate::error::FsError;
use crate::fusion::{Fusion, FusionConfig};
use crate::ingest::{GpsSource, ImuSource};
use crate::types::SpoofingEvent;
use futures::StreamExt;
use std::sync::Arc;
use std::sync::Mutex as StdMutex;
use tokio::sync::{broadcast, mpsc};
use tokio::task::{JoinHandle, JoinSet};
use tokio::time::Instant;
use tracing::warn;

/// Shared registry for background tasks spawned by Fusion (forensic dumps,
/// closed-loop RTL verification). Exposed via `Handles::pending_bg` so the
/// main shutdown path can drain in-flight work before tearing the runtime
/// down — audit E-10 fix: previously these tasks were orphaned, causing
/// forensic data loss on `systemctl stop`.
///
/// Uses `std::sync::Mutex` (not tokio's): `JoinSet::spawn` is sync, and the
/// drain path takes ownership of the JoinSet (via `mem::take`) before
/// awaiting any tasks — so the lock is never held across an await point.
pub type PendingBgTasks = Arc<StdMutex<JoinSet<()>>>;

pub struct Handles {
    pub gps_ingest: JoinHandle<()>,
    pub imu_ingest: JoinHandle<()>,
    pub fusion: JoinHandle<()>,
    pub action: JoinHandle<()>,
    pub events_rx: broadcast::Receiver<SpoofingEvent>,
    /// Clone-able sender so callers can spawn additional event producers
    /// (e.g. the MAVLink heartbeat watchdog) that share the broadcast bus
    /// with fusion's spoof events.
    pub events_tx: broadcast::Sender<SpoofingEvent>,
    /// Operator-driven reset: send `()` to clear latched SPOOFED and resume
    /// detection from NORMAL. Drive this from a signal handler (SIGHUP on
    /// Unix, Ctrl-Break on Windows), a MAVLink ground-station command, or
    /// any other control plane appropriate to the deployment.
    pub reset_tx: mpsc::Sender<()>,
    /// Monotonic anchor used for SpoofingEvent.mono_ns. External event
    /// producers should use the same anchor so timestamps align.
    pub boot: Instant,
    /// Registry of in-flight background tasks (forensic dumps, RTL verify).
    /// Main's shutdown path should drain these with a bounded timeout before
    /// dropping the runtime. Audit E-10 fix.
    pub pending_bg: PendingBgTasks,
}

/// Optional MAV-aware preflight inputs. `None` = preflight ignores
/// HEARTBEAT / autopilot checks; appropriate for synth + console deployments.
pub struct MavPreflight {
    pub monitor: Arc<crate::mav::monitor::MavMonitor>,
    pub expected_autopilot: u8,
}

/// Optional forensic-dump configuration. `None` = no forensic capture.
pub struct ForensicCfg {
    pub dir: std::path::PathBuf,
    pub window_s: f64,
}

impl ForensicCfg {
    /// AUDIT F-03 fix: reject NaN, Inf, and non-positive window values at
    /// startup. Previously `window_s = NaN` produced a silently-broken buffer
    /// (every record was pruned because `t > NaN` is always false), and
    /// `window_s = INF` allocated a `VecDeque` of `usize::MAX` items and
    /// aborted the process. Both cases happened with no operator-visible
    /// hint that forensic capture was non-functional.
    pub fn validate(&self) -> Result<(), String> {
        if !self.window_s.is_finite() {
            return Err(format!(
                "forensic.window_s must be finite (got {})",
                self.window_s
            ));
        }
        if self.window_s <= 0.0 {
            return Err(format!(
                "forensic.window_s must be > 0 (got {})",
                self.window_s
            ));
        }
        // 1 hour cap — anything bigger is almost certainly a typo and would
        // produce a multi-GB ring buffer.
        if self.window_s > 3600.0 {
            return Err(format!(
                "forensic.window_s must be <= 3600s (got {}); larger values are \
                 almost certainly a typo and would blow ring buffer memory",
                self.window_s
            ));
        }
        Ok(())
    }
}

pub fn spawn_all(
    gps: Box<dyn GpsSource>,
    imu: Box<dyn ImuSource>,
    controller: Arc<dyn FlightController>,
    cfg: FusionConfig,
    mav_preflight: Option<MavPreflight>,
    forensic_cfg: Option<ForensicCfg>,
) -> Result<Handles, FsError> {
    // Fail at startup if any tunable is degenerate. Better to crash on bootup
    // than to silently produce false RTLs or no-ops in flight.
    cfg.validate().map_err(|msg| {
        FsError::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            format!("invalid FusionConfig: {}", msg),
        ))
    })?;
    // AUDIT F-03 fix: validate forensic config (NaN/Inf/negative window_s)
    // before constructing the ring buffer.
    if let Some(fc) = &forensic_cfg {
        fc.validate().map_err(|msg| {
            FsError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("invalid ForensicCfg: {}", msg),
            ))
        })?;
    }

    let boot = Instant::now();
    let (gps_tx, gps_rx) = mpsc::channel(64);
    let (imu_tx, imu_rx) = mpsc::channel(256);
    let (reset_tx, reset_rx) = mpsc::channel::<()>(4);
    let (events_tx, events_rx) = broadcast::channel(32);

    let gps_handle = tokio::spawn(async move {
        let mut s = gps.into_stream();
        while let Some(item) = s.next().await {
            match item {
                Ok(fix) => {
                    if gps_tx.send(fix).await.is_err() {
                        break;
                    }
                }
                Err(e) => warn!(error = %e, "gps ingest error"),
            }
        }
    });

    let imu_handle = tokio::spawn(async move {
        let mut s = imu.into_stream();
        while let Some(item) = s.next().await {
            match item {
                Ok(sample) => {
                    // Try to send without awaiting; on full, drop the OLDEST by recv'ing once.
                    match imu_tx.try_send(sample) {
                        Ok(()) => {}
                        Err(tokio::sync::mpsc::error::TrySendError::Full(s)) => {
                            // Channel full — fall back to awaiting send to apply backpressure.
                            // For v1 this is acceptable: the simulator paces at ~100 Hz; we
                            // would only observe a full channel if Fusion stalled.
                            if imu_tx.send(s).await.is_err() {
                                break;
                            }
                        }
                        Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => break,
                    }
                }
                Err(e) => warn!(error = %e, "imu ingest error"),
            }
        }
    });

    // Subscribe BEFORE spawning the action task. The previous pattern subscribed
    // inside the spawned future, which meant fusion could publish an event before
    // the action future polled — that event would be lost with no warning.
    let mut action_rx = events_tx.subscribe();
    let controller_for_action = controller.clone();
    let action_handle = tokio::spawn(async move {
        loop {
            match action_rx.recv().await {
                Ok(ev) => controller_for_action.on_event(&ev).await,
                Err(broadcast::error::RecvError::Closed) => break,
                Err(broadcast::error::RecvError::Lagged(n)) => {
                    warn!(skipped = n, "action subscriber lagged");
                }
            }
        }
    });

    let max_imu_rate_hz = cfg.max_imu_rate_hz;
    let pending_bg: PendingBgTasks = Arc::new(StdMutex::new(JoinSet::new()));
    let mut fusion = Fusion::new(cfg, boot).with_pending_bg(pending_bg.clone());
    if let Some(mp) = mav_preflight {
        fusion = fusion.with_mav_monitor(mp.monitor, mp.expected_autopilot);
    }
    if let Some(fc) = forensic_cfg {
        fusion = fusion.with_forensic(fc.dir, fc.window_s, max_imu_rate_hz);
    }
    let events_tx_for_handle = events_tx.clone();
    let fusion_handle = tokio::spawn(async move {
        fusion
            .run(gps_rx, imu_rx, reset_rx, events_tx, controller)
            .await;
    });

    Ok(Handles {
        gps_ingest: gps_handle,
        imu_ingest: imu_handle,
        fusion: fusion_handle,
        action: action_handle,
        events_rx,
        events_tx: events_tx_for_handle,
        reset_tx,
        boot,
        pending_bg,
    })
}

/// Drain in-flight background tasks (forensic dumps, RTL verification) up to
/// `timeout`. Returns `(completed, still_running_aborted)`. Audit E-10 fix.
///
/// Takes ownership of the JoinSet under the sync lock (via `mem::take`) so
/// the lock is never held across an await — new `spawn_bg` registrations
/// during drain go into a fresh empty JoinSet (acceptable race: main is
/// shutting down, last-second registrations from fusion are the loss the
/// fallback bare-spawn already accepts).
///
/// Bounded by `timeout` because: (1) a hung disk shouldn't block process
/// exit indefinitely, (2) RTL verification can take up to 5s by design.
/// 7s is enough headroom for both.
pub async fn drain_pending_bg(
    pending: PendingBgTasks,
    timeout: std::time::Duration,
) -> (usize, usize) {
    let mut to_drain = {
        let mut guard = pending.lock().unwrap_or_else(|e| e.into_inner());
        std::mem::take(&mut *guard)
    };
    if to_drain.is_empty() {
        return (0, 0);
    }
    let mut completed = 0usize;
    let deadline = tokio::time::Instant::now() + timeout;
    while !to_drain.is_empty() {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        if remaining.is_zero() {
            let still = to_drain.len();
            to_drain.abort_all();
            return (completed, still);
        }
        match tokio::time::timeout(remaining, to_drain.join_next()).await {
            Ok(Some(Ok(()))) => completed += 1,
            Ok(Some(Err(e))) => {
                tracing::warn!(error = %e, "background task panicked or was aborted");
                completed += 1;
            }
            Ok(None) => break,
            Err(_) => {
                let still = to_drain.len();
                to_drain.abort_all();
                return (completed, still);
            }
        }
    }
    (completed, 0)
}

#[cfg(test)]
mod drain_tests {
    use super::*;
    use std::time::Duration;

    #[tokio::test]
    async fn drains_completed_tasks() {
        let pending: PendingBgTasks = Arc::new(StdMutex::new(JoinSet::new()));
        {
            let mut g = pending.lock().unwrap();
            for _ in 0..3 {
                g.spawn(async {});
            }
        }
        let (completed, aborted) = drain_pending_bg(pending, Duration::from_secs(1)).await;
        assert_eq!(completed, 3);
        assert_eq!(aborted, 0);
    }

    #[tokio::test]
    async fn drain_aborts_runaway_tasks_past_timeout() {
        let pending: PendingBgTasks = Arc::new(StdMutex::new(JoinSet::new()));
        {
            let mut g = pending.lock().unwrap();
            // Two tasks that would run forever — drain must abort.
            for _ in 0..2 {
                g.spawn(async {
                    loop {
                        tokio::time::sleep(Duration::from_secs(60)).await;
                    }
                });
            }
        }
        let (completed, aborted) = drain_pending_bg(pending, Duration::from_millis(100)).await;
        assert_eq!(completed, 0);
        assert_eq!(aborted, 2);
    }

    #[tokio::test]
    async fn drain_empty_returns_immediately() {
        let pending: PendingBgTasks = Arc::new(StdMutex::new(JoinSet::new()));
        let (completed, aborted) = drain_pending_bg(pending, Duration::from_secs(5)).await;
        assert_eq!((completed, aborted), (0, 0));
    }
}
