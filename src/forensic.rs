//! Forensic ring buffer.
//!
//! When a `Spoofed` transition fires, operators need to reconstruct what
//! the detector was SEEING in the lead-up. The JSON event log captures
//! events but NOT the underlying GPS/IMU/residual stream that fed the
//! decision — without that data, "did the detector fire on a real attack
//! or on bad sensor noise?" is unanswerable.
//!
//! `ForensicBuffer` keeps the last N seconds of sensor and decision state.
//! On the first `Spoofed` transition of a process lifetime, fusion takes
//! a snapshot and writes a JSON file to a configured directory:
//!
//!   {dir}/spoof-{YYYYMMDDTHHMMSSZ}-{pid}.json
//!
//! Properties of the dump:
//!
//! * **Captured BEFORE actions fire.** The snapshot is taken in the same
//!   call that emits the Spoofed transition event, BEFORE `try_action`
//!   runs sever_gps + engage_rtb. That preserves the exact pre-action
//!   state — if you take it after, you've also captured the ~600ms of
//!   send latency.
//!
//! * **Atomic file write.** Write to `.tmp`, then `rename` — the file
//!   either doesn't exist or has the complete payload. No half-written
//!   logs in a post-incident kernel panic.
//!
//! * **`O_EXCL` create + `0o600` mode.** Never overwrite an existing
//!   dump (forensic data is sacred); owner-only on disk so other users
//!   on the SBC can't read flight trajectories.
//!
//! * **Once per process.** An attacker who can trigger Spoofed (or a
//!   buggy detector in a false-positive loop) could otherwise fill the
//!   disk with dumps. After the first dump, subsequent triggers emit a
//!   `ForensicDumpSuppressed` event instead. Operator restart re-arms
//!   the buffer for a fresh dump.

use crate::detect::residual::Residual;
use crate::error::FsError;
use crate::types::{GpsFix, ImuSample, NavStateKind};
use serde::Serialize;
use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// Defaults the operator can override via `[forensic].window_s` in the
/// config. 30 seconds picked as the audit's recommendation for capturing
/// the typical Suspicious-dwell window plus a buffer of normal flight.
pub const DEFAULT_WINDOW_S: f64 = 30.0;

/// Serializable mirror of `GpsFix`. The runtime `GpsFix` uses
/// `tokio::time::Instant` which is intentionally non-serializable —
/// translate to `mono_secs` (seconds since boot) at push time.
#[derive(Debug, Clone, Serialize)]
pub struct GpsFixRecord {
    pub t_secs: f64,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f64,
    pub speed_mps: Option<f64>,
    pub course_deg: Option<f64>,
    pub hdop: Option<f32>,
    pub sats: Option<u8>,
}

impl From<(f64, &GpsFix)> for GpsFixRecord {
    fn from((t_secs, f): (f64, &GpsFix)) -> Self {
        Self {
            t_secs,
            lat_deg: f.lat_deg,
            lon_deg: f.lon_deg,
            alt_m: f.alt_m,
            speed_mps: f.speed_mps,
            course_deg: f.course_deg,
            hdop: f.hdop,
            sats: f.sats,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct ImuSampleRecord {
    pub t_secs: f64,
    pub accel_mps2: [f32; 3],
    pub gyro_rps: [f32; 3],
}

impl From<(f64, &ImuSample)> for ImuSampleRecord {
    fn from((t_secs, s): (f64, &ImuSample)) -> Self {
        Self {
            t_secs,
            accel_mps2: s.accel_mps2,
            gyro_rps: s.gyro_rps,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct ResidualRecord {
    pub t_secs: f64,
    pub dpos_n: f64,
    pub dpos_e: f64,
    pub mag_pos: f64,
    pub mag_vel: f64,
    pub dvel_known: bool,
    pub fsm_state: NavStateKind,
}

impl ResidualRecord {
    pub fn from_components(t_secs: f64, r: &Residual, state: NavStateKind) -> Self {
        Self {
            t_secs,
            dpos_n: r.dpos.n,
            dpos_e: r.dpos.e,
            mag_pos: r.mag_pos,
            mag_vel: r.mag_vel,
            dvel_known: r.dvel_known,
            fsm_state: state,
        }
    }
}

/// What kind of event triggered this forensic dump. Currently only
/// `SpoofedTransition`, but the enum exists so we can add other triggers
/// (e.g., operator-requested snapshot via SIGUSR1) without re-versioning
/// the dump schema.
#[derive(Debug, Clone, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ForensicTrigger {
    SpoofedTransition {
        from: NavStateKind,
        to: NavStateKind,
        residual_m: f32,
        event_detail: serde_json::Value,
    },
}

/// Self-contained dump payload. Serialized as a single JSON document so
/// post-incident tooling can do `cat dump.json | jq` without parsing
/// per-line.
#[derive(Debug, Clone, Serialize)]
pub struct ForensicSnapshot {
    /// Schema version. Increment on incompatible changes.
    pub schema_version: u32,
    /// UTC wall-clock timestamp at which the snapshot was taken.
    pub captured_at_utc: chrono::DateTime<chrono::Utc>,
    /// Monotonic ns since process boot, matching SpoofingEvent.mono_ns.
    pub captured_at_mono_ns: u128,
    /// What the buffer window setting was. Lets analysts interpret the
    /// "earliest record is X seconds ago" timeline correctly.
    pub window_s: f64,
    /// The crate version + binary SHA-256, mirroring the startup
    /// attestation event so operators can correlate "this dump came from
    /// THAT binary."
    pub attestation: AttestationSummary,
    /// What triggered the snapshot.
    pub trigger: ForensicTrigger,
    /// The full residual buffer. One record per evaluated GPS fix.
    pub residuals: Vec<ResidualRecord>,
    /// GPS fixes in the window. Time-aligned with residuals (residual N
    /// is computed for gps[N], roughly).
    pub gps_fixes: Vec<GpsFixRecord>,
    /// IMU samples in the window. Dense — typically 100 Hz × window_s.
    /// Trimmed by the buffer's capacity heuristic.
    pub imu_samples: Vec<ImuSampleRecord>,
}

#[derive(Debug, Clone, Serialize)]
pub struct AttestationSummary {
    pub crate_version: String,
    pub binary_sha256: Option<String>,
    pub host_os: String,
    pub host_arch: String,
}

/// Ring buffer over the three sensor streams. Capacity-bounded both by
/// total time (`window_s`) and by per-stream item count (so an IMU at
/// 1 kHz can't grow unboundedly).
pub struct ForensicBuffer {
    gps: VecDeque<GpsFixRecord>,
    imu: VecDeque<ImuSampleRecord>,
    residuals: VecDeque<ResidualRecord>,
    window_s: f64,
    /// Hard caps to bound memory if rate estimates are wrong.
    /// IMU @ 100 Hz × 30 s = 3000 samples; cap at 4× for slack.
    max_imu: usize,
    /// GPS @ 10 Hz × 30 s = 300 fixes; cap at 4× for slack.
    max_gps: usize,
    /// Residuals @ 10 Hz × 30 s = 300; cap at 4× for slack.
    max_residuals: usize,
    /// One-shot guard. After a dump fires, this flips to true and the
    /// buffer ignores further snapshot requests in the same process
    /// lifetime. Defends against attack-driven disk fill.
    dump_fired: Arc<AtomicBool>,
}

impl ForensicBuffer {
    pub fn new(window_s: f64, max_imu_rate_hz: f64) -> Self {
        let imu_cap = ((window_s * max_imu_rate_hz * 4.0).ceil() as usize).max(64);
        let gps_cap = ((window_s * 10.0 * 4.0).ceil() as usize).max(64);
        Self {
            gps: VecDeque::with_capacity(gps_cap),
            imu: VecDeque::with_capacity(imu_cap),
            residuals: VecDeque::with_capacity(gps_cap),
            window_s,
            max_imu: imu_cap,
            max_gps: gps_cap,
            max_residuals: gps_cap,
            dump_fired: Arc::new(AtomicBool::new(false)),
        }
    }

    /// Returns a clone of the one-shot flag so the dump task can flip it
    /// AFTER a successful write — preserves the "once per process" semantic
    /// without coupling fusion to the file-write task.
    pub fn dump_fired_flag(&self) -> Arc<AtomicBool> {
        self.dump_fired.clone()
    }

    pub fn already_dumped(&self) -> bool {
        self.dump_fired.load(Ordering::Acquire)
    }

    pub fn push_gps(&mut self, t_secs: f64, fix: &GpsFix) {
        self.gps.push_back(GpsFixRecord::from((t_secs, fix)));
        self.trim_gps(t_secs);
    }

    pub fn push_imu(&mut self, t_secs: f64, sample: &ImuSample) {
        self.imu.push_back(ImuSampleRecord::from((t_secs, sample)));
        self.trim_imu(t_secs);
    }

    pub fn push_residual(&mut self, t_secs: f64, r: &Residual, state: NavStateKind) {
        self.residuals
            .push_back(ResidualRecord::from_components(t_secs, r, state));
        self.trim_residuals(t_secs);
    }

    fn trim_gps(&mut self, now: f64) {
        let cutoff = now - self.window_s;
        while let Some(front) = self.gps.front() {
            if front.t_secs < cutoff {
                self.gps.pop_front();
            } else {
                break;
            }
        }
        while self.gps.len() > self.max_gps {
            self.gps.pop_front();
        }
    }

    fn trim_imu(&mut self, now: f64) {
        let cutoff = now - self.window_s;
        while let Some(front) = self.imu.front() {
            if front.t_secs < cutoff {
                self.imu.pop_front();
            } else {
                break;
            }
        }
        while self.imu.len() > self.max_imu {
            self.imu.pop_front();
        }
    }

    fn trim_residuals(&mut self, now: f64) {
        let cutoff = now - self.window_s;
        while let Some(front) = self.residuals.front() {
            if front.t_secs < cutoff {
                self.residuals.pop_front();
            } else {
                break;
            }
        }
        while self.residuals.len() > self.max_residuals {
            self.residuals.pop_front();
        }
    }

    /// Materialize a `ForensicSnapshot` for serialization. Called BEFORE
    /// action firing so the captured state is the moment of decision.
    /// Cloning is intentional — fusion keeps running and pushing new
    /// records during the async file write.
    pub fn snapshot(
        &self,
        captured_at_mono_ns: u128,
        trigger: ForensicTrigger,
        attestation: AttestationSummary,
    ) -> ForensicSnapshot {
        ForensicSnapshot {
            schema_version: 1,
            captured_at_utc: chrono::Utc::now(),
            captured_at_mono_ns,
            window_s: self.window_s,
            attestation,
            trigger,
            residuals: self.residuals.iter().cloned().collect(),
            gps_fixes: self.gps.iter().cloned().collect(),
            imu_samples: self.imu.iter().cloned().collect(),
        }
    }
}

/// Write a snapshot to disk. Atomic via tmp+rename. O_EXCL on create.
/// 0o600 mode on unix.
///
/// Filename pattern: `spoof-{YYYYMMDDTHHMMSSZ}-{pid}.json`. PID dis-
/// ambiguates rapid back-to-back invocations (though `dump_fired_flag`
/// should prevent that within one process).
pub async fn dump_to_disk(snap: &ForensicSnapshot, dir: &Path) -> Result<PathBuf, FsError> {
    use tokio::io::AsyncWriteExt;

    // Serialize FIRST so a bad serialization doesn't leave .tmp on disk.
    let json = serde_json::to_vec_pretty(snap)
        .map_err(|e| FsError::Io(std::io::Error::other(format!("forensic serialize: {}", e))))?;

    // Build filename from captured_at_utc and current PID.
    let ts = snap.captured_at_utc.format("%Y%m%dT%H%M%SZ").to_string();
    let pid = std::process::id();
    let final_path = dir.join(format!("spoof-{}-{}.json", ts, pid));
    let tmp_path = dir.join(format!("spoof-{}-{}.json.tmp", ts, pid));

    // Create the directory if it doesn't exist. Operator should have
    // created it via install.sh, but be resilient.
    if let Err(e) = tokio::fs::create_dir_all(dir).await {
        return Err(FsError::Io(std::io::Error::new(
            e.kind(),
            format!("forensic mkdir {}: {}", dir.display(), e),
        )));
    }

    // Atomic write via tmp + rename. O_EXCL so a colliding .tmp from a
    // crashed prior write doesn't get reused (operator should clean those).
    let mut open_opts = tokio::fs::OpenOptions::new();
    open_opts.write(true).create_new(true);
    // tokio's OpenOptions has an inherent `mode()` on unix — no std trait import
    // needed (importing OpenOptionsExt is an unused-import error under -D warnings,
    // a Linux-only build that Windows dev can't see; CI caught it).
    #[cfg(unix)]
    open_opts.mode(0o600);
    let mut file = open_opts.open(&tmp_path).await.map_err(|e| {
        FsError::Io(std::io::Error::new(
            e.kind(),
            format!("forensic open {}: {}", tmp_path.display(), e),
        ))
    })?;
    file.write_all(&json).await.map_err(|e| {
        // Best-effort cleanup of the partial .tmp.
        let _ = std::fs::remove_file(&tmp_path);
        FsError::Io(e)
    })?;
    file.sync_all().await.map_err(FsError::Io)?;
    drop(file);

    tokio::fs::rename(&tmp_path, &final_path)
        .await
        .map_err(|e| {
            let _ = std::fs::remove_file(&tmp_path);
            FsError::Io(std::io::Error::new(
                e.kind(),
                format!(
                    "forensic rename {} → {}: {}",
                    tmp_path.display(),
                    final_path.display(),
                    e
                ),
            ))
        })?;

    Ok(final_path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{NedPos, NedVel, Timestamp};

    fn make_gps(t_secs: f64, lat: f64, lon: f64) -> (f64, GpsFix) {
        (
            t_secs,
            GpsFix {
                t: Timestamp::now_mono(),
                lat_deg: lat,
                lon_deg: lon,
                alt_m: 0.0,
                speed_mps: Some(8.0),
                course_deg: Some(0.0),
                hdop: Some(1.0),
                sats: Some(10),
            },
        )
    }

    fn make_imu(t_secs: f64) -> (f64, ImuSample) {
        (
            t_secs,
            ImuSample {
                t: Timestamp::now_mono(),
                accel_mps2: [0.0, 0.0, 9.81],
                gyro_rps: [0.0; 3],
            },
        )
    }

    fn make_residual(t_secs: f64, n: f64, e: f64) -> (f64, Residual) {
        let r = Residual {
            dpos: NedPos { n, e, d: 0.0 },
            dvel: NedVel::default(),
            mag_pos: (n * n + e * e).sqrt(),
            mag_vel: 0.0,
            dvel_known: true,
        };
        (t_secs, r)
    }

    #[test]
    fn buffer_trims_by_window() {
        let mut b = ForensicBuffer::new(2.0, 100.0);
        for t in 0..30 {
            let (ts, fix) = make_gps(t as f64 * 0.1, 40.0, -100.0);
            b.push_gps(ts, &fix);
        }
        // Window is 2.0s, latest push at t=2.9 → cutoff at 0.9
        // Records at t=0.0..0.8 should be trimmed.
        let earliest = b.gps.front().map(|r| r.t_secs).unwrap();
        assert!(earliest >= 0.9 - 1e-9, "earliest={earliest}");
    }

    #[test]
    fn imu_buffer_caps_count_when_rate_estimate_blown() {
        let mut b = ForensicBuffer::new(1.0, 100.0);
        // Cap is 100 * 1 * 4 = 400. Push 1000.
        for i in 0..1000 {
            // All at t=0 so window trim doesn't fire — only count cap should.
            let (ts, s) = make_imu(0.0);
            let _ = ts;
            b.push_imu(0.0, &s);
            let _ = i;
        }
        assert!(b.imu.len() <= 400, "got {}", b.imu.len());
    }

    #[test]
    fn snapshot_round_trips_through_json() {
        let mut b = ForensicBuffer::new(30.0, 100.0);
        for t in 0..50 {
            let (ts, fix) = make_gps(t as f64, 40.0, -100.0);
            b.push_gps(ts, &fix);
            let (its, isample) = make_imu(t as f64);
            b.push_imu(its, &isample);
            let (rts, r) = make_residual(t as f64, 1.0, 2.0);
            b.push_residual(rts, &r, NavStateKind::Normal);
        }
        let snap = b.snapshot(
            123_456_789,
            ForensicTrigger::SpoofedTransition {
                from: NavStateKind::Suspicious,
                to: NavStateKind::Spoofed,
                residual_m: 75.0,
                event_detail: serde_json::json!({"reason": "test"}),
            },
            AttestationSummary {
                crate_version: "0.1.0".into(),
                binary_sha256: Some("0".repeat(64)),
                host_os: "test".into(),
                host_arch: "test".into(),
            },
        );
        let json = serde_json::to_string(&snap).unwrap();
        // Sanity: must contain known field names.
        assert!(json.contains("\"schema_version\":1"));
        assert!(json.contains("\"residuals\""));
        assert!(json.contains("\"gps_fixes\""));
        assert!(json.contains("\"imu_samples\""));
        // Round-trip parsability.
        let _v: serde_json::Value = serde_json::from_str(&json).unwrap();
    }

    #[tokio::test]
    async fn dump_writes_atomic_file_and_returns_path() {
        let dir = tempfile::tempdir().unwrap();
        let mut b = ForensicBuffer::new(5.0, 100.0);
        let (ts, fix) = make_gps(0.0, 40.0, -100.0);
        b.push_gps(ts, &fix);
        let snap = b.snapshot(
            0,
            ForensicTrigger::SpoofedTransition {
                from: NavStateKind::Suspicious,
                to: NavStateKind::Spoofed,
                residual_m: 50.0,
                event_detail: serde_json::json!({}),
            },
            AttestationSummary {
                crate_version: "0.1.0".into(),
                binary_sha256: None,
                host_os: "test".into(),
                host_arch: "test".into(),
            },
        );
        let path = dump_to_disk(&snap, dir.path()).await.unwrap();
        assert!(path.exists());
        assert!(path.to_string_lossy().contains("spoof-"));
        assert!(path.to_string_lossy().ends_with(".json"));
        // No .tmp left behind.
        let entries: Vec<_> = std::fs::read_dir(dir.path()).unwrap().collect();
        assert_eq!(entries.len(), 1);
    }

    #[test]
    fn dump_fired_flag_is_shared() {
        let b = ForensicBuffer::new(5.0, 100.0);
        let flag = b.dump_fired_flag();
        assert!(!b.already_dumped());
        flag.store(true, Ordering::Release);
        assert!(
            b.already_dumped(),
            "flag mutation must be visible via the buffer"
        );
    }
}
