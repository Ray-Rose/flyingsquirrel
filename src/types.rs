use serde::{Deserialize, Serialize};
use std::time::Duration;
use tokio::time::Instant;

/// Two-clock timestamp.
///
/// `mono` is the source of truth for all fusion math — it cannot leap,
/// regress, or be influenced by a spoofed GPS time field. `utc` is
/// derived from GPS sentences and is preserved only for human-readable
/// logging.
#[derive(Debug, Clone, Copy)]
pub struct Timestamp {
    pub mono: Instant,
    pub utc: Option<chrono::DateTime<chrono::Utc>>,
}

impl Timestamp {
    pub fn now_mono() -> Self {
        Self {
            mono: Instant::now(),
            utc: None,
        }
    }

    pub fn at(mono: Instant) -> Self {
        Self { mono, utc: None }
    }

    pub fn with_utc(mut self, utc: chrono::DateTime<chrono::Utc>) -> Self {
        self.utc = Some(utc);
        self
    }

    pub fn elapsed_since(&self, earlier: &Timestamp) -> Duration {
        self.mono.saturating_duration_since(earlier.mono)
    }
}

/// One GPS position+velocity fix, frame-agnostic.
#[derive(Debug, Clone, Copy)]
pub struct GpsFix {
    pub t: Timestamp,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f64,
    pub speed_mps: Option<f64>,
    pub course_deg: Option<f64>,
    pub hdop: Option<f32>,
    pub sats: Option<u8>,
}

/// One IMU sample in body frame. Right-handed, X-forward / Y-right / Z-down
/// is what the rest of the code assumes.
#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    pub t: Timestamp,
    pub accel_mps2: [f32; 3],
    pub gyro_rps: [f32; 3],
}

/// Position in a local NED tangent plane (meters from the plane origin).
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct NedPos {
    pub n: f64,
    pub e: f64,
    pub d: f64,
}

impl NedPos {
    pub fn horizontal_norm(&self) -> f64 {
        (self.n * self.n + self.e * self.e).sqrt()
    }
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct NedVel {
    pub n: f64,
    pub e: f64,
    pub d: f64,
}

impl NedVel {
    pub fn horizontal_norm(&self) -> f64 {
        (self.n * self.n + self.e * self.e).sqrt()
    }
}

/// Detector state. Latched at `Spoofed`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NavStateKind {
    Normal,
    Suspicious,
    Spoofed,
}

/// What kind of anomaly the detector emitted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SpoofKind {
    Jump,
    Drift,
    StateTransition,
    SyncWarning,
    /// A controller action (sever_gps / engage_rtb) failed to put bytes on
    /// the wire. The drone may NOT actually be in RTL.
    ActionFailed,
    /// We fired a command and the autopilot's HEARTBEAT confirms it took
    /// effect (e.g. flight mode transitioned to RTL). Recovery is in progress.
    ActionAcked,
    /// We fired a command but the autopilot's mode/state did NOT change
    /// within the verification window. Operator intervention may be needed.
    ActionUnconfirmed,
    /// HEARTBEAT from the autopilot has gone silent past the watchdog timeout.
    /// The link is down — neither GPS-spoof detection nor RTL command delivery
    /// can be trusted.
    LinkDown,
    /// HEARTBEAT has returned after a LinkDown.
    LinkRestored,
    /// The autopilot's flight mode changed for a reason OTHER than our own
    /// engage_rtb (e.g. pilot input, battery failsafe, geofence). Recorded so
    /// post-incident analysis can distinguish "our RTL succeeded" from
    /// "autopilot landed for its own reasons."
    PilotModeChange,
    /// First-fix anchor was rejected because it failed plausibility checks
    /// (too far from configured home, unreasonable HDOP for first lock, etc.).
    /// Defends against meaconing-at-boot.
    BootAnchorRejected,
    /// Pre-flight self-test failed — the system refuses to enter NORMAL
    /// detection state until the operator corrects whatever the test flagged.
    PreflightFailed,
    /// Pre-flight self-test passed; detector is now actively defending.
    PreflightPassed,
    /// Excessive consecutive GPS-dropout-recovery fixes — the FSM has stopped
    /// pausing the suspicious-to-spoofed dwell timer because an attacker may
    /// be throttling GPS to permanently stall escalation. Operator should
    /// investigate. After this fires, normal FSM accounting resumes.
    DwellPauseExceeded,
    /// The same GPS lat/lon has been reported for many consecutive fixes
    /// while the IMU shows the vehicle is moving — either the GPS module
    /// has frozen or an attacker is replaying a captured fix. Treated as
    /// a detector firing (same severity as Jump/Drift).
    FrozenGps,
    /// Forensic dump succeeded: the JSON snapshot of the last N seconds
    /// of GPS/IMU/residual state was written to disk. Detail carries the
    /// path so operators can grep for it.
    ForensicDumpWritten,
    /// Forensic dump failed (disk full, permission denied, already-fired
    /// flag set, etc.). The Spoofed transition still happened; only the
    /// post-mortem artifact is missing.
    ForensicDumpFailed,
    /// Spoofed re-fired but a forensic dump is already on disk for this
    /// process. Suppressed by the once-per-process guard (DoS defense).
    /// Operator can grep for this kind to confirm "the original dump is
    /// the authoritative one for this incident."
    ForensicDumpSuppressed,
}

/// Single anomaly record emitted on the broadcast channel.
#[derive(Debug, Clone, Serialize)]
pub struct SpoofingEvent {
    #[serde(serialize_with = "serialize_instant")]
    pub mono_ns: u128,
    #[serde(with = "chrono::serde::ts_milliseconds_option")]
    pub utc: Option<chrono::DateTime<chrono::Utc>>,
    pub kind: SpoofKind,
    pub state: NavStateKind,
    pub residual_m: f32,
    pub detail: serde_json::Value,
}

fn serialize_instant<S>(v: &u128, s: S) -> Result<S::Ok, S::Error>
where
    S: serde::Serializer,
{
    s.serialize_u128(*v)
}

impl SpoofingEvent {
    pub fn new(
        t: Timestamp,
        kind: SpoofKind,
        state: NavStateKind,
        residual_m: f32,
        detail: serde_json::Value,
        boot: Instant,
    ) -> Self {
        let mono_ns = t.mono.saturating_duration_since(boot).as_nanos();
        Self {
            mono_ns,
            utc: t.utc,
            kind,
            state,
            residual_m,
            detail,
        }
    }
}
