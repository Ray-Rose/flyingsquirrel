//! `MavMonitor` — shared atomic state about the connected autopilot.
//!
//! Populated by the listener as `HEARTBEAT` and `COMMAND_ACK` messages
//! arrive. Read by:
//!   - the heartbeat watchdog task (emits `LinkDown` / `LinkRestored`)
//!   - the `MavFlightController` (verifies mode change after RTL)
//!   - operator dashboards / tests
//!
//! All fields are atomic so reads are lock-free from any task.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicU8, Ordering};
use std::sync::Mutex;
use tokio::time::Instant;

/// ArduCopter custom_mode values that count as "RTL engaged" for post-action
/// verification. Flat enum values — ArduPilot uses a single 8-bit mode id
/// packed into the low byte of custom_mode.
pub const ARDUCOPTER_MODE_RTL: u32 = 6;
pub const ARDUCOPTER_MODE_SMART_RTL: u32 = 21;
pub const ARDUCOPTER_MODE_LAND: u32 = 9; // also acceptable as "the autopilot got the message"

/// PX4 mode encoding uses two bytes packed into custom_mode:
///   bits 31..24 = `main_mode` (e.g. AUTO=4, POSCTL=3, MANUAL=1)
///   bits 23..16 = `sub_mode`  (within AUTO: TAKEOFF=2, LOITER=3, MISSION=4,
///                              RTL=5, LAND=6, RTGS=7, FOLLOW_TARGET=8)
///   bits 15..0  = reserved
///
/// "RTL engaged" for PX4 = `main_mode=AUTO` AND `sub_mode ∈ {RTL, LAND, RTGS}`.
/// Source: PX4 Firmware `src/modules/commander/px4_custom_mode.h`.
pub const PX4_MAIN_MODE_AUTO: u8 = 4;
pub const PX4_SUB_MODE_AUTO_RTL: u8 = 5;
pub const PX4_SUB_MODE_AUTO_LAND: u8 = 6;
pub const PX4_SUB_MODE_AUTO_RTGS: u8 = 7;

/// Extract (main, sub) from a PX4 `custom_mode` value.
pub fn px4_split_mode(custom_mode: u32) -> (u8, u8) {
    let main = ((custom_mode >> 24) & 0xFF) as u8;
    let sub = ((custom_mode >> 16) & 0xFF) as u8;
    (main, sub)
}

/// Encode a PX4 (main, sub) pair into a 32-bit `custom_mode`. Inverse of
/// `px4_split_mode` (the low 16 bits are reserved → zero).
pub fn px4_encode_mode(main: u8, sub: u8) -> u32 {
    ((main as u32) << 24) | ((sub as u32) << 16)
}

/// Supported autopilot family. The verifier uses this to:
///   - interpret `custom_mode` correctly (each vehicle has different mode IDs
///     AND different encodings — PX4 uses a packed 24-bit form),
///   - construct the right "engage RTL" MAVLink message (ArduPilot uses
///     `MAV_CMD_NAV_RETURN_TO_LAUNCH`; PX4 uses `MAV_CMD_DO_SET_MODE` with
///     an encoded custom_mode),
///   - cross-check the autopilot's reported `MAV_AUTOPILOT` byte (refuse to
///     run verification if the actual autopilot disagrees with the operator's
///     `--vehicle` flag).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleProfile {
    /// ArduPilot Copter / Heli — flat custom_mode, MAV_AUTOPILOT_ARDUPILOTMEGA.
    ArduCopter,
    /// PX4 (any vehicle type). Packed custom_mode, MAV_AUTOPILOT_PX4,
    /// MAV_CMD_DO_SET_MODE for mode changes.
    Px4,
}

impl std::fmt::Display for VehicleProfile {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ArduCopter => write!(f, "ardu-copter"),
            Self::Px4 => write!(f, "px4"),
        }
    }
}

impl std::str::FromStr for VehicleProfile {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "ardu-copter" | "ardu_copter" | "arducopter" => Ok(Self::ArduCopter),
            "px4" | "px4-copter" | "px4-fixed-wing" | "px4-rover" => Ok(Self::Px4),
            other => Err(format!(
                "unknown vehicle profile {:?}; supported: ardu-copter, px4",
                other
            )),
        }
    }
}

/// True if `custom_mode` represents an RTL-equivalent state for `profile`.
///
///   ArduCopter: flat enum, RTL (6) / SMART_RTL (21) / LAND (9).
///   PX4:        packed `(main, sub)`, AUTO+{RTL, LAND, RTGS}.
///
/// We accept LAND-class states as "RTL engaged" because the operator
/// intent — the drone is no longer being navigated by the (potentially
/// spoofed) GPS — is satisfied whether the autopilot returns home or just
/// lands on the spot.
pub fn is_rtl_mode(profile: VehicleProfile, custom_mode: u32) -> bool {
    match profile {
        VehicleProfile::ArduCopter => matches!(
            custom_mode,
            ARDUCOPTER_MODE_RTL | ARDUCOPTER_MODE_SMART_RTL | ARDUCOPTER_MODE_LAND
        ),
        VehicleProfile::Px4 => {
            let (main, sub) = px4_split_mode(custom_mode);
            main == PX4_MAIN_MODE_AUTO
                && matches!(
                    sub,
                    PX4_SUB_MODE_AUTO_RTL | PX4_SUB_MODE_AUTO_LAND | PX4_SUB_MODE_AUTO_RTGS
                )
        }
    }
}

/// MAV_MODE_FLAG bits from the MAVLink common dialect.
///   * `SAFETY_ARMED` (bit 7, 0x80) — `verify_rtb_engaged` cross-checks
///     this so an attacker can't fake confirmation with `custom_mode=RTL`
///     while the autopilot is plainly disarmed.
///   * `CUSTOM_MODE_ENABLED` (bit 0, 0x01) — PX4's `MAV_CMD_DO_SET_MODE`
///     REQUIRES this bit in `base_mode` for the autopilot to honor the
///     custom_mode field. Without it, PX4 silently ignores the command.
pub const MAV_MODE_FLAG_SAFETY_ARMED: u8 = 0x80;
pub const MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: u8 = 0x01;

/// MAV_AUTOPILOT values that the operator's `--vehicle` flag may legitimately
/// match.
///   * ArduCopter → `MAV_AUTOPILOT_ARDUPILOTMEGA` = 3
///   * PX4        → `MAV_AUTOPILOT_PX4`           = 12
pub const MAV_AUTOPILOT_ARDUPILOTMEGA: u8 = 3;
pub const MAV_AUTOPILOT_PX4: u8 = 12;

pub fn expected_autopilot_byte(profile: VehicleProfile) -> u8 {
    match profile {
        VehicleProfile::ArduCopter => MAV_AUTOPILOT_ARDUPILOTMEGA,
        VehicleProfile::Px4 => MAV_AUTOPILOT_PX4,
    }
}

/// Atomic-state view of the connected autopilot. Constructed once and shared
/// via Arc across the listener (writer) and watchdog/controller (readers).
pub struct MavMonitor {
    /// Monotonic nanoseconds (relative to `boot`) of the last HEARTBEAT we
    /// successfully parsed. 0 means "never seen".
    pub last_heartbeat_ns: AtomicU64,
    /// Most recently observed `custom_mode` field. Meaning is autopilot-specific
    /// (ArduCopter: 6 = RTL, 21 = SMART_RTL, etc.).
    pub current_custom_mode: AtomicU32,
    /// Monotonic ns at which `current_custom_mode` LAST TRANSITIONED to its
    /// present value. Lets verification check "the mode changed AFTER we sent
    /// the command" — without this, an attacker can plant a `custom_mode=RTL`
    /// HEARTBEAT before we ever fire RTL, and we'd false-confirm.
    pub mode_changed_at_ns: AtomicU64,
    /// True if we've parsed at least one HEARTBEAT.
    pub heartbeat_ever: AtomicBool,
    /// Most recent `base_mode` flags. Bit 0x80 = `MAV_MODE_FLAG_SAFETY_ARMED`
    /// — `verify_rtb_engaged` cross-checks this so an attacker can't fake
    /// confirmation by injecting `custom_mode=RTL` while `base_mode` clearly
    /// shows the autopilot is disarmed (i.e. crashed / on-ground / pre-arm).
    pub current_base_mode: AtomicU8,
    /// Most recent `autopilot` field from a HEARTBEAT. Compared once at the
    /// first HEARTBEAT against the operator's `--vehicle` choice; mismatch
    /// blocks Preflight from passing.
    pub current_autopilot: AtomicU8,
    /// Most recent `system_status` value (MAV_STATE enum).
    pub system_status: AtomicU8,
    /// Result of the most recent COMMAND_ACK we observed. Stored as
    /// (command_id, result) packed into a single u32 so reads stay atomic:
    ///   high 16 bits = command id
    ///   low  16 bits = result (MAV_RESULT enum)
    /// 0 means "no ack observed yet". Kept for back-compat / legacy reads;
    /// the per-command map below is the source of truth for verification.
    pub last_ack_packed: AtomicU32,
    /// Monotonic nanoseconds of the last COMMAND_ACK we observed.
    pub last_ack_ns: AtomicU64,
    /// AUDIT C-14 fix: per-command-id ACK record. Previously the verifier
    /// used `last_ack()` which is whatever ACK arrived most recently. If the
    /// autopilot ACK'd other commands (mission upload, geofence, status
    /// request) during our verify window, those ACKs stomped the RTL ACK
    /// and we never confirmed. With this map, `last_ack_for_cmd(cmd_id)`
    /// returns the most recent ACK for THAT specific command, immune to
    /// noise from unrelated traffic.
    pub acks_by_cmd: Mutex<HashMap<u16, (u16, u64)>>,
    /// Per-parameter PARAM_VALUE record: trimmed param name → (value, mono ns
    /// of receipt). The autopilot echoes a PARAM_VALUE for every PARAM_SET it
    /// applies — this map is what lets `sever_gps` CONFIRM the GPS-disable
    /// parameter actually took effect instead of trusting fire-and-forget UDP
    /// (an unconfirmed sever means the autopilot may still be navigating on
    /// spoofed GPS when RTL fires). Same 64-entry / evict-oldest bounding as
    /// `acks_by_cmd`.
    params_by_name: Mutex<HashMap<Vec<u8>, (f32, u64)>>,
    /// Source `(IP, port)` of the first HEARTBEAT we accepted. After this is
    /// set, the listener rejects packets from any other source — even if the
    /// IP is on the source-IP allowlist. Defends against a co-located attacker
    /// who can forge MAVLink from the same host as the legitimate autopilot.
    locked_source: Mutex<Option<SocketAddr>>,
}

impl Default for MavMonitor {
    fn default() -> Self {
        Self {
            last_heartbeat_ns: AtomicU64::new(0),
            current_custom_mode: AtomicU32::new(0),
            mode_changed_at_ns: AtomicU64::new(0),
            heartbeat_ever: AtomicBool::new(false),
            current_base_mode: AtomicU8::new(0),
            current_autopilot: AtomicU8::new(0),
            system_status: AtomicU8::new(0),
            last_ack_packed: AtomicU32::new(0),
            last_ack_ns: AtomicU64::new(0),
            acks_by_cmd: Mutex::new(HashMap::new()),
            params_by_name: Mutex::new(HashMap::new()),
            locked_source: Mutex::new(None),
        }
    }
}

impl MavMonitor {
    pub fn new() -> Self {
        Self::default()
    }

    /// Record one HEARTBEAT observation. `now` is the wall-clock `Instant`
    /// at which we received it; the monitor stores it relative to `boot`.
    /// Returns `true` if `custom_mode` differs from the prior value — used
    /// by the listener to emit `PilotModeChange` events.
    pub fn record_heartbeat(
        &self,
        now: Instant,
        boot: Instant,
        custom_mode: u32,
        base_mode: u8,
        system_status: u8,
        autopilot: u8,
    ) -> bool {
        let ns = now.saturating_duration_since(boot).as_nanos() as u64;
        let prev_mode = self.current_custom_mode.load(Ordering::Acquire);
        let mode_changed = prev_mode != custom_mode;
        self.last_heartbeat_ns.store(ns, Ordering::Release);
        self.current_custom_mode
            .store(custom_mode, Ordering::Release);
        self.current_base_mode.store(base_mode, Ordering::Release);
        self.system_status.store(system_status, Ordering::Release);
        self.current_autopilot.store(autopilot, Ordering::Release);
        if mode_changed {
            self.mode_changed_at_ns.store(ns, Ordering::Release);
        }
        let was_first = !self.heartbeat_ever.swap(true, Ordering::AcqRel);
        // First HB always counts as a "change" for downstream PilotModeChange
        // emission so operator sees the initial mode in the event stream.
        mode_changed || was_first
    }

    pub fn current_base_mode(&self) -> u8 {
        self.current_base_mode.load(Ordering::Acquire)
    }

    pub fn current_autopilot(&self) -> u8 {
        self.current_autopilot.load(Ordering::Acquire)
    }

    /// True if the autopilot is armed (HEARTBEAT base_mode has SAFETY_ARMED).
    pub fn is_armed(&self) -> bool {
        (self.current_base_mode.load(Ordering::Acquire) & MAV_MODE_FLAG_SAFETY_ARMED) != 0
    }

    /// Record one COMMAND_ACK observation. Stored both in the legacy
    /// `last_ack_packed` slot (back-compat) AND in the per-command map
    /// (`acks_by_cmd`) so verification can look up the latest ACK for a
    /// specific command id rather than racing with unrelated traffic.
    pub fn record_ack(&self, now: Instant, boot: Instant, command: u16, result: u16) {
        let packed = ((command as u32) << 16) | (result as u32);
        let ns = now.saturating_duration_since(boot).as_nanos() as u64;
        self.last_ack_packed.store(packed, Ordering::Release);
        self.last_ack_ns.store(ns, Ordering::Release);
        let mut map = self.acks_by_cmd.lock().unwrap_or_else(|e| e.into_inner());
        map.insert(command, (result, ns));
        // Bound the map so a misbehaving peer can't grow it without limit.
        // MAVLink defines hundreds of commands, but in practice an autopilot
        // ACKs maybe 10-20 distinct ones per session. Cap at 64.
        if map.len() > 64 {
            // Evict the oldest entry by ns.
            if let Some(oldest_cmd) = map.iter().min_by_key(|(_, (_, ns))| *ns).map(|(c, _)| *c) {
                map.remove(&oldest_cmd);
            }
        }
    }

    /// Read the most recent (command, result) pair, if any. Kept for
    /// back-compat; new code should use `last_ack_for_cmd` for targeted
    /// verification.
    pub fn last_ack(&self) -> Option<(u16, u16)> {
        let packed = self.last_ack_packed.load(Ordering::Acquire);
        if packed == 0 {
            return None;
        }
        let cmd = (packed >> 16) as u16;
        let res = (packed & 0xFFFF) as u16;
        Some((cmd, res))
    }

    /// Read the most recent ACK for a specific command id. Returns
    /// `(result, mono_ns)` or `None` if no ACK has been observed for that
    /// command. Audit C-14 fix: this is the right hook for closed-loop
    /// verification — looking up by cmd id is immune to noise from other
    /// commands being ACK'd during the verify window.
    pub fn last_ack_for_cmd(&self, command: u16) -> Option<(u16, u64)> {
        let map = self.acks_by_cmd.lock().unwrap_or_else(|e| e.into_inner());
        map.get(&command).copied()
    }

    /// Record one PARAM_VALUE observation. `param_id` is the raw 16-byte
    /// field from the wire; trailing NULs are stripped so lookups can use the
    /// logical name (`b"GPS_TYPE"`). Keyed per-name for the same reason ACKs
    /// are keyed per-command: a GCS parameter download streaming hundreds of
    /// PARAM_VALUEs during our read-back window must not stomp the one echo
    /// we care about.
    pub fn record_param_value(&self, now: Instant, boot: Instant, param_id: &[u8], value: f32) {
        let ns = now.saturating_duration_since(boot).as_nanos() as u64;
        let name: Vec<u8> = param_id.iter().copied().take_while(|&b| b != 0).collect();
        if name.is_empty() {
            return; // malformed echo; nothing to key on
        }
        let mut map = self
            .params_by_name
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        map.insert(name, (value, ns));
        // Bound the map: an autopilot has thousands of params and a full GCS
        // param download would otherwise grow this without limit. We only
        // ever look up the handful of sever-relevant names; cap at 64 and
        // evict the oldest by receipt time (mirrors `acks_by_cmd`).
        if map.len() > 64 {
            if let Some(oldest) = map
                .iter()
                .min_by_key(|(_, (_, ns))| *ns)
                .map(|(k, _)| k.clone())
            {
                map.remove(&oldest);
            }
        }
    }

    /// Read the most recent PARAM_VALUE for a specific (trimmed) param name.
    /// Returns `(value, mono_ns)` or `None` if never observed.
    pub fn last_param_value(&self, name: &[u8]) -> Option<(f32, u64)> {
        let map = self
            .params_by_name
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        map.get(name).copied()
    }

    /// True if a HEARTBEAT has ever been observed AND the last one was
    /// within `staleness_ns` of `now`.
    pub fn heartbeat_fresh(&self, now: Instant, boot: Instant, staleness_ns: u64) -> bool {
        if !self.heartbeat_ever.load(Ordering::Acquire) {
            return false;
        }
        let last = self.last_heartbeat_ns.load(Ordering::Acquire);
        let now_ns = now.saturating_duration_since(boot).as_nanos() as u64;
        now_ns.saturating_sub(last) <= staleness_ns
    }

    pub fn current_mode(&self) -> u32 {
        self.current_custom_mode.load(Ordering::Acquire)
    }

    /// Monotonic ns at which the autopilot's mode most recently transitioned.
    /// Used for causality checks: a verified action MUST have its mode
    /// transition timestamped strictly AFTER the controller's send-time, else
    /// the "transition" predated our command and is unrelated to us.
    pub fn mode_changed_at_ns(&self) -> u64 {
        self.mode_changed_at_ns.load(Ordering::Acquire)
    }

    pub fn last_ack_ns(&self) -> u64 {
        self.last_ack_ns.load(Ordering::Acquire)
    }

    /// Returns `true` if this `src` is allowed to update the monitor.
    /// First caller sets the lock; subsequent callers must match.
    /// Recovering from `LinkDown` does NOT clear the lock — the operator
    /// must restart the process to accept a new autopilot endpoint, which
    /// is the desired behavior (otherwise a transient outage could let
    /// an attacker take over the slot).
    ///
    /// `expected_port` is the port the configured remote address is sending
    /// FROM. If `Some(port)`, the lock is established ONLY if the first
    /// observed source matches that port — defeats the co-located bootstrap
    /// race where any local process can plant the first HEARTBEAT from a
    /// random ephemeral port. Pass `None` to accept any source port (less
    /// safe; only sensible for tests).
    pub fn check_or_lock_source(&self, src: SocketAddr, expected_port: Option<u16>) -> bool {
        let mut g = self.locked_source.lock().unwrap_or_else(|e| e.into_inner());
        match *g {
            Some(locked) => locked == src,
            None => {
                if let Some(want) = expected_port {
                    if src.port() != want {
                        // Refuse to LOCK to a port that isn't the configured
                        // peer's sending port. Reject the packet without
                        // pinning anything.
                        return false;
                    }
                }
                *g = Some(src);
                true
            }
        }
    }

    pub fn locked_source(&self) -> Option<SocketAddr> {
        *self.locked_source.lock().unwrap_or_else(|e| e.into_inner())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn heartbeat_recorded_and_freshness_check() {
        let m = MavMonitor::new();
        let boot = Instant::now();
        assert!(!m.heartbeat_fresh(boot, boot, 1_000_000_000));
        let _ = m.record_heartbeat(boot + Duration::from_millis(50), boot, 6, 0x80, 4, 3);
        assert!(m.heartbeat_fresh(boot + Duration::from_millis(100), boot, 1_000_000_000));
        // Past the staleness window:
        assert!(!m.heartbeat_fresh(boot + Duration::from_secs(5), boot, 1_000_000_000));
        assert_eq!(m.current_mode(), 6);
        assert!(m.is_armed());
        assert_eq!(m.current_autopilot(), 3);
    }

    #[test]
    fn mode_change_returns_true_only_on_actual_change() {
        let m = MavMonitor::new();
        let boot = Instant::now();
        // First HB is always a "change" (was_first).
        assert!(m.record_heartbeat(boot + Duration::from_millis(10), boot, 4, 1, 4, 3));
        // Same mode → no change.
        assert!(!m.record_heartbeat(boot + Duration::from_millis(20), boot, 4, 1, 4, 3));
        // New mode → change.
        assert!(m.record_heartbeat(boot + Duration::from_millis(30), boot, 6, 1, 4, 3));
    }

    #[test]
    fn mode_changed_at_ns_advances_only_on_change() {
        let m = MavMonitor::new();
        let boot = Instant::now();
        let _ = m.record_heartbeat(boot + Duration::from_millis(10), boot, 4, 1, 4, 3);
        let t1 = m.mode_changed_at_ns();
        let _ = m.record_heartbeat(boot + Duration::from_millis(20), boot, 4, 1, 4, 3);
        let t2 = m.mode_changed_at_ns();
        assert_eq!(
            t1, t2,
            "mode_changed_at_ns must not advance when mode is stable"
        );
        let _ = m.record_heartbeat(boot + Duration::from_millis(30), boot, 6, 1, 4, 3);
        let t3 = m.mode_changed_at_ns();
        assert!(
            t3 > t2,
            "mode_changed_at_ns must advance on real transition"
        );
    }

    #[test]
    fn arming_flag_recognized() {
        let m = MavMonitor::new();
        let boot = Instant::now();
        // base_mode = 0x80 → armed
        let _ = m.record_heartbeat(boot + Duration::from_millis(1), boot, 6, 0x80, 4, 3);
        assert!(m.is_armed());
        // base_mode = 0x01 (CUSTOM_MODE_ENABLED only, no armed bit)
        let _ = m.record_heartbeat(boot + Duration::from_millis(2), boot, 6, 0x01, 4, 3);
        assert!(
            !m.is_armed(),
            "disarmed autopilot must be reported as not armed"
        );
    }

    #[test]
    fn ack_round_trip() {
        let m = MavMonitor::new();
        let boot = Instant::now();
        assert_eq!(m.last_ack(), None);
        m.record_ack(boot, boot, 20, 0); // MAV_CMD_NAV_RETURN_TO_LAUNCH, MAV_RESULT_ACCEPTED
        assert_eq!(m.last_ack(), Some((20, 0)));
    }

    #[test]
    fn param_value_round_trip_and_name_trim() {
        let m = MavMonitor::new();
        let boot = Instant::now();
        assert_eq!(m.last_param_value(b"GPS_TYPE"), None);
        // 16-byte field with trailing NULs, as it arrives on the wire.
        let mut pid = [0u8; 16];
        pid[..8].copy_from_slice(b"GPS_TYPE");
        m.record_param_value(boot + Duration::from_millis(5), boot, &pid, 0.0);
        let (val, ns) = m.last_param_value(b"GPS_TYPE").expect("param recorded");
        assert_eq!(val, 0.0);
        assert!(ns > 0);
        // Unrelated param doesn't satisfy the lookup.
        assert_eq!(m.last_param_value(b"EKF2_GPS_CTRL"), None);
    }

    #[test]
    fn param_value_not_stomped_by_unrelated_params() {
        // A GCS param download streaming many PARAM_VALUEs around our sever
        // read-back must not displace the echo we care about. The sever echo
        // arrives AFTER our send (recent), so it's among the newest and the
        // evict-oldest bound never touches it — exactly the acks_by_cmd story.
        let m = MavMonitor::new();
        let boot = Instant::now();
        // Flood of older, unrelated params.
        for i in 0..100u64 {
            let mut other = [0u8; 16];
            let name = format!("PARAM_{i}");
            other[..name.len()].copy_from_slice(name.as_bytes());
            m.record_param_value(boot + Duration::from_millis(1 + i), boot, &other, i as f32);
        }
        // Our recent sever echo.
        let mut gps = [0u8; 16];
        gps[..8].copy_from_slice(b"GPS_TYPE");
        m.record_param_value(boot + Duration::from_millis(500), boot, &gps, 0.0);
        let (val, ns) = m
            .last_param_value(b"GPS_TYPE")
            .expect("recent sever echo must survive the flood");
        assert_eq!(val, 0.0);
        assert!(ns > 0);
    }

    #[test]
    fn ack_by_cmd_is_not_stomped_by_other_acks() {
        // Audit C-14: verifier looking for RTL ACK (cmd=20) must still find
        // it after the autopilot ACKs an unrelated command (cmd=400) later
        // in the verify window.
        let m = MavMonitor::new();
        let boot = Instant::now();
        // RTL ACK arrives first.
        m.record_ack(boot + Duration::from_millis(100), boot, 20, 0);
        // Unrelated ACK arrives a moment later — stomps `last_ack` but
        // must NOT affect `last_ack_for_cmd(20)`.
        m.record_ack(boot + Duration::from_millis(150), boot, 400, 0);
        // legacy lookup sees the most recent ACK
        assert_eq!(m.last_ack(), Some((400, 0)));
        // targeted lookup still finds RTL ACK
        let (result, ns) = m
            .last_ack_for_cmd(20)
            .expect("RTL ACK must be retrievable by command id");
        assert_eq!(result, 0);
        assert!(ns > 0);
        // unrelated command also resolves
        assert!(m.last_ack_for_cmd(400).is_some());
        // command we never ACK'd → None
        assert!(m.last_ack_for_cmd(176).is_none());
    }

    #[test]
    fn ack_map_evicts_oldest_when_capped() {
        // Bound check: many distinct ACK command ids must not grow the map
        // without bound. Push 70 distinct cmd ids — cap is 64 — confirm
        // the oldest are evicted.
        let m = MavMonitor::new();
        let boot = Instant::now();
        for i in 0..70u16 {
            m.record_ack(boot + Duration::from_millis(i as u64), boot, i, 0);
        }
        let map = m.acks_by_cmd.lock().unwrap();
        assert!(map.len() <= 64, "map must be bounded, got {}", map.len());
        // Earliest cmd ids (0, 1, ...) should be evicted.
        assert!(!map.contains_key(&0));
        // Latest cmd ids should be retained.
        assert!(map.contains_key(&69));
    }

    #[test]
    fn rtl_mode_recognition_arducopter() {
        let p = VehicleProfile::ArduCopter;
        assert!(is_rtl_mode(p, ARDUCOPTER_MODE_RTL));
        assert!(is_rtl_mode(p, ARDUCOPTER_MODE_SMART_RTL));
        assert!(is_rtl_mode(p, ARDUCOPTER_MODE_LAND));
        assert!(!is_rtl_mode(p, 0));
        assert!(!is_rtl_mode(p, 4)); // GUIDED, not RTL
    }

    #[test]
    fn rtl_mode_recognition_px4() {
        let p = VehicleProfile::Px4;
        // AUTO + RTL
        assert!(is_rtl_mode(
            p,
            px4_encode_mode(PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_RTL)
        ));
        // AUTO + LAND (acceptable per "operator intent satisfied" rule)
        assert!(is_rtl_mode(
            p,
            px4_encode_mode(PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_LAND)
        ));
        // AUTO + RTGS (return-to-groundstation, also acceptable)
        assert!(is_rtl_mode(
            p,
            px4_encode_mode(PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_RTGS)
        ));
        // AUTO + LOITER — NOT RTL
        assert!(!is_rtl_mode(p, px4_encode_mode(PX4_MAIN_MODE_AUTO, 3)));
        // POSCTL (main=3) + anything — NOT RTL
        assert!(!is_rtl_mode(p, px4_encode_mode(3, PX4_SUB_MODE_AUTO_RTL)));
        // Zero — NOT RTL
        assert!(!is_rtl_mode(p, 0));
        // ArduCopter's flat encoding accidentally matching — must NOT match.
        // ArduCopter RTL=6 has all bits in the low byte; PX4 decoder reads
        // main from the HIGH byte (0) → not AUTO → not RTL. Good.
        assert!(!is_rtl_mode(p, ARDUCOPTER_MODE_RTL));
    }

    #[test]
    fn px4_mode_encoding_round_trips() {
        for &(main, sub) in &[(4u8, 5u8), (4, 6), (4, 7), (1, 0), (3, 0)] {
            let encoded = px4_encode_mode(main, sub);
            let (m, s) = px4_split_mode(encoded);
            assert_eq!((m, s), (main, sub), "round-trip failed for ({main},{sub})");
        }
    }

    #[test]
    fn vehicle_profile_parses_aliases() {
        use std::str::FromStr;
        for alias in &["ardu-copter", "ardu_copter", "arducopter"] {
            assert_eq!(
                VehicleProfile::from_str(alias).unwrap(),
                VehicleProfile::ArduCopter
            );
        }
        for alias in &["px4", "px4-copter", "px4-fixed-wing", "px4-rover"] {
            assert_eq!(
                VehicleProfile::from_str(alias).unwrap(),
                VehicleProfile::Px4
            );
        }
        assert!(VehicleProfile::from_str("ardurover").is_err());
        assert!(VehicleProfile::from_str("paparazzi").is_err());
    }

    #[test]
    fn expected_autopilot_byte_per_profile() {
        assert_eq!(expected_autopilot_byte(VehicleProfile::ArduCopter), 3);
        assert_eq!(expected_autopilot_byte(VehicleProfile::Px4), 12);
    }

    #[test]
    fn source_lock_pins_first_caller() {
        use std::net::{IpAddr, Ipv4Addr};
        let m = MavMonitor::new();
        let a: SocketAddr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(192, 168, 1, 10)), 14550);
        let b: SocketAddr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(192, 168, 1, 11)), 14550);
        // First caller wins.
        assert!(m.check_or_lock_source(a, None));
        // Same source = accepted.
        assert!(m.check_or_lock_source(a, None));
        // Different source = rejected.
        assert!(!m.check_or_lock_source(b, None));
        // Lock persists across many calls.
        for _ in 0..5 {
            assert!(!m.check_or_lock_source(b, None));
        }
        assert!(m.check_or_lock_source(a, None));
    }

    #[test]
    fn source_lock_rejects_same_ip_different_port() {
        use std::net::{IpAddr, Ipv4Addr};
        let m = MavMonitor::new();
        let a: SocketAddr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 14550);
        let b: SocketAddr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 14551);
        assert!(m.check_or_lock_source(a, None));
        // Co-located attacker on same IP, different port — must be rejected.
        assert!(!m.check_or_lock_source(b, None));
    }

    #[test]
    fn source_lock_rejects_wrong_port_at_bootstrap() {
        use std::net::{IpAddr, Ipv4Addr};
        let m = MavMonitor::new();
        let attacker: SocketAddr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 31337);
        let real: SocketAddr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 14550);
        // Expected port is 14550 (configured remote). Attacker from ephemeral port
        // tries to plant the bootstrap heartbeat — must be refused WITHOUT pinning.
        assert!(!m.check_or_lock_source(attacker, Some(14550)));
        // Lock is still empty — real autopilot can still establish.
        assert!(m.locked_source().is_none());
        assert!(m.check_or_lock_source(real, Some(14550)));
        // Now attacker can't take over.
        assert!(!m.check_or_lock_source(attacker, Some(14550)));
    }

    #[test]
    fn source_lock_any_port_accepts_ephemeral_bridge() {
        use std::net::{IpAddr, Ipv4Addr};
        // --mav-allow-any-source-port path: expected_port = None. A bridge
        // (mavproxy `--out udp:`) forwarding from an ephemeral source port must
        // be able to establish the lock — the strict default would silently
        // reject it and the detector would go deaf.
        let m = MavMonitor::new();
        let bridge: SocketAddr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 54321);
        assert!(m.check_or_lock_source(bridge, None));
        // Once locked, the same (ip, ephemeral port) keeps working...
        assert!(m.check_or_lock_source(bridge, None));
        // ...but a different port from the same IP is now rejected (the lock
        // pinned the full socket addr, so this stays as strong as before once
        // bootstrapped).
        let other: SocketAddr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 54322);
        assert!(!m.check_or_lock_source(other, None));
    }
}
