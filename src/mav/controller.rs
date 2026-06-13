//! `FlightController` that drives a MAVLink endpoint.
//!
//! On `engage_rtb()` we send `MAV_CMD_NAV_RETURN_TO_LAUNCH`. On `sever_gps()`
//! we send a `PARAM_SET` to zero out `GPS_TYPE` — that's how ArduPilot
//! disables its primary GPS driver. Real autopilot will then fall back to
//! the EKF's other position sources (or pure inertial nav if none).
//!
//! UDP is best-effort, so every command is sent N times with a short spacing
//! to maximize the chance that at least one datagram reaches the autopilot.
//! Single-packet loss must never mean a stolen drone.

use super::monitor::{
    is_rtl_mode, px4_encode_mode, MavMonitor, VehicleProfile, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_RTL,
};
use super::MavLink;
use crate::action::FlightController;
use crate::error::FsError;
use crate::types::SpoofingEvent;
use async_trait::async_trait;
use mavlink::common::{MavCmd, MavMessage, MavParamType, COMMAND_LONG_DATA, PARAM_SET_DATA};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::io::AsyncWriteExt;
use tokio::sync::Mutex as AsyncMutex;
use tokio::time::Instant;
use tracing::{error, info, warn};

/// MAV_RESULT_ACCEPTED — the autopilot accepted the command.
const MAV_RESULT_ACCEPTED: u16 = 0;
/// MAV_CMD_NAV_RETURN_TO_LAUNCH value (ArduPilot RTL command).
const MAV_CMD_NAV_RETURN_TO_LAUNCH_ID: u16 = 20;
/// MAV_CMD_DO_SET_MODE value (PX4 RTL command — sets autopilot mode directly).
const MAV_CMD_DO_SET_MODE_ID: u16 = 176;

/// Number of times to send each safety-critical MAVLink command.
const COMMAND_REPEAT: u8 = 3;
/// Spacing between repeated sends.
const COMMAND_SPACING: Duration = Duration::from_millis(100);
/// How long to wait for the autopilot's HEARTBEAT to reflect mode change to
/// RTL after we send the command. ArduPilot typically updates within 1 HEARTBEAT
/// cycle (~1s); allow 5s for slow/lossy links.
const RTB_VERIFY_WINDOW: Duration = Duration::from_secs(5);
/// Polling cadence while waiting for the mode change.
const RTB_VERIFY_POLL: Duration = Duration::from_millis(200);
/// Number of consecutive polls where `current_mode` must read as RTL before
/// we accept the transition. Defeats single-HEARTBEAT mode-flap spoofs that
/// would otherwise inject one packet to fake confirmation. With a 200ms poll
/// and 3 required observations, the autopilot must hold RTL for ~600ms — a
/// real autopilot trivially does this; a spoofer has to commit much harder.
const RTB_VERIFY_DWELL_POLLS: u8 = 3;
/// How long to wait for the autopilot's PARAM_VALUE echo confirming the
/// GPS-disable parameter took effect. Shorter than the RTL window — a param
/// echo is a single immediate response, not a mode transition that may lag a
/// HEARTBEAT cycle. Runs concurrently with RTL verification (it does not delay
/// the RTL command itself), so it stays within the shutdown drain budget.
const SEVER_VERIFY_WINDOW: Duration = Duration::from_secs(3);
/// Polling cadence while waiting for the PARAM_VALUE echo.
const SEVER_VERIFY_POLL: Duration = Duration::from_millis(150);
/// A sever-param read-back this close to zero counts as "GPS aiding disabled"
/// (GPS_TYPE=0 / EKF2_GPS_CTRL=0 echo back as 0.0; tolerate float noise).
const SEVER_CONFIRMED_EPS: f32 = 0.5;

/// The autopilot parameter whose zeroing disables GPS aiding, with its MAVLink
/// type, for `profile`. Split out as a pure function so the per-profile choice
/// is unit-testable without constructing a live `MavFlightController` (which
/// binds a UDP socket).
///
///   * ArduPilot: `GPS_TYPE = 0` disables the primary GPS driver (INT8); the
///     EKF falls back to its remaining position sources / inertial nav. This is
///     the SITL-validated ArduCopter path.
///   * PX4: has no `GPS_TYPE`. GPS fusion is controlled by `EKF2_GPS_CTRL` (a
///     bitmask on modern firmware); `0` disables all GPS aiding (INT32). This is
///     the control the README's PX4 parameter table targets. (Pre-1.13 PX4 used
///     `EKF2_AID_MASK`; `EKF2_GPS_CTRL` is the modern equivalent.)
///
/// Both names fit MAVLink's 16-byte `param_id` field (asserted in tests).
fn sever_gps_param(profile: VehicleProfile) -> (&'static [u8], MavParamType) {
    match profile {
        VehicleProfile::ArduCopter => (b"GPS_TYPE", MavParamType::MAV_PARAM_TYPE_INT8),
        VehicleProfile::Px4 => (b"EKF2_GPS_CTRL", MavParamType::MAV_PARAM_TYPE_INT32),
    }
}

pub struct MavFlightController {
    link: Arc<MavLink>,
    /// Optional JSON-Lines event log (same format as ConsoleController).
    json_writer: Option<AsyncMutex<tokio::fs::File>>,
    target_system: u8,
    target_component: u8,
    /// Operator-declared vehicle. Used to (a) interpret `custom_mode` in
    /// `verify_rtb_engaged`, (b) construct the right RTL command, (c)
    /// reject the verification if the autopilot's MAV_AUTOPILOT byte in
    /// HEARTBEAT disagrees.
    profile: VehicleProfile,
    /// Reader handle on the autopilot-state monitor (HEARTBEAT-derived mode).
    /// Used by `verify_rtb_engaged` to confirm the autopilot actually moved
    /// to RTL after we sent the command.
    monitor: Arc<MavMonitor>,
    /// Monotonic ns relative to `link.boot` at which `engage_rtb` last sent
    /// its first packet. Verification compares this to the autopilot's
    /// `mode_changed_at_ns` and `last_ack_ns` to enforce causality — i.e.,
    /// the autopilot's reaction must have happened AFTER we asked.
    rtb_sent_at_ns: AtomicU64,
    /// Monotonic ns relative to `link.boot` at which `sever_gps` last sent its
    /// first packet. `verify_sever_engaged` requires the autopilot's
    /// PARAM_VALUE echo to be timestamped strictly AFTER this, so a stale
    /// pre-send echo (or a replayed one) can't fake confirmation.
    sever_sent_at_ns: AtomicU64,
}

impl MavFlightController {
    pub async fn new(
        link: Arc<MavLink>,
        json_log: Option<PathBuf>,
        target_system: u8,
        target_component: u8,
        profile: VehicleProfile,
    ) -> Result<Self, FsError> {
        let json_writer = match json_log {
            Some(path) => {
                #[cfg(unix)]
                let file = {
                    use std::os::unix::fs::OpenOptionsExt;
                    tokio::fs::OpenOptions::new()
                        .create(true)
                        .append(true)
                        .mode(0o600) // owner-only — event log can carry trajectory hints
                        .open(&path)
                        .await?
                };
                #[cfg(not(unix))]
                let file = tokio::fs::OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(&path)
                    .await?;
                Some(AsyncMutex::new(file))
            }
            None => None,
        };
        let monitor = link.monitor.clone();
        Ok(Self {
            link,
            json_writer,
            target_system,
            target_component,
            profile,
            monitor,
            rtb_sent_at_ns: AtomicU64::new(0),
            sever_sent_at_ns: AtomicU64::new(0),
        })
    }

    /// Snapshot of `Instant::now() - link.boot` in ns. Same time base as the
    /// monitor's `last_heartbeat_ns` etc. so causality comparisons work.
    fn now_ns(&self) -> u64 {
        Instant::now()
            .saturating_duration_since(self.link.boot)
            .as_nanos() as u64
    }

    /// Construct the right MAVLink message to engage RTL for `self.profile`.
    ///
    /// * ArduCopter: `COMMAND_LONG MAV_CMD_NAV_RETURN_TO_LAUNCH` —
    ///   purpose-built RTL command, takes no params.
    /// * PX4: `COMMAND_LONG MAV_CMD_DO_SET_MODE` with
    ///   `param1 = base_mode | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`,
    ///   `param2 = main_mode = AUTO (4)`,
    ///   `param3 = sub_mode  = AUTO_RTL (5)`. PX4 silently ignores
    ///   DO_SET_MODE if the CUSTOM_MODE_ENABLED bit is missing —
    ///   easy to get wrong.
    fn engage_rtb_message(&self) -> MavMessage {
        match self.profile {
            VehicleProfile::ArduCopter => MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
                target_system: self.target_system,
                target_component: self.target_component,
                command: MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH,
                confirmation: 0,
                param1: 0.0,
                param2: 0.0,
                param3: 0.0,
                param4: 0.0,
                param5: 0.0,
                param6: 0.0,
                param7: 0.0,
            }),
            VehicleProfile::Px4 => {
                // Sanity-encode the mode so log inspection shows the value:
                let _encoded = px4_encode_mode(PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_RTL);
                MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
                    target_system: self.target_system,
                    target_component: self.target_component,
                    command: MavCmd::MAV_CMD_DO_SET_MODE,
                    confirmation: 0,
                    param1: f32::from(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                    param2: f32::from(PX4_MAIN_MODE_AUTO),
                    param3: f32::from(PX4_SUB_MODE_AUTO_RTL),
                    param4: 0.0,
                    param5: 0.0,
                    param6: 0.0,
                    param7: 0.0,
                })
            }
        }
    }

    /// Send a MAVLink message N times with COMMAND_SPACING between attempts.
    /// Returns Ok if AT LEAST ONE send succeeded; otherwise the last error.
    /// Caller is responsible for idempotency (RTL etc. are idempotent in ArduPilot).
    async fn send_repeated(&self, msg: &MavMessage, label: &'static str) -> Result<(), FsError> {
        let mut last_err: Option<FsError> = None;
        let mut succeeded = 0u8;
        for attempt in 1..=COMMAND_REPEAT {
            match self.link.send(msg).await {
                Ok(()) => {
                    succeeded += 1;
                    info!(
                        attempt,
                        of = COMMAND_REPEAT,
                        cmd = label,
                        "mav action send ok"
                    );
                }
                Err(e) => {
                    warn!(attempt, of = COMMAND_REPEAT, cmd = label, error = %e, "mav action send failed");
                    last_err = Some(e);
                }
            }
            if attempt < COMMAND_REPEAT {
                tokio::time::sleep(COMMAND_SPACING).await;
            }
        }
        if succeeded > 0 {
            Ok(())
        } else {
            error!(cmd = label, "mav action: ALL {COMMAND_REPEAT} sends failed");
            Err(last_err.unwrap_or_else(|| FsError::Io(std::io::Error::other("no send attempted"))))
        }
    }
}

#[async_trait]
impl FlightController for MavFlightController {
    async fn on_event(&self, ev: &SpoofingEvent) {
        info!(
            kind = ?ev.kind,
            state = ?ev.state,
            residual_m = ev.residual_m,
            "mav: spoofing event"
        );
        if let Some(writer) = &self.json_writer {
            match serde_json::to_string(ev) {
                Ok(line) => {
                    let mut w = writer.lock().await;
                    if let Err(e) = w.write_all(line.as_bytes()).await {
                        warn!(error = %e, "json event log write failed");
                    } else if let Err(e) = w.write_all(b"\n").await {
                        warn!(error = %e, "json event log newline failed");
                    } else if let Err(e) = w.flush().await {
                        warn!(error = %e, "json event log flush failed");
                    }
                }
                Err(e) => warn!(error = %e, "json event serialize failed"),
            }
        }
    }

    async fn sever_gps(&self) -> Result<(), FsError> {
        // The parameter whose zeroing disables GPS aiding is autopilot-specific.
        // Sending the ArduPilot name to PX4 (or vice-versa) is a SILENT no-op:
        // the autopilot ignores an unknown PARAM_SET, `send_repeated` still
        // reports success (a datagram left the host), and we proceed to RTL — so
        // the drone flies home on the still-spoofed GPS, the exact RQ-170 failure
        // this system exists to prevent. Branch by profile (see `sever_gps_param`).
        let (name, param_type) = sever_gps_param(self.profile);
        // Capture send-time BEFORE the first packet leaves so
        // `verify_sever_engaged` can require a causal (post-send) PARAM_VALUE
        // echo from the autopilot.
        self.sever_sent_at_ns
            .store(self.now_ns(), Ordering::Release);
        warn!(
            param = %String::from_utf8_lossy(name),
            profile = %self.profile,
            "MAV ACTION: PARAM_SET to sever GPS aiding (x{COMMAND_REPEAT})"
        );
        let mut param_id = [0u8; 16];
        param_id[..name.len()].copy_from_slice(name);
        let msg = MavMessage::PARAM_SET(PARAM_SET_DATA {
            target_system: self.target_system,
            target_component: self.target_component,
            param_id,
            param_value: 0.0,
            param_type,
        });
        self.send_repeated(&msg, "sever_gps").await
    }

    async fn reset(&self) -> Result<(), FsError> {
        // Clear the recorded send-time so a stale value from a previous
        // detection cycle can't accidentally satisfy verify_rtb_engaged's
        // causality gate after operator reset. The MAVLink controller has
        // no per-process latches (those were removed in Phase F in favor of
        // FSM-owned latching), so this is the only state to clear.
        self.rtb_sent_at_ns.store(0, Ordering::Release);
        self.sever_sent_at_ns.store(0, Ordering::Release);
        tracing::warn!("MAV ACTION: send-time trackers cleared (operator reset)");
        Ok(())
    }

    async fn engage_rtb(&self) -> Result<(), FsError> {
        // Capture send-time BEFORE first packet leaves so `verify_rtb_engaged`
        // can require causal (post-send) mode transition + ACK.
        let sent_at = self.now_ns();
        self.rtb_sent_at_ns.store(sent_at, Ordering::Release);
        let msg = self.engage_rtb_message();
        let label = match self.profile {
            VehicleProfile::ArduCopter => "engage_rtb(arducopter NAV_RETURN_TO_LAUNCH)",
            VehicleProfile::Px4 => "engage_rtb(px4 DO_SET_MODE→AUTO/RTL)",
        };
        warn!("MAV ACTION: {} (x{COMMAND_REPEAT})", label);
        self.send_repeated(&msg, "engage_rtb").await
    }

    /// Confirm the autopilot ACTUALLY engaged RTL. FOUR independent gates,
    /// ALL required:
    ///
    /// 1. **ACK causality**: a COMMAND_ACK for MAV_CMD_NAV_RETURN_TO_LAUNCH
    ///    with result=ACCEPTED arrived AFTER our send-time. Without this,
    ///    `last_ack` could be from a pre-send unrelated command (or zero).
    ///
    /// 2. **Mode-transition causality**: the autopilot's mode last changed
    ///    AFTER our send-time AND the new mode is RTL-equivalent. Defeats
    ///    "the autopilot was already in RTL for unrelated reasons (low
    ///    battery LAND, pilot manual RTL, geofence) when we fired."
    ///
    /// 3. **Mode-dwell**: the RTL mode must be observed on
    ///    `RTB_VERIFY_DWELL_POLLS` consecutive polls.
    ///
    /// 4. **Dwell HEARTBEAT freshness**: between consecutive RTL observations,
    ///    `last_heartbeat_ns` MUST advance. Without this, the dwell counter
    ///    would tick off the same cached `current_mode` value from a single
    ///    injected HEARTBEAT — three polls 200ms apart all returning the
    ///    same latched mode=6 would falsely satisfy the dwell. With this
    ///    gate, the attacker must send N HEARTBEATs over the window, not 1.
    ///
    /// Without all four, the closed-loop verification was Phase F theater
    /// — a single-packet attacker could fool the operator into believing
    /// the drone was recovering while it kept flying GUIDED.
    async fn verify_rtb_engaged(&self) -> Result<bool, FsError> {
        let sent_at = self.rtb_sent_at_ns.load(Ordering::Acquire);
        if sent_at == 0 {
            warn!("MAV VERIFY called without prior engage_rtb send — refusing to verify");
            return Ok(false);
        }
        let deadline = tokio::time::Instant::now() + RTB_VERIFY_WINDOW;
        let mut consecutive_rtl: u8 = 0;
        // Initialize with the LATEST observed HEARTBEAT at function entry, not
        // zero. With sentinel=0, any cached HB timestamp > 0 would count as
        // "fresh" on the first poll — defeating Phase G2's intent of requiring
        // N freshly-arrived HEARTBEATs over the dwell window. Now the first
        // poll requires a HB STRICTLY newer than what was already on record.
        let mut last_hb_seen: u64 = self.monitor.last_heartbeat_ns.load(Ordering::Acquire);
        loop {
            // Gate 1: ACK causality. The command id depends on which RTL
            // command shape we sent (ArduPilot vs PX4).
            //
            // AUDIT C-14 fix: previously this used `last_ack()` (the most
            // recently observed ACK regardless of command id). If the
            // autopilot ACK'd OTHER commands (mission upload, geofence,
            // status request, MAV_CMD_REQUEST_MESSAGE) during our 5s verify
            // window, those ACKs stomped the RTL ACK and we never confirmed
            // — false ActionUnconfirmed event. Now we look up ACKs by the
            // specific command id we sent, so unrelated traffic doesn't
            // interfere. `last_ack_for_cmd` returns the latest (result, ns)
            // for THIS command only.
            let expected_ack_cmd: u16 = match self.profile {
                VehicleProfile::ArduCopter => MAV_CMD_NAV_RETURN_TO_LAUNCH_ID,
                VehicleProfile::Px4 => MAV_CMD_DO_SET_MODE_ID,
            };
            let (ack_matches, ack_after_send) =
                match self.monitor.last_ack_for_cmd(expected_ack_cmd) {
                    Some((result, ns)) => (result == MAV_RESULT_ACCEPTED, ns > sent_at),
                    None => (false, false),
                };
            let ack_ok = ack_after_send && ack_matches;

            // Gate 2: causal mode transition.
            let mode_changed_at = self.monitor.mode_changed_at_ns();
            let mode = self.monitor.current_mode();
            let mode_changed_after_send = mode_changed_at > sent_at;
            let mode_is_rtl = is_rtl_mode(self.profile, mode);
            // Cross-check: if the autopilot is reporting "disarmed" via
            // base_mode, "RTL confirmed" is meaningless — the drone is on
            // the ground or has crashed. Refuse to confirm in that case.
            let armed = self.monitor.is_armed();
            let causal_mode = mode_changed_after_send && mode_is_rtl && armed;

            // Gate 3+4: dwell counter advances only when (a) mode is RTL AND
            // (b) a NEW HEARTBEAT has arrived since the last observation.
            let cur_hb = self.monitor.last_heartbeat_ns.load(Ordering::Acquire);
            let fresh_hb = cur_hb > last_hb_seen;
            if mode_is_rtl && fresh_hb {
                consecutive_rtl = consecutive_rtl.saturating_add(1);
            } else if !mode_is_rtl {
                consecutive_rtl = 0;
            }
            // Note: if mode_is_rtl but !fresh_hb (we observed the same cached
            // value as last poll), we neither advance nor reset — just wait
            // for the next real HEARTBEAT to confirm.
            last_hb_seen = cur_hb;
            let dwell_ok = consecutive_rtl >= RTB_VERIFY_DWELL_POLLS;

            if ack_ok && causal_mode && dwell_ok {
                info!(
                    custom_mode = mode,
                    ack_command = expected_ack_cmd,
                    profile = %self.profile,
                    dwell_polls = consecutive_rtl,
                    "MAV VERIFY: autopilot confirmed RTL (ACK + causal mode change + fresh-HB dwell)"
                );
                return Ok(true);
            }

            if tokio::time::Instant::now() >= deadline {
                warn!(
                    last_seen_mode = mode,
                    ack_ok,
                    causal_mode,
                    dwell_polls = consecutive_rtl,
                    "MAV VERIFY: RTL NOT confirmed — possible spoofed verification or autopilot failure"
                );
                return Ok(false);
            }
            tokio::time::sleep(RTB_VERIFY_POLL).await;
        }
    }

    /// Confirm the autopilot APPLIED the GPS-disable parameter by waiting for
    /// its PARAM_VALUE echo to read back ≈0, timestamped AFTER our send. This
    /// closes the gap where best-effort UDP delivered the PARAM_SET but the
    /// autopilot dropped/NAK'd/ignored it — leaving the drone on spoofed GPS
    /// even as RTL fires. Returns Ok(true) on confirmed read-back, Ok(false)
    /// if no causal zero-echo arrives within the window.
    async fn verify_sever_engaged(&self) -> Result<bool, FsError> {
        let sent_at = self.sever_sent_at_ns.load(Ordering::Acquire);
        if sent_at == 0 {
            warn!("MAV SEVER VERIFY called without a prior sever_gps send — refusing to verify");
            return Ok(false);
        }
        let (name, _ty) = sever_gps_param(self.profile);
        let deadline = tokio::time::Instant::now() + SEVER_VERIFY_WINDOW;
        loop {
            if let Some((value, ns)) = self.monitor.last_param_value(name) {
                if ns > sent_at && value.abs() < SEVER_CONFIRMED_EPS {
                    info!(
                        param = %String::from_utf8_lossy(name),
                        value,
                        "MAV SEVER VERIFY: autopilot confirmed GPS aiding disabled (PARAM_VALUE read-back)"
                    );
                    return Ok(true);
                }
            }
            if tokio::time::Instant::now() >= deadline {
                warn!(
                    param = %String::from_utf8_lossy(name),
                    "MAV SEVER VERIFY: no causal zero read-back within window — \
                     autopilot may NOT have disabled GPS aiding"
                );
                return Ok(false);
            }
            tokio::time::sleep(SEVER_VERIFY_POLL).await;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sever_param_is_profile_specific() {
        // ArduPilot disables its GPS driver via GPS_TYPE.
        let (name, ty) = sever_gps_param(VehicleProfile::ArduCopter);
        assert_eq!(name, b"GPS_TYPE");
        assert!(matches!(ty, MavParamType::MAV_PARAM_TYPE_INT8));

        // PX4 has no GPS_TYPE — it must use EKF2_GPS_CTRL, or the sever is a
        // silent no-op and the drone RTLs on the spoofed GPS. This is the
        // regression guard for that bug.
        let (name, ty) = sever_gps_param(VehicleProfile::Px4);
        assert_eq!(name, b"EKF2_GPS_CTRL");
        assert!(matches!(ty, MavParamType::MAV_PARAM_TYPE_INT32));

        // Both names must fit MAVLink's fixed 16-byte param_id field.
        assert!(sever_gps_param(VehicleProfile::ArduCopter).0.len() <= 16);
        assert!(sever_gps_param(VehicleProfile::Px4).0.len() <= 16);
    }
}
