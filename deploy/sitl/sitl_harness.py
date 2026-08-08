#!/usr/bin/env python3
"""
FlyingSquirrel SITL validation harness — a bidirectional MAVLink relay +
flight choreographer that wires a real ArduPilot SITL to the FlyingSquirrel
detector and drives a full closed-loop spoof test.

Topology (all on one docker network):

    ArduPilot SITL  <--TCP 5760-->  [ sitl_harness ]  <--UDP-->  FlyingSquirrel
       (autopilot)                   (this script)               (detector + RTL)

The harness is the single MAVLink hub:
  * It connects to SITL over TCP (SITL is a TCP *server* on :5760).
  * It binds/sends UDP to the detector (FlyingSquirrel's --mav-bind).
  * It forwards EVERY SITL->harness message out to the detector (so the
    detector sees real HEARTBEAT / GPS_RAW_INT / GLOBAL_POSITION_INT /
    SCALED_IMU traffic), AND every detector->harness message back into SITL
    (so FlyingSquirrel's RTL COMMAND_LONG + PARAM_SET actually reach and
    actuate the autopilot — the loop truly closes).
  * It ALSO originates flight choreography (arm/takeoff) and injects the GPS
    spoof via SITL's SIM_GPS* params.

Why a separate container: the SITL image has no pip and its vendored
pymavlink needs dialect generation; the FlyingSquirrel runtime image is
debian-slim with no Python. A tiny python:slim + `pip install pymavlink`
image (modern, pre-generated dialects) is the clean, reusable home for this.

Exit code 0 only if the detector-driven autopilot reaches an RTL-equivalent
mode (RTL/LAND/SMART_RTL) after the spoof — i.e. the whole chain worked.
"""
import argparse
import math
import random
import socket
import sys
import threading
import time

from pymavlink import mavutil

# ArduCopter custom_mode values.
MODE_NAMES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
              5: "LOITER", 6: "RTL", 9: "LAND", 16: "POSHOLD", 20: "GUIDED_NOGPS",
              21: "SMART_RTL"}
# ArduCopter flat custom_mode values that count as "no longer GPS-navigating".
ARDU_RTL_EQUIV = {6, 9, 21}  # RTL, LAND, SMART_RTL

# PX4 packs custom_mode as (sub_mode<<24)|(main_mode<<16) — main_mode in bits
# 16..23, sub_mode in bits 24..31 (the px4_custom_mode union). AUTO main=4; the
# return/land sub-modes are RTL=5, LAND=6, RTGS=7, DESCEND=20 — PX4's "no longer
# GPS-navigating" states. DESCEND is the position-LESS emergency descent PX4
# falls into when it can't navigate an RTL (observed live 2026-08-08 as
# 0x14040000 after the detector's GPS sever). VERIFIED against live PX4 SITL:
# AUTO.RTL = 0x05040000. Mirrors the detector's is_rtl_mode.
PX4_MAIN_AUTO = 4
PX4_AUTO_RTL_SUBS = {5, 6, 7, 20}


def is_rtl_equiv(vehicle, custom_mode):
    """True if `custom_mode` is an RTL-equivalent for the given vehicle —
    mirrors the detector's own `is_rtl_mode` so the harness's PASS criterion
    matches what the detector considers a successful RTL engagement."""
    if custom_mode is None:
        return False
    if vehicle == "px4":
        main = (custom_mode >> 16) & 0xFF
        sub = (custom_mode >> 24) & 0xFF
        return main == PX4_MAIN_AUTO and sub in PX4_AUTO_RTL_SUBS
    return custom_mode in ARDU_RTL_EQUIV


def log(msg):
    print("[harness] {}".format(msg), flush=True)


def mode_name(vehicle, custom_mode):
    """Human-readable mode for the periodic logs. ArduCopter uses the flat
    MODE_NAMES table; PX4 prints hex (its packed encoding never matches the
    ArduCopter table, which previously rendered every PX4 mode as '?')."""
    if custom_mode is None:
        return "?"
    if vehicle == "px4":
        return "0x{:08x}".format(custom_mode)
    return MODE_NAMES.get(custom_mode, "?")


class Relay:
    """Bidirectional MAVLink relay between SITL (TCP master) and the detector
    (UDP). Runs the SITL<->UDP pumping on a background thread so the main
    thread can choreograph the flight."""

    def __init__(self, sitl_url, det_host, det_port):
        log("connecting to SITL at {}".format(sitl_url))
        self.sitl = mavutil.mavlink_connection(sitl_url, retries=20, timeout=60)
        self.sitl.wait_heartbeat(timeout=90)
        log("SITL heartbeat: sys={} comp={}".format(
            self.sitl.target_system, self.sitl.target_component))

        # Raw UDP socket to the detector. We send raw MAVLink bytes to it and
        # receive the detector's outbound commands back. Detector binds
        # det_host:det_port; we connect so recv only sees its datagrams.
        #
        # CRITICAL (the bug that made the detector go silent on 10k msgs):
        # FlyingSquirrel's source-port LOCK requires inbound packets to arrive
        # FROM the same port it targets (det_port). `--mav-no-source-filter`
        # only disables the source-IP filter, NOT the port lock. So we MUST
        # send from det_port, not an ephemeral port, or every relayed packet
        # is silently dropped (dropped_source_locked) and preflight never
        # arms. Bind our source port = det_port. This also means the
        # detector's RTL replies (which it sends TO our src) land right back
        # on this same socket — closing the loop on one fd.
        self.det_addr = (det_host, det_port)
        self.usock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.usock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.usock.bind(("0.0.0.0", det_port))
        self.usock.setblocking(False)
        # Send an initial keepalive so the detector learns our return address
        # (it replies to whoever last sent from the expected source port).
        self.last_mode = None
        self.mode_changed_after = None
        self._stop = threading.Event()
        self._fwd_to_det = 0
        self._fwd_to_sitl = 0
        # Shared GPS state, written ONLY by the relay thread (the sole reader
        # of the SITL connection) and observed by the main choreography thread.
        # Two threads calling recv_match() on one mavutil connection steal
        # messages from each other — so the relay is the single reader and
        # everything else watches these fields.
        self.gps_fix_type = 0
        self.gps_sats = 0
        self.last_base_mode = None
        self.rel_alt_m = 0.0
        # Relay-side wire-level GPS spoof (meaconing model). When `spoof_active`,
        # the relay rewrites the lat/lon of every GPS_RAW_INT / GLOBAL_POSITION_INT
        # it forwards to the detector, adding these degree offsets. This is the
        # vehicle-agnostic injection path: it works identically for PX4 and
        # ArduPilot and doesn't depend on a simulator's GPS-glitch param (PX4
        # has no clean SIM_GPS_GLITCH equivalent). It models exactly the attack
        # the detector defends against — the GPS the autopilot/companion sees is
        # offset from truth — and exercises the full detection + RTL-dispatch
        # path on the real firmware under test.
        self.spoof_active = False
        self.spoof_dlat_deg = 0.0
        self.spoof_dlon_deg = 0.0
        # Receiver-noise model for the DETECTOR-bound GPS_RAW_INT copy only
        # (PX4's own GPS and GLOBAL_POSITION_INT are untouched). Gazebo's navsat
        # plugin is NOISELESS: while the vehicle hovers or climbs vertically its
        # lat/lon repeat BIT-IDENTICALLY fix after fix — which is precisely the
        # frozen/replay attack signature (detector FROZEN_FIX_RADIUS_M = 0.5 m),
        # so the detector correctly latched FrozenGps mid-climb and RTL'd before
        # the scripted spoof (observed live 2026-08-08). A real receiver always
        # jitters fix-to-fix (~1-2.5 m CEP); adding gaussian noise restores that
        # physical realism rather than desensitizing the detector. 0 = off
        # (ArduPilot SITL already models receiver noise).
        self.gps_jitter_m = 0.0
        # Smart-relay (consistent-velocity) extension. When `spoof_smart`, the
        # position offset RAMPS over `spoof_ramp_secs` instead of stepping, and a
        # MATCHING east velocity bias (`spoof_rate_e_mps`) is written into the
        # forwarded GPS so reported position and Doppler stay mutually consistent
        # — the faked-Doppler "smart spoofer" the velocity-aiding lane targets,
        # which neither the step relay nor param mode produces (param glitches the
        # raw receiver position only, leaving Doppler honest = a naive signature).
        self.spoof_smart = False
        self.spoof_start_t = 0.0
        self.spoof_ramp_secs = 60.0
        self.spoof_rate_e_mps = 0.0
        # Latest EKF_STATUS_REPORT (ArduPilot streams it once data streams are
        # requested). The param-mode evidence: whether EK3 FUSED the gradual
        # SIM_GPS_GLITCH ramp (GPS_GLITCHING clear, POS_HORIZ_ABS set) rather than
        # innovation-gating it, and whether the detector's GPS sever later makes
        # the EKF drop its absolute-position lane (POS_HORIZ_ABS clears /
        # CONST_POS_MODE sets).
        self.ekf_flags = None
        self.ekf_vel_var = None
        self.ekf_pos_horiz_var = None
        # Latest TRUE (un-spoofed) position from GLOBAL_POSITION_INT, tracked by
        # the relay thread BEFORE the spoof rewrite. Feeds the external-vision
        # fallback (PX4 ActionAcked): when `ev_active`, the relay streams
        # VISION_POSITION_ESTIMATE to PX4 at ~30 Hz so PX4 retains a NON-GPS
        # position source and can hold an armed RTL through the detector's verify
        # dwell AFTER the GPS sever (EKF2_GPS_CTRL=0 otherwise removes PX4's only
        # position -> LAND -> disarm -> the detector's armed cross-check correctly
        # reports ActionUnconfirmed). The EV stream goes ONLY harness->PX4; the
        # detector never sees it, so detection is unaffected.
        self.true_lat = None
        self.true_lon = None
        self.ev_active = False
        self.ev_origin = None  # (lat0, lon0) captured when EV is enabled
        self._last_ev_send = 0.0
        # GCS heartbeat — PX4 won't arm ("No connection to the GCS") without a
        # ground-station link. Enabled only on the PX4 path (ArduPilot unaffected).
        self.gcs_hb_active = False
        self._last_gcs_hb = 0.0
        # EKF-validity evidence for the pre-arm gate (PX4). PX4 publishes
        # LOCAL_POSITION_NED / GLOBAL_POSITION_INT only from the EKF output, and
        # broadcasts HOME_POSITION once the EKF's global position first becomes
        # valid; ESTIMATOR_STATUS carries the estimator's own health flags (the
        # PX4 counterpart of ArduPilot's EKF_STATUS_REPORT, which PX4 never
        # sends — waiting on EKF_STATUS_REPORT from PX4 waits forever).
        self.est_flags = None  # ESTIMATOR_STATUS.flags
        self.home_seen = False
        self.first_local_t = None  # LOCAL_POSITION_NED first/last arrival
        self.last_local_t = None
        self.first_gpos_t = None  # GLOBAL_POSITION_INT first/last arrival
        self.last_gpos_t = None
        # Latest COMMAND_ACK result per command id — lets the choreography see
        # WHY an arm was rejected instead of blindly retrying.
        self.cmd_acks = {}

    def _current_spoof_offset(self):
        """(dlat_deg, dlon_deg, dvel_e_mps) for this instant. The step relay
        returns the fixed offset with no velocity bias; the smart relay ramps the
        position offset and reports a consistent east velocity bias WHILE ramping
        (zero once it caps, where the position holds steady)."""
        if not self.spoof_smart:
            return (self.spoof_dlat_deg, self.spoof_dlon_deg, 0.0)
        frac = (time.time() - self.spoof_start_t) / max(self.spoof_ramp_secs, 0.1)
        if frac >= 1.0:
            return (self.spoof_dlat_deg, self.spoof_dlon_deg, 0.0)
        return (frac * self.spoof_dlat_deg, frac * self.spoof_dlon_deg,
                self.spoof_rate_e_mps)

    def _rewrite_velocity_east(self, msg, mt, dvel_e_mps):
        """Add `dvel_e_mps` (east, m/s) to the message's reported velocity so the
        Doppler stays CONSISTENT with the ramping position. GLOBAL_POSITION_INT
        carries NED cm/s (vx/vy/vz, vy=east); GPS_RAW_INT carries polar ground
        speed (`vel`, cm/s) + course (`cog`, cdeg), so we go via N/E components."""
        de_cms = dvel_e_mps * 100.0
        if mt == "GLOBAL_POSITION_INT":
            msg.vy = int(round(msg.vy + de_cms))
            return
        # GPS_RAW_INT — skip if the receiver didn't report velocity/course
        # (UINT16_MAX unknown sentinel); the detector would ignore it anyway.
        if msg.vel == 65535 or msg.cog == 65535:
            return
        cog_rad = math.radians(msg.cog / 100.0)
        vn = msg.vel * math.cos(cog_rad)
        ve = msg.vel * math.sin(cog_rad) + de_cms
        msg.vel = int(round(math.hypot(vn, ve)))
        msg.cog = int(round(math.degrees(math.atan2(ve, vn)) % 360.0 * 100.0))

    def _maybe_spoof_and_serialize(self, msg, mt):
        """Return the bytes to forward for `msg`. When the spoof is armed and
        `msg` carries position (GPS_RAW_INT / GLOBAL_POSITION_INT), rewrite its
        lat/lon (and, in smart mode, its velocity) and re-encode. Receiver
        jitter (`gps_jitter_m`) additionally perturbs the RAW-receiver message
        (GPS_RAW_INT — the message a real receiver's noise lives in) whether or
        not the spoof is active. Otherwise return the original bytes unchanged.

        lat/lon in both messages are int32 in 1e-7 degrees, so the offset is
        `round(deg * 1e7)`."""
        jitter_this = self.gps_jitter_m > 0.0 and mt == "GPS_RAW_INT"
        spoof_this = self.spoof_active and mt in ("GPS_RAW_INT", "GLOBAL_POSITION_INT")
        if not (jitter_this or spoof_this):
            return msg.get_msgbuf()
        try:
            if jitter_this:
                # Horizontal-only gaussian noise, sigma = gps_jitter_m per axis.
                # (m -> deg: 111320 m/deg latitude; scaled by cos(lat) for lon.)
                lat_now = msg.lat / 1e7
                coslat = max(math.cos(math.radians(lat_now)), 1e-6)
                msg.lat += int(round(random.gauss(0.0, self.gps_jitter_m) / 111320.0 * 1e7))
                msg.lon += int(round(
                    random.gauss(0.0, self.gps_jitter_m) / (111320.0 * coslat) * 1e7))
            if spoof_this:
                dlat_deg, dlon_deg, dvel_e_mps = self._current_spoof_offset()
                msg.lat += int(round(dlat_deg * 1e7))
                msg.lon += int(round(dlon_deg * 1e7))
                if dvel_e_mps != 0.0:
                    self._rewrite_velocity_east(msg, mt, dvel_e_mps)
            # Re-encode against the relay's own mav so the CRC is recomputed.
            # Use the original sender's sysid/compid so the detector's sysid
            # filter still accepts it (the autopilot is sysid 1).
            self.sitl.mav.srcSystem = msg.get_srcSystem()
            self.sitl.mav.srcComponent = msg.get_srcComponent()
            return msg.pack(self.sitl.mav)
        except Exception:
            # On any re-encode hiccup, fall back to forwarding the original so
            # the stream never stalls.
            return msg.get_msgbuf()

    def start(self):
        self._t = threading.Thread(target=self._pump, daemon=True)
        self._t.start()

    def stop(self):
        self._stop.set()

    def _pump(self):
        """Forward SITL->detector (raw bytes) and detector->SITL (raw bytes)."""
        sitl_fd = self.sitl.port  # underlying socket for select
        while not self._stop.is_set():
            # 1) Drain everything SITL has, forward to detector, track mode.
            while True:
                msg = self.sitl.recv_match(blocking=False)
                if msg is None:
                    break
                mt = msg.get_type()
                # Track GPS health from the TRUE (un-spoofed) message.
                if mt == "HEARTBEAT":
                    if self.last_mode is not None and msg.custom_mode != self.last_mode:
                        self.mode_changed_after = time.time()
                    self.last_mode = msg.custom_mode
                    self.last_base_mode = msg.base_mode
                elif mt == "GPS_RAW_INT":
                    self.gps_fix_type = msg.fix_type
                    self.gps_sats = msg.satellites_visible
                elif mt == "GLOBAL_POSITION_INT":
                    self.rel_alt_m = msg.relative_alt / 1000.0
                    # TRUE position — read BEFORE _maybe_spoof_and_serialize
                    # rewrites msg.lat/lon below, so the EV fallback feeds PX4
                    # its real position, not the spoofed one.
                    self.true_lat = msg.lat / 1e7
                    self.true_lon = msg.lon / 1e7
                    now_g = time.time()
                    if self.first_gpos_t is None:
                        self.first_gpos_t = now_g
                    self.last_gpos_t = now_g
                elif mt == "LOCAL_POSITION_NED":
                    now_l = time.time()
                    if self.first_local_t is None:
                        self.first_local_t = now_l
                    self.last_local_t = now_l
                elif mt == "ESTIMATOR_STATUS":
                    self.est_flags = msg.flags
                elif mt == "HOME_POSITION":
                    self.home_seen = True
                elif mt == "COMMAND_ACK":
                    self.cmd_acks[msg.command] = msg.result
                elif mt == "STATUSTEXT":
                    # Surface the autopilot's own words (arm denials, failsafe
                    # reasons, EKF resets) straight into harness.log — the
                    # artifact-side truth without grepping a giant px4.log.
                    try:
                        log("[autopilot sev={}] {}".format(msg.severity, msg.text))
                    except Exception:
                        pass
                elif mt == "EKF_STATUS_REPORT":
                    self.ekf_flags = msg.flags
                    self.ekf_vel_var = msg.velocity_variance
                    self.ekf_pos_horiz_var = msg.pos_horiz_variance
                # Forward to the detector — spoofing the position en route if armed.
                buf = self._maybe_spoof_and_serialize(msg, mt)
                if buf:
                    try:
                        self.usock.sendto(buf, self.det_addr)
                        self._fwd_to_det += 1
                    except OSError:
                        pass
            # 2) Drain detector->harness, forward into SITL.
            while True:
                try:
                    data, _ = self.usock.recvfrom(4096)
                except (BlockingIOError, OSError):
                    break
                if not data:
                    break
                try:
                    # Write raw bytes straight to SITL's MAVLink socket.
                    self.sitl.write(data)
                    self._fwd_to_sitl += 1
                except Exception:
                    pass
            # 3) External-vision fallback: stream VISION_POSITION_ESTIMATE to PX4
            #    at ~30 Hz from the TRUE position (NED relative to the EV origin),
            #    so PX4 has a non-GPS position source through the verify dwell.
            if self.ev_active and self.ev_origin is not None and self.true_lat is not None:
                now = time.time()
                if now - self._last_ev_send >= 1.0 / 30.0:
                    self._last_ev_send = now
                    self._send_ev()
            # 4) GCS heartbeat @ ~1 Hz — satisfies PX4's "connection to the GCS"
            #    arming/datalink check (nothing else here is a GCS).
            if self.gcs_hb_active:
                nowh = time.time()
                if nowh - self._last_gcs_hb >= 1.0:
                    self._last_gcs_hb = nowh
                    self._send_gcs_heartbeat()
            time.sleep(0.002)

    def _send_ev(self):
        """Send one VISION_POSITION_ESTIMATE (NED rel. to ev_origin) to PX4."""
        lat0, lon0 = self.ev_origin
        north = (self.true_lat - lat0) * 111320.0
        east = (self.true_lon - lon0) * 111320.0 * math.cos(math.radians(lat0))
        down = -self.rel_alt_m
        usec = int(time.time() * 1e6)
        # 21-element upper-triangular 6x6 pose covariance; small diagonals =
        # high confidence so the EKF fuses it. Diagonal indices: 0,6,11,15,18,20.
        cov = [0.0] * 21
        cov[0] = cov[6] = cov[11] = 0.01  # x,y,z position variance (m^2)
        cov[15] = cov[18] = cov[20] = 0.05  # roll,pitch,yaw variance
        # Send as a GCS/companion sysid, not the autopilot's, so PX4 treats it as
        # an external source. (Sequential within this thread with the spoof
        # re-encode, which sets srcSystem right before it packs.)
        self.sitl.mav.srcSystem = 255
        self.sitl.mav.srcComponent = 200
        # The VISION_POSITION_ESTIMATE covariance + reset_counter are MAVLink2
        # EXTENSION fields whose presence in the generated `_send` signature varies
        # by pymavlink version. Try the modern 9-arg form (with covariance), then
        # fall back to the 7-arg form — and swallow EVERYTHING, because an
        # unhandled exception here would kill the relay pump thread and stop ALL
        # forwarding. (EKF2_EV_NOISE_MD=0 + EKF2_EVP_NOISE makes the EKF use a
        # param-based noise, so the message covariance isn't load-bearing anyway.)
        for send in (
            lambda: self.sitl.mav.vision_position_estimate_send(
                usec, north, east, down, 0.0, 0.0, 0.0, cov, 0),
            lambda: self.sitl.mav.vision_position_estimate_send(
                usec, north, east, down, 0.0, 0.0, 0.0),
        ):
            try:
                send()
                return
            except Exception:
                continue

    def _send_gcs_heartbeat(self):
        """One MAV_TYPE_GCS heartbeat so PX4's datalink/GCS arming check passes.
        Sent from a GCS sysid (255), distinct from the autopilot and the vision
        source; srcSystem is reset per-message by the relay/EV packers, same as
        _send_ev. Swallow-all so a transient send error can't kill the pump."""
        self.sitl.mav.srcSystem = 255
        self.sitl.mav.srcComponent = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
        try:
            self.sitl.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
        except Exception:
            pass

    # --- choreography helpers (run on main thread) ---
    def wait_gps(self, timeout=180):
        # Observe the relay thread's shared GPS state — do NOT call recv_match
        # here (that would steal messages from the relay, the sole reader).
        log("waiting for GPS 3D fix (observing relay state)...")
        end = time.time() + timeout
        last_report = 0
        while time.time() < end:
            if self.gps_fix_type >= 3 and self.gps_sats >= 6:
                log("GPS ready: fix_type={} sats={}".format(self.gps_fix_type, self.gps_sats))
                return True
            now = time.time()
            if now - last_report >= 10:
                log("  ...still waiting (fix_type={} sats={})".format(
                    self.gps_fix_type, self.gps_sats))
                last_report = now
            time.sleep(0.5)
        return False

    def is_armed(self):
        return bool((self.last_base_mode or 0) & 0x80)

    def wait_airborne(self, min_alt_m, timeout=40):
        log("waiting for takeoff (rel_alt >= {:.0f} m)...".format(min_alt_m))
        end = time.time() + timeout
        while time.time() < end:
            if self.rel_alt_m >= min_alt_m:
                log("airborne: rel_alt={:.1f} m".format(self.rel_alt_m))
                return True
            time.sleep(0.5)
        log("WARN: takeoff not confirmed (rel_alt={:.1f} m) — a later RTL may disarm "
            "before the detector's read-back dwell completes".format(self.rel_alt_m))
        return False

    def set_param(self, name, value, param_type=None):
        if param_type is None:
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        self.sitl.mav.param_set_send(
            self.sitl.target_system, self.sitl.target_component,
            name.encode("ascii"), float(value), param_type)

    def set_param_int(self, name, value):
        """Set an INT32 param. PX4 rejects an int param sent as REAL32 with a
        'param types mismatch' (observed live on EKF2_EV_NOISE_MD)."""
        self.set_param(name, value, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

    def enable_ev_fallback(self):
        """Capture the current true position as the EV origin and begin streaming
        VISION_POSITION_ESTIMATE (see _pump / _send_ev). The caller sets the
        EKF2 params that make PX4 fuse it. Returns False if no position is known
        yet (the EV origin couldn't be captured)."""
        if self.true_lat is None:
            log("WARN: no GLOBAL_POSITION_INT seen yet — EV fallback NOT started")
            return False
        self.ev_origin = (self.true_lat, self.true_lon)
        self.ev_active = True
        log("EV fallback ON: streaming VISION_POSITION_ESTIMATE @30Hz from origin "
            "({:.7f},{:.7f})".format(self.true_lat, self.true_lon))
        return True

    def ekf_summary(self):
        """Human-readable estimator snapshot. ArduPilot streams EKF_STATUS_REPORT;
        PX4 streams ESTIMATOR_STATUS instead (it NEVER sends EKF_STATUS_REPORT —
        the old code waited on it forever). Both enums share the low bits:
        POS_HORIZ_ABS=16 (absolute horizontal position), CONST_POS_MODE=128 (no
        position aiding, set when GPS is dropped). ArduPilot GPS_GLITCHING=32768;
        PX4 GPS_GLITCH=1024. For a fused gradual ramp expect pos_horiz_abs=True +
        glitching=False; after the sever expect pos_horiz_abs to clear (or, with
        the EV fallback fused, HOLD) / const_pos_mode to set."""
        if self.est_flags is not None:
            f = self.est_flags
            return (
                "EST flags=0x{:04x} pos_horiz_abs={} pos_vert_abs={} "
                "const_pos_mode={} gps_glitch={}".format(
                    f, bool(f & 16), bool(f & 32), bool(f & 128), bool(f & 1024)))
        if self.ekf_flags is None:
            return "EKF/ESTIMATOR_STATUS not yet seen"
        f = self.ekf_flags
        return (
            "EKF flags=0x{:04x} pos_horiz_abs={} const_pos_mode={} gps_glitching={} "
            "| vel_var={:.2f} pos_horiz_var={:.2f}".format(
                f, bool(f & 16), bool(f & 128), bool(f & 32768),
                self.ekf_vel_var or 0.0, self.ekf_pos_horiz_var or 0.0))

    def wait_px4_ekf_ready(self, timeout=150, stable_secs=10.0):
        """Block until PX4's EKF plausibly has a valid absolute position — the
        health gate the blind-land runs jumped: force-arming past 'ekf2 missing
        data' let AUTO.TAKEOFF start position control with no valid local
        position, so mc_pos_control got 'invalid setpoints' and failsafe'd into
        a blind land seconds after 'Takeoff detected'. Ready means ANY of:
          * ESTIMATOR_STATUS reports POS_HORIZ_ABS(16) + POS_VERT_ABS(32), or
          * PX4 broadcast HOME_POSITION (it does so exactly when the EKF global
            position first becomes valid), or
          * LOCAL_POSITION_NED AND GLOBAL_POSITION_INT (both EKF outputs) have
            been streaming steadily for `stable_secs` and are fresh (<3 s old)
            — the fallback when neither status message is streamed.
        The timeout is generous: under CI lockstep the sim clock can run well
        below wall time, stretching EKF convergence in wall-clock terms."""
        log("PX4: waiting for a valid EKF position before arming...")
        end = time.time() + timeout
        last_report = 0.0
        while time.time() < end:
            now = time.time()
            if self.est_flags is not None and (self.est_flags & 16) and (self.est_flags & 32):
                log("PX4 EKF ready: {}".format(self.ekf_summary()))
                return True
            if self.home_seen:
                log("PX4 EKF ready: HOME_POSITION broadcast (global position valid)")
                return True
            if (self.first_local_t is not None and self.first_gpos_t is not None
                    and now - max(self.first_local_t, self.first_gpos_t) >= stable_secs
                    and now - self.last_local_t < 3.0 and now - self.last_gpos_t < 3.0):
                log("PX4 EKF ready: LOCAL_POSITION_NED + GLOBAL_POSITION_INT "
                    "streaming steadily for {:.0f}s".format(
                        now - max(self.first_local_t, self.first_gpos_t)))
                return True
            if now - last_report >= 10:
                log("  ...EKF not ready yet ({} | local_pos={} gpos={} home={})".format(
                    self.ekf_summary(),
                    self.first_local_t is not None, self.first_gpos_t is not None,
                    self.home_seen))
                last_report = now
            time.sleep(0.5)
        log("WARN: EKF readiness NOT confirmed within {}s — arming anyway "
            "(expect a possible blind-land failsafe)".format(timeout))
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sitl", default="tcp:sitl:5760")
    ap.add_argument("--det-host", default="flyingsquirrel")
    ap.add_argument("--det-port", type=int, default=14551)
    ap.add_argument("--takeoff-alt", type=float, default=30.0)
    ap.add_argument("--clean-secs", type=float, default=25.0)
    ap.add_argument("--glitch-m", type=float, default=400.0)
    ap.add_argument("--watch-secs", type=float, default=45.0)
    # After the autopilot reaches RTL, keep the relay running this long so the
    # detector's RTL read-back DWELL (several fresh HEARTBEATs in RTL) completes
    # and it logs ActionAcked — those heartbeats only reach it through the relay.
    ap.add_argument("--verify-grace-secs", type=float, default=15.0)
    # Home latitude — used to convert the desired east glitch (metres) to
    # degrees of longitude. Defaults to CMAC (Canberra), ArduPilot's default
    # SITL home; PX4's default is Zurich (47.397742) — pass --home-lat.
    ap.add_argument("--home-lat", type=float, default=-35.363261)
    # Spoof injection mechanism:
    #   relay       — step-rewrite GPS lat/lon in the forwarded stream (naive
    #                 position jump; vehicle-agnostic, the only path for PX4).
    #   relay-smart — RAMP lat/lon AND rewrite Doppler consistently, so the
    #                 detector sees a faked-Doppler "smart spoofer" — the
    #                 consistent-velocity signature the velocity-aiding lane
    #                 targets (Phase 2c).
    #   param       — spoof ArduPilot's simulated GPS via a gradual SIM_GPS_GLITCH
    #                 ramp the EKF fuses (proves EKF-fusion + sever-drops-lane).
    ap.add_argument("--spoof-mode", choices=["relay", "relay-smart", "param"], default="relay")
    # PARAM-MODE ramp duration. A STEP SIM_GPS_GLITCH is innovation-gated by
    # ArduPilot's EK3 (rejected — proves nothing); ramping it gradually over this
    # many seconds keeps each epoch's position jump under the gate so the EKF
    # FUSES it — the realistic "smart spoofer" / EKF-laundered ramp whose fused
    # output the detector's velocity-aiding lane must catch. Ignored in relay mode.
    ap.add_argument("--glitch-ramp-secs", type=float, default=60.0)
    # Autopilot family — affects only the GUIDED-mode number used to arm.
    # ArduCopter GUIDED = custom_mode 4; PX4 doesn't use that path (we just
    # arm + let it hold), so the value is harmless there.
    ap.add_argument("--vehicle", choices=["ardu-copter", "px4"], default="ardu-copter")
    # PX4 ActionAcked fallback (experimental, opt-in). When set on a PX4 run, the
    # harness configures EKF2 to fuse external vision and streams a true-position
    # VISION_POSITION_ESTIMATE, so PX4 keeps a NON-GPS position estimate after the
    # detector severs EKF2_GPS_CTRL=0 and can hold an armed RTL through the verify
    # dwell -> the detector's STRONG ActionAcked read-back (vs ActionUnconfirmed
    # when PX4 LANDs+disarms for lack of position). The detector is unchanged.
    ap.add_argument("--px4-ev-fallback", action="store_true",
                    help="PX4 only: inject external-vision position so PX4 holds "
                         "armed RTL after the GPS sever (enables ActionAcked).")
    # Receiver-noise model (metres, 1-sigma per horizontal axis) applied to the
    # DETECTOR-bound GPS_RAW_INT stream. Needed for simulators whose GPS is
    # noiseless (gz navsat): bit-identical fixes while the vehicle climbs are
    # indistinguishable from a frozen/replay attack, so the detector (correctly)
    # latched FrozenGps mid-takeoff. Real receivers always jitter; 0 = off.
    ap.add_argument("--gps-jitter-m", type=float, default=0.0)
    args = ap.parse_args()

    # Param mode needs the watch window to outlast the ramp plus a detection
    # margin, or the loop tears down before the EKF-fused offset grows enough to
    # detect. Auto-extend so callers don't have to keep --watch-secs in sync with
    # --glitch-ramp-secs.
    if args.spoof_mode in ("param", "relay-smart"):
        needed = args.glitch_ramp_secs + 45.0
        if args.watch_secs < needed:
            log("{} mode: extending --watch-secs {:.0f} -> {:.0f} to cover the "
                "ramp + detection".format(args.spoof_mode, args.watch_secs, needed))
            args.watch_secs = needed

    r = Relay(args.sitl, args.det_host, args.det_port)
    if args.gps_jitter_m > 0.0:
        r.gps_jitter_m = args.gps_jitter_m
        log("GPS receiver-noise model ON: {:.2f} m 1-sigma/axis on the "
            "detector-bound GPS_RAW_INT".format(args.gps_jitter_m))
    r.start()
    log("relay started (SITL<->detector both directions)")

    is_px4 = args.vehicle == "px4"

    # ArduPilot SITL does not stream GPS_RAW_INT / SCALED_IMU until a GCS requests
    # them (legacy REQUEST_DATA_STREAM, still honored by ArduPilot). PX4 streams
    # the offboard set (HIGHRES_IMU + GPS_RAW_INT) on 14540 by DEFAULT — the
    # bring-up smoke confirmed the detector reaches preflight Ready without a
    # request — so skip it for PX4.
    if not is_px4:
        sysid0, compid0 = r.sitl.target_system, r.sitl.target_component
        for _ in range(3):
            r.sitl.mav.request_data_stream_send(
                sysid0, compid0,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)  # 10 Hz, start
            time.sleep(1)
        log("requested all MAVLink data streams @10Hz")

    if not r.wait_gps():
        log("FAIL: no GPS fix (SITL never reported a 3D fix — see notes in docs/sitl.md)")
        return 2

    sysid, compid = r.sitl.target_system, r.sitl.target_component

    # Arm + take off. SEND-ONLY: the relay thread is the sole reader of the SITL
    # connection, so we must not call any recv-blocking helper (it would deadlock
    # against the relay) — we send raw commands, pace with sleeps, and observe
    # arm/mode/alt via the relay state. The sequence is vehicle-specific.
    if is_px4:
        # PX4 refuses to arm ("Preflight Fail: No connection to the GCS") until it
        # sees a ground-station link, and it has no ARMING_CHECK bypass param. So:
        #   (1) stream a MAV_TYPE_GCS heartbeat (see _send_gcs_heartbeat),
        #   (2) disable the data-link-loss failsafe (NAV_DLL_ACT=0) so the check is
        #       skipped AND a heartbeat hiccup can't disarm us mid-flight,
        #   (3) relax RC-loss arming (COM_RCL_EXCEPT bit2=4) and disable stick
        #       input entirely (COM_RC_IN_MODE=4) — there is no RC in this rig.
        # All SITL-only relaxations, never a real vehicle. INT32 params are typed
        # INT32 (PX4 rejects an int param sent as REAL32). COM_TKO_ALT is NOT set:
        # live SITL logged "unknown param: COM_TKO_ALT" on this firmware — the
        # AUTO.TAKEOFF climb altitude actually comes from MIS_TAKEOFF_ALT
        # (navigator logged "Using default takeoff altitude: 30.0 m").
        r.gcs_hb_active = True
        log("PX4: GCS heartbeat ON + NAV_DLL_ACT=0 + COM_RCL_EXCEPT=4 + takeoff alt {} m".format(
            args.takeoff_alt))
        for _ in range(3):
            r.set_param_int("NAV_DLL_ACT", 0)
            r.set_param_int("COM_RCL_EXCEPT", 4)
            r.set_param_int("COM_RC_IN_MODE", 4)
            r.set_param("MIS_TAKEOFF_ALT", float(args.takeoff_alt))
            # Don't let PX4 auto-disarm the grounded vehicle before it climbs
            # (COM_DISARM_PRFLT) or the instant an RTL touches down mid-dwell
            # (COM_DISARM_LAND). 0 disables each. Both are seconds (REAL32).
            r.set_param("COM_DISARM_PRFLT", 0)
            r.set_param("COM_DISARM_LAND", 0)
            # The gazebo SITL battery drains to CRITICAL within a few minutes and
            # its low-battery failsafe force-lands mid-climb ("blind land"). Set
            # the action to Warning-only (0) so it can't abort the flight. INT32.
            r.set_param_int("COM_LOW_BAT_ACT", 0)
            time.sleep(0.5)
        # Ask PX4 to stream the EKF-health evidence: ESTIMATOR_STATUS (230) and
        # HOME_POSITION (242) @1 Hz. PX4 does NOT send ArduPilot's
        # EKF_STATUS_REPORT, so without this the harness is blind to the
        # estimator (every prior run logged "EKF_STATUS not yet seen").
        for mid in (230, 242):
            for _ in range(3):
                r.sitl.mav.command_long_send(
                    sysid, compid, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0, mid, 1e6, 0, 0, 0, 0, 0)
                time.sleep(0.3)
        time.sleep(3)  # let params apply / GCS link register
        # THE core fix for the blind-land failure chain: do NOT force-arm past an
        # EKF that has no valid position. Wait for genuine EKF validity first —
        # arming into AUTO.TAKEOFF without a local position made mc_pos_control
        # emit 'invalid setpoints' and blind-land seconds after liftoff.
        r.wait_px4_ekf_ready()
        # Arm NORMALLY first (param2=0) — a normal arm is only ACCEPTED when
        # PX4's own health checks pass, so acceptance re-verifies EKF readiness.
        # STATUSTEXT (relayed into this log) says exactly why a denial happened.
        RESULT_NAMES = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
                        3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS", 6: "CANCELLED"}
        ARM_CMD = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        log("PX4 arming (normal COMPONENT_ARM_DISARM, health-checked)")
        arm_deadline = time.time() + 45
        last_ack = None
        while time.time() < arm_deadline and not r.is_armed():
            r.sitl.mav.command_long_send(
                sysid, compid, ARM_CMD, 0, 1, 0, 0, 0, 0, 0, 0)
            time.sleep(1.5)
            ack = r.cmd_acks.get(ARM_CMD)
            if ack is not None and ack != last_ack:
                log("  arm COMMAND_ACK: {}".format(RESULT_NAMES.get(ack, ack)))
                last_ack = ack
        if not r.is_armed():
            # Backstop only. If health never converges, fall back to the force
            # magic so the run still produces diagnostics downstream.
            log("PX4 normal arm not accepted; falling back to force-arm (21196)")
            arm_deadline = time.time() + 20
            while time.time() < arm_deadline and not r.is_armed():
                r.sitl.mav.command_long_send(
                    sysid, compid, ARM_CMD, 0, 1, 21196, 0, 0, 0, 0, 0)
                time.sleep(1.5)
        log("PX4 armed={} ; AUTO.TAKEOFF mode + NAV_TAKEOFF to {} m (rel via "
            "MIS_TAKEOFF_ALT)".format(r.is_armed(), args.takeoff_alt))
        for _ in range(4):
            # Command AUTO.TAKEOFF mode explicitly (main=4/sub=2): PX4 climbs to
            # MIS_TAKEOFF_ALT, which is RELATIVE — sidestepping NAV_TAKEOFF's
            # param7 AMSL-vs-relative ambiguity (30 read as AMSL is far below
            # Zurich's ~488 m ground, so the vehicle never climbed).
            r.sitl.mav.command_long_send(
                sysid, compid, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 2, 0, 0, 0, 0)
            # NAV_TAKEOFF as well (belt-and-suspenders).
            r.sitl.mav.command_long_send(
                sysid, compid, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, args.takeoff_alt)
            time.sleep(1.5)
    else:
        # ArduCopter: relax pre-arm checks + DO_SET_HOME, GUIDED, force-arm,
        # takeoff. W4/W-ARMED: with only ARMING_CHECK=0 and a plain arm the bare
        # copter often stayed disarmed on the ground — then a later RTL disarms
        # instantly and the detector (correctly) reports it RTL-unconfirmed.
        # ARMING_CHECK=0 is the SITL scripted-test standard (NEVER a real vehicle).
        log("ARMING_CHECK=0 + DO_SET_HOME(current) for SITL scripted arm")
        for _ in range(3):
            r.set_param("ARMING_CHECK", 0)
            time.sleep(0.5)
        # MAV_CMD_DO_SET_HOME param1=1 -> use current location as home/origin.
        r.sitl.mav.command_long_send(sysid, compid, mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                                     0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(8)  # let the EKF settle / set its origin with checks relaxed
        log("mode GUIDED (custom_mode=4)")
        r.sitl.mav.set_mode_send(
            sysid, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
        time.sleep(3)
        # Force-arm (param2 = 21196 magic) and CONFIRM via the HEARTBEAT armed bit
        # before proceeding — a plain arm silently failed.
        log("force-arming (COMPONENT_ARM_DISARM param1=1 param2=21196)")
        arm_deadline = time.time() + 30
        while time.time() < arm_deadline and not r.is_armed():
            r.sitl.mav.command_long_send(
                sysid, compid, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21196, 0, 0, 0, 0, 0)
            time.sleep(1.5)
        log("armed={} ; takeoff to {} m".format(r.is_armed(), args.takeoff_alt))
        for _ in range(3):
            r.sitl.mav.command_long_send(
                sysid, compid, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, args.takeoff_alt)
            time.sleep(1)
    # Confirm airborne so a later RTL keeps the vehicle armed through the
    # detector's read-back dwell (descending from altitude takes >> the 5 s
    # verify window, so it stays armed long enough to confirm ActionAcked).
    # PX4 gets a longer window: under CI lockstep the sim clock runs below wall
    # time, stretching the climb in wall-clock terms.
    r.wait_airborne(args.takeoff_alt * 0.5, timeout=90 if is_px4 else 40)

    # PX4 ActionAcked fallback: give PX4 a non-GPS position source so it can hold
    # an armed RTL after the detector severs GPS (otherwise PX4 loses position ->
    # LANDs+disarms -> the detector's armed cross-check correctly returns
    # ActionUnconfirmed, the C-15 trade-off). Configure EKF2 to fuse external
    # vision, then stream a true-position VISION_POSITION_ESTIMATE (the relay
    # thread, _send_ev). Opt-in + PX4-only; the detector is untouched.
    if is_px4 and args.px4_ev_fallback:
        # EKF2_EV_CTRL is the modern (>=1.13) external-vision bitmask:
        #   bit0(1)=horizontal position, bit1(2)=vertical position,
        #   bit2(4)=velocity, bit3(8)=yaw. 11 = horiz+vert pos + yaw.
        # (The pre-1.13 EKF2_AID_MASK is intentionally NOT set — live SITL logged
        # "unknown param: EKF2_AID_MASK", i.e. this firmware is >=1.13, matching
        # the EKF2_GPS_CTRL sever path. Setting it only spammed mavlink errors.)
        log("PX4 EV fallback: EKF2_EV_CTRL=11 (fuse external vision)")
        for _ in range(3):
            r.set_param_int("EKF2_EV_CTRL", 11)
            r.set_param_int("EKF2_EV_NOISE_MD", 0)  # param noise below, not msg cov [INT32]
            r.set_param("EKF2_EVP_NOISE", 0.1)  # 0.1 m EV position noise [REAL32]
            time.sleep(0.5)
        if r.enable_ev_fallback():
            # Let the EKF fuse EV alongside GPS for a few seconds before the
            # spoof, so the EV lane is already converged when GPS is severed.
            log("EV fallback: letting EKF converge on vision for 8 s before the spoof")
            time.sleep(8)

    log("clean flight window {} s (detector should stay Normal, preflight Ready)".format(
        args.clean_secs))
    t_end = time.time() + args.clean_secs
    while time.time() < t_end:
        time.sleep(1)
    mode_before = r.last_mode
    log("mode before spoof = {} ({})".format(mode_before, mode_name(args.vehicle, mode_before)))

    # Inject the spoof. Convert the desired east offset (metres) to degrees of
    # longitude at the home latitude: dlon_deg = metres / (111320 * cos(lat)).
    import math
    lat_rad = math.radians(args.home_lat)
    dlon_deg = args.glitch_m / (111320.0 * max(math.cos(lat_rad), 1e-6))

    param_mode = args.spoof_mode == "param"
    if args.spoof_mode == "relay":
        # Wire-level meaconing: the relay rewrites the lat/lon of every GPS
        # message it forwards to the detector. Vehicle-agnostic — works
        # identically for PX4 and ArduPilot, no simulator GPS-glitch param
        # needed (PX4 has no clean equivalent). Models the attack the detector
        # exists to catch: the position the companion sees is offset from truth.
        # Applied as a step here, Doppler left honest (a naive signature).
        log("INJECTING SPOOF (relay wire-level, step): +{:.6f} deg lon (~{} m east at lat {})".format(
            dlon_deg, args.glitch_m, args.home_lat))
        r.spoof_dlat_deg = 0.0
        r.spoof_dlon_deg = dlon_deg
        r.spoof_active = True
    elif args.spoof_mode == "relay-smart":
        # Consistent-velocity wire spoof (Phase 2c): the relay RAMPS the lat/lon
        # offset AND rewrites the forwarded GPS Doppler to match, so the detector
        # (which reads GPS_RAW_INT) sees a position+velocity-CONSISTENT walk-off —
        # the faked-Doppler "smart spoofer" / EKF-laundered signature the
        # velocity-aiding lane (Phase 2a) targets. The position blend tracks the
        # spoofed velocity and masks the position residual, so this should be
        # caught by the velocity-aiding lane, not the position lane. The ramp runs
        # in the relay thread over --glitch-ramp-secs.
        rate_e = args.glitch_m / max(args.glitch_ramp_secs, 0.1)
        log("INJECTING SPOOF (relay-smart consistent-velocity): ramp +{:.6f} deg lon "
            "(~{} m east) over {:.0f}s = {:.2f} m/s east, with matching Doppler".format(
                dlon_deg, args.glitch_m, args.glitch_ramp_secs, rate_e))
        r.spoof_dlat_deg = 0.0
        r.spoof_dlon_deg = dlon_deg
        r.spoof_rate_e_mps = rate_e
        r.spoof_ramp_secs = args.glitch_ramp_secs
        r.spoof_start_t = time.time()
        r.spoof_smart = True
        r.spoof_active = True
    else:
        # SITL-param injection (ArduPilot only): bias the autopilot's OWN
        # simulated GPS receiver via SIM_GPS_GLITCH_Y (DEGREES; verified against
        # ArduPilot SIM_GPS.cpp). A STEP is innovation-gated by EK3 (rejected),
        # so we RAMP it in the watch loop below over --glitch-ramp-secs: each
        # epoch's jump stays under the gate and the EKF FUSES the offset. This
        # proves the part the relay/wire path does NOT: the autopilot's OWN
        # estimator was on the spoofed GPS (EKF_STATUS gps_glitching stays clear,
        # pos_horiz_var grows) AND the detector's GPS sever makes EK3 drop its
        # absolute-position lane (pos_horiz_abs clears / const_pos_mode sets) —
        # i.e. GPS_TYPE=0 actually STOPS the aiding. NOTE: the detector reads the
        # RAW receiver (GPS_RAW_INT), which SIM_GPS_GLITCH offsets in position
        # only (honest Doppler), so detection is via the position lane (a naive
        # signature). Exercising the velocity-aiding lane end-to-end would need a
        # consistent-velocity WIRE spoof (a "smart relay" that fakes Doppler too).
        log("INJECTING SPOOF (SITL param, gradual ramp over {:.0f}s to ~{} m east). "
            "Pre-injection {}".format(args.glitch_ramp_secs, args.glitch_m, r.ekf_summary()))

    log("watching {} s for detector-driven RTL ({})...".format(args.watch_secs, args.vehicle))
    spoof_t = time.time()
    end = spoof_t + args.watch_secs
    grace = args.verify_grace_secs
    reached_rtl = False
    rtl_at = None
    glitch_full = False
    last_ekf_log = 0.0
    # After the autopilot reaches RTL, KEEP the relay running for `grace` more
    # seconds. The detector confirms RTL via a read-back DWELL (several fresh
    # HEARTBEATs observed in RTL mode), and those heartbeats only reach it
    # THROUGH this relay. The old code tore down on the first RTL heartbeat,
    # starving the dwell, so the detector never emitted ActionAcked.
    while True:
        now = time.time()
        # PARAM-MODE: ramp SIM_GPS_GLITCH_Y so EK3 fuses it instead of gating it.
        # Set both the flat (this image) and instance (newer master) param names;
        # the nonexistent one is ignored.
        if param_mode and not glitch_full:
            frac = (now - spoof_t) / max(args.glitch_ramp_secs, 0.1)
            if frac >= 1.0:
                frac = 1.0
                glitch_full = True
            target = frac * dlon_deg
            for name in ("SIM_GPS_GLITCH_Y", "SIM_GPS1_GLTCH_Y"):
                r.set_param(name, target)
        # Periodic EKF-status evidence: did the EKF FUSE the ramp (pos_horiz_abs
        # set, gps_glitching clear)? After the detector severs GPS, does the EKF
        # drop its absolute-position lane (pos_horiz_abs clears)?
        if now - last_ekf_log >= 5.0:
            log("  t+{:.0f}s mode={} {}".format(
                now - spoof_t, mode_name(args.vehicle, r.last_mode), r.ekf_summary()))
            last_ekf_log = now
        if rtl_at is None and is_rtl_equiv(args.vehicle, r.last_mode):
            rtl_at = now
            reached_rtl = True
            log("autopilot reached RTL-equivalent; holding the relay {:.0f}s so the "
                "detector's RTL read-back dwell completes (ActionAcked). {}".format(
                    grace, r.ekf_summary()))
        if rtl_at is not None and now - rtl_at >= grace:
            break
        if rtl_at is None and now >= end:
            break
        time.sleep(0.5)

    final = r.last_mode
    log("final mode = {} ({}) | {}".format(
        final, mode_name(args.vehicle, final), r.ekf_summary()))
    log("fwd SITL->det={} det->SITL={}".format(r._fwd_to_det, r._fwd_to_sitl))
    r.stop()

    if reached_rtl:
        log("PASS: autopilot reached RTL-equivalent mode after spoof (closed loop worked)")
        return 0
    log("RESULT: autopilot did NOT reach RTL within window. "
        "Detector may have fired (check its log); the autopilot response is the open question.")
    # Not necessarily a harness failure — the detector log is the source of
    # truth for whether detection fired. Return 3 to distinguish from setup
    # failures (2).
    return 3


if __name__ == "__main__":
    sys.exit(main())
