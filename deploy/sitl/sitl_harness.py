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

# PX4 packs custom_mode as (main_mode<<24)|(sub_mode<<16). AUTO main=4; the
# return/land sub-modes are RTL=5, LAND=6, RTGS=7 — these are PX4's
# "no longer GPS-navigating" states, matching the detector's is_rtl_mode(px4).
PX4_MAIN_AUTO = 4
PX4_AUTO_RTL_SUBS = {5, 6, 7}


def is_rtl_equiv(vehicle, custom_mode):
    """True if `custom_mode` is an RTL-equivalent for the given vehicle —
    mirrors the detector's own `is_rtl_mode` so the harness's PASS criterion
    matches what the detector considers a successful RTL engagement."""
    if custom_mode is None:
        return False
    if vehicle == "px4":
        main = (custom_mode >> 24) & 0xFF
        sub = (custom_mode >> 16) & 0xFF
        return main == PX4_MAIN_AUTO and sub in PX4_AUTO_RTL_SUBS
    return custom_mode in ARDU_RTL_EQUIV


def log(msg):
    print("[harness] {}".format(msg), flush=True)


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
        # Latest EKF_STATUS_REPORT (ArduPilot streams it once data streams are
        # requested). The param-mode evidence: whether EK3 FUSED the gradual
        # SIM_GPS_GLITCH ramp (GPS_GLITCHING clear, POS_HORIZ_ABS set) rather than
        # innovation-gating it, and whether the detector's GPS sever later makes
        # the EKF drop its absolute-position lane (POS_HORIZ_ABS clears /
        # CONST_POS_MODE sets).
        self.ekf_flags = None
        self.ekf_vel_var = None
        self.ekf_pos_horiz_var = None

    def _maybe_spoof_and_serialize(self, msg, mt):
        """Return the bytes to forward for `msg`. When the spoof is armed and
        `msg` carries position (GPS_RAW_INT / GLOBAL_POSITION_INT), rewrite its
        lat/lon by the configured degree offset and re-encode; otherwise return
        the original raw bytes unchanged.

        lat/lon in both messages are int32 in 1e-7 degrees, so the offset is
        `round(deg * 1e7)`."""
        if not self.spoof_active or mt not in ("GPS_RAW_INT", "GLOBAL_POSITION_INT"):
            return msg.get_msgbuf()
        dlat = int(round(self.spoof_dlat_deg * 1e7))
        dlon = int(round(self.spoof_dlon_deg * 1e7))
        try:
            msg.lat += dlat
            msg.lon += dlon
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
            time.sleep(0.002)

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

    def set_param(self, name, value):
        self.sitl.mav.param_set_send(
            self.sitl.target_system, self.sitl.target_component,
            name.encode("ascii"), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def ekf_summary(self):
        """Human-readable EKF_STATUS_REPORT snapshot. Flag bits are the stable
        MAVLink EKF_STATUS_FLAGS enum: POS_HORIZ_ABS=16 (absolute horizontal
        position — GPS-aided), CONST_POS_MODE=128 (no position aiding, set when
        GPS is dropped), GPS_GLITCHING=32768 (innovation gate tripped — a jump
        the EKF REJECTED rather than fused). For a fused gradual ramp expect
        pos_horiz_abs=True + gps_glitching=False; after the sever expect
        pos_horiz_abs to clear / const_pos_mode to set."""
        if self.ekf_flags is None:
            return "EKF_STATUS not yet seen"
        f = self.ekf_flags
        return (
            "EKF flags=0x{:04x} pos_horiz_abs={} const_pos_mode={} gps_glitching={} "
            "| vel_var={:.2f} pos_horiz_var={:.2f}".format(
                f, bool(f & 16), bool(f & 128), bool(f & 32768),
                self.ekf_vel_var or 0.0, self.ekf_pos_horiz_var or 0.0))


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
    # Spoof injection mechanism: `relay` rewrites GPS lat/lon in the forwarded
    # stream (vehicle-agnostic; the only path that works for PX4). `param`
    # spoofs ArduPilot's simulated GPS via SIM_GPS_GLITCH_Y.
    ap.add_argument("--spoof-mode", choices=["relay", "param"], default="relay")
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
    args = ap.parse_args()

    # Param mode needs the watch window to outlast the ramp plus a detection
    # margin, or the loop tears down before the EKF-fused offset grows enough to
    # detect. Auto-extend so callers don't have to keep --watch-secs in sync with
    # --glitch-ramp-secs.
    if args.spoof_mode == "param":
        needed = args.glitch_ramp_secs + 45.0
        if args.watch_secs < needed:
            log("param mode: extending --watch-secs {:.0f} -> {:.0f} to cover the "
                "ramp + detection".format(args.watch_secs, needed))
            args.watch_secs = needed

    r = Relay(args.sitl, args.det_host, args.det_port)
    r.start()
    log("relay started (SITL<->detector both directions)")

    # ArduPilot SITL does not stream GPS_RAW_INT / SCALED_IMU at a useful rate
    # until a GCS requests them. Ask for ALL streams at 10 Hz (legacy
    # REQUEST_DATA_STREAM, still honored by ArduPilot) so the detector — and
    # our wait_gps — actually see position + IMU traffic. Send a few times
    # since early requests can be dropped during SITL bring-up.
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

    # Disable pre-arm checks, set home at the current location, and let the EKF
    # settle so GUIDED can arm in the bare SITL (no RC/battery). W4/W-ARMED: with
    # only ARMING_CHECK=0 and a plain arm command the copter often stayed
    # `armed=False` on the ground — then a later RTL disarms instantly and the
    # detector (correctly, its armed cross-check) reports the disarmed autopilot
    # as RTL-unconfirmed. ARMING_CHECK=0 is the SITL scripted-test standard
    # (NEVER for a real vehicle).
    log("ARMING_CHECK=0 + DO_SET_HOME(current) for SITL scripted arm")
    for _ in range(3):
        r.set_param("ARMING_CHECK", 0)
        time.sleep(0.5)
    # MAV_CMD_DO_SET_HOME param1=1 -> use current location as home/origin.
    r.sitl.mav.command_long_send(sysid, compid, mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                                 0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(8)  # let the EKF settle / set its origin with checks relaxed

    # Choreograph: GUIDED, force-arm, takeoff. All SEND-ONLY — the relay thread
    # is the sole reader of the SITL connection, so we must not call any
    # recv-blocking helper (it would deadlock against the relay). We send raw
    # commands, pace with sleeps, and observe arm/mode/alt via the relay state.
    log("mode GUIDED (custom_mode=4)")
    r.sitl.mav.set_mode_send(
        sysid, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
    time.sleep(3)
    # Force-arm (param2 = 21196 magic, bypasses residual checks) and CONFIRM via
    # the HEARTBEAT armed bit before proceeding — a plain arm silently failed.
    log("force-arming (MAV_CMD_COMPONENT_ARM_DISARM param1=1 param2=21196)")
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
    # Confirm airborne so the later RTL keeps the copter armed through the
    # detector's read-back dwell (descending from altitude takes >> the 5 s
    # verify window, so it stays armed long enough to confirm ActionAcked).
    r.wait_airborne(args.takeoff_alt * 0.5)

    log("clean flight window {} s (detector should stay Normal, preflight Ready)".format(
        args.clean_secs))
    t_end = time.time() + args.clean_secs
    while time.time() < t_end:
        time.sleep(1)
    mode_before = r.last_mode
    log("mode before spoof = {} ({})".format(mode_before, MODE_NAMES.get(mode_before, "?")))

    # Inject the spoof. Convert the desired east offset (metres) to degrees of
    # longitude at the home latitude: dlon_deg = metres / (111320 * cos(lat)).
    import math
    lat_rad = math.radians(args.home_lat)
    dlon_deg = args.glitch_m / (111320.0 * max(math.cos(lat_rad), 1e-6))

    param_mode = args.spoof_mode == "param"
    if not param_mode:
        # Wire-level meaconing: the relay rewrites the lat/lon of every GPS
        # message it forwards to the detector. Vehicle-agnostic — works
        # identically for PX4 and ArduPilot, no simulator GPS-glitch param
        # needed (PX4 has no clean equivalent). Models the attack the detector
        # exists to catch: the position the companion sees is offset from truth.
        # Applied as a step here.
        log("INJECTING SPOOF (relay wire-level): +{:.6f} deg lon (~{} m east at lat {})".format(
            dlon_deg, args.glitch_m, args.home_lat))
        r.spoof_dlat_deg = 0.0
        r.spoof_dlon_deg = dlon_deg
        r.spoof_active = True
    else:
        # SITL-param injection (ArduPilot only): bias the autopilot's OWN
        # simulated GPS receiver via SIM_GPS_GLITCH_Y (DEGREES; verified against
        # ArduPilot SIM_GPS.cpp). A STEP is innovation-gated by EK3 (rejected),
        # so we RAMP it in the watch loop below over --glitch-ramp-secs: each
        # epoch's jump stays under the gate and the EKF FUSES the offset. This is
        # the EKF-laundered "smart spoofer" whose fused output (position AND a
        # self-consistent velocity) the detector's velocity-aiding lane must
        # catch — the part the relay/wire path does NOT exercise.
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
                now - spoof_t, MODE_NAMES.get(r.last_mode, "?"), r.ekf_summary()))
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
        final, MODE_NAMES.get(final, "?"), r.ekf_summary()))
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
