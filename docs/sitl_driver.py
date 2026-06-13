#!/usr/bin/env python3
"""
SITL closed-loop driver for FlyingSquirrel validation (Phase V).

Runs INSIDE the ArduPilot SITL container (uses the vendored pymavlink at
/ardupilot/modules/mavlink). Drives a full closed-loop test:

  1. Connect to SITL's MAVLink (tcp:127.0.0.1:5760).
  2. Wait for EKF + GPS ready, then arm + takeoff to altitude.
  3. Fan out every inbound MAVLink message to a UDP endpoint
     (flyingsquirrel's --mav-bind) so the detector sees real GPS_RAW_INT /
     GLOBAL_POSITION_INT / HEARTBEAT / SCALED_IMU traffic.
  4. After a clean-flight observation window, inject a GPS position glitch
     via SITL's SIM_GPS1_POS_X/Y parameters (the autopilot's OWN simulated
     GPS is spoofed — the realistic attack, not a faked wire signal).
  5. Watch the autopilot's flight-mode response and report it.

This is the empirical answer to the four open questions in docs/sitl.md:
does PARAM_SET/GPS-glitch apply mid-flight, does the AP go RTL or LAND, does
our verify pass on real timing, does the EKF lane-switch.

Usage (inside the container):
  PYTHONPATH=/ardupilot/modules/mavlink python3 /work/sitl_driver.py \
      --sitl tcp:127.0.0.1:5760 --fanout udpout:FS_HOST:14550 \
      --takeoff-alt 30 --clean-secs 20 --glitch-m 300 --watch-secs 40

The --fanout target is where flyingsquirrel listens (its --mav-bind). We use
udpout so SITL-side acts as the client connecting to the detector's bound
UDP socket.
"""
import argparse
import sys
import time

from pymavlink import mavutil


def log(msg):
    print("[driver] {}".format(msg), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sitl", default="tcp:127.0.0.1:5760")
    ap.add_argument("--fanout", default=None,
                    help="udpout:HOST:PORT to forward all MAVLink to (flyingsquirrel)")
    ap.add_argument("--takeoff-alt", type=float, default=30.0)
    ap.add_argument("--clean-secs", type=float, default=20.0)
    ap.add_argument("--glitch-m", type=float, default=300.0)
    ap.add_argument("--watch-secs", type=float, default=40.0)
    args = ap.parse_args()

    log("connecting to SITL at {}".format(args.sitl))
    m = mavutil.mavlink_connection(args.sitl, retries=10)
    m.wait_heartbeat(timeout=60)
    log("heartbeat from sys {} comp {}".format(m.target_system, m.target_component))

    fan = None
    if args.fanout:
        # input=False → we are the originator side (udpout client).
        fan = mavutil.mavlink_connection(args.fanout, input=False)
        log("fanning MAVLink out to {}".format(args.fanout))

    def forward(msg):
        # Forward RAW bytes, not a re-encode — preserves the original CRC,
        # sequence, and signing exactly as ArduPilot emitted them, so
        # flyingsquirrel sees a byte-faithful stream.
        if fan is None:
            return
        try:
            buf = msg.get_msgbuf()
            if buf:
                fan.write(buf)
        except Exception:
            pass  # best-effort forwarding

    def pump(duration, also_forward=True):
        """Receive for `duration` secs, optionally forwarding to fanout.
        Returns the last HEARTBEAT custom_mode seen."""
        end = time.time() + duration
        last_mode = None
        while time.time() < end:
            msg = m.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            if also_forward:
                forward(msg)
            t = msg.get_type()
            if t == "HEARTBEAT":
                last_mode = msg.custom_mode
        return last_mode

    # --- Wait for EKF/GPS ready ---
    log("waiting for GPS 3D fix + EKF...")
    deadline = time.time() + 90
    got_fix = False
    while time.time() < deadline:
        msg = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2.0)
        if msg is not None:
            forward(msg)
        if msg and msg.fix_type >= 3 and msg.satellites_visible >= 6:
            got_fix = True
            log("GPS ready: fix_type={} sats={}".format(msg.fix_type, msg.satellites_visible))
            break
    if not got_fix:
        log("FAIL: no GPS 3D fix within 90s")
        return 2

    # --- Set mode GUIDED, arm, takeoff ---
    log("setting GUIDED")
    m.set_mode("GUIDED")
    pump(2)
    log("arming")
    m.arducopter_arm()
    m.motors_armed_wait()
    log("armed; takeoff to {} m".format(args.takeoff_alt))
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, args.takeoff_alt)

    # Climb out.
    pump(12)

    # --- Clean observation window (detector should stay NORMAL) ---
    log("clean flight window: {} s (detector should stay Normal)".format(args.clean_secs))
    pump(args.clean_secs)

    # --- Inject GPS glitch via SITL sim params ---
    # SIM_GPS1_POS_X/Y are body-frame offsets (m) added to the simulated GPS.
    # A 300 m east step is far above any jump threshold.
    log("INJECTING GPS GLITCH: SIM_GPS1_POS_Y = {} m".format(args.glitch_m))
    for name in ("SIM_GPS1_POS_Y", "SIM_GPS_POS_Y"):  # param name varies by AP version
        m.mav.param_set_send(
            m.target_system, m.target_component,
            name.encode("ascii"),
            float(args.glitch_m),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    log("watching autopilot mode response for {} s".format(args.watch_secs))
    last_mode = pump(args.watch_secs)

    # ArduCopter custom_mode: 0=STABILIZE 3=AUTO 4=GUIDED 5=LOITER 6=RTL 9=LAND 21=SMART_RTL
    mode_names = {0: "STABILIZE", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
                  6: "RTL", 9: "LAND", 16: "POSHOLD", 21: "SMART_RTL"}
    log("final autopilot custom_mode = {} ({})".format(
        last_mode, mode_names.get(last_mode, "?")))
    log("DONE")
    return 0


if __name__ == "__main__":
    sys.exit(main())
