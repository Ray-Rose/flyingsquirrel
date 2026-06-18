#!/usr/bin/env bash
#
# docs/sitl_smoke.sh — end-to-end FlyingSquirrel + ArduPilot SITL smoke test.
#
# Prereqs:
#   * ArduPilot SITL built (~/ardupilot/Tools/autotest/sim_vehicle.py works)
#   * flyingsquirrel built: `cargo build --release`
#   * mavproxy + pymavlink in PATH (installed by install-prereqs-ubuntu.sh)
#   * tmux (for parallel processes)
#
# Run from the flyingsquirrel repo root:
#   bash docs/sitl_smoke.sh
#
# Exit code:
#   0  -> ActionAcked observed (closed-loop verified end-to-end)
#   1  -> ActionUnconfirmed   (autopilot did NOT transition; capture logs)
#   2  -> No Spoofed transition (detector did not fire — possibly tuning issue)
#   3  -> Environment / setup failure (SITL didn't start, binary missing, etc.)

set -u
set -o pipefail

# --- Paths ---
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ARDUPILOT_ROOT="${ARDUPILOT_ROOT:-$HOME/ardupilot}"
FS_BIN="${FS_BIN:-$REPO_ROOT/target/release/flyingsquirrel}"
WORK_DIR="${WORK_DIR:-/tmp/fs-sitl-smoke}"
JSON_LOG="$WORK_DIR/spoof-events.jsonl"
FS_LOG="$WORK_DIR/flyingsquirrel.log"
SITL_LOG="$WORK_DIR/sitl.log"

mkdir -p "$WORK_DIR"

# --- Preflight: binaries exist ---
if [[ ! -x "$FS_BIN" ]]; then
    echo "ERROR: flyingsquirrel binary not found at $FS_BIN" >&2
    echo "       Run 'cargo build --release' first." >&2
    exit 3
fi
if [[ ! -d "$ARDUPILOT_ROOT" ]]; then
    echo "ERROR: ArduPilot not found at $ARDUPILOT_ROOT" >&2
    echo "       Set ARDUPILOT_ROOT or follow the SITL build steps in docs/sitl.md." >&2
    exit 3
fi
if ! command -v mavproxy.py >/dev/null 2>&1; then
    echo "ERROR: mavproxy.py not in PATH" >&2
    exit 3
fi

# --- Pick a test location (Canberra; ArduPilot's default sim home) ---
SIM_HOME_LAT="-35.363261"
SIM_HOME_LON="149.165230"

# --- Start SITL in the background ---
echo "[1/4] Starting ArduPilot SITL (Copter, home=$SIM_HOME_LAT,$SIM_HOME_LON)..."
cd "$ARDUPILOT_ROOT"
# --out=udpout sends MAVLink to FlyingSquirrel; default MAVProxy port on 14550
# stays available for our manual interaction.
sim_vehicle.py -v ArduCopter \
    --no-mavproxy \
    --out=udpout:127.0.0.1:14551 \
    --out=udpout:127.0.0.1:14550 \
    > "$SITL_LOG" 2>&1 &
SITL_PID=$!

cleanup() {
    echo "Cleaning up..."
    kill "$SITL_PID" 2>/dev/null || true
    pkill -f arducopter || true
    pkill -f mavproxy || true
    pkill -f flyingsquirrel || true
}
trap cleanup EXIT

# Wait for SITL to come up (GPS lock + heartbeat)
echo "[2/4] Waiting for SITL GPS lock (up to 60s)..."
for i in $(seq 1 60); do
    if grep -q "GPS 1: detected as u-blox" "$SITL_LOG" 2>/dev/null; then
        echo "       SITL ready after ${i}s."
        break
    fi
    if [[ $i -eq 60 ]]; then
        echo "ERROR: SITL did not acquire GPS in 60s. Tail:" >&2
        tail -20 "$SITL_LOG" >&2
        exit 3
    fi
    sleep 1
done

# Give it another 5s to stabilize
sleep 5

# --- Arm + takeoff via mavproxy (one-shot command sequence) ---
echo "[3/4] Arming + takeoff..."
mavproxy.py --master=udpin:127.0.0.1:14550 --cmd "
mode guided
arm throttle
takeoff 10
" --quadcopter --daemon &
MAVPROXY_PID=$!
sleep 15  # wait for takeoff to climb

# --- Start FlyingSquirrel ---
echo "[4/4] Starting FlyingSquirrel + injecting spoof at t=20s..."
"$FS_BIN" \
    --gps-source mav --imu-source mav --controller mav \
    --vehicle ardu-copter \
    --mav-bind 127.0.0.1:14551 \
    --mav-target 127.0.0.1:14550 \
    --mav-target-system 1 --mav-target-component 1 \
    --expected-home "$SIM_HOME_LAT,$SIM_HOME_LON" \
    --max-home-distance-m 5000 \
    --json-log "$JSON_LOG" \
    --duration 60 \
    > "$FS_LOG" 2>&1 &
FS_PID=$!

# Inject the spoof via SITL params after FlyingSquirrel preflight passes
sleep 20
# NOTE: SIM_GPS_GLITCH_* is in DEGREES, not metres (verified against ArduPilot
# SIM_GPS.cpp). _Y is longitude (east). At this home latitude (-35.36),
# 0.0044 deg ~= 400 m east — well above the per-axis CUSUM noise floor.
echo "       Injecting ~400m east spoof via SIM_GPS_GLITCH_Y (degrees, not metres)..."
mavproxy.py --master=udpin:127.0.0.1:14550 --cmd "
param set SIM_GPS_GLITCH_Y 0.0044
" --quadcopter --daemon &

# Wait for FlyingSquirrel to finish
wait "$FS_PID"

# --- Analyze results ---
echo
echo "=== Results ==="
if grep -q '"kind":"ActionAcked"' "$JSON_LOG" 2>/dev/null; then
    echo "PASS: ActionAcked observed — closed loop verified."
    grep -E '"kind":"(StateTransition|ActionAcked|ActionUnconfirmed|MAV ACTION)"' "$JSON_LOG" | head
    exit 0
elif grep -q '"kind":"ActionUnconfirmed"' "$JSON_LOG" 2>/dev/null; then
    echo "FAIL: ActionUnconfirmed — RTL did NOT take effect on the autopilot." >&2
    echo "      Autopilot may have rejected the command, or transitioned to a mode we" >&2
    echo "      don't recognize as RTL. Capture autopilot log:" >&2
    echo "        $(ls -t logs/*.BIN | head -1)" >&2
    echo "      And FlyingSquirrel events:" >&2
    echo "        $JSON_LOG" >&2
    exit 1
elif grep -q '"kind":"StateTransition".*"to":"Spoofed"' "$JSON_LOG" 2>/dev/null; then
    echo "PARTIAL: Detector fired Spoofed but no Action confirmation observed." >&2
    echo "         Verification timed out before mode change." >&2
    exit 1
else
    echo "NO DETECT: Detector did not transition to Spoofed." >&2
    echo "           Spoof magnitude may be below threshold, or preflight did not pass." >&2
    echo "           FlyingSquirrel log tail:" >&2
    tail -20 "$FS_LOG" >&2
    exit 2
fi
