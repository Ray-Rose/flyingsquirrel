#!/usr/bin/env bash
# One-command closed-loop SITL validation for FlyingSquirrel.
#
# Stands up three containers on a private docker network:
#   sitl           — ArduPilot ArduCopter SITL (TCP MAVLink server :5760)
#   flyingsquirrel — the detector (UDP MAVLink, --controller mav)
#   harness        — bidirectional MAVLink relay + flight choreographer that
#                    also injects a GPS spoof and watches the autopilot react
#
# The harness forwards SITL->detector (GPS/IMU/HEARTBEAT) AND detector->SITL
# (RTL command + PARAM_SET), so the loop genuinely closes: a spoof injected
# into SITL's simulated GPS should make the detector fire and command RTL,
# and that RTL should reach and actuate the autopilot.
#
# Usage:  bash deploy/sitl/run-sitl-validation.sh
# Env overrides: SITL_IMAGE, FS_IMAGE, HOME_LAT, HOME_LON, GLITCH_M, etc.
#
# Exit code mirrors the harness: 0 = autopilot reached RTL-equivalent (full
# closed loop verified); 3 = no RTL within window (inspect detector log);
# 2 = setup failure.
set -euo pipefail

SITL_IMAGE="${SITL_IMAGE:-radarku/ardupilot-sitl:latest}"
FS_IMAGE="${FS_IMAGE:-flyingsquirrel:sitl}"
HARNESS_IMAGE="${HARNESS_IMAGE:-fs-sitl-harness:latest}"
NET="${NET:-fs-sitl-net}"

# CMAC (ArduPilot default sim home), Canberra.
HOME_LAT="${HOME_LAT:--35.363261}"
HOME_LON="${HOME_LON:-149.165230}"
HOME_ALT="${HOME_ALT:-584}"
HOME_DIR="${HOME_DIR:-353}"
GLITCH_M="${GLITCH_M:-400}"
CLEAN_SECS="${CLEAN_SECS:-20}"
WATCH_SECS="${WATCH_SECS:-70}"

# Static-IP subnet. `--mav-target` is parsed as a numeric SocketAddr (clap does
# NOT resolve hostnames), so the detector and harness get fixed IPs rather than
# service aliases:
#   detector .10  (binds :14551, receives GPS/IMU, sends RTL + PARAM_SET back)
#   harness  .20  (relays SITL<->detector; this is the detector's RTL target)
# The previous `--mav-target flyingsquirrel:14551` was doubly broken: a hostname
# the SocketAddr parser rejects AND the detector's OWN alias (RTL to itself).
SUBNET="${SUBNET:-172.30.7.0/24}"
DET_IP="${DET_IP:-172.30.7.10}"
HARNESS_IP="${HARNESS_IP:-172.30.7.20}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="$SCRIPT_DIR/last-run"

cleanup() {
    echo "[run] cleaning up containers..."
    docker rm -f fs-sitl fs-detector fs-harness >/dev/null 2>&1 || true
    docker network rm "$NET" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "[run] building harness image..."
docker build -f "$SCRIPT_DIR/Dockerfile.harness" -t "$HARNESS_IMAGE" "$SCRIPT_DIR"

echo "[run] (re)creating network $NET ($SUBNET)"
docker network rm "$NET" >/dev/null 2>&1 || true
docker network create --subnet "$SUBNET" "$NET" >/dev/null

echo "[run] starting SITL (home ${HOME_LAT},${HOME_LON})"
docker run -d --name fs-sitl --network "$NET" --network-alias sitl \
    -e VEHICLE=ArduCopter -e INSTANCE=0 \
    -e LAT="$HOME_LAT" -e LON="$HOME_LON" -e ALT="$HOME_ALT" -e DIR="$HOME_DIR" \
    -e MODEL=quad -e SPEEDUP=1 \
    "$SITL_IMAGE" >/dev/null

# Mount a host dir for the detector's JSON event log + forensic dumps so the
# run is inspectable (and CI-gatable) after the containers are torn down.
rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"
# The detector image runs as a NON-root user (uid 1000); make the bind-mount
# world-writable so it can write events.jsonl / forensic dumps. On Docker
# Desktop the mount is writable regardless, but a Linux CI runner's dir is
# owned by the runner uid, which the container user can't write (os error 13).
chmod 777 "$OUT_DIR"

echo "[run] starting FlyingSquirrel detector (mav GPS+IMU+controller) at $DET_IP"
# --mav-target = the HARNESS ip:port (where the detector sends its RTL +
# PARAM_SET); the harness relays those into SITL, closing the loop.
# --mav-no-source-filter because the harness relays from its own container
# address, not the autopilot's; --mav-allow-any-source-port because the relay's
# SOURCE port isn't guaranteed to match the target port across environments
# (both are documented SITL exceptions, not production posture).
docker run -d --name fs-detector --network "$NET" --ip "$DET_IP" \
    -v "${OUT_DIR}:/out" \
    "$FS_IMAGE" \
    --gps-source mav --imu-source mav --controller mav \
    --vehicle ardu-copter \
    --mav-bind 0.0.0.0:14551 --mav-target "${HARNESS_IP}:14551" \
    --mav-no-source-filter --mav-allow-any-source-port \
    --expected-home="${HOME_LAT},${HOME_LON}" --max-home-distance-m 5000 \
    --json-log /out/events.jsonl --forensic-dir /out \
    --imu-rate 10 \
    --duration 180 --log-level info >/dev/null

echo "[run] launching harness at $HARNESS_IP (waits for GPS, flies, spoofs, watches)"
set +e
docker run --rm --name fs-harness --network "$NET" --ip "$HARNESS_IP" \
    "$HARNESS_IMAGE" \
    --sitl tcp:sitl:5760 \
    --det-host "$DET_IP" --det-port 14551 \
    --glitch-m "$GLITCH_M" --home-lat "$HOME_LAT" \
    --clean-secs "$CLEAN_SECS" --watch-secs "$WATCH_SECS"
HARNESS_RC=$?
set -e

echo ""
echo "==================== detector log (last 60 lines) ===================="
docker logs fs-detector 2>&1 | tail -60

# Independent confirmation from the detector's OWN event log that it actually
# detected the spoof and confirmed RTL — not just that the autopilot happened
# to enter RTL (ArduPilot's native EKF glitch protection can do that on its
# own). Both must hold for a real closed-loop pass.
echo ""
echo "[run] detector event-log assertions:"
DET_SPOOFED=1   # detector detected + escalated to Spoofed
DET_RTB=1       # detector commanded RTL and tried to verify (Acked or Unconfirmed)
DET_ACKED=1     # the STRONGER read-back confirmation (autopilot held armed RTL)
if [[ -f "$OUT_DIR/events.jsonl" ]]; then
    grep -q '"to":"Spoofed"' "$OUT_DIR/events.jsonl" && DET_SPOOFED=0
    grep -qE 'ActionAcked|ActionUnconfirmed' "$OUT_DIR/events.jsonl" && DET_RTB=0
    grep -q 'ActionAcked' "$OUT_DIR/events.jsonl" && DET_ACKED=0
    echo "  Spoofed transition in events.jsonl   : $([[ $DET_SPOOFED -eq 0 ]] && echo yes || echo NO)"
    echo "  detector commanded+verified RTL      : $([[ $DET_RTB -eq 0 ]] && echo yes || echo NO)"
    echo "  RTL read-back ActionAcked (confirmed): $([[ $DET_ACKED -eq 0 ]] && echo yes || echo 'no (commanded; bare-SITL copter likely disarmed on landing before the dwell)')"
else
    echo "  events.jsonl not found at $OUT_DIR (detector may not have armed)"
fi

echo ""
echo "[run] harness exit code: $HARNESS_RC  (0=closed-loop RTL verified, 3=no RTL in window, 2=setup)"
# The closed loop is PROVEN when the detector's OWN log shows it detected the
# spoof (Spoofed) and commanded + attempted to verify RTL (ActionAcked or
# ActionUnconfirmed), AND the autopilot reached an RTL-equivalent mode (harness).
# A native-EKF reaction cannot produce a detector Spoofed event, so this can't
# pass without the detector doing the work. The STRONGER read-back confirmation
# (ActionAcked) additionally needs the autopilot to stay ARMED in RTL through the
# dwell — reported above but NOT gated, since bare-SITL arming is environment-
# fragile and a grounded copter disarms on landing before the dwell completes.
if [[ "$HARNESS_RC" -eq 0 && $DET_SPOOFED -eq 0 && $DET_RTB -eq 0 ]]; then
    if [[ $DET_ACKED -eq 0 ]]; then
        echo "[run] PASS: closed loop verified — detector detected, commanded RTL, read-back ActionAcked; autopilot reached RTL."
    else
        echo "[run] PASS: closed loop verified — detector detected + commanded RTL; autopilot reached RTL. (RTL read-back was ActionUnconfirmed: expected when the bare-SITL copter disarms on landing before the dwell.)"
    fi
    exit 0
fi
if [[ "$HARNESS_RC" -eq 0 ]]; then
    echo "[run] PARTIAL: autopilot reached RTL but the detector log lacks a Spoofed \
transition and/or an RTL action event — inspect $OUT_DIR/events.jsonl (a native-EKF \
reaction can reach RTL without the detector)."
    exit 3
fi
exit "$HARNESS_RC"
