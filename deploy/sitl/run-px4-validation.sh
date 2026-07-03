#!/usr/bin/env bash
# Closed-loop PX4 SITL validation for FlyingSquirrel (step 2 of the PX4
# milestone). The PX4 sibling of run-sitl-validation.sh.
#
# Topology differs from ArduPilot: the headless PX4 image PUSHES MAVLink UDP to a
# target, so PX4 is pointed at the HARNESS, which binds udpin:14540, relays
# PX4<->detector, choreographs the (PX4-specific) arm/takeoff, injects the spoof,
# and watches the autopilot react:
#
#   PX4 SITL  --UDP push 14540-->  [ harness (udpin) ]  <--UDP-->  FlyingSquirrel
#   (autopilot)                     (relay+choreograph)            (detector + RTL)
#
# Pass requires the same gate as the ArduPilot loop: the detector's OWN event log
# shows a Spoofed transition + an RTL action (ActionAcked/ActionUnconfirmed) AND
# the autopilot reached an RTL-equivalent (PX4 AUTO.RTL).
set -uo pipefail

PX4_IMAGE="${PX4_IMAGE:-jonasvautherin/px4-gazebo-headless:latest}"
FS_IMAGE="${FS_IMAGE:-flyingsquirrel:sitl}"
HARNESS_IMAGE="${HARNESS_IMAGE:-fs-sitl-harness:latest}"
NET="${NET:-fs-px4-net}"

# PX4 default SITL home: Zurich.
HOME_LAT="${HOME_LAT:-47.397742}"
HOME_LON="${HOME_LON:-8.545594}"
GLITCH_M="${GLITCH_M:-400}"
CLEAN_SECS="${CLEAN_SECS:-25}"
WATCH_SECS="${WATCH_SECS:-70}"
SPOOF_MODE="${SPOOF_MODE:-relay}"
GLITCH_RAMP_SECS="${GLITCH_RAMP_SECS:-60}"
DETECTOR_DURATION="${DETECTOR_DURATION:-320}"
IMU_RATE="${IMU_RATE:-50}"
# Experimental PX4 ActionAcked path (opt-in). When set, the harness injects an
# external-vision position so PX4 keeps a non-GPS position estimate after the GPS
# sever and can hold an armed RTL through the detector's verify dwell. The proven
# closed-loop gate does NOT use this; it is exercised only by a non-gating CI leg.
PX4_EV_FALLBACK="${PX4_EV_FALLBACK:-}"
# When set, PASS additionally requires the STRONG ActionAcked read-back (not just
# a commanded RTL). Used by the experimental EV-fallback leg.
REQUIRE_ACTION_ACKED="${REQUIRE_ACTION_ACKED:-}"
# Seconds to let PX4 boot (Gazebo headless) + GPS-lock before the harness begins
# its choreography. PX4 is slower to come up than ArduPilot's in-process sim.
PX4_BOOT_SECS="${PX4_BOOT_SECS:-75}"

SUBNET="${SUBNET:-172.31.9.0/24}"
DET_IP="${DET_IP:-172.31.9.10}"
HARNESS_IP="${HARNESS_IP:-172.31.9.20}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="$SCRIPT_DIR/last-run"

cleanup() {
    echo "[px4] cleaning up containers..."
    docker rm -f fs-px4 fs-detector fs-harness >/dev/null 2>&1 || true
    docker network rm "$NET" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "[px4] building harness image..."
docker build -f "$SCRIPT_DIR/Dockerfile.harness" -t "$HARNESS_IMAGE" "$SCRIPT_DIR"

echo "[px4] (re)creating network $NET ($SUBNET)"
docker network rm "$NET" >/dev/null 2>&1 || true
docker network create --subnet "$SUBNET" "$NET" >/dev/null

rm -rf "$OUT_DIR"; mkdir -p "$OUT_DIR"; chmod 777 "$OUT_DIR"

echo "[px4] starting FlyingSquirrel detector (mav, --vehicle px4) at $DET_IP"
# The harness relays PX4's GPS/IMU here and forwards the detector's RTL+PARAM
# back to PX4. --mav-target = the HARNESS (where the detector sends RTL).
docker run -d --name fs-detector --network "$NET" --ip "$DET_IP" \
    -v "${OUT_DIR}:/out" \
    "$FS_IMAGE" \
    --gps-source mav --imu-source mav --controller mav \
    --vehicle px4 \
    --mav-bind 0.0.0.0:14551 --mav-target "${HARNESS_IP}:14551" \
    --mav-no-source-filter --mav-allow-any-source-port \
    --expected-home="${HOME_LAT},${HOME_LON}" --max-home-distance-m 50000 \
    --json-log /out/events.jsonl --forensic-dir /out \
    --imu-rate "$IMU_RATE" \
    --duration "$DETECTOR_DURATION" --log-level info >/dev/null

echo "[px4] starting PX4 SITL ($PX4_IMAGE), pushing MAVLink to harness $HARNESS_IP"
# Single arg -> PX4 sends both API (14540) and QGC (14550) to the harness IP.
docker run -d --name fs-px4 --network "$NET" "$PX4_IMAGE" "$HARNESS_IP" >/dev/null

echo "[px4] letting PX4 boot + GPS-lock for ${PX4_BOOT_SECS}s before choreography..."
sleep "$PX4_BOOT_SECS"

EV_ARG=""
if [[ -n "$PX4_EV_FALLBACK" ]]; then
    EV_ARG="--px4-ev-fallback"
    echo "[px4] EV fallback ENABLED (experimental ActionAcked path)"
fi

echo "[px4] launching harness at $HARNESS_IP (binds udpin:14540, flies PX4, spoofs, watches)"
set +e
# Tee the harness stdout into the artifact dir as well. It carries the PX4-SIDE
# truth — EV-fallback status, the periodic EKF_STATUS_REPORT summary (whether the
# vision position was actually fused vs. const-pos after the sever), and arm/
# disarm — which the flaky GitHub Actions log API frequently refuses to return.
# PIPESTATUS[0] keeps HARNESS_RC as the harness's exit code, not tee's.
docker run --rm --name fs-harness --network "$NET" --ip "$HARNESS_IP" \
    "$HARNESS_IMAGE" \
    --sitl udpin:0.0.0.0:14540 \
    --det-host "$DET_IP" --det-port 14551 \
    --vehicle px4 \
    --glitch-m "$GLITCH_M" --home-lat "$HOME_LAT" \
    --spoof-mode "$SPOOF_MODE" --glitch-ramp-secs "$GLITCH_RAMP_SECS" \
    --clean-secs "$CLEAN_SECS" --watch-secs "$WATCH_SECS" $EV_ARG 2>&1 | tee "$OUT_DIR/harness.log"
HARNESS_RC=${PIPESTATUS[0]}
set -e

echo ""
# Save the FULL detector stdout into the artifact dir — the GitHub Actions log
# API has been flaky for these runs, so the downloadable detector.log is the
# reliable place to read the gate-by-gate "MAV VERIFY" line from.
docker logs fs-detector > "$OUT_DIR/detector.log" 2>&1 || true
# Capture PX4's OWN console too. It states the EKF vision-fusion state and the
# exact reason an EV-fallback run did or didn't hold an armed RTL — "EKF commander:
# ... GPS fusion", "Landing detected", "Disarmed", or a GPS-loss failsafe. This is
# what distinguishes "EV never fused -> failsafe land" from "EV fused, RTL flew home
# and landed" (opposite fixes), so it must be in the artifact, not the flaky job log.
docker logs fs-px4 > "$OUT_DIR/px4.log" 2>&1 || true
echo "==================== detector log (last 70 lines) ===================="
tail -70 "$OUT_DIR/detector.log"

echo ""
echo "[px4] RTL verify line(s):"
grep -aE 'MAV VERIFY|MAV SEVER VERIFY' "$OUT_DIR/detector.log" | tail -3 || echo "  (no MAV VERIFY line found)"

echo ""
echo "[px4] detector event-log assertions:"
DET_SPOOFED=1
DET_RTB=1
DET_ACKED=1
if [[ -f "$OUT_DIR/events.jsonl" ]]; then
    grep -q '"to":"Spoofed"' "$OUT_DIR/events.jsonl" && DET_SPOOFED=0
    grep -qE 'ActionAcked|ActionUnconfirmed' "$OUT_DIR/events.jsonl" && DET_RTB=0
    grep -q 'ActionAcked' "$OUT_DIR/events.jsonl" && DET_ACKED=0
    echo "  Spoofed transition in events.jsonl   : $([[ $DET_SPOOFED -eq 0 ]] && echo yes || echo NO)"
    echo "  detector commanded+verified RTL      : $([[ $DET_RTB -eq 0 ]] && echo yes || echo NO)"
    echo "  RTL read-back ActionAcked (confirmed): $([[ $DET_ACKED -eq 0 ]] && echo yes || echo 'no (commanded; verify dwell may have been starved)')"
    if grep -q 'VelocityAiding' "$OUT_DIR/events.jsonl"; then
        echo "  velocity-aiding lane fired           : yes"
    else
        echo "  velocity-aiding lane fired           : no (position/velocity-mismatch lane)"
    fi
else
    echo "  events.jsonl not found at $OUT_DIR (detector may not have armed)"
fi

echo ""
echo "[px4] harness exit code: $HARNESS_RC  (0=closed-loop RTL verified, 3=no RTL in window, 2=setup)"
# Base closed-loop gate (same as the ArduPilot loop): detector did the work
# (Spoofed + RTL action) AND the autopilot reached an RTL-equivalent (harness RC 0).
BASE_OK=1
[[ "$HARNESS_RC" -eq 0 && $DET_SPOOFED -eq 0 && $DET_RTB -eq 0 ]] && BASE_OK=0

if [[ -n "$REQUIRE_ACTION_ACKED" ]]; then
    # Experimental EV-fallback leg: PASS requires the STRONG ActionAcked read-back
    # (PX4 held an armed RTL through the verify dwell), not just a commanded RTL.
    if [[ $BASE_OK -eq 0 && $DET_ACKED -eq 0 ]]; then
        echo "[px4] PASS: PX4 closed loop + STRONG ActionAcked read-back verified (EV fallback held PX4 armed through the dwell)."
        exit 0
    fi
    if [[ $BASE_OK -eq 0 ]]; then
        echo "[px4] PARTIAL: closed loop verified but ActionAcked NOT reached — the EV fallback likely needs tuning (PX4 may still have LANDed/disarmed for lack of a fused vision position). Inspect $OUT_DIR."
        exit 3
    fi
    echo "[px4] FAIL: closed loop not verified (harness_rc=$HARNESS_RC spoofed=$DET_SPOOFED rtb=$DET_RTB)."
    exit "${HARNESS_RC:-1}"
fi

# Default (proven) closed-loop gate — unchanged.
if [[ $BASE_OK -eq 0 ]]; then
    echo "[px4] PASS: PX4 closed loop verified — detector detected + commanded RTL; PX4 reached RTL-equivalent."
    exit 0
fi
if [[ "$HARNESS_RC" -eq 0 ]]; then
    echo "[px4] PARTIAL: PX4 reached RTL but the detector log lacks a Spoofed transition and/or an RTL action — inspect $OUT_DIR/events.jsonl."
    exit 3
fi
exit "$HARNESS_RC"
