#!/usr/bin/env bash
# PX4 SITL bring-up SMOKE test (step 1 of the PX4 live-SITL milestone).
#
# PX4 SITL differs topologically from ArduPilot SITL: the headless PX4 image
# (jonasvautherin/px4-gazebo-headless) PUSHES MAVLink UDP to a target IP on
# 14540 (offboard/API) + 14550 (QGC), rather than being a TCP server the harness
# dials into. So here PX4 is pointed straight at the detector, which binds 14540
# and receives. No spoof / closed loop yet — this de-risks the unknowns before
# building the full PX4 choreography harness:
#   * does the PX4 image come up headless on a GH runner?
#   * does the detector receive PX4's HEARTBEAT / GPS_RAW_INT / HIGHRES_IMU?
#   * what is PX4's HIGHRES_IMU rate (the --imu-rate footgun)?
#   * does the detector reach preflight Ready on a (grounded, disarmed) PX4?
#
# Exit 0 if the detector reached preflight Ready on PX4's stream; the detector +
# PX4 logs (and any GPS/IMU-rate warnings) are printed regardless for diagnosis.
set -uo pipefail

PX4_IMAGE="${PX4_IMAGE:-jonasvautherin/px4-gazebo-headless:latest}"
FS_IMAGE="${FS_IMAGE:-flyingsquirrel:sitl}"
NET="${NET:-fs-px4-net}"
SUBNET="${SUBNET:-172.31.9.0/24}"
DET_IP="${DET_IP:-172.31.9.10}"
# PX4's default SITL home is Zurich.
HOME_LAT="${HOME_LAT:-47.397742}"
HOME_LON="${HOME_LON:-8.545594}"
# Sizes the IMU integration-gap gate. PX4 HIGHRES_IMU on the 14540 onboard link
# streams at an as-yet-unconfirmed rate; 50 Hz is a conservative starting guess
# (gate = 2.5/50 = 50 ms). The detector logs the rate it derives + any
# DEAD-RECKONING STALLED warning, which reveals the true rate for tuning.
IMU_RATE="${IMU_RATE:-50}"
BOOT_SECS="${BOOT_SECS:-170}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="$SCRIPT_DIR/last-run"

cleanup() {
    echo "[px4] cleaning up..."
    docker rm -f fs-px4 fs-detector >/dev/null 2>&1 || true
    docker network rm "$NET" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "[px4] (re)creating network $NET ($SUBNET)"
docker network rm "$NET" >/dev/null 2>&1 || true
docker network create --subnet "$SUBNET" "$NET" >/dev/null

rm -rf "$OUT_DIR"; mkdir -p "$OUT_DIR"; chmod 777 "$OUT_DIR"

echo "[px4] starting detector at $DET_IP (binds :14540, --vehicle px4)"
# PX4 pushes MAVLink to the detector; the detector binds 14540 and receives.
# --controller mav with a dead target (no spoof => no RTL ever dispatched).
# --mav-no-source-filter / --mav-allow-any-source-port: PX4 pushes from its own
# ephemeral source (documented SITL exceptions, never production).
docker run -d --name fs-detector --network "$NET" --ip "$DET_IP" \
    -v "${OUT_DIR}:/out" \
    "$FS_IMAGE" \
    --gps-source mav --imu-source mav --controller mav \
    --vehicle px4 \
    --mav-bind 0.0.0.0:14540 --mav-target "127.0.0.1:14541" \
    --mav-no-source-filter --mav-allow-any-source-port \
    --expected-home="${HOME_LAT},${HOME_LON}" --max-home-distance-m 50000 \
    --json-log /out/events.jsonl --forensic-dir /out \
    --imu-rate "$IMU_RATE" \
    --duration 220 --log-level info >/dev/null

echo "[px4] starting PX4 SITL ($PX4_IMAGE), pushing MAVLink to detector $DET_IP"
docker run -d --name fs-px4 --network "$NET" "$PX4_IMAGE" "$DET_IP" >/dev/null

echo "[px4] letting PX4 boot (Gazebo headless) + GPS-lock + stream for ${BOOT_SECS}s..."
sleep "$BOOT_SECS"

echo ""
echo "==================== detector log (last 90) ===================="
docker logs fs-detector 2>&1 | tail -90

echo ""
echo "==================== PX4 log (last 25) ===================="
docker logs fs-px4 2>&1 | tail -25

echo ""
echo "[px4] smoke assertions:"
RC=1
LOG="$(docker logs fs-detector 2>&1)"
if grep -qiE "PREFLIGHT.*pass|preflight.*ready|detector arming" <<<"$LOG"; then
    echo "  detector reached preflight Ready on PX4 stream : yes"
    RC=0
else
    echo "  detector reached preflight Ready on PX4 stream : NO"
fi
# Informative breadcrumbs regardless of pass/fail.
grep -qiE "GPS|GLOBAL_POSITION|fix" <<<"$LOG" && echo "  saw GPS traffic                                : yes" || echo "  saw GPS traffic                                : no"
grep -qiE "DEAD-RECKONING STALLED" <<<"$LOG" && echo "  IMU integration-gap STALL (imu-rate too high)  : YES (lower --imu-rate)" || echo "  IMU integration-gap STALL                      : no"
exit $RC
