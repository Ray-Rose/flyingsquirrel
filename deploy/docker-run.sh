#!/usr/bin/env bash
# deploy/docker-run.sh — run FlyingSquirrel in a container with
# defense-in-depth runtime flags applied.
#
# Edit the CONFIG_PATH below to point at your deployment config, then run.
# Uses host network so MAVLink UDP just works.
#
# Logs are captured by docker (journald or json-file driver depending on
# host config). Use `docker logs -f flyingsquirrel` to follow.
#
# AUDIT F-09 fix: hardened the runtime invocation with --read-only,
# --user, --cap-drop=ALL, --security-opt=no-new-privileges, --pids-limit,
# --memory, --tmpfs. The Dockerfile already sets USER flyingsquirrel
# (uid=1000), but without --read-only the rootfs is writable, defeating
# half the point of the unprivileged user. Container escape via a dep CVE
# was previously catastrophic — the container had default Docker caps
# (CAP_NET_RAW, CAP_NET_BIND_SERVICE, etc.) and full network access.
#
# Devices: only mount --device entries for nodes that ACTUALLY exist on
# the host. Hard-coding /dev/i2c-1 and /dev/ttyUSB0 makes the run fail
# cryptically on hosts without those nodes.

set -euo pipefail

IMAGE="${IMAGE:-flyingsquirrel:latest}"
CONFIG_PATH="${CONFIG_PATH:-$PWD/examples/sitl-copter.toml}"
STATE_DIR="${STATE_DIR:-$PWD/var-lib-flyingsquirrel}"
LOG_DIR="${LOG_DIR:-$PWD/var-log-flyingsquirrel}"
# Memory and PID limits — generous defaults but bounded so a runaway
# can't take down the host. Override via env for constrained / large setups.
MEMORY_LIMIT="${MEMORY_LIMIT:-512m}"
PIDS_LIMIT="${PIDS_LIMIT:-128}"

if [[ ! -e "$CONFIG_PATH" ]]; then
    echo "ERROR: config file not found at $CONFIG_PATH" >&2
    echo "       Set CONFIG_PATH=/path/to/your.toml or copy examples/sitl-copter.toml." >&2
    exit 1
fi

mkdir -p "$STATE_DIR" "$LOG_DIR"

# Build --device args only for nodes that exist. Avoids
#   "error gathering device information ... no such file or directory"
# on hosts that don't have I2C or USB-serial wired.
DEVICE_ARGS=()
for d in /dev/i2c-1 /dev/ttyUSB0; do
    if [[ -e "$d" ]]; then
        DEVICE_ARGS+=(--device "$d:$d")
    fi
done

# Host networking gives us direct access to UDP 14551 for MAVLink.
# For more isolation, swap `--network host` for `-p 14551:14551/udp`,
# but you'll have to ensure the autopilot endpoint matches the
# container's perceived bind address.
exec docker run --rm -it \
    --name flyingsquirrel \
    --network host \
    --read-only \
    --user 1000:1000 \
    --cap-drop=ALL \
    --security-opt=no-new-privileges:true \
    --memory="$MEMORY_LIMIT" \
    --pids-limit="$PIDS_LIMIT" \
    --tmpfs /tmp:rw,nosuid,nodev,noexec,size=64m \
    -v "$CONFIG_PATH:/etc/flyingsquirrel.toml:ro" \
    -v "$STATE_DIR:/var/lib/flyingsquirrel" \
    -v "$LOG_DIR:/var/log/flyingsquirrel" \
    "${DEVICE_ARGS[@]}" \
    "$IMAGE" "$@"
