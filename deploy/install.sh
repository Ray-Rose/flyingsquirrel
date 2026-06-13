#!/usr/bin/env bash
# deploy/install.sh — install FlyingSquirrel as a systemd-managed daemon.
#
# Run as root. Assumes the binary is already built:
#   cargo build --release --features hw-i2c,journald
#
# Idempotent — re-running upgrades the binary + service file but does NOT
# overwrite an existing /etc/flyingsquirrel.toml.

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "ERROR: must run as root (sudo bash deploy/install.sh)" >&2
    exit 1
fi

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN_SRC="$REPO_ROOT/target/release/flyingsquirrel"

if [[ ! -x "$BIN_SRC" ]]; then
    echo "ERROR: $BIN_SRC not found. Build first:" >&2
    echo "       cd $REPO_ROOT && cargo build --release --features hw-i2c,journald" >&2
    exit 1
fi

echo "[1/5] Creating flyingsquirrel user + group..."
if ! getent passwd flyingsquirrel >/dev/null; then
    useradd --system --shell /usr/sbin/nologin --home-dir /var/lib/flyingsquirrel \
            --comment "FlyingSquirrel anti-spoofing daemon" flyingsquirrel
fi
# Add to groups needed for sensor access. dialout for /dev/ttyUSB*, i2c for /dev/i2c-*.
# `usermod -aG` is idempotent.
for grp in dialout i2c; do
    if getent group "$grp" >/dev/null; then
        usermod -aG "$grp" flyingsquirrel
    fi
done

echo "[2/5] Installing binary to /usr/local/bin/flyingsquirrel..."
install -m 0755 -o root -g root "$BIN_SRC" /usr/local/bin/flyingsquirrel

echo "[3/5] Setting up directories..."
install -d -m 0750 -o flyingsquirrel -g flyingsquirrel /var/lib/flyingsquirrel
install -d -m 0750 -o flyingsquirrel -g flyingsquirrel /var/log/flyingsquirrel
install -d -m 0755 -o root            -g root            /usr/share/doc/flyingsquirrel
install -m 0644 "$REPO_ROOT/README.md" /usr/share/doc/flyingsquirrel/README.md

echo "[4/5] Installing config (if not already present)..."
if [[ ! -e /etc/flyingsquirrel.toml ]]; then
    install -m 0640 -o root -g flyingsquirrel \
        "$REPO_ROOT/examples/sitl-copter.toml" /etc/flyingsquirrel.toml
    echo "       → /etc/flyingsquirrel.toml installed from examples/sitl-copter.toml"
    echo "         EDIT THIS FILE before starting the service — at minimum set"
    echo "         [boot_anchor].expected_home to your actual launch site."
else
    echo "       /etc/flyingsquirrel.toml already exists — preserved unchanged."
fi

echo "[5/5] Installing + enabling systemd unit..."
install -m 0644 "$REPO_ROOT/deploy/flyingsquirrel.service" \
                /etc/systemd/system/flyingsquirrel.service
systemctl daemon-reload

cat <<EOF

Install complete.

Next steps:
  1. Edit /etc/flyingsquirrel.toml — set expected_home and any deployment-
     specific paths/ports.
  2. Start the service:
       sudo systemctl start flyingsquirrel
  3. Watch the logs:
       sudo journalctl -u flyingsquirrel -f
  4. Enable on boot when ready:
       sudo systemctl enable flyingsquirrel

Operator reset (clears SPOOFED latch):
  sudo systemctl kill -s HUP flyingsquirrel
EOF
