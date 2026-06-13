#!/usr/bin/env bash
# deploy/cross-compile.sh — build flyingsquirrel for ARM targets from an
# x86_64 Linux dev host (Pi 4/5 + Jetson are aarch64; older Pi/BeagleBone
# are armv7).
#
# This is the script the README's "Build for the target" section points at.
# Operators on macOS / Windows can run this inside a Linux container:
#
#   docker run --rm -v "$PWD:/work" -w /work rust:1.88 \
#       bash deploy/cross-compile.sh aarch64
#
# Usage:
#   deploy/cross-compile.sh aarch64    # Pi 4/5, Jetson, modern ARM SBCs
#   deploy/cross-compile.sh armv7      # older Pi, BeagleBone
#   deploy/cross-compile.sh both       # builds both
#
# Output:
#   target/aarch64-unknown-linux-gnu/release/flyingsquirrel
#   target/armv7-unknown-linux-gnueabihf/release/flyingsquirrel

set -euo pipefail

TARGET_REQUESTED="${1:-aarch64}"

# Map shorthand to rustc target triples.
case "$TARGET_REQUESTED" in
    aarch64)  TARGETS=(aarch64-unknown-linux-gnu) ;;
    armv7)    TARGETS=(armv7-unknown-linux-gnueabihf) ;;
    both)     TARGETS=(aarch64-unknown-linux-gnu armv7-unknown-linux-gnueabihf) ;;
    *)
        echo "ERROR: unknown target '$TARGET_REQUESTED'." >&2
        echo "Usage: $0 [aarch64|armv7|both]" >&2
        exit 1
        ;;
esac

# Required: a Linux build host. macOS users should run this inside a Linux
# container; Windows users should use WSL2 or a Linux container.
if [[ "$(uname -s)" != "Linux" ]]; then
    cat >&2 <<EOF
ERROR: this script must run on Linux (cross-compile toolchains expect glibc).
       macOS:   docker run --rm -v "\$PWD:/work" -w /work rust:1.88 bash deploy/cross-compile.sh $TARGET_REQUESTED
       Windows: open WSL2 Ubuntu, then run the same script.
EOF
    exit 1
fi

echo "[1/4] Checking rustup and target installation..."
if ! command -v rustup >/dev/null 2>&1; then
    echo "ERROR: rustup not installed. https://rustup.rs/" >&2
    exit 1
fi
for tgt in "${TARGETS[@]}"; do
    if ! rustup target list --installed 2>/dev/null | grep -q "^$tgt$"; then
        echo "       Installing rustup target $tgt..."
        rustup target add "$tgt"
    fi
done

echo "[2/4] Checking cross linker..."
# Debian/Ubuntu cross-toolchain package names. Install via:
#   sudo apt install gcc-aarch64-linux-gnu gcc-arm-linux-gnueabihf
for tgt in "${TARGETS[@]}"; do
    case "$tgt" in
        aarch64-unknown-linux-gnu)
            if ! command -v aarch64-linux-gnu-gcc >/dev/null 2>&1; then
                cat >&2 <<EOF
ERROR: aarch64-linux-gnu-gcc not in PATH.
       Install on Debian/Ubuntu:
         sudo apt install gcc-aarch64-linux-gnu pkg-config
       Or in a container, add to your image:
         RUN apt-get install -y gcc-aarch64-linux-gnu pkg-config
EOF
                exit 1
            fi
            export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc
            export CC_aarch64_unknown_linux_gnu=aarch64-linux-gnu-gcc
            ;;
        armv7-unknown-linux-gnueabihf)
            if ! command -v arm-linux-gnueabihf-gcc >/dev/null 2>&1; then
                cat >&2 <<EOF
ERROR: arm-linux-gnueabihf-gcc not in PATH.
       Install on Debian/Ubuntu:
         sudo apt install gcc-arm-linux-gnueabihf pkg-config
EOF
                exit 1
            fi
            export CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABIHF_LINKER=arm-linux-gnueabihf-gcc
            export CC_armv7_unknown_linux_gnueabihf=arm-linux-gnueabihf-gcc
            ;;
    esac
done

echo "[3/4] Building..."
# Features: hw-i2c (real IMU on Pi/Jetson over /dev/i2c-1) +
#           journald (systemd journal integration).
FEATURES="hw-i2c,journald"
for tgt in "${TARGETS[@]}"; do
    echo "       Building flyingsquirrel for $tgt..."
    cargo build --release --features "$FEATURES" --target "$tgt"
done

echo "[4/4] Verifying ELF architecture..."
for tgt in "${TARGETS[@]}"; do
    BIN="target/$tgt/release/flyingsquirrel"
    if [[ ! -f "$BIN" ]]; then
        echo "ERROR: build did not produce $BIN" >&2
        exit 2
    fi
    if command -v file >/dev/null 2>&1; then
        echo "       $BIN  →  $(file -b "$BIN")"
    fi
    # Also report the binary size — operators care for image-baking.
    SIZE=$(du -h "$BIN" | cut -f1)
    echo "       Size: $SIZE"
done

cat <<EOF

Cross-compile complete.

Deploy to the target device:
  scp target/aarch64-unknown-linux-gnu/release/flyingsquirrel pi@DEVICE:/tmp/
  ssh pi@DEVICE 'sudo install -m 0755 /tmp/flyingsquirrel /usr/local/bin/'

Then follow deploy/install.sh on the target to set up the systemd unit and
the unprivileged user. (install.sh assumes the binary is already built —
copy this binary into target/release/ on the device first if you ran the
cross-compile on a different host.)
EOF
