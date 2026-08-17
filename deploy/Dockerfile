# Multi-stage Dockerfile for FlyingSquirrel.
#
# Build:   docker build -f deploy/Dockerfile -t flyingsquirrel:latest .
# Run:     see deploy/docker-run.sh
#
# The runtime image is debian-slim + the binary (~30MB). It runs as a
# non-root user (uid 1000) and expects three things mounted in:
#   /etc/flyingsquirrel.toml   — config file (read-only)
#   /var/lib/flyingsquirrel/   — runtime state (rw, for JSON event log)
#   host network for MAVLink UDP (or expose 14551/udp explicitly)

# -------- builder --------
# Rust 1.88 is the MSRV, empirically determined by building this exact locked
# dependency tree (Phase V, in Docker): darling 0.23 + instability 0.3.12
# (transitive via ratatui) require rustc 1.88; clap_lex 1.1 requires the
# edition2024 feature (1.85). 1.88 is the binding constraint. Keep this in
# sync with Cargo.toml `rust-version` and the CI `msrv` job.
# Digest-pinned for a reproducible, supply-chain-hardened build. The tag is kept
# for humans; Docker resolves by digest. Dependabot (.github/dependabot.yml,
# docker ecosystem) opens a PR when the base image updates, so the pin stays
# current instead of freezing security patches.
FROM rust:1.97-slim-bookworm@sha256:2775a09d208ff0d7c1f50490c45b62db929e87ba1dcbc3f2132ac71a704bcdd3 AS builder

# Build deps:
#   pkg-config / libudev-dev   — hw-i2c via linux-embedded-hal
#   libsystemd-dev             — tracing-journald (journald feature)
#   git                        — REQUIRED by the `mavlink` crate's build
#                                script, which runs `git submodule update
#                                --init` and `git apply` on its XML dialect
#                                patches. The crates.io package ships the XML
#                                definitions, so git only needs to EXIST (the
#                                submodule update is a no-op in a non-repo dir);
#                                without the binary the build dies with
#                                "No such file or directory (os error 2)".
#                                This is invisible on a dev box whose cargo
#                                registry already has the definitions unpacked.
RUN apt-get update && apt-get install -y --no-install-recommends \
        pkg-config \
        libudev-dev \
        libsystemd-dev \
        git \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build

# Cache dependencies separately from source. First copy just the manifests,
# build a dummy main, then copy real source — this layer-caches deps so a
# source-only change doesn't re-download crates.
COPY Cargo.toml Cargo.lock ./
RUN mkdir -p src && \
    echo "fn main() {}" > src/main.rs && \
    cargo build --release --features hw-i2c,journald 2>/dev/null || true
RUN rm -rf src

# Now the real source.
COPY src/ ./src/
COPY tests/ ./tests/
COPY examples/ ./examples/

# Release build with both features. hw-i2c needed for I2C IMU on Linux;
# journald for systemd integration if the host runs systemd-in-container.
RUN cargo build --release --features hw-i2c,journald --bin flyingsquirrel --bin mavsim

# -------- runtime --------
# Digest-pinned (see the builder stage note). Dependabot keeps it fresh.
FROM debian:bookworm-slim@sha256:abd67ffcfa541b485a3dff59865ab629aa048a6c613e639d36e7456b0b229241 AS runtime

# Runtime deps: libudev (hw-i2c via linux-embedded-hal), libsystemd
# (tracing-journald). ca-certificates for any HTTPS callers (future).
RUN apt-get update && apt-get install -y --no-install-recommends \
        libudev1 \
        libsystemd0 \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && groupadd --system --gid 1000 flyingsquirrel \
    && useradd --system --uid 1000 --gid 1000 --shell /usr/sbin/nologin \
               --home-dir /var/lib/flyingsquirrel \
               --comment "FlyingSquirrel daemon" flyingsquirrel \
    && mkdir -p /var/lib/flyingsquirrel /var/log/flyingsquirrel \
    && chown -R flyingsquirrel:flyingsquirrel /var/lib/flyingsquirrel /var/log/flyingsquirrel

# Copy the binary. mavsim is also bundled so the same image can be used
# for SITL-style smoke testing.
COPY --from=builder /build/target/release/flyingsquirrel /usr/local/bin/flyingsquirrel
COPY --from=builder /build/target/release/mavsim         /usr/local/bin/mavsim

# Example config (operator overrides via volume mount).
COPY examples/sitl-copter.toml /etc/flyingsquirrel.toml.example
COPY README.md                 /usr/share/doc/flyingsquirrel/README.md

USER flyingsquirrel
WORKDIR /var/lib/flyingsquirrel

# Default command: run with /etc/flyingsquirrel.toml. Operator MUST mount
# their own config; the .example file is a fallback so the container
# doesn't crash if no mount is provided.
ENTRYPOINT ["/usr/local/bin/flyingsquirrel"]
CMD ["--config", "/etc/flyingsquirrel.toml"]

# MAVLink UDP listener default (override via config).
EXPOSE 14551/udp

# Documentation labels.
LABEL org.opencontainers.image.title="flyingsquirrel"
LABEL org.opencontainers.image.description="GPS/IMU anti-spoofing detector for drones"
LABEL org.opencontainers.image.licenses="MIT OR Apache-2.0"
LABEL org.opencontainers.image.source="https://github.com/Ray-Rose/flyingsquirrel"
