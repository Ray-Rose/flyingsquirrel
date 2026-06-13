# FlyingSquirrel

[![CI](https://github.com/Ray-Rose/flyingsquirrel/actions/workflows/ci.yml/badge.svg)](https://github.com/Ray-Rose/flyingsquirrel/actions/workflows/ci.yml)

**GPS / IMU drift cross-correlator. Detects spoofed GPS in flight, severs the GPS link, and engages Return-to-Launch on inertial sensors.**

Signal-capture attacks like the RQ-170 incident in 2011 work because the
autopilot trusts its GPS coordinates blindly. A spoofer feeds false
coordinates that drift gradually from reality — each fix looks plausible
relative to the last, so naive sanity checks pass. FlyingSquirrel sits
alongside the autopilot, dead-reckons an independent position estimate
from the IMU, and flags the moment GPS diverges from physical reality.
On detection it disables the autopilot's GPS source and commands a
return-to-launch over MAVLink — verified by a four-gate closed-loop
check, so a single forged packet can't fake "all clear."

The system is intended for a Pi / Jetson companion computer alongside
ArduPilot or PX4, but the trait surface is hardware-agnostic — the same
binary runs against a fully synthetic simulator for development.

## Status

Supported autopilots (set via `--vehicle <profile>`):

- **`ardu-copter`** — ArduCopter on Pixhawk-class hardware. Uses
  `MAV_CMD_NAV_RETURN_TO_LAUNCH` for the RTL command and ArduPilot's
  flat custom_mode encoding.
- **`px4`** — PX4 on Pixhawk-class hardware (any vehicle type). Uses
  `MAV_CMD_DO_SET_MODE` with PX4's packed `(main<<24)|(sub<<16)` custom
  mode encoding; sets `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED` in base_mode
  as PX4 silently ignores DO_SET_MODE without that bit.

ArduPlane / ArduRover deferred — same protocol surface as ArduCopter
but different mode IDs; needs per-firmware test coverage before
shipping.

Real autopilot integration is **validated end-to-end against real
ArduPilot ArduCopter SITL firmware**: clean flight → injected 400 m GPS
spoof → detector escalates to `Spoofed` (≈400 m residual) → commands RTL →
the autopilot switches to RTL. One-command reproduction in
[`docs/sitl.md`](docs/sitl.md) (`deploy/sitl/run-sitl-validation.{ps1,sh}`).
This validation surfaced and fixed four real-firmware bugs that the `mavsim`
self-test could not (ArduPilot streams `SCALED_IMU` not `HIGHRES_IMU`, emits
MAVLink v1 not v2, runs a ~10 Hz IMU that needs residual-buffer
extrapolation, and uses `SIM_GPS_GLITCH_Y` in degrees). PX4 SITL validation
remains the next milestone.

## Quick start (no hardware)

```bash
cargo run --release -- --duration 60
```

Runs a 60-second synthetic flight with a `gradual-drift` spoof injected
at t=20s. You should see a `Spoofed` state transition around t=35s
followed by `MAV ACTION: ...` log lines confirming the controller fired.

## Deployment modes

| Mode | GPS source | IMU source | Controller | Use case |
|---|---|---|---|---|
| Synth (default) | synthetic | synthetic | console | development, regression tests |
| SITL | mavlink | mavlink | mavlink | validation against ArduPilot SITL |
| Sidecar | serial | i2c | mavlink | production: companion computer with real sensors talking to a real Pixhawk over MAVLink |
| Hardware-only | serial | i2c | console | bench testing real sensors without an autopilot |

Each is selected via the `--gps-source`, `--imu-source`, `--controller`
CLI flags (or the equivalent fields in a TOML config). See
`flyingsquirrel --help` for the full surface.

## Configuration

Two ways to configure: CLI flags (everything has one), or a TOML config
file passed via `--config path.toml`. CLI flags explicitly set on the
command line override config-file values, so you can pin a known-good
deployment config in version control and override individual fields for
one-off tests.

Generate a starter config:

```bash
flyingsquirrel --print-config > starter.toml
```

A worked example for an ArduCopter SITL/live deployment is in
[`examples/sitl-copter.toml`](examples/sitl-copter.toml).

## Real-flight deployment (sidecar pattern)

The reference deployment is a Raspberry Pi 4 / Jetson Nano running
alongside the autopilot on a drone, reading sensors over USB-serial
(GPS) + I2C (IMU) and sending MAVLink commands to the Pixhawk over
UDP / a USB-telem radio.

### 1. Hardware

| Component | Recommended |
|---|---|
| Companion computer | Raspberry Pi 4 (2GB+) or Jetson Nano |
| Autopilot | Cube Orange / Pixhawk 6X (any ArduPilot-compatible) |
| GPS | u-blox NEO-M9N or M10N over USB-serial @ 38400 baud |
| IMU | BMI088 or ICM-42688 over I2C (MPU-6050 acceptable for testing) |
| Telemetry | SiK 433/915 MHz radio or direct UDP over WiFi |

### 2. Autopilot pre-flight parameters

#### ArduPilot (Copter / Plane / Rover)

| Param | Recommended | Why |
|---|---|---|
| `SYSID_THISMAV` | `1` (default) | Must match `--mav-target-system` |
| `SR2_HEARTBEAT` | `1` Hz | Source-port lock + HEARTBEAT watchdog depend on it |
| `SR2_RAW_SENS` | `4` Hz | IMU streaming rate — **set `--imu-rate` to match this** (it sizes the dead-reckoning integration gate; a mismatch stalls DR) |
| `SR2_POSITION` | `4` Hz | GPS streaming rate |
| `EK3_SRC1_POSXY` | `3` (GPS) | The default — what we expect to disable on spoof |
| `RTL_ALT` | `1500` (15m AGL, or per mission) | Default RTL altitude |
| `LAND_SPEED` | `30` (cm/s) | RTL → LAND descent speed |

#### PX4

PX4's parameter system uses different names. Equivalents:

| Param | Recommended | Why |
|---|---|---|
| `MAV_SYS_ID` | `1` (default) | Must match `--mav-target-system` |
| `MAV_X_MODE` (per stream) | `Onboard` or `Custom` | Enables RAW_SENS / POSITION / HEARTBEAT streaming to companion |
| `EKF2_AID_MASK` | `0x01` (GPS) | What we expect to disable on spoof (via `EKF2_GPS_CTRL` if newer) |
| `RTL_RETURN_ALT` | `30` (m) | RTL altitude |
| `RTL_LAND_DELAY` | `0` (immediate) | Land immediately after returning |
| `COM_RC_LOSS_T` | leave default | Independent failsafe; should NOT conflict with our RTL |

Specifically for PX4: our `engage_rtb` sends `MAV_CMD_DO_SET_MODE` with
`base_mode = CUSTOM_MODE_ENABLED (0x01)`, `main = AUTO (4)`, `sub = RTL (5)`.
PX4 acknowledges with `COMMAND_ACK::DO_SET_MODE` (not
`NAV_RETURN_TO_LAUNCH`), which the verifier expects.

### 3. Build for the target

```bash
# On the Pi / target host (native):
git clone <this-repo>
cd flyingsquirrel
cargo build --release --features hw-i2c,journald

# Or cross-compile from a Linux dev workstation:
bash deploy/cross-compile.sh aarch64   # Pi 4/5, Jetson
bash deploy/cross-compile.sh armv7     # older Pi / BeagleBone

# Then install + enable as a systemd service:
sudo bash deploy/install.sh
```

### 3a. Docker alternative

```bash
docker build -f deploy/Dockerfile -t flyingsquirrel:latest .
bash deploy/docker-run.sh
```

### 4. Run

```toml
# /etc/flyingsquirrel.toml
[sources]
gps_source = "serial"
imu_source = "i2c"
controller = "mav"

[serial]
gps_port = "/dev/ttyUSB0"
gps_baud = 38400

[i2c]
imu_bus = "/dev/i2c-1"
imu_addr = 0x68
imu_rate = 100.0

[mav]
bind = "0.0.0.0:14551"          # listen on all interfaces
target = "127.0.0.1:14550"      # mavproxy / autopilot endpoint
target_system = 1
target_component = 1
vehicle = "ardu-copter"

[boot_anchor]
# Operator MUST set this to the actual launch coordinates each mission.
# A first GPS fix outside `max_home_distance_m` of this point is rejected.
expected_home = "40.7128,-74.0060"
max_home_distance_m = 1000.0

[process]
duration = 7200   # 2-hour mission cap; tune to your endurance
log_level = "info"
json_log = "/var/log/flyingsquirrel/events.jsonl"
```

```bash
sudo flyingsquirrel --config /etc/flyingsquirrel.toml
```

A `systemd` unit, installer, and Dockerfile are in [`deploy/`](deploy/) —
see steps 3 and 3a above.

### 5. Operator reset (clearing a `Spoofed` latch)

`SIGHUP` (Unix) or `Ctrl-Break` (Windows). The FSM returns to `Normal`
and detector state is fully cleared:

```bash
pkill -HUP flyingsquirrel
```

This is for legitimate false-positive recovery only. SPOOFED latches on
purpose — without that, a smart attacker who drops the spoof exactly
when the autopilot reacts could re-engage GPS just in time to be
recaptured.

## Spoofing-event reference

The detector emits structured events on a broadcast bus. They show up in
the JSON log (`--json-log`) and in the `tracing` output. Each carries a
monotonic timestamp, the FSM state, the residual magnitude, and a
JSON-typed `detail` payload.

| Event | Meaning |
|---|---|
| `Jump` | Instantaneous position OR velocity discontinuity (likely teleport spoof) |
| `Drift` | Cumulative CUSUM exceeded threshold (slow walk-off, possibly circular if magnitude axis fires) |
| `FrozenGps` | Same lat/lon for multiple fixes while IMU shows motion (stuck module or replay) |
| `StateTransition` | FSM moved between `Normal` ↔ `Suspicious` ↔ `Spoofed` |
| `SyncWarning` | GPS fix outside IMU ring-buffer window, or persistent missing Doppler |
| `BootAnchorRejected` | First fix (or post-Susp clear) too far from expected/dead-reckoned position |
| `DwellPauseExceeded` | GPS dropout streak exceeded cap; FSM dwell timer resumed |
| `LinkDown` / `LinkRestored` | Autopilot HEARTBEAT silence past the watchdog |
| `PilotModeChange` | Autopilot flight-mode transitioned (correlate with `MAV ACTION` logs to distinguish ours from operator/failsafe) |
| `ActionFailed` | A controller call (`sever_gps` / `engage_rtb`) failed to put bytes on the wire |
| `ActionAcked` | Closed-loop verification: ACK matched + causal mode change + fresh-HB dwell — drone is recovering |
| `ActionUnconfirmed` | RTL was sent but autopilot did not transition to RTL within verification window |
| `PreflightFailed` / `PreflightPassed` | Self-test gate (rate-limited; `Passed` fires once on transition) |

## Architecture

```
       Sensors                           Controller
   ┌─────────────┐                     ┌─────────────┐
   │ GpsSource   │  ───┐         ┌─── │ FlightCtlr  │
   │ (synth/mav/ │     │  Fusion │     │ (console/   │
   │  serial)    │     ├─────────┤     │  mavlink)   │
   ├─────────────┤     │  task   │     ├─────────────┤
   │ ImuSource   │  ───┘         └─── │ MavMonitor  │
   │ (synth/mav/ │                     │ (verifier)  │
   │  i2c)       │                     │             │
   └─────────────┘                     └─────────────┘
        │                                    │
        ▼                                    ▼
   mpsc channels                  broadcast<SpoofingEvent>
                                            │
                                            ▼
                                  Operator-facing events
                                  (tracing, JSON log)

Inside the Fusion task:
   GPS fix  ──►  preflight gate  ──►  residual = GPS − DR(IMU)
   IMU sample  ──►  Madgwick attitude  ──►  strapdown integration  ──►  DR
                                                  │
   Residual ──►  JumpDetector (velocity + hard)   │
              ──►  DriftDetector (4 per-axis CUSUM + magnitude CUSUM)
              ──►  FrozenGps detector
              ──►  StateMachine (Normal → Suspicious → Spoofed-latched)
                       │
                       ▼ on Spoofed
            try_action(sever_gps) → try_action(engage_rtb) → verify_rtb_engaged
                                                                  │
                                                                  ▼
                                                            ActionAcked
                                                            ActionUnconfirmed
```

## Defenses at a glance

The detector and controller together close the following attack classes:

- **Sudden teleport** — JumpDetector (velocity mismatch + hard residual)
- **Gradual / smooth drift** — Per-axis CUSUM + adaptive magnitude CUSUM (catches circular drift; noise floor learned online so it doesn't false-fire on real GPS)
- **Vertical / altitude spoof (sudden)** — GPS-only altitude-rate sanity check (>30 m/s apparent climb/descent), independent of vertical dead-reckoning
- **Frozen / replayed GPS** — `FrozenGps` detector (lat/lon identical while IMU shows motion)
- **Jam-then-spoof timing attacks** — FSM dwell freeze during GPS dropouts, capped streak
- **Meaconing at boot** — Boot-anchor cross-check against operator's expected home
- **Wait-out-then-anchor** — Re-anchor plausibility veto on Suspicious → Normal (cumulative dwell preserved across vetoes)
- **MAVLink injection** — Source-IP + source-port lock + sysid filter, with bootstrap port check (a wrong-port bootstrap is now a LOUD warning, not a silent drop; `--mav-allow-any-source-port` opts into IP-only locking for ephemeral-port bridges)
- **Unconfirmed GPS-sever (RTL on spoofed GPS)** — Closed-loop sever read-back: confirms the autopilot echoed the GPS-disable param as applied (`PARAM_VALUE`), in parallel with RTL verification; an unconfirmed sever raises a CRITICAL event
- **Cadence-modulated dropout stall** — Cumulative per-episode dwell-pause budget so a spoofer toggling GPS rate can't freeze Susp→Spoofed escalation indefinitely
- **Single-packet RTL-confirm bypass** — Four-gate verify: ACK matches command + causal mode change + dwell + fresh HEARTBEATs between polls
- **ACK stomping during RTL verify** — Per-command-id ACK lookup, immune to unrelated traffic in the verify window
- **Disarmed-but-mode-RTL spoof** — Verification additionally requires `MAV_MODE_FLAG_SAFETY_ARMED`
- **Friendly-fire RTL from a synth scenario** — CLI refuses `synth source + mav controller` unless `--allow-synth-to-mav`
- **TOML config silently enabling safety bypasses** — Safety-bypass bools can only be enabled with an explicit CLI flag; rejected at TOML load time
- **Sever-fails-then-RTL leaves drone on spoofed GPS** — Critical-warning event fires between failed sever and RTL so the operator knows to take manual control
- **Action latches stuck after operator reset** — `FlightController::reset()` clears them; Fusion also clears `last_gps_t_secs` / `last_gps_lla` / `frozen_fix_streak` / `no_doppler_streak`
- **NaN / out-of-range injection** — Plausibility gates on every ingest path
- **MAVLink "unknown" sentinel poisoning** — UINT16_MAX in `vel`/`cog`/`eph`/`sats` translated to `None` so the detector treats them as missing evidence rather than 655 m/s garbage
- **Attacker-controlled HDOP defeating multipath gate** — Widening requires BOTH `hdop > threshold` AND `sats < threshold`, both fields present
- **CUSUM reset via IMU-stall** — `on_clean_anchor()` no longer fires on post-Ready interpolate failure, so slow-drift evidence persists across attacker-induced buffer gaps
- **Forensic dump lost on `systemctl stop`** — SIGTERM handler + shared `JoinSet` drained with a 7 s bounded timeout before exit
- **Serial GPS USB blip kills process** — Reconnect-with-backoff loop inside the GpsSource
- **I²C IMU bus down silently** — Consecutive-error counter emits DEGRADED warning at 100 errors, exits stream at 1000 (systemd restarts)
- **Operator launch coordinates leaked at INFO level** — `lat`/`lon` demoted to DEBUG; INFO line shows only `radius_m` + `home_configured=true`
- **Console controller silently false-confirming RTL** — `verify_rtb_engaged` overridden to honestly return `Ok(false)` ("cannot observe an autopilot")
- **NMEA OOM via no `\n`** — Line cap in serial GPS decoder
- **Task panic / silent degradation** — Supervisor exits non-zero on any task death
- **Forensic disk-fill DoS** — Once-per-process dump flag

**Full attack catalog with code sites and proof tests:** see
[`docs/threats.md`](docs/threats.md) — every defense above maps to a
specific `file.rs:line-range` and a unit or integration test that
demonstrates the defense holds.

## Troubleshooting

**"--vehicle is required when any MAV source or controller is in use"**
Set `[mav].vehicle = "ardu-copter"` in your config or pass `--vehicle ardu-copter`.

**"production MAV deployment requires --expected-home"**
Set `[boot_anchor].expected_home = "lat,lon"` (decimal degrees). Or, for
ad-hoc field tests where the launch site changes per run, pass
`--allow-no-boot-anchor-check` — but the meaconing-at-boot defense is
inactive in that mode.

**"real deployment requires an explicit run duration"**
Any non-synth deployment (serial / i2c / mav) must set a run duration, so a
forgotten `[process].duration` can't silently exit the daemon mid-flight (the
old 60 s default did exactly that, and `Restart=on-failure` does not restart a
clean exit). Set `[process].duration` to a mission cap in seconds (e.g. `7200`),
or `0` to run until stopped by signal (Ctrl-C / SIGTERM / SIGHUP). Synthetic
scenarios are unaffected (they end their own stream).

**Dead-reckoning never runs / sustained `SyncWarning` on a MAV link**
The IMU integration gate is sized from `--imu-rate`. If it defaults to 100 Hz
but the autopilot streams `SCALED_IMU` at 4–10 Hz, every integration step is
skipped and you'll see a loud "DEAD-RECKONING STALLED" warning. Set `--imu-rate`
(or `[i2c].imu_rate`) to the autopilot's raw-sensor stream rate — i.e. match
ArduPilot's `SR2_RAW_SENS` (or the PX4 equivalent).

**"refusing to pair a synthetic GPS/IMU source with the MAV controller"**
You're about to send a real GPS-sever `PARAM_SET` (`GPS_TYPE=0` on ArduPilot,
`EKF2_GPS_CTRL=0` on PX4) + RTL to a real autopilot from a simulated attack.
Either point `--mav-target` at a SITL endpoint, or pass `--allow-synth-to-mav`
for an HIL test.

**`ActionUnconfirmed` events**
The autopilot didn't transition to RTL within the verification window
(default 5s). Real ArduPilot may refuse RTL if `EK3_SRC*_POSXY` was set
to 0 too aggressively — see [`docs/sitl.md`](docs/sitl.md) for the
known unknowns.

**`SyncWarning` events at startup**
Expected during the first second or two while the IMU buffer fills.
Sustained `SyncWarning` typically means a GPS-IMU clock-skew problem
or the GPS is producing fixes faster than the buffer window
(default 2s).

**`LinkDown` after a working session**
Autopilot HEARTBEAT silent for >3s. Check the telemetry radio, the
MAVLink endpoint, and the autopilot's `SR2_HEARTBEAT` param.

**Source-port lock contention / "HEARTBEAT REJECTED: unexpected source port"**
The first valid HEARTBEAT pins (IP, port) and rejects anything else. If you see
the loud "HEARTBEAT REJECTED … unexpected source port" warning, the detector is
receiving nothing because a bridge is forwarding from an ephemeral port
(mavproxy `--out udp:`, mavlink-router, SiK). Either point the bridge at the
expected port (mavproxy `--out udpin:`) or pass `--mav-allow-any-source-port` to
lock on IP alone. Repeated "dropped mavlink: source-locked" (debug) instead means
another peer on the same IP is contending for the already-formed lock.

## Testing

```bash
cargo test                                 # 100 lib + 14 integration scenarios
cargo test --all-features                  # also compiles/tests the Linux-only I2C + journald paths
cargo clippy --all-targets --all-features -- -D warnings
cargo fmt --all --check
```

The integration tests cover:

- Clean flight (no spurious detection)
- Sudden-jump teleport
- Gradual-drift walk-off
- Velocity/position inconsistency (isolates the velocity-mismatch detector)
- Vertical / altitude-teleport spoof (GPS-only altitude-rate check)
- End-to-end MAVLink closed-loop against `mavsim` (ACK + causal mode + dwell verification)
- Boot-anchor rejection vs. expected-home
- Pre-flight gate Initializing → Ready transition
- Frozen-GPS (replay/stuck-module) detection
- Re-anchor plausibility veto on Susp → Normal
- Forensic ring-buffer dump-on-Spoofed (schema + atomic write)

Plus a deterministic Monte Carlo unit test that runs 600 s of realistic
σ=2.5 m GPS noise and asserts the adaptive drift detector produces **zero**
false alarms, while still firing on a genuine drift above the learned noise
floor.

## Continuous integration & releases

Two GitHub Actions workflows live in `.github/workflows/`:

**`ci.yml`** runs on every push and pull request:

| Job | What it gates |
|---|---|
| `fmt` | `cargo fmt --all --check` — formatting is enforced, not suggested |
| `test` | `clippy --all-targets --all-features -D warnings`, then `cargo test` with **default AND all features** + doc tests. The all-features run compiles the Linux-only `hw-i2c` / `journald` paths so they can't silently rot |
| `msrv` | builds against the declared MSRV (Rust 1.88, `--locked`) so `rust-version` can never drift from reality. 1.88 is the real floor (set by `darling`/`instability` via `ratatui`), determined empirically by building in Docker; this job exists because earlier unverified MSRV claims silently didn't build |
| `cross` | cross-compiles release binaries for `aarch64-unknown-linux-gnu` and `armv7-unknown-linux-gnueabihf` (the real deploy targets) with `hw-i2c,journald`, and verifies the ELF architecture |
| `audit` | `cargo audit` — fails on any RUSTSEC advisory in the dependency tree |
| `docker` | builds `deploy/Dockerfile` to validate the container image |

The all-features test job exists specifically to catch the class of bug where a
real-hardware code path breaks while the default test suite stays green — the
NMEA-feature regression (which made the serial-GPS backend silently produce
zero fixes) is the canonical example, and it now has a dedicated regression
test this job runs.

**`release.yml`** fires on a version tag (`git tag v0.1.0 && git push --tags`):
it re-verifies fmt + clippy + tests, cross-compiles release binaries for
aarch64 / armv7 / x86_64, computes a **SHA-256 manifest**, and publishes a
GitHub Release. The `flyingsquirrel` binary's hash in `SHA256SUMS` matches the
`sha256=` field the binary prints in its startup `ATTESTATION` log line — so an
operator can cross-check that the binary running on the drone is exactly the
released artifact. `mavsim` (the attack-trajectory test fixture) is deliberately
excluded from release artifacts.

## What's deferred

Items that are deliberately not in v1, with rationale. (PX4 vehicle profiles,
the forensic ring buffer, and the systemd unit / Dockerfile were on this list
in earlier drafts and have since SHIPPED — see the deployment sections above.)

| Item | Why deferred |
|---|---|
| MAVLink v2 signing | Substantial key-management story; source-port lock + sysid filter close most of the same attack surface. Operator opt-in is a clean future addition |
| Cumulative re-anchor dwell cap | Default sizing requires real-flight measurement (the audit specifically warned against shipping with a guess) |
| ArduPlane / Rover vehicle profiles | `is_rtl_mode` + RTL command shape differ per vehicle; ArduCopter and PX4 ship today, fixed-wing/rover need real-firmware testing first |
| Gradual altitude-drift detection | GPS altitude is inherently weak with no reliable inertial vertical reference; sudden vertical teleports ARE caught (see `docs/threats.md` §3.1) |
| Two-receiver GNSS diversity | Defeats a co-spoofed single receiver; a hardware + integration project of its own |

A full attack-class catalog with code sites, proof tests, and the honest
residual-risk list is in [`docs/threats.md`](docs/threats.md).

## License

(TBD — placeholder for operator to add.)
