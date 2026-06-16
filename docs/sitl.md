# Validating FlyingSquirrel against ArduPilot SITL

> ## ✅ VALIDATED — full closed loop, continuously re-checked in CI
>
> FlyingSquirrel runs end-to-end against **real ArduPilot ArduCopter SITL
> firmware** in a nightly + on-demand GitHub Actions job
> ([`.github/workflows/sitl.yml`](../.github/workflows/sitl.yml)), so the
> headline "validated against ArduPilot SITL" claim is *continuously* re-proven
> rather than a one-time manual result. The job stands up three containers on a
> private Docker network (ArduPilot SITL + the detector + a bidirectional MAVLink
> relay/choreographer), force-arms and flies the copter, injects a GPS spoof, and
> **fails unless the detector's own event log proves it did the work** — a
> native-EKF reaction alone cannot pass it.
>
> **The proven chain** (one CI run): clean airborne flight → ~400 m GPS spoof →
> detector escalates `Normal → Suspicious → Spoofed` (cumulative residual ≈ the
> injected offset) → detector severs GPS (`PARAM_SET GPS_TYPE=0`) + commands RTL
> (`MAV_CMD_NAV_RETURN_TO_LAUNCH`) → **the autopilot switches to an RTL-equivalent
> mode** → detector read-back returns **`ActionAcked`** (COMMAND_ACK + causal mode
> change + fresh-HEARTBEAT dwell while still armed), with a forensic dump
> capturing the real incident. The CI gate requires the `Spoofed` transition + an
> RTL action event + the autopilot's mode flip; recent runs additionally confirm
> the stronger `ActionAcked` read-back.
>
> **One-command reproduction** (Windows + Docker Desktop, or Linux + Docker):
> ```
> pwsh deploy/sitl/run-sitl-validation.ps1      # Windows
> bash deploy/sitl/run-sitl-validation.sh        # Linux
> ```
>
> ### ⚠️ Scope — what CI proves, and the one gap it does NOT
> CI injects the spoof at the **wire level**: the relay rewrites the lat/lon of
> every `GPS_RAW_INT` / `GLOBAL_POSITION_INT` it forwards to the detector
> (`--spoof-mode relay`, the default). This faithfully models a meaconing /
> position-offset attack *as the companion computer sees it*, and exercises the
> entire detection → sever → RTL → read-back path against real firmware and real
> MAVLink timing. It does **not** route the offset through ArduPilot's **EKF**, so
> it does not by itself prove the autopilot's own estimator was on the spoofed
> GPS, nor that the sever actually STOPS the aiding. That proof is the harness's
> `--spoof-mode param` path, **validated green as of SITL Phase 2b** (a second CI
> matrix leg, `SPOOF_MODE=param` in [`sitl.yml`](../.github/workflows/sitl.yml),
> independent of relay via `fail-fast: false`). It ramps `SIM_GPS_GLITCH_*`
> gradually so ArduPilot's EK3 FUSES the offset (a step is innovation-gated) and
> logs `EKF_STATUS_REPORT` throughout. The first run answered both halves of the
> open question: EK3 **fused** the ramp (`gps_glitching` stayed clear,
> `pos_horiz_variance` grew 0→6 m) AND the detector's GPS sever made the EKF
> **drop its absolute-position lane** (`pos_horiz_abs` cleared, `const_pos_mode`
> set) — i.e. `GPS_TYPE=0` actually stops the aiding, not just echoes the param.
>
> **Scope of param mode (important):** the detector reads the RAW receiver
> (`GPS_RAW_INT`), which `SIM_GPS_GLITCH` offsets in POSITION only (honest
> Doppler), so from the detector's view this is a NAIVE spoof caught by the
> position lane — param mode validates the EKF-fusion + sever-drops-lane path, NOT
> the velocity-aiding lane. The velocity-aiding lane (Phase 2a) is exercised
> end-to-end by the **`relay-smart`** matrix leg (SITL **Phase 2c**): the relay
> ramps lat/lon AND fakes a matching Doppler, so the position blend masks the
> residual and the velocity-aiding lane makes the catch on real SITL/MAVLink
> timing. That lane is also proven by the local `ConsistentDrift` characterization
> (`tests/scenario_consistent_drift.rs`).
>
> ### Bugs this validation found and fixed (invisible to `mavsim`)
> The whole point of testing against real firmware: `mavsim` was circular (it
> emitted exactly what our parser expected). Real ArduPilot exposed:
> - **V-IMU 🔴** — we only parsed `HIGHRES_IMU` (PX4); ArduPilot streams
>   `SCALED_IMU`. `--imu-source mav` got zero IMU → preflight hung forever.
> - **V-MAVVER 🔴** — we hard-coded MAVLink v2; ArduPilot emits v1 by default
>   → every frame rejected, detector totally deaf.
> - **W1 (V-SYNC) 🟡** — ArduPilot's ~10 Hz `SCALED_IMU` put GPS ~100 ms past
>   the IMU buffer; the 20 ms tolerance rejected every fix. Fixed with
>   velocity forward-extrapolation.
> - **W-RATE 🟡 (the `--imu-rate` footgun)** — the default 100 Hz sizes the IMU
>   integration-gap gate to 25 ms, but ArduPilot's ~10 Hz stream has ~100 ms
>   gaps, so the gate skipped every step and the detector silently failed open
>   (no GPS cross-check). The harness passes `--imu-rate 10`; real deployments
>   must set it to the link's actual raw-sensor stream rate.
> - **W-ARMED 🟡** — RTL read-back legitimately returned `ActionUnconfirmed`
>   against a *grounded, disarmed* SITL copter (RTL disarms instantly on the
>   ground, so there is no causal *armed* mode change to confirm). The detector
>   was correct to refuse certification; the harness now force-arms
>   (`MAV_CMD_COMPONENT_ARM_DISARM` param2=21196) + `DO_SET_HOME` and confirms the
>   copter is airborne before spoofing — which is what unlocked `ActionAcked`.
> - **Harness mechanics** — `--mav-no-source-filter` + `--mav-allow-any-source-port`
>   are needed because the relay forwards from its own container address/port, and
>   `ARMING_CHECK=0` enables the scripted SITL arm. **All three are SITL-only —
>   never production posture.** (The `--spoof-mode param` route additionally uses
>   `SIM_GPS_GLITCH_*`, which is in **degrees**, not meters.)
>
> ### Empirical answers to the open questions below
> - **Does the autopilot RTL?** YES — observed an RTL-equivalent `custom_mode`
>   after the detector's command (the verifier also accepts LAND/SMART_RTL).
> - **Does the spoof reach the detector?** YES — the relayed `GPS_RAW_INT` /
>   `GLOBAL_POSITION_INT` carried the wire-level offset and the detector saw a
>   ~400 m cumulative residual.
> - **Does verification pass on real timing?** YES — with the copter force-armed
>   and airborne, the read-back returns `ActionAcked` within the dwell window.
>
> The notes below remain as the manual / from-source reproduction path (including
> the `--spoof-mode param` EKF route) and the original open-questions worksheet.

---

End-to-end testing against `mavsim` (our fake autopilot) proves the
wire protocol works. It does NOT prove that real ArduPilot reacts the
way we expect when we send `PARAM_SET GPS_TYPE=0` + `MAV_CMD_NAV_RETURN_TO_LAUNCH`
mid-flight. This document is the reproduction recipe for closing that gap.

The vision audit flagged this as the single biggest gap between
"passes synthetic tests" and "ready to defend a real drone." **As of Phase
W4 that gap is closed — see the VALIDATED banner above.**

## Open questions to answer

When you run the steps below, observe the autopilot and capture what
actually happens for each of these:

1. **Does `PARAM_SET GPS_TYPE=0` apply mid-flight?** Real ArduPilot may
   ignore parameter changes for some params during flight, or require
   a specific `MAV_MISSION_TYPE`. We expect the autopilot to disable
   its primary GPS driver; observe via `params show GPS_TYPE` in
   mavproxy after the spoof event.

2. **Does the autopilot transition to RTL, LAND, or refuse?** RTL on
   ArduCopter requires a usable position estimate. After we kill GPS
   via `GPS_TYPE=0`, the EKF may force `LAND` instead of `RTL` because
   it can't compute a return path without position. If that's what
   happens, our `ActionAcked` path needs to accept `LAND` as a valid
   "the autopilot got the message" outcome (it already does — see
   `ARDUCOPTER_MODE_LAND` in `src/mav/monitor.rs`).

3. **Does our verification pass on real timing?** The 5s verification
   window assumes autopilot HEARTBEAT at ≥1Hz with mode transition
   within ~1 cycle. Real ArduPilot HEARTBEAT cadence varies under
   load — we may need to lengthen the window.

4. **Does the EKF lane-switch back to inertial-only?** ArduPilot's
   EKF3 has multiple position sources via `EK3_SRC*` parameters.
   When GPS is killed, the lane should fall back to optical flow /
   external pose / inertial-only depending on the airframe. A
   research finding: if no fallback source is configured, the EKF
   will halt and the autopilot will force LAND. If your airframe
   has no fallback, expect LAND.

## Prerequisites

You need a Linux host to build ArduPilot SITL. On Windows, use WSL2
with Ubuntu 22.04 or newer. On Mac, a Linux VM. Native Mac builds
of ArduPilot are not officially supported.

```bash
# WSL2 first-time setup (Windows host):
wsl --install Ubuntu-22.04
wsl
```

## Build ArduPilot SITL (one-time, ~20 minutes)

```bash
sudo apt-get update
sudo apt-get install -y git python3-pip python3-venv

cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# Build SITL for Copter (~5-10 min):
./waf configure --board sitl
./waf copter
```

If you're already familiar with ArduPilot SITL and have a working
build, skip to "Run the smoke test" below.

## Run the smoke test

The script [`docs/sitl_smoke.sh`](sitl_smoke.sh) automates the
end-to-end test. It assumes ArduPilot SITL is built and the
FlyingSquirrel binary has been compiled for the host with
`cargo build --release`.

```bash
# From a checked-out flyingsquirrel repo on Linux:
cargo build --release

# Then from the repo root:
bash docs/sitl_smoke.sh
```

### What the smoke test does

1. Starts `sim_vehicle.py -v ArduCopter` with `--out=udp:127.0.0.1:14551`
   so MAVLink is forwarded to FlyingSquirrel.
2. Waits for the SITL drone to be ready (heartbeat + GPS fix).
3. Configures the drone: arms, takes off to 10m, switches to GUIDED.
4. Starts FlyingSquirrel with `--config examples/sitl-copter.toml`,
   pointing at the SITL's MAVLink endpoint.
5. Injects a synthetic GPS spoof using ArduPilot's `SIM_GPS_GLITCH_*`
   parameters (this is the real test — we're spoofing the autopilot's
   own simulated GPS, not faking the wire signal).
6. Observes:
   - FlyingSquirrel transitions Normal → Suspicious → Spoofed.
   - FlyingSquirrel sends `PARAM_SET GPS_TYPE=0` + `MAV_CMD_NAV_RETURN_TO_LAUNCH`.
   - SITL's flight mode changes (RTL or LAND).
   - FlyingSquirrel's `verify_rtb_engaged` returns `ActionAcked` or
     `ActionUnconfirmed`.

### Expected outcome

If everything works, you'll see something like this in the FlyingSquirrel logs:

```
[INFO] PREFLIGHT: passed — detector arming
[INFO] PilotModeChange  current_custom_mode=4   (GUIDED)
[INFO] StateTransition  to=Suspicious
[INFO] Drift            reason=NorthPositive
[WARN] StateTransition  to=Spoofed
[WARN] MAV ACTION: PARAM_SET GPS_TYPE=0 (x3)
[WARN] MAV ACTION: MAV_CMD_NAV_RETURN_TO_LAUNCH (x3)
[INFO] PilotModeChange  current_custom_mode=6   (RTL)   ← or 9 (LAND)
[INFO] MAV VERIFY: autopilot confirmed RTL (ACK + causal mode change + fresh-HB dwell)
[INFO] ActionAcked
```

If you get `ActionUnconfirmed`, capture the autopilot log
(`logs/00000001.BIN` in the SITL output directory) and the
FlyingSquirrel JSON event log (`spoof-events.jsonl`) and file an
issue describing:

- Autopilot version (`Tools/scripts/git_describe_version.py`)
- Vehicle profile (Copter / Plane / etc.)
- The final `custom_mode` observed in the JSON log
- Whether `EK3_SRC*` params were at their defaults
- Whether the autopilot was airborne or on the ground when the spoof fired

## Manual reproduction (if you want to drive each step yourself)

In a tmux session:

```bash
# Pane 1 — SITL:
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map --out=udpout:127.0.0.1:14551

# Pane 2 — wait for "APM: AHRS: EKF3 IMU0 is using GPS" in pane 1, then arm + takeoff:
# Use the mavproxy prompt in pane 1:
#   STABILIZE> mode guided
#   STABILIZE> arm throttle
#   STABILIZE> takeoff 10
#   STABILIZE> mode auto    # or stay in guided; either works

# Pane 3 — FlyingSquirrel:
cd /path/to/flyingsquirrel
cargo run --release -- \
    --config examples/sitl-copter.toml \
    --mav-bind 127.0.0.1:14551 \
    --mav-target 127.0.0.1:14550 \
    --expected-home "-35.363261,149.165230" \
    --duration 120

# Pane 1 again — inject the spoof via SITL parameters. NOTE: SIM_GPS_GLITCH_*
# is in DEGREES, not meters, and X = latitude (north), Y = longitude (east):
#   STABILIZE> param set SIM_GPS_GLITCH_Y 0.0044  # ~400m east at this latitude
# (~400m east ≈ 400 / (111320 · cos(lat)) degrees; at -35.36° that's ~0.0044.
#  Shift X then Y over time to create a gradual diagonal walk.)
```

## Common SITL gotchas

- **No GPS lock at startup.** SITL needs ~30 seconds to acquire a "lock"
  in simulation. Wait for the autopilot to print "GPS 1: detected as
  u-blox" before starting FlyingSquirrel.
- **`--out=udpout` vs `--out=udp`.** ArduPilot uses `udpout:` for
  outbound MAVLink to a specific peer. FlyingSquirrel listens on
  `--mav-bind`, so use `udpout:127.0.0.1:14551`.
- **Spoof too small to detect.** A glitch of `0.0001°` (~11 m) is near the
  per-axis CUSUM noise floor. Use ~0.0005°+ (~50 m) for jump scenarios or a
  steady ramp for drift. (Remember the param is degrees, not meters.)
- **SITL HEARTBEAT cadence.** Real SITL emits HEARTBEAT at ~1Hz under
  no load. Under heavy MAVLink traffic it can dip to 0.5Hz. The
  3-second LinkDown watchdog should tolerate this, but you'll
  occasionally see transient `LinkDown` / `LinkRestored` events on
  slow hosts.
- **Source-port lock.** SITL's MAVLink router (and mavproxy `--out udp:`)
  often uses ephemeral source ports that don't match the expected
  `--mav-target` port. The detector now makes this LOUD instead of silently
  dropping ("HEARTBEAT REJECTED: peer is sending from an unexpected source
  port…"). Fix it by either pointing the bridge at the expected port
  (mavproxy `--out udpin:`) or passing `--mav-allow-any-source-port` to lock
  on IP alone (acceptable in SITL; weakens the bootstrap defense in
  production). `--mav-no-source-filter` additionally drops the source-IP
  check, which the relay harness needs because it forwards from its own
  address.

## Running against PX4 SITL

**Status (be precise about what is and isn't validated):**

- ✅ **Detector PX4 support is implemented and unit-tested.** The PX4 profile
  drives: the GPS-sever parameter (`EKF2_GPS_CTRL=0`, not ArduPilot's
  `GPS_TYPE` — see `sever_gps_param`), RTL via `DO_SET_MODE` with the
  `CUSTOM_MODE_ENABLED` bit, the packed `(main<<24)|(sub<<16)` custom-mode
  decode (`is_rtl_mode` / `px4_split_mode`), and the `MAV_AUTOPILOT_PX4`
  cross-check. Covered by `sever_param_is_profile_specific`,
  `rtl_mode_recognition_px4`, `px4_mode_encoding_round_trips`, and the
  `proptest` PX4-codec invariants.
- ✅ **The manual run is documented** (below) and works against a PX4 SITL you
  bring up yourself.
- ⏳ **A containerized one-command PX4 harness** (the PX4 sibling of
  `deploy/sitl/run-sitl-validation.sh`) and an **actual end-to-end PX4 SITL
  run** are NOT done. The ArduPilot harness is ArduPilot-specific (its SITL
  container image, env, and `SIM_GPS_GLITCH_*` spoof injection don't carry over
  to PX4's Gazebo/jMAVSim stack). PX4 SITL also can't be exercised from the
  Windows dev box this was built on. **This is the remaining "PX4 SITL
  milestone": run the manual path below against live PX4 SITL, record the
  observed mode sequence in a `## Findings` section, and (if it works) build a
  containerized PX4 harness + a `sitl-px4.yml` CI job.** The relay-mode spoof in
  `sitl_harness.py` is autopilot-agnostic, so a PX4 containerized harness can
  reuse it once a working PX4 SITL image is wired up.

PX4 has its own SITL stack (uses Gazebo or jMAVSim instead of ArduPilot's
in-process simulator). The FlyingSquirrel side is identical — only the
`--vehicle` flag changes.

```bash
# One-time PX4 SITL build:
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl jmavsim
# (Or `make px4_sitl gazebo` for the Gazebo simulator.)

# When the simulator is up (you'll see "[mavlink] partner IP: ..." in the
# console), PX4 emits MAVLink on UDP 14540 by default. Forward to FlyingSquirrel:
#
#   In the px4 shell:  mavlink stream -u 14540 -s HEARTBEAT -r 1
#                      mavlink stream -u 14540 -s ATTITUDE  -r 100
#                      mavlink stream -u 14540 -s GPS_RAW_INT -r 5
#   (Default streaming usually has these on; adjust if absent.)
#
# Or use `mavproxy.py --master=udp:127.0.0.1:14540 --out=udpout:127.0.0.1:14551`.

# Run FlyingSquirrel against PX4 SITL:
cargo run --release -- \
    --gps-source mav --imu-source mav --controller mav \
    --vehicle px4 \
    --mav-bind 127.0.0.1:14551 --mav-target 127.0.0.1:14540 \
    --expected-home "47.397742,8.545594"   # PX4 default sim home
```

### PX4-specific open questions

These mirror the ArduPilot open questions and need empirical answers:

1. **Does PX4 accept `DO_SET_MODE` mid-flight?** PX4 generally honors
   mode changes from any sysid/component combo, but some firmware
   builds have stricter checks. Confirm `COMMAND_ACK::DO_SET_MODE`
   with `MAV_RESULT_ACCEPTED` arrives.
2. **Does PX4 transition to RTL or fall back to LAND on GPS loss?**
   PX4's commander has a `COM_FAIL_NAV` / `COM_OBS_AVOID` interaction
   that may force LAND. Our verifier accepts both (`AUTO_RTL` and
   `AUTO_LAND` are RTL-equivalents in `is_rtl_mode`).
3. **Does the `CUSTOM_MODE_ENABLED` bit suffice?** Some PX4 builds also
   require the `MANUAL_INPUT_ENABLED` bit cleared. If verification
   times out, capture the autopilot's HEARTBEAT and see what
   `base_mode` bits it reports.

The `docs/sitl_smoke.sh` script defaults to ArduPilot. To smoke-test PX4
instead, edit the script to set `--vehicle px4` and point at the PX4
SITL endpoint.

### Findings — live PX4 SITL (2026-06-14)

Run via the containerized PX4 path (`deploy/sitl/run-px4-validation.sh` +
`.github/workflows/sitl-px4.yml`, using the headless `jonasvautherin/px4-gazebo-headless`
image, which PUSHES MAVLink to a target). The detector reaches preflight Ready on
PX4's HEARTBEAT/GPS_RAW_INT/HIGHRES_IMU, detects a spoof, severs
`EKF2_GPS_CTRL=0` (read-back **verified**), and commands RTL; PX4 reaches AUTO.RTL
and the closed-loop gate passes. Two concrete answers to the open questions:

- **PX4 `custom_mode` byte order (a real bug, now fixed).** The observed wire
  values are AUTO.LOITER=`0x03040000`, AUTO.TAKEOFF=`0x02040000`,
  AUTO.RTL=`0x05040000` — i.e. `main_mode` is bits 16–23 and `sub_mode` is bits
  24–31 (the `px4_custom_mode` union). The detector's `px4_split_mode` had these
  SWAPPED, so a real PX4 RTL decoded as a non-AUTO mode and verification reported
  `ActionUnconfirmed` even though PX4 had RTL'd. Fixed + pinned with
  `px4_split_mode_matches_real_sitl_wire_values`. The self-consistent round-trip
  unit test could not have caught this.
- **Q2 answered: PX4 LANDs/disarms on GPS sever (no fallback).** With
  `EKF2_GPS_CTRL=0` and no non-GPS position source configured, PX4 loses its
  position estimate and cannot hold/navigate an armed RTL — it lands and disarms.
  So the detector's RTL read-back is `ActionUnconfirmed` (its armed cross-check
  correctly refuses to certify RTL on a disarming autopilot; `causal_mode` reads
  `armed=false` while the mode is AUTO.RTL). The closed-loop GATE still passes
  (Spoofed + sever-verified + PX4 reaches an RTL-equivalent), but the STRONG
  `ActionAcked` read-back on PX4 needs a non-GPS position fallback
  (`EKF2_*` optical-flow / external-vision) so PX4 can stay airborne+armed
  through the verify dwell — a deferred follow-up. (ArduCopter holds armed RTL
  because its EKF tolerates the `GPS_TYPE=0` sever differently and stays
  navigable.)
- **ActionAcked path (experimental, non-gating CI).** The detector is correct —
  `ActionUnconfirmed` on a disarming autopilot is the deliberate C-15 safety
  trade-off (relaxing the armed gate would weaken the W-ARMED defence), so the
  fix is harness-side, not detector-side. `sitl_harness.py --px4-ev-fallback`
  configures EKF2 to fuse external vision (`EKF2_EV_CTRL=11` / `EKF2_AID_MASK|=8`)
  and streams a **true-position** `VISION_POSITION_ESTIMATE` at ~30 Hz (NED
  relative to a captured origin; the EV goes only harness→PX4, the detector never
  sees it). With a non-GPS position source, PX4 should hold an armed RTL after the
  `EKF2_GPS_CTRL=0` sever and the read-back should reach `ActionAcked`. This runs
  as a **`continue-on-error`** job (`px4-action-acked` in
  [`sitl-px4.yml`](../.github/workflows/sitl-px4.yml)) with `REQUIRE_ACTION_ACKED=1`
  so it can never regress the proven `px4-closed-loop` gate while the EV-fusion
  setup is tuned. **Status: implemented, pending nightly-CI validation** — PX4
  SITL can't run on the Windows dev box, so the EKF2-EV params / message frames
  may need an iteration or two against the live Gazebo image.

## When SITL passes, what's next

A passing SITL smoke test answers the four open questions above and
gives us empirical confidence the protocol-level integration matches a
real autopilot. The next milestones, in order:

1. **Document the observed autopilot behavior** in this file — does it
   RTL or LAND? Does the EKF fall back gracefully? Capture the actual
   `custom_mode` sequence in a `## Findings (YYYY-MM-DD)` section.

2. **Adjust the code if necessary.** Likely candidates: lengthen the
   verify window if real timing demands it; add an `EK3_SRC*_POSXY=0`
   PARAM_SET to the sever-GPS sequence if the EKF refuses to
   lane-switch from `GPS_TYPE=0` alone.

3. **Bench test against real hardware.** Replace SITL with an actual
   Cube Orange + real u-blox GPS + an RF-quiet test range. The
   MAVLink protocol layer is exercised here; bench testing now
   covers the sensor-hardware paths and real flight dynamics.

4. **First powered flight test.** Drone tethered, operator hands on
   the controls. Inject the spoof from a portable GPS simulator
   (typically a HackRF + `gps-sdr-sim`). The detector should fire and
   the autopilot should RTL/LAND. Operator immediately re-takes
   manual control if anything looks wrong.
