# FlyingSquirrel — Threat Model

This document catalogs every attack class FlyingSquirrel is designed to
detect or resist, the defense that catches it, and the exact code site +
proof test that demonstrates the defense holds. Use it as:

- The checklist an integrator's security review starts from.
- The map from "what could go wrong" to "where in the source it's
  handled."
- The honest answer about what is **not** defended (see [Out of scope](#out-of-scope-and-accepted-risks)).

Audit findings that drove each defense are tagged inline (e.g. `B-24`,
`E-10`, `MAV-01`) so you can correlate this doc against `cargo test` and
`git log`.

---

## 1. Scope and threat actors

**What the system does.** FlyingSquirrel ingests two independent position
sources — GPS (NMEA serial, MAVLink, or synthetic) and IMU (I²C MPU-6050
class, MAVLink HIGHRES_IMU, or synthetic) — dead-reckons a predicted
position from inertial integration, compares it to the GPS-reported
position, and on divergence beyond physical plausibility severs the GPS
link and engages Return-to-Launch on inertial sensors alone.

**Threat actors considered.**

| Actor | Capability | Defended? |
|---|---|---|
| **GPS spoofer** | Can transmit attacker-chosen GNSS signals to the vehicle's receiver. Cannot observe / inject MAVLink or IMU. | ✅ Primary mission |
| **Meaconing GPS spoofer** | Records legitimate GPS and replays with a fresh timestamp. | ✅ Frozen-fix detector + boot-anchor cross-check |
| **MAVLink off-host adversary** | Can send UDP datagrams to the bind socket from another host on the LAN. Cannot observe local I/O. | ✅ Source-IP filter + sysid filter + source-port lock |
| **MAVLink co-located adversary** | Local-process attacker on the companion computer; can bind any UDP port and forge MAVLink. | ✅ Source-port lock (won't lock to non-configured port) — partial; full defense requires MAVLink message signing (operator opt-in, not enabled by default) |
| **Operator footgun** | Honest operator misconfigures or deploys carelessly. | ✅ Default-deny CLI guards (`--vehicle`, `--expected-home`, `--allow-*`), TOML safety-bypass rejection (F-01) |
| **Supply-chain adversary** | Compromise of upstream Rust crates, Docker base images, or build pipeline. | ⚠️ Partial: binary attestation logs SHA-256 at startup; Cargo.lock pins versions; **Docker base images not yet pinned by digest** (deferred) |
| **Local-root attacker** | Already has root on the device. | ❌ Out of scope. TPM-backed measured boot is the right primitive. |
| **Physical attacker** | Has direct access to the airframe. | ❌ Out of scope. |

**The detector is one defense, not the only one.** A complete air-vehicle
security posture also needs: MAVLink signing, secure firmware update,
secure-boot of the autopilot, physical tamper detection, ground-station
authentication. FlyingSquirrel addresses the GPS/IMU cross-check layer
specifically, leaving the others to other components.

---

## 2. How to read the attack tables

Each row has five columns:

- **Attack** — what the adversary does.
- **Defense** — the specific mechanism that catches or contains it.
- **Code site** — `path/to/file.rs:line-range` of the canonical
  implementation. Search the line for inline `AUDIT <id>` markers.
- **Proof test** — the unit or integration test that demonstrates the
  defense works. Run `cargo test <name>` to verify.
- **Residual risk** — what the defense does *not* cover. The honest
  remainder; not a complete list, but the cases most likely to bite.

If a row's defense is partial or under refinement, an `[deferred]` or
`[partial]` tag flags it in the residual-risk column with a link to the
follow-up issue.

---

## 3. GPS attack class catalog

### 3.1 Position attacks

| Attack | Defense | Code site | Proof test | Residual risk |
|---|---|---|---|---|
| **Sudden teleport** — GPS jumps by ≥50 m (ground) / 200 m (air) in one fix | Hard-residual jump detector compares `|gps_ned − dead_reckoned_ned|` against `max_jump_m` | `src/detect/jump.rs:58-60` | `fires_on_hard_residual`, `tests/scenario_jump.rs` | Sub-50m teleports slip past the hard threshold; the slow-drift CUSUM catches accumulated motion over several fixes |
| **Velocity-mismatch teleport** — GPS reports velocity inconsistent with IMU integration | Two-of-two persistence: `|v_gps − v_imu| > 15 m/s` sustained ≥2 consecutive fixes | `src/detect/jump.rs:46-55, 41` | `vmismatch_requires_persistence` | Attacker who can keep `|Δv|` below 15 m/s wins this single test, falls through to CUSUM and hard-residual layers |
| **Slow drift (naive)** — GPS walks off course at sub-jump speed, reported Doppler left honest | Per-axis two-sided CUSUM with `k=1.0 m`, `h=25 m`. Sums accumulate any per-fix residual above noise floor until threshold | `src/detect/drift.rs:79-98` | `persistent_north_drift_fires`, `tests/scenario_drift.rs` | If attacker pins `|r|` to *exactly* `k=1.0 m`, per-axis accumulators stagnate (B-01 risk); magnitude CUSUM is the backup |
| **Consistent-velocity walk-off ("smart" / EKF-laundered)** — GPS position ramps off course AND the reported Doppler is faked to match, so the complementary velocity blend tracks it and the position + velocity-mismatch lanes are driven to ~0 (the RQ-170 class; also the param-mode SITL case where ArduPilot's EKF has fused a slow ramp). Found by the SITL Phase-2 characterization: evaded the detector entirely below 2 m/s | Velocity-aiding CUSUM over the FREE-INERTIAL velocity residual `mag_vel_free = \|v_gps − v_free_inertial\|`, where `v_free_inertial = v_blended − Σ(blend corrections)` is reconstructed GPS-velocity-INDEPENDENT (so it retains the masked bias ≈ the spoof rate). Adaptive floor learned only from clearly-quiet (`< base k`) non-maneuvering fixes; base `k=0.55 m/s`, `h=8`. SUSPENDED while maneuvering (gyro-gated) — a coordinated turn makes the free-inertial velocity diverge ~2 m/s of legitimate attitude/centripetal error | `src/detect/drift.rs` (`s_vel_aiding`, `vel_aiding_*`), `src/nav/mod.rs` (`aiding_vel`, `is_maneuvering`) | `velocity_aiding_fires_on_sustained_masked_bias`, `velocity_aiding_suspended_while_maneuvering`, `velocity_aiding_no_false_alarm_on_realistic_doppler_noise`, `tests/scenario_consistent_drift.rs` (caught ≥1 m/s; clean 600 s + sustained turn no false-latch) | Bounds (all 🟡, by design — each trades detection of a vanishingly-slow attack for zero false alarms on honest dynamic flight): (1) a walk-off below ~0.5 m/s sits at/under the free-inertial velocity-bias floor — undetected; (2) a walk-off confined to turns is unobservable (lane suspended while maneuvering); (3) a spoofer who ramps the bias on over many minutes can be tracked by the adaptive floor |
| **Circular / spiral drift** — per-axis sums oscillate around zero so signed CUSUM misses it | Magnitude CUSUM: one-sided sum over `|r|`, with an ADAPTIVE reference — the effective `k_mag` is `max(base, learned_noise_floor × 1.5)`. The noise floor is an online running-mean (warmup) then quiescence-gated EWMA of `|r|`, so the detector adapts to the actual GPS noise level instead of a fixed constant | `src/detect/drift.rs` (`mag_noise_ewma`, warmup logic) | `circular_drift_caught_by_magnitude_cusum_only`, `magnitude_cusum_no_false_alarm_on_realistic_gps_noise`, `real_attack_still_fires_after_noise_floor_learned_on_noisy_gps` | None for the false-alarm case (B-02 fixed: verified 0 fires over 600 s of σ=2.5 m noise while a 6 m/fix drift still fires). Two residuals (both 🟢, audit U-01): (1) a constant-radius circular attack present from boot with NO clean baseline reads as the noise floor — indistinguishable from noisy GPS by construction; (2) a circular spoof sustained through the 20-fix warmup window (begins at preflight-Ready) can inflate the learned floor, desensitizing ONLY the magnitude lane — the per-axis CUSUM and jump detector are unaffected during warmup, so a linear/teleport component is still caught |
| **Replay attack** — attacker re-injects an old GPS_RAW_INT with fresh timestamp, GPS lat/lon identical across fixes | Frozen-fix detector: if last 3+ fixes are within `FROZEN_FIX_RADIUS_M = 0.5 m` AND IMU reports `|v| > 1 m/s` motion, fire `FrozenGps` event and register external anomaly with FSM | `src/fusion.rs:198-205, 665-763` | `tests/scenario_frozen_gps.rs` | Attacker who alternates replay with one drift-step per cycle (B-04) keeps streak below threshold; circular-replay also defeats unless covered by magnitude CUSUM |
| **Vertical-only spoof (sudden)** — altitude teleported but lat/lon unchanged | GPS-only vertical-rate sanity check: if reported altitude changes faster than `max_vertical_rate_mps` (default 30 m/s, ~2× the fastest real multirotor descent) between consecutive fixes, fire a `Jump` event with reason `VerticalRate` and register an external anomaly with the FSM. Deliberately does NOT use the unreliable vertical dead-reckoning | `src/fusion.rs:847-883`, `src/detect/mod.rs:46-58` | `altitude_teleport_fires_vertical_rate_jump`, `clean_flight_emits_no_vertical_rate_jump` | Two gaps (both 🟢): (1) **gradual** altitude drift below the 30 m/s rate bound — GPS altitude is inherently 2-3× noisier than horizontal with no reliable inertial vertical reference; (2) an altitude teleport that lands exactly on a GPS-dropout boundary (gap > freeze threshold) is not assessed — the rate check is skipped across dropouts (audit U-02) to avoid false-firing on a legitimate sustained descent during an outage. Sudden teleports during normal operation are caught (B-42) |
| **NaN / Inf injection** — adversarial GPS_RAW_INT with NaN lat / Inf alt | Plausibility gate at ingest: rejects non-finite or out-of-range lat/lon/alt/speed/course | `src/mav/mod.rs:393-415` | `rejects_nan_lat`, `rejects_inf_alt`, `rejects_out_of_range_lat`, `rejects_out_of_range_lon`, `rejects_absurd_altitude` | Plausible-but-malicious values (e.g. 89° N at a deployment that's actually at 40° N) require boot-anchor check + ongoing residual to catch |
| **MAVLink "unknown" sentinel poisoning** — autopilot sends `vel=UINT16_MAX`/`cog=UINT16_MAX`/`eph=UINT16_MAX` for fields it can't measure | Sentinels translated to `Option::None` at ingest; downstream skips the velocity-mismatch detector and falls back to position-only with tightened threshold | `src/mav/mod.rs:353-415` | `gps_from_msg_translates_velocity_sentinel_to_none`, `..._course_..._none`, `..._hdop_..._none`, `..._sats_..._none` | None — this is a deterministic protocol-level bug, fully closed (MAV-01) |
| **Attacker-controlled HDOP widens detector** — adversary spoofs `eph > 4 m` to trigger multipath tolerance widening (2× jump threshold, 1.5× CUSUM threshold) | Multipath widening requires BOTH `hdop > threshold` AND `sats < threshold`, AND both fields must be present (not the unknown sentinel) | `src/detect/jump.rs:22-44`, `src/detect/drift.rs:64-78` | `high_hdop_alone_does_not_widen_thresholds`, `high_hdop_alone_with_good_sats_does_not_widen`, `both_quality_indicators_bad_widens_thresholds`, `unknown_quality_does_not_widen` | Attacker who spoofs both `eph` AND `sats < 6` consistently still triggers widening; partial mitigation. Long-term: derive quality from observed residual variance (B-23 follow-up) |
| **No-Doppler persistence bypass** — attacker fires vmismatch on fix N, sends no-Doppler fix N+1 (counter unchanged), fires again on N+2 → completes the 2-fix persistence requirement non-consecutively | Persistence counter decays (saturating decrement) on no-Doppler fixes so absence-of-evidence doesn't carry the counter | `src/detect/jump.rs:55-58` | `no_doppler_gap_decays_persistence_counter` | Single no-Doppler fix between fires now resets to 0; attacker needs back-to-back fires to trigger |

### 3.2 Boot and re-anchor attacks

| Attack | Defense | Code site | Proof test | Residual risk |
|---|---|---|---|---|
| **Meaconing at boot** — attacker is already broadcasting spoofed GNSS when the vehicle powers up; first fix anchors at attacker's coordinate | Boot-anchor cross-check: first fix must land within `max_home_distance_m` of operator-declared `--expected-home`; otherwise `BootAnchorRejected` and the detector refuses to anchor | `src/nav/mod.rs:329-344`, `src/fusion.rs:537-577` | `boot_anchor_rejects_far_first_fix`, `boot_anchor_check_skipped_when_no_expected_home`, `tests/scenario_boot_anchor.rs` | Requires operator to set `--expected-home`. Default CLI guard refuses MAV deployment without it (B-01 fix); console deployments can opt out via `--allow-no-boot-anchor-check`. Operator typo of `--expected-home 0,0` accepts attacker fixes near null island (B-06 — deferred) |
| **Wait-out-then-spoof** — attacker forces FSM into Suspicious via jamming, waits for clear-dwell to expire, then drops a clean-looking fix at attacker-chosen coordinates → without veto, re-anchor commits to the spoofed position | Susp→Normal re-anchor plausibility veto: the post-jam fix must be within `re_anchor_max_distance_m` of the dead-reckoned position; otherwise force back to Suspicious and emit `BootAnchorRejected` with `phase: susp_to_normal_re_anchor` | `src/fusion.rs:800-868` | `tests/scenario_re_anchor_rejected.rs` | DR drift over long Suspicious dwells (~5-10 m typical, up to ~90 m at extreme bias × time) eats into the 500 m default threshold but leaves wide margin (B-11) |
| **Re-anchor veto cycling** — attacker triggers veto, FSM resets to Suspicious with fresh 10 s dwell, never escalates to Spoofed | `force_back_to_suspicious` preserves the *original* `suspicious_since` timestamp passed in by Fusion; cumulative dwell accrues across vetoes | `src/detect/state_machine.rs:289-302`, `src/fusion.rs:791-815` | `force_back_to_suspicious_preserves_original_dwell`, `force_back_to_suspicious_falls_back_to_now_when_no_original` | None for the dwell-cycling pattern; cumulative dwell now escalates as intended (B-26) |
| **CUSUM-reset via interpolate failure** — attacker stalls IMU stream so `buffer.interpolate(t_gps)` fails on every Nth fix; previously this called `on_clean_anchor()` which reset the drift CUSUM | Post-Ready interpolate failure no longer calls `on_clean_anchor`. The CUSUM accumulator persists across transient buffer misses; reset only happens at genuine re-anchor points (preflight first fix, Susp→Normal commit) | `src/fusion.rs:676-696` | Verified by inspection (`AUDIT B-20 / B-09` comment); no specific unit test yet — covered indirectly by full integration tests | None — IMU-stall bypass closed (B-20 / B-09) |
| **Pre-Ready re-anchor chain** — attacker floods preflight GPS, hoping the last preflight fix anchors at attacker's grid | Anchor is locked after the very first accepted fix; subsequent preflight fixes only blend velocity, never re-anchor position. Anchor itself is gated by the boot-anchor check above | `src/nav/mod.rs:280-298, 329-344`, `src/fusion.rs:591-603` | `tests/scenario_preflight.rs` | The first accepted fix anchors unconditionally if no `--expected-home`; covered by the boot-anchor row above |

### 3.3 Dropout / link manipulation

| Attack | Defense | Code site | Proof test | Residual risk |
|---|---|---|---|---|
| **Jam-then-spoof** — attacker jams GPS for N seconds while FSM is Suspicious, then drops spoof. Wall-clock dwell would tick across the gap and falsely escalate to Spoofed on the resumption fix | GPS-dropout detection: when inter-fix gap exceeds `gps_dropout_freeze_s` (default 2.5 s), the FSM's `suspicious_since` walks forward by the gap duration so dwell timer math is unchanged | `src/fusion.rs:619-657`, `src/detect/state_machine.rs:143-167` | `dropout_recovery_does_not_advance_dwell` | None for this attack pattern |
| **Sustained dropout to stall dwell** — attacker throttles GPS to keep `consecutive_dropout` flag perpetually set, freezing FSM forever | TWO caps, either resumes the dwell: (1) consecutive-streak cap (`MAX_CONSECUTIVE_DROPOUT = 5`); (2) **cumulative per-episode pause budget** (`detect.max_dwell_pause_s`, default 20 s) that a cadence-modulating attacker cannot reset. `DwellPauseExceeded` fires when either trips | `src/detect/state_machine.rs` (dropout branch + `paused_cumulative_s`) | `dropout_recovery_does_not_advance_dwell`, `cadence_modulation_cannot_freeze_dwell_forever`, `pause_budget_renews_on_committed_clear_but_not_on_veto` | None for the cadence-modulation pattern: the cumulative budget bounds total stall regardless of streak resets (B-03 FIXED, Phase 2). The budget renews only on a genuine episode end, never on a re-anchor veto |
| **GPS link permanently down** — link drops entirely; no spoof, just outage | Heartbeat-watchdog emits `LinkDown` / `LinkRestored` on the broadcast bus; fusion treats absent GPS the same as Suspicious-but-not-escalating | `src/mav/mod.rs:281-330` | Covered by `mav` integration test scaffold; no specific unit test | Operator-visible via JSON log; no automatic response (graceful degradation expected) |

---

## 4. MAVLink attack class catalog

### 4.1 Network-level attacks

| Attack | Defense | Code site | Proof test | Residual risk |
|---|---|---|---|---|
| **Off-host MAVLink injection** — attacker on the same LAN sends GPS_RAW_INT to our UDP bind port from a different IP | Source-IP filter: `MavFilter::allowed_source_ip = Some(target_ip)`. Default is `Some` populated by `--mav-target`; `--mav-no-source-filter` opt-out triggers a startup warning | `src/mav/mod.rs:153-164`, `src/bin/flyingsquirrel.rs:294-306` | Visual inspection + `MavStats::dropped_source_ip` counter | Operator who passes `--mav-no-source-filter` AND binds to `0.0.0.0` accepts any peer (loud warning, but the door is open) |
| **Sysid spoof / hostile ground station** — adversary sends MAVLink with a system_id different from the autopilot | Sysid filter: drops messages whose `header.system_id != target_system`. Default populated from `--mav-target-system`; `--mav-no-sysid-filter` opt-out warns | `src/mav/mod.rs:170-181`, `src/bin/flyingsquirrel.rs:300-306` | `MavStats::dropped_sysid` counter | Same opt-out caveat as above |
| **Bootstrap race / co-located attacker** — local process on the companion computer binds an ephemeral port and sends a HEARTBEAT before the real autopilot, claiming the source lock | Source-port lock: the first accepted HEARTBEAT/COMMAND_ACK pins `(src_ip, src_port)`. The lock only forms if the source port matches the operator-declared `--mav-target` port; ephemeral attacker ports are refused without pinning | `src/mav/monitor.rs:294-323`, `src/mav/mod.rs:191-231` | `source_lock_rejects_wrong_port_at_bootstrap`, `source_lock_pins_first_caller`, `source_lock_rejects_same_ip_different_port` | Attacker who controls the autopilot's exact bind port (e.g. spoofs via raw socket from the same host) bypasses; MAVLink signing is the right next layer (operator opt-in, not yet wired) |
| **HEARTBEAT replay** — attacker captures a HEARTBEAT (`custom_mode=RTL`, `base_mode=ARMED`) and re-sends it AFTER we trigger RTL, trying to fake confirmation | Four-gate `verify_rtb_engaged`: (1) per-command ACK with `result=ACCEPTED` arrives *after* send-time; (2) mode-transition timestamp is *after* send-time AND new mode is RTL-equivalent; (3) RTL mode observed on `RTB_VERIFY_DWELL_POLLS=3` consecutive polls; (4) between polls, `last_heartbeat_ns` MUST advance (forces N freshly-arrived HEARTBEATs over the window) | `src/mav/controller.rs:277-381` | `tests/scenario_mavlink.rs` (closed-loop ACK + causal + dwell) | Attacker who can MITM the link can drive all four gates simultaneously; MAVLink signing is the right defense |
| **ACK stomping** — autopilot ACKs unrelated commands (mission upload, geofence, status request) during our 5 s verify window, overwriting `last_ack` | Per-command-id ACK map: `last_ack_for_cmd(MAV_CMD_NAV_RETURN_TO_LAUNCH)` returns the most recent ACK for that specific command id, ignoring noise from others. Map capped at 64 entries to bound memory | `src/mav/monitor.rs:166-180, 248-294` | `ack_by_cmd_is_not_stomped_by_other_acks`, `ack_map_evicts_oldest_when_capped` | None — fully closed (C-14) |
| **Disarmed-after-RTL false negative** — autopilot enters RTL → flies home → lands → disarms within the 5 s verify window | `is_armed` check uses `last_heartbeat`'s `base_mode & SAFETY_ARMED`. A transient arm-bit flicker can fail the gate even on a successful RTL | `src/mav/controller.rs:340, src/mav/monitor.rs:242-245` | `arming_flag_recognized` | `verify_rtb_engaged` requires armed during dwell; very fast RTL→LAND→disarm races might emit `ActionUnconfirmed`. Trade-off favors safety: false-positive ActionUnconfirmed is a UX issue, not a security failure (C-15 — deferred refinement) |

### 4.2 Protocol-level attacks

| Attack | Defense | Code site | Proof test | Residual risk |
|---|---|---|---|---|
| **Autopilot byte mismatch / firmware swap** — operator declares `--vehicle ardu-copter` but the autopilot's HEARTBEAT reports `MAV_AUTOPILOT_PX4=12` | Preflight gate requires `autopilot_matches = (observed == expected_autopilot_byte(profile))`. Until match, FSM stays Initializing and detector does NOT run | `src/fusion.rs:475-489, 127-145`, `src/mav/monitor.rs:124-138` | `tests/scenario_preflight.rs` (autopilot-mismatch case) | None — mismatch loudly fails preflight |
| **PX4 vs ArduCopter mode confusion** — same `custom_mode` integer means different things; flat (`6=RTL`) for ArduCopter vs packed `(main<<24)|(sub<<16)` for PX4 | `is_rtl_mode(profile, custom_mode)` dispatches per-profile. PX4 path checks `main=AUTO=4` and `sub ∈ {RTL=5, LAND=6, RTGS=7}`; ArduCopter flat enum (6/9/21) | `src/mav/monitor.rs:90-114` | `rtl_mode_recognition_arducopter`, `rtl_mode_recognition_px4`, `px4_mode_encoding_round_trips` | None for these two profiles. Plane / Rover / other profiles would need their own dispatch arm |
| **PX4 DO_SET_MODE silently ignored** — PX4 requires `base_mode | CUSTOM_MODE_ENABLED` bit set for `MAV_CMD_DO_SET_MODE` to take effect; missing bit silently fails | Controller's PX4 path always sets `param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (0x01)` | `src/mav/controller.rs:154-172`, `src/mav/monitor.rs:124` | Visual + ACK gate above catches the failure mode | None — bit always set |
| **Mode-change race** — autopilot was already in RTL for unrelated reasons (low battery, pilot manual, geofence) when we fired RTL | Mode-transition causality: `mode_changed_at_ns > send_ns` required, NOT just "current mode is RTL." Defeats false-positive on pre-existing RTL | `src/mav/controller.rs:332-341`, `src/mav/monitor.rs:281-287` | `mode_changed_at_ns_advances_only_on_change`, `tests/scenario_mavlink.rs` | None for this race |

---

## 5. Composite chain attacks

These compose multiple defenses; failure of any one weakens the chain.

| Attack | Defense | Code site | Proof test | Residual risk |
|---|---|---|---|---|
| **Operator SIGHUP after first action = silent disarm** — composite attack: trigger Spoofed once, wait for operator to dismiss with SIGHUP, the next Spoofed transition produces `AlreadyEngaged` and silently fails to re-engage | Operator-reset path calls `controller.reset()` to clear action latches AND also resets FSM-level ephemeral state (`last_gps_t_secs`, `last_gps_lla`, `frozen_fix_streak`, `no_doppler_streak`) | `src/fusion.rs:313-348`, `src/action/console.rs:82-99`, `src/mav/controller.rs:252-261` | `manual_reset_clears_spoofed_and_resumes_detection` | Forensic `dump_fired` flag intentionally NOT cleared (one-dump-per-process is policy); restart re-arms |
| **Sever-fails-then-RTL** — `sever_gps` fails (transient network blip or PARAM_SET refused), `engage_rtb` proceeds; autopilot RTLs on still-spoofed GPS to attacker's "home" | Critical-warning event + `tracing::error` fires between failed sever and successful RTL: `consequence: engage_rtb will fire next but the autopilot is still using the (spoofed) GPS to navigate. Operator should assume RTL is compromised and take manual control` | `src/fusion.rs:874-913` | Verified by inspection (`AUDIT B-13` comment); covered by full Spoofed-transition integration test | We don't refuse RTL (autopilot may still be safer in some RTL than continuing GUIDED with bad GPS). The defense is *visibility*; operator must intervene |
| **TOML-baked safety bypass** — operator pins a config in version control with `allow_synth_to_mav=true` baked in; no CLI override possible to "explicitly false" | TOML rejects ALL four safety-bypass bools at load time (`allow_synth_to_mav`, `allow_no_boot_anchor_check`, `no_source_filter`, `no_sysid_filter`). To enable them, the operator must pass the explicit `--allow-*` CLI flag, where it's visible per-invocation in `ps`/journald | `src/config.rs:186-260` | `toml_safety_bypasses_are_rejected_at_load_time`, `toml_safety_bypasses_false_load_ok` | None — fully closed (F-01) |
| **Synth-to-MAV friendly fire** — simulated spoof scenario fires real PARAM_SET + RTL at a real autopilot | CLI guard refuses the combo unless `--allow-synth-to-mav` is explicitly passed (and that flag cannot be enabled via TOML — see above) | `src/bin/flyingsquirrel.rs:237-251` | `tests/scenario_*` exercises the combo's allowed path | None |
| **Forensic disk-fill DoS** — attacker (or buggy detector in false-positive loop) triggers many Spoofed transitions to fill the disk with `spoof-*.json` dumps | One-shot guard: `dump_fired_flag` flips to `true` only on a successful write; subsequent triggers emit `ForensicDumpSuppressed` instead. Restart re-arms | `src/forensic.rs:193-194, 213-222`, `src/fusion.rs:388-460` | `dump_fired_flag_is_shared` | A failed write does NOT set the flag (retries on next trigger), so a sustained "fail then trigger" combination could write the same dump many times. Disk space is bounded by single-dump size (~500 KB) × retry frequency — acceptable |
| **Process killed mid-forensic** — `systemctl stop` sends SIGTERM while forensic dump task is mid-write; runtime drops and the dump is aborted | SIGTERM handler in shutdown select (Unix); shared `JoinSet` tracks in-flight forensic + verify tasks; `drain_pending_bg` awaits them with a 7 s bounded timeout before exit | `src/bin/flyingsquirrel.rs:480-571`, `src/runtime.rs:1-260`, `src/fusion.rs:425-484` | `drains_completed_tasks`, `drain_aborts_runaway_tasks_past_timeout`, `drain_empty_returns_immediately` | A `SIGKILL` (kill -9) is unstoppable by user-space; data loss in that case is expected and noted (E-10/E-11) |

---

## 6. Sensor and operator failure modes

| Failure | Defense | Code site | Proof test | Residual risk |
|---|---|---|---|---|
| **Serial GPS USB disconnect** (cable jiggle, EMI, kernel renumeration) | Reconnect-with-exponential-backoff loop inside `SerialGpsSource::into_stream`: 250 ms initial, doubling up to 5 s. Logs disconnect + reconnect at warn level. Only genuine **I/O errors** reach this path: framing faults (over-long buffer, non-UTF-8 line) are reported by the codec as data events instead, so garbage bytes no longer tear down and reopen a healthy port | `src/hw/serial_gps.rs` (reconnect loop), `src/hw/nmea_link.rs` (framing) | `binary_garbage_does_not_tear_down_a_working_connection` (end-to-end over an in-memory pipe — a burst of unframed binary is followed by a sentence that must still arrive), `overlong_buffer_is_discarded_and_framing_resyncs`, `non_utf8_line_is_reported_not_silently_dropped`. The reconnect/backoff policy itself is still visual; live exercise on real hardware recommended | If the OS doesn't release the FD promptly on disconnect (kernel-driver bug), reconnect may EBUSY-loop for ~1-2 s until renumeration completes |
| **Serial GPS connected but not delivering** — the port opens, so the detector logs "connected" and then silently never gets a fix. Four ordinary bring-up faults are indistinguishable downstream: nothing on the wire (TX/RX swapped, unpowered); bytes but no NMEA (baud mismatch, or a u-blox left in UBX binary mode — a setting that persists across power cycles); NMEA but no fix (no antenna / indoors / cold start); fixes decoded but **all** rejected by the plausibility gate | `nmea_link::LinkHealth` watches what reaches each pipeline stage (bytes → framed lines → `$` sentences → parsed fixes → gate) and names the evidence and the likely cause. Short 10 s fuse for the never-normal conditions; 60 s for no-fix-yet, so a cold start does not cry wolf; re-reported once a minute while it persists, with a single `RECOVERED` line closing the episode. Byte counting lives in the codec so "not a single byte arrived" stays truthful even when the module sends a partial line and stops | `src/hw/nmea_link.rs` (policy), `src/hw/serial_gps.rs` (poll tick) | `dead_wire_is_diagnosed_as_no_bytes_after_the_grace`, `binary_module_is_diagnosed_as_no_sentences_not_no_bytes`, `baud_mismatch_shows_up_as_non_utf8_evidence`, `cold_start_is_tolerated_then_diagnosed_as_no_fix`, `gate_rejecting_everything_is_reported_promptly`, `a_fix_clears_stale_rejections`, `repeats_are_rate_limited_then_recovery_closes_the_episode`, `healthy_link_never_reports`; the surrounding read loop is exercised end-to-end over an in-memory pipe by `nmea_bytes_become_fixes_and_eof_ends_the_connection`, `a_link_that_never_fixes_reports_no_first_fix`, `lines_that_are_not_sentences_are_skipped_not_fatal` | **Observability only — deliberately changes no detection behavior.** Absence of GPS is already handled: the preflight checklist will not go Ready without `first_fix_accepted`. Escalating to stream-end here (the IMU ladder's response) would be wrong — stale IMU samples poison dead-reckoning, whereas a slow GPS lock is just slow, and killing the process would turn a cold start into an outage. Diagnosis is heuristic: it names the *likely* cause from the evidence, it does not prove one |
| **NMEA line-buffer OOM** — a peer that never emits `\n` (binary output, wedged module, hostile splice on the UART) grows the framing buffer without bound on a companion computer with a few hundred MB of RAM | 512-byte cap (6× the NMEA 0183 maximum sentence length): past it the buffer is discarded and framing resyncs on the next `\n` | `src/hw/nmea_link.rs` (`NmeaLineCodec::decode`) | `buffer_never_grows_past_the_cap_under_a_newline_less_flood`; also asserted after every read chunk by the `nmea_parse` fuzz target | Up to one read's worth of bytes above the cap exists transiently inside `decode` before the discard |
| **I²C IMU bus down** (chip gone, kernel I2C controller error, driver rebind) | Escalation ladder (`mpu6050::ReadHealth`): 100 consecutive bad reads → `warn!` "DEGRADED" once; every 200 → **in-process re-open + re-init** (preserves detector FSM/CUSUM/DR state, unlike a process restart, which also costs the 2–5 s startup blind window); 1000 → end the stream so systemd restarts the process. Recovery (good read) logs `info!` "RECOVERED" with the streak length | `src/hw/mpu6050.rs` (policy), `src/hw/i2c_imu.rs` (loop) | `error_streak_warns_once_at_degraded`, `reinit_requested_on_interval_but_not_at_bus_down`, `recovery_resets_the_ladder_and_reports_streak` | Between degraded (1 s at 100 Hz) and bus-down (10 s) the IMU is absent; operator must watch logs. Ladder thresholds are counted in read ticks, sized for the ~100 Hz default rate |
| **Substituted / wrong IMU chip at the configured address** — an MPU-9250 / ICM-class part speaks the same protocol but has different full-scale factors: every sample silently mis-scaled by a constant, cross-check desensitized, plausibility gate blind to it | WHO_AM_I identity gate at init (and on every in-process re-init): anything but `0x68` (MPU-6050/6000) refuses to start, with an actionable error naming the value read and the likely chip | `src/hw/mpu6050.rs` (`check_who_am_i`), `src/hw/i2c_imu.rs` (`open_and_init`) | `who_am_i_accepts_mpu6050`, `who_am_i_rejects_substitutes_with_actionable_message` | A counterfeit that fakes `WHO_AM_I=0x68` with off-spec scales is not software-detectable; bench-verify scale (1 g at rest) before flight. Burst coherence (no mid-frame tearing) is an MPU-60X0 latching guarantee — one more reason the identity gate is load-bearing |
| **IMU frozen output** — a brown-out resets the MPU-6050 with the SLEEP bit set: I²C reads keep SUCCEEDING, sensor registers stop updating; samples arrive bit-identical, individually plausible, at full rate — dead-reckoning integrates a stale specific-force vector and the "DEAD-RECKONING STALLED" escalation (which only catches MISSING samples) never fires | 100 consecutive bit-identical raw 14-byte frames (the frame includes the temperature register, whose LSB dithers continuously on an awake die) → frames DROPPED (never yielded) and routed into the same escalation ladder; the re-init step re-writes `PWR_MGMT_1` and wakes the chip | `src/hw/mpu6050.rs` (`ReadHealth::on_read_ok`), `src/hw/i2c_imu.rs` | `frozen_output_detected_at_threshold_and_not_yielded`, `frozen_run_survives_interleaved_read_errors`, `frozen_and_errors_reach_reinit_together`, `frozen_streak_eventually_ends_stream` | Up to ~1 s of stale samples is yielded before the threshold trips (bounded by design — the alternative is delaying every sample). Sub-second freezes pass undetected; the DR error from ≤1 s of stale IMU is within the detectors' noise floors |
| **IMU vibration / saturation** | Plausibility gate rejects `|accel| > 200 m/s²` (≈20 g) and `|gyro| > 35 rad/s` (≈2000 °/s). Sample is dropped, downstream sees the gap as missing IMU | `src/mav/mod.rs:420-434` | `rejects_saturated_accel`, `rejects_saturated_gyro`, `accepts_normal_imu` | Real high-G maneuvers near these limits drop samples. Sustained gap triggers SyncWarning |
| **GPS multipath / urban canyon** — legitimate noisy environment, HDOP genuinely > 4 with sats < 6 | Detector widens both jump and drift thresholds by 1.5-2× when both quality indicators are bad (see B-23/B-24 above) | `src/detect/jump.rs:23-44`, `src/detect/drift.rs:64-78` | `both_quality_indicators_bad_widens_thresholds` | False positives in known multipath zones; operator should pre-survey deployment area |
| **Accelerometer sign-convention mismatch** — a real MEMS IMU and ArduPilot / PX4 `SCALED_IMU` / `HIGHRES_IMU` emit the *specific-force* convention (`[0,0,-9.81]` at rest); the nav stack previously assumed the *gravity-reaction* convention (`[0,0,+9.81]`), so real hardware dead-reckoned in a chirality-mirrored frame during any lateral/turning motion | **FIXED (N-01):** the nav stack now uses the physical specific-force convention natively — `init_from_accel` seeds level from a `-g` reading, the Madgwick error term and gravity compensation (`a_lin = R·f + g`) match, and the sim + `mavsim` emit `[0,0,-g]` at rest like real firmware. The boot IMU sanity check now warns if the Down-axis is strongly **positive** at rest (a gravity-reaction IMU feeding the specific-force stack) | `src/nav/attitude.rs` (`init_from_accel`, Madgwick `fx/fy/fz`), `src/nav/mod.rs` (gravity comp + `BOOT IMU SANITY`), `src/sim/mod.rs`, `src/bin/mavsim.rs` | `init_from_specific_force_at_rest_is_level`, `update_holds_level_under_rest_specific_force` (unit); all `scenario_*` integration tests (sim + detector consistent) | **Math is unit-tested; SITL/HIL maneuver validation still recommended before real flight** (the sim flies no turns yet — see S-09/S-10). The Linux-only **I2C path still passes raw chip axes through** with no body-frame remap, so a non-FRD mounting needs a per-axis sign/rotation map (hw reviewer finding; the boot check now flags a wrong-sign mount loudly) |
| **Disconnected / mis-scaled IMU at boot** — IMU reads ~0 (unplugged), wrong LSB→m/s² scale, or is in motion at boot → attitude filter seeds from garbage, silently disabling dead-reckoning | Boot gravity-magnitude sanity check: static-average accel magnitude must be 1 g ± ~8% (`9.0–10.6 m/s²`); outside that band a loud warning fires | `src/nav/mod.rs:156-181` | Inspect; warning path | Best-effort init still proceeds (degraded), so the operator sees events but should treat them as unreliable until the warning clears (audit N-10) |
| **Serial GPS produces zero fixes (NMEA features compiled out)** — the `nmea` crate feature-gates every sentence type; a misconfigured Cargo.toml disables GGA/RMC/VTG parsing, so a real receiver yields no fixes and the detector is silently blind | `nmea` dependency now enables the `GNSS` sentence bundle; a regression test feeds real GGA/RMC sentences through the parser and asserts a fix is produced | `Cargo.toml` nmea line, `src/ingest/nmea.rs:44-110` | `parses_real_gga_sentence_to_fix`, `parses_real_rmc_sentence`, `rejects_bad_checksum` | None — fixed and regression-tested (audit I-01) |
| **Operator typo in `--expected-home`** — e.g. `--expected-home 40,75` instead of `40,-75` (sign flip), or `40,-100,300` (trailing altitude silently truncated) | CLI parser validates `|lat| ≤ 90`, `|lon| ≤ 180`. Sign typos pass parse but boot-anchor check fires `BootAnchorRejected` if launch site isn't actually at the typo'd coords | `src/bin/flyingsquirrel.rs:392-422` | `boot_anchor_rejects_far_first_fix` | `--expected-home 0,0` (null island) accepts attacker fixes near that point (B-06 — deferred); trailing extra commas silently truncate (F-23 — deferred) |
| **Operator forgets `--vehicle`** in MAV mode | Default-deny CLI guard: refuses to start MAV-controller without `--vehicle`, listing supported profiles | `src/bin/flyingsquirrel.rs:263-275` | Manual: `cargo run -- --controller mav` without `--vehicle` exits with the error message | None |
| **Operator forgets `--expected-home`** in MAV mode | Default-deny CLI guard: refuses to start MAV deployment without `--expected-home`, unless `--allow-no-boot-anchor-check` is explicitly passed (NOT RECOMMENDED for production) | `src/bin/flyingsquirrel.rs:282-290` | Manual: `cargo run -- --controller mav --vehicle ardu-copter` without `--expected-home` exits | None |
| **Launch coordinates leaked at INFO via journald-forwarded logs** — opsec issue: drone home location ends up in centralized log sinks | Coordinates logged at `tracing::debug` only; `tracing::info` line emits radius + `home_configured=true` without lat/lon | `src/bin/flyingsquirrel.rs:397-414` | Visual; verified in live demo output | Operators who set `RUST_LOG=debug` re-enable the leak (acceptable for debug sessions) |
| **`--forensic-window-s` NaN/Inf** — operator typos a config value, ring buffer either aborts (Inf → `VecDeque::with_capacity(usize::MAX)`) or silently never retains anything (NaN → cutoff is NaN, every record pruned) | `ForensicCfg::validate` rejects NaN, Inf, ≤0, and >3600s at startup with a clear error | `src/runtime.rs:48-79` | Validates via `validate()` directly; covered by `cfg.validate` integration | None |
| **`--json-log` / `--forensic-dir` path traversal** | systemd unit's `ReadWritePaths=/var/lib/flyingsquirrel /var/log/flyingsquirrel` constrains writes at the kernel level | `deploy/flyingsquirrel.service` | Manual: pointing the flag at `/etc/passwd` under systemd produces EROFS at write time | Non-systemd deployments (raw `cargo run`, Docker without `--read-only`) bypass this. Docker now has `--read-only` (F-09 fix) so the bypass is closed in the recommended container path |

---

## 7. Supply-chain and build integrity

| Concern | Defense | Code site | Proof test | Residual risk |
|---|---|---|---|---|
| **Wrong binary deployed by accident** | Startup attestation: SHA-256 of `/proc/self/exe` (or `current_exe` on Win/macOS), crate version, build profile, host OS/arch all emitted to `tracing::warn` AND as the first line of the JSON event log. Path + hash are derived from a **single** `current_exe()` lookup so the pair cannot describe two different files if the binary is swapped mid-startup (audit D5) | `src/attestation.rs:42-104`, `src/bin/flyingsquirrel.rs:225-228` | `capture_returns_finite_values`, `hex_encode_known_value` | Not a security boundary against an attacker with code execution; it's provenance. Right defense for that threat: TPM-backed measured boot |
| **Tampered upstream Rust crate** | `Cargo.lock` pins exact versions; `cargo audit` should run in CI (not yet wired) | `Cargo.lock` | `cargo audit` (manual) | `[deferred]` Add `cargo audit` to a GitHub Actions release pipeline |
| **Hostile bytes on an untrusted ingest boundary** (NMEA over UART; MAVLink datagrams on a shared link) — a panic in a decode path kills the listener task and with it the whole detector; a decoder that "succeeds" into garbage poisons the residual math silently | Coverage-guided fuzzing of the three decode surfaces, driving the REAL public functions the listener calls: `nmea_parse` (read chunks → **the real `NmeaLineCodec`** → fix; chunked input so the codec's cross-read buffering AND the parser's cross-sentence state are both exercised, and the 512-byte buffer cap is asserted after every read), `mav_ingest` (datagram → version sniff → frame parse → typed conversion → gate → align), `clock_aligner` (attacker-controlled wire timestamps). Each asserts no-panic AND that whatever the plausibility gates ACCEPT is finite/in-range | `fuzz/fuzz_targets/*.rs`, `.github/workflows/fuzz.yml` | Nightly `fuzz.yml` run (300 s/target, crash fails the job + uploads the reproducer); targets build-checked on every push touching the covered paths. The `ClockAligner` bound also runs on every platform as `clock_aligner_never_leads_arrival_beyond_skew_bound` (proptest) | Fuzzing is a search, not a proof — absence of a crash after N seconds is not absence of bugs. libFuzzer is Linux-only here (no MSVC linkage), so this coverage exists in CI, not on the Windows dev box. The listener's stateful layer (source-lock, monitor bookkeeping) is covered by integration suites, not by these targets |
| **Tampered Docker base image** | Multi-stage build; runtime stage is `debian:slim`. **NOT pinned by digest** | `deploy/Dockerfile` | — | `[deferred]` F-08: pin both `rust:slim` and `debian:slim` to specific digests |
| **Container escape via dep CVE** | Docker runs with `--read-only --user 1000:1000 --cap-drop=ALL --security-opt=no-new-privileges --memory --pids-limit --tmpfs /tmp` | `deploy/docker-run.sh` | Manual; `docker inspect flyingsquirrel` after launch | A successful escape from a hardened-config container still has network access (host-network mode for MAVLink UDP). Real isolation requires giving up host-network and managing UDP through a bridge |

---

## 8. Forensic and post-incident

| Need | Mechanism | Code site | Proof test |
|---|---|---|---|
| **Post-incident reconstruction** — "what was the detector seeing in the lead-up to the Spoofed transition?" | Ring buffer of last 30 s (configurable) of GPS / IMU / residual records. On first Spoofed transition per process lifetime, atomic JSON dump to `--forensic-dir/spoof-{ts}-{pid}.json` | `src/forensic.rs:1-375`, `src/fusion.rs:382-484` | `tests/scenario_forensic.rs`, plus 5 unit tests in `forensic.rs` |
| **Atomic write** — process crash mid-dump must NOT leave partial JSON on disk | `.tmp` + rename pattern; `O_EXCL` on create; `0o600` mode on Unix. After the rename, the parent directory is fsynced (Unix) so the rename itself survives a power cut — file contents alone being durable is not enough on an SBC's SD card (audit D3) | `src/forensic.rs:312-430` | `dump_writes_atomic_file_and_returns_path` |
| **Resist disk-fill DoS** | Once-per-process flag set on successful write only | `src/forensic.rs:193-194, 213-222` | `dump_fired_flag_is_shared` |
| **Forensic dump captured BEFORE actions fire** | Snapshot taken in the same call that emits the Spoofed transition, BEFORE `try_action` runs sever + RTL (which can take ~600 ms) | `src/fusion.rs:874-884` | Reviewable in source; `AUDIT B-13` comment |
| **Shutdown drain** — `systemctl stop` must not abort an in-flight dump | SIGTERM handler + shared `JoinSet` + `drain_pending_bg(7s)` | `src/bin/flyingsquirrel.rs:480-571`, `src/runtime.rs:212-260` | `drain_*` tests |

---

## 9. Out of scope and accepted risks

This is the honest "we don't do this" list. Each item is a real attack
class; we chose not to defend it (with reasoning) or it is the right job
of a different system.

- **Local root attacker.** An attacker who can `cat /etc/shadow` can also
  `gdb --pid=$(pidof flyingsquirrel)`, swap the binary on disk, or modify
  the SHA-256 hash log line. The right defense is measured boot with a
  TPM; FlyingSquirrel runs as an unprivileged service and relies on the
  kernel boundary, no more.

- **Symlink swap of the forensic directory's parent (audit D2).** `O_EXCL`
  create already blocks a symlink swap of the dump file itself; what remains
  is a hostile user replacing a PARENT directory with a symlink so dumps land
  elsewhere. In the shipped systemd layout this requires root: the state dir
  is created by `StateDirectory=` under root-owned `/var/lib`, and
  `ProtectSystem=strict` + `ReadWritePaths` confine where the service may
  write at the kernel level — and local root is already accepted above.
  Operators who relocate `--forensic-dir` outside the unit's managed paths
  take on the requirement that its parent chain not be writable by untrusted
  users.

- **Physical airframe access.** An attacker who can replace the GPS
  module or wire-tap the I²C bus is outside this layer. Hardware
  tamper-evident enclosures are a different control.

- **MAVLink signing.** The MAVLink v2 protocol supports HMAC-SHA256
  message signing. FlyingSquirrel does NOT currently sign or verify, so a
  MITM attacker between the companion computer and the autopilot can
  forge messages that pass the source-port and sysid filters. Operator
  opt-in is a clean future addition; defaults must remain
  signing-disabled for compatibility with autopilots that don't speak
  signed MAVLink.

- **GPS receiver hardware compromise.** If the receiver is replaced with
  an attacker-controlled module that injects NMEA at the serial port,
  every fix it produces is treated as legitimate by the NMEA parser.
  Cross-check via IMU still catches gross spoofs, but a co-spoofed
  inertial source would defeat the system. Two-receiver diversity (RTK +
  GNSS, or GPS + GLONASS-only) is a future enhancement.

- **Time-source attacks.** The fusion clock base is `tokio::time::Instant`
  (monotonic, immune to wall-clock manipulation). On the MAV path the
  *inter-sample spacing* now follows the autopilot's own sensor timestamps
  (`ClockAligner`, see the TIME-SYNC row in §10) to remove link jitter — but
  those deltas are mapped onto the monotonic base and bounded by
  `CLOCK_ALIGN_MAX_SKEW` with an arrival-time fallback, so a manipulated
  `time_usec` can shift the alignment by at most that bound before re-anchoring
  (and an on-link forger can already spoof position directly). The JSON event log
  also emits a UTC `captured_at_utc`; if NTP is being attacked, those
  values may be wrong. Forensic dumps include both monotonic-ns AND UTC
  so analysts can detect skew post-hoc.

- **Side-channel attacks** (power analysis, EM emissions to recover
  internal state). Not relevant to this software; addressed by hardware
  countermeasures.

- **DoS via repeated process restarts.** A persistent attacker who can
  trigger ingest-task-died exits (e.g. by repeatedly disconnecting the
  GPS USB) will cycle the process through restart 5× / 60 s before
  systemd parks it. During each ~3 s restart window, the drone is
  undefended. Operators should pair the unit with an `OnFailure=`
  notification hook so they at least *know* when the daemon parks (F-21
  — documented limitation).

---

## 10. Known deferred fixes

These are 🟡-severity findings from the recent audit that landed in the
follow-up backlog. Each is filed against a specific audit ID so it's
greppable; severity reflects exploitability.

| ID | Issue | Why deferred |
|---|---|---|
| **B-06** | Operator typo `--expected-home 0,0` accepts attacker fixes near null island | Add `validate()` rejection for `|lat|+|lon| < ε` |
| **B-23 follow-up** | Long-term: derive multipath quality from observed residual variance instead of trusting reported `eph`/`sats` (defeats clever spoofers who fake both) | Larger redesign |
| **C-15** | `is_armed` check can falsely fail RTL verify during a fast RTL → LAND → disarm sequence | Requires N-consecutive-HB-disarmed gate; low impact (UX) |
| **F-08** | Dockerfile base images not pinned by digest | Coordinate with CI pipeline that updates pins on `renovate` schedule |
| **F-16** | No `logrotate.d/flyingsquirrel` config; `events.jsonl` grows unbounded | One-off addition during a deploy-tooling pass |
| **F-07** | systemd unit could add `CapabilityBoundingSet=`, `SystemCallFilter=@system-service`, `MemoryDenyWriteExecute=yes`, `PrivateUsers=yes` for defense-in-depth | Each needs testing against the dialout/i2c group requirements |
| **N-01** | ~~Specific-force IMUs need driver-level sign normalization~~ **FIXED**: the nav stack uses the specific-force convention natively (real ArduPilot/PX4/MEMS IMUs are correct by default; unit-tested). Remaining: (a) SITL/HIL maneuver validation; (b) the I2C path still needs a body-frame **axis** remap for non-FRD mountings (the sign is now flagged at boot, but axis order/rotation is not) | Add a `[i2c].axis_map` (signed 3×3 / per-axis sign) and validate against a real MPU-6050; the sign-convention half is done |
| **N-01b** | The corrected sign convention is verified by unit tests + a self-consistent sim, but the sim flies no turns, so the *mirrored-during-maneuver* failure it fixes is not yet exercised end-to-end | Tied to S-09/S-10 (add turning trajectories + real MEMS noise), then re-run SITL |
| **I-03** | NMEA VTG sentences are dropped; a VTG-only marine module yields no fix even after the I-01 feature fix | Add a `SentenceType::VTG` arm that merges speed/course into the next position fix |
| **S-04** | `scenario_mavlink` uses hardcoded UDP ports (24600/24601) and a 20 s wall-clock deadline → can collide / flake under CI contention | Bind port 0 and read back the assigned port; or serialize the test |
| **S-01** | `scenario_re_anchor_rejected` assertion (`rejected > 0 \|\| normal_transitions == 0`) short-circuits to true when the FSM never attempts a clear, so the integration test can't actually fail | Drive a scenario that provably opens a clean window so a Susp→Normal is attempted, then assert `rejected > 0` unconditionally |
| **S-09** | `scenario_clean` uses deliberately near-zero IMU noise — it's a detector-logic test, not a realistic false-alarm-rate test | Add a separate scenario with realistic MEMS sigmas (accel σ≈0.03, gyro σ≈0.002, GPS σ≈2.5 m) over 120 s, assert a bounded false-event budget |
| **S-10** | Sim has no accel-bias or random-walk-bias dynamics, so the `BiasEstimator` machinery is never meaningfully exercised by integration tests | Add bias inputs to `SyntheticImu` + a long clean-flight scenario asserting zero false Spoofed latches |
| **I-04** | `SpoofingEvent` is `Serialize`-only; forensic/JSONL dumps can't be parsed back by a Rust consumer | Add `Deserialize` (+ a `mono_ns` deserializer) or document the schema as write-only |

### Phase R full-coverage audit (this pass)

The Phase R audit read every file in `src/` (incl. the never-before-audited
`nav/attitude.rs`, `nav/strapdown.rs`, `bin/mavsim.rs`, `ingest/nmea.rs`, and
the full test harness). Fixed this pass:

| ID | Severity | Fix |
|---|---|---|
| **I-01** | 🔴 | NMEA sentence features were compiled out → serial GPS produced zero fixes on real hardware. Enabled the `GNSS` bundle + added parse regression tests |
| **N-10** | 🟡 | Boot gravity-magnitude sanity check (catches disconnected / mis-scaled IMU) |
| **N-01** | 🟡 | Boot accelerometer sign-convention warning + documentation |
| **N-02** | 🟡 | Complementary blend now uses the actual inter-fix interval (was hardcoded 1.0 s → 10× too aggressive at 10 Hz GPS) |
| **N-05** | 🟡 | Yaw seeding from GPS course no longer gated behind `!anchor_locked` (a vehicle stationary at boot then moving now gets yaw seeded) |
| **I-02** | 🟡 | `mavsim` binds loopback by default (was `0.0.0.0` — an open MAVLink endpoint emitting attack trajectories) |
| **S-02/S-03** | 🟡 | Added `VelocityInconsistent` spoof pattern + `scenario_velocity_mismatch` test — the velocity-mismatch detector path now has end-to-end coverage |
| **test harness** | 🟡 | `run_scenario` now drains `pending_bg` before collecting events, fixing latent `scenario_forensic` flakiness (forensic dump's real-time I/O racing virtual time) |

Verified correct (no change needed): Madgwick quaternion integration, gravity
prediction, Jacobian, and gradient-descent step; strapdown trapezoidal
velocity+position; `lla_to_ned`/`ned_to_lla` exact inverse between sim and
detector; sim IMU gravity sign matches the detector's expectation; spoof
drift/jump math; virtual-time clock alignment under `start_paused`.

### Phase S detection-coverage hardening

| ID | Severity | Fix |
|---|---|---|
| **B-02** | 🟡 | Adaptive magnitude-CUSUM noise floor — the fixed `k_mag=1.0` sat below the Rayleigh mean of real-GPS residual noise and false-fired every ~14 s on σ=2.5 m links. Now learns the floor online (running-mean warmup → quiescence-gated EWMA) and clamps the effective reference above it. Monte Carlo test: 0 false fires over 600 s of σ=2.5 m noise; real attacks above the floor still fire |
| **B-42** | 🟡 | Vertical-spoof detection — GPS-only altitude-rate sanity check (>30 m/s apparent climb/descent = spoof), independent of the unreliable vertical dead-reckoning. Catches sudden altitude teleports; emits `Jump`/`VerticalRate` + escalates the FSM |

### Phase V/W real-ArduPilot SITL validation

Found by running the detector against **real ArduPilot SITL firmware** (not our
own `mavsim`, which was circular by construction). Each of these would have
made the MAVLink ingest path non-functional or silent on a real autopilot:

| ID | Severity | Fix |
|---|---|---|
| **V-IMU** | 🔴 | Listener accepted only `HIGHRES_IMU` (PX4 message); ArduPilot streams `SCALED_IMU/2/3`. `--imu-source mav` got ZERO IMU on real ArduCopter → preflight hung forever. Now parses SCALED_IMU with mG→m/s² / mrad/s→rad/s conversion. Tests: `imu_from_scaled_imu_converts_units`, `imu_from_highres_imu_is_direct` |
| **V-MAVVER** | 🔴 | Listener hard-coded MAVLink v2; ArduPilot emits v1 (0xFE) by default → every frame rejected, detector stone-deaf. Now picks version from the frame magic byte. Test: `mavlink_version_picked_from_magic_byte` |
| **X-02** | 🟡 | Version-pick read `buf.first()` on the persistent receive buffer (never `None`), so a 0-length/stale datagram mis-picked the version from a leftover byte. Now slices to `n` received bytes and drops sub-8-byte datagrams before inspecting |
| **W1 (V-SYNC)** | 🟡 | Over a real ~10 Hz `SCALED_IMU` link, GPS lands ~1 IMU period (~100 ms) past the newest buffered sample; the fixed 20 ms post-`latest` tolerance rejected nearly every fix (`GpsOutsideBuffer`) so no residual was computed. The buffer now forward-extrapolates the newest sample along its velocity up to `DEFAULT_MAX_EXTRAPOLATION_S` (0.25 s), bounded so a genuine dropout still errors. No behavior change for a fast (100 Hz) IMU (dt≈0 → reduces to the prior clamp). Tests: `extrapolates_forward_within_horizon`, `extrapolation_horizon_is_configurable` |

Validated end-to-end via a reusable containerized harness
(`deploy/sitl/run-sitl-validation.{ps1,sh}`): real SITL → MAVLink relay →
detector. After V-IMU + V-MAVVER, the detector **arms on live ArduPilot**
(`PreflightPassed`); W1 closes the residual-computation gap.

### Phase 1 correctness lockdown (this pass)

Five safety-critical correctness defects surfaced by a full re-audit. Each
broke the core promise on a path the sim could not exercise (PX4, real-hardware
IMU sign, the fusion-crash path, a production duration, low-rate IMU streams).

| ID | Severity | Fix |
|---|---|---|
| **P1-PX4SEVER** | 🔴 | `sever_gps` sent ArduPilot-only `PARAM_SET GPS_TYPE=0` with no profile branch → on `--vehicle px4` the spoofed GPS was **never severed** (PX4 ignores the unknown param) and the drone RTL'd on it. Now branches per profile (`GPS_TYPE` for ArduPilot, `EKF2_GPS_CTRL` for PX4). Test: `sever_param_is_profile_specific`. (Residual: sever is still fire-and-forget on both — closing the loop with a `PARAM_VALUE` read-back is the next reliability step, MAVLink reviewer H1) |
| **P1-IMUSIGN** | 🔴 | Nav stack assumed the non-physical gravity-reaction convention (`[0,0,+g]` at rest); real IMUs/ArduPilot/PX4 emit specific force (`[0,0,-g]`), so real-hardware DR was chirality-mirrored during lateral/turning motion. Switched the nav math + sim + `mavsim` to specific force; boot check now flags the wrong sign. See the §6 row / N-01 |
| **P1-SUPCRASH** | 🔴 | The supervisor re-awaited the fusion `JoinHandle` after the select had already polled it to completion on the `task_died:fusion` path → Tokio panic → the `drain_pending_bg` forensic drain never ran, losing the incident record exactly when fusion crashed mid-spoof. Now guarded with `is_finished()` |
| **P1-DURATION** | 🟠 | `duration` defaulted to 60 s; a real deployment that omitted `[process].duration` ran ~62 s, exited 0, and `Restart=on-failure` did not restart it → drone undefended with no alarm. Now: real (non-synth) deployments **refuse to start** without an explicit duration; `duration = 0` means run-until-signal. `Process.duration` became `Option<u32>` to tell "chose 60" from "defaulted" |
| **P1-IMURATE** | 🟠 | `MAX_IMU_DT_S = 0.10` skipped integration on any gap > 100 ms; the README's own recommended 4 Hz IMU (250 ms) made **every** step skip → no residual ever computed → detector failed open silently. The gate is now derived from `--imu-rate` (≈2.5 nominal periods), and a run of skips escalates to one loud "DEAD-RECKONING STALLED" warning instead of silent failure |

All `cargo test` (107 lib + integration), `cargo clippy --all-targets`, and
`cargo fmt --all --check` stay green. The IMU-sign and PX4-sever fixes change
real-hardware behavior the sim cannot cover, so **SITL/HIL re-validation (incl.
the still-pending PX4 SITL milestone) is the right gate before real flight.**

### Phase 2 detection & MAVLink hardening (this pass)

Closed the high-severity evasion / false-positive / silent-failure findings
that ride on top of the Phase 1 correctness work.

| ID | Severity | Fix |
|---|---|---|
| **P2-DWELL (B-03)** | 🟠 | The dropout-pause cap was consecutive-only, so a spoofer modulating GPS cadence (N slow fixes + 1 fast, repeating) reset the streak every cycle and froze the Susp→Spoofed dwell **forever** — Spoofed never latched, RTL never fired. Added a CUMULATIVE per-episode pause budget (`detect.max_dwell_pause_s`, default 20 s): once exhausted the dwell resumes regardless of cadence, and `DwellPauseExceeded` fires. The budget renews on a genuine episode end (fresh entry / committed clear / operator reset) but is preserved across re-anchor vetoes. Tests: `cadence_modulation_cannot_freeze_dwell_forever`, `pause_budget_renews_on_committed_clear_but_not_on_veto`, `dropouts_in_normal_do_not_raise_dwell_pause_event` |
| **P2-FROZEN** | 🟠 | `FrozenGps` compared *consecutive* fixes within 0.5 m with a 3-fix streak — really a per-fix-displacement threshold tuned for the 1 Hz sim. At a real 5–10 Hz receiver, slow flight (1–2.5 m/s) moved <0.5 m per fix and false-fired within ~0.5 s → false Spoofed + RTL on a healthy aircraft. Now anchored at the streak's first fix and fired on cumulative **IMU-implied displacement** (≥5 m) while GPS stays pinned — GPS-rate-independent, and slow honest flight steadily exits the radius and resets |
| **P2-SEVERVERIFY (H1)** | 🟠 | `sever_gps` was fire-and-forget — the *most* safety-critical action had no confirmation while the less-critical RTL had four gates. Added a closed-loop read-back: the monitor tracks `PARAM_VALUE` echoes per-name (bounded, evict-oldest like the ACK map), and `verify_sever_engaged` confirms the GPS-disable param read back ≈0 with a causal (post-send) timestamp. Runs in parallel with RTL verify (never delays RTL); an unconfirmed read-back emits a CRITICAL event. `mavsim` now echoes `PARAM_VALUE` like a real autopilot. Tests: `param_value_round_trip_and_name_trim`, `param_value_not_stomped_by_unrelated_params`, end-to-end in `scenario_mavlink`. **(Further hardened in Phase 0 — D1 below: type-match + multi-echo dwell, since a single causal echo still left it weaker than the RTL verifier.)** |
| **P2-PORTLOCK** | 🟡 | The strict source-port lock silently dropped **all** telemetry (debug-level) under common bridge topologies (mavproxy `--out udp:`, mavlink-router, SiK) that forward from an ephemeral port — the detector went invisibly deaf. Now a rejected bootstrap HEARTBEAT from an unexpected port emits a LOUD rate-limited warning naming the remediation, and `--mav-allow-any-source-port` (CLI-only, TOML-rejected) locks on IP alone for those topologies. Test: `source_lock_any_port_accepts_ephemeral_bridge` |
| **P2-DRIFTNAN** | 🟡 | Defense-in-depth: a non-finite residual reaching the drift detector would `(s + NaN − k).max(0)` → 0, silently wiping every CUSUM and permanently poisoning `mag_noise_ewma`. Now skipped with the accumulators preserved. Test: `non_finite_residual_preserves_accumulated_evidence` |
| **P2-ERRUX (M5)** | 🟢 | Operator-facing CLI/config guards printed as a Debug-wrapped `Error: Io(Custom { .. })`. Added `FsError::Config`; `main` now prints the Display form and exits non-zero, so guards read as the clean one-line messages they were written as |

Also: rewrote the broken Linux `deploy/sitl/run-sitl-validation.sh` (it passed a
hostname to a clap `SocketAddr` and targeted the detector's own alias) to the
working static-IP topology, with a detector-event-log assertion so a native-EKF
LAND can't pass as a detector success; added a `proptest` invariant suite
(`tests/proptest_invariants.rs`) over the PX4 mode codec, the tangent-plane
projection, and the ingest plausibility gate, making the declared dev-dependency
real; and corrected the `SIM_GPS_GLITCH_*` units (degrees, not meters) in
`docs/sitl.md`.

All `cargo test` (114 lib + 9 property + integration), `cargo clippy
--all-targets -D warnings`, and `cargo fmt --all --check` stay green.

### Phase 3 hardware-mounting + realistic-robustness (this pass)

| ID | Severity | Fix / finding |
|---|---|---|
| **P3-AXISMAP** | 🟡 | **Fixed.** I2C IMUs mounted in any non-FRD orientation were passed through with raw chip axes (N-01 residual), so DR integrated a rotated signal. New `hw/axis_map.rs` — a signed axis permutation validated as a proper rotation (det +1, so it can't chirality-flip the gyro), applied at the i2c read path with an `--imu-axis-map "Y,-X,Z"` / `[i2c].axis_map` surface. Cross-platform unit-tested; Linux compile CI-verified. Physical validation still needs real hardware. |
| **P3-SITLCI** | 🟢 | **Fixed (S-04).** SITL validation was manual-only. Added `.github/workflows/sitl.yml` — a nightly + on-demand job that builds the detector image, runs `deploy/sitl/run-sitl-validation.sh` (closed-loop ArduPilot SITL), gates on the detector's own event log (Spoofed + ActionAcked), and uploads the event log/forensic dumps. Best-effort (third-party SITL image + Docker networking), so it's NOT a per-push gate. |
| **P3-PX4** | 🟡 | **Detector side done + documented; live run pending environment.** PX4 support (sever param, packed-mode RTL, autopilot cross-check) is implemented and unit-tested (Phase 1), and the manual PX4 SITL run is documented (`docs/sitl.md` §"Running against PX4 SITL", with a precise status callout). NOT done: a containerized one-command PX4 harness + an actual end-to-end PX4 SITL run — PX4's Gazebo/jMAVSim stack differs from ArduPilot's and can't be exercised from the Windows dev box. This is the standing **PX4 SITL milestone**. |

#### ✅ FIXED (within the tested envelope) — false alarms under realistic sensor error

A realistic-noise harness (`tests/scenario_realistic.rs`, + accel-bias and a
`CircleHorizontal` turning trajectory in the sim) characterized a detector
false-latch: under realistic sensor error the detector could sever GPS and
command RTL on a perfectly healthy drone. The existing integration scenarios
pass only because they use ~10× cleaner GPS (σ=0.3 m) and ~30× cleaner IMU noise
than reality. Three compounding causes were identified; **steps 1 and 3 are now
landed and all three realistic characterizations pass (`spoofed=false`, identical
to the low-noise baseline):**

1. **Per-axis CUSUM is not adaptive — ✅ FIXED (step 1).** The per-axis drift `k`
   was fixed at 1.0 m (0.4σ at GPS σ=2.5 m) while only the *magnitude* lane learned
   its noise floor, so the per-axis CUSUM ramped on honest noise (~29 false fires /
   30k fixes, `per_axis_no_false_alarm_on_realistic_gps_noise`). Fixed in
   `detect/drift.rs`: per-axis `eff_k = max(cusum_k_m, axis_noise × 2.5)` where
   `axis_noise` is an online `|rn|`/`|re|` EWMA, learned during a warmup then
   updated ONLY while that axis's CUSUMs are quiescent — so an in-progress drift
   can't inflate the floor and absorb its own signal (the missed-detection trap).
   Mirrors the B-02 magnitude lane.
2. **DR drift vs the cumulative-since-anchor residual — ✅ no longer false-fires on
   LINEAR flight (step 1); sliding-window residual still advisable (step 2).**
   Realistic IMU noise/bias random-walks the never-re-anchored DR position, and the
   residual reads that wander as a persistent offset. On the 90 s linear
   characterizations the adaptive per-axis `eff_k` (≈2σ) also tolerates this
   *bounded* excursion: `realistic_noise_does_not_false_fire` and
   `realistic_bias_does_not_false_fire` now produce the same clean result as the
   low-noise baseline and are PROMOTED (no longer `#[ignore]`d). The principled
   fix — a sliding-window residual (GPS-vs-DR *displacement* over a window) and/or
   in-flight accel-bias estimation — remains the recommended **step 2** for
   robustness on longer / more-dynamic flights where unbounded cumulative drift
   could still exceed the adaptive floor.
3. **Centripetal attitude coupling — ✅ FIXED (step 3).** In a sustained coordinated
   turn the centripetal specific force tilted the Madgwick "down" (~6°), coupling
   through gravity compensation into a ~90 m residual. Fixed in `nav/attitude.rs` +
   `nav/mod.rs`: subtract an `ω × v` linear-acceleration estimate from the
   accelerometer before the gravity gradient (`update_with_lin_accel`).
   `sustained_turn_does_not_false_fire` now produces the clean baseline result.
   **SECURITY NOTE:** the estimate uses the bias-corrected gyro and the
   DEAD-RECKONED velocity — NOT GPS, despite the original "GPS-velocity-derived"
   framing. Feeding the attacker-controlled GPS velocity into attitude would let a
   spoofed velocity bias DR toward the attacker's track (a new missed-detection
   surface); `v_dr` freezes GPS-independent in Suspicious/Spoofed, so the correction
   stays honest exactly when it matters. The compensation is bounded and acts only
   during real angular maneuvers, so it can only remove a false alarm — never hide a
   spoof: guarded by `spoof_during_turn_is_still_detected` (a real drift mid-turn
   still latches Spoofed) and the `centripetal_compensation_keeps_turn_attitude_level`
   unit test. (Gating the accel correction off during turns was tried and is WRONG —
   it removes the gravity reference and lets gyro bias diverge the attitude.)

**Status:** all three realistic-noise characterizations pass (no false-latch on
linear flight, accel/gyro bias, or a sustained coordinated turn), with the full
attack-regression suite green — no desensitization (`per_axis_real_drift_still_fires…`,
`spoof_during_turn_is_still_detected`, plus every `scenario_*` attack test pass).

**Step 2 — no-Doppler degraded-mode policy (done).** The sliding-window residual
originally scoped for step 2 turned out NOT to be justified for normal flight: the
complementary velocity blend already bounds DR drift indefinitely when GPS Doppler
is present (verified clean to 20 min — `long_doppler_flight_does_not_false_fire`).
The real false-latch is the **no-Doppler** case: when GPS reports position but no
velocity (a failing receiver, an intermittent dropout, or an attacker stripping
Doppler to disable velocity-mismatch detection), the blend goes silent, DR drifts
unbounded, and the cumulative residual false-latched a healthy drone in ~2 min.
Fix (`fusion.rs` + `state_machine.rs`): once Doppler has been absent for ≥10
consecutive fixes, DISABLE the cumulative-residual detectors (drift CUSUM +
hard-residual jump) — without a velocity reference, accumulated DR drift is
indistinguishable from a spoof, so they only false-latch. The Doppler-INDEPENDENT
detectors (frozen-GPS, vertical-rate) stay active and can still escalate to Spoofed.
Inherent tradeoff (accepted; operator warned via the no-Doppler SyncWarning):
horizontal slow-drift / teleport detection is suspended while Doppler is absent —
there is no independent velocity reference to separate a spoof from honest DR drift,
and coupling GPS position into DR to bound it would both hide slow spoofs AND blind
frozen-GPS. Guarded by `no_doppler_flight_does_not_false_fire` (no false-latch when
Doppler drops mid-flight) and `frozen_gps_during_no_doppler_is_still_detected` (a
stuck/replayed GPS with no Doppler still latches Spoofed).

**FSM clear-gate (done):** `pos_clean` used the base `cusum_k_m` (1 m), below the
realistic GPS residual (~3 m at σ=2.5 m), so a clean-but-noisy flight could get
STUCK in Suspicious after a transient anomaly. Now noise-aware (`pos_thresh =
max(base, mag_noise_floor × 3)` — the same step-1 adaptive floor), with two safety
gates so the generous threshold can never clear a real attack: clearing requires
NO detector firing this fix AND the drift CUSUMs quiescent (a slow drift
accumulating below the clean threshold keeps a CUSUM elevated, barring a clear —
it escalates instead). Tests: `noisy_clean_flight_clears_from_suspicious`,
`accumulating_drift_is_not_cleared_away`. Each detection-math change above was a
dedicated, separately-validated pass: a careless change would cause MISSED
detections (worse than false alarms).

All `cargo test` (incl. all promoted regressions and the missed-detection guards),
`cargo clippy --all-targets -D warnings`, and `cargo fmt --all --check` are green.
No realistic characterizations remain `#[ignore]`d.

### Phase 0 — real-hardware-readiness hardening (this pass)

The detection math and live-SITL validation (ArduPilot 3-mode + PX4) are complete;
this pass closes the standing pre-flight readiness findings — the gaps between
"green against SITL loopback" and "trustworthy over a real, jittery, attacker-reachable
link."

| ID | Severity | Fix |
|---|---|---|
| **D1** | 🟠 | **Sever read-back was under-hardened vs. RTL.** The *most* safety-critical action (`verify_sever_engaged`) confirmed on the FIRST causal `PARAM_VALUE`≈0 — no type-match, no dwell — while the less-critical RTL verifier had four gates. A single injected `PARAM_VALUE=0` (an on-link attacker who wins the source lock, or under `--mav-no-source-filter`) could fake the sever confirmation AND thereby SUPPRESS the CRITICAL "sever unconfirmed" operator warning, masking that the drone was RTL-ing on still-spoofed GPS. Now three gates, all required (`sever_echo_confirms`): **causality** (echo received after our send), **type-match** (echo's wire `param_type` == the type we set — `INT8`/`GPS_TYPE`, `INT32`/`EKF2_GPS_CTRL`), and a **multi-echo dwell** (`SEVER_VERIFY_DWELL_ECHOES`=2 distinct post-send echoes, the sever analogue of the RTL fresh-HEARTBEAT dwell). The dwell counts arrivals at the *recording* site (`ParamEcho.recv_count`) against a baseline snapshotted at send-time, because the verify task only begins polling after every PARAM_SET retry (hence every echo) has already landed — a latest-only sample would coalesce them and starve the dwell. `sever_gps` repeats the PARAM_SET 3× so a real sever reliably yields the dwell (and tolerates one dropped datagram). Tests: `sever_echo_confirms_requires_all_gates` (exhaustive gate coverage), `scenario_mavlink` now echoes `PARAM_VALUE` and asserts the read-back CONFIRMS (no CRITICAL `ActionFailed`). **SITL note:** the exact per-PARAM_SET echo cadence is firmware behavior the nightly SITL job exercises against real ArduPilot/PX4; if a real autopilot echoes only once per set, lower the dwell or solicit echoes with `PARAM_REQUEST_READ`. |
| **IMU-RATE-AUTO** | 🟠 | **A mis-set `--imu-rate` silently fails the detector OPEN.** `max_imu_dt_s` (the integration-gap gate) is sized from `--imu-rate` at startup; if the operator leaves the 100 Hz default against a real ~4–10 Hz autopilot stream, every inter-sample gap exceeds the gate, so dead-reckoning never runs, no residual is computed, and the GPS cross-check is silently absent (the `DEAD-RECKONING STALLED` warning fires, but only after a run of skips — a sharp trap). `DeadReckoner::step` now AUTO-DERIVES the gate from the cadence measured during static-init and WIDENS it (`IMU_GATE_PERIOD_FACTOR`×observed period) when the configured value is too tight, logging the observed rate. It only widens (never tightens — tightening could false-trip the stall gate on jitter) and only up to `AUTO_IMU_GATE_MAX_S` (1 s ≈ 2.5 Hz floor), so a boot-time cadence manipulation can't open the runtime stall gate without bound; an implausibly slow stream is left to the STALLED warning. Runs once at init, before any post-init integration, so runtime stall detection stays sharp. Tests: `imu_gate_auto_widens_on_slow_stream`, `imu_gate_not_widened_on_fast_stream`, `imu_gate_widen_is_capped_for_implausibly_slow_stream` |
| **AXISMAP-MAV** | 🟡 | **`--imu-axis-map` was a silent no-op on the MAV IMU path.** The signed axis permutation (`hw/axis_map.rs`) is applied ONLY on the I2C read path; `imu_from_msg` passes `SCALED_IMU`/`HIGHRES_IMU` axes straight through. An operator who set `--imu-axis-map` with `--imu-source mav` (e.g. believing it corrected a non-FRD mounting) got no remap and no warning. The autopilot has already rotated its telemetry into the FRD body frame, so the remap is correctly inert there — but the silence was the bug. The binary now emits a LOUD warning when a non-identity `--imu-axis-map` is set with `--imu-source mav`, naming that the remap applies only to `--imu-source i2c`. (No detector behavior change; the MAV IMU is documented trusted-FRD.) |
| **TIME-SYNC** | 🟠 | **GPS/IMU were stamped at ARRIVAL time, not sensor time** — the single biggest real-link risk. `imu_from_msg`/`gps_from_msg` stamped every sample `Timestamp::now_mono()` at parse, discarding the autopilot's `time_usec`/`time_boot_ms`. The residual interpolation assumes a consistent relative GPS↔IMU latency; that holds on SITL loopback but NOT over a real USB/UART/SiK/UDP link, where ~50 ms of differential jitter at 15 m/s injects ~0.75 m of residual NOISE — inflating the adaptive floor and desensitizing the detector. The MAV listener now runs a per-stream `ClockAligner` that anchors on the first message's arrival and advances by the SENSOR-reported deltas, so per-message jitter no longer moves the GPS↔IMU alignment (only one constant first-message-latency offset remains, absorbed by the adaptive floor). **SAFE BY CONSTRUCTION:** on any anomaly — no sensor timestamp, a regression (reboot/wrap/reorder), or a skew from arrival beyond `CLOCK_ALIGN_MAX_SKEW` (2 s; the sensor clock isn't tracking wall time) — it falls back to arrival time, i.e. exactly the prior behavior. Per-stream so a boot-relative IMU base and an epoch GPS base each map by their own deltas (the absolute base cancels). **Security:** on an unsigned link an attacker forging MAVLink could shift `time_usec` to move the alignment, but only by the bounded 2 s (≤30 m at 15 m/s) before the guard re-anchors — and such an attacker can already forge `lat`/`lon` directly (the spoof the detector exists to catch), so this adds no capability beyond the already-accepted on-link-forger threat (MAVLink signing remains the defense). Tests: `ClockAligner` gate suite (`aligner_*`) + `gps_sensor_us`/`imu_sensor_us` extractors; `scenario_mavlink` exercises it end-to-end (mavsim stamps `time_usec`). **Residual: the jitter-REDUCTION benefit is unit-validated for the mapping logic and exercised on the ~0-jitter SITL/mavsim path; quantifying it against a real jittery link needs HITL.** |
| **PX4-ACKED** | 🟢 | **PX4 closed loop logged `ActionUnconfirmed`, not the stronger `ActionAcked`.** Severing `EKF2_GPS_CTRL=0` removes PX4's only position source → PX4 LANDs + disarms → the verifier's armed-during-dwell gate (correctly) won't certify RTL on a disarming autopilot. **The detector is correct and is NOT changed** — this is exactly the deliberate **C-15** trade-off ("requires armed during dwell; false-positive ActionUnconfirmed is a UX issue, not a security failure"), and relaxing the armed gate to win it would weaken the **W-ARMED** single-packet defence. The fix is harness-side: `sitl_harness.py --px4-ev-fallback` configures EKF2 external-vision fusion (`EKF2_EV_CTRL`/`EKF2_AID_MASK`) and streams a true-position `VISION_POSITION_ESTIMATE` (harness→PX4 only; detector never sees it), so PX4 retains a non-GPS position and holds an armed RTL through the verify dwell → `ActionAcked`. Wired as a **non-gating** `continue-on-error` CI leg (`px4-action-acked`, `REQUIRE_ACTION_ACKED=1`) so it can never regress the proven `px4-closed-loop` gate. **Status: implemented, pending nightly-CI validation** (PX4 SITL can't run on the Windows dev box; the EV params/frames may need CI iteration). The closed-loop gate (Spoofed + sever-verified + PX4 reaches RTL-equivalent) already passes regardless — this is the *stronger* read-back, not a blocker. |

---

## 11. References and proof execution

- **Run every defense test:**
  ```bash
  cargo test --lib && cargo test --tests
  ```
- **Just the integration scenarios:**
  ```bash
  cargo test --test scenario_clean --test scenario_jump --test scenario_drift \
             --test scenario_boot_anchor --test scenario_re_anchor_rejected \
             --test scenario_frozen_gps --test scenario_preflight \
             --test scenario_mavlink --test scenario_forensic
  ```
- **Lint:**
  ```bash
  cargo clippy --all-targets -- -D warnings
  ```
- **Live demo against the synth pipeline:**
  ```bash
  cargo run --release -- --scenario sudden-jump --duration 50 \
      --json-log /tmp/events.jsonl --forensic-dir /tmp/forensic
  ```
- **SITL reproduction:** see `docs/sitl.md` and `docs/sitl_smoke.sh`.

The current as-of date for this document matches the most recent commit
to `src/`; when defenses change materially, this file should change in
the same commit.
