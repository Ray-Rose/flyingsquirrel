//! Verifies the Phase O forensic ring buffer end to end.
//!
//! Drives a sudden-jump scenario with `forensic_dir` set to a tempdir,
//! asserts that on the Spoofed transition a single `spoof-*.json` file
//! appears with the expected top-level shape (schema_version, trigger,
//! residuals, gps_fixes, imu_samples, attestation), and that fusion
//! emitted a `ForensicDumpWritten` event so the rest of the system can
//! observe the dump completion.
//!
//! Why this lives as an integration test (not a unit test on `forensic`):
//! the unit tests cover `ForensicBuffer` and `dump_to_disk` in isolation,
//! but the actual production path is "Fusion sees Spoofed → calls snapshot
//! → spawns disk write → flips dump_fired_flag → emits event." Only an
//! end-to-end run exercises that wiring. A regression that broke the
//! trigger condition (e.g., snapshotted after `try_action` instead of
//! before) would still pass the unit tests.

mod common;

use common::{count_kind, reached_state, run_scenario, Scenario};
use flyingsquirrel::sim::spoof::SpoofPattern;
use flyingsquirrel::types::{NavStateKind, SpoofKind};

#[tokio::test(start_paused = true)]
async fn spoofed_transition_writes_forensic_dump_with_expected_schema() {
    let dir = tempfile::tempdir().expect("tempdir");
    let outcome = run_scenario(Scenario {
        duration_s: 50,
        speed_mps: 8.0,
        // 200 m east teleport at t=30 s — same shape as scenario_jump,
        // chosen because it reliably escalates to Spoofed within the
        // 10 s dwell.
        pattern: SpoofPattern::SuddenJump {
            apply_at_s: 30.0,
            jump_north_m: 0.0,
            jump_east_m: 200.0,
        },
        seed: 0xF0_5E_71_C0,
        forensic_dir: Some(dir.path().to_path_buf()),
        ..Scenario::default()
    })
    .await;

    // Sanity: the scenario reached Spoofed. If it didn't, the rest of the
    // assertions are meaningless (the dump only fires on Spoofed transition).
    assert!(
        reached_state(&outcome.events, NavStateKind::Spoofed),
        "precondition: state machine must reach Spoofed for the dump to trigger"
    );

    // The fusion task should have emitted exactly one ForensicDumpWritten
    // event. More than one would mean the once-per-process guard broke;
    // zero would mean the trigger didn't fire (or the write failed silently
    // — in which case there'd be a ForensicDumpFailed instead, asserted below).
    let written = count_kind(&outcome.events, SpoofKind::ForensicDumpWritten);
    let failed = count_kind(&outcome.events, SpoofKind::ForensicDumpFailed);
    let suppressed = count_kind(&outcome.events, SpoofKind::ForensicDumpSuppressed);
    assert_eq!(
        failed, 0,
        "forensic dump must not fail in synth scenario (got {} failures)",
        failed
    );
    assert_eq!(
        written, 1,
        "expected exactly 1 ForensicDumpWritten event; got written={} failed={} suppressed={}",
        written, failed, suppressed
    );

    // Exactly one spoof-*.json file should exist on disk, no .tmp leftovers.
    let entries: Vec<std::path::PathBuf> = std::fs::read_dir(dir.path())
        .expect("read tempdir")
        .filter_map(|e| e.ok().map(|d| d.path()))
        .collect();
    let json_files: Vec<&std::path::PathBuf> = entries
        .iter()
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("json"))
        .collect();
    let tmp_files: Vec<&std::path::PathBuf> = entries
        .iter()
        .filter(|p| {
            p.extension()
                .and_then(|s| s.to_str())
                .map(|s| s == "tmp")
                .unwrap_or(false)
        })
        .collect();
    assert_eq!(
        json_files.len(),
        1,
        "expected exactly 1 spoof-*.json dump file, got {:?}",
        entries
    );
    assert!(
        tmp_files.is_empty(),
        "no .tmp files should remain after a successful atomic write, got {:?}",
        tmp_files
    );
    let dump_path = json_files[0];
    let fname = dump_path.file_name().unwrap().to_string_lossy().to_string();
    assert!(
        fname.starts_with("spoof-") && fname.ends_with(".json"),
        "filename should match spoof-*.json pattern, got {:?}",
        fname
    );

    // Parse the dump and assert on its shape. Failing to parse means the
    // schema regressed.
    let body = std::fs::read_to_string(dump_path).expect("read dump");
    let v: serde_json::Value = serde_json::from_str(&body).expect("dump must be valid JSON");

    // Top-level required fields.
    for field in [
        "schema_version",
        "captured_at_utc",
        "captured_at_mono_ns",
        "window_s",
        "attestation",
        "trigger",
        "residuals",
        "gps_fixes",
        "imu_samples",
    ] {
        assert!(
            v.get(field).is_some(),
            "dump missing required top-level field `{}`; full body: {}",
            field,
            body
        );
    }

    // Schema version must match the producer's current version. If this
    // ever needs bumping the test should be updated deliberately — that's
    // the whole point of versioning the schema.
    assert_eq!(
        v["schema_version"].as_u64(),
        Some(1),
        "schema_version regression"
    );

    // Trigger envelope: tag="spoofed_transition", to="spoofed", residual_m>0.
    let trigger = &v["trigger"];
    assert_eq!(
        trigger["kind"].as_str(),
        Some("spoofed_transition"),
        "trigger kind regression"
    );
    // NavStateKind serializes PascalCase (the default for serde derives
    // without `rename_all`). The ForensicTrigger enum itself uses snake_case
    // via its `#[serde(rename_all = "snake_case")]`, but the inner field
    // delegates to NavStateKind's own impl.
    assert_eq!(
        trigger["to"].as_str(),
        Some("Spoofed"),
        "trigger.to should be the new latched state"
    );
    assert!(
        trigger["residual_m"].as_f64().unwrap_or(0.0) > 0.0,
        "residual_m should be the magnitude that caused the latch, got {:?}",
        trigger.get("residual_m")
    );

    // The three telemetry arrays should be populated. Exact counts depend
    // on rates and the window, but at minimum:
    //   * gps_fixes: ~1 Hz × <=30 s window → expect at least a handful.
    //   * imu_samples: 100 Hz × window → expect hundreds.
    //   * residuals: 1 per evaluated GPS fix → expect at least a handful.
    let gps_n = v["gps_fixes"].as_array().expect("gps_fixes array").len();
    let imu_n = v["imu_samples"]
        .as_array()
        .expect("imu_samples array")
        .len();
    let res_n = v["residuals"].as_array().expect("residuals array").len();
    assert!(
        gps_n >= 5,
        "expected several GPS fixes in window, got {}",
        gps_n
    );
    assert!(
        imu_n >= 100,
        "expected hundreds of IMU samples in window, got {}",
        imu_n
    );
    assert!(
        res_n >= 5,
        "expected several residuals in window, got {}",
        res_n
    );

    // Spot-check the first GPS record has the fields the JSON schema promises.
    let first_gps = &v["gps_fixes"][0];
    for field in ["t_secs", "lat_deg", "lon_deg", "alt_m"] {
        assert!(
            first_gps.get(field).is_some(),
            "gps_fix record missing field `{}`",
            field
        );
    }

    // Attestation must at minimum carry the crate version + host metadata.
    let att = &v["attestation"];
    assert!(
        att.get("crate_version").is_some(),
        "attestation.crate_version missing"
    );
    assert!(att.get("host_os").is_some(), "attestation.host_os missing");
    assert!(
        att.get("host_arch").is_some(),
        "attestation.host_arch missing"
    );
}
