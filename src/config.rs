//! TOML configuration file support.
//!
//! Operators pin known-good deployment configs in version control and run
//! `flyingsquirrel --config path.toml`. CLI flags still work and OVERRIDE
//! the config file's values — handy for one-off adjustments without
//! mutating the canonical config.
//!
//! Use `--print-config` to emit the merged config back out (useful for
//! debugging and for generating a starter template from defaults).
//!
//! The shape mirrors the CLI flags 1:1 — every flag has a config field
//! under the matching section. Sections are grouped for readability:
//!   [sources]      gps_source, imu_source, controller
//!   [scenario]     synth-only knobs (scenario, seed)
//!   [serial]       gps_port, gps_baud
//!   [i2c]          imu_bus, imu_addr, imu_rate
//!   [mav]          bind, target, target_system, target_component, no_source_filter, no_sysid_filter, allow_any_source_port, vehicle
//!   [boot_anchor]  expected_home, max_home_distance_m, allow_no_boot_anchor_check
//!   [process]      duration, json_log, log_level, allow_synth_to_mav

use crate::error::FsError;
use serde::{Deserialize, Serialize};
use std::path::Path;

/// Top-level config matching the CLI surface. All fields are optional in
/// the TOML representation; defaults match the CLI defaults.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct AppConfig {
    pub sources: Sources,
    pub scenario: Scenario,
    pub serial: Serial,
    pub i2c: I2c,
    pub mav: Mav,
    pub boot_anchor: BootAnchor,
    pub forensic: Forensic,
    pub process: Process,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Sources {
    pub gps_source: String, // synth | mav | serial
    pub imu_source: String, // synth | mav | i2c
    pub controller: String, // console | mav
}

impl Default for Sources {
    fn default() -> Self {
        Self {
            gps_source: "synth".into(),
            imu_source: "synth".into(),
            controller: "console".into(),
        }
    }
}

/// Scenario config section. The struct is renamed to `Scenario_` in re-exports
/// because the binary uses a separate `Scenario` enum.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Scenario {
    pub scenario: String, // clean | sudden-jump | gradual-drift
    pub seed: u64,
}

// Public alias so the binary can import `flyingsquirrel::config::Scenario_`
// without colliding with its own `enum Scenario`.
pub use Scenario as Scenario_;

impl Default for Scenario {
    fn default() -> Self {
        Self {
            scenario: "gradual-drift".into(),
            // High bit cleared so the value fits in TOML's i64 range
            // (toml-rs serializes integers as i64). Keep this constraint in
            // mind when documenting custom seeds for operators.
            seed: 0x7F1F_5184_0000_0000,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Serial {
    pub gps_port: String,
    pub gps_baud: u32,
}

impl Default for Serial {
    fn default() -> Self {
        Self {
            gps_port: "/dev/ttyUSB0".into(),
            gps_baud: 9600,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct I2c {
    pub imu_bus: String,
    pub imu_addr: u8,
    pub imu_rate: f32,
    /// Chip-to-body axis remap, e.g. `"Y,-X,Z"`. Default `"X,Y,Z"` (identity).
    /// See `hw::axis_map::AxisMap`; applied only on the I2C IMU path.
    pub axis_map: String,
}

impl Default for I2c {
    fn default() -> Self {
        Self {
            imu_bus: "/dev/i2c-1".into(),
            imu_addr: 0x68,
            imu_rate: 100.0,
            axis_map: "X,Y,Z".into(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Mav {
    pub bind: String,
    pub target: String,
    pub target_system: u8,
    pub target_component: u8,
    pub no_source_filter: bool,
    pub no_sysid_filter: bool,
    /// Lock the source on IP alone (accept any source port). Safety-bypass —
    /// can only be set via the `--mav-allow-any-source-port` CLI flag, never
    /// via TOML (rejected at load). The field exists so `--print-config` can
    /// round-trip and so the TOML reject check can see an attempted `= true`.
    pub allow_any_source_port: bool,
    /// Vehicle profile. Supported values: `"ardu-copter"`, `"px4"`.
    /// See `mav::monitor::VehicleProfile` for the dispatch surface.
    pub vehicle: Option<String>,
}

impl Default for Mav {
    fn default() -> Self {
        Self {
            bind: "127.0.0.1:14551".into(),
            target: "127.0.0.1:14550".into(),
            target_system: 1,
            target_component: 1,
            no_source_filter: false,
            no_sysid_filter: false,
            allow_any_source_port: false,
            vehicle: None,
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Forensic {
    /// Directory where forensic dump JSON files are written. `None` =
    /// forensic capture disabled. Operator-supplied path; install.sh
    /// creates `/var/lib/flyingsquirrel/forensic/` as a sensible default.
    pub dir: Option<String>,
    /// Seconds of GPS/IMU/residual history to retain in the ring buffer.
    /// `None` uses `forensic::DEFAULT_WINDOW_S` (30s).
    pub window_s: Option<f64>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct BootAnchor {
    /// `"lat,lon"` decimal degrees. `None` = boot-anchor cross-check disabled.
    pub expected_home: Option<String>,
    pub max_home_distance_m: Option<f64>,
    pub allow_no_boot_anchor_check: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Process {
    /// Run duration in seconds. `None` = not set in this config (the binary
    /// then falls back to the CLI default for synth scenarios, but REFUSES to
    /// start a real/MAV deployment — see the duration footgun guard in
    /// `flyingsquirrel.rs`). `Some(0)` = run until stopped by signal. This is
    /// an `Option` specifically so the binary can tell "operator explicitly
    /// chose 60s" apart from "operator forgot to set a duration" — the latter
    /// silently disarmed production deployments after 60s (Restart=on-failure
    /// does not restart a clean exit).
    pub duration: Option<u32>,
    pub json_log: Option<String>,
    pub log_level: String,
    pub allow_synth_to_mav: bool,
}

impl Default for Process {
    fn default() -> Self {
        Self {
            duration: None,
            json_log: None,
            log_level: "info".into(),
            allow_synth_to_mav: false,
        }
    }
}

impl AppConfig {
    /// Load a config from a TOML file. Returns `Default::default()` if the
    /// path is None.
    ///
    /// AUDIT F-01 fix: TOML config cannot enable safety bypasses. The CLI
    /// merge rule is "explicit CLI wins, otherwise TOML fills in" — but
    /// clap can't distinguish "explicitly --no-flag" from "flag absent and
    /// defaulted to false" without paired `--no-*` flags. That meant a
    /// pinned config with e.g. `allow_synth_to_mav = true` PERMANENTLY
    /// disabled the friendly-fire-RTL guard with no CLI override possible.
    ///
    /// Policy: safety-bypass bools (`allow_synth_to_mav`,
    /// `allow_no_boot_anchor_check`, `no_source_filter`, `no_sysid_filter`,
    /// `allow_any_source_port`) can only be ENABLED via explicit CLI flag,
    /// never via TOML. The
    /// config file may comment-document them, but the actual `true` value
    /// has to be present at the command line where it's visible in
    /// `ps`/journald and reviewable per-invocation.
    pub fn load(path: Option<&Path>) -> Result<Self, FsError> {
        let Some(p) = path else {
            return Ok(Self::default());
        };
        let text = std::fs::read_to_string(p).map_err(|e| {
            FsError::Io(std::io::Error::new(
                e.kind(),
                format!("config file {}: {}", p.display(), e),
            ))
        })?;
        let cfg: AppConfig = toml::from_str(&text).map_err(|e| {
            FsError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("config file {} parse error: {}", p.display(), e),
            ))
        })?;
        cfg.reject_toml_safety_bypasses(p)?;
        Ok(cfg)
    }

    /// Fail-closed check that the TOML doesn't attempt to enable any
    /// safety-bypass bool. See `load` for the rationale.
    fn reject_toml_safety_bypasses(&self, path: &Path) -> Result<(), FsError> {
        let offenders: Vec<&'static str> = [
            (
                "process.allow_synth_to_mav",
                self.process.allow_synth_to_mav,
            ),
            (
                "boot_anchor.allow_no_boot_anchor_check",
                self.boot_anchor.allow_no_boot_anchor_check,
            ),
            ("mav.no_source_filter", self.mav.no_source_filter),
            ("mav.no_sysid_filter", self.mav.no_sysid_filter),
            ("mav.allow_any_source_port", self.mav.allow_any_source_port),
        ]
        .into_iter()
        .filter_map(|(name, v)| if v { Some(name) } else { None })
        .collect();
        if !offenders.is_empty() {
            return Err(FsError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!(
                    "config file {} attempts to enable safety bypass(es) via TOML: {}. \
                     Safety bypasses can only be enabled with an EXPLICIT CLI flag \
                     (e.g. --allow-synth-to-mav). Remove the `= true` from the TOML \
                     and pass the equivalent flag at the command line where it's \
                     visible per-invocation in ps/journald.",
                    path.display(),
                    offenders.join(", ")
                ),
            )));
        }
        Ok(())
    }

    /// Emit the config back as TOML — used by `--print-config` so an
    /// operator can capture the resolved runtime state (defaults + file +
    /// CLI overrides combined).
    pub fn to_toml(&self) -> Result<String, FsError> {
        toml::to_string_pretty(self)
            .map_err(|e| FsError::Io(std::io::Error::other(format!("config serialize: {}", e))))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_round_trips_through_toml() {
        let c = AppConfig::default();
        let s = c.to_toml().unwrap();
        let parsed: AppConfig = toml::from_str(&s).unwrap();
        // Spot-check a few fields
        assert_eq!(parsed.sources.gps_source, "synth");
        assert_eq!(parsed.mav.target_system, 1);
        // duration defaults to None (unset) — an omitted [process].duration must
        // NOT silently become a 60s run on a real deployment. None round-trips as
        // an absent key (like the other Option fields).
        assert_eq!(parsed.process.duration, None);
    }

    #[test]
    fn partial_config_uses_defaults_for_missing_fields() {
        let s = r#"
            [sources]
            gps_source = "mav"
            controller = "mav"
        "#;
        let c: AppConfig = toml::from_str(s).unwrap();
        assert_eq!(c.sources.gps_source, "mav");
        assert_eq!(c.sources.imu_source, "synth"); // default
        assert_eq!(c.sources.controller, "mav");
        // Unrelated section uses defaults:
        assert_eq!(c.mav.target_system, 1);
    }

    #[test]
    fn unknown_field_rejected() {
        let s = r#"
            [sources]
            gps_source = "synth"
            bogus_field = "oops"
        "#;
        let r: Result<AppConfig, _> = toml::from_str(s);
        assert!(r.is_err(), "deny_unknown_fields should reject typos");
    }

    #[test]
    fn toml_safety_bypasses_are_rejected_at_load_time() {
        // Audit F-01: a pinned TOML must NEVER be able to silently enable
        // safety bypasses. Each of these `true` values should produce a
        // load-time error so the operator either removes them or moves them
        // to the CLI where they're visible per-invocation.
        for snippet in &[
            "[process]\nallow_synth_to_mav = true\n",
            "[boot_anchor]\nallow_no_boot_anchor_check = true\n",
            "[mav]\nno_source_filter = true\n",
            "[mav]\nno_sysid_filter = true\n",
            "[mav]\nallow_any_source_port = true\n",
        ] {
            // Write to tempfile, attempt load.
            let mut tmp = tempfile::NamedTempFile::new().unwrap();
            use std::io::Write;
            tmp.write_all(snippet.as_bytes()).unwrap();
            let result = AppConfig::load(Some(tmp.path()));
            assert!(
                result.is_err(),
                "TOML snippet must be rejected at load: {:?}",
                snippet
            );
            let err = format!("{}", result.unwrap_err());
            assert!(
                err.contains("safety bypass"),
                "error must explain why: {}",
                err
            );
        }
    }

    #[test]
    fn toml_safety_bypasses_false_load_ok() {
        // Setting them to `false` (or omitting them) is fine — the default
        // is the safe value. Only `true` is rejected.
        let snippet = "[process]\nallow_synth_to_mav = false\n\
            [boot_anchor]\nallow_no_boot_anchor_check = false\n\
            [mav]\nno_source_filter = false\nno_sysid_filter = false\nallow_any_source_port = false\n";
        let mut tmp = tempfile::NamedTempFile::new().unwrap();
        use std::io::Write;
        tmp.write_all(snippet.as_bytes()).unwrap();
        AppConfig::load(Some(tmp.path())).expect("explicit-false should load fine");
    }

    #[test]
    fn sample_full_config_loads() {
        let s = r#"
[sources]
gps_source = "mav"
imu_source = "mav"
controller = "mav"

[mav]
bind = "0.0.0.0:14551"
target = "192.168.1.50:14550"
target_system = 1
target_component = 1
vehicle = "ardu-copter"

[boot_anchor]
expected_home = "40.7128,-74.0060"
max_home_distance_m = 5000.0

[process]
duration = 300
log_level = "info"
        "#;
        let c: AppConfig = toml::from_str(s).unwrap();
        assert_eq!(c.sources.gps_source, "mav");
        assert_eq!(c.mav.vehicle.as_deref(), Some("ardu-copter"));
        assert_eq!(
            c.boot_anchor.expected_home.as_deref(),
            Some("40.7128,-74.0060")
        );
        assert_eq!(c.process.duration, Some(300));
    }
}
