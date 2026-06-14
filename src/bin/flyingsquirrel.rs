use clap::parser::ValueSource;
use clap::{CommandFactory, Parser, ValueEnum};
use flyingsquirrel::action::console::ConsoleController;
use flyingsquirrel::action::FlightController;
use flyingsquirrel::attestation::Attestation;
use flyingsquirrel::config::AppConfig;
use flyingsquirrel::error::FsError;
use flyingsquirrel::fusion::FusionConfig;
use flyingsquirrel::hw::serial_gps::{SerialGpsConfig, SerialGpsSource};
use flyingsquirrel::ingest::{GpsSource, ImuSource};
use flyingsquirrel::mav::controller::MavFlightController;
use flyingsquirrel::mav::monitor::expected_autopilot_byte;
use flyingsquirrel::mav::monitor::VehicleProfile;
use flyingsquirrel::mav::source::{MavGpsSource, MavImuSource};
use flyingsquirrel::mav::{MavFilter, MavLink};
use flyingsquirrel::nav::NavConfig;
use flyingsquirrel::runtime::{spawn_all, MavPreflight};
use flyingsquirrel::sim::spoof::{SpoofInjector, SpoofPattern};
use flyingsquirrel::sim::trajectory::LinearNorth;
use flyingsquirrel::sim::{SyntheticGps, SyntheticImu};
use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::mpsc;
use tracing_subscriber::EnvFilter;

/// Default seed (matches AppConfig::Scenario default). High bit cleared so
/// the value fits in TOML's i64 range — keep these in sync.
const DEFAULT_SEED: u64 = 0x7F1F_5184_0000_0000;

/// Returns true if the CLI argument named `name` was set explicitly on the
/// command line (as opposed to taking its `default_value_t` or being unset).
/// Used to merge config-file values with CLI overrides: explicit CLI wins.
fn cli_explicit(matches: &clap::ArgMatches, name: &str) -> bool {
    matches.value_source(name) == Some(ValueSource::CommandLine)
}

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq)]
enum GpsSourceKind {
    /// Synthetic GPS with optional spoof injection.
    Synth,
    /// MAVLink GPS_RAW_INT from a connected peer.
    Mav,
    /// NMEA over serial (real hardware).
    Serial,
}

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq)]
enum ImuSourceKind {
    /// Synthetic IMU from the same ground-truth trajectory as Synth GPS.
    Synth,
    /// MAVLink HIGHRES_IMU from a connected peer.
    Mav,
    /// I2C MPU-6050-class IMU (Linux only, requires `hw-i2c` feature).
    I2c,
}

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq)]
enum ControllerKind {
    /// Console + JSON-Lines log.
    Console,
    /// MAVLink RTL + PARAM_SET to the autopilot.
    Mav,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum Scenario {
    Clean,
    SuddenJump,
    GradualDrift,
}

#[derive(Debug, Parser)]
#[command(
    name = "flyingsquirrel",
    version,
    about = "GPS/IMU drift cross-correlator: anti-spoofing detector."
)]
struct Cli {
    /// Load defaults from a TOML config file. CLI flags explicitly set on
    /// the command line still override the config file's values. Generate a
    /// starter config by running with `--print-config` (no other args).
    #[arg(long, value_name = "PATH")]
    config: Option<PathBuf>,
    /// Emit the resolved (defaults + config + CLI) configuration as TOML to
    /// stdout and exit. Useful for capturing a known-good runtime config in
    /// version control: `flyingsquirrel --config base.toml ... --print-config > pinned.toml`.
    #[arg(long, default_value_t = false)]
    print_config: bool,

    #[arg(long, value_enum, default_value_t = GpsSourceKind::Synth)]
    gps_source: GpsSourceKind,
    #[arg(long, value_enum, default_value_t = ImuSourceKind::Synth)]
    imu_source: ImuSourceKind,
    #[arg(long, value_enum, default_value_t = ControllerKind::Console)]
    controller: ControllerKind,

    // --- synth options ---
    #[arg(long, value_enum, default_value_t = Scenario::GradualDrift)]
    scenario: Scenario,
    #[arg(long, default_value_t = DEFAULT_SEED)]
    seed: u64,

    // --- serial GPS options ---
    #[arg(long, default_value = "/dev/ttyUSB0")]
    gps_port: String,
    #[arg(long, default_value_t = 9600)]
    gps_baud: u32,

    // --- I2C IMU options ---
    #[arg(long, default_value = "/dev/i2c-1")]
    imu_bus: String,
    #[arg(long, default_value_t = 0x68)]
    imu_addr: u8,
    /// Nominal IMU sample rate in Hz. Drives the I2C driver poll rate AND sizes
    /// the dead-reckoning integration-gap gate for every source. For a MAVLink
    /// deployment, set this to the autopilot's raw-sensor stream rate (ArduPilot
    /// SR2_RAW_SENS / PX4 equivalent) — a value far above the real stream rate
    /// makes the gate skip every integration step and silently disables the
    /// GPS cross-check.
    #[arg(long, default_value_t = 100.0)]
    imu_rate: f32,
    /// Chip-to-body axis remap for an I2C IMU mounted in a non-standard
    /// orientation. Three comma-separated signed axes naming the CHIP axis that
    /// feeds each BODY axis (FRD: X-forward, Y-right, Z-down), e.g. "Y,-X,Z"
    /// for a 90° yaw mount. Default "X,Y,Z" = chip mounted X-forward/Y-right/
    /// Z-down. Must be a proper rotation (a reflection is rejected). Applies to
    /// --imu-source i2c only.
    #[arg(long, default_value = "X,Y,Z")]
    imu_axis_map: String,

    // --- MAVLink options (used when any --*-source = mav OR --controller mav) ---
    /// Local UDP bind for MAVLink. Defaults to loopback for safety; only use
    /// `0.0.0.0:14551` for cross-host setups and pair with `--mav-no-source-filter`
    /// only when the network is otherwise trusted.
    #[arg(long, default_value = "127.0.0.1:14551")]
    mav_bind: SocketAddr,
    #[arg(long, default_value = "127.0.0.1:14550")]
    mav_target: SocketAddr,
    #[arg(long, default_value_t = 1)]
    mav_target_system: u8,
    #[arg(long, default_value_t = 1)]
    mav_target_component: u8,
    /// Disable the inbound source-IP filter. NOT RECOMMENDED — leaves the
    /// UDP listener accepting GPS/IMU from any host on the network.
    #[arg(long, default_value_t = false)]
    mav_no_source_filter: bool,
    /// Disable the inbound system_id filter. NOT RECOMMENDED.
    #[arg(long, default_value_t = false)]
    mav_no_sysid_filter: bool,
    /// Lock the MAVLink source on IP alone, accepting the autopilot's first
    /// message from ANY source port. Needed for bridges that forward from an
    /// ephemeral port (mavproxy `--out udp:`, mavlink-router, SiK), where the
    /// strict source-port check would otherwise silently reject every packet.
    /// Weakens the co-located-attacker bootstrap defense — CLI-only, cannot be
    /// enabled via TOML.
    #[arg(long, default_value_t = false)]
    mav_allow_any_source_port: bool,
    /// Allow the `synth` GPS or IMU source to be paired with the `mav`
    /// controller. By default this combination is REFUSED because a
    /// simulated spoof scenario would fire real PARAM_SET + RTL at a real
    /// autopilot. Only set this for hardware-in-the-loop testing where the
    /// MAV endpoint points at a SITL — never against a flying drone.
    #[arg(long, default_value_t = false)]
    allow_synth_to_mav: bool,

    /// Allow MAV deployments without `--expected-home`. NOT RECOMMENDED for
    /// production — disables the meaconing-at-boot cross-check that would
    /// otherwise reject a first GPS fix far from where the drone is supposed
    /// to be. Use only for ad-hoc field tests where the launch site changes
    /// between runs.
    #[arg(long, default_value_t = false)]
    allow_no_boot_anchor_check: bool,

    /// Directory for forensic dumps. When set, on the FIRST `Spoofed`
    /// transition of this process lifetime, a `spoof-<ts>-<pid>.json` file
    /// is written containing the last N seconds of GPS/IMU/residual state
    /// plus the trigger event and binary attestation. Without this, only
    /// the in-process JSON event log captures the incident — operators
    /// can't reconstruct what the detector was SEEING.
    #[arg(long, value_name = "DIR")]
    forensic_dir: Option<PathBuf>,
    /// Forensic ring buffer window in seconds. Default 30s.
    #[arg(long, default_value_t = 30.0)]
    forensic_window_s: f64,

    /// Autopilot/vehicle family. Required when `--controller mav`. v1 ships
    /// `ardu-copter` only — PX4 / Plane / Rover support is deferred to a
    /// future phase (each has non-trivial mode-ID, encoding, or RTL-message
    /// differences that need their own verification). The detector cross-
    /// checks this against the autopilot's MAV_AUTOPILOT byte at first
    /// HEARTBEAT and refuses to leave Preflight on mismatch.
    #[arg(long, value_parser = clap::value_parser!(VehicleProfile))]
    vehicle: Option<VehicleProfile>,

    /// Operator-supplied expected home coordinates `lat,lon` (decimal
    /// degrees). When set, the FIRST GPS fix must be within
    /// `--max-home-distance-m` of this point or the detector emits
    /// `BootAnchorRejected` and refuses to anchor — defends against
    /// meaconing-at-boot (attacker already broadcasting a spoofed location
    /// when the drone powers up).
    // allow_hyphen_values: a southern/western launch site has a negative
    // coordinate (e.g. `--expected-home -35.36,149.16`); without this clap reads
    // the leading `-3...` as an unknown flag ("unexpected argument '-3'"). This
    // bit the SITL harness against ArduPilot's Canberra default home.
    #[arg(long, allow_hyphen_values = true)]
    expected_home: Option<String>,
    #[arg(long, default_value_t = 1000.0)]
    max_home_distance_m: f64,

    // --- common ---
    #[arg(long, default_value_t = 60)]
    duration: u32,
    #[arg(long)]
    json_log: Option<PathBuf>,
    #[arg(long, default_value = "info")]
    log_level: String,
}

#[tokio::main]
async fn main() {
    // Print operator-facing errors as their Display form (a clean one-line
    // message) and exit non-zero, instead of letting `Result`-returning main
    // emit the Debug-wrapped `Error: Io(Custom { .. })` struct that buried our
    // carefully-worded CLI/config guards. Audit M5.
    if let Err(e) = run().await {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

async fn run() -> Result<(), FsError> {
    let mut cli = Cli::parse();
    // Re-parse to get ArgMatches so we can detect explicit-on-CLI vs default.
    // This is cheap and isolates the value-source query to a fresh parse —
    // avoids tangling clap-derive parsing with our merge logic.
    let matches = Cli::command().get_matches();

    // Load TOML config (or defaults if --config absent). Then merge:
    // CLI value wins if explicitly set on the command line; otherwise the
    // config-file value fills in.
    let file_cfg = AppConfig::load(cli.config.as_deref())?;
    merge_config_into_cli(&mut cli, &matches, &file_cfg)?;

    // Was a run duration explicitly chosen (CLI flag OR a [process].duration in
    // the TOML)? If neither set it, `cli.duration` is the clap default (60),
    // which is a *development scenario* length — fine for synth, dangerous for a
    // real deployment (it would exit cleanly after ~60s and systemd's
    // Restart=on-failure would NOT restart it, leaving the drone undefended).
    // The footgun guard below uses this to refuse a defaulted real deployment.
    let duration_explicit =
        cli_explicit(&matches, "duration") || file_cfg.process.duration.is_some();

    // --print-config emits the resolved config and exits BEFORE we do any
    // network / sensor wiring. Lets operators capture their effective
    // runtime state without side effects.
    if cli.print_config {
        let resolved = cli_to_app_config(&cli);
        let toml = resolved.to_toml()?;
        print!("{}", toml);
        return Ok(());
    }

    let filter = EnvFilter::try_from_default_env()
        .or_else(|_| EnvFilter::try_new(&cli.log_level))
        .unwrap_or_else(|_| EnvFilter::new("info"));
    init_tracing(filter);

    // Attestation: emit one log line + JSON-log entry with the binary's
    // SHA-256, version, host details. First thing after tracing init so
    // operators always see it. Provenance only — see attestation.rs.
    let att = Attestation::capture();
    att.emit(cli.json_log.as_ref()).await;

    tracing::info!(
        gps = ?cli.gps_source,
        imu = ?cli.imu_source,
        controller = ?cli.controller,
        "starting"
    );

    // Safety guard: synthetic source + real MAV controller = friendly-fire RTL
    // on a simulated attack. Refuse unless operator explicitly acknowledges
    // (only meaningful for HIL testing against SITL).
    let synth_source = matches!(cli.gps_source, GpsSourceKind::Synth)
        || matches!(cli.imu_source, ImuSourceKind::Synth);
    let mav_controller = matches!(cli.controller, ControllerKind::Mav);
    if synth_source && mav_controller && !cli.allow_synth_to_mav {
        return Err(FsError::Config(
            "refusing to pair a synthetic GPS/IMU source with the MAV controller: \
             a simulated spoof scenario would send real PARAM_SET + RTL to a real \
             autopilot. If this is intentional (e.g. HIL testing against SITL), pass \
             --allow-synth-to-mav."
                .to_string(),
        ));
    }

    // Duration footgun guard. Synthetic/replay sources end their own stream, so
    // a defaulted duration is harmless there. But a real deployment (any non-synth
    // GPS/IMU source, i.e. serial/i2c/mav) runs an infinite ingest stream — the
    // ONLY things that stop it are the duration timer or an operator signal. The
    // 60s clap default is a development scenario length; if it silently applied to
    // a real flight, the daemon would log "scenario complete" and exit 0 after
    // ~60s, and systemd's `Restart=on-failure` would NOT restart a clean exit —
    // the drone would fly the rest of the mission with no spoof detection and no
    // alarm. Refuse to start unless the operator explicitly chose a duration.
    // `duration = 0` is the explicit "run until stopped by signal" sentinel.
    if !synth_source && !duration_explicit {
        return Err(FsError::Config(
            "real deployment requires an explicit run duration: set [process].duration \
             (seconds — e.g. 7200 for a 2-hour mission cap) or pass --duration. To run \
             until stopped by signal (Ctrl-C / SIGTERM / SIGHUP), use duration = 0. \
             Refusing to start on the 60s default: it is a development scenario length \
             and would silently exit mid-flight, leaving the drone undefended (systemd \
             Restart=on-failure does not restart a clean exit)."
                .to_string(),
        ));
    }

    // Wire up the MAVLink connection lazily if any source/sink needs it.
    let needs_mav = cli.gps_source == GpsSourceKind::Mav
        || cli.imu_source == ImuSourceKind::Mav
        || cli.controller == ControllerKind::Mav;

    // Vehicle profile is REQUIRED when ANY mav path is in use — not just the
    // controller. The autopilot-byte cross-check at first HEARTBEAT depends
    // on a declared expected value, and we want that defense whether the
    // operator is reading IMU/GPS from MAV (with a console controller) or
    // commanding via MAV. Default-fail: refuse to launch without it.
    let vehicle_profile = match (needs_mav, cli.vehicle) {
        (true, Some(v)) => Some(v),
        (true, None) => {
            return Err(FsError::Config(
                "--vehicle is required when any MAV source or controller is in use. \
                 Supported: ardu-copter, px4. (Plane / Rover are deferred to a future \
                 phase.) This guard enables the autopilot-byte cross-check that defends \
                 against firmware swaps and operator typos."
                    .to_string(),
            ));
        }
        (false, _) => None, // no MAV path: vehicle profile irrelevant
    };

    // Boot-anchor cross-check is a core defense against meaconing-at-boot.
    // When MAV is in use (production deployment), refuse to launch without
    // either an `--expected-home` declaration or an explicit opt-out flag.
    // Without this guard, the boot-anchor check is silently no-op and an
    // attacker broadcasting a spoofed position when the drone powers up wins.
    if needs_mav && cli.expected_home.is_none() && !cli.allow_no_boot_anchor_check {
        return Err(FsError::Config(
            "production MAV deployment requires --expected-home <lat,lon> for the \
             boot-anchor cross-check (meaconing-at-boot defense). To explicitly opt \
             out (NOT RECOMMENDED — e.g. for ad-hoc field tests where the launch \
             site changes between runs), pass --allow-no-boot-anchor-check."
                .to_string(),
        ));
    }

    // Source-IP / sysid filter opt-outs are real footguns. Emit a loud
    // warning at startup so the choice is visible in the log.
    if cli.mav_no_source_filter {
        tracing::warn!(
            "--mav-no-source-filter is SET — UDP listener accepts MAVLink from ANY \
             source IP. NOT RECOMMENDED outside a tightly-controlled lab network."
        );
    }
    if cli.mav_no_sysid_filter {
        tracing::warn!(
            "--mav-no-sysid-filter is SET — MAVLink messages from any system_id are \
             accepted. A hostile autopilot on the same link can inject GPS/IMU. NOT \
             RECOMMENDED."
        );
    }
    if cli.mav_allow_any_source_port {
        tracing::warn!(
            "--mav-allow-any-source-port is SET — the MAVLink source-lock pins on IP \
             alone, accepting the first peer from any source port. Use only when a \
             bridge forwards from an ephemeral port; it weakens the co-located-attacker \
             bootstrap defense."
        );
    }

    let mav_link = if needs_mav {
        let filter = MavFilter {
            allowed_source_ip: if cli.mav_no_source_filter {
                None
            } else {
                Some(cli.mav_target.ip())
            },
            autopilot_sysid: if cli.mav_no_sysid_filter {
                None
            } else {
                Some(cli.mav_target_system)
            },
            allow_any_source_port: cli.mav_allow_any_source_port,
        };
        tracing::info!(
            bind = %cli.mav_bind,
            target = %cli.mav_target,
            filter = ?filter,
            "binding MAVLink"
        );
        Some(Arc::new(
            MavLink::bind_with_filter(cli.mav_bind, cli.mav_target, filter).await?,
        ))
    } else {
        None
    };

    // The MAVLink listener is shared by all MAV-backed sources, but we own the
    // gps_rx and imu_rx channels — each MAV source consumes one. If the user
    // mixes (e.g. mav GPS + synth IMU), the imu_rx is dropped.
    let mav_listener_handle = mav_link.as_ref().map(|link| {
        let (handle, gps_rx, imu_rx, _other_rx) = link.spawn_listener();
        (handle, gps_rx, imu_rx)
    });

    let (mut mav_listener_join, mav_gps_rx, mav_imu_rx) = match mav_listener_handle {
        Some((h, g, i)) => (Some(h), Some(g), Some(i)),
        None => (None, None, None),
    };

    // --- Build GPS source ---
    let gps: Box<dyn GpsSource> = match cli.gps_source {
        GpsSourceKind::Synth => Box::new(build_synth_gps(&cli)),
        GpsSourceKind::Mav => Box::new(MavGpsSource {
            rx: mav_gps_rx.expect("mav listener absent for mav gps"),
        }),
        GpsSourceKind::Serial => Box::new(SerialGpsSource::new(SerialGpsConfig {
            port: cli.gps_port.clone(),
            baud: cli.gps_baud,
        })),
    };

    // --- Build IMU source ---
    let imu: Box<dyn ImuSource> = match cli.imu_source {
        ImuSourceKind::Synth => Box::new(build_synth_imu(&cli)),
        ImuSourceKind::Mav => Box::new(MavImuSource {
            rx: mav_imu_rx.expect("mav listener absent for mav imu"),
        }),
        ImuSourceKind::I2c => build_i2c_imu(&cli)?,
    };

    // --- Build controller ---
    let controller: Arc<dyn FlightController> = match cli.controller {
        ControllerKind::Console => Arc::new(ConsoleController::new(cli.json_log.clone()).await?),
        ControllerKind::Mav => {
            let link = mav_link
                .clone()
                .expect("mav link absent for mav controller");
            let profile = vehicle_profile
                .expect("vehicle profile required for mav controller (CLI guard above)");
            Arc::new(
                MavFlightController::new(
                    link,
                    cli.json_log.clone(),
                    cli.mav_target_system,
                    cli.mav_target_component,
                    profile,
                )
                .await?,
            )
        }
    };

    // Parse expected_home if provided, building the NavConfig override.
    let mut nav = NavConfig::default();
    // Size the IMU integration-gap gate to the configured IMU rate. A fixed
    // 100 ms bound silently disables dead-reckoning on the 4–10 Hz streams a real
    // ArduPilot/PX4 link produces (the detector then fails open). Allow ~2.5
    // nominal sample periods of jitter, but never tighter than the default. For
    // a MAV deployment, set --imu-rate to the autopilot's raw-sensor stream rate
    // (ArduPilot SR2_RAW_SENS / PX4 equivalent).
    let nominal_imu_period_s: f32 = if cli.imu_rate > 0.0 {
        1.0 / cli.imu_rate
    } else {
        0.0
    };
    nav.max_imu_dt_s = (2.5_f32 * nominal_imu_period_s).max(NavConfig::default().max_imu_dt_s);
    tracing::info!(
        imu_rate_hz = cli.imu_rate,
        max_imu_dt_ms = nav.max_imu_dt_s * 1000.0,
        "IMU integration-gap gate sized from --imu-rate"
    );
    if let Some(h) = &cli.expected_home {
        let mut parts = h.split(',');
        let lat = parts.next().and_then(|s| s.trim().parse::<f64>().ok());
        let lon = parts.next().and_then(|s| s.trim().parse::<f64>().ok());
        match (lat, lon) {
            (Some(lat), Some(lon)) if lat.abs() <= 90.0 && lon.abs() <= 180.0 => {
                nav.expected_home = Some((lat, lon));
                nav.max_home_distance_m = cli.max_home_distance_m;
                // AUDIT F-04 fix: previously this logged `lat` and `lon` at
                // INFO level. The operator's launch coordinates are
                // operationally sensitive (precise launch site is a
                // mission-defining detail for adversarial deployments) and
                // INFO-level logs are forwarded by journald to any
                // centralized logging sink. Log only the radius at INFO; the
                // actual coordinates are available at DEBUG for diagnostic
                // sessions where the operator has explicitly opted in.
                tracing::info!(
                    radius_m = cli.max_home_distance_m,
                    home_configured = true,
                    "boot-anchor cross-check ARMED"
                );
                tracing::debug!(lat, lon, "boot-anchor home coordinates");
            }
            _ => {
                return Err(FsError::Config(format!(
                    "--expected-home must be 'lat,lon' decimal degrees, got: {h}"
                )));
            }
        }
    }
    // If MAV is in use (any source or controller), thread the monitor through
    // to preflight so it can require a HEARTBEAT and validate the autopilot
    // byte before arming the detector.
    let mav_preflight = match (mav_link.as_ref(), vehicle_profile) {
        (Some(link), Some(profile)) => Some(MavPreflight {
            monitor: link.monitor.clone(),
            expected_autopilot: expected_autopilot_byte(profile),
        }),
        _ => None,
    };
    let forensic_cfg = cli
        .forensic_dir
        .as_ref()
        .map(|p| flyingsquirrel::runtime::ForensicCfg {
            dir: p.clone(),
            window_s: cli.forensic_window_s,
        });
    if let Some(fc) = &forensic_cfg {
        tracing::info!(dir = %fc.dir.display(), window_s = fc.window_s,
            "forensic capture ARMED");
    }
    let mut handles = spawn_all(
        gps,
        imu,
        controller,
        FusionConfig {
            nav,
            ..FusionConfig::default()
        },
        mav_preflight,
        forensic_cfg,
    )?;

    // Operator reset signal: SIGHUP on Unix, Ctrl-Break on Windows. Sends a
    // unit to the fusion reset channel; fusion clears the SPOOFED latch and
    // resumes detection from NORMAL.
    spawn_reset_signal_handler(handles.reset_tx.clone());

    // If MAVLink is in use, spawn the heartbeat watchdog so LinkDown /
    // LinkRestored events join the same broadcast stream as spoof events.
    let mut watchdog_handle: Option<tokio::task::JoinHandle<()>> = None;
    if let Some(link) = mav_link.as_ref() {
        watchdog_handle = Some(link.spawn_heartbeat_watchdog(
            handles.events_tx.clone(),
            handles.boot,
            Duration::from_secs(3),     // staleness threshold
            Duration::from_millis(500), // poll interval
        ));
    }

    // `duration = 0` ⇒ run until stopped by a signal (no time limit); the
    // supervisor's timer arm becomes a never-completing future. Otherwise the
    // +2s headroom lets a synthetic scenario fully play out past its nominal end.
    let duration_limit: Option<Duration> =
        (cli.duration > 0).then(|| Duration::from_secs(cli.duration as u64 + 2));

    // Task supervisor: distinguish three exit modes:
    //   (a) ctrl-c / duration  — orderly operator-driven shutdown
    //   (b) source exhaustion   — synthetic scenarios end cleanly; this is
    //                              NORMAL when --gps-source synth or replay
    //   (c) task panic / crash  — actual failure, fail loud, exit non-zero
    //
    // (b) only applies to ingest tasks (fusion/action are infinite loops
    // until their input channels close). When ingest returns Ok(()) the
    // stream simply ended; we let fusion drain naturally afterward.
    let synth_source = matches!(cli.gps_source, GpsSourceKind::Synth)
        || matches!(cli.imu_source, ImuSourceKind::Synth);
    // Audit E-11 fix: also intercept SIGTERM (used by systemd for graceful
    // stop). Without this arm, systemctl stop would let SIGTERM kill the
    // process directly without draining in-flight forensic writes. On
    // Windows, ctrl_c handles both Ctrl-C and Ctrl-Break already.
    let exit_reason: &'static str = {
        #[cfg(unix)]
        {
            use tokio::signal::unix::{signal, SignalKind};
            let mut sigterm = signal(SignalKind::terminate()).map_err(FsError::Io)?;
            tokio::select! {
                _ = tokio::signal::ctrl_c() => {
                    tracing::info!("ctrl-c received, shutting down");
                    "ctrl_c"
                }
                _ = sigterm.recv() => {
                    tracing::info!("SIGTERM received (systemd graceful stop), shutting down");
                    "sigterm"
                }
                _ = async {
                    match duration_limit {
                        Some(d) => tokio::time::sleep(d).await,
                        None => std::future::pending::<()>().await,
                    }
                } => {
                    tracing::info!("run duration limit reached");
                    "duration_elapsed"
                }
                r = &mut handles.gps_ingest => {
                    classify_ingest_exit("gps_ingest", r, synth_source)
                }
                r = &mut handles.imu_ingest => {
                    classify_ingest_exit("imu_ingest", r, synth_source)
                }
                r = &mut handles.fusion => {
                    tracing::error!(result = ?r, "fusion task exited unexpectedly");
                    "task_died:fusion"
                }
                r = &mut handles.action => {
                    tracing::error!(result = ?r, "action task exited unexpectedly");
                    "task_died:action"
                }
            }
        }
        #[cfg(not(unix))]
        {
            tokio::select! {
                _ = tokio::signal::ctrl_c() => {
                    tracing::info!("ctrl-c received, shutting down");
                    "ctrl_c"
                }
                _ = async {
                    match duration_limit {
                        Some(d) => tokio::time::sleep(d).await,
                        None => std::future::pending::<()>().await,
                    }
                } => {
                    tracing::info!("run duration limit reached");
                    "duration_elapsed"
                }
                r = &mut handles.gps_ingest => {
                    classify_ingest_exit("gps_ingest", r, synth_source)
                }
                r = &mut handles.imu_ingest => {
                    classify_ingest_exit("imu_ingest", r, synth_source)
                }
                r = &mut handles.fusion => {
                    tracing::error!(result = ?r, "fusion task exited unexpectedly");
                    "task_died:fusion"
                }
                r = &mut handles.action => {
                    tracing::error!(result = ?r, "action task exited unexpectedly");
                    "task_died:action"
                }
            }
        }
    };

    handles.gps_ingest.abort();
    handles.imu_ingest.abort();
    // The select above borrows `handles.fusion` mutably; if the *fusion* arm is
    // what fired (task_died:fusion), that JoinHandle has already been polled to
    // completion. Awaiting it again here panics with "JoinHandle polled after
    // completion" — which would abort main BEFORE the forensic drain below runs,
    // losing the incident record at exactly the moment fusion crashed mid-spoof.
    // Only await when it's still pending (every non-fusion exit reason).
    if !handles.fusion.is_finished() {
        let _ = handles.fusion.await;
    }
    // AUDIT (shutdown event-trail loss): do NOT abort the action task here.
    // It is the ONLY broadcast subscriber that persists events to the JSON log
    // (ConsoleController::on_event), and the background tasks drained below
    // (forensic dump, RTL verify, sever read-back) emit their outcome events —
    // including the CRITICAL sever-readback-unconfirmed — DURING the drain. The
    // old order aborted the logger first, so those events were silently dropped
    // even though the drain preserved the forensic dump file itself: the event
    // trail was lost on the exact shutdown-during-spoof path the drain exists to
    // protect. The action task is stopped AFTER the drain (below).

    // Audit E-10 fix: drain in-flight background tasks (forensic dump,
    // RTL verify) with a bounded timeout. 7s = 5s verify window + 2s
    // forensic-write headroom. Tasks still running past the deadline are
    // aborted to release resources promptly; the partial loss is logged.
    let (drained, aborted) = flyingsquirrel::runtime::drain_pending_bg(
        handles.pending_bg.clone(),
        Duration::from_secs(7),
    )
    .await;
    if aborted > 0 {
        tracing::warn!(
            drained,
            aborted,
            "{} background task(s) aborted past drain deadline (forensic dump or RTL \
             verification may be partial)",
            aborted
        );
    } else if drained > 0 {
        tracing::info!(drained, "drained pending background tasks cleanly");
    }
    // Stop the remaining event PRODUCERS (heartbeat watchdog, MAV listener),
    // awaiting their teardown so their broadcast-sender clones are dropped.
    if let Some(h) = mav_listener_join.take() {
        h.abort();
        let _ = h.await;
    }
    if let Some(h) = watchdog_handle.take() {
        h.abort();
        let _ = h.await;
    }
    // With fusion (awaited above), the drained background tasks, and the
    // producers all gone, drop main's own sender clone: the broadcast channel
    // now has no senders, so the action subscriber returns Closed as soon as it
    // has flushed the events buffered during the drain, then exits. Bounded so a
    // stray sender can never hang process exit; abort is the safety net. The
    // is_finished() guard avoids re-polling a handle the select already drove to
    // completion (the task_died:action exit reason).
    drop(handles.events_tx);
    if !handles.action.is_finished()
        && tokio::time::timeout(Duration::from_secs(2), &mut handles.action)
            .await
            .is_err()
    {
        handles.action.abort();
    }
    // Hold the `mav_link` Arc in scope until shutdown so tasks holding clones
    // (listener, watchdog, controller) see consistent lifetime ordering, then
    // drop it explicitly here as the last step.
    drop(mav_link);

    if exit_reason.starts_with("task_died:") {
        // Surface to the operator unambiguously and exit non-zero so a watchdog
        // / systemd can restart us.
        tracing::error!(reason = exit_reason, "FlyingSquirrel exiting with FAILURE");
        std::process::exit(2);
    }
    tracing::info!(reason = exit_reason, "FlyingSquirrel exiting normally");
    Ok(())
}

/// Merge `cfg` (from --config file) into `cli`: for each field that wasn't
/// explicitly set on the command line, replace `cli`'s default with the
/// config file's value. Strings are parsed back into their CLI types where
/// applicable. Unknown enum values produce a clear error.
fn merge_config_into_cli(
    cli: &mut Cli,
    matches: &clap::ArgMatches,
    cfg: &AppConfig,
) -> Result<(), FsError> {
    // Sources
    if !cli_explicit(matches, "gps_source") {
        cli.gps_source = parse_gps_source(&cfg.sources.gps_source)?;
    }
    if !cli_explicit(matches, "imu_source") {
        cli.imu_source = parse_imu_source(&cfg.sources.imu_source)?;
    }
    if !cli_explicit(matches, "controller") {
        cli.controller = parse_controller(&cfg.sources.controller)?;
    }
    // Scenario
    if !cli_explicit(matches, "scenario") {
        cli.scenario = parse_scenario(&cfg.scenario.scenario)?;
    }
    if !cli_explicit(matches, "seed") {
        cli.seed = cfg.scenario.seed;
    }
    // Serial
    if !cli_explicit(matches, "gps_port") {
        cli.gps_port = cfg.serial.gps_port.clone();
    }
    if !cli_explicit(matches, "gps_baud") {
        cli.gps_baud = cfg.serial.gps_baud;
    }
    // I2C
    if !cli_explicit(matches, "imu_bus") {
        cli.imu_bus = cfg.i2c.imu_bus.clone();
    }
    if !cli_explicit(matches, "imu_addr") {
        cli.imu_addr = cfg.i2c.imu_addr;
    }
    if !cli_explicit(matches, "imu_rate") {
        cli.imu_rate = cfg.i2c.imu_rate;
    }
    if !cli_explicit(matches, "imu_axis_map") {
        cli.imu_axis_map = cfg.i2c.axis_map.clone();
    }
    // MAV
    if !cli_explicit(matches, "mav_bind") {
        cli.mav_bind = cfg
            .mav
            .bind
            .parse()
            .map_err(|e| FsError::Config(format!("config mav.bind: {}", e)))?;
    }
    if !cli_explicit(matches, "mav_target") {
        cli.mav_target = cfg
            .mav
            .target
            .parse()
            .map_err(|e| FsError::Config(format!("config mav.target: {}", e)))?;
    }
    if !cli_explicit(matches, "mav_target_system") {
        cli.mav_target_system = cfg.mav.target_system;
    }
    if !cli_explicit(matches, "mav_target_component") {
        cli.mav_target_component = cfg.mav.target_component;
    }
    if !cli_explicit(matches, "mav_no_source_filter") {
        cli.mav_no_source_filter = cfg.mav.no_source_filter;
    }
    if !cli_explicit(matches, "mav_no_sysid_filter") {
        cli.mav_no_sysid_filter = cfg.mav.no_sysid_filter;
    }
    // Safety-bypass: TOML can only ever carry `false` here (load rejects
    // `true`), so this merge can never silently enable it — it must be set via
    // the explicit CLI flag.
    if !cli_explicit(matches, "mav_allow_any_source_port") {
        cli.mav_allow_any_source_port = cfg.mav.allow_any_source_port;
    }
    if !cli_explicit(matches, "vehicle") && cli.vehicle.is_none() {
        if let Some(v) = &cfg.mav.vehicle {
            cli.vehicle = Some(v.parse().map_err(FsError::Config)?);
        }
    }
    // Boot-anchor
    if !cli_explicit(matches, "expected_home") && cli.expected_home.is_none() {
        cli.expected_home = cfg.boot_anchor.expected_home.clone();
    }
    if !cli_explicit(matches, "max_home_distance_m") {
        if let Some(v) = cfg.boot_anchor.max_home_distance_m {
            cli.max_home_distance_m = v;
        }
    }
    if !cli_explicit(matches, "allow_no_boot_anchor_check") {
        cli.allow_no_boot_anchor_check = cfg.boot_anchor.allow_no_boot_anchor_check;
    }
    if !cli_explicit(matches, "forensic_dir") && cli.forensic_dir.is_none() {
        cli.forensic_dir = cfg.forensic.dir.as_ref().map(PathBuf::from);
    }
    if !cli_explicit(matches, "forensic_window_s") {
        if let Some(w) = cfg.forensic.window_s {
            cli.forensic_window_s = w;
        }
    }
    // Process
    if !cli_explicit(matches, "duration") {
        // Only override from TOML when the file actually set a duration. A
        // missing [process].duration leaves `cli.duration` at its clap default
        // (60) but the caller separately tracks that it was never explicitly
        // chosen (see `duration_explicit` in main) so a real deployment is
        // refused rather than silently running 60s.
        if let Some(d) = cfg.process.duration {
            cli.duration = d;
        }
    }
    if !cli_explicit(matches, "json_log") && cli.json_log.is_none() {
        cli.json_log = cfg.process.json_log.as_ref().map(PathBuf::from);
    }
    if !cli_explicit(matches, "log_level") {
        cli.log_level = cfg.process.log_level.clone();
    }
    if !cli_explicit(matches, "allow_synth_to_mav") {
        cli.allow_synth_to_mav = cfg.process.allow_synth_to_mav;
    }
    Ok(())
}

fn parse_gps_source(s: &str) -> Result<GpsSourceKind, FsError> {
    match s {
        "synth" => Ok(GpsSourceKind::Synth),
        "mav" => Ok(GpsSourceKind::Mav),
        "serial" => Ok(GpsSourceKind::Serial),
        other => Err(FsError::Config(format!(
            "unknown gps_source {:?}; valid: synth|mav|serial",
            other
        ))),
    }
}

fn parse_imu_source(s: &str) -> Result<ImuSourceKind, FsError> {
    match s {
        "synth" => Ok(ImuSourceKind::Synth),
        "mav" => Ok(ImuSourceKind::Mav),
        "i2c" => Ok(ImuSourceKind::I2c),
        other => Err(FsError::Config(format!(
            "unknown imu_source {:?}; valid: synth|mav|i2c",
            other
        ))),
    }
}

fn parse_controller(s: &str) -> Result<ControllerKind, FsError> {
    match s {
        "console" => Ok(ControllerKind::Console),
        "mav" => Ok(ControllerKind::Mav),
        other => Err(FsError::Config(format!(
            "unknown controller {:?}; valid: console|mav",
            other
        ))),
    }
}

fn parse_scenario(s: &str) -> Result<Scenario, FsError> {
    match s {
        "clean" => Ok(Scenario::Clean),
        "sudden-jump" | "sudden_jump" => Ok(Scenario::SuddenJump),
        "gradual-drift" | "gradual_drift" => Ok(Scenario::GradualDrift),
        other => Err(FsError::Config(format!(
            "unknown scenario {:?}; valid: clean|sudden-jump|gradual-drift",
            other
        ))),
    }
}

/// Convert the resolved Cli back into an AppConfig so `--print-config` can
/// emit it as TOML. This is purely for observability — the binary's runtime
/// state is driven by `cli`, not this round-trip value.
fn cli_to_app_config(cli: &Cli) -> AppConfig {
    use flyingsquirrel::config::*;
    AppConfig {
        sources: Sources {
            gps_source: format!("{:?}", cli.gps_source).to_lowercase(),
            imu_source: format!("{:?}", cli.imu_source).to_lowercase(),
            controller: format!("{:?}", cli.controller).to_lowercase(),
        },
        scenario: Scenario_ {
            scenario: format!("{:?}", cli.scenario)
                .to_lowercase()
                .replace("suddenjump", "sudden-jump")
                .replace("gradualdrift", "gradual-drift"),
            seed: cli.seed,
        },
        serial: Serial {
            gps_port: cli.gps_port.clone(),
            gps_baud: cli.gps_baud,
        },
        i2c: I2c {
            imu_bus: cli.imu_bus.clone(),
            imu_addr: cli.imu_addr,
            imu_rate: cli.imu_rate,
            axis_map: cli.imu_axis_map.clone(),
        },
        mav: Mav {
            bind: cli.mav_bind.to_string(),
            target: cli.mav_target.to_string(),
            target_system: cli.mav_target_system,
            target_component: cli.mav_target_component,
            no_source_filter: cli.mav_no_source_filter,
            no_sysid_filter: cli.mav_no_sysid_filter,
            allow_any_source_port: cli.mav_allow_any_source_port,
            vehicle: cli.vehicle.map(|v| v.to_string()),
        },
        boot_anchor: BootAnchor {
            expected_home: cli.expected_home.clone(),
            max_home_distance_m: Some(cli.max_home_distance_m),
            allow_no_boot_anchor_check: cli.allow_no_boot_anchor_check,
        },
        forensic: Forensic {
            dir: cli.forensic_dir.as_ref().map(|p| p.display().to_string()),
            window_s: Some(cli.forensic_window_s),
        },
        process: Process {
            duration: Some(cli.duration),
            json_log: cli.json_log.as_ref().map(|p| p.display().to_string()),
            log_level: cli.log_level.clone(),
            allow_synth_to_mav: cli.allow_synth_to_mav,
        },
    }
}

/// Initialize the `tracing` subscriber. Always streams to stderr (matches
/// systemd's `StandardError=journal` capture). On Linux with the `journald`
/// feature enabled, ALSO writes to systemd's journal directly, which gives
/// us structured fields (every span/event field becomes a JOURNAL field)
/// rather than the flat lines you get from stderr capture.
fn init_tracing(filter: tracing_subscriber::EnvFilter) {
    use tracing_subscriber::prelude::*;

    let stderr_layer = tracing_subscriber::fmt::layer().with_writer(std::io::stderr);

    #[cfg(all(target_os = "linux", feature = "journald"))]
    let journald_layer = tracing_journald::layer().ok();

    let registry = tracing_subscriber::registry()
        .with(filter)
        .with(stderr_layer);

    #[cfg(all(target_os = "linux", feature = "journald"))]
    let registry = registry.with(journald_layer);

    registry.init();
}

/// Categorize an ingest-task termination. For synthetic sources, a clean
/// `Ok(())` is the normal "scenario ended" path — not an error. For real
/// sources (MAV / serial / I2C), an `Ok(())` means the stream closed
/// unexpectedly, which IS a failure: the drone is no longer defended.
fn classify_ingest_exit(
    name: &'static str,
    result: Result<(), tokio::task::JoinError>,
    synth_source: bool,
) -> &'static str {
    match result {
        Ok(()) if synth_source => {
            tracing::info!(task = name, "ingest source exhausted (synth scenario end)");
            // Returning a non-failure string here lets `wait_for_completion`
            // proceed through the graceful shutdown path, draining fusion.
            "source_exhausted"
        }
        Ok(()) => {
            tracing::error!(
                task = name,
                "ingest source ended unexpectedly (real source closed)"
            );
            match name {
                "gps_ingest" => "task_died:gps_ingest",
                "imu_ingest" => "task_died:imu_ingest",
                _ => "task_died:unknown",
            }
        }
        Err(e) => {
            tracing::error!(task = name, error = %e, "ingest task panicked or was aborted");
            match name {
                "gps_ingest" => "task_died:gps_ingest",
                "imu_ingest" => "task_died:imu_ingest",
                _ => "task_died:unknown",
            }
        }
    }
}

fn build_synth_gps(cli: &Cli) -> SpoofInjector<SyntheticGps<LinearNorth>> {
    let origin = (40.0_f64, -100.0_f64, 0.0_f64);
    let traj = LinearNorth { speed_mps: 8.0 };
    let pattern = match cli.scenario {
        Scenario::Clean => SpoofPattern::Clean,
        Scenario::SuddenJump => SpoofPattern::SuddenJump {
            apply_at_s: 30.0,
            jump_north_m: 0.0,
            jump_east_m: 200.0,
        },
        Scenario::GradualDrift => SpoofPattern::GradualDrift {
            apply_at_s: 20.0,
            drift_north_mps: 0.0,
            drift_east_mps: 1.0,
        },
    };
    let gps = SyntheticGps {
        traj,
        rate_hz: 1.0,
        pos_sigma_m: 0.3,
        vel_sigma_mps: 0.05,
        origin,
        seed: cli.seed,
        duration_s: cli.duration as f32,
        start_after_s: 0.5,
    };
    SpoofInjector::new(gps, pattern)
}

fn build_synth_imu(cli: &Cli) -> SyntheticImu<LinearNorth> {
    let traj = LinearNorth { speed_mps: 8.0 };
    SyntheticImu {
        traj,
        rate_hz: 100.0,
        accel_sigma: 0.001,
        gyro_sigma: 0.0001,
        gyro_bias: [0.0; 3],
        accel_bias: [0.0; 3],
        seed: cli.seed.wrapping_add(1),
        duration_s: cli.duration as f32 + 2.0,
    }
}

#[cfg(all(feature = "hw-i2c", target_os = "linux"))]
fn build_i2c_imu(cli: &Cli) -> Result<Box<dyn ImuSource>, FsError> {
    use flyingsquirrel::hw::axis_map::AxisMap;
    use flyingsquirrel::hw::i2c_imu::{I2cImuConfig, I2cImuSource};
    let axis_map = AxisMap::parse(&cli.imu_axis_map)
        .map_err(|e| FsError::Config(format!("--imu-axis-map: {e}")))?;
    Ok(Box::new(I2cImuSource::new(I2cImuConfig {
        bus: cli.imu_bus.clone(),
        addr: cli.imu_addr,
        rate_hz: cli.imu_rate,
        axis_map,
    })))
}

#[cfg(not(all(feature = "hw-i2c", target_os = "linux")))]
fn build_i2c_imu(_cli: &Cli) -> Result<Box<dyn ImuSource>, FsError> {
    Err(FsError::Io(std::io::Error::other(
        "I2C IMU support requires building with --features hw-i2c on Linux. \
         On Windows/Mac, use --imu-source mav (sidecar pattern) or --imu-source synth.",
    )))
}

#[cfg(unix)]
fn spawn_reset_signal_handler(tx: mpsc::Sender<()>) {
    tokio::spawn(async move {
        use tokio::signal::unix::{signal, SignalKind};
        let mut sig = match signal(SignalKind::hangup()) {
            Ok(s) => s,
            Err(e) => {
                tracing::warn!(error = %e, "could not install SIGHUP handler; operator reset disabled");
                return;
            }
        };
        while sig.recv().await.is_some() {
            tracing::warn!("SIGHUP received -> requesting operator reset");
            if tx.send(()).await.is_err() {
                break;
            }
        }
    });
}

#[cfg(windows)]
fn spawn_reset_signal_handler(tx: mpsc::Sender<()>) {
    tokio::spawn(async move {
        let mut sig = match tokio::signal::windows::ctrl_break() {
            Ok(s) => s,
            Err(e) => {
                tracing::warn!(error = %e, "could not install Ctrl-Break handler; operator reset disabled");
                return;
            }
        };
        while sig.recv().await.is_some() {
            tracing::warn!("Ctrl-Break received -> requesting operator reset");
            if tx.send(()).await.is_err() {
                break;
            }
        }
    });
}

#[cfg(test)]
mod cli_tests {
    use super::*;

    #[test]
    fn expected_home_accepts_negative_coordinates() {
        // A southern/western launch site has a negative coordinate. Without
        // `allow_hyphen_values` clap rejects the leading '-' as an unknown flag
        // ("unexpected argument '-3'") — which crashed the detector container in
        // the SITL harness against ArduPilot's Canberra default home.
        let cli =
            Cli::try_parse_from(["flyingsquirrel", "--expected-home", "-35.363261,149.16523"])
                .expect("negative expected-home must parse");
        assert_eq!(cli.expected_home.as_deref(), Some("-35.363261,149.16523"));
    }
}
