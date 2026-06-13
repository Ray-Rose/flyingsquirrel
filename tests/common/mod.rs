//! Shared test harness. Builds a `SyntheticGps` + `SyntheticImu` from a
//! `Scenario` description, wires the runtime via `spawn_all`, and provides
//! a `TestController` that records all events into a shared Vec.

use async_trait::async_trait;
use flyingsquirrel::action::FlightController;
use flyingsquirrel::error::FsError;
use flyingsquirrel::fusion::FusionConfig;
use flyingsquirrel::ingest::{GpsSource, ImuSource};
use flyingsquirrel::nav::NavConfig;
use flyingsquirrel::runtime::{spawn_all, ForensicCfg, Handles};
use flyingsquirrel::sim::spoof::{SpoofInjector, SpoofPattern};
use flyingsquirrel::sim::trajectory::LinearNorth;
use flyingsquirrel::sim::{SyntheticGps, SyntheticImu};
use flyingsquirrel::types::SpoofingEvent;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::Mutex;
use std::time::Duration;

pub struct TestController {
    pub events: Arc<Mutex<Vec<SpoofingEvent>>>,
    pub sever_count: Arc<Mutex<u32>>,
    pub rtb_count: Arc<Mutex<u32>>,
}

impl TestController {
    pub fn new() -> Self {
        Self {
            events: Arc::new(Mutex::new(Vec::new())),
            sever_count: Arc::new(Mutex::new(0)),
            rtb_count: Arc::new(Mutex::new(0)),
        }
    }
}

#[async_trait]
impl FlightController for TestController {
    async fn on_event(&self, ev: &SpoofingEvent) {
        self.events.lock().unwrap().push(ev.clone());
    }
    async fn sever_gps(&self) -> Result<(), FsError> {
        *self.sever_count.lock().unwrap() += 1;
        Ok(())
    }
    async fn engage_rtb(&self) -> Result<(), FsError> {
        *self.rtb_count.lock().unwrap() += 1;
        Ok(())
    }
}

// Not `Copy` — `forensic_dir: Option<PathBuf>` is owned. Existing tests
// construct the struct once per call and pass by value, so Clone is enough.
#[derive(Debug, Clone)]
pub struct Scenario {
    pub duration_s: u32,
    pub speed_mps: f64,
    pub pattern: SpoofPattern,
    pub seed: u64,
    /// If set, the SyntheticGps reports lat/lon centered on this origin
    /// instead of the default (40, -100). Used for BootAnchorRejected tests
    /// where the operator's `expected_home` is far from where the GPS
    /// actually reports.
    pub gps_origin: Option<(f64, f64)>,
    /// If set, FusionConfig.nav.expected_home is populated with this
    /// (lat, lon) — operator's declared launch site. The first GPS fix
    /// must land within `expected_home_radius_m` or BootAnchorRejected
    /// fires.
    pub expected_home: Option<(f64, f64)>,
    pub expected_home_radius_m: f64,
    /// Optional override for FusionConfig.detect.re_anchor_max_distance_m.
    /// Used by ReAnchorRejected tests to make the veto easier to trigger.
    pub re_anchor_max_distance_m: Option<f32>,
    /// If set, fusion writes a forensic snapshot JSON to this directory on
    /// the first Spoofed transition. Used by `scenario_forensic` to verify
    /// the dump-on-trigger path end to end.
    pub forensic_dir: Option<PathBuf>,
}

impl Default for Scenario {
    fn default() -> Self {
        Self {
            duration_s: 60,
            speed_mps: 8.0,
            pattern: SpoofPattern::Clean,
            seed: 0xC1EA_C1EA_C1EA_C1EA,
            gps_origin: None,
            expected_home: None,
            expected_home_radius_m: 1000.0,
            re_anchor_max_distance_m: None,
            forensic_dir: None,
        }
    }
}

pub struct ScenarioOutcome {
    pub events: Vec<SpoofingEvent>,
    // dead_code: integration tests compile as separate binaries; not every
    // test uses every field. The harness exposes them all for the tests
    // that do.
    #[allow(dead_code)]
    pub sever_count: u32,
    #[allow(dead_code)]
    pub rtb_count: u32,
    #[allow(dead_code)]
    pub handles: Handles,
}

/// Run a scenario under virtual time. Pauses tokio time, advances by
/// `duration_s + 2` simulated seconds in small ticks to let events drain.
pub async fn run_scenario(s: Scenario) -> ScenarioOutcome {
    let traj = LinearNorth {
        speed_mps: s.speed_mps,
    };
    let (origin_lat, origin_lon) = s.gps_origin.unwrap_or((40.0, -100.0));
    let origin = (origin_lat, origin_lon, 0.0);

    // Test IMU is very quiet (true noise sigmas matched would let random walk
    // drift the integrated position several meters over 80 s, triggering
    // false CUSUM events on a clean trajectory). The DETECTION logic is
    // what's under test here, not the IMU error model.
    let imu = SyntheticImu {
        traj,
        rate_hz: 100.0,
        accel_sigma: 0.001,
        gyro_sigma: 0.0001,
        gyro_bias: [0.0; 3],
        seed: s.seed.wrapping_add(1),
        duration_s: s.duration_s as f32 + 2.0,
    };
    let gps = SyntheticGps {
        traj,
        rate_hz: 1.0,
        pos_sigma_m: 0.3,
        vel_sigma_mps: 0.05,
        origin,
        seed: s.seed,
        duration_s: s.duration_s as f32,
        start_after_s: 0.5,
    };

    let controller = Arc::new(TestController::new());
    let events_handle = controller.events.clone();
    let sever_handle = controller.sever_count.clone();
    let rtb_handle = controller.rtb_count.clone();

    let gps_boxed: Box<dyn GpsSource> = Box::new(SpoofInjector::new(gps, s.pattern));
    let imu_boxed: Box<dyn ImuSource> = Box::new(imu);

    // Build NavConfig with optional boot-anchor cross-check.
    let mut nav = NavConfig::default();
    if let Some((lat, lon)) = s.expected_home {
        nav.expected_home = Some((lat, lon));
        nav.max_home_distance_m = s.expected_home_radius_m;
    }
    let mut cfg = FusionConfig {
        nav,
        ..FusionConfig::default()
    };
    if let Some(m) = s.re_anchor_max_distance_m {
        cfg.detect.re_anchor_max_distance_m = m;
    }
    let forensic_cfg = s.forensic_dir.clone().map(|dir| ForensicCfg {
        dir,
        window_s: flyingsquirrel::forensic::DEFAULT_WINDOW_S,
    });
    let handles = spawn_all(gps_boxed, imu_boxed, controller, cfg, None, forensic_cfg).unwrap();

    // Drive virtual time forward. With tokio::time::pause(), sleeps are virtual.
    let ticks = (s.duration_s + 2) * 10;
    for _ in 0..ticks {
        tokio::time::advance(Duration::from_millis(100)).await;
        // Yield so spawned tasks can make progress.
        tokio::task::yield_now().await;
    }

    // Drain in-flight background tasks (forensic dump, RTL verify) before
    // collecting events — mirrors the binary's shutdown drain. Without this,
    // the forensic dump's REAL-TIME disk I/O (tokio::fs on the blocking pool)
    // is still in flight when we snapshot the event Vec under paused virtual
    // time, so the `ForensicDumpWritten` event is missed and the
    // scenario_forensic assertion fails nondeterministically. drain_pending_bg
    // awaits the real completion of those tasks regardless of virtual time.
    let _ = flyingsquirrel::runtime::drain_pending_bg(
        handles.pending_bg.clone(),
        Duration::from_secs(5),
    )
    .await;
    // One more yield so the drained tasks' broadcast events reach the action
    // task and land in the collected Vec.
    for _ in 0..5 {
        tokio::task::yield_now().await;
    }

    let events = events_handle.lock().unwrap().clone();
    let sever_count = *sever_handle.lock().unwrap();
    let rtb_count = *rtb_handle.lock().unwrap();

    ScenarioOutcome {
        events,
        sever_count,
        rtb_count,
        handles,
    }
}

pub fn count_kind(events: &[SpoofingEvent], kind: flyingsquirrel::types::SpoofKind) -> usize {
    events.iter().filter(|e| e.kind == kind).count()
}

#[allow(dead_code)]
pub fn reached_state(events: &[SpoofingEvent], state: flyingsquirrel::types::NavStateKind) -> bool {
    events.iter().any(|e| e.state == state)
}
