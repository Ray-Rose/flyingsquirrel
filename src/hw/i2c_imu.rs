//! MPU-6050-class IMU over Linux I2C.
//!
//! Reads accel + gyro registers (0x3B..0x48, 14 bytes: AX/AY/AZ/TEMP/GX/GY/GZ
//! as 16-bit big-endian) at the configured rate, scales to SI units, and
//! yields `ImuSample` on the stream. Wakes the chip from sleep on init.
//!
//! This file owns the bus I/O and the tick loop only. Register meanings, the
//! WHO_AM_I identity gate, and the read-health escalation policy (degraded
//! warning → in-process re-init → stream-end) live in [`crate::hw::mpu6050`],
//! which is not feature-gated so that logic is unit-tested on every platform.
//!
//! Other I2C IMUs (BMI088, ICM-42688, MPU-9250) drop in by replacing the
//! register addresses, scale factors, and init sequence. The trait surface
//! stays the same.
//!
//! Cfg-gated on Linux + the `hw-i2c` feature.

use crate::error::{FsError, IngestError};
use crate::hw::axis_map::AxisMap;
use crate::hw::mpu6050::{
    self, ReadHealth, Trouble, TroubleKind, Verdict, MPU6050_DEFAULT_ADDR, RAW_FRAME_LEN,
};
use crate::ingest::{BoxStream, ImuSource};
use crate::mav::imu_sample_is_plausible;
use crate::types::ImuSample;
use async_stream::stream;
use embedded_hal::i2c::I2c;
use linux_embedded_hal::I2cdev;
use std::time::Duration;
use tracing::warn;

#[derive(Debug, Clone)]
pub struct I2cImuConfig {
    pub bus: String,
    pub addr: u8,
    pub rate_hz: f32,
    /// Chip-to-body axis remap for the mounting orientation. Default is
    /// identity (chip mounted X-forward / Y-right / Z-down). See
    /// [`crate::hw::axis_map::AxisMap`].
    pub axis_map: AxisMap,
}

impl Default for I2cImuConfig {
    fn default() -> Self {
        Self {
            bus: "/dev/i2c-1".to_string(),
            addr: MPU6050_DEFAULT_ADDR,
            rate_hz: 100.0,
            axis_map: AxisMap::IDENTITY,
        }
    }
}

pub struct I2cImuSource {
    cfg: I2cImuConfig,
}

impl I2cImuSource {
    pub fn new(cfg: I2cImuConfig) -> Self {
        Self { cfg }
    }
}

fn open_and_init(cfg: &I2cImuConfig) -> Result<I2cdev, FsError> {
    let mut bus = I2cdev::new(&cfg.bus).map_err(|e| {
        FsError::Ingest(IngestError::I2c(format!("open i2c bus {}: {}", cfg.bus, e)))
    })?;
    // Identity gate BEFORE any configuration: a same-protocol substitute
    // (MPU-9250 / ICM class) would accept every write below and then deliver
    // silently mis-scaled samples forever. WHO_AM_I is readable in sleep mode.
    let mut who = [0u8; 1];
    bus.write_read(cfg.addr, &[mpu6050::REG_WHO_AM_I], &mut who)
        .map_err(|e| {
            FsError::Ingest(IngestError::I2c(format!("mpu6050 who_am_i read: {:?}", e)))
        })?;
    mpu6050::check_who_am_i(who[0]).map_err(|m| FsError::Ingest(IngestError::I2c(m)))?;
    // Wake chip from sleep (PWR_MGMT_1 = 0). A power-on/brown-out reset sets
    // the SLEEP bit again — this same write is the recovery action when the
    // read loop detects frozen output and asks for a re-init.
    bus.write(cfg.addr, &[mpu6050::REG_PWR_MGMT_1, 0x00])
        .map_err(|e| FsError::Ingest(IngestError::I2c(format!("mpu6050 wake: {:?}", e))))?;
    // Set default ranges: ±2g accel, ±250 °/s gyro (matches the scale
    // constants in `mpu6050`). These are also the power-on-reset values, so a
    // chip that reset behind our back still matches the decode assumptions.
    bus.write(cfg.addr, &[mpu6050::REG_ACCEL_CONFIG, 0x00])
        .map_err(|e| FsError::Ingest(IngestError::I2c(format!("mpu6050 accel cfg: {:?}", e))))?;
    bus.write(cfg.addr, &[mpu6050::REG_GYRO_CONFIG, 0x00])
        .map_err(|e| FsError::Ingest(IngestError::I2c(format!("mpu6050 gyro cfg: {:?}", e))))?;
    Ok(bus)
}

/// One 14-byte burst read of the sensor register block. Decode/scaling is
/// [`mpu6050::sample_from_raw`]; the raw frame is also what the freeze
/// detector compares (it includes the temperature register, which dithers
/// on an awake die).
fn read_raw(bus: &mut I2cdev, addr: u8) -> Result<[u8; RAW_FRAME_LEN], std::io::Error> {
    let mut buf = [0u8; RAW_FRAME_LEN];
    bus.write_read(addr, &[mpu6050::REG_ACCEL_XOUT_H], &mut buf)
        .map_err(|e| std::io::Error::other(format!("i2c read: {:?}", e)))?;
    Ok(buf)
}

/// Emit the escalation ladder's log lines for one bad read. The caller acts
/// on `t.reinit` / `t.end_stream` — those need `&mut bus` / `return`, which
/// only the stream body has.
fn log_trouble(t: &Trouble, cfg: &I2cImuConfig) {
    let kind = match t.kind {
        TroubleKind::ReadError => "read failures",
        TroubleKind::FrozenOutput => {
            "bit-identical frames (sensor registers frozen; chip asleep after brown-out?)"
        }
    };
    if t.warn_degraded {
        tracing::warn!(
            bus = %cfg.bus,
            addr = format!("{:#x}", cfg.addr),
            consecutive_errors = t.consecutive_errors,
            "i2c imu DEGRADED: {} consecutive {}. Detector cross-check is \
             compromised — verify cable / chip / pull-ups",
            t.consecutive_errors,
            kind
        );
    }
    if t.end_stream {
        tracing::error!(
            bus = %cfg.bus,
            addr = format!("{:#x}", cfg.addr),
            consecutive_errors = t.consecutive_errors,
            "i2c imu BUS DOWN ({} consecutive {}; in-process re-init did not \
             recover); ending stream so systemd restarts the process and \
             re-initializes",
            t.consecutive_errors,
            kind
        );
    }
}

impl ImuSource for I2cImuSource {
    fn into_stream(self: Box<Self>) -> BoxStream<'static, Result<ImuSample, FsError>> {
        let cfg = self.cfg;
        let stream = stream! {
            let mut bus = match open_and_init(&cfg) {
                Ok(b) => b,
                Err(e) => { yield Err(e); return; }
            };
            let interval_dur = Duration::from_secs_f32(1.0 / cfg.rate_hz.max(1.0));
            let mut ticker = tokio::time::interval(interval_dur);
            ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            // AUDIT E-14 + hardware-readiness: the ladder (warn → re-init →
            // end-stream) and the frozen-output detector live in `mpu6050::
            // ReadHealth`; see that module for thresholds and rationale.
            let mut health = ReadHealth::new();
            loop {
                ticker.tick().await;
                // I2C is blocking; for 100 Hz at ~1 ms per read this is acceptable
                // on its own task. If it becomes a problem, wrap in spawn_blocking.
                let mut trouble: Option<Trouble> = None;
                match read_raw(&mut bus, cfg.addr) {
                    Ok(raw) => match health.on_read_ok(&raw) {
                        Verdict::Fresh { recovered_after } => {
                            if let Some(streak) = recovered_after {
                                tracing::info!(
                                    bus = %cfg.bus,
                                    addr = format!("{:#x}", cfg.addr),
                                    prior_errors = streak,
                                    "i2c imu RECOVERED after consecutive bad reads"
                                );
                            }
                            let sample = mpu6050::sample_from_raw(&raw, &cfg.axis_map);
                            if imu_sample_is_plausible(&sample) {
                                yield Ok(sample);
                            } else {
                                warn!("dropped implausible i2c imu sample (NaN/saturated)");
                            }
                        }
                        // Frozen frame at/past threshold: stale data, not a
                        // measurement — never yield it (the dead-reckoner
                        // would integrate it); run the ladder instead.
                        Verdict::Frozen(t) => trouble = Some(t),
                    },
                    Err(e) => {
                        let t = health.on_read_err();
                        if t.consecutive_errors == 1 {
                            // First error of a streak — log at debug, may be transient.
                            tracing::debug!(error = %e, "i2c imu read error (transient?)");
                        }
                        trouble = Some(t);
                    }
                }
                if let Some(t) = trouble {
                    log_trouble(&t, &cfg);
                    if t.end_stream {
                        // End the stream — runtime treats this as task exit;
                        // bin/flyingsquirrel classify_ingest_exit fires.
                        return;
                    }
                    if t.reinit {
                        // In-process recovery: drop the fd and redo open+init.
                        // Covers a slept chip (frozen output), a wedged fd
                        // (kernel driver rebind), and persistent bus faults —
                        // WITHOUT the process restart that would wipe the
                        // detector's FSM/CUSUM/dead-reckoning state and leave
                        // the vehicle undefended for the 2–5 s startup window.
                        // Only an actual good READ resets the ladder, so a
                        // "successful" re-init of a still-broken sensor keeps
                        // escalating toward stream-end (systemd backstop).
                        match open_and_init(&cfg) {
                            Ok(fresh) => {
                                bus = fresh;
                                tracing::info!(
                                    bus = %cfg.bus,
                                    addr = format!("{:#x}", cfg.addr),
                                    consecutive_errors = t.consecutive_errors,
                                    "i2c imu re-opened + re-initialized (in-process recovery attempt)"
                                );
                            }
                            Err(e) => tracing::warn!(
                                bus = %cfg.bus,
                                addr = format!("{:#x}", cfg.addr),
                                error = %e,
                                "i2c imu re-init attempt failed; will retry per the ladder"
                            ),
                        }
                    }
                }
            }
        };
        Box::pin(stream)
    }
}
