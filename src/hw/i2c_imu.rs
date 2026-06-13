//! MPU-6050-class IMU over Linux I2C.
//!
//! Reads accel + gyro registers (0x3B..0x48, 14 bytes: AX/AY/AZ/TEMP/GX/GY/GZ
//! as 16-bit big-endian) at the configured rate, scales to SI units, and
//! yields `ImuSample` on the stream. Wakes the chip from sleep on init.
//!
//! Other I2C IMUs (BMI088, ICM-42688, MPU-9250) drop in by replacing the
//! register addresses, scale factors, and init sequence. The trait surface
//! stays the same.
//!
//! Cfg-gated on Linux + the `hw-i2c` feature.

use crate::error::{FsError, IngestError};
use crate::ingest::{BoxStream, ImuSource};
use crate::mav::imu_sample_is_plausible;
use crate::types::{ImuSample, Timestamp};
use async_stream::stream;
use embedded_hal::i2c::I2c;
use linux_embedded_hal::I2cdev;
use std::time::Duration;
use tracing::warn;

const MPU6050_DEFAULT_ADDR: u8 = 0x68;

/// Register addresses (see MPU-6050 Register Map RM-MPU-6000A v4.2).
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_ACCEL_CONFIG: u8 = 0x1C;
const REG_ACCEL_XOUT_H: u8 = 0x3B;

/// Scale factors at default ranges (±2g, ±250 °/s).
const G_PER_LSB: f32 = 1.0 / 16384.0;
const DEG_S_PER_LSB: f32 = 1.0 / 131.0;
const G_TO_MPS2: f32 = 9.806_65;

#[derive(Debug, Clone)]
pub struct I2cImuConfig {
    pub bus: String,
    pub addr: u8,
    pub rate_hz: f32,
}

impl Default for I2cImuConfig {
    fn default() -> Self {
        Self {
            bus: "/dev/i2c-1".to_string(),
            addr: MPU6050_DEFAULT_ADDR,
            rate_hz: 100.0,
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
        FsError::Ingest(IngestError::NmeaParse(format!(
            "open i2c bus {}: {}",
            cfg.bus, e
        )))
    })?;
    // Wake chip from sleep (PWR_MGMT_1 = 0).
    bus.write(cfg.addr, &[REG_PWR_MGMT_1, 0x00])
        .map_err(|e| FsError::Ingest(IngestError::NmeaParse(format!("mpu6050 wake: {:?}", e))))?;
    // Set default ranges: ±2g accel, ±250 °/s gyro (matches our scale constants).
    bus.write(cfg.addr, &[REG_ACCEL_CONFIG, 0x00])
        .map_err(|e| {
            FsError::Ingest(IngestError::NmeaParse(format!(
                "mpu6050 accel cfg: {:?}",
                e
            )))
        })?;
    bus.write(cfg.addr, &[REG_GYRO_CONFIG, 0x00]).map_err(|e| {
        FsError::Ingest(IngestError::NmeaParse(format!("mpu6050 gyro cfg: {:?}", e)))
    })?;
    Ok(bus)
}

fn read_one(bus: &mut I2cdev, addr: u8) -> Result<ImuSample, std::io::Error> {
    let mut buf = [0u8; 14];
    bus.write_read(addr, &[REG_ACCEL_XOUT_H], &mut buf)
        .map_err(|e| std::io::Error::other(format!("i2c read: {:?}", e)))?;
    let ax = i16::from_be_bytes([buf[0], buf[1]]);
    let ay = i16::from_be_bytes([buf[2], buf[3]]);
    let az = i16::from_be_bytes([buf[4], buf[5]]);
    // buf[6..8] = TEMP, unused
    let gx = i16::from_be_bytes([buf[8], buf[9]]);
    let gy = i16::from_be_bytes([buf[10], buf[11]]);
    let gz = i16::from_be_bytes([buf[12], buf[13]]);

    Ok(ImuSample {
        t: Timestamp::now_mono(),
        accel_mps2: [
            ax as f32 * G_PER_LSB * G_TO_MPS2,
            ay as f32 * G_PER_LSB * G_TO_MPS2,
            az as f32 * G_PER_LSB * G_TO_MPS2,
        ],
        gyro_rps: [
            (gx as f32 * DEG_S_PER_LSB).to_radians(),
            (gy as f32 * DEG_S_PER_LSB).to_radians(),
            (gz as f32 * DEG_S_PER_LSB).to_radians(),
        ],
    })
}

/// AUDIT E-14 fix: after this many consecutive read errors, emit a single
/// loud warning. Operator dashboards / log alerts can pick this up so a
/// silently-dead I2C bus is visible. At 100 Hz this is 1 second of failures.
const I2C_DEGRADED_THRESHOLD: u32 = 100;
/// After this many consecutive read errors, give up and end the stream.
/// systemd will then restart the process and re-init the bus. At 100 Hz this
/// is 10 seconds — long enough to absorb a real transient (cable jiggle)
/// but short enough that a wedged bus doesn't waste flight time. The
/// detector should NOT continue running GPS-only past this point: the
/// cross-check defense REQUIRES IMU samples to be meaningful.
const I2C_BUS_DOWN_THRESHOLD: u32 = 1000;

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
            let mut consecutive_errors: u32 = 0;
            let mut warned_degraded = false;
            loop {
                ticker.tick().await;
                // I2C is blocking; for 100 Hz at ~1 ms per read this is acceptable
                // on its own task. If it becomes a problem, wrap in spawn_blocking.
                match read_one(&mut bus, cfg.addr) {
                    Ok(sample) => {
                        // Reset error tracking on a successful read.
                        if consecutive_errors > 0 {
                            tracing::info!(
                                bus = %cfg.bus,
                                addr = format!("{:#x}", cfg.addr),
                                prior_errors = consecutive_errors,
                                "i2c imu RECOVERED after consecutive errors"
                            );
                        }
                        consecutive_errors = 0;
                        warned_degraded = false;
                        if imu_sample_is_plausible(&sample) {
                            yield Ok(sample);
                        } else {
                            warn!("dropped implausible i2c imu sample (NaN/saturated)");
                        }
                    }
                    Err(e) => {
                        consecutive_errors = consecutive_errors.saturating_add(1);
                        // AUDIT E-14 fix: previously these errors were logged
                        // at warn! on every iteration and yielded as Err, but
                        // runtime.rs ingest only logs the error and continues
                        // — the IMU silently goes dark while GPS keeps flowing,
                        // breaking the cross-check. We now emit ONCE at the
                        // degraded threshold and END the stream at the bus-down
                        // threshold so systemd restarts the process.
                        if consecutive_errors == 1 {
                            // First error of a streak — log at debug, may be transient.
                            tracing::debug!(error = %e, "i2c imu read error (transient?)");
                        }
                        if consecutive_errors == I2C_DEGRADED_THRESHOLD && !warned_degraded {
                            warned_degraded = true;
                            tracing::warn!(
                                bus = %cfg.bus,
                                addr = format!("{:#x}", cfg.addr),
                                consecutive_errors,
                                "i2c imu DEGRADED: {} consecutive read failures. \
                                 Detector cross-check is compromised — verify cable / \
                                 chip / pull-ups",
                                consecutive_errors
                            );
                        }
                        if consecutive_errors >= I2C_BUS_DOWN_THRESHOLD {
                            tracing::error!(
                                bus = %cfg.bus,
                                addr = format!("{:#x}", cfg.addr),
                                consecutive_errors,
                                "i2c imu BUS DOWN ({} consecutive errors); ending stream \
                                 so systemd restarts the process and re-initializes",
                                consecutive_errors
                            );
                            // End the stream — runtime treats this as task exit;
                            // bin/flyingsquirrel classify_ingest_exit fires.
                            return;
                        }
                        // Don't yield Err — that propagates as a log line only,
                        // and would otherwise flood the log at the read rate.
                    }
                }
            }
        };
        Box::pin(stream)
    }
}
