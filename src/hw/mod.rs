//! Real-hardware sensor backends.
//!
//! - `serial_gps`: NMEA-over-serial GPS receiver (u-blox NEO-M8/M9/M10
//!   class, generic NMEA modules). Cross-platform via tokio-serial.
//! - `i2c_imu` (Linux-only, feature `hw-i2c`): MPU-6050-class IMU over
//!   /dev/i2c-N. Other chips drop in by replacing the register read.
//!
//! Both implement the existing `GpsSource` / `ImuSource` traits — the
//! algorithm is identical regardless of where the bytes come from.

pub mod serial_gps;

/// Sensor-to-body axis remap (for arbitrarily-mounted IMUs). Not feature-gated
/// so the remap math is unit-tested on every platform, even though it is only
/// applied on the Linux I2C path today.
pub mod axis_map;

/// MPU-6050 register decode, WHO_AM_I identity check, and the read-health
/// escalation ladder (degraded-warn → in-process re-init → stream-end).
/// Pure logic, not feature-gated — same rationale as `axis_map`: the policy
/// is unit-tested on every platform; only the bus I/O is Linux-gated.
pub mod mpu6050;

#[cfg(all(feature = "hw-i2c", target_os = "linux"))]
pub mod i2c_imu;
