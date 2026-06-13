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

#[cfg(all(feature = "hw-i2c", target_os = "linux"))]
pub mod i2c_imu;
