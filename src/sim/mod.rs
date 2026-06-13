//! Simulation backends for `GpsSource` and `ImuSource`. Ground truth is
//! produced by `trajectory::TrajectoryGenerator`, then noised into sensor
//! samples. `spoof::SpoofInjector` optionally overlays a GPS spoofing attack.

pub mod spoof;
pub mod trajectory;

use crate::error::FsError;
use crate::ingest::{BoxStream, GpsSource, ImuSource};
use crate::nav::{G, R_EARTH_M};
use crate::types::{GpsFix, ImuSample, Timestamp};
use async_stream::stream;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use rand::Rng;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;
use std::time::Duration;
use trajectory::TrajectoryGenerator;

/// Synthetic GPS source. Produces fixes at `rate_hz` derived from the
/// ground-truth trajectory plus per-fix Gaussian position+velocity noise.
pub struct SyntheticGps<T: TrajectoryGenerator + Send + 'static> {
    pub traj: T,
    pub rate_hz: f32,
    pub pos_sigma_m: f32,
    pub vel_sigma_mps: f32,
    pub origin: (f64, f64, f64),
    pub seed: u64,
    pub duration_s: f32,
    pub start_after_s: f32,
}

impl<T: TrajectoryGenerator + Send + 'static> GpsSource for SyntheticGps<T> {
    fn into_stream(self: Box<Self>) -> BoxStream<'static, Result<GpsFix, FsError>> {
        let SyntheticGps {
            traj,
            rate_hz,
            pos_sigma_m,
            vel_sigma_mps,
            origin,
            seed,
            duration_s,
            start_after_s,
        } = *self;
        let dt = 1.0 / rate_hz as f64;
        let stream = stream! {
            let mut rng = ChaCha8Rng::seed_from_u64(seed);
            let mut t = start_after_s as f64;
            tokio::time::sleep(Duration::from_secs_f64(start_after_s as f64)).await;
            while t <= duration_s as f64 + 1e-9 {
                let gt = traj.sample(t);
                let n_noise = gauss(&mut rng) * pos_sigma_m as f64;
                let e_noise = gauss(&mut rng) * pos_sigma_m as f64;
                let (lat, lon) = ned_to_lla(gt.pos_n + n_noise, gt.pos_e + e_noise, origin.0, origin.1);
                let alt = origin.2 - gt.pos_d;
                let speed = (gt.vel_n * gt.vel_n + gt.vel_e * gt.vel_e).sqrt();
                let course = gt.vel_e.atan2(gt.vel_n).to_degrees();
                let course_deg = ((course % 360.0) + 360.0) % 360.0;
                let fix = GpsFix {
                    t: Timestamp::now_mono(),
                    lat_deg: lat,
                    lon_deg: lon,
                    alt_m: alt,
                    speed_mps: Some(speed + gauss(&mut rng) * vel_sigma_mps as f64),
                    course_deg: Some(course_deg),
                    hdop: Some(0.9),
                    sats: Some(12),
                };
                yield Ok(fix);
                t += dt;
                tokio::time::sleep(Duration::from_secs_f64(dt)).await;
            }
        };
        Box::pin(stream)
    }
}

/// Synthetic IMU source. Produces samples at `rate_hz` from ground truth.
/// Body frame X-forward, Y-right, Z-down.
pub struct SyntheticImu<T: TrajectoryGenerator + Send + 'static> {
    pub traj: T,
    pub rate_hz: f32,
    pub accel_sigma: f32,
    pub gyro_sigma: f32,
    pub gyro_bias: [f32; 3],
    pub seed: u64,
    pub duration_s: f32,
}

impl<T: TrajectoryGenerator + Send + 'static> ImuSource for SyntheticImu<T> {
    fn into_stream(self: Box<Self>) -> BoxStream<'static, Result<ImuSample, FsError>> {
        let SyntheticImu {
            traj,
            rate_hz,
            accel_sigma,
            gyro_sigma,
            gyro_bias,
            seed,
            duration_s,
        } = *self;
        let dt = 1.0 / rate_hz as f64;
        let stream = stream! {
            let mut rng = ChaCha8Rng::seed_from_u64(seed);
            let mut t = 0.0_f64;
            while t <= duration_s as f64 + 1e-9 {
                let gt = traj.sample(t);
                // Physical specific force in body frame: f = R(q)ᵀ·(a_ned − g_ned),
                // with g_ned = [0,0,+g]. At rest, level → [0,0,-g], exactly what a
                // real MEMS IMU / ArduPilot SCALED_IMU emits. (Was acc_d + g, the
                // non-physical gravity-reaction convention; the detector now
                // expects specific force so the sim must produce it.)
                let q = Quaternion::new(gt.q[0], gt.q[1], gt.q[2], gt.q[3]);
                let uq = UnitQuaternion::from_quaternion(q);
                let total_ned = Vector3::new(
                    (gt.acc_n) as f32,
                    (gt.acc_e) as f32,
                    (gt.acc_d - G) as f32,
                );
                let body = uq.inverse_transform_vector(&total_ned);
                let accel = [
                    body.x + gauss(&mut rng) as f32 * accel_sigma,
                    body.y + gauss(&mut rng) as f32 * accel_sigma,
                    body.z + gauss(&mut rng) as f32 * accel_sigma,
                ];
                let gyro_body = uq.inverse_transform_vector(&Vector3::new(
                    gt.gyro_n as f32,
                    gt.gyro_e as f32,
                    gt.gyro_d as f32,
                ));
                let gyro = [
                    gyro_body.x + gyro_bias[0] + gauss(&mut rng) as f32 * gyro_sigma,
                    gyro_body.y + gyro_bias[1] + gauss(&mut rng) as f32 * gyro_sigma,
                    gyro_body.z + gyro_bias[2] + gauss(&mut rng) as f32 * gyro_sigma,
                ];
                let sample = ImuSample {
                    t: Timestamp::now_mono(),
                    accel_mps2: accel,
                    gyro_rps: gyro,
                };
                yield Ok(sample);
                t += dt;
                tokio::time::sleep(Duration::from_secs_f64(dt)).await;
            }
        };
        Box::pin(stream)
    }
}

fn gauss(rng: &mut ChaCha8Rng) -> f64 {
    // Box-Muller
    let u1: f64 = rng.gen_range(1e-9..1.0);
    let u2: f64 = rng.gen_range(0.0..1.0);
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

/// Inverse of `nav::lla_to_ned` (equirectangular).
pub fn ned_to_lla(n: f64, e: f64, lat0: f64, lon0: f64) -> (f64, f64) {
    let lat0_rad = lat0.to_radians();
    let dlat_deg = (n / R_EARTH_M).to_degrees();
    let dlon_deg = (e / (R_EARTH_M * lat0_rad.cos())).to_degrees();
    (lat0 + dlat_deg, lon0 + dlon_deg)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nav::lla_to_ned;

    #[test]
    fn ned_to_lla_roundtrips() {
        let (lat0, lon0) = (40.0, -100.0);
        let p = lla_to_ned(40.0 + 0.001, -100.0 + 0.001, 0.0, lat0, lon0);
        let (lat, lon) = ned_to_lla(p.n, p.e, lat0, lon0);
        assert!((lat - (40.0 + 0.001)).abs() < 1e-9, "lat={}", lat);
        assert!((lon - (-100.0 + 0.001)).abs() < 1e-9, "lon={}", lon);
    }
}
