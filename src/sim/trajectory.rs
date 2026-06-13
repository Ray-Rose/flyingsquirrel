//! Ground-truth trajectory generators (no noise). Produce position, velocity,
//! linear acceleration in NED, plus body attitude as a quaternion (w,x,y,z).

/// One trajectory sample at time `t`.
#[derive(Debug, Clone, Copy)]
pub struct GroundTruth {
    pub t_secs: f64,
    pub pos_n: f64,
    pub pos_e: f64,
    pub pos_d: f64,
    pub vel_n: f64,
    pub vel_e: f64,
    pub vel_d: f64,
    pub acc_n: f64,
    pub acc_e: f64,
    pub acc_d: f64,
    /// Quaternion (w, x, y, z), body -> NED.
    pub q: [f32; 4],
    /// Angular rate in NED. Will be rotated into body for the IMU.
    pub gyro_n: f64,
    pub gyro_e: f64,
    pub gyro_d: f64,
}

pub trait TrajectoryGenerator: Clone {
    fn sample(&self, t_secs: f64) -> GroundTruth;
}

/// Straight-line constant-velocity trajectory heading along +N at `speed`.
#[derive(Debug, Clone, Copy)]
pub struct LinearNorth {
    pub speed_mps: f64,
}

impl TrajectoryGenerator for LinearNorth {
    fn sample(&self, t: f64) -> GroundTruth {
        // Body Z-down stays aligned with NED Z-down. Yaw 0 -> nose pointed +N.
        // Identity quaternion is correct.
        GroundTruth {
            t_secs: t,
            pos_n: self.speed_mps * t,
            pos_e: 0.0,
            pos_d: 0.0,
            vel_n: self.speed_mps,
            vel_e: 0.0,
            vel_d: 0.0,
            acc_n: 0.0,
            acc_e: 0.0,
            acc_d: 0.0,
            q: [1.0, 0.0, 0.0, 0.0],
            gyro_n: 0.0,
            gyro_e: 0.0,
            gyro_d: 0.0,
        }
    }
}

/// Stationary at the origin (with body attitude identity). Useful for
/// false-alarm tests.
#[derive(Debug, Clone, Copy, Default)]
pub struct Static;

impl TrajectoryGenerator for Static {
    fn sample(&self, t: f64) -> GroundTruth {
        GroundTruth {
            t_secs: t,
            pos_n: 0.0,
            pos_e: 0.0,
            pos_d: 0.0,
            vel_n: 0.0,
            vel_e: 0.0,
            vel_d: 0.0,
            acc_n: 0.0,
            acc_e: 0.0,
            acc_d: 0.0,
            q: [1.0, 0.0, 0.0, 0.0],
            gyro_n: 0.0,
            gyro_e: 0.0,
            gyro_d: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn linear_north_advances() {
        let t = LinearNorth { speed_mps: 10.0 };
        let a = t.sample(0.0);
        let b = t.sample(1.0);
        assert!((b.pos_n - a.pos_n - 10.0).abs() < 1e-9);
    }
}
