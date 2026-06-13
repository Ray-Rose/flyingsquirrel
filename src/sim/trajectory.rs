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

/// Constant-speed level circle in the horizontal plane: starts at the origin
/// heading +N and turns right (toward +E) at yaw rate ω = speed / radius.
///
/// Unlike `LinearNorth`/`Static` (identity attitude, zero accel, zero gyro),
/// this exercises the WHOLE nav stack under rotation: the Madgwick filter must
/// track a continuously-changing yaw, the strapdown must integrate a non-zero
/// (centripetal) linear acceleration, and the gyro path carries a real yaw
/// rate. That's where attitude/integration errors and accel-bias coupling show
/// up — exactly the regime the old sim never covered (audit S-09/S-10).
#[derive(Debug, Clone, Copy)]
pub struct CircleHorizontal {
    pub speed_mps: f64,
    pub radius_m: f64,
}

impl TrajectoryGenerator for CircleHorizontal {
    fn sample(&self, t: f64) -> GroundTruth {
        let v = self.speed_mps;
        let r = self.radius_m.max(1e-3);
        let omega = v / r; // yaw rate (rad/s) about NED-down
        let theta = omega * t; // heading from +N, increasing toward +E
        let (s, c) = theta.sin_cos();
        // body→NED attitude: pure yaw of θ about the (down) Z axis. Level.
        let half = (theta * 0.5) as f32;
        GroundTruth {
            t_secs: t,
            pos_n: r * s,
            pos_e: r * (1.0 - c),
            pos_d: 0.0,
            vel_n: v * c,
            vel_e: v * s,
            vel_d: 0.0,
            // Centripetal acceleration (d/dt of velocity), pointing toward the
            // circle centre.
            acc_n: -v * omega * s,
            acc_e: v * omega * c,
            acc_d: 0.0,
            q: [half.cos(), 0.0, 0.0, half.sin()],
            gyro_n: 0.0,
            gyro_e: 0.0,
            gyro_d: omega,
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
