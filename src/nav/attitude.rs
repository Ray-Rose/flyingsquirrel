//! Madgwick attitude filter — quaternion-form gradient descent.
//!
//! Reference: Madgwick, "An efficient orientation filter for inertial and
//! inertial/magnetic sensor arrays" (2010). IMU-only variant (no magnetometer).
//! Body frame: X-forward, Y-right, Z-down. NED world frame, gravity = +Z.

use nalgebra::{Quaternion, UnitQuaternion, Vector3};

#[derive(Debug, Clone, Copy)]
pub struct Madgwick {
    q: Quaternion<f32>,
    beta: f32,
}

impl Madgwick {
    pub fn new(beta: f32) -> Self {
        Self {
            q: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            beta,
        }
    }

    pub fn q(&self) -> Quaternion<f32> {
        self.q
    }

    /// Initialize roll/pitch from a static accel reading.
    ///
    /// The nav stack uses the physical SPECIFIC-FORCE convention: a level,
    /// stationary accelerometer reads approximately [0, 0, −g] in our
    /// X-forward/Y-right/Z-down body frame (the field points "up", opposite to
    /// gravity). This is what real MEMS IMUs and ArduPilot/PX4
    /// SCALED_IMU/HIGHRES_IMU emit. We negate the reading to recover the gravity
    /// direction the roll/pitch formulas below are written against (so a level
    /// −g reading initializes to level, not 180° roll).
    pub fn init_from_accel(&mut self, accel: [f32; 3]) {
        let ax = -accel[0];
        let ay = -accel[1];
        let az = -accel[2];
        let n = (ax * ax + ay * ay + az * az).sqrt().max(1e-6);
        let axn = ax / n;
        let ayn = ay / n;
        let azn = az / n;
        // roll = atan2(-ay, az) ; pitch = atan2(ax, sqrt(ay²+az²))
        // (NED convention: positive roll = right wing down; positive pitch = nose up)
        let roll = (-ayn).atan2(azn);
        let pitch = axn.atan2((ayn * ayn + azn * azn).sqrt());
        let yaw: f32 = 0.0;
        let half_r = roll * 0.5;
        let half_p = pitch * 0.5;
        let half_y = yaw * 0.5;
        let cr = half_r.cos();
        let sr = half_r.sin();
        let cp = half_p.cos();
        let sp = half_p.sin();
        let cy = half_y.cos();
        let sy = half_y.sin();
        self.q = Quaternion::new(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        );
        self.normalize();
    }

    pub fn seed_yaw_deg(&mut self, yaw_deg: f32) {
        // Extract roll/pitch from current q, rebuild with given yaw.
        let (roll, pitch, _) = self.to_euler_rad();
        let yaw = yaw_deg.to_radians();
        let half_r = roll * 0.5;
        let half_p = pitch * 0.5;
        let half_y = yaw * 0.5;
        let cr = half_r.cos();
        let sr = half_r.sin();
        let cp = half_p.cos();
        let sp = half_p.sin();
        let cy = half_y.cos();
        let sy = half_y.sin();
        self.q = Quaternion::new(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        );
        self.normalize();
    }

    /// Returns (roll, pitch, yaw) in radians.
    pub fn to_euler_rad(&self) -> (f32, f32, f32) {
        let q = self.q;
        let w = q.w;
        let x = q.i;
        let y = q.j;
        let z = q.k;
        let sinr_cosp = 2.0 * (w * x + y * z);
        let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        let roll = sinr_cosp.atan2(cosr_cosp);
        let sinp = 2.0 * (w * y - z * x);
        let pitch = if sinp.abs() >= 1.0 {
            sinp.signum() * std::f32::consts::FRAC_PI_2
        } else {
            sinp.asin()
        };
        let siny_cosp = 2.0 * (w * z + x * y);
        let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        let yaw = siny_cosp.atan2(cosy_cosp);
        (roll, pitch, yaw)
    }

    /// Single update step. `dt` in seconds, gyro in rad/s, accel in m/s².
    pub fn update(&mut self, accel: &[f32; 3], gyro: &[f32; 3], dt: f32) {
        let q = self.q;
        let w = q.w;
        let x = q.i;
        let y = q.j;
        let z = q.k;

        // Gyro quaternion derivative: q_dot_gyro = 0.5 * q ⊗ [0, gx, gy, gz]
        let gx = gyro[0];
        let gy = gyro[1];
        let gz = gyro[2];
        let mut qdot_w = 0.5 * (-x * gx - y * gy - z * gz);
        let mut qdot_x = 0.5 * (w * gx + y * gz - z * gy);
        let mut qdot_y = 0.5 * (w * gy - x * gz + z * gx);
        let mut qdot_z = 0.5 * (w * gz + x * gy - y * gx);

        // Accel-based gradient correction (if accel is reasonable, i.e. non-zero).
        let an = (accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]).sqrt();
        if an > 1e-3 {
            let ax = accel[0] / an;
            let ay = accel[1] / an;
            let az = accel[2] / an;

            // Body-frame direction of NED "down" predicted from current q:
            // R(q)ᵀ · [0,0,1].
            let gx_pred = 2.0 * (x * z - w * y);
            let gy_pred = 2.0 * (w * x + y * z);
            let gz_pred = 1.0 - 2.0 * (x * x + y * y);
            // SPECIFIC-FORCE convention: at rest the (normalized) accelerometer
            // reads [0,0,−1] (the field points opposite gravity). The predicted
            // specific-force direction is therefore −R(q)ᵀ·[0,0,1] = −g_pred, and
            // the error e = predicted − measured = −g_pred − a. The gradient
            // ∇F = Jᵀe then equals Jᵀ(g_pred + a) using the SAME Jacobian terms
            // below (the two sign flips cancel), so this differs from the
            // gravity-reaction form only by `−a → +a`. At rest e = 0 (no
            // correction), as required.
            let fx = gx_pred + ax;
            let fy = gy_pred + ay;
            let fz = gz_pred + az;

            // Jacobian J of f wrt q (4x3 transpose).
            let jw = 2.0 * (-y * fx + x * fy + 0.0 * fz);
            let jx = 2.0 * (z * fx + w * fy - 2.0 * x * fz);
            let jy = 2.0 * (-w * fx + z * fy - 2.0 * y * fz);
            let jz = 2.0 * (x * fx + y * fy + 0.0 * fz);

            // Normalize gradient.
            let gn = (jw * jw + jx * jx + jy * jy + jz * jz).sqrt().max(1e-9);
            let beta = self.beta;
            qdot_w -= beta * jw / gn;
            qdot_x -= beta * jx / gn;
            qdot_y -= beta * jy / gn;
            qdot_z -= beta * jz / gn;
        }

        let new = Quaternion::new(
            w + qdot_w * dt,
            x + qdot_x * dt,
            y + qdot_y * dt,
            z + qdot_z * dt,
        );
        self.q = new;
        self.normalize();
    }

    fn normalize(&mut self) {
        let n =
            (self.q.w * self.q.w + self.q.i * self.q.i + self.q.j * self.q.j + self.q.k * self.q.k)
                .sqrt()
                .max(1e-9);
        self.q = Quaternion::new(self.q.w / n, self.q.i / n, self.q.j / n, self.q.k / n);
    }
}

/// Rotate a body-frame vector into the NED world frame given attitude quaternion `q`.
pub fn rotate_body_to_ned(q: &Quaternion<f32>, v: &[f32; 3]) -> [f32; 3] {
    let uq = UnitQuaternion::from_quaternion(*q);
    let r = uq.transform_vector(&Vector3::new(v[0], v[1], v[2]));
    [r.x, r.y, r.z]
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn identity_rotation_passes_vector_through() {
        let m = Madgwick::new(0.05);
        // Pure rotation check: identity attitude rotates any body vector
        // unchanged into NED. (At-rest specific force would be [0,0,-g].)
        let v = rotate_body_to_ned(&m.q(), &[0.0, 0.0, -9.81]);
        assert_abs_diff_eq!(v[0], 0.0, epsilon = 1e-5);
        assert_abs_diff_eq!(v[1], 0.0, epsilon = 1e-5);
        assert_abs_diff_eq!(v[2], -9.81, epsilon = 1e-5);
    }

    #[test]
    fn init_from_specific_force_at_rest_is_level() {
        let mut m = Madgwick::new(0.05);
        // Real IMU at rest, level: specific force reads [0,0,-g] (ArduPilot/PX4
        // SCALED_IMU/HIGHRES_IMU and any MEMS accelerometer). This MUST init to
        // level — the pre-fix bug initialized at ~180° roll, mirroring DR.
        m.init_from_accel([0.0, 0.0, -9.81]);
        let (roll, pitch, _) = m.to_euler_rad();
        assert!(roll.abs() < 1e-3, "roll={} (expected level)", roll);
        assert!(pitch.abs() < 1e-3, "pitch={} (expected level)", pitch);
        // The NED gravity direction [0,0,+g] maps back to body [0,0,+g] at level.
        let v = rotate_body_to_ned(&m.q(), &[0.0, 0.0, 9.81]);
        assert_abs_diff_eq!(v[2], 9.81, epsilon = 1e-3);
        assert!(v[0].abs() < 1e-3);
        assert!(v[1].abs() < 1e-3);
    }

    #[test]
    fn update_holds_level_under_rest_specific_force() {
        // Feeding the at-rest specific force [0,0,-g] to update() must produce
        // ZERO attitude correction (error term is exactly zero at rest in the
        // specific-force convention). A wrong sign would drive the filter away
        // from level.
        let mut m = Madgwick::new(0.5);
        for _ in 0..200 {
            m.update(&[0.0, 0.0, -9.81], &[0.0, 0.0, 0.0], 0.01);
        }
        let (roll, pitch, _) = m.to_euler_rad();
        assert!(roll.abs() < 1e-2, "roll drifted to {}", roll);
        assert!(pitch.abs() < 1e-2, "pitch drifted to {}", pitch);
    }

    #[test]
    fn yaw_seed_preserves_attitude_magnitude() {
        let mut m = Madgwick::new(0.05);
        m.init_from_accel([0.0, 0.0, -9.81]); // at-rest specific force → level
        m.seed_yaw_deg(45.0);
        let n = (m.q().w.powi(2) + m.q().i.powi(2) + m.q().j.powi(2) + m.q().k.powi(2)).sqrt();
        assert_abs_diff_eq!(n, 1.0, epsilon = 1e-5);
        let (_, _, yaw) = m.to_euler_rad();
        assert!(
            (yaw.to_degrees() - 45.0).abs() < 0.5,
            "yaw={}",
            yaw.to_degrees()
        );
    }
}
