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

    /// Single update step (no maneuver compensation). `dt` in seconds, gyro in
    /// rad/s, accel (specific force) in m/s². Equivalent to
    /// `update_with_lin_accel` with a zero linear-acceleration estimate.
    pub fn update(&mut self, accel: &[f32; 3], gyro: &[f32; 3], dt: f32) {
        self.update_with_lin_accel(accel, gyro, dt, &[0.0; 3]);
    }

    /// Update step with a body-frame linear-acceleration estimate subtracted
    /// from the accelerometer before the gravity-direction correction.
    ///
    /// The accelerometer measures SPECIFIC FORCE `f = a_kinematic − g` (body
    /// frame). The Madgwick accel correction assumes `f ≈ −g` (the field points
    /// opposite gravity at rest), so any sustained kinematic acceleration —
    /// most importantly the centripetal `ω × v` of a coordinated turn — tilts
    /// the estimated "down" and corrupts dead-reckoning (the cause-#3
    /// false-latch). Subtracting a `lin_accel_body` estimate of `a_kinematic`
    /// recovers `−g` for the correction. The GYRO integration is unaffected (it
    /// never touches the accelerometer), so a bad estimate can only weaken the
    /// slow gravity correction — it can never inject motion into the attitude.
    ///
    /// `lin_accel_body` is supplied from the gyro × the DEAD-RECKONED velocity
    /// (see `nav::DeadReckoner::step`), never GPS velocity directly. The DR
    /// velocity is lightly GPS-blended only in NORMAL and freezes fully
    /// GPS-independent in Suspicious/Spoofed (when an attack is being confirmed);
    /// and this term only nudges the slow gravity correction (β-bounded, never
    /// the gyro path), so a spoofed velocity cannot meaningfully bias the
    /// attitude before the position residual it also produces trips detection.
    pub fn update_with_lin_accel(
        &mut self,
        accel: &[f32; 3],
        gyro: &[f32; 3],
        dt: f32,
        lin_accel_body: &[f32; 3],
    ) {
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

        // Remove the caller's body-frame linear-acceleration estimate so the
        // gradient sees gravity alone (centripetal compensation). At rest /
        // straight flight this is ~0 and the behavior is unchanged.
        let acc = [
            accel[0] - lin_accel_body[0],
            accel[1] - lin_accel_body[1],
            accel[2] - lin_accel_body[2],
        ];

        // Accel-based gradient correction (if the compensated accel is
        // reasonable, i.e. non-zero).
        let an = (acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]).sqrt();
        if an > 1e-3 {
            let ax = acc[0] / an;
            let ay = acc[1] / an;
            let az = acc[2] / an;

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

/// Rotate an NED-world vector into the body frame given attitude `q` (Rᵀ·v).
/// Inverse of `rotate_body_to_ned`. Used to express the dead-reckoned velocity
/// in body frame for the centripetal (ω × v) attitude compensation.
pub fn rotate_ned_to_body(q: &Quaternion<f32>, v: &[f32; 3]) -> [f32; 3] {
    let uq = UnitQuaternion::from_quaternion(*q);
    let r = uq.inverse_transform_vector(&Vector3::new(v[0], v[1], v[2]));
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

    #[test]
    fn centripetal_compensation_keeps_turn_attitude_level() {
        // A coordinated level turn (v=8 m/s, r=60 m) produces, in BODY frame, a
        // CONSTANT specific force [0, v·ω, −g] (centripetal on +Y, gravity on
        // −Z) and a constant yaw rate ω about +Z. Without compensation the
        // Madgwick accel correction reads the centripetal as a ~6° tilt and
        // corrupts dead-reckoning (the cause-#3 false-latch); WITH the inertial
        // ω×v compensation ([0, v·ω, 0]) the attitude stays level.
        let v = 8.0_f32;
        let r = 60.0_f32;
        let omega = v / r;
        let cent = v * omega; // ≈ 1.067 m/s²
        let accel = [0.0, cent, -9.81];
        let gyro = [0.0, 0.0, omega];
        let lin = [0.0, cent, 0.0];

        let mut comp = Madgwick::new(0.1);
        comp.init_from_accel([0.0, 0.0, -9.81]); // start level
        let mut raw = Madgwick::new(0.1);
        raw.init_from_accel([0.0, 0.0, -9.81]);
        for _ in 0..2000 {
            comp.update_with_lin_accel(&accel, &gyro, 0.01, &lin);
            raw.update(&accel, &gyro, 0.01);
        }
        let (roll_c, pitch_c, _) = comp.to_euler_rad();
        let (roll_r, _, _) = raw.to_euler_rad();
        assert!(
            roll_c.abs() < 0.01 && pitch_c.abs() < 0.01,
            "compensated turn must stay level: roll={roll_c} pitch={pitch_c}"
        );
        // ~atan(cent/g) ≈ 0.108 rad. Confirms the test actually exercises the bug.
        assert!(
            roll_r.abs() > 0.05,
            "uncompensated turn must tilt (control): roll={roll_r}"
        );
    }

    #[test]
    fn zero_lin_accel_matches_plain_update() {
        // update_with_lin_accel with a zero estimate must be byte-identical to
        // the plain update() — backward-compatibility guard for the delegation.
        let accel = [0.3, -0.2, -9.7];
        let gyro = [0.01, -0.02, 0.05];
        let mut a = Madgwick::new(0.05);
        let mut b = Madgwick::new(0.05);
        for _ in 0..100 {
            a.update(&accel, &gyro, 0.01);
            b.update_with_lin_accel(&accel, &gyro, 0.01, &[0.0; 3]);
        }
        assert_abs_diff_eq!(a.q().w, b.q().w, epsilon = 1e-9);
        assert_abs_diff_eq!(a.q().i, b.q().i, epsilon = 1e-9);
        assert_abs_diff_eq!(a.q().j, b.q().j, epsilon = 1e-9);
        assert_abs_diff_eq!(a.q().k, b.q().k, epsilon = 1e-9);
    }
}
