//! Strapdown integration: bias estimation, velocity, position.
//!
//! Pure math — `Strapdown` integrates linear NED acceleration with
//! trapezoidal rule. `BiasEstimator` maintains an EMA of accel + gyro bias
//! during low-motion windows.

use crate::types::{NedPos, NedVel};

#[derive(Debug, Clone, Copy, Default)]
pub struct Strapdown {
    pos: NedPos,
    vel: NedVel,
    last_a: Option<[f64; 3]>,
}

impl Strapdown {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn pos(&self) -> NedPos {
        self.pos
    }

    pub fn vel(&self) -> NedVel {
        self.vel
    }

    pub fn set_pos(&mut self, p: NedPos) {
        self.pos = p;
    }

    pub fn set_vel(&mut self, v: NedVel) {
        self.vel = v;
    }

    pub fn reset(&mut self) {
        self.pos = NedPos::default();
        self.vel = NedVel::default();
        self.last_a = None;
    }

    /// Trapezoidal integration of linear NED acceleration into velocity and position.
    pub fn integrate(&mut self, a_ned: &[f64; 3], dt: f32) {
        let dt = dt as f64;
        let a_prev = self.last_a.unwrap_or(*a_ned);
        let ax = 0.5 * (a_prev[0] + a_ned[0]);
        let ay = 0.5 * (a_prev[1] + a_ned[1]);
        let az = 0.5 * (a_prev[2] + a_ned[2]);

        let v0 = self.vel;
        self.vel = NedVel {
            n: v0.n + ax * dt,
            e: v0.e + ay * dt,
            d: v0.d + az * dt,
        };

        self.pos = NedPos {
            n: self.pos.n + 0.5 * (v0.n + self.vel.n) * dt,
            e: self.pos.e + 0.5 * (v0.e + self.vel.e) * dt,
            d: self.pos.d + 0.5 * (v0.d + self.vel.d) * dt,
        };

        self.last_a = Some(*a_ned);
    }
}

/// Estimate of body-frame gyro and NED-frame linear-accel biases via EMA
/// during quiescent windows.
///
/// Why two separate biases:
/// - Gyro bias is body-frame (sensor offset, independent of attitude).
/// - Accel bias is fundamentally body-frame too, but the practical effect
///   of un-modeled accel offset shows up as a non-zero `a_lin` in NED after
///   gravity compensation. We absorb that residual directly in NED — cheaper
///   than maintaining an attitude-dependent body-frame model, and covers
///   the dominant failure mode (vertical bias coupling through gravity).
///   Trade-off: NED-frame absorption couples to attitude changes; we limit
///   the damage by only updating during quiet windows.
#[derive(Debug, Clone, Copy)]
pub struct BiasEstimator {
    alpha: f32,
    accel_ned: [f64; 3],
    gyro: [f32; 3],
    low_motion_secs: f32,
    window_target_s: f32,
    g_norm: f32,
}

impl BiasEstimator {
    pub fn new(alpha: f32, window_target_s: f32) -> Self {
        Self {
            alpha,
            accel_ned: [0.0; 3],
            gyro: [0.0; 3],
            low_motion_secs: 0.0,
            window_target_s,
            g_norm: 9.806_65,
        }
    }

    /// Apply NED-accel bias correction and (when quiet AND allowed) update
    /// the EMA toward absorbing any residual gravity-comp error. The caller
    /// must pass the SAME `a_lin` it intends to integrate; the returned value
    /// is the bias-corrected version to use for integration.
    ///
    /// Quiet detection uses the body-frame accel magnitude (already gated by
    /// `observe()`); this method assumes the caller has set up that gate
    /// via a recent `observe()` call.
    pub fn correct_and_update_accel_ned(
        &mut self,
        a_lin: [f64; 3],
        allow_update: bool,
    ) -> [f64; 3] {
        let corrected = [
            a_lin[0] - self.accel_ned[0],
            a_lin[1] - self.accel_ned[1],
            a_lin[2] - self.accel_ned[2],
        ];
        // EMA toward absorbing the residual: in quiet, true a_lin should be 0,
        // so the corrected residual IS the new bias signal.
        if allow_update && self.low_motion_secs >= self.window_target_s {
            let a = self.alpha as f64;
            self.accel_ned[0] =
                (1.0 - a) * self.accel_ned[0] + a * (self.accel_ned[0] + corrected[0]);
            self.accel_ned[1] =
                (1.0 - a) * self.accel_ned[1] + a * (self.accel_ned[1] + corrected[1]);
            self.accel_ned[2] =
                (1.0 - a) * self.accel_ned[2] + a * (self.accel_ned[2] + corrected[2]);
        }
        corrected
    }

    pub fn gyro(&self) -> [f32; 3] {
        self.gyro
    }

    /// Update gates: low motion (gyro & accel norm) AND `allow`.
    pub fn observe(&mut self, accel: &[f32; 3], gyro: &[f32; 3], dt: f32, allow: bool) {
        let gn = (gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]).sqrt();
        let an = (accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]).sqrt();

        let quiet = gn < 0.05 && (an - self.g_norm).abs() < 0.3;
        if quiet && allow {
            self.low_motion_secs += dt;
        } else {
            self.low_motion_secs = 0.0;
            return;
        }

        if self.low_motion_secs >= self.window_target_s {
            // Gyro bias: average towards measured gyro (which at rest should be 0).
            for (k, g) in gyro.iter().enumerate() {
                self.gyro[k] = (1.0 - self.alpha) * self.gyro[k] + self.alpha * g;
            }
            // Note: accel bias is NOT estimated here in v1 — the body-frame accel at rest
            // is dominated by gravity, and decoupling bias from gravity requires a known
            // attitude, which itself depends on bias. We rely on the GPS blend during NORMAL
            // to absorb low-frequency accel error instead. Field left as a hook for v2.
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn constant_accel_matches_kinematics() {
        let mut s = Strapdown::new();
        let dt = 0.01_f32;
        let a = [1.0_f64, 0.0, 0.0];
        // Trapezoidal integration of a constant: v should be exact, p should match exactly
        // because the average of two equal endpoints is the constant itself.
        // BUT the first step uses last_a = a_ned (the current), so initial v_step is exact too.
        for _ in 0..100 {
            s.integrate(&a, dt);
        }
        let t = 1.0;
        // dt is f32; 100 * 0.01_f32 accumulates ~2e-8 IEEE error. 1e-5 is plenty tight.
        assert_abs_diff_eq!(s.vel().n, a[0] * t, epsilon = 1e-5);
        let expected_p = 0.5 * a[0] * t * t;
        assert!(
            (s.pos().n - expected_p).abs() < 0.05,
            "got p={}, expected ~{}",
            s.pos().n,
            expected_p
        );
    }

    #[test]
    fn zero_accel_keeps_state() {
        let mut s = Strapdown::new();
        for _ in 0..1000 {
            s.integrate(&[0.0; 3], 0.01);
        }
        assert_abs_diff_eq!(s.pos().n, 0.0, epsilon = 1e-9);
        assert_abs_diff_eq!(s.vel().n, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn bias_estimator_learns_gyro_during_quiet() {
        let mut b = BiasEstimator::new(0.1, 0.5);
        // 10s of quiet samples at 100Hz, raw gyro reads 0.01 rad/s on all axes.
        // Accel is at-rest specific force [0,0,-g] (quiet gate keys off magnitude).
        for _ in 0..1000 {
            b.observe(&[0.0, 0.0, -9.81], &[0.01, 0.01, 0.01], 0.01, true);
        }
        for k in 0..3 {
            assert!(
                b.gyro()[k] > 0.005 && b.gyro()[k] < 0.011,
                "gyro[{}]={}",
                k,
                b.gyro()[k]
            );
        }
    }

    #[test]
    fn bias_estimator_skips_when_disallowed() {
        let mut b = BiasEstimator::new(0.5, 0.1);
        for _ in 0..1000 {
            b.observe(&[0.0, 0.0, -9.81], &[1.0, 0.0, 0.0], 0.01, false);
        }
        assert_abs_diff_eq!(b.gyro()[0], 0.0, epsilon = 1e-9);
    }
}
