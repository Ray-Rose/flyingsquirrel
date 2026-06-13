//! Sensor-to-body axis remap for arbitrarily-mounted IMUs.
//!
//! A raw IMU reports in its OWN chip frame. The nav stack assumes the vehicle
//! body frame is FRD — X-forward, Y-right, Z-down — with the specific-force
//! convention. When the chip is mounted in any orientation other than
//! "X-forward, Y-right, Z-down," its axes must be remapped to the body frame
//! at ingest, or dead-reckoning integrates a rotated signal (the I2C driver
//! previously passed raw chip axes straight through — see docs/threats.md N-01).
//!
//! Representation: a signed axis permutation — for each body axis it records
//! which source (chip) axis feeds it and with what sign. This covers every
//! physical box mounting (the 24 proper-rotation orientations). The SAME map is
//! applied to both the accelerometer and the gyroscope because a proper
//! rotation transforms vectors and pseudovectors identically.
//!
//! We REJECT maps that are not proper rotations (determinant ≠ +1 — i.e.
//! reflections). A physical sensor can only be rotated into place, never
//! mirrored; a reflection would flip the gyro's sign relative to the accel and
//! silently corrupt dead-reckoning during turns. This module is intentionally
//! NOT feature-gated so the remap math is unit-tested on every platform, even
//! though it is only applied on the Linux I2C path today.

/// A signed axis permutation mapping a sensor's chip frame to the FRD body
/// frame. Construct via [`AxisMap::parse`] or use [`AxisMap::IDENTITY`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisMap {
    /// `map[i] = (source_axis_index, sign)` for body axis `i` (0=X, 1=Y, 2=Z).
    map: [(usize, f32); 3],
}

impl Default for AxisMap {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl AxisMap {
    /// Identity: body axes == chip axes (no remap). Matches the prior
    /// pass-through behavior, so an unset map changes nothing.
    pub const IDENTITY: Self = Self {
        map: [(0, 1.0), (1, 1.0), (2, 1.0)],
    };

    /// Remap a body-frame-bound 3-vector (accelerometer or gyroscope sample).
    pub fn apply(&self, v: [f32; 3]) -> [f32; 3] {
        [
            self.map[0].1 * v[self.map[0].0],
            self.map[1].1 * v[self.map[1].0],
            self.map[2].1 * v[self.map[2].0],
        ]
    }

    /// True if this map is a no-op.
    pub fn is_identity(&self) -> bool {
        *self == Self::IDENTITY
    }

    /// Parse a compact spec: three comma-separated signed axis tokens, where
    /// each token names the CHIP axis that feeds the corresponding BODY axis.
    /// e.g. `"Y,-X,Z"` → body X = +chip Y, body Y = -chip X, body Z = +chip Z
    /// (a 90° yaw mount). Case-insensitive; a leading `+` is optional. The
    /// result is validated as a proper rotation (see [`AxisMap::validate`]).
    pub fn parse(s: &str) -> Result<Self, String> {
        let tokens: Vec<&str> = s
            .split(',')
            .map(str::trim)
            .filter(|t| !t.is_empty())
            .collect();
        if tokens.len() != 3 {
            return Err(format!(
                "axis map must be 3 comma-separated axes like \"Y,-X,Z\" (got {s:?})"
            ));
        }
        let mut map = [(0usize, 1.0f32); 3];
        for (i, tok) in tokens.iter().enumerate() {
            let (sign, letter) = if let Some(rest) = tok.strip_prefix('-') {
                (-1.0, rest)
            } else {
                (1.0, tok.strip_prefix('+').unwrap_or(tok))
            };
            let src = match letter.to_ascii_uppercase().as_str() {
                "X" => 0,
                "Y" => 1,
                "Z" => 2,
                other => {
                    return Err(format!(
                        "axis token {other:?} must be X, Y, or Z (optionally signed, e.g. -Y)"
                    ))
                }
            };
            map[i] = (src, sign);
        }
        let m = Self { map };
        m.validate()?;
        Ok(m)
    }

    /// Reject anything that is not a proper rotation of the axes: each chip
    /// axis must appear exactly once (a permutation) AND the signed
    /// permutation's determinant must be +1. A determinant of -1 is a
    /// reflection — physically unrealizable for a mounted sensor and would
    /// chirality-flip the gyro relative to the accel.
    pub fn validate(&self) -> Result<(), String> {
        let mut seen = [false; 3];
        for (src, _) in self.map {
            if src > 2 {
                return Err(format!(
                    "axis source index {src} out of range (must be 0..=2)"
                ));
            }
            if seen[src] {
                return Err(format!(
                    "axis map reuses chip axis {}; each of X/Y/Z must appear exactly once",
                    ["X", "Y", "Z"][src]
                ));
            }
            seen[src] = true;
        }
        let det = self.determinant();
        if (det - 1.0).abs() > 1e-6 {
            return Err(format!(
                "axis map determinant is {det:+.0} but must be +1: a physical sensor can only be \
                 ROTATED into place, not mirrored. A reflection would flip the gyroscope's sign \
                 relative to the accelerometer and corrupt dead-reckoning during turns. Flip \
                 exactly one axis sign to make it a proper rotation."
            ));
        }
        Ok(())
    }

    fn determinant(&self) -> f32 {
        let mut m = [[0.0f32; 3]; 3];
        for (i, (src, sign)) in self.map.into_iter().enumerate() {
            m[i][src] = sign;
        }
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_is_passthrough() {
        let m = AxisMap::IDENTITY;
        assert!(m.is_identity());
        assert_eq!(m.apply([1.0, 2.0, 3.0]), [1.0, 2.0, 3.0]);
        assert_eq!(AxisMap::parse("X,Y,Z").unwrap(), AxisMap::IDENTITY);
        assert!(AxisMap::default().is_identity());
    }

    #[test]
    fn yaw_90_rotation_parses_and_applies() {
        // 90° yaw: body X = +chip Y, body Y = -chip X, body Z = +chip Z.
        let m = AxisMap::parse("Y,-X,Z").unwrap();
        assert!(!m.is_identity());
        assert_eq!(m.apply([1.0, 2.0, 3.0]), [2.0, -1.0, 3.0]);
    }

    #[test]
    fn yaw_180_and_cyclic_are_proper_rotations() {
        assert_eq!(
            AxisMap::parse("-X,-Y,Z").unwrap().apply([1.0, 2.0, 3.0]),
            [-1.0, -2.0, 3.0]
        );
        // Cyclic X->Y->Z->X is an even permutation (det +1).
        assert_eq!(
            AxisMap::parse("Y,Z,X").unwrap().apply([1.0, 2.0, 3.0]),
            [2.0, 3.0, 1.0]
        );
    }

    #[test]
    fn reflections_are_rejected() {
        // Flipping a single axis is a reflection (det -1).
        assert!(AxisMap::parse("X,Y,-Z").is_err());
        assert!(AxisMap::parse("-X,Y,Z").is_err());
        // A bare axis swap (odd permutation, no sign flip) is also det -1.
        assert!(AxisMap::parse("Y,X,Z").is_err());
    }

    #[test]
    fn non_permutations_and_bad_tokens_rejected() {
        assert!(AxisMap::parse("X,X,Z").is_err()); // reuse
        assert!(AxisMap::parse("X,Q,Z").is_err()); // bad letter
        assert!(AxisMap::parse("X,Y").is_err()); // too few
        assert!(AxisMap::parse("X,Y,Z,W").is_err()); // too many
    }

    #[test]
    fn whitespace_case_and_plus_tolerated() {
        assert_eq!(
            AxisMap::parse(" +y , -x , +z ").unwrap(),
            AxisMap::parse("Y,-X,Z").unwrap()
        );
    }

    #[test]
    fn all_documented_valid_maps_have_determinant_plus_one() {
        for s in ["X,Y,Z", "Y,-X,Z", "-X,-Y,Z", "Y,Z,X", "Z,X,Y", "-Y,X,Z"] {
            let m = AxisMap::parse(s).expect("should parse");
            assert!(
                (m.determinant() - 1.0).abs() < 1e-6,
                "{s} should be a proper rotation"
            );
        }
    }

    #[test]
    fn gyro_and_accel_use_the_same_map() {
        // A proper rotation applies identically to a vector and a pseudovector.
        let m = AxisMap::parse("Y,-X,Z").unwrap();
        let accel = [0.1, 0.2, 9.8];
        let gyro = [0.01, -0.02, 0.03];
        assert_eq!(m.apply(accel), [0.2, -0.1, 9.8]);
        assert_eq!(m.apply(gyro), [-0.02, -0.01, 0.03]);
    }
}
