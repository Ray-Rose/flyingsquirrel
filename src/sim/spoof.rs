//! Spoof injector — wraps a `GpsSource` and overlays attack patterns on the
//! reported lat/lon (and optionally velocity).

use crate::error::FsError;
use crate::ingest::{BoxStream, GpsSource};
use crate::nav::{lla_to_ned, R_EARTH_M};
use crate::types::GpsFix;
use async_stream::stream;
use futures::StreamExt;
use std::sync::Arc;
use tokio::sync::Mutex;

#[derive(Debug, Clone, Copy)]
pub enum SpoofPattern {
    /// Pass-through. No attack.
    Clean,
    /// Sudden jump of `jump_m` to the East at `apply_at_s`.
    SuddenJump {
        apply_at_s: f32,
        jump_north_m: f32,
        jump_east_m: f32,
    },
    /// Gradual drift starting at `apply_at_s`. North/east components are in m/s.
    GradualDrift {
        apply_at_s: f32,
        drift_north_mps: f32,
        drift_east_mps: f32,
    },
    /// Stuck/replayed GPS — after `freeze_at_s`, every reported fix has the
    /// SAME lat/lon as whatever the inner source reported at `freeze_at_s`.
    /// Simulates a frozen GPS module or an attacker replaying a captured
    /// fix. The `FrozenGps` detector should fire after a few consecutive
    /// identical fixes while the IMU shows motion.
    Frozen { freeze_at_s: f32 },
    /// Velocity/position inconsistency: after `apply_at_s`, ADD
    /// `extra_speed_mps` to the reported GPS Doppler speed while leaving
    /// lat/lon honest. This produces a fix whose reported velocity disagrees
    /// with the IMU-integrated velocity by `extra_speed_mps` WITHOUT moving
    /// the position — isolating the velocity-mismatch detector (which the
    /// position-only SuddenJump/GradualDrift patterns never exercise).
    /// Audit S-02/S-03: the velocity-mismatch path had zero end-to-end test
    /// coverage before this variant existed.
    VelocityInconsistent {
        apply_at_s: f32,
        extra_speed_mps: f32,
    },
    /// Sudden altitude teleport: at `apply_at_s`, ADD `jump_m` to the
    /// reported altitude (and hold it) while leaving lat/lon and velocity
    /// honest. Exercises the vertical-rate spoof check (audit B-42), which
    /// the horizontal-only patterns never trigger.
    AltitudeJump { apply_at_s: f32, jump_m: f32 },
}

pub struct SpoofInjector<G: GpsSource> {
    inner: G,
    pattern: SpoofPattern,
    /// Wall-clock anchor — taken on first fix.
    start: Arc<Mutex<Option<tokio::time::Instant>>>,
    /// For `Frozen` pattern: the first lat/lon observed at-or-after
    /// `freeze_at_s`. Subsequent fixes are overwritten with this value.
    frozen_at: Arc<Mutex<Option<(f64, f64)>>>,
}

impl<G: GpsSource> SpoofInjector<G> {
    pub fn new(inner: G, pattern: SpoofPattern) -> Self {
        Self {
            inner,
            pattern,
            start: Arc::new(Mutex::new(None)),
            frozen_at: Arc::new(Mutex::new(None)),
        }
    }
}

impl<G: GpsSource> GpsSource for SpoofInjector<G> {
    fn into_stream(self: Box<Self>) -> BoxStream<'static, Result<GpsFix, FsError>> {
        let SpoofInjector {
            inner,
            pattern,
            start,
            frozen_at,
        } = *self;
        let inner_stream = Box::new(inner).into_stream();
        let stream = stream! {
            futures::pin_mut!(inner_stream);
            while let Some(item) = inner_stream.next().await {
                match item {
                    Ok(mut fix) => {
                        let now = tokio::time::Instant::now();
                        let mut s = start.lock().await;
                        let anchor = *s.get_or_insert(now);
                        let elapsed = now.saturating_duration_since(anchor).as_secs_f64();
                        drop(s);
                        // Frozen pattern is stateful — needs to capture the
                        // first post-freeze fix and replay it. Other patterns
                        // are stateless and handled in apply_spoof.
                        if let SpoofPattern::Frozen { freeze_at_s } = pattern {
                            if elapsed >= freeze_at_s as f64 {
                                let mut fz = frozen_at.lock().await;
                                let pinned = *fz.get_or_insert((fix.lat_deg, fix.lon_deg));
                                drop(fz);
                                fix.lat_deg = pinned.0;
                                fix.lon_deg = pinned.1;
                                // Critical: leave speed/course unmodified so the
                                // vehicle's reported velocity still indicates motion.
                                // That's what the FrozenGps detector compares against.
                            }
                        } else {
                            apply_spoof(&mut fix, &pattern, elapsed);
                        }
                        yield Ok(fix);
                    }
                    Err(e) => yield Err(e),
                }
            }
        };
        Box::pin(stream)
    }
}

fn apply_spoof(fix: &mut GpsFix, pattern: &SpoofPattern, elapsed_s: f64) {
    // Velocity-inconsistency is handled separately because it perturbs the
    // reported Doppler speed, not position. Done BEFORE the position early-return.
    if let SpoofPattern::VelocityInconsistent {
        apply_at_s,
        extra_speed_mps,
    } = pattern
    {
        if elapsed_s >= *apply_at_s as f64 {
            // Add the extra speed to whatever the inner source reported.
            // Leave course unchanged and position untouched.
            let base = fix.speed_mps.unwrap_or(0.0);
            fix.speed_mps = Some(base + *extra_speed_mps as f64);
        }
        return;
    }
    // Altitude teleport — perturbs alt only, leaving lat/lon/velocity honest.
    if let SpoofPattern::AltitudeJump { apply_at_s, jump_m } = pattern {
        if elapsed_s >= *apply_at_s as f64 {
            fix.alt_m += *jump_m as f64;
        }
        return;
    }

    let (dn, de) = match pattern {
        SpoofPattern::Clean => (0.0, 0.0),
        SpoofPattern::SuddenJump {
            apply_at_s,
            jump_north_m,
            jump_east_m,
        } => {
            if elapsed_s >= *apply_at_s as f64 {
                (*jump_north_m as f64, *jump_east_m as f64)
            } else {
                (0.0, 0.0)
            }
        }
        SpoofPattern::GradualDrift {
            apply_at_s,
            drift_north_mps,
            drift_east_mps,
        } => {
            let t = (elapsed_s - *apply_at_s as f64).max(0.0);
            (t * *drift_north_mps as f64, t * *drift_east_mps as f64)
        }
        SpoofPattern::Frozen { .. } => {
            // Handled stateful at the stream level — see into_stream.
            (0.0, 0.0)
        }
        SpoofPattern::VelocityInconsistent { .. } | SpoofPattern::AltitudeJump { .. } => {
            unreachable!("handled above")
        }
    };
    if dn == 0.0 && de == 0.0 {
        return;
    }
    // Translate (dn, de) into lat/lon delta, equirectangular.
    let lat0 = fix.lat_deg.to_radians();
    let dlat_deg = (dn / R_EARTH_M).to_degrees();
    let dlon_deg = (de / (R_EARTH_M * lat0.cos())).to_degrees();
    fix.lat_deg += dlat_deg;
    fix.lon_deg += dlon_deg;
    // Note: we don't tamper with reported velocity. A naive spoofer doesn't either —
    // and that's part of what gives the detector a chance: integrated drift in position
    // diverges from velocity-implied displacement.
    let _ = lla_to_ned; // silence unused-imports if optimized away
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clean_is_identity() {
        let mut fix = GpsFix {
            t: crate::types::Timestamp::now_mono(),
            lat_deg: 40.0,
            lon_deg: -100.0,
            alt_m: 0.0,
            speed_mps: Some(0.0),
            course_deg: Some(0.0),
            hdop: None,
            sats: None,
        };
        apply_spoof(&mut fix, &SpoofPattern::Clean, 100.0);
        assert!((fix.lat_deg - 40.0).abs() < 1e-12);
    }

    #[test]
    fn gradual_drift_accumulates() {
        let mut fix = GpsFix {
            t: crate::types::Timestamp::now_mono(),
            lat_deg: 40.0,
            lon_deg: -100.0,
            alt_m: 0.0,
            speed_mps: Some(0.0),
            course_deg: Some(0.0),
            hdop: None,
            sats: None,
        };
        apply_spoof(
            &mut fix,
            &SpoofPattern::GradualDrift {
                apply_at_s: 0.0,
                drift_north_mps: 1.0,
                drift_east_mps: 0.0,
            },
            100.0,
        );
        // 100 m north corresponds to ~100/R_EARTH radians latitude.
        let dlat_deg_expected = (100.0_f64 / R_EARTH_M).to_degrees();
        assert!((fix.lat_deg - (40.0 + dlat_deg_expected)).abs() < 1e-9);
    }
}
