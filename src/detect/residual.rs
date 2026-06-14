//! Ring buffer of recent nav snapshots and the residual computation.

use crate::error::DetectError;
use crate::nav::NavStateSnapshot;
use crate::types::{NedPos, NedVel};
use std::collections::VecDeque;

#[derive(Debug, Clone)]
pub struct ResidualBuffer {
    buf: VecDeque<NavStateSnapshot>,
    capacity: usize,
    window_secs: f64,
    /// How far PAST the newest buffered IMU sample a GPS timestamp may be and
    /// still be served — by forward-extrapolating the newest sample along its
    /// own velocity rather than rejecting the fix. Audit V-SYNC / W1: over a
    /// real MAVLink link the IMU (`SCALED_IMU`) streams at only ~10 Hz, so a
    /// GPS fix routinely lands up to one IMU period (~100 ms) newer than the
    /// freshest buffered sample. The old fixed 20 ms tolerance (sized for a
    /// 50 Hz IMU) rejected nearly every fix as `GpsOutsideBuffer`, so no
    /// residual was ever computed and the detector never fired. Extrapolating
    /// along velocity is the standard slow-aiding-sensor fusion technique and
    /// is accurate over these sub-period horizons; we cap it so a genuine GPS
    /// dropout still errors out instead of dead-reckoning unbounded.
    max_extrapolation_s: f64,
}

/// Default forward-extrapolation horizon: 0.25 s. Covers a ~5 Hz IMU period
/// (200 ms) with margin. Honest 1–10 Hz GPS against a 10 Hz IMU lands well
/// inside this; a multi-second dropout is far outside it and still errors.
pub const DEFAULT_MAX_EXTRAPOLATION_S: f64 = 0.25;

impl ResidualBuffer {
    pub fn new(window_secs: f64, max_rate_hz: f64) -> Self {
        let capacity = (window_secs * max_rate_hz * 1.1).ceil() as usize + 4;
        Self {
            buf: VecDeque::with_capacity(capacity),
            capacity,
            window_secs,
            max_extrapolation_s: DEFAULT_MAX_EXTRAPOLATION_S,
        }
    }

    /// Override the forward-extrapolation horizon (seconds). Builder-style.
    pub fn with_max_extrapolation_s(mut self, secs: f64) -> Self {
        self.max_extrapolation_s = secs;
        self
    }

    pub fn push(&mut self, s: NavStateSnapshot) {
        self.buf.push_back(s);
        // Trim by time window.
        let latest = self.buf.back().map(|x| x.mono_secs).unwrap_or(0.0);
        while let Some(front) = self.buf.front() {
            if latest - front.mono_secs > self.window_secs {
                self.buf.pop_front();
            } else {
                break;
            }
        }
        while self.buf.len() > self.capacity {
            self.buf.pop_front();
        }
    }

    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }

    /// Discard all snapshots. Used after a nav re-anchor so subsequent
    /// `interpolate()` calls don't see pre-reset positions mixed with
    /// post-reset ones.
    pub fn clear(&mut self) {
        self.buf.clear();
    }

    pub fn earliest(&self) -> Option<f64> {
        self.buf.front().map(|s| s.mono_secs)
    }

    pub fn latest(&self) -> Option<f64> {
        self.buf.back().map(|s| s.mono_secs)
    }

    /// Linearly interpolate position+velocity at `t_secs`. Returns
    /// `Err(GpsOutsideBuffer)` if `t_secs` is outside the buffer's span.
    pub fn interpolate(&self, t_secs: f64) -> Result<(NedPos, NedVel), DetectError> {
        if self.buf.is_empty() {
            return Err(DetectError::BufferEmpty);
        }
        let earliest = self.earliest().unwrap();
        let latest = self.latest().unwrap();

        // Bounds check. Past `latest` we allow up to `max_extrapolation_s`
        // (served by velocity-extrapolation below); before `earliest` we keep
        // a tight one-IMU-period tolerance (there's nothing to extrapolate
        // backward from without lookahead, and a GPS fix older than the whole
        // buffer is a genuine sync fault). Audit W1.
        let back_tolerance = 0.02_f64;
        if t_secs < earliest - back_tolerance || t_secs > latest + self.max_extrapolation_s {
            let gap_ms = if t_secs < earliest {
                ((earliest - t_secs) * 1000.0) as i64
            } else {
                ((t_secs - latest) * 1000.0) as i64
            };
            return Err(DetectError::GpsOutsideBuffer { gap_ms });
        }

        // Just past the newest sample: forward-extrapolate along its velocity
        // (constant-velocity model) for the short horizon up to
        // `max_extrapolation_s`. For a fast IMU `dt` is ~0 so this reduces to
        // the old clamp (no behavior change); for a slow (10 Hz) MAVLink IMU
        // it bridges the ~100 ms gap that previously rejected the fix.
        if t_secs >= latest {
            let s = self.buf.back().unwrap();
            let dt = t_secs - latest;
            let pos = NedPos {
                n: s.pos.n + s.vel.n * dt,
                e: s.pos.e + s.vel.e * dt,
                d: s.pos.d + s.vel.d * dt,
            };
            return Ok((pos, s.vel));
        }
        if t_secs <= earliest {
            let s = self.buf.front().unwrap();
            return Ok((s.pos, s.vel));
        }

        // Find bracket.
        let (a, b) = self.bracket(t_secs);
        let span = b.mono_secs - a.mono_secs;
        if span <= 0.0 {
            return Ok((a.pos, a.vel));
        }
        let alpha = ((t_secs - a.mono_secs) / span).clamp(0.0, 1.0);
        let pos = NedPos {
            n: a.pos.n + alpha * (b.pos.n - a.pos.n),
            e: a.pos.e + alpha * (b.pos.e - a.pos.e),
            d: a.pos.d + alpha * (b.pos.d - a.pos.d),
        };
        let vel = NedVel {
            n: a.vel.n + alpha * (b.vel.n - a.vel.n),
            e: a.vel.e + alpha * (b.vel.e - a.vel.e),
            d: a.vel.d + alpha * (b.vel.d - a.vel.d),
        };
        Ok((pos, vel))
    }

    fn bracket(&self, t: f64) -> (&NavStateSnapshot, &NavStateSnapshot) {
        // Linear scan from the back — recent samples first.
        let mut prev = self.buf.front().unwrap();
        for s in self.buf.iter() {
            if s.mono_secs >= t {
                return (prev, s);
            }
            prev = s;
        }
        let last = self.buf.back().unwrap();
        (prev, last)
    }
}

/// 2D residual (Δpos and Δvel in the local NED tangent plane).
///
/// `dvel_known = false` indicates the GPS fix did NOT carry usable
/// speed/course (e.g. NMEA RMC marked invalid, or MAVLink GPS_RAW_INT with
/// no Doppler). In that case `dvel` and `mag_vel` are zero but the
/// velocity-mismatch detector MUST NOT interpret them as evidence — silent
/// zero-defaulting was the root cause of a class of false RTL triggers on
/// real GPS that drops course while still publishing lat/lon.
#[derive(Debug, Clone, Copy)]
pub struct Residual {
    pub dpos: NedPos,
    pub dvel: NedVel,
    pub mag_pos: f64,
    pub mag_vel: f64,
    pub dvel_known: bool,
    /// Velocity residual against the FREE-INERTIAL track: GPS velocity minus the
    /// dead-reckoned velocity with the GPS-velocity blend REMOVED, i.e.
    /// `dvel + nav.aiding_vel()`. Where `dvel` (vs the blended DR velocity) is
    /// driven to ~0 by the complementary blend for a SMART consistent-velocity
    /// spoof, this exposes the velocity bias the blend had to import — the
    /// GPS-velocity-independent signal the velocity-aiding lane fires on. Only
    /// meaningful when `dvel_known` (needs GPS velocity); zero otherwise.
    pub dvel_free: NedVel,
    pub mag_vel_free: f64,
    /// True when the IMU shows the vehicle is maneuvering (turning) around this
    /// fix. The velocity-aiding lane is SUSPENDED while set: a coordinated turn
    /// makes the free-inertial velocity diverge from GPS velocity by ~2 m/s of
    /// legitimate attitude/centripetal error (the blend masks it for position),
    /// which would otherwise read as a consistent-velocity walk-off. Derived from
    /// the gyro, so it is independent of the (GPS-borne) spoof.
    pub maneuvering: bool,
}

impl Default for Residual {
    fn default() -> Self {
        Self {
            dpos: NedPos::default(),
            dvel: NedVel::default(),
            mag_pos: 0.0,
            mag_vel: 0.0,
            dvel_known: true,
            dvel_free: NedVel::default(),
            mag_vel_free: 0.0,
            maneuvering: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn snap(t: f64, n: f64, e: f64) -> NavStateSnapshot {
        NavStateSnapshot {
            mono_secs: t,
            pos: NedPos { n, e, d: 0.0 },
            vel: NedVel::default(),
        }
    }

    fn snap_v(t: f64, n: f64, e: f64, vn: f64, ve: f64) -> NavStateSnapshot {
        NavStateSnapshot {
            mono_secs: t,
            pos: NedPos { n, e, d: 0.0 },
            vel: NedVel {
                n: vn,
                e: ve,
                d: 0.0,
            },
        }
    }

    #[test]
    fn interp_midpoint() {
        let mut b = ResidualBuffer::new(2.0, 100.0);
        b.push(snap(1.0, 0.0, 0.0));
        b.push(snap(2.0, 10.0, 0.0));
        let (p, _) = b.interpolate(1.5).unwrap();
        assert!((p.n - 5.0).abs() < 1e-9);
    }

    #[test]
    fn interp_outside_errors() {
        let mut b = ResidualBuffer::new(2.0, 100.0);
        b.push(snap(1.0, 0.0, 0.0));
        b.push(snap(2.0, 10.0, 0.0));
        // Far before earliest → error. Far past latest (beyond the 0.25s
        // extrapolation horizon) → error.
        assert!(b.interpolate(0.0).is_err());
        assert!(b.interpolate(10.0).is_err());
        // Just before earliest, within the tight back-tolerance → ok (clamp).
        assert!(b.interpolate(0.99).is_ok());
    }

    #[test]
    fn extrapolates_forward_within_horizon() {
        // Audit W1: a slow (10 Hz) IMU means GPS lands ~one period past the
        // newest sample. We must serve it by extrapolating along velocity,
        // not reject it. Newest sample at t=2.0, pos n=10, moving 8 m/s north.
        let mut b = ResidualBuffer::new(2.0, 10.0);
        b.push(snap_v(1.0, 2.0, 0.0, 8.0, 0.0));
        b.push(snap_v(2.0, 10.0, 0.0, 8.0, 0.0));
        // 100 ms past latest: should extrapolate n = 10 + 8*0.1 = 10.8.
        let (p, v) = b
            .interpolate(2.1)
            .expect("100ms past latest must extrapolate, not reject");
        assert!((p.n - 10.8).abs() < 1e-9, "n={}", p.n);
        assert!((v.n - 8.0).abs() < 1e-9, "velocity carried forward");
        // 240 ms past latest: still within 0.25s horizon → n = 10 + 8*0.24.
        let (p2, _) = b.interpolate(2.24).expect("within horizon");
        assert!((p2.n - 11.92).abs() < 1e-9, "n={}", p2.n);
        // 300 ms past latest: beyond horizon → error.
        assert!(
            b.interpolate(2.30).is_err(),
            "beyond horizon must still error"
        );
    }

    #[test]
    fn extrapolation_horizon_is_configurable() {
        let b = ResidualBuffer::new(2.0, 10.0).with_max_extrapolation_s(0.5);
        let mut b = b;
        b.push(snap_v(1.0, 0.0, 0.0, 0.0, 0.0));
        b.push(snap_v(2.0, 0.0, 0.0, 0.0, 0.0));
        // 400 ms past latest: rejected at default 0.25s, allowed at 0.5s.
        assert!(b.interpolate(2.4).is_ok());
        assert!(b.interpolate(2.6).is_err());
    }

    #[test]
    fn buffer_trims_old() {
        let mut b = ResidualBuffer::new(1.0, 100.0);
        for i in 0..300 {
            b.push(snap(i as f64 * 0.01, i as f64, 0.0));
        }
        let span = b.latest().unwrap() - b.earliest().unwrap();
        assert!(span <= 1.0 + 1e-6, "span={}", span);
    }
}
