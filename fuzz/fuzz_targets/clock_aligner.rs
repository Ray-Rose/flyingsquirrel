//! Fuzz `ClockAligner::align` — the surface that already produced a
//! one-packet remote kill of the detector.
//!
//! History: `align` fed an attacker-controlled wire timestamp
//! (`GPS_RAW_INT.time_usec` / `SCALED_IMU.time_boot_ms`) into
//! `anchor_arrival + Duration::from_micros(...)`. A crafted value spanned
//! centuries, overflowed the monotonic `Instant`, and PANICKED — killing the
//! listener task and, with it, the whole detector. That bug was found by
//! adversarial review; this target is the standing regression net for its
//! entire class, driving the REAL implementation (not a copy).
//!
//! Properties asserted:
//!
//! 1. No panic for ANY sequence of (sensor timestamp, arrival) pairs,
//!    including `u64::MAX`, zeros, regressions, and wraps.
//! 2. **The output is bounded**: an aligned stamp never lands more than
//!    `CLOCK_ALIGN_MAX_SKEW` (2 s) ahead of the arrival that produced it.
//!    That bound is what keeps a hostile clock from shifting GPS and IMU
//!    relative to each other — an unbounded lead would let an attacker inject
//!    residual (the detector's core signal) without touching a coordinate.
//!
//! Arrivals are monotonically non-decreasing, matching real message arrival;
//! sensor timestamps are unconstrained, matching an attacker-controlled wire.

#![no_main]

use libfuzzer_sys::fuzz_target;
use std::time::Duration;

/// Mirror of the private `CLOCK_ALIGN_MAX_SKEW` in `mav`. Kept in sync by the
/// assertion below: if the production constant shrinks, this stays a valid
/// (looser) bound; if it grows, the fuzzer reports the divergence rather than
/// silently accepting a weaker guarantee.
const MAX_SKEW: Duration = Duration::from_secs(2);

fuzz_target!(|ops: Vec<(Option<u64>, u32)>| {
    let mut aligner = flyingsquirrel::mav::ClockAligner::default();
    let base = tokio::time::Instant::now();
    let mut arrival_us: u64 = 0;

    for (sensor_us, arrival_advance_us) in ops {
        // Arrival advances monotonically, as real message arrivals do. The
        // sensor timestamp is whatever the wire says — including nothing.
        arrival_us = arrival_us.saturating_add(arrival_advance_us as u64);
        let arrival = base + Duration::from_micros(arrival_us);

        let aligned = aligner.align(sensor_us, arrival);

        assert!(
            aligned.saturating_duration_since(arrival) <= MAX_SKEW,
            "aligned stamp ran {:?} ahead of arrival (bound {:?}); sensor_us={:?}",
            aligned.saturating_duration_since(arrival),
            MAX_SKEW,
            sensor_us
        );
    }
});
