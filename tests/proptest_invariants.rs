//! Property-based invariants over the public API.
//!
//! `proptest` is declared in `[dev-dependencies]` precisely so the
//! adversarial-input surfaces this project cares about (ingest plausibility
//! gates, the PX4 mode codec, the tangent-plane projection) are checked across
//! a wide random input space, not just hand-picked examples. Each property
//! below is a structural invariant that must hold for ALL inputs in range.

use flyingsquirrel::mav::monitor::{is_rtl_mode, px4_encode_mode, px4_split_mode, VehicleProfile};
use flyingsquirrel::mav::{gps_fix_is_plausible, ClockAligner};
use flyingsquirrel::nav::{lla_to_ned, R_EARTH_M};
use flyingsquirrel::sim::ned_to_lla;
use flyingsquirrel::types::{GpsFix, Timestamp};
use proptest::prelude::*;

fn fix(lat: f64, lon: f64, alt: f64) -> GpsFix {
    GpsFix {
        t: Timestamp::now_mono(),
        lat_deg: lat,
        lon_deg: lon,
        alt_m: alt,
        speed_mps: Some(0.0),
        course_deg: Some(0.0),
        hdop: Some(1.0),
        sats: Some(10),
    }
}

proptest! {
    /// The PX4 custom_mode codec must round-trip for every (main, sub) byte
    /// pair — a mismatch here silently mis-classifies the autopilot's mode in
    /// RTL verification.
    #[test]
    fn px4_mode_encode_split_roundtrips(main in any::<u8>(), sub in any::<u8>()) {
        let (m2, s2) = px4_split_mode(px4_encode_mode(main, sub));
        prop_assert_eq!(m2, main);
        prop_assert_eq!(s2, sub);
    }

    /// `is_rtl_mode` for PX4 must be exactly "main==AUTO(4) AND sub in {5,6,7}",
    /// derived independently here from the same encoder. Guards against the
    /// flat-vs-packed confusion the codec exists to prevent.
    #[test]
    fn px4_is_rtl_matches_definition(main in any::<u8>(), sub in any::<u8>()) {
        let cm = px4_encode_mode(main, sub);
        let expect = main == 4 && (sub == 5 || sub == 6 || sub == 7);
        prop_assert_eq!(is_rtl_mode(VehicleProfile::Px4, cm), expect);
    }

    /// ArduCopter mode ids are flat; only 6/9/21 count as RTL-equivalent, and
    /// the PX4 packed interpretation must never leak in.
    #[test]
    fn arducopter_is_rtl_only_for_known_ids(cm in any::<u32>()) {
        let expect = matches!(cm, 6 | 9 | 21);
        prop_assert_eq!(is_rtl_mode(VehicleProfile::ArduCopter, cm), expect);
    }

    /// lla_to_ned -> ned_to_lla round-trips within tight tolerance across a
    /// realistic tangent-plane patch (±0.4°, ~±45 km — well beyond any mission
    /// radius). Equirectangular projection error stays far below the detector's
    /// metre-scale thresholds.
    #[test]
    fn lla_ned_round_trips(
        dlat in -0.4f64..0.4,
        dlon in -0.4f64..0.4,
        lat0 in -60.0f64..60.0,
    ) {
        let lon0 = 10.0;
        let (lat, lon) = (lat0 + dlat, lon0 + dlon);
        let p = lla_to_ned(lat, lon, 0.0, lat0, lon0);
        let (rlat, rlon) = ned_to_lla(p.n, p.e, lat0, lon0);
        prop_assert!((rlat - lat).abs() < 1e-6, "lat {} -> {}", lat, rlat);
        prop_assert!((rlon - lon).abs() < 1e-6, "lon {} -> {}", lon, rlon);
    }

    /// North displacement is monotone in latitude on the tangent plane: moving
    /// further north always increases the NED north coordinate. (A sign error
    /// here would invert the residual direction.)
    #[test]
    fn ned_north_is_monotone_in_latitude(dlat in 0.001f64..0.4) {
        let (lat0, lon0) = (35.0, -120.0);
        let near = lla_to_ned(lat0 + 0.0005, lon0, 0.0, lat0, lon0);
        let far = lla_to_ned(lat0 + dlat, lon0, 0.0, lat0, lon0);
        prop_assert!(far.n > near.n, "north not monotone: {} !> {}", far.n, near.n);
    }

    /// Plausibility gate: ANY finite, in-range fix is accepted. The detector
    /// must never silently drop an honest fix as "implausible."
    #[test]
    fn in_range_fix_is_always_plausible(
        lat in -90.0f64..=90.0,
        lon in -180.0f64..=180.0,
        alt in -10_000.0f64..=50_000.0,
    ) {
        prop_assert!(gps_fix_is_plausible(&fix(lat, lon, alt)));
    }

    /// Plausibility gate: ANY out-of-latitude-range fix is rejected, regardless
    /// of the other fields — the adversarial-injection floor.
    #[test]
    fn out_of_range_latitude_is_always_rejected(
        lat in prop_oneof![90.001f64..1e6, -1e6..-90.001f64],
        lon in -180.0f64..=180.0,
        alt in -1000.0f64..1000.0,
    ) {
        prop_assert!(!gps_fix_is_plausible(&fix(lat, lon, alt)));
    }

    /// `ClockAligner::align` must never panic and must never return a stamp
    /// more than `CLOCK_ALIGN_MAX_SKEW` (2 s) ahead of the arrival that
    /// produced it — for ANY sequence of attacker-controlled sensor
    /// timestamps.
    ///
    /// This is the invariant whose violation was a REAL one-packet remote DoS:
    /// a crafted `time_usec` made `anchor_arrival + Duration::from_micros(..)`
    /// overflow the monotonic `Instant` and PANIC, killing the listener task
    /// and with it the whole detector. The unbounded-lead half matters too — a
    /// hostile clock that could shift GPS relative to IMU would inject
    /// residual (the detector's core signal) without touching a coordinate.
    ///
    /// The `fuzz/` harness covers this same surface with coverage guidance,
    /// but only on Linux CI (libFuzzer cannot link on Windows MSVC). This
    /// property runs everywhere, on every push.
    #[test]
    fn clock_aligner_never_leads_arrival_beyond_skew_bound(
        ops in prop::collection::vec((any::<Option<u64>>(), 0u32..5_000_000), 1..64)
    ) {
        const MAX_SKEW: std::time::Duration = std::time::Duration::from_secs(2);
        let mut aligner = ClockAligner::default();
        let base = tokio::time::Instant::now();
        let mut arrival_us: u64 = 0;
        for (sensor_us, advance_us) in ops {
            // Arrivals advance monotonically, as real message arrivals do;
            // the sensor stamp is whatever the wire claims.
            arrival_us = arrival_us.saturating_add(advance_us as u64);
            let arrival = base + std::time::Duration::from_micros(arrival_us);
            let aligned = aligner.align(sensor_us, arrival);
            prop_assert!(
                aligned.saturating_duration_since(arrival) <= MAX_SKEW,
                "aligned stamp led arrival by {:?} (bound {:?}) for sensor_us={:?}",
                aligned.saturating_duration_since(arrival), MAX_SKEW, sensor_us
            );
        }
    }

    /// Plausibility gate: a non-finite coordinate is ALWAYS rejected (NaN/Inf
    /// injection must never reach the detector math).
    #[test]
    fn non_finite_coordinate_is_always_rejected(which in 0u8..3) {
        let nan = f64::NAN;
        let f = match which {
            0 => fix(nan, 0.0, 0.0),
            1 => fix(0.0, nan, 0.0),
            _ => fix(0.0, 0.0, f64::INFINITY),
        };
        prop_assert!(!gps_fix_is_plausible(&f));
    }
}

/// Sanity: R_EARTH_M is the value the projection assumes; pin it so a careless
/// edit to the constant can't silently shift every conversion.
#[test]
fn earth_radius_is_wgs84_semi_major() {
    assert_eq!(R_EARTH_M, 6_378_137.0);
}
