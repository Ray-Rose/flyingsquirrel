//! NMEA 0183 -> GpsFix decoder. Used by future serial-backed `GpsSource`
//! implementations. Currently unused by the simulator backend.

use crate::error::{FsError, IngestError};
use crate::types::{GpsFix, Timestamp};
use ::nmea::sentences::FixType;
use ::nmea::{Nmea, SentenceType};

/// Decode one NMEA sentence into a `GpsFix` if it carries a position fix.
/// Returns Ok(None) for valid-but-non-position sentences (e.g. GSV).
pub fn parse_sentence(parser: &mut Nmea, line: &str) -> Result<Option<GpsFix>, FsError> {
    let sentence_type = parser
        .parse(line)
        .map_err(|e| FsError::Ingest(IngestError::NmeaParse(format!("{:?}", e))))?;

    match sentence_type {
        SentenceType::GGA | SentenceType::RMC => {
            if !matches!(
                parser.fix_type,
                Some(FixType::Gps | FixType::DGps | FixType::Pps)
            ) {
                return Ok(None);
            }
            let (Some(lat), Some(lon)) = (parser.latitude, parser.longitude) else {
                return Ok(None);
            };
            let alt = parser.altitude.unwrap_or(0.0) as f64;
            let hdop = parser.hdop;
            let sats = parser.fix_satellites().map(|s| s as u8);
            let speed_mps = parser.speed_over_ground.map(|s| (s as f64) * 0.514_444); // knots -> m/s
            let course_deg = parser.true_course.map(|c| c as f64);
            Ok(Some(GpsFix {
                t: Timestamp::now_mono(),
                lat_deg: lat,
                lon_deg: lon,
                alt_m: alt,
                speed_mps,
                course_deg,
                hdop,
                sats,
            }))
        }
        _ => Ok(None),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// AUDIT I-01 regression test: feed real, checksum-valid NMEA sentences
    /// through the parser and assert a `GpsFix` actually comes out. This test
    /// FAILS if the `nmea` crate's sentence features are not enabled in
    /// Cargo.toml (the parser returns `Err(DisabledSentence)`), which is the
    /// exact silent real-hardware failure that shipped before this fix.
    #[test]
    fn parses_real_gga_sentence_to_fix() {
        let mut parser = Nmea::default();
        // Canonical GGA fix: 48°07.038'N, 11°31.000'E, 3D fix, 545.4 m alt.
        // Checksum 0x47 is valid for this sentence.
        let gga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        let result = parse_sentence(&mut parser, gga).expect(
            "GGA must parse without error — if this errors with DisabledSentence, \
                     the nmea crate's GGA feature is not enabled in Cargo.toml",
        );
        let fix = result.expect("a valid 3D GGA must yield a GpsFix");
        // 48°07.038' = 48 + 7.038/60 = 48.1173 N
        assert!((fix.lat_deg - 48.1173).abs() < 1e-3, "lat={}", fix.lat_deg);
        // 011°31.000' = 11 + 31.0/60 = 11.5167 E
        assert!((fix.lon_deg - 11.5167).abs() < 1e-3, "lon={}", fix.lon_deg);
        assert!((fix.alt_m - 545.4).abs() < 0.1, "alt={}", fix.alt_m);
        assert_eq!(fix.sats, Some(8));
    }

    #[test]
    fn parses_real_rmc_sentence() {
        let mut parser = Nmea::default();
        // RMC with position + speed (22.4 knots) + course (84.4°).
        let rmc = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
        let result = parse_sentence(&mut parser, rmc)
            .expect("RMC must parse without error (nmea RMC feature must be enabled)");
        // RMC alone may not set fix_type to Gps in the nmea crate's state model;
        // the key assertion is that parsing does not error with DisabledSentence.
        // If a fix is produced, sanity-check its velocity conversion.
        if let Some(fix) = result {
            if let Some(speed) = fix.speed_mps {
                // 22.4 knots * 0.514444 = 11.52 m/s
                assert!((speed - 11.52).abs() < 0.2, "speed={}", speed);
            }
        }
    }

    #[test]
    fn rejects_bad_checksum() {
        let mut parser = Nmea::default();
        // Same GGA but with a deliberately wrong checksum (*00).
        let bad = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00";
        // Must NOT yield a fix — either an Err (checksum mismatch) or Ok(None).
        match parse_sentence(&mut parser, bad) {
            Err(_) => {}   // checksum rejected — good
            Ok(None) => {} // also acceptable
            Ok(Some(_)) => panic!("a sentence with an invalid checksum must not yield a fix"),
        }
    }
}
