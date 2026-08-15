//! Fuzz the NMEA ingest path: arbitrary bytes → `parse_sentence` → plausibility.
//!
//! This is the serial-GPS attack surface. A GPS module (or anything spliced
//! onto the UART) can send arbitrary bytes, and unlike the MAVLink path there
//! is no checksum-authenticated framing in front of the parser — the line
//! codec splits on `\n` and hands whatever it got to the decoder.
//!
//! Properties asserted:
//!
//! 1. Neither the parser nor the plausibility gate panics on any input.
//! 2. **Whatever the gate ACCEPTS is finite and in range.** The gate is the
//!    contract between untrusted bytes and the detector's residual
//!    arithmetic; a NaN or 400°-longitude slipping past it poisons that math
//!    silently (the same class of bug as the `ClockAligner` overflow — code
//!    that "succeeds" into garbage is worse than code that errors).
//!
//! Note what is deliberately NOT asserted: that the parser never PRODUCES an
//! implausible fix. It legitimately can — that is exactly why `serial_gps`
//! filters every fix through the gate before yielding it. Asserting otherwise
//! would flag correct production behavior as a crash.
//!
//! The parser is fed a MULTI-LINE sequence against ONE parser instance,
//! because `nmea::Nmea` is stateful: fields accumulate across sentences (GGA
//! supplies altitude, RMC supplies speed/course), so single-line fuzzing would
//! miss every cross-sentence state bug.

#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    // Non-UTF-8 lines are dropped by the line codec before the parser sees
    // them, so mirror that here rather than fuzzing an unreachable path.
    let Ok(text) = std::str::from_utf8(data) else {
        return;
    };
    let mut parser = ::nmea::Nmea::default();
    for line in text.split('\n') {
        // The serial source only forwards lines beginning with '$'.
        if !line.starts_with('$') {
            continue;
        }
        if let Ok(Some(fix)) = flyingsquirrel::ingest::nmea::parse_sentence(&mut parser, line) {
            if flyingsquirrel::mav::gps_fix_is_plausible(&fix) {
                assert!(
                    fix.lat_deg.is_finite() && fix.lon_deg.is_finite() && fix.alt_m.is_finite(),
                    "gate accepted a non-finite fix from {line:?}: {fix:?}"
                );
                assert!(
                    fix.lat_deg.abs() <= 90.0 && fix.lon_deg.abs() <= 180.0,
                    "gate accepted out-of-range coordinates from {line:?}: {fix:?}"
                );
                if let Some(s) = fix.speed_mps {
                    assert!(s.is_finite(), "gate accepted non-finite speed: {fix:?}");
                }
                if let Some(c) = fix.course_deg {
                    assert!(c.is_finite(), "gate accepted non-finite course: {fix:?}");
                }
            }
        }
    }
});
