//! Fuzz the NMEA ingest path: arbitrary bytes → line codec → `parse_sentence`
//! → plausibility gate.
//!
//! This is the serial-GPS attack surface. A GPS module (or anything spliced
//! onto the UART) can send arbitrary bytes, and unlike the MAVLink path there
//! is no checksum-authenticated framing in front of the parser — the line
//! codec splits on `\n` and hands whatever it got to the decoder.
//!
//! The input is a SEQUENCE OF READ CHUNKS, not one blob, because both stages
//! are stateful and the interesting bugs live in that state:
//!
//! * `NmeaLineCodec` buffers across reads, so where a chunk boundary falls
//!   decides what a "line" even is. A blob would only ever exercise
//!   already-complete input.
//! * `nmea::Nmea` accumulates fields across sentences (GGA supplies altitude,
//!   RMC supplies speed/course), so single-sentence fuzzing would miss every
//!   cross-sentence state bug.
//!
//! The codec is driven directly rather than modeled here. An earlier version
//! of this target hand-mirrored its rules ("split on `\n`, skip non-UTF-8,
//! keep lines starting with `$`"), which meant a change to the real codec
//! would silently stop being fuzzed — a nightly net quietly covering a
//! function that no longer exists. Same reasoning as `ClockAligner` being
//! `pub` for the `clock_aligner` target: on a security boundary, a divergent
//! test double is worse than no test.
//!
//! Properties asserted:
//!
//! 1. Neither the codec, the parser, nor the plausibility gate panics on any
//!    input.
//! 2. **The codec's buffer stays bounded.** A peer that never emits `\n` must
//!    not grow it without limit — that is an OOM DoS on a companion computer
//!    with a few hundred MB of RAM.
//! 3. **Whatever the gate ACCEPTS is finite and in range.** The gate is the
//!    contract between untrusted bytes and the detector's residual
//!    arithmetic; a NaN or 400°-longitude slipping past it poisons that math
//!    silently (the same class of bug as the `ClockAligner` overflow — code
//!    that "succeeds" into garbage is worse than code that errors).
//!
//! Note what is deliberately NOT asserted: that the parser never PRODUCES an
//! implausible fix. It legitimately can — that is exactly why `serial_gps`
//! filters every fix through the gate before yielding it. Asserting otherwise
//! would flag correct production behavior as a crash.

#![no_main]

use bytes::BytesMut;
use flyingsquirrel::hw::nmea_link::{NmeaLine, NmeaLineCodec, NMEA_MAX_LINE_BYTES};
use libfuzzer_sys::fuzz_target;
use tokio_util::codec::Decoder;

fuzz_target!(|reads: Vec<Vec<u8>>| {
    let mut codec = NmeaLineCodec::new();
    let mut buf = BytesMut::new();
    let mut parser = ::nmea::Nmea::default();

    for chunk in reads {
        buf.extend_from_slice(&chunk);
        // Drain exactly as `FramedRead` does: decode until it asks for more.
        while let Ok(Some(evt)) = codec.decode(&mut buf) {
            // Non-line events are framing evidence for the health tracker;
            // the parser never sees them (mirrors `serial_gps`).
            let NmeaLine::Line(line) = evt else { continue };
            // The serial source only forwards lines beginning with '$'.
            if !line.starts_with('$') {
                continue;
            }
            if let Ok(Some(fix)) = flyingsquirrel::ingest::nmea::parse_sentence(&mut parser, &line)
            {
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

        // The OOM guard, checked after every read: once the codec has finished
        // draining, anything it kept is an unterminated partial line, and that
        // must never exceed the cap regardless of what the peer sends.
        assert!(
            buf.len() <= NMEA_MAX_LINE_BYTES,
            "codec retained {} buffered bytes, above the {NMEA_MAX_LINE_BYTES}-byte cap",
            buf.len()
        );
    }
});
