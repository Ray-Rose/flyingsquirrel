//! NMEA-over-serial `GpsSource`.
//!
//! Opens a serial port (e.g. `COM3` on Windows, `/dev/ttyUSB0` on Linux,
//! `/dev/tty.usbserial-XXXX` on Mac), reads CR/LF-delimited NMEA sentences,
//! decodes them via `ingest::nmea::parse_sentence`, and yields `GpsFix`
//! items on the stream.
//!
//! Supports any GPS module that speaks NMEA 0183 — u-blox NEO-M8/M9/M10,
//! MTK3339, generic puck modules, etc. Baud rate is typically 9600 (default)
//! or 38400 for u-blox modules at higher update rates.
//!
//! This file owns the port, the reconnect loop, and the read loop only. Byte
//! framing and the link-health diagnosis live in [`crate::hw::nmea_link`],
//! which is not feature-gated so both are unit-tested on every platform (the
//! same split as `i2c_imu` vs [`crate::hw::mpu6050`]).

use crate::error::FsError;
use crate::hw::nmea_link::{LinkHealth, LinkReport, NmeaLine, NmeaLineCodec};
use crate::ingest::{nmea, BoxStream, GpsSource};
use crate::mav::gps_fix_is_plausible;
use crate::types::GpsFix;
use async_stream::stream;
use futures::StreamExt;
use tokio_serial::SerialPortBuilderExt;
use tokio_util::codec::FramedRead;
use tracing::{debug, warn};

/// How often the link-health tracker is asked whether it has something to say.
/// Well under `nmea_link::ACTIVITY_WINDOW` so "bytes are still flowing" is
/// sampled several times per window.
const HEALTH_POLL_INTERVAL: std::time::Duration = std::time::Duration::from_secs(2);

#[derive(Debug, Clone)]
pub struct SerialGpsConfig {
    pub port: String,
    pub baud: u32,
}

impl Default for SerialGpsConfig {
    fn default() -> Self {
        Self {
            port: "/dev/ttyUSB0".to_string(),
            baud: 9600,
        }
    }
}

pub struct SerialGpsSource {
    cfg: SerialGpsConfig,
}

impl SerialGpsSource {
    pub fn new(cfg: SerialGpsConfig) -> Self {
        Self { cfg }
    }
}

/// Initial reconnect delay. Doubles on each failed attempt up to
/// `RECONNECT_BACKOFF_MAX`. Defaults sized so a USB-serial jiggle (~1s)
/// recovers in roughly one cycle, while sustained failure backs off enough
/// to avoid log spam.
const RECONNECT_BACKOFF_INITIAL: std::time::Duration = std::time::Duration::from_millis(250);
const RECONNECT_BACKOFF_MAX: std::time::Duration = std::time::Duration::from_secs(5);

/// What one connection produces. The terminal `Disconnected` lets the outer
/// reconnect loop stay in `into_stream` while the per-connection work — the
/// part with all the logic — lives in a function that takes any byte source.
#[derive(Debug)]
enum SerialEvent {
    Fix(GpsFix),
    Disconnected { reason: String, had_first_fix: bool },
}

/// One connection's worth of work: frame bytes, decode sentences, gate fixes,
/// track link health. Ends by yielding exactly one [`SerialEvent::Disconnected`].
///
/// Generic over the byte source purely for testability. `tokio_serial` can only
/// open a real device, so as long as this loop was welded to it, the whole
/// serial ingest path — framing, parsing, gating, health — had ZERO test
/// coverage and could only be exercised by plugging in hardware. With an
/// `AsyncRead` parameter an in-memory pipe drives the same code end to end.
fn connection_stream<R>(
    reader: R,
    port: String,
    baud: u32,
) -> impl futures::Stream<Item = SerialEvent>
where
    R: tokio::io::AsyncRead + Unpin,
{
    stream! {
        let mut framed = FramedRead::new(reader, NmeaLineCodec::new());
        let mut parser = ::nmea::Nmea::default();
        let mut disconnect_reason: Option<String> = None;
        // A port that opens but never delivers a usable fix used to be
        // completely silent (see `nmea_link` for the four ways that happens
        // and why they are indistinguishable downstream). The health tracker
        // watches what actually reaches each stage of the pipeline and names
        // the likely cause. Observability only: it never drops a fix or ends
        // the stream.
        let opened_at = tokio::time::Instant::now();
        let mut health = LinkHealth::new();
        let mut probe = tokio::time::interval(HEALTH_POLL_INTERVAL);
        probe.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        probe.tick().await; // the first tick resolves immediately
        loop {
            // NB: no `biased;`. Preferring the data branch would let a byte
            // flood — a module dumping binary at line rate, i.e. exactly the
            // case the diagnosis exists for — keep `framed.next()`
            // permanently ready and starve the health poll. Random branch
            // selection bounds that; both futures are cancel-safe, so an
            // unpicked branch loses nothing.
            tokio::select! {
                item = framed.next() => {
                    let Some(line_res) = item else { break };
                    let line = match line_res {
                        Ok(evt) => {
                            let now = opened_at.elapsed();
                            health.on_event(&evt, now);
                            match evt {
                                NmeaLine::Line(l) => l,
                                // Framing faults are evidence for the
                                // diagnosis, not a reason to tear down a
                                // healthy port.
                                NmeaLine::NonUtf8(_) | NmeaLine::Overlong(_) => continue,
                            }
                        }
                        // A real I/O error — the port IS gone.
                        Err(e) => {
                            disconnect_reason = Some(format!("{}", e));
                            break;
                        }
                    };
                    if !line.starts_with('$') {
                        continue;
                    }
                    match nmea::parse_sentence(&mut parser, &line) {
                        Ok(Some(fix)) => {
                            if gps_fix_is_plausible(&fix) {
                                health.on_fix(opened_at.elapsed());
                                yield SerialEvent::Fix(fix);
                            } else {
                                health.on_fix_rejected();
                                debug!(line = %line, "dropped implausible nmea fix");
                            }
                        }
                        Ok(None) => {}
                        Err(e) => {
                            health.on_parse_error();
                            debug!(error = %e, line = %line, "nmea parse error");
                        }
                    }
                }
                _ = probe.tick() => {
                    let now = opened_at.elapsed();
                    health.observe_bytes(framed.decoder().bytes_seen(), now);
                    match health.poll(now) {
                        Some(LinkReport::Trouble(d)) => warn!(
                            port = %port, baud,
                            state = ?d.state,
                            silent_for_s = d.silent_for.as_secs(),
                            fixes_stopped = d.fixes_stopped(),
                            bytes = d.counts.bytes,
                            lines = d.counts.lines,
                            nmea_lines = d.counts.nmea_lines,
                            non_utf8_lines = d.counts.non_utf8_lines,
                            overlong_discards = d.counts.overlong_discards,
                            parse_errors = d.counts.parse_errors,
                            fixes = d.counts.fixes,
                            rejected_fixes = d.counts.rejected_fixes,
                            "serial GPS NOT DELIVERING FIXES: {}",
                            d.state.advice()
                        ),
                        Some(LinkReport::Recovered { was, counts }) => tracing::info!(
                            port = %port, was = ?was, fixes = counts.fixes,
                            "serial GPS RECOVERED: plausible fixes are flowing again"
                        ),
                        None => {}
                    }
                }
            }
        }
        // Loop ended: either the stream closed (peer EOF) or a read error
        // fired. Both mean "reconnect."
        yield SerialEvent::Disconnected {
            reason: disconnect_reason
                .unwrap_or_else(|| "stream ended (peer EOF)".to_string()),
            had_first_fix: health.counts().fixes > 0,
        };
    }
}

impl GpsSource for SerialGpsSource {
    fn into_stream(self: Box<Self>) -> BoxStream<'static, Result<GpsFix, FsError>> {
        // AUDIT E-13 fix: wrap the read loop in an outer reconnect-with-backoff
        // loop. USB-serial dongles disconnect transiently in the field
        // (vibration, EMI, kernel renumeration) and the prior code yielded
        // an error then returned, killing the ingest task and forcing a
        // process-level restart for every disconnect. Each restart left
        // the drone undefended for the 2-5s startup window. Now the source
        // logs the disconnect, sleeps with exponential backoff, and retries.
        // The fixes received before the disconnect are still valid; new
        // fixes resume on reconnect.
        let SerialGpsConfig { port, baud } = self.cfg;
        let stream = stream! {
            let mut backoff = RECONNECT_BACKOFF_INITIAL;
            loop {
                let port_result = tokio_serial::new(&port, baud).open_native_async();
                let open_port = match port_result {
                    Ok(p) => {
                        // Reset backoff on a clean open — long uptime with one
                        // transient failure shouldn't penalize the next one.
                        backoff = RECONNECT_BACKOFF_INITIAL;
                        tracing::info!(port = %port, baud, "serial GPS connected");
                        p
                    }
                    Err(e) => {
                        // Don't yield an Err — that propagates through ingest
                        // and (currently) terminates the task. Just log and
                        // back off. Yield is reserved for actual fixes.
                        warn!(
                            port = %port, baud, error = %e, backoff_ms = backoff.as_millis() as u64,
                            "serial GPS open failed; will retry"
                        );
                        tokio::time::sleep(backoff).await;
                        backoff = (backoff * 2).min(RECONNECT_BACKOFF_MAX);
                        continue;
                    }
                };
                // All per-connection work lives in `connection_stream`,
                // which is generic over the byte source so it can be tested
                // without hardware. This loop owns only the reconnect policy.
                let events = connection_stream(open_port, port.clone(), baud);
                futures::pin_mut!(events);
                while let Some(evt) = events.next().await {
                    match evt {
                        SerialEvent::Fix(fix) => yield Ok(fix),
                        SerialEvent::Disconnected { reason, had_first_fix } => warn!(
                            port = %port, reason = %reason, had_first_fix,
                            backoff_ms = backoff.as_millis() as u64,
                            "serial GPS disconnected; reconnecting"
                        ),
                    }
                }
                tokio::time::sleep(backoff).await;
                backoff = (backoff * 2).min(RECONNECT_BACKOFF_MAX);
            }
        };
        Box::pin(stream)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hw::nmea_link::NMEA_MAX_LINE_BYTES;
    use tokio::io::AsyncWriteExt;

    /// Canonical GGA: 48°07.038'N, 11°31.000'E, 3D fix, 545.4 m. Checksum valid.
    const GGA: &[u8] = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";

    /// Drive the REAL per-connection loop over an in-memory pipe: write the
    /// bytes a module would send, drop the writer (EOF = unplugged), and
    /// collect everything the loop produced. This is the coverage that was
    /// impossible while the loop was welded to `tokio_serial`.
    async fn run(bytes: &[u8]) -> Vec<SerialEvent> {
        let (mut writer, reader) = tokio::io::duplex(1024);
        let bytes = bytes.to_vec();
        // Written from a task so a payload larger than the pipe buffer cannot
        // deadlock against a reader that has not drained yet.
        tokio::spawn(async move {
            let _ = writer.write_all(&bytes).await;
        });
        let events = connection_stream(reader, "test-port".to_string(), 9600);
        futures::pin_mut!(events);
        let mut out = Vec::new();
        while let Some(e) = events.next().await {
            out.push(e);
        }
        out
    }

    #[tokio::test]
    async fn nmea_bytes_become_fixes_and_eof_ends_the_connection() {
        let events = run(GGA).await;
        match events.as_slice() {
            [SerialEvent::Fix(fix), SerialEvent::Disconnected {
                had_first_fix: true,
                ..
            }] => {
                assert!((fix.lat_deg - 48.1173).abs() < 1e-3, "lat={}", fix.lat_deg);
                assert!((fix.lon_deg - 11.5167).abs() < 1e-3, "lon={}", fix.lon_deg);
                assert!((fix.alt_m - 545.4).abs() < 0.1, "alt={}", fix.alt_m);
            }
            other => panic!("expected one fix then a disconnect, got {other:?}"),
        }
    }

    #[tokio::test]
    async fn lines_that_are_not_sentences_are_skipped_not_fatal() {
        let mut bytes = b"garbage\r\n\r\nnot a sentence\n".to_vec();
        bytes.extend_from_slice(GGA);
        let events = run(&bytes).await;
        assert!(
            matches!(events.first(), Some(SerialEvent::Fix(_))),
            "noise before a valid sentence must not stop the fix: {events:?}"
        );
    }

    #[tokio::test]
    async fn a_link_that_never_fixes_reports_no_first_fix() {
        // Valid NMEA, fix quality 0: the no-antenna / indoors / cold-start
        // case. Nothing is yielded, and the disconnect says so.
        let events = run(b"$GPGGA,,,,,,0,00,,,M,,M,,*66\r\n").await;
        assert!(
            matches!(
                events.as_slice(),
                [SerialEvent::Disconnected {
                    had_first_fix: false,
                    ..
                }]
            ),
            "expected only a no-fix disconnect, got {events:?}"
        );
    }

    #[tokio::test]
    async fn unplug_mid_sentence_reports_eof_not_an_io_error() {
        // REGRESSION: a module yanked mid-sentence leaves a partial line
        // buffered, and `Decoder`'s default `decode_eof` turns that into
        // `Err("bytes remaining on stream")` — so the commonest real
        // disconnect surfaced as an opaque I/O string instead of a plain EOF.
        // The fix earns its keep in the operator log, so assert the wording.
        let mut bytes = GGA.to_vec();
        bytes.extend_from_slice(b"$GPRMC,truncated-by-unplug");
        let events = run(&bytes).await;
        match events.as_slice() {
            [SerialEvent::Fix(_), SerialEvent::Disconnected {
                reason,
                had_first_fix: true,
            }] => assert!(
                reason.contains("peer EOF"),
                "a truncated tail is not a port failure, got reason {reason:?}"
            ),
            other => panic!("expected a fix then a clean EOF disconnect, got {other:?}"),
        }
    }

    #[tokio::test]
    async fn binary_garbage_does_not_tear_down_a_working_connection() {
        // REGRESSION: the codec used to return `Err` on an over-long line,
        // and an `Err` from a `Decoder` terminates `FramedRead` for good — so
        // one burst from a module in UBX binary mode killed the connection,
        // logged a misleading "disconnected", and cost a reconnect. The
        // sentence AFTER the burst proves the connection survived.
        let mut bytes = vec![0xB5u8; NMEA_MAX_LINE_BYTES * 2]; // UBX sync char, no '\n'
        bytes.push(b'\n'); // resync point
        bytes.extend_from_slice(GGA);
        let events = run(&bytes).await;
        assert!(
            events.iter().any(|e| matches!(e, SerialEvent::Fix(_))),
            "a fix after the garbage burst proves the port was not torn down: {events:?}"
        );
    }
}
