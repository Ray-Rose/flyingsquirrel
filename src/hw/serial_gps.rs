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

use crate::error::FsError;
use crate::ingest::{nmea, BoxStream, GpsSource};
use crate::mav::gps_fix_is_plausible;
use crate::types::GpsFix;
use async_stream::stream;
use bytes::BytesMut;
use futures::StreamExt;
use tokio_serial::SerialPortBuilderExt;
use tokio_util::codec::{Decoder, FramedRead};
use tracing::{debug, warn};

/// NMEA 0183 limits sentences to 82 chars. We accept a generous 512 to
/// tolerate non-conformant modules but no further — past this, a peer that
/// never emits `\n` would grow the buffer unboundedly (OOM DoS).
const NMEA_MAX_LINE_BYTES: usize = 512;

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

/// Split incoming bytes on `\n`, yielding each line (without the newline)
/// as a UTF-8 String. Lines that aren't valid UTF-8 are dropped.
struct NmeaLineCodec;

impl Decoder for NmeaLineCodec {
    type Item = String;
    type Error = std::io::Error;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<String>, std::io::Error> {
        if let Some(pos) = src.iter().position(|&b| b == b'\n') {
            let line = src.split_to(pos + 1);
            let trimmed = line
                .iter()
                .take(line.len() - 1)
                .copied()
                .filter(|&b| b != b'\r')
                .collect::<Vec<u8>>();
            match String::from_utf8(trimmed) {
                Ok(s) => Ok(Some(s)),
                Err(_) => Ok(Some(String::new())), // skip non-UTF-8 lines as empty
            }
        } else if src.len() > NMEA_MAX_LINE_BYTES {
            // Hostile or malfunctioning peer that never closes a line — discard
            // the accumulated buffer and return an error so the upper loop can
            // log it. Don't OOM.
            src.clear();
            Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "nmea line exceeds maximum allowed length",
            ))
        } else {
            Ok(None)
        }
    }
}

/// Initial reconnect delay. Doubles on each failed attempt up to
/// `RECONNECT_BACKOFF_MAX`. Defaults sized so a USB-serial jiggle (~1s)
/// recovers in roughly one cycle, while sustained failure backs off enough
/// to avoid log spam.
const RECONNECT_BACKOFF_INITIAL: std::time::Duration = std::time::Duration::from_millis(250);
const RECONNECT_BACKOFF_MAX: std::time::Duration = std::time::Duration::from_secs(5);

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
                let mut framed = FramedRead::new(open_port, NmeaLineCodec);
                let mut parser = ::nmea::Nmea::default();
                let mut had_first_fix = false;
                let mut disconnect_reason: Option<String> = None;
                while let Some(line_res) = framed.next().await {
                    match line_res {
                        Ok(line) if line.starts_with('$') => {
                            match nmea::parse_sentence(&mut parser, &line) {
                                Ok(Some(fix)) => {
                                    if gps_fix_is_plausible(&fix) {
                                        had_first_fix = true;
                                        yield Ok(fix);
                                    } else {
                                        debug!(line = %line, "dropped implausible nmea fix");
                                    }
                                }
                                Ok(None) => {}
                                Err(e) => debug!(error = %e, line = %line, "nmea parse error"),
                            }
                        }
                        Ok(_) => {}
                        Err(e) => {
                            disconnect_reason = Some(format!("{}", e));
                            break;
                        }
                    }
                }
                // Loop ended: either the stream closed (peer EOF) or a read
                // error fired. Both mean "reconnect."
                let reason = disconnect_reason
                    .unwrap_or_else(|| "stream ended (peer EOF)".to_string());
                warn!(
                    port = %port, reason = %reason, had_first_fix,
                    backoff_ms = backoff.as_millis() as u64,
                    "serial GPS disconnected; reconnecting"
                );
                tokio::time::sleep(backoff).await;
                backoff = (backoff * 2).min(RECONNECT_BACKOFF_MAX);
            }
        };
        Box::pin(stream)
    }
}
