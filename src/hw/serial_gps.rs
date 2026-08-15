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
                let mut framed = FramedRead::new(open_port, NmeaLineCodec::new());
                let mut parser = ::nmea::Nmea::default();
                let mut disconnect_reason: Option<String> = None;
                // A port that opens but never delivers a usable fix used to be
                // completely silent (see `nmea_link` for the four ways that
                // happens and why they are indistinguishable downstream). The
                // health tracker watches what actually reaches each stage of
                // the pipeline and names the likely cause. Observability only:
                // it never drops a fix or ends the stream.
                let opened_at = tokio::time::Instant::now();
                let mut health = LinkHealth::new();
                let mut probe = tokio::time::interval(HEALTH_POLL_INTERVAL);
                probe.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
                probe.tick().await; // the first tick resolves immediately
                loop {
                    // NB: no `biased;`. Preferring the data branch would let a
                    // byte flood — a module dumping binary at line rate, i.e.
                    // exactly the case the diagnosis exists for — keep
                    // `framed.next()` permanently ready and starve the health
                    // poll. Random branch selection bounds that; both futures
                    // are cancel-safe, so an unpicked branch loses nothing.
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
                                        // diagnosis, not a reason to tear down
                                        // a healthy port.
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
                                        yield Ok(fix);
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
                // Loop ended: either the stream closed (peer EOF) or a read
                // error fired. Both mean "reconnect."
                let reason = disconnect_reason
                    .unwrap_or_else(|| "stream ended (peer EOF)".to_string());
                warn!(
                    port = %port, reason = %reason,
                    had_first_fix = health.counts().fixes > 0,
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
