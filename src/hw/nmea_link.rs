//! NMEA-over-serial link layer: byte framing + link-health diagnosis.
//!
//! Pure logic, no I/O — the same split as [`crate::hw::mpu6050`] vs
//! `i2c_imu`: [`crate::hw::serial_gps`] owns the port and the read loop, this
//! module owns what the bytes MEAN and when a quiet link is trouble. Neither
//! is feature-gated, so the framing and the policy are unit-tested on every
//! platform even though the serial port itself can only be opened on a box
//! with the hardware attached.
//!
//! # Why a health tracker exists at all
//!
//! The I²C IMU path refuses to start behind the wrong chip (WHO_AM_I) and
//! escalates a bus that stops delivering. The serial GPS path had no
//! equivalent: `open_native_async` succeeding logged "serial GPS connected"
//! and, from then on, *every* way the link could fail to deliver a usable fix
//! was reported at `debug!` or not at all. All four are ordinary bring-up
//! mistakes, and all four look identical from the outside — a detector that
//! silently never gets a GPS fix:
//!
//! 1. **Nothing on the wire.** TX/RX swapped, module unpowered, wrong port.
//! 2. **Bytes, but not NMEA.** Wrong baud (framing errors produce high-bit
//!    garbage) or a u-blox left in UBX binary mode — a genuinely common
//!    misconfiguration, since a module configured once for binary output
//!    keeps that setting across power cycles.
//! 3. **NMEA, but no fix.** No antenna, indoors, or a cold start that has not
//!    converged yet. Only this one is ever *normal*, which is why it gets a
//!    much longer fuse than the others.
//! 4. **Fixes, but all rejected** by the plausibility gate. That gate is a
//!    security boundary; it rejecting 100% of a live module's output is an
//!    anomaly the operator must see, not a `debug!` line.
//!
//! Distinguishing them is possible precisely because this layer sees the raw
//! byte stream, and each one has a different fix. So the diagnosis names the
//! evidence and the likely cause instead of reporting a generic "no GPS".
//!
//! # What this is NOT
//!
//! Observability only. Nothing here changes a threshold, gates a detector, or
//! ends a stream: absence of GPS is already handled correctly downstream (the
//! preflight checklist refuses to become Ready without `first_fix_accepted`,
//! and the dead-reckoner keeps running). A cold start legitimately takes
//! a minute, so escalating to stream-end here would turn a slow boot into an
//! outage — the opposite of the IMU ladder's tradeoff, where stale samples
//! actively poison dead-reckoning and must be stopped.

use bytes::BytesMut;
use std::time::Duration;
use tokio_util::codec::Decoder;

/// NMEA 0183 limits sentences to 82 chars. We accept a generous 512 to
/// tolerate non-conformant modules but no further — past this, a peer that
/// never emits `\n` would grow the buffer unboundedly (OOM DoS).
pub const NMEA_MAX_LINE_BYTES: usize = 512;

/// One decoded event from the byte stream.
///
/// The non-line variants are not noise to be swallowed: they are the evidence
/// that separates "wrong baud" from "unplugged" in [`LinkHealth`], so the
/// codec reports them rather than silently dropping the bytes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NmeaLine {
    /// A complete `\n`-terminated line, CR stripped, valid UTF-8. May still be
    /// empty or non-NMEA — framing succeeded, that is all this says.
    Line(String),
    /// A complete line that was not valid UTF-8 (`n` bytes discarded). The
    /// signature of a baud mismatch: framing errors set the high bit.
    NonUtf8(usize),
    /// `n` buffered bytes discarded without ever seeing `\n`. The signature of
    /// a module emitting binary (UBX) rather than NMEA.
    Overlong(usize),
}

/// Split incoming bytes on `\n`, yielding each line (CR stripped) as a
/// [`NmeaLine`].
///
/// Framing faults are reported as items, NOT as `Err`. `Err` from a
/// [`Decoder`] terminates the `FramedRead` stream permanently, which the
/// serial source can only interpret as a disconnect — so the previous
/// behavior (erroring on an over-long line) tore down and reopened a
/// perfectly healthy port, logged the misleading "serial GPS disconnected",
/// and cost a reconnect blind window per event. A module stuck in binary mode
/// produced an endless open/close churn instead of one clear diagnosis.
/// Keeping the two apart — **I/O errors mean the port is gone; framing faults
/// mean the DATA is wrong** — lets each get its correct response.
#[derive(Debug, Default)]
pub struct NmeaLineCodec {
    bytes_seen: u64,
    /// Buffer length at the end of the previous `decode` call, so growth can
    /// be attributed to newly-arrived bytes (the buffer only ever shrinks by
    /// our own hand).
    last_len: usize,
}

impl NmeaLineCodec {
    pub fn new() -> Self {
        Self::default()
    }

    /// Total bytes that have arrived on the port, including bytes still
    /// sitting in a partial line. Counted here rather than inferred from
    /// emitted items so that "not a single byte arrived" is a claim the
    /// diagnosis can make truthfully — a module that sends half a sentence
    /// and then goes quiet emits no items at all.
    pub fn bytes_seen(&self) -> u64 {
        self.bytes_seen
    }
}

impl Decoder for NmeaLineCodec {
    type Item = NmeaLine;
    type Error = std::io::Error;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<NmeaLine>, std::io::Error> {
        self.bytes_seen += src.len().saturating_sub(self.last_len) as u64;

        let out = if let Some(pos) = src.iter().position(|&b| b == b'\n') {
            let line = src.split_to(pos + 1);
            let trimmed = line
                .iter()
                .take(line.len() - 1)
                .copied()
                .filter(|&b| b != b'\r')
                .collect::<Vec<u8>>();
            let len = trimmed.len();
            match String::from_utf8(trimmed) {
                Ok(s) => Some(NmeaLine::Line(s)),
                Err(_) => Some(NmeaLine::NonUtf8(len)),
            }
        } else if src.len() > NMEA_MAX_LINE_BYTES {
            // No terminator in 6x the maximum legal sentence length: whatever
            // is buffered cannot be a valid NMEA line. Drop it and resync on
            // the next `\n`. Bounded discard, so no OOM either way.
            let dropped = src.len();
            src.clear();
            Some(NmeaLine::Overlong(dropped))
        } else {
            None
        };

        self.last_len = src.len();
        Ok(out)
    }

    /// At EOF, drain any complete line, then DISCARD an unterminated tail
    /// instead of erroring on it.
    ///
    /// The [`Decoder`] default turns leftover bytes into
    /// `Err("bytes remaining on stream")`, which would break the contract
    /// above through the back door: a module unplugged mid-sentence — the
    /// commonest real disconnect there is — leaves a partial line buffered, so
    /// the operator got that opaque string in place of "stream ended (peer
    /// EOF)". A truncated sentence is incomplete DATA, not a port failure, and
    /// its bytes are already counted in [`Self::bytes_seen`], so dropping it
    /// costs the diagnosis no evidence.
    fn decode_eof(&mut self, src: &mut BytesMut) -> Result<Option<NmeaLine>, std::io::Error> {
        if let Some(item) = self.decode(src)? {
            return Ok(Some(item));
        }
        src.clear();
        self.last_len = 0;
        Ok(None)
    }
}

/// How recently something must have happened to still count as "flowing".
/// NMEA modules emit continuously (a GGA per fix interval, 1–10 Hz), so a
/// multi-second gap is already meaningful.
pub const ACTIVITY_WINDOW: Duration = Duration::from_secs(6);

/// First-diagnosis fuse for the conditions that are NEVER normal: a powered,
/// correctly-wired, correctly-configured module puts NMEA on the wire within
/// a second of the port opening.
pub const QUIET_LINK_GRACE: Duration = Duration::from_secs(10);

/// First-diagnosis fuse for "sentences flowing, no position fix". Much longer,
/// because this one IS normal for a while: a receiver without a current
/// almanac can take 30–60 s to first fix, longer under poor sky view. Warning
/// at 10 s would cry wolf on every cold boot and train operators to ignore it.
pub const NO_FIX_GRACE: Duration = Duration::from_secs(60);

/// Re-report cadence while a condition persists — often enough to stay
/// visible on a dashboard, rare enough not to bury the rest of the log.
pub const REPEAT_INTERVAL: Duration = Duration::from_secs(60);

/// What the link is actually doing, most-specific cause first.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkState {
    /// Plausible fixes are flowing.
    Healthy,
    /// Port open, nothing on the wire.
    NoBytes,
    /// Bytes arriving, but nothing that frames as an NMEA sentence.
    NoSentences,
    /// NMEA sentences arriving, but none carries a usable position fix.
    NoFix,
    /// Fixes decoded, but the plausibility gate rejected every one.
    FixesRejected,
}

impl LinkState {
    /// How long the condition must hold before the first report.
    pub fn grace(self) -> Duration {
        match self {
            LinkState::Healthy => Duration::ZERO,
            LinkState::NoFix => NO_FIX_GRACE,
            _ => QUIET_LINK_GRACE,
        }
    }

    /// Operator-facing cause and action. Named causes, not "check your setup".
    pub fn advice(self) -> &'static str {
        match self {
            LinkState::Healthy => "link healthy",
            LinkState::NoBytes => {
                "no bytes at all on the port. Check power to the module, the TX/RX pair \
                 (swapped TX/RX is the classic wiring error and looks exactly like this), \
                 and that --gps-port names the right device"
            }
            LinkState::NoSentences => {
                "bytes are arriving but none of them frame as NMEA sentences. Two usual \
                 causes: (a) baud mismatch — check --gps-baud against the module (9600 is \
                 the common default, u-blox at higher rates often ship 38400); (b) the \
                 module is emitting BINARY, e.g. a u-blox left in UBX-only mode, which \
                 persists across power cycles — re-enable NMEA output on it"
            }
            LinkState::NoFix => {
                "NMEA sentences are arriving but carry no position fix. Normal for the \
                 first 30-60 s of a cold start; past that, suspect the antenna (connected? \
                 sky view?) or an indoor bench run"
            }
            LinkState::FixesRejected => {
                "position fixes are decoding but EVERY one is being rejected by the \
                 plausibility gate (non-finite or out-of-range coordinates). A live module \
                 should never do this — suspect a faulty receiver or a tampered stream"
            }
        }
    }
}

/// Cumulative evidence, logged alongside a diagnosis so the operator can see
/// which stage of the pipeline the data actually reaches.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LinkCounts {
    /// Bytes read from the port, including bytes in a still-partial line.
    pub bytes: u64,
    /// Complete lines framed (of any content).
    pub lines: u64,
    /// Lines beginning with `$` — i.e. candidate NMEA sentences.
    pub nmea_lines: u64,
    /// Lines dropped as invalid UTF-8 (baud-mismatch signature).
    pub non_utf8_lines: u64,
    /// Unterminated buffers discarded (binary-output signature).
    pub overlong_discards: u64,
    /// Sentences the decoder rejected (bad checksum, malformed fields).
    pub parse_errors: u64,
    /// Fixes that passed the plausibility gate and were yielded.
    pub fixes: u64,
    /// Fixes that decoded but failed the plausibility gate.
    pub rejected_fixes: u64,
}

/// A trouble report for one link condition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LinkDiagnosis {
    pub state: LinkState,
    pub counts: LinkCounts,
    /// Time since the last plausible fix, or since the port opened if there
    /// has never been one.
    pub silent_for: Duration,
}

impl LinkDiagnosis {
    /// True if this link delivered fixes and then stopped, as opposed to never
    /// having acquired one. Different situation, different operator response:
    /// a link that worked and quit points at the antenna/cable, not at the
    /// configuration.
    pub fn fixes_stopped(&self) -> bool {
        self.counts.fixes > 0
    }
}

/// What [`LinkHealth::poll`] decided to say, if anything.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkReport {
    Trouble(LinkDiagnosis),
    /// Fixes resumed after a reported problem. Emitted once per episode, so
    /// the log shows a closed interval rather than an unresolved warning.
    Recovered {
        was: LinkState,
        counts: LinkCounts,
    },
}

/// Tracks what is flowing on the link and decides when to say something.
///
/// Time is supplied by the caller as a monotone `Duration` since the port
/// opened, which keeps the whole policy testable without clocks or sleeps.
#[derive(Debug, Default)]
pub struct LinkHealth {
    counts: LinkCounts,
    last_byte_at: Option<Duration>,
    last_nmea_line_at: Option<Duration>,
    last_fix_at: Option<Duration>,
    /// Rejections since the most recent accepted fix — so an old rejection
    /// cannot keep re-diagnosing a link that has since recovered.
    rejected_since_fix: u64,
    last_report_at: Option<Duration>,
    reported: Option<LinkState>,
}

impl LinkHealth {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn counts(&self) -> LinkCounts {
        self.counts
    }

    /// Record the codec's cumulative byte count. Called on the poll tick
    /// rather than per byte; growth since the last observation is what marks
    /// the link as live.
    pub fn observe_bytes(&mut self, total: u64, now: Duration) {
        if total > self.counts.bytes {
            self.counts.bytes = total;
            self.last_byte_at = Some(now);
        }
    }

    /// Record one framed event from the codec.
    pub fn on_event(&mut self, evt: &NmeaLine, now: Duration) {
        match evt {
            NmeaLine::Line(s) => {
                self.counts.lines += 1;
                if s.starts_with('$') {
                    self.counts.nmea_lines += 1;
                    self.last_nmea_line_at = Some(now);
                }
            }
            NmeaLine::NonUtf8(_) => {
                self.counts.lines += 1;
                self.counts.non_utf8_lines += 1;
            }
            NmeaLine::Overlong(_) => self.counts.overlong_discards += 1,
        }
        // Any event proves bytes arrived, even if the poll tick has not run.
        self.last_byte_at = Some(now);
    }

    /// A sentence failed to decode (bad checksum, malformed field).
    pub fn on_parse_error(&mut self) {
        self.counts.parse_errors += 1;
    }

    /// A fix passed the plausibility gate and was yielded downstream.
    pub fn on_fix(&mut self, now: Duration) {
        self.counts.fixes += 1;
        self.rejected_since_fix = 0;
        self.last_fix_at = Some(now);
    }

    /// A fix decoded but failed the plausibility gate.
    pub fn on_fix_rejected(&mut self) {
        self.counts.rejected_fixes += 1;
        self.rejected_since_fix += 1;
    }

    fn classify(&self, now: Duration) -> LinkState {
        let recent =
            |t: Option<Duration>| t.is_some_and(|t| now.saturating_sub(t) < ACTIVITY_WINDOW);
        if recent(self.last_fix_at) {
            LinkState::Healthy
        } else if !recent(self.last_byte_at) {
            LinkState::NoBytes
        } else if !recent(self.last_nmea_line_at) {
            LinkState::NoSentences
        } else if self.rejected_since_fix > 0 {
            LinkState::FixesRejected
        } else {
            LinkState::NoFix
        }
    }

    /// Decide whether to report, given the time since the port opened.
    ///
    /// Returns at most one report per call: the first once the condition has
    /// outlived its grace period, then one per [`REPEAT_INTERVAL`] while it
    /// persists, then a single `Recovered` when fixes resume.
    pub fn poll(&mut self, now: Duration) -> Option<LinkReport> {
        let state = self.classify(now);
        if state == LinkState::Healthy {
            return self.reported.take().map(|was| {
                // Clear the repeat timer too, so a later episode reports
                // immediately once its own grace elapses instead of waiting
                // out this one's interval.
                self.last_report_at = None;
                LinkReport::Recovered {
                    was,
                    counts: self.counts,
                }
            });
        }

        let silent_for = now.saturating_sub(self.last_fix_at.unwrap_or(Duration::ZERO));
        if silent_for < state.grace() {
            return None;
        }
        if let Some(last) = self.last_report_at {
            if now.saturating_sub(last) < REPEAT_INTERVAL {
                return None;
            }
        }
        self.last_report_at = Some(now);
        self.reported = Some(state);
        Some(LinkReport::Trouble(LinkDiagnosis {
            state,
            counts: self.counts,
            silent_for,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed(codec: &mut NmeaLineCodec, buf: &mut BytesMut, bytes: &[u8]) -> Vec<NmeaLine> {
        buf.extend_from_slice(bytes);
        let mut out = Vec::new();
        while let Some(item) = codec.decode(buf).expect("codec never errors on data") {
            out.push(item);
        }
        out
    }

    fn s(line: &str) -> NmeaLine {
        NmeaLine::Line(line.to_string())
    }

    // ---- codec ----

    #[test]
    fn splits_lines_and_strips_cr() {
        let mut c = NmeaLineCodec::new();
        let mut b = BytesMut::new();
        let out = feed(&mut c, &mut b, b"$GPGGA,1\r\n$GPRMC,2\n");
        assert_eq!(out, vec![s("$GPGGA,1"), s("$GPRMC,2")]);
    }

    #[test]
    fn partial_line_is_buffered_until_terminated() {
        let mut c = NmeaLineCodec::new();
        let mut b = BytesMut::new();
        assert!(feed(&mut c, &mut b, b"$GPGGA,partial").is_empty());
        assert_eq!(
            feed(&mut c, &mut b, b"-rest\n"),
            vec![s("$GPGGA,partial-rest")]
        );
    }

    #[test]
    fn counts_bytes_still_sitting_in_a_partial_line() {
        // The claim "not a single byte arrived" must be truthful, so bytes are
        // counted on arrival, not when a line completes.
        let mut c = NmeaLineCodec::new();
        let mut b = BytesMut::new();
        assert!(feed(&mut c, &mut b, b"$GPGGA,no-terminator").is_empty());
        assert_eq!(c.bytes_seen(), 20);
        feed(&mut c, &mut b, b"\n");
        assert_eq!(c.bytes_seen(), 21, "consuming a line must not re-count it");
    }

    #[test]
    fn non_utf8_line_is_reported_not_silently_dropped() {
        let mut c = NmeaLineCodec::new();
        let mut b = BytesMut::new();
        let out = feed(
            &mut c,
            &mut b,
            &[0xff, 0xfe, b'\n', b'$', b'o', b'k', b'\n'],
        );
        assert_eq!(out, vec![NmeaLine::NonUtf8(2), s("$ok")]);
    }

    #[test]
    fn overlong_buffer_is_discarded_and_framing_resyncs() {
        let mut c = NmeaLineCodec::new();
        let mut b = BytesMut::new();
        let junk = vec![b'x'; NMEA_MAX_LINE_BYTES + 1];
        let out = feed(&mut c, &mut b, &junk);
        assert_eq!(out, vec![NmeaLine::Overlong(NMEA_MAX_LINE_BYTES + 1)]);
        // The port is NOT torn down; the next terminated line decodes normally.
        assert_eq!(
            feed(&mut c, &mut b, b"$GPGGA,back\n"),
            vec![s("$GPGGA,back")]
        );
    }

    #[test]
    fn eof_drains_complete_lines_then_drops_an_unterminated_tail() {
        // The `Decoder` default would turn the tail into `Err`, which this
        // codec reserves for "the port is gone". A module unplugged
        // mid-sentence must not be reported as an I/O failure.
        let mut c = NmeaLineCodec::new();
        let mut b = BytesMut::new();
        b.extend_from_slice(b"$GPGGA,done\n$GPRMC,trunc");
        assert_eq!(
            c.decode_eof(&mut b).expect("eof must not error"),
            Some(s("$GPGGA,done"))
        );
        assert_eq!(
            c.decode_eof(&mut b).expect("eof must not error"),
            None,
            "the truncated tail is incomplete data, not a line and not an error"
        );
        assert!(b.is_empty(), "tail discarded");
        // The tail's bytes still count, so the diagnosis keeps its evidence.
        assert_eq!(c.bytes_seen(), 24);
    }

    #[test]
    fn buffer_never_grows_past_the_cap_under_a_newline_less_flood() {
        // The OOM guard: a peer that never sends `\n` must not grow the buffer.
        let mut c = NmeaLineCodec::new();
        let mut b = BytesMut::new();
        for _ in 0..64 {
            feed(&mut c, &mut b, &vec![b'x'; 256]);
            assert!(
                b.len() <= NMEA_MAX_LINE_BYTES,
                "buffer grew to {} bytes",
                b.len()
            );
        }
        assert_eq!(c.bytes_seen(), 64 * 256);
    }

    // ---- health policy ----

    fn secs(n: u64) -> Duration {
        Duration::from_secs(n)
    }

    /// Drive a healthy link: bytes, sentences, and a fix at each step.
    fn healthy_tick(h: &mut LinkHealth, t: Duration, total_bytes: u64) {
        h.observe_bytes(total_bytes, t);
        h.on_event(&s("$GPGGA,x"), t);
        h.on_fix(t);
    }

    #[test]
    fn healthy_link_never_reports() {
        let mut h = LinkHealth::new();
        for i in 0..300u64 {
            let t = secs(i);
            healthy_tick(&mut h, t, (i + 1) * 100);
            assert_eq!(h.poll(t), None, "healthy link must stay quiet at t={i}s");
        }
    }

    #[test]
    fn dead_wire_is_diagnosed_as_no_bytes_after_the_grace() {
        let mut h = LinkHealth::new();
        // Nothing at all arrives. Silence before the grace is not yet news.
        assert_eq!(h.poll(secs(9)), None);
        let Some(LinkReport::Trouble(d)) = h.poll(QUIET_LINK_GRACE) else {
            panic!("expected a diagnosis at the grace boundary");
        };
        assert_eq!(d.state, LinkState::NoBytes);
        assert_eq!(d.counts.bytes, 0);
        assert!(!d.fixes_stopped());
        assert!(d.state.advice().contains("TX/RX"));
    }

    #[test]
    fn binary_module_is_diagnosed_as_no_sentences_not_no_bytes() {
        // A u-blox in UBX mode: plenty of bytes, no `$` lines. The distinction
        // is the whole point — "no bytes" would send the operator to the wiring.
        let mut h = LinkHealth::new();
        let mut total = 0;
        for i in 0..=QUIET_LINK_GRACE.as_secs() {
            total += 600;
            h.observe_bytes(total, secs(i));
            h.on_event(&NmeaLine::Overlong(600), secs(i));
        }
        let Some(LinkReport::Trouble(d)) = h.poll(QUIET_LINK_GRACE) else {
            panic!("expected a diagnosis");
        };
        assert_eq!(d.state, LinkState::NoSentences);
        assert!(d.counts.overlong_discards > 0);
        assert!(d.state.advice().contains("UBX"));
        assert!(d.state.advice().contains("baud"));
    }

    #[test]
    fn baud_mismatch_shows_up_as_non_utf8_evidence() {
        let mut h = LinkHealth::new();
        for i in 0..=QUIET_LINK_GRACE.as_secs() {
            h.observe_bytes((i + 1) * 200, secs(i));
            h.on_event(&NmeaLine::NonUtf8(40), secs(i));
        }
        let Some(LinkReport::Trouble(d)) = h.poll(QUIET_LINK_GRACE) else {
            panic!("expected a diagnosis");
        };
        assert_eq!(d.state, LinkState::NoSentences);
        assert!(
            d.counts.non_utf8_lines > 0,
            "evidence is carried in the log"
        );
    }

    #[test]
    fn cold_start_is_tolerated_then_diagnosed_as_no_fix() {
        // Sentences flow from the start but carry no fix. This is NORMAL for
        // ~a minute, so it must stay quiet well past the short grace.
        let mut h = LinkHealth::new();
        let check = |h: &mut LinkHealth, t: u64| {
            h.observe_bytes(t * 500 + 500, secs(t));
            h.on_event(&s("$GPGGA,no-fix"), secs(t));
            h.poll(secs(t))
        };
        for t in 0..NO_FIX_GRACE.as_secs() {
            assert_eq!(check(&mut h, t), None, "must not cry wolf at t={t}s");
        }
        let Some(LinkReport::Trouble(d)) = check(&mut h, NO_FIX_GRACE.as_secs()) else {
            panic!("expected a diagnosis once the cold-start fuse burns out");
        };
        assert_eq!(d.state, LinkState::NoFix);
        assert!(!d.fixes_stopped(), "never acquired, so nothing 'stopped'");
        assert!(d.state.advice().contains("antenna"));
    }

    #[test]
    fn gate_rejecting_everything_is_reported_promptly() {
        // Not a cold start — the module IS producing fixes and the security
        // gate is throwing all of them away. Short fuse, distinct state.
        let mut h = LinkHealth::new();
        for i in 0..=QUIET_LINK_GRACE.as_secs() {
            h.observe_bytes((i + 1) * 500, secs(i));
            h.on_event(&s("$GPGGA,x"), secs(i));
            h.on_fix_rejected();
        }
        let Some(LinkReport::Trouble(d)) = h.poll(QUIET_LINK_GRACE) else {
            panic!("expected a diagnosis");
        };
        assert_eq!(d.state, LinkState::FixesRejected);
        assert_eq!(d.counts.rejected_fixes, QUIET_LINK_GRACE.as_secs() + 1);
    }

    #[test]
    fn a_fix_clears_stale_rejections() {
        // One accepted fix must retire the rejection evidence, or a single
        // early bad fix would mislabel every later quiet period.
        let mut h = LinkHealth::new();
        h.observe_bytes(100, secs(0));
        h.on_event(&s("$GPGGA,x"), secs(0));
        h.on_fix_rejected();
        h.on_fix(secs(1));
        // Sentences keep flowing, fixes stop. That is NoFix, not FixesRejected.
        for i in 2..=NO_FIX_GRACE.as_secs() + 1 {
            h.observe_bytes(100 + i * 100, secs(i));
            h.on_event(&s("$GPGGA,x"), secs(i));
        }
        let Some(LinkReport::Trouble(d)) = h.poll(secs(NO_FIX_GRACE.as_secs() + 1)) else {
            panic!("expected a diagnosis");
        };
        assert_eq!(d.state, LinkState::NoFix);
        assert!(
            d.fixes_stopped(),
            "this link DID deliver before going quiet"
        );
    }

    #[test]
    fn repeats_are_rate_limited_then_recovery_closes_the_episode() {
        let mut h = LinkHealth::new();
        assert!(matches!(
            h.poll(QUIET_LINK_GRACE),
            Some(LinkReport::Trouble(_))
        ));
        // Silent until the repeat interval elapses...
        let mut t = QUIET_LINK_GRACE;
        for _ in 0..(REPEAT_INTERVAL.as_secs() - 1) {
            t += secs(1);
            assert_eq!(h.poll(t), None, "no spam at t={t:?}");
        }
        t += secs(1);
        assert!(
            matches!(h.poll(t), Some(LinkReport::Trouble(_))),
            "condition persists, so it must resurface once per interval"
        );
        // ...and recovery is announced exactly once.
        t += secs(1);
        healthy_tick(&mut h, t, 10_000);
        let Some(LinkReport::Recovered { was, .. }) = h.poll(t) else {
            panic!("expected a recovery report");
        };
        assert_eq!(was, LinkState::NoBytes);
        t += secs(1);
        healthy_tick(&mut h, t, 11_000);
        assert_eq!(h.poll(t), None, "recovery is announced once, not forever");
    }

    #[test]
    fn a_new_episode_reports_without_waiting_out_the_old_repeat_timer() {
        let mut h = LinkHealth::new();
        assert!(matches!(
            h.poll(QUIET_LINK_GRACE),
            Some(LinkReport::Trouble(_))
        ));
        let mut t = QUIET_LINK_GRACE + secs(1);
        healthy_tick(&mut h, t, 5_000);
        assert!(matches!(h.poll(t), Some(LinkReport::Recovered { .. })));
        // Link dies again immediately. The next diagnosis must wait only for
        // its own grace, not for the previous episode's repeat interval.
        t += QUIET_LINK_GRACE;
        let Some(LinkReport::Trouble(d)) = h.poll(t) else {
            panic!("a fresh episode must report on its own grace");
        };
        assert_eq!(d.state, LinkState::NoBytes);
        assert!(d.fixes_stopped());
    }
}
