//! MAVLink integration: real-protocol sources and a real-protocol
//! `FlightController`. Wire-compatible with ArduPilot/PX4 SITL — pointing
//! this at a real autopilot is just an endpoint change.
//!
//! Architecture: one bidirectional UDP socket per peer. We bind to
//! `local_bind` and send commands to `remote_addr`. The `MavLink` task
//! drives the socket, demultiplexes incoming messages into per-stream
//! channels (`GPS_RAW_INT` -> gps_tx, `HIGHRES_IMU` -> imu_tx), and serves
//! outgoing commands posted to a single command channel.

pub mod controller;
pub mod monitor;
pub mod source;

use crate::error::FsError;
use crate::types::{GpsFix, ImuSample, Timestamp};
use mavlink::common::MavMessage;
use mavlink::peek_reader::PeekReader;
use mavlink::{MavHeader, MavlinkVersion};
use monitor::MavMonitor;
use std::io::Cursor;
use std::net::{IpAddr, SocketAddr};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use tokio::net::UdpSocket;
use tokio::sync::{mpsc, Mutex};
use tokio::task::JoinHandle;
use tokio::time::Instant;
use tracing::{debug, warn};

/// Inbound-packet filter for the MAVLink listener.
///
/// `allowed_source_ip = Some(ip)` rejects any datagram not from `ip` —
/// prevents off-host injection on a multihomed or shared LAN. `None` =
/// promiscuous (only safe for loopback or tightly-controlled lab setups).
///
/// `autopilot_sysid = Some(id)` rejects MAVLink messages whose
/// `header.system_id != id` — prevents a hostile ground station on the
/// same network from feeding us GPS/IMU.
///
/// `allow_any_source_port = true` relaxes the bootstrap source-PORT check:
/// the lock then pins on source IP alone, accepting the autopilot's first
/// message from ANY source port. Needed for bridges that forward MAVLink from
/// an ephemeral port (mavproxy `--out udp:`, many SiK/mavlink-router setups),
/// where the strict default would silently reject every packet and the
/// detector would go deaf. It weakens the co-located-attacker bootstrap
/// defense, so it is a CLI-only opt-in (TOML-rejected like the other safety
/// bypasses) and the listener warns loudly when the strict check is the reason
/// nothing is being accepted.
#[derive(Debug, Clone, Copy, Default)]
pub struct MavFilter {
    pub allowed_source_ip: Option<IpAddr>,
    pub autopilot_sysid: Option<u8>,
    pub allow_any_source_port: bool,
}

/// Observability counters for the listener — exposed for operator dashboards
/// and test assertions. All fields are atomic so they can be read from any
/// task without locking.
#[derive(Debug, Default)]
pub struct MavStats {
    pub recv_total: AtomicU64,
    pub dropped_source_ip: AtomicU64,
    pub dropped_sysid: AtomicU64,
    pub dropped_oversize: AtomicU64,
    pub dropped_nonfinite: AtomicU64,
    pub dropped_source_locked: AtomicU64,
    pub parse_errors: AtomicU64,
}

/// One MAVLink connection, full-duplex.
///
/// The listener task is the sole reader of the socket and pushes parsed
/// messages into typed channels. The controller path is the sole writer,
/// guarded by an async mutex (UDP `send_to` is itself thread-safe but the
/// sequence counter is not).
pub struct MavLink {
    pub socket: Arc<UdpSocket>,
    pub remote: SocketAddr,
    pub seq: Arc<Mutex<u8>>,
    pub filter: MavFilter,
    pub stats: Arc<MavStats>,
    pub monitor: Arc<MavMonitor>,
    /// Boot instant used to compute relative timestamps stored in the monitor.
    /// Must match `Fusion`'s boot anchor — both share `Instant::now()` at
    /// `spawn_all` time in practice.
    pub boot: Instant,
}

impl MavLink {
    /// Bind a UDP socket with no inbound filtering. Useful for tests and
    /// loopback. Production callers should use [`bind_with_filter`].
    pub async fn bind(local: SocketAddr, remote: SocketAddr) -> Result<Self, FsError> {
        Self::bind_with_filter(local, remote, MavFilter::default()).await
    }

    /// Bind a UDP socket and apply the given ingress filter to every packet.
    pub async fn bind_with_filter(
        local: SocketAddr,
        remote: SocketAddr,
        filter: MavFilter,
    ) -> Result<Self, FsError> {
        let socket = UdpSocket::bind(local).await?;
        if local.ip().is_unspecified() && filter.allowed_source_ip.is_none() {
            warn!(
                bind = %local,
                "MAVLink listener bound to wildcard with NO source-IP filter — this accepts \
                 packets from any host on the network. Production callers must pass an \
                 allowed_source_ip filter or bind to 127.0.0.1."
            );
        }
        Ok(Self {
            socket: Arc::new(socket),
            remote,
            seq: Arc::new(Mutex::new(0)),
            filter,
            stats: Arc::new(MavStats::default()),
            monitor: Arc::new(MavMonitor::new()),
            boot: Instant::now(),
        })
    }

    /// Spawn the listener task. Returns:
    ///   - join handle for the listener
    ///   - mpsc::Receiver<GpsFix> (drain via the `MavGpsSource`)
    ///   - mpsc::Receiver<ImuSample> (drain via the `MavImuSource`)
    ///   - mpsc::Receiver<MavMessage> for any unhandled messages (useful for diagnostics)
    pub fn spawn_listener(
        &self,
    ) -> (
        JoinHandle<()>,
        mpsc::Receiver<GpsFix>,
        mpsc::Receiver<ImuSample>,
        mpsc::Receiver<MavMessage>,
    ) {
        let (gps_tx, gps_rx) = mpsc::channel::<GpsFix>(64);
        let (imu_tx, imu_rx) = mpsc::channel::<ImuSample>(256);
        let (other_tx, other_rx) = mpsc::channel::<MavMessage>(32);
        let sock = self.socket.clone();
        let filter = self.filter;
        let stats = self.stats.clone();
        let monitor = self.monitor.clone();
        let boot = self.boot;
        // Expected source port = the port the configured remote address is
        // sending FROM. For a directly-connected autopilot, that's
        // `self.remote.port()` — the port we send commands TO. Enforced at
        // bootstrap to prevent a co-located attacker from claiming the
        // source-lock first. `None` (operator opt-in) locks on IP alone, for
        // bridges that forward from an ephemeral source port.
        let expected_port_opt: Option<u16> = if self.filter.allow_any_source_port {
            None
        } else {
            Some(self.remote.port())
        };
        let strict_port = expected_port_opt.is_some();
        let handle = tokio::spawn(async move {
            let mut buf = vec![0u8; 2048];
            // Rate-limiter for the "deaf because of the strict source-port
            // check" warning, so a 1 Hz autopilot HEARTBEAT from the wrong
            // port doesn't spam the log every second.
            let mut last_port_deaf_warn: Option<Instant> = None;
            // Per-stream clock aligners map the autopilot's own sensor
            // timestamps onto our monotonic clock so link jitter doesn't shift
            // GPS vs IMU and inject residual. They fall back to arrival time on
            // any anomaly (see `ClockAligner`).
            let mut gps_aligner = ClockAligner::default();
            let mut imu_aligner = ClockAligner::default();
            loop {
                let (n, src) = match sock.recv_from(&mut buf).await {
                    Ok(v) => v,
                    Err(e) => {
                        // Windows surfaces ICMP "port unreachable" (10054) as a recv
                        // error on UDP. That's benign — a peer wasn't listening yet
                        // when we sent to them. Don't kill the listener over it.
                        if is_transient_udp_error(&e) {
                            warn!(error = %e, "transient UDP recv error, continuing");
                            continue;
                        }
                        warn!(error = %e, "fatal UDP recv error, exiting listener");
                        break;
                    }
                };
                stats.recv_total.fetch_add(1, Ordering::Relaxed);

                // AUDIT X-02: `buf` is a persistent 2048-byte Vec reused every
                // iteration, so `buf.first()` is NEVER None even on a 0-length
                // UDP datagram — it would return a STALE byte from a prior
                // packet and mis-pick the MAVLink version. Work strictly with
                // the `n` received bytes. A datagram too short to hold even a
                // MAVLink header (smallest v1 frame is 8 bytes) can't be a
                // valid message — drop it before touching the version byte.
                let datagram = &buf[..n];
                if datagram.len() < 8 {
                    stats.parse_errors.fetch_add(1, Ordering::Relaxed);
                    debug!(src = %src, len = n, "dropped mavlink: datagram too short for a frame");
                    continue;
                }

                // Source-IP filter (security boundary — drop before any parsing).
                if let Some(allowed) = filter.allowed_source_ip {
                    if src.ip() != allowed {
                        stats.dropped_source_ip.fetch_add(1, Ordering::Relaxed);
                        debug!(src = %src, "dropped mavlink packet: source IP not allowed");
                        continue;
                    }
                }

                // Zero-copy parse over the received slice. AUDIT V-MAVVER
                // (found via ArduPilot SITL): the listener originally hard-
                // coded MavlinkVersion::V2 (0xFD magic). ArduPilot — and many
                // other autopilots — emit MAVLink **v1** (0xFE) by DEFAULT, so
                // a V2-only parser silently rejects every frame and the
                // detector goes completely deaf on a real v1 link. Pick the
                // version from the leading magic byte (see `mavlink_version_of`).
                let version = mavlink_version_of(datagram[0]);
                let mut reader = PeekReader::new(Cursor::new(datagram));
                match mavlink::read_versioned_msg::<MavMessage, _>(&mut reader, version) {
                    Ok((hdr, msg)) => {
                        // System-id filter (security boundary — drop before any use).
                        if let Some(want) = filter.autopilot_sysid {
                            if hdr.system_id != want {
                                stats.dropped_sysid.fetch_add(1, Ordering::Relaxed);
                                debug!(
                                    src = %src,
                                    sysid = hdr.system_id,
                                    expected = want,
                                    "dropped mavlink packet: system_id mismatch"
                                );
                                continue;
                            }
                        }

                        // Update autopilot-state monitor on HEARTBEAT / COMMAND_ACK
                        // BEFORE dispatching to typed channels — these are observability
                        // side effects, not data flow.
                        //
                        // Source-port lock: the FIRST accepted HEARTBEAT pins
                        // (src_ip, src_port) for the rest of the process lifetime.
                        // After that, any packet from a different source is rejected
                        // — closes the bootstrap race where a co-located attacker
                        // could plant the first heartbeat before the real autopilot.
                        match &msg {
                            MavMessage::HEARTBEAT(hb) => {
                                if !monitor.check_or_lock_source(src, expected_port_opt) {
                                    stats.dropped_source_locked.fetch_add(1, Ordering::Relaxed);
                                    // Distinguish two rejection causes. If no lock
                                    // has formed yet AND we're enforcing the strict
                                    // port, this is the "deaf detector" failure: the
                                    // autopilot IS heartbeating but from an
                                    // unexpected source port (mavproxy `--out udp:`,
                                    // a SiK/mavlink-router bridge), so we refuse to
                                    // lock and silently ingest nothing. Make it LOUD
                                    // and actionable instead of a debug drop.
                                    if strict_port && monitor.locked_source().is_none() {
                                        let warn_now = last_port_deaf_warn
                                            .map(|t| {
                                                Instant::now().saturating_duration_since(t)
                                                    >= std::time::Duration::from_secs(5)
                                            })
                                            .unwrap_or(true);
                                        if warn_now {
                                            last_port_deaf_warn = Some(Instant::now());
                                            warn!(
                                                src = %src,
                                                expected_port = ?expected_port_opt,
                                                "MAVLink HEARTBEAT REJECTED: peer is sending from an \
                                                 unexpected source port and the source-lock has not \
                                                 formed — the detector is receiving NOTHING. If you use \
                                                 a bridge that forwards from an ephemeral port (mavproxy \
                                                 `--out udp:`, mavlink-router, SiK), either switch it to \
                                                 `--out udpin:` so it sends from the expected port, or \
                                                 pass --mav-allow-any-source-port to lock on IP alone."
                                            );
                                        }
                                    } else {
                                        debug!(src = %src, locked = ?monitor.locked_source(),
                                            "dropped mavlink: source-locked to a different peer");
                                    }
                                    continue;
                                }
                                let _changed = monitor.record_heartbeat(
                                    Instant::now(),
                                    boot,
                                    hb.custom_mode,
                                    hb.base_mode.bits(),
                                    hb.system_status as u8,
                                    hb.autopilot as u8,
                                );
                            }
                            MavMessage::COMMAND_ACK(ack) => {
                                if !monitor.check_or_lock_source(src, expected_port_opt) {
                                    stats.dropped_source_locked.fetch_add(1, Ordering::Relaxed);
                                    continue;
                                }
                                monitor.record_ack(
                                    Instant::now(),
                                    boot,
                                    ack.command as u16,
                                    ack.result as u16,
                                );
                            }
                            MavMessage::PARAM_VALUE(pv) => {
                                // PARAM_VALUE is the autopilot's echo that a
                                // parameter was applied. Recording it lets
                                // `sever_gps` CONFIRM the GPS-disable param took
                                // effect instead of trusting best-effort UDP.
                                // Subject to the same source lock as the other
                                // monitor messages.
                                if !monitor.check_or_lock_source(src, expected_port_opt) {
                                    stats.dropped_source_locked.fetch_add(1, Ordering::Relaxed);
                                    continue;
                                }
                                monitor.record_param_value(
                                    Instant::now(),
                                    boot,
                                    &pv.param_id,
                                    pv.param_value,
                                    pv.param_type as u8,
                                );
                            }
                            _ => {
                                // For non-monitor messages, still enforce the lock
                                // if it's been set. Pre-lock, source-IP filter is
                                // the only gate.
                                if !monitor.check_or_lock_source(src, expected_port_opt) {
                                    stats.dropped_source_locked.fetch_add(1, Ordering::Relaxed);
                                    continue;
                                }
                            }
                        }

                        if let Some(mut fix) = gps_from_msg(&msg) {
                            // Replace the parse-time arrival stamp with a
                            // sensor-time-aligned monotonic instant so link
                            // jitter doesn't move the GPS↔IMU residual alignment.
                            fix.t.mono = gps_aligner.align(gps_sensor_us(&msg), Instant::now());
                            // Reject non-finite / out-of-range values (defense in depth).
                            if !gps_fix_is_plausible(&fix) {
                                stats.dropped_nonfinite.fetch_add(1, Ordering::Relaxed);
                                debug!(src = %src, "dropped mavlink GPS: implausible values");
                                continue;
                            }
                            if gps_tx.send(fix).await.is_err() {
                                break;
                            }
                        } else if let Some(mut sample) = imu_from_msg(&msg) {
                            sample.t.mono = imu_aligner.align(imu_sensor_us(&msg), Instant::now());
                            if !imu_sample_is_plausible(&sample) {
                                stats.dropped_nonfinite.fetch_add(1, Ordering::Relaxed);
                                debug!(src = %src, "dropped mavlink IMU: implausible values");
                                continue;
                            }
                            if imu_tx.send(sample).await.is_err() {
                                break;
                            }
                        } else {
                            // Diagnostic stream — drop on full.
                            let _ = other_tx.try_send(msg);
                        }
                    }
                    Err(e) => {
                        stats.parse_errors.fetch_add(1, Ordering::Relaxed);
                        debug!(error = ?e, "mavlink parse failed");
                    }
                }
            }
        });
        (handle, gps_rx, imu_rx, other_rx)
    }

    /// Read-only access to listener stats. Safe to call from any task.
    pub fn stats(&self) -> Arc<MavStats> {
        self.stats.clone()
    }

    /// Spawn the heartbeat watchdog. Emits a `LinkDown` event when no
    /// HEARTBEAT has been observed for `staleness` time; emits `LinkRestored`
    /// when one returns. Also emits `PilotModeChange` whenever the
    /// autopilot's `custom_mode` transitions for any reason — so operators
    /// can distinguish "our RTL succeeded" from "autopilot landed because of
    /// low battery / pilot manual takeover / geofence."
    ///
    /// Polls every `poll`. `boot` must match the boot instant the events use
    /// for `mono_ns` so timestamps line up.
    pub fn spawn_heartbeat_watchdog(
        &self,
        event_tx: tokio::sync::broadcast::Sender<crate::types::SpoofingEvent>,
        boot: Instant,
        staleness: std::time::Duration,
        poll: std::time::Duration,
    ) -> JoinHandle<()> {
        let monitor = self.monitor.clone();
        let staleness_ns = staleness.as_nanos() as u64;
        tokio::spawn(async move {
            let mut ticker = tokio::time::interval(poll);
            ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            let mut last_state_up = false;
            let mut have_seen_heartbeat = false;
            let mut last_mode_change_ns: u64 = 0;
            loop {
                ticker.tick().await;
                let now = Instant::now();
                // Don't fire LinkDown until we've seen at least one heartbeat —
                // otherwise startup time would spam events before the autopilot
                // even comes online.
                if !have_seen_heartbeat {
                    if monitor.heartbeat_fresh(now, boot, staleness_ns) {
                        have_seen_heartbeat = true;
                        last_state_up = true;
                        last_mode_change_ns = monitor.mode_changed_at_ns();
                        // First heartbeat — operator wants to know we're connected.
                        emit_link_event(&event_tx, boot, true, monitor.current_mode());
                    }
                    continue;
                }
                let up = monitor.heartbeat_fresh(now, boot, staleness_ns);
                if up != last_state_up {
                    last_state_up = up;
                    emit_link_event(&event_tx, boot, up, monitor.current_mode());
                }
                // Mode-change emission. Edge-triggered on the monitor's
                // mode_changed_at_ns. Watchdog is the right home for this
                // because it already runs at the same poll cadence as
                // verify_rtb_engaged, so the operator sees mode transitions
                // independently of who caused them (pilot, autopilot failsafe,
                // OR our RTL — operator can correlate timestamps).
                let mc_ns = monitor.mode_changed_at_ns();
                if mc_ns > last_mode_change_ns {
                    last_mode_change_ns = mc_ns;
                    emit_mode_change_event(&event_tx, boot, monitor.current_mode());
                }
            }
        })
    }

    /// Encode and send one MAVLink message to the remote endpoint.
    pub async fn send(&self, msg: &MavMessage) -> Result<(), FsError> {
        let mut buf: Vec<u8> = Vec::with_capacity(280);
        let header = {
            let mut g = self.seq.lock().await;
            let h = MavHeader {
                system_id: 255, // GCS sysid
                component_id: 0,
                sequence: *g,
            };
            *g = g.wrapping_add(1);
            h
        };
        mavlink::write_versioned_msg(&mut buf, MavlinkVersion::V2, header, msg)
            .map_err(|e| FsError::Io(std::io::Error::other(format!("mavlink encode: {:?}", e))))?;
        self.socket.send_to(&buf, self.remote).await?;
        Ok(())
    }
}

/// MAVLink "unknown" sentinel for u16 fields in GPS_RAW_INT (vel/cog/eph/epv).
/// Per the MAVLink common dialect spec, these fields are set to UINT16_MAX
/// when the autopilot doesn't have a value to report.
const GPS_U16_UNKNOWN: u16 = u16::MAX;
/// MAVLink "unknown" sentinel for satellites_visible (u8).
const GPS_U8_UNKNOWN: u8 = u8::MAX;

/// Pick the MAVLink wire version from a frame's start-of-frame magic byte.
/// `0xFD` = v2, `0xFE` = v1 (the historical STX). Anything else isn't a
/// MAVLink frame start; we default to v2 so the reader attempts a parse and
/// records a parse error rather than silently guessing. Audit V-MAVVER:
/// hard-coding v2 made the detector deaf to ArduPilot's default v1 stream.
pub fn mavlink_version_of(magic: u8) -> MavlinkVersion {
    match magic {
        0xFD => MavlinkVersion::V2,
        0xFE => MavlinkVersion::V1,
        _ => MavlinkVersion::V2,
    }
}

/// Convert an inbound MAVLink message to a `GpsFix`, if it carries one.
///
/// CRITICAL: MAVLink GPS_RAW_INT uses UINT16_MAX as a sentinel for "field is
/// unknown" on `vel`, `cog`, `eph`, `epv`. Naively scaling these by 1e-2
/// produces 655.35 — a value that masquerades as a real measurement and
/// poisons the detector:
///   * `vel = UINT16_MAX` → 655 m/s reported speed → constant velocity-mismatch
///     fires on every fix (false-positive storm)
///   * `cog = UINT16_MAX` → 655° course → garbage NED velocity vector
///   * `eph = UINT16_MAX` → 655 m HDOP → above multipath threshold → both jump
///     and drift detectors widen their gates 1.5-2× (false-negative storm,
///     also an attacker bypass if they spoof eph deliberately)
///
/// We translate sentinels to `None` so the detector handles "unknown" as
/// "missing evidence" (the dvel_known=false path) rather than as a real value.
pub fn gps_from_msg(msg: &MavMessage) -> Option<GpsFix> {
    if let MavMessage::GPS_RAW_INT(g) = msg {
        // fix_type >= 2 = 2D fix; >= 3 = 3D fix.
        let fix_type_ok = (g.fix_type as u8) >= 2;
        if !fix_type_ok {
            return None;
        }
        // GPS speed: drop sentinel, otherwise convert cm/s → m/s.
        let speed_mps = if g.vel == GPS_U16_UNKNOWN {
            None
        } else {
            Some(g.vel as f64 * 1e-2)
        };
        // Course: 0..35999 cdeg, sentinel = UINT16_MAX. Anything > 35999 is
        // invalid (even pre-sentinel) → reject.
        let course_deg = if g.cog == GPS_U16_UNKNOWN || g.cog > 35999 {
            None
        } else {
            Some(g.cog as f64 * 1e-2)
        };
        // HDOP: sentinel = UINT16_MAX. Translate to None so the detector
        // treats it as "no quality info" rather than "very bad quality"
        // (which would widen detector thresholds and reduce sensitivity).
        let hdop = if g.eph == GPS_U16_UNKNOWN {
            None
        } else {
            Some(g.eph as f32 * 1e-2)
        };
        // Sats visible: u8 sentinel = 255. Real values fit comfortably in
        // u8 (max GNSS visibility is ~50 across all constellations).
        let sats = if g.satellites_visible == GPS_U8_UNKNOWN {
            None
        } else {
            Some(g.satellites_visible)
        };
        Some(GpsFix {
            t: Timestamp::now_mono(),
            lat_deg: g.lat as f64 * 1e-7,
            lon_deg: g.lon as f64 * 1e-7,
            alt_m: g.alt as f64 * 1e-3,
            speed_mps,
            course_deg,
            hdop,
            sats,
        })
    } else {
        None
    }
}

/// Standard gravity, m/s² — converts SCALED_IMU's milli-g accel to m/s².
const G_MPS2: f32 = 9.806_65;

/// Convert an inbound MAVLink message to an `ImuSample`, if it carries one.
///
/// AUDIT V-IMU (Phase V, found via ArduPilot SITL): the detector originally
/// accepted ONLY `HIGHRES_IMU` (message 105). That is a PX4-native message —
/// **ArduPilot does NOT emit it by default**; ArduPilot streams `SCALED_IMU`
/// / `SCALED_IMU2` / `SCALED_IMU3` (messages 26/116/129). So `--imu-source
/// mav` against a real ArduCopter received ZERO IMU samples — preflight would
/// hang forever waiting for IMU init and the cross-correlation would never
/// run. This was invisible to the `mavsim` test because mavsim emitted
/// HIGHRES_IMU to match our own parser (a circular test).
///
/// Unit conversions (per the MAVLink common dialect):
///   * HIGHRES_IMU — accel already m/s², gyro already rad/s. Direct.
///   * SCALED_IMU* — accel in milli-g (int16), gyro in milli-rad/s (int16).
///     accel_mps2 = mG * 1e-3 * g ;  gyro_rps = mrad/s * 1e-3.
pub fn imu_from_msg(msg: &MavMessage) -> Option<ImuSample> {
    match msg {
        MavMessage::HIGHRES_IMU(i) => Some(ImuSample {
            t: Timestamp::now_mono(),
            accel_mps2: [i.xacc, i.yacc, i.zacc],
            gyro_rps: [i.xgyro, i.ygyro, i.zgyro],
        }),
        // ArduPilot's primary IMU telemetry. mG → m/s², mrad/s → rad/s.
        MavMessage::SCALED_IMU(i) => Some(ImuSample {
            t: Timestamp::now_mono(),
            accel_mps2: [
                i.xacc as f32 * 1e-3 * G_MPS2,
                i.yacc as f32 * 1e-3 * G_MPS2,
                i.zacc as f32 * 1e-3 * G_MPS2,
            ],
            gyro_rps: [
                i.xgyro as f32 * 1e-3,
                i.ygyro as f32 * 1e-3,
                i.zgyro as f32 * 1e-3,
            ],
        }),
        MavMessage::SCALED_IMU2(i) => Some(ImuSample {
            t: Timestamp::now_mono(),
            accel_mps2: [
                i.xacc as f32 * 1e-3 * G_MPS2,
                i.yacc as f32 * 1e-3 * G_MPS2,
                i.zacc as f32 * 1e-3 * G_MPS2,
            ],
            gyro_rps: [
                i.xgyro as f32 * 1e-3,
                i.ygyro as f32 * 1e-3,
                i.zgyro as f32 * 1e-3,
            ],
        }),
        MavMessage::SCALED_IMU3(i) => Some(ImuSample {
            t: Timestamp::now_mono(),
            accel_mps2: [
                i.xacc as f32 * 1e-3 * G_MPS2,
                i.yacc as f32 * 1e-3 * G_MPS2,
                i.zacc as f32 * 1e-3 * G_MPS2,
            ],
            gyro_rps: [
                i.xgyro as f32 * 1e-3,
                i.ygyro as f32 * 1e-3,
                i.zgyro as f32 * 1e-3,
            ],
        }),
        _ => None,
    }
}

/// Maximum |arrival − aligned| skew the clock aligner tolerates before it
/// distrusts the autopilot's sensor clock and re-anchors (falling back to
/// arrival time). Real link jitter / latency spikes are tens to a few hundred
/// ms; a skew beyond this means the sensor clock isn't tracking wall time
/// (different tick rate, reboot, or wrap), so we re-anchor rather than feed a
/// wild timestamp into the residual buffer.
const CLOCK_ALIGN_MAX_SKEW: std::time::Duration = std::time::Duration::from_secs(2);

/// Maps an autopilot per-stream sensor timestamp onto the local monotonic clock,
/// removing per-message ARRIVAL jitter from the GPS↔IMU residual alignment.
///
/// Both GPS (`GPS_RAW_INT.time_usec`) and IMU (`HIGHRES_IMU.time_usec` /
/// `SCALED_IMU*.time_boot_ms`) carry the autopilot's own sensor timestamps. The
/// detector previously stamped both at PARSE time (`now_mono()`), so any jitter
/// in the USB / UART / SiK / UDP path between the autopilot and the companion
/// computer shifted GPS and IMU relative to each other and injected residual
/// (~0.75 m per 50 ms of differential jitter at 15 m/s) — invisible on SITL
/// loopback, a real risk on a real link. This aligner anchors each stream on the
/// local arrival of its FIRST message and advances by the SENSOR-reported deltas
/// thereafter, so per-message jitter no longer moves the alignment; only one
/// constant offset (the first message's latency) remains, which the detector's
/// adaptive noise floor absorbs.
///
/// SAFE BY CONSTRUCTION: on any anomaly — no sensor timestamp, a regression
/// (reboot / wrap / reorder), or a skew from arrival beyond
/// `CLOCK_ALIGN_MAX_SKEW` (the sensor clock isn't tracking wall time) — it falls
/// back to arrival time, i.e. exactly the prior behavior. It is PER-STREAM
/// because the GPS and IMU clocks may use different bases (boot-relative vs UNIX
/// epoch); mapping each stream by its OWN deltas cancels the absolute base.
#[derive(Debug, Default)]
struct ClockAligner {
    /// (local arrival Instant, sensor µs) captured at the current anchor.
    anchor: Option<(Instant, u64)>,
}

impl ClockAligner {
    /// Local-monotonic Instant to stamp on a message whose autopilot sensor
    /// timestamp is `sensor_us` (None if absent) and that arrived at `arrival`.
    fn align(&mut self, sensor_us: Option<u64>, arrival: Instant) -> Instant {
        let Some(s) = sensor_us else {
            // No usable sensor timestamp → arrival time. Leave the anchor intact
            // so one missing stamp doesn't reset the established alignment.
            return arrival;
        };
        match self.anchor {
            None => {
                self.anchor = Some((arrival, s));
                arrival
            }
            Some((anchor_arrival, anchor_sensor)) => {
                if s < anchor_sensor {
                    // Sensor clock regressed (reboot / wrap / reorder) →
                    // re-anchor and use arrival for this message.
                    self.anchor = Some((arrival, s));
                    return arrival;
                }
                let sensor_delta = std::time::Duration::from_micros(s - anchor_sensor);
                // Bound the delta BEFORE the `Instant + Duration` add. `s` is an
                // attacker-controlled wire timestamp (GPS_RAW_INT.time_usec /
                // SCALED_IMU.time_boot_ms); a crafted huge value would make
                // `from_micros(s - anchor_sensor)` span centuries and overflow the
                // monotonic `Instant`, which PANICS (tokio's `Instant: Add`
                // delegates to std's panicking add) — a one-packet remote kill of
                // the listener task, i.e. of the whole detector. The aligned stamp
                // can legitimately sit at most `CLOCK_ALIGN_MAX_SKEW` ahead of
                // arrival (= anchor_arrival + real elapsed), so any larger delta is
                // a non-wall-time / garbage clock: re-anchor and use arrival. This
                // check runs before the arithmetic, so the add can never overflow.
                let real_elapsed = arrival.saturating_duration_since(anchor_arrival);
                if sensor_delta > real_elapsed + CLOCK_ALIGN_MAX_SKEW {
                    self.anchor = Some((arrival, s));
                    return arrival;
                }
                let aligned = anchor_arrival + sensor_delta;
                // The aligned-BEHIND-arrival direction (sensor ran slower than wall
                // clock / large latency) is still possible within the bound above;
                // re-anchor if it drifts more than the skew bound back, too.
                if arrival.saturating_duration_since(aligned) > CLOCK_ALIGN_MAX_SKEW {
                    self.anchor = Some((arrival, s));
                    return arrival;
                }
                aligned
            }
        }
    }
}

/// The autopilot's per-stream sensor timestamp (µs) for a GPS message, or None
/// if absent (`time_usec = 0`, the "unknown" sentinel). The absolute base
/// (boot-relative or UNIX epoch) doesn't matter — `ClockAligner` uses per-message
/// deltas, so the base cancels.
fn gps_sensor_us(msg: &MavMessage) -> Option<u64> {
    match msg {
        MavMessage::GPS_RAW_INT(g) if g.time_usec != 0 => Some(g.time_usec),
        _ => None,
    }
}

/// The autopilot's per-stream sensor timestamp (µs) for an IMU message, or None
/// if absent. `HIGHRES_IMU.time_usec` is already µs; `SCALED_IMU*` carry
/// `time_boot_ms` (ms since boot) → ×1000.
fn imu_sensor_us(msg: &MavMessage) -> Option<u64> {
    match msg {
        MavMessage::HIGHRES_IMU(i) if i.time_usec != 0 => Some(i.time_usec),
        MavMessage::SCALED_IMU(i) if i.time_boot_ms != 0 => Some(i.time_boot_ms as u64 * 1000),
        MavMessage::SCALED_IMU2(i) if i.time_boot_ms != 0 => Some(i.time_boot_ms as u64 * 1000),
        MavMessage::SCALED_IMU3(i) if i.time_boot_ms != 0 => Some(i.time_boot_ms as u64 * 1000),
        _ => None,
    }
}

/// Plausibility gate for inbound GPS — drop NaN/Inf and obviously-out-of-range
/// fields. Returning `false` causes the listener to discard the fix. This is
/// the second line of defense after source-IP / sysid filtering; even if a
/// hostile peer makes it past the network filters, NaN injection can't
/// silently disable downstream detection.
pub fn gps_fix_is_plausible(f: &GpsFix) -> bool {
    if !(f.lat_deg.is_finite() && f.lon_deg.is_finite() && f.alt_m.is_finite()) {
        return false;
    }
    if f.lat_deg.abs() > 90.0 || f.lon_deg.abs() > 180.0 {
        return false;
    }
    if f.alt_m.abs() > 100_000.0 {
        // 100 km — above any drone, below any meaningful elevation
        return false;
    }
    if let Some(s) = f.speed_mps {
        if !s.is_finite() || s.abs() > 1000.0 {
            return false;
        }
    }
    if let Some(c) = f.course_deg {
        if !c.is_finite() || !(-720.0..=720.0).contains(&c) {
            return false;
        }
    }
    true
}

/// Plausibility gate for inbound IMU samples — drop NaN/Inf and saturation
/// values. Real IMUs can saturate but values beyond physical limits indicate
/// either a hardware fault or an injection attempt.
pub fn imu_sample_is_plausible(s: &ImuSample) -> bool {
    for v in &s.accel_mps2 {
        if !v.is_finite() || v.abs() > 200.0 {
            // 200 m/s² ≈ 20 g — past anything we'd see on a real drone
            return false;
        }
    }
    for v in &s.gyro_rps {
        if !v.is_finite() || v.abs() > 35.0 {
            // 35 rad/s ≈ 2000 deg/s — past consumer-IMU full scale
            return false;
        }
    }
    true
}

/// Emit a `PilotModeChange` event. Fires every time the autopilot's
/// `custom_mode` changes from any cause — pilot manual override, battery
/// failsafe, geofence, or our own RTL. Verification logic distinguishes
/// "ours" from "pilot's" by comparing `mode_changed_at_ns` to the
/// controller's recorded send time; this event just surfaces the raw
/// transition to the operator.
fn emit_mode_change_event(
    event_tx: &tokio::sync::broadcast::Sender<crate::types::SpoofingEvent>,
    boot: Instant,
    new_mode: u32,
) {
    use crate::types::{NavStateKind, SpoofKind, SpoofingEvent};
    let ev = SpoofingEvent::new(
        Timestamp::now_mono(),
        SpoofKind::PilotModeChange,
        NavStateKind::Normal,
        0.0,
        serde_json::json!({"new_custom_mode": new_mode}),
        boot,
    );
    tracing::info!(custom_mode = new_mode, "MAV: autopilot mode changed");
    let _ = event_tx.send(ev);
}

/// Emit a `LinkRestored` (up=true) or `LinkDown` (up=false) event on the
/// broadcast channel. Best-effort: a failed send means there are no
/// subscribers, which is acceptable here.
fn emit_link_event(
    event_tx: &tokio::sync::broadcast::Sender<crate::types::SpoofingEvent>,
    boot: Instant,
    up: bool,
    current_mode: u32,
) {
    use crate::types::{NavStateKind, SpoofKind, SpoofingEvent};
    let kind = if up {
        SpoofKind::LinkRestored
    } else {
        SpoofKind::LinkDown
    };
    let ev = SpoofingEvent::new(
        Timestamp::now_mono(),
        kind,
        NavStateKind::Normal, // link events are orthogonal to detector state
        0.0,
        serde_json::json!({
            "current_custom_mode": current_mode,
        }),
        boot,
    );
    if up {
        tracing::info!("MAV LINK: HEARTBEAT restored (current_mode={current_mode})");
    } else {
        tracing::warn!("MAV LINK: HEARTBEAT silent past watchdog timeout — link down");
    }
    let _ = event_tx.send(ev);
}

/// Recognize OS errors that should not tear down a UDP listener.
/// On Windows, ICMP "port unreachable" from a previous send to a closed
/// peer is reported as os error 10054 (ECONNRESET semantics) on the next
/// `recv_from`. Linux raises EHOSTUNREACH / ECONNREFUSED similarly.
fn is_transient_udp_error(e: &std::io::Error) -> bool {
    if let Some(code) = e.raw_os_error() {
        if matches!(code, 10054 | 10052 | 10040) {
            return true;
        }
    }
    matches!(
        e.kind(),
        std::io::ErrorKind::ConnectionReset
            | std::io::ErrorKind::ConnectionRefused
            | std::io::ErrorKind::HostUnreachable
    )
}

#[cfg(test)]
mod plausibility_tests {
    use super::*;
    use crate::types::Timestamp;

    fn fix(lat: f64, lon: f64, alt: f64) -> GpsFix {
        GpsFix {
            t: Timestamp::now_mono(),
            lat_deg: lat,
            lon_deg: lon,
            alt_m: alt,
            speed_mps: Some(5.0),
            course_deg: Some(90.0),
            hdop: Some(1.0),
            sats: Some(10),
        }
    }

    #[test]
    fn rejects_nan_lat() {
        assert!(!gps_fix_is_plausible(&fix(f64::NAN, 0.0, 0.0)));
    }

    #[test]
    fn rejects_inf_alt() {
        assert!(!gps_fix_is_plausible(&fix(0.0, 0.0, f64::INFINITY)));
    }

    #[test]
    fn rejects_out_of_range_lat() {
        assert!(!gps_fix_is_plausible(&fix(95.0, 0.0, 0.0)));
    }

    #[test]
    fn rejects_out_of_range_lon() {
        assert!(!gps_fix_is_plausible(&fix(0.0, 200.0, 0.0)));
    }

    #[test]
    fn rejects_absurd_altitude() {
        assert!(!gps_fix_is_plausible(&fix(0.0, 0.0, 200_000.0)));
    }

    #[test]
    fn accepts_normal_fix() {
        assert!(gps_fix_is_plausible(&fix(40.0, -100.0, 500.0)));
    }

    fn imu(ax: f32, gx: f32) -> ImuSample {
        ImuSample {
            t: Timestamp::now_mono(),
            accel_mps2: [ax, 0.0, 9.81],
            gyro_rps: [gx, 0.0, 0.0],
        }
    }

    #[test]
    fn rejects_nan_accel() {
        assert!(!imu_sample_is_plausible(&imu(f32::NAN, 0.0)));
    }

    #[test]
    fn rejects_saturated_accel() {
        assert!(!imu_sample_is_plausible(&imu(300.0, 0.0)));
    }

    #[test]
    fn rejects_saturated_gyro() {
        assert!(!imu_sample_is_plausible(&imu(0.0, 40.0)));
    }

    #[test]
    fn accepts_normal_imu() {
        assert!(imu_sample_is_plausible(&imu(0.5, 0.1)));
    }

    // ---------- GPS_RAW_INT sentinel-handling tests (audit MAV-01) ----------
    //
    // Construct GPS_RAW_INT messages with the MAVLink "unknown" sentinels
    // and confirm `gps_from_msg` translates them to `None` rather than
    // numeric garbage (655 m/s, 655°, etc.). This is the cheapest defense
    // against the most damaging real-world bug class for the mav source —
    // a benign autopilot that doesn't report Doppler should NOT cause
    // false-positive jump fires.
    use mavlink::common::{GpsFixType, GPS_RAW_INT_DATA};

    fn raw_int(vel: u16, cog: u16, eph: u16, sats: u8) -> MavMessage {
        MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
            time_usec: 0,
            lat: 400_000_000,    // 40 N
            lon: -1_000_000_000, // 100 W
            alt: 500_000,        // 500 m MSL in mm
            eph,
            epv: u16::MAX,
            vel,
            cog,
            fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
            satellites_visible: sats,
        })
    }

    #[test]
    fn gps_from_msg_translates_velocity_sentinel_to_none() {
        let msg = raw_int(u16::MAX, 9000, 100, 10);
        let fix = gps_from_msg(&msg).expect("3D fix should be accepted");
        assert!(
            fix.speed_mps.is_none(),
            "vel=UINT16_MAX must become speed_mps=None (got {:?}); otherwise the \
             detector sees 655 m/s and the velocity-mismatch detector fires constantly",
            fix.speed_mps
        );
    }

    #[test]
    fn gps_from_msg_translates_course_sentinel_to_none() {
        let msg = raw_int(500, u16::MAX, 100, 10);
        let fix = gps_from_msg(&msg).expect("3D fix should be accepted");
        assert!(
            fix.course_deg.is_none(),
            "cog=UINT16_MAX must become course_deg=None (got {:?})",
            fix.course_deg
        );
    }

    #[test]
    fn gps_from_msg_rejects_out_of_range_course_as_none() {
        // 50000 cdeg = 500° (>359.99°) is invalid even though it's < UINT16_MAX.
        let msg = raw_int(500, 50_000, 100, 10);
        let fix = gps_from_msg(&msg).expect("fix should be accepted; only course is garbage");
        assert!(
            fix.course_deg.is_none(),
            "out-of-range cog must become None"
        );
    }

    #[test]
    fn gps_from_msg_translates_hdop_sentinel_to_none() {
        // Without this, hdop = 655 m and BOTH the jump detector and drift
        // detector widen their thresholds (1.5-2×) because they treat it as
        // "very poor quality" — silently reducing detection sensitivity.
        let msg = raw_int(500, 9000, u16::MAX, 10);
        let fix = gps_from_msg(&msg).expect("3D fix should be accepted");
        assert!(
            fix.hdop.is_none(),
            "eph=UINT16_MAX must become hdop=None (got {:?}); otherwise the detector \
             silently widens its gates and an attacker spoofing eph wins",
            fix.hdop
        );
    }

    #[test]
    fn gps_from_msg_translates_sats_sentinel_to_none() {
        let msg = raw_int(500, 9000, 100, u8::MAX);
        let fix = gps_from_msg(&msg).expect("3D fix should be accepted");
        assert!(
            fix.sats.is_none(),
            "satellites_visible=255 must become sats=None (got {:?})",
            fix.sats
        );
    }

    #[test]
    fn gps_from_msg_passes_real_values_through_unchanged() {
        // Regression: the sentinel fixes must not break the normal path.
        let msg = raw_int(523, 9012, 145, 8);
        let fix = gps_from_msg(&msg).expect("normal fix accepted");
        assert!((fix.speed_mps.unwrap() - 5.23).abs() < 1e-9);
        assert!((fix.course_deg.unwrap() - 90.12).abs() < 1e-9);
        assert!((fix.hdop.unwrap() - 1.45).abs() < 1e-3);
        assert_eq!(fix.sats.unwrap(), 8);
    }

    // ---------- SCALED_IMU parsing tests (audit V-IMU) ----------
    //
    // ArduPilot streams SCALED_IMU (mG accel, mrad/s gyro), NOT the PX4-native
    // HIGHRES_IMU. Before the Phase V SITL run, `imu_from_msg` only accepted
    // HIGHRES_IMU, so `--imu-source mav` against ArduCopter received zero IMU
    // samples and preflight hung forever. These tests pin the conversion so
    // it can never silently regress.
    use mavlink::common::{HIGHRES_IMU_DATA, SCALED_IMU_DATA};

    #[test]
    fn imu_from_scaled_imu_converts_units() {
        // 1000 mG = 1 g = 9.80665 m/s²; 500 mrad/s = 0.5 rad/s.
        let msg = MavMessage::SCALED_IMU(SCALED_IMU_DATA {
            time_boot_ms: 0,
            xacc: 1000, // 1 g
            yacc: 0,
            zacc: -1000, // -1 g (typical level-Z reading depends on convention)
            xgyro: 500,  // 0.5 rad/s
            ygyro: -250, // -0.25 rad/s
            zgyro: 0,
            xmag: 0,
            ymag: 0,
            zmag: 0,
        });
        let s = imu_from_msg(&msg).expect("SCALED_IMU must now produce an ImuSample");
        assert!(
            (s.accel_mps2[0] - 9.806_65).abs() < 1e-3,
            "xacc={}",
            s.accel_mps2[0]
        );
        assert!(s.accel_mps2[1].abs() < 1e-6, "yacc={}", s.accel_mps2[1]);
        assert!(
            (s.accel_mps2[2] + 9.806_65).abs() < 1e-3,
            "zacc={}",
            s.accel_mps2[2]
        );
        assert!(
            (s.gyro_rps[0] - 0.5).abs() < 1e-6,
            "xgyro={}",
            s.gyro_rps[0]
        );
        assert!(
            (s.gyro_rps[1] + 0.25).abs() < 1e-6,
            "ygyro={}",
            s.gyro_rps[1]
        );
    }

    #[test]
    fn mavlink_version_picked_from_magic_byte() {
        // Audit V-MAVVER / X-02: v2=0xFD, v1=0xFE, else default v2.
        assert!(matches!(mavlink_version_of(0xFD), MavlinkVersion::V2));
        assert!(matches!(mavlink_version_of(0xFE), MavlinkVersion::V1));
        // Garbage / non-frame byte defaults to V2 (reader will parse-error).
        assert!(matches!(mavlink_version_of(0x00), MavlinkVersion::V2));
        assert!(matches!(mavlink_version_of(0xAB), MavlinkVersion::V2));
    }

    #[test]
    fn imu_from_highres_imu_is_direct() {
        // HIGHRES_IMU is already SI — accel m/s², gyro rad/s. No conversion.
        let hr = HIGHRES_IMU_DATA {
            xacc: 1.5,
            zacc: 9.80665,
            ygyro: 0.1,
            ..Default::default()
        };
        let s = imu_from_msg(&MavMessage::HIGHRES_IMU(hr))
            .expect("HIGHRES_IMU must still produce an ImuSample");
        assert!((s.accel_mps2[0] - 1.5).abs() < 1e-6);
        assert!((s.accel_mps2[2] - 9.80665).abs() < 1e-5);
        assert!((s.gyro_rps[1] - 0.1).abs() < 1e-6);
    }
}

#[cfg(test)]
mod clock_align_tests {
    use super::*;
    use mavlink::common::{GPS_RAW_INT_DATA, HIGHRES_IMU_DATA, SCALED_IMU_DATA};
    use std::time::Duration;

    #[test]
    fn aligner_first_message_uses_arrival_and_anchors() {
        let mut a = ClockAligner::default();
        let t0 = Instant::now();
        // Regardless of the sensor value, the first message stamps arrival time.
        assert_eq!(a.align(Some(1_000_000), t0), t0);
    }

    #[test]
    fn aligner_removes_arrival_jitter() {
        let mut a = ClockAligner::default();
        let t0 = Instant::now();
        assert_eq!(a.align(Some(1_000_000), t0), t0); // anchor
                                                      // Sensor advanced +100 ms, but this message arrived 30 ms LATE (link
                                                      // jitter). The aligned stamp must track the SENSOR delta, not arrival.
        let arrival = t0 + Duration::from_millis(130);
        let aligned = a.align(Some(1_100_000), arrival);
        assert_eq!(aligned, t0 + Duration::from_millis(100));
        assert_ne!(
            aligned, arrival,
            "the 30 ms of arrival jitter must be removed"
        );
    }

    #[test]
    fn aligner_tolerates_latency_spike_within_bound() {
        // A one-off latency spike WITHIN CLOCK_ALIGN_MAX_SKEW is CORRECTED (the
        // aligned stamp tracks the sensor), not mistaken for a clock fault — this
        // is the whole point: removing exactly this jitter from the residual.
        let mut a = ClockAligner::default();
        let t0 = Instant::now();
        a.align(Some(1_000_000), t0);
        // Sensor +500 ms, arrival +1500 ms (a 1 s spike) → skew 1 s < 2 s bound.
        let aligned = a.align(Some(1_500_000), t0 + Duration::from_millis(1500));
        assert_eq!(aligned, t0 + Duration::from_millis(500));
    }

    #[test]
    fn aligner_missing_sensor_ts_uses_arrival_without_disturbing_anchor() {
        let mut a = ClockAligner::default();
        let t0 = Instant::now();
        a.align(Some(1_000_000), t0); // anchor
                                      // A message with no sensor timestamp → arrival time, anchor untouched.
        let arrival = t0 + Duration::from_millis(100);
        assert_eq!(a.align(None, arrival), arrival);
        // The next good message still aligns against the ORIGINAL anchor.
        let aligned = a.align(Some(1_200_000), t0 + Duration::from_millis(999));
        assert_eq!(aligned, t0 + Duration::from_millis(200));
    }

    #[test]
    fn aligner_re_anchors_on_sensor_regression() {
        let mut a = ClockAligner::default();
        let t0 = Instant::now();
        a.align(Some(5_000_000), t0); // anchor at sensor = 5 s
                                      // Sensor went BACKWARDS (autopilot reboot / counter wrap) → fall back to
                                      // arrival and re-anchor on the new base.
        let arrival = t0 + Duration::from_millis(100);
        assert_eq!(a.align(Some(1_000_000), arrival), arrival);
        // Subsequent messages align against the NEW anchor.
        let aligned = a.align(Some(1_050_000), t0 + Duration::from_millis(900));
        assert_eq!(aligned, arrival + Duration::from_millis(50));
    }

    #[test]
    fn aligner_re_anchors_on_implausible_skew() {
        let mut a = ClockAligner::default();
        let t0 = Instant::now();
        a.align(Some(1_000_000), t0); // anchor
                                      // Sensor jumped +10 s but arrival only advanced 100 ms: a ~9.9 s skew,
                                      // far beyond CLOCK_ALIGN_MAX_SKEW → the sensor clock isn't tracking wall
                                      // time (base mismatch / garbage), so distrust it and use arrival.
        let arrival = t0 + Duration::from_millis(100);
        assert_eq!(a.align(Some(11_000_000), arrival), arrival);
    }

    #[test]
    fn aligner_huge_sensor_timestamp_does_not_panic() {
        // Regression (one-packet DoS): `s` is an attacker-controlled wire
        // timestamp (GPS_RAW_INT.time_usec / SCALED_IMU.time_boot_ms). A crafted
        // huge value after a small anchor used to make `anchor_arrival +
        // Duration::from_micros(s - anchor_sensor)` overflow the monotonic Instant
        // and PANIC — killing the listener task and blinding the detector. It must
        // instead re-anchor and fall back to arrival time.
        let mut a = ClockAligner::default();
        let t0 = Instant::now();
        a.align(Some(1_000), t0); // anchor at a tiny sensor value
        let arrival = t0 + Duration::from_millis(50);
        // u64::MAX µs ≈ 584,000 years past the anchor: must NOT panic, use arrival.
        assert_eq!(a.align(Some(u64::MAX), arrival), arrival);
        // The fix must not break normal alignment: a +1 s sensor delta arriving
        // +1 s later (within skew) still tracks the sensor, not arrival jitter.
        let mut b = ClockAligner::default();
        let s0 = Instant::now();
        b.align(Some(10_000_000), s0); // anchor
        let aligned = b.align(Some(11_000_000), s0 + Duration::from_millis(1000));
        assert_eq!(aligned, s0 + Duration::from_millis(1000));
    }

    #[test]
    fn gps_sensor_us_reads_time_usec() {
        let g = MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
            time_usec: 7_000_000,
            ..Default::default()
        });
        assert_eq!(gps_sensor_us(&g), Some(7_000_000));
        // The 0 "unknown" sentinel → None (the aligner then uses arrival time).
        let z = MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
            time_usec: 0,
            ..Default::default()
        });
        assert_eq!(gps_sensor_us(&z), None);
    }

    #[test]
    fn imu_sensor_us_reads_per_message_type() {
        // HIGHRES_IMU.time_usec is already µs.
        let hr = MavMessage::HIGHRES_IMU(HIGHRES_IMU_DATA {
            time_usec: 4_200_000,
            ..Default::default()
        });
        assert_eq!(imu_sensor_us(&hr), Some(4_200_000));
        // SCALED_IMU.time_boot_ms is ms → ×1000.
        let si = MavMessage::SCALED_IMU(SCALED_IMU_DATA {
            time_boot_ms: 5_000,
            ..Default::default()
        });
        assert_eq!(imu_sensor_us(&si), Some(5_000_000));
        // Zero sentinel → None.
        let z = MavMessage::SCALED_IMU(SCALED_IMU_DATA {
            time_boot_ms: 0,
            ..Default::default()
        });
        assert_eq!(imu_sensor_us(&z), None);
    }
}
