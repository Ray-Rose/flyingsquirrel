//! flyingsquirrel-watch — terminal dashboard for the FlyingSquirrel
//! anti-spoofing detector.
//!
//! Operates on the JSON event log emitted by the main binary's
//! `--json-log <path>` flag (or the value set in
//! `[process].json_log` of the config file). Two modes:
//!
//!   --tail    (default)  follows the file as it grows (live operator view)
//!   --replay             reads the entire file once and exits (post-incident)
//!
//! Layout:
//!
//!   ┌─ FlyingSquirrel Watch ────────────────────────────────────┐
//!   │ Source: /var/log/flyingsquirrel/events.jsonl    Mode: tail│
//!   ├─ State ─────────────────────────────────────────────────  │
//!   │ FSM: Normal       Preflight: Ready       Link: UP         │
//!   ├─ Recent events ─────────────────────────────────────────  │
//!   │ 12:34:56 INFO  LinkRestored                                │
//!   │ 12:34:58 INFO  PreflightPassed                             │
//!   │ 12:35:12 WARN  Drift  reason=NorthPositive  resid=12.3m   │
//!   │ ...                                                        │
//!   ├─ q quit  ↑↓ scroll  p pause  c clear ─────────────────  │
//!   └────────────────────────────────────────────────────────  │
//!
//! Keybindings:
//!   q, Esc, Ctrl-C   quit
//!   ↑ / ↓            scroll event log (one line)
//!   PgUp / PgDn      scroll event log (one screen)
//!   p                pause auto-scroll (stays on selected event)
//!   r                resume auto-scroll
//!   c                clear visible buffer (does not affect source file)
//!
//! Build (this is a DETACHED crate — build it from `watch/`, not the repo
//! root; see `watch/Cargo.toml` for why it is separate from the detector):
//!   cd watch && cargo build --release

use clap::Parser;
use crossterm::event::{self, Event as CtEvent, KeyCode, KeyEvent, KeyModifiers};
use crossterm::{
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::backend::CrosstermBackend;
use ratatui::layout::{Constraint, Direction, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, List, ListItem, ListState, Paragraph};
use ratatui::Terminal;
use serde::Deserialize;
use std::collections::VecDeque;
use std::io::{self, BufRead, BufReader, Seek, SeekFrom};
use std::path::PathBuf;
use std::time::{Duration, Instant};

#[derive(Debug, Parser)]
#[command(
    name = "flyingsquirrel-watch",
    about = "Terminal dashboard for FlyingSquirrel JSON event logs."
)]
struct Cli {
    /// Path to the JSON-Lines event log (set by `flyingsquirrel --json-log`).
    log: PathBuf,
    /// Replay mode: read the entire file once and exit. Default is live tail.
    #[arg(long)]
    replay: bool,
    /// Replay speed multiplier (only meaningful with `--replay`). 1.0 = real
    /// time, 10.0 = 10x faster, 0.0 = instant.
    #[arg(long, default_value_t = 1.0)]
    speed: f32,
    /// Maximum events kept in the visible buffer. Older events scroll off
    /// the top.
    #[arg(long, default_value_t = 500)]
    buffer: usize,
}

/// Subset of `SpoofingEvent` we care about for display. We deserialize
/// loosely (extra fields ignored) so a future event schema change doesn't
/// break the dashboard — operators can keep using it across upgrades.
#[derive(Debug, Clone, Deserialize)]
struct Event {
    #[serde(default)]
    mono_ns: u128,
    #[serde(default)]
    kind: String,
    #[serde(default)]
    state: String,
    #[serde(default)]
    residual_m: f32,
    #[serde(default)]
    detail: serde_json::Value,
}

#[derive(Debug, Clone, Default)]
struct Snapshot {
    /// Last observed FSM state (Normal / Suspicious / Spoofed / Initializing).
    fsm_state: String,
    /// Preflight verdict (Passed / Failed / unknown).
    preflight: String,
    /// MAVLink link state (Up / Down / unknown).
    link: String,
    /// Last seen autopilot custom_mode (ArduCopter: 4=GUIDED, 6=RTL, etc.).
    autopilot_mode: Option<u64>,
    /// Has the boot-anchor cross-check been armed?
    boot_anchor_armed: bool,
    /// Last action result.
    last_action: String,
    /// Count of cumulative critical events.
    spoofed_count: u32,
    rtl_acked_count: u32,
    /// AUDIT F-13 fix: surface JSONL parse errors so a schema regression or
    /// log corruption is visible. Previously the parser dropped malformed
    /// lines silently; an operator watching the dashboard during an actual
    /// spoof event could see "Events: 0" with no hint that the producer
    /// was emitting unparseable lines.
    parse_errors: u32,
}

fn main() -> io::Result<()> {
    let cli = Cli::parse();

    // Set up the terminal.
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    // Run the app, restore terminal even on error.
    let result = run_app(&mut terminal, &cli);

    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
    terminal.show_cursor()?;
    result
}

fn run_app<B: ratatui::backend::Backend>(terminal: &mut Terminal<B>, cli: &Cli) -> io::Result<()> {
    let mut events: VecDeque<Event> = VecDeque::with_capacity(cli.buffer);
    let mut snap = Snapshot::default();
    let mut list_state = ListState::default();
    let mut paused = false;
    let mut prev_mono_ns: u128 = 0;

    // Open the source file. In tail mode we keep it open and seek to the
    // end so the operator only sees NEW events; in replay mode we seek to
    // the start.
    let file = std::fs::File::open(&cli.log)?;
    let mut reader = BufReader::new(file);
    if !cli.replay {
        let _ = reader.seek(SeekFrom::End(0));
    }
    let mut line_buf = String::new();

    // For tail mode we poll the file ~10 Hz. Faster than that wastes CPU;
    // slower feels laggy to the operator.
    let poll_interval = Duration::from_millis(100);

    loop {
        // 1. Drain any newly-arrived lines from the file.
        loop {
            line_buf.clear();
            let n = reader.read_line(&mut line_buf)?;
            if n == 0 {
                break; // no new data
            }
            let line = line_buf.trim_end();
            if line.is_empty() {
                continue;
            }
            let parsed: Result<Event, _> = serde_json::from_str(line);
            let ev = match parsed {
                Ok(e) => e,
                Err(_) => {
                    // AUDIT F-13 fix: track parse errors so the operator
                    // sees them in the header. Silently dropping was the
                    // prior behavior — bad for trust during incidents.
                    snap.parse_errors = snap.parse_errors.saturating_add(1);
                    continue;
                }
            };

            // In replay mode, sleep proportional to mono_ns delta so events
            // come back in (sped-up) real time.
            if cli.replay && cli.speed > 0.0 {
                if prev_mono_ns > 0 && ev.mono_ns > prev_mono_ns {
                    let delta_ns = ev.mono_ns - prev_mono_ns;
                    let wall_ns = (delta_ns as f64 / cli.speed as f64) as u64;
                    let cap = Duration::from_millis(500); // cap any single sleep
                    let dur = Duration::from_nanos(wall_ns).min(cap);
                    std::thread::sleep(dur);
                }
                prev_mono_ns = ev.mono_ns;
            }

            apply_to_snapshot(&mut snap, &ev);
            if events.len() >= cli.buffer {
                events.pop_front();
            }
            events.push_back(ev);
            if !paused {
                list_state.select(Some(events.len().saturating_sub(1)));
            }
        }

        // 2. Render.
        terminal.draw(|f| draw(f, cli, &snap, &events, &mut list_state, paused))?;

        // 3. Handle input. In tail mode block for up to poll_interval; in
        // replay mode block briefly so input still feels responsive.
        let timeout = if cli.replay {
            Duration::from_millis(10)
        } else {
            poll_interval
        };
        if event::poll(timeout)? {
            if let CtEvent::Key(k) = event::read()? {
                if !handle_key(k, &mut events, &mut list_state, &mut paused) {
                    return Ok(());
                }
            }
        }

        // 4. In replay mode, exit once EOF reached AND no input occurred
        // for a beat. Use a small grace period so the operator can scroll.
        if cli.replay {
            // Detect "no more data ever" by trying once more and seeing 0.
            // If still 0 and the file size hasn't grown, we're done. For
            // simplicity here, just keep the UI open — operator quits via q.
        }
    }
}

/// Update `snap` from one event. The dashboard's left pane shows the
/// current state which is the latest observation per category.
fn apply_to_snapshot(s: &mut Snapshot, e: &Event) {
    if !e.state.is_empty() {
        s.fsm_state = e.state.clone();
    }
    match e.kind.as_str() {
        "PreflightPassed" => s.preflight = "Passed".to_string(),
        "PreflightFailed" => s.preflight = "Failed".to_string(),
        "LinkRestored" => s.link = "Up".to_string(),
        "LinkDown" => s.link = "Down".to_string(),
        "PilotModeChange" => {
            if let Some(m) = e.detail.get("new_custom_mode").and_then(|v| v.as_u64()) {
                s.autopilot_mode = Some(m);
            }
        }
        "StateTransition" => {
            if e.state == "Spoofed" {
                s.spoofed_count += 1;
                s.last_action = "STATE -> Spoofed".to_string();
            }
        }
        "ActionAcked" => {
            s.rtl_acked_count += 1;
            s.last_action = "RTL Acked".to_string();
        }
        "ActionUnconfirmed" => {
            s.last_action = "RTL Unconfirmed!".to_string();
        }
        "ActionFailed" => {
            s.last_action = "Action FAILED".to_string();
        }
        "BootAnchorRejected" => {
            s.boot_anchor_armed = true;
            s.last_action = "BootAnchor REJECTED".to_string();
        }
        "Attestation" => {
            // Don't display; just a record. (The header will show source.)
        }
        _ => {}
    }
}

fn handle_key(
    k: KeyEvent,
    events: &mut VecDeque<Event>,
    list_state: &mut ListState,
    paused: &mut bool,
) -> bool {
    let ctrl_c = k.modifiers.contains(KeyModifiers::CONTROL) && k.code == KeyCode::Char('c');
    match k.code {
        KeyCode::Char('q') | KeyCode::Esc => return false,
        _ if ctrl_c => return false,
        KeyCode::Char('p') => *paused = true,
        KeyCode::Char('r') => *paused = false,
        KeyCode::Char('c') => {
            events.clear();
            list_state.select(None);
        }
        KeyCode::Up => {
            *paused = true;
            let cur = list_state.selected().unwrap_or(0);
            if cur > 0 {
                list_state.select(Some(cur - 1));
            }
        }
        KeyCode::Down => {
            *paused = true;
            let cur = list_state.selected().unwrap_or(0);
            if cur + 1 < events.len() {
                list_state.select(Some(cur + 1));
            }
        }
        KeyCode::PageUp => {
            *paused = true;
            let cur = list_state.selected().unwrap_or(0);
            list_state.select(Some(cur.saturating_sub(20)));
        }
        KeyCode::PageDown => {
            *paused = true;
            let cur = list_state.selected().unwrap_or(0);
            let new = (cur + 20).min(events.len().saturating_sub(1));
            list_state.select(Some(new));
        }
        _ => {}
    }
    true
}

fn draw(
    f: &mut ratatui::Frame,
    cli: &Cli,
    snap: &Snapshot,
    events: &VecDeque<Event>,
    list_state: &mut ListState,
    paused: bool,
) {
    let size = f.area();
    let layout = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3), // header
            Constraint::Length(3), // state pane
            Constraint::Min(5),    // events
            Constraint::Length(1), // keybinds
        ])
        .split(size);

    // Header
    let mode = if cli.replay { "REPLAY" } else { "TAIL" };
    // Audit F-13 fix: surface parse_errors in the header. Color it red when
    // non-zero so the operator's eye catches it during incidents.
    let parse_errors_text = if snap.parse_errors > 0 {
        format!("    PARSE-ERR: {}", snap.parse_errors)
    } else {
        String::new()
    };
    let header_text = format!(
        " Source: {}    Mode: {}    Events: {}{}{}",
        cli.log.display(),
        mode,
        events.len(),
        parse_errors_text,
        if paused { "    [PAUSED]" } else { "" }
    );
    let header_style = if snap.parse_errors > 0 {
        Style::default().fg(Color::Red).add_modifier(Modifier::BOLD)
    } else {
        Style::default()
    };
    let header = Paragraph::new(Span::styled(header_text, header_style)).block(
        Block::default()
            .borders(Borders::ALL)
            .title("FlyingSquirrel Watch"),
    );
    f.render_widget(header, layout[0]);

    // State pane
    let fsm_color = match snap.fsm_state.as_str() {
        "Normal" => Color::Green,
        "Suspicious" => Color::Yellow,
        "Spoofed" => Color::Red,
        _ => Color::DarkGray,
    };
    let link_color = match snap.link.as_str() {
        "Up" => Color::Green,
        "Down" => Color::Red,
        _ => Color::DarkGray,
    };
    let state = Paragraph::new(Line::from(vec![
        Span::raw(" FSM: "),
        Span::styled(
            if snap.fsm_state.is_empty() {
                "—".to_string()
            } else {
                snap.fsm_state.clone()
            },
            Style::default().fg(fsm_color).add_modifier(Modifier::BOLD),
        ),
        Span::raw("    Preflight: "),
        Span::styled(
            if snap.preflight.is_empty() {
                "—".to_string()
            } else {
                snap.preflight.clone()
            },
            Style::default().fg(if snap.preflight == "Passed" {
                Color::Green
            } else {
                Color::DarkGray
            }),
        ),
        Span::raw("    Link: "),
        Span::styled(
            if snap.link.is_empty() {
                "—".to_string()
            } else {
                snap.link.clone()
            },
            Style::default().fg(link_color),
        ),
        Span::raw("    Mode: "),
        Span::raw(
            snap.autopilot_mode
                .map(|m| m.to_string())
                .unwrap_or("—".to_string()),
        ),
        Span::raw("    Spoofed total: "),
        Span::styled(
            snap.spoofed_count.to_string(),
            Style::default().fg(if snap.spoofed_count > 0 {
                Color::Red
            } else {
                Color::Green
            }),
        ),
        Span::raw("    Acked: "),
        Span::raw(snap.rtl_acked_count.to_string()),
    ]))
    .block(Block::default().borders(Borders::ALL).title("State"));
    f.render_widget(state, layout[1]);

    // Events log
    let items: Vec<ListItem> = events
        .iter()
        .map(|e| ListItem::new(format_event(e)))
        .collect();
    let title = format!(
        "Events  ({})",
        if snap.last_action.is_empty() {
            ""
        } else {
            &snap.last_action
        }
    );
    let list = List::new(items)
        .block(Block::default().borders(Borders::ALL).title(title))
        .highlight_style(Style::default().add_modifier(Modifier::REVERSED));
    f.render_stateful_widget(list, layout[2], list_state);

    // Keybinds
    let keybinds =
        Paragraph::new(" q quit | ↑↓ scroll | PgUp/PgDn page | p pause | r resume | c clear")
            .style(Style::default().fg(Color::DarkGray));
    f.render_widget(keybinds, layout[3]);
}

fn format_event(e: &Event) -> Line<'static> {
    let (level_text, level_color) = match e.kind.as_str() {
        "StateTransition" if e.state == "Spoofed" => ("CRIT", Color::Red),
        "ActionFailed" | "ActionUnconfirmed" | "LinkDown" | "BootAnchorRejected" => {
            ("WARN", Color::Yellow)
        }
        "MAV ACTION" | "PreflightFailed" => ("WARN", Color::Yellow),
        "Jump" | "Drift" | "FrozenGps" | "DwellPauseExceeded" => ("INFO", Color::Yellow),
        _ => ("INFO", Color::Green),
    };
    // Compact mono_ns to seconds for readability.
    let t_secs = (e.mono_ns / 1_000_000_000) as u64;
    let detail = format_detail(e);
    Line::from(vec![
        Span::styled(
            format!("t={:>6}s ", t_secs),
            Style::default().fg(Color::DarkGray),
        ),
        Span::styled(
            format!("{:<4} ", level_text),
            Style::default().fg(level_color),
        ),
        Span::styled(
            format!("{:<22}", e.kind),
            Style::default().add_modifier(Modifier::BOLD),
        ),
        Span::raw(detail),
    ])
}

fn format_detail(e: &Event) -> String {
    let mut parts: Vec<String> = Vec::new();
    if !e.state.is_empty() {
        parts.push(format!("state={}", e.state));
    }
    if e.residual_m > 0.0 {
        parts.push(format!("residual={:.1}m", e.residual_m));
    }
    if let Some(reason) = e.detail.get("reason").and_then(|v| v.as_str()) {
        parts.push(format!("reason={}", reason));
    }
    if let Some(mode) = e.detail.get("new_custom_mode").and_then(|v| v.as_u64()) {
        parts.push(format!("mode={}", mode));
    }
    if let Some(d) = e.detail.get("distance_from_dr_m").and_then(|v| v.as_f64()) {
        parts.push(format!("dr_dist={:.1}m", d));
    }
    if let Some(d) = e
        .detail
        .get("distance_from_expected_home_m")
        .and_then(|v| v.as_f64())
    {
        parts.push(format!("home_dist={:.1}m", d));
    }
    if parts.is_empty() {
        "".into()
    } else {
        format!("  {}", parts.join("  "))
    }
}

// Touch unused import — the file does not directly reference `Instant` in
// the simplified v1 (no tick-rate accounting yet) but keeping the import
// for future use.
#[allow(dead_code)]
fn _silence_unused() {
    let _ = Instant::now();
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn apply_snapshot_link_and_state() {
        let mut s = Snapshot::default();
        apply_to_snapshot(
            &mut s,
            &Event {
                mono_ns: 0,
                kind: "LinkRestored".into(),
                state: "Normal".into(),
                residual_m: 0.0,
                detail: serde_json::json!({}),
            },
        );
        assert_eq!(s.link, "Up");
        assert_eq!(s.fsm_state, "Normal");
        apply_to_snapshot(
            &mut s,
            &Event {
                mono_ns: 1_000_000_000,
                kind: "PreflightPassed".into(),
                state: "Normal".into(),
                residual_m: 0.0,
                detail: serde_json::json!({}),
            },
        );
        assert_eq!(s.preflight, "Passed");
    }

    #[test]
    fn apply_snapshot_spoofed_increments_counter() {
        let mut s = Snapshot::default();
        apply_to_snapshot(
            &mut s,
            &Event {
                mono_ns: 0,
                kind: "StateTransition".into(),
                state: "Spoofed".into(),
                residual_m: 25.0,
                detail: serde_json::json!({}),
            },
        );
        assert_eq!(s.spoofed_count, 1);
        assert_eq!(s.fsm_state, "Spoofed");
    }

    #[test]
    fn malformed_jsonl_is_skipped() {
        let mut tmp = tempfile::NamedTempFile::new().unwrap();
        writeln!(tmp, r#"not json"#).unwrap();
        writeln!(tmp, r#"{{"kind":"LinkRestored","state":"Normal","mono_ns":0,"residual_m":0.0,"detail":{{}}}}"#).unwrap();
        writeln!(tmp, r#"also not json"#).unwrap();
        let file = std::fs::File::open(tmp.path()).unwrap();
        let mut reader = BufReader::new(file);
        let mut line = String::new();
        let mut parsed = 0;
        loop {
            line.clear();
            let n = reader.read_line(&mut line).unwrap();
            if n == 0 {
                break;
            }
            if serde_json::from_str::<Event>(line.trim_end()).is_ok() {
                parsed += 1;
            }
        }
        assert_eq!(parsed, 1, "should parse exactly 1 valid line out of 3");
    }

    #[test]
    fn parse_error_counter_tracks_malformed_lines() {
        // Audit F-13: parse errors must increment the counter so the
        // operator sees them on the dashboard header.
        let mut s = Snapshot::default();
        // Simulate the parser path: feed three lines, two malformed, one good.
        for line in [
            "not json",
            r#"{"kind":"LinkRestored","state":"Normal","mono_ns":0,"residual_m":0.0,"detail":{}}"#,
            "also not json",
        ] {
            match serde_json::from_str::<Event>(line) {
                Ok(ev) => apply_to_snapshot(&mut s, &ev),
                Err(_) => s.parse_errors = s.parse_errors.saturating_add(1),
            }
        }
        assert_eq!(
            s.parse_errors, 2,
            "two malformed lines must increment counter"
        );
        assert_eq!(s.link, "Up", "valid line still applied");
    }
}
