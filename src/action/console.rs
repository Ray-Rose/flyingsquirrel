//! Console + JSON-Lines FlightController implementation.

use super::FlightController;
use crate::error::FsError;
use crate::types::SpoofingEvent;
use async_trait::async_trait;
use std::path::PathBuf;
use std::sync::Mutex;
use tokio::io::AsyncWriteExt;
use tokio::sync::Mutex as AsyncMutex;
use tracing::{info, warn};

pub struct ConsoleController {
    json_writer: Option<AsyncMutex<tokio::fs::File>>,
    gps_severed: Mutex<bool>,
    rtb_engaged: Mutex<bool>,
}

impl ConsoleController {
    pub async fn new(json_log: Option<PathBuf>) -> Result<Self, FsError> {
        let json_writer = match json_log {
            Some(path) => {
                let f = tokio::fs::OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(&path)
                    .await?;
                Some(AsyncMutex::new(f))
            }
            None => None,
        };
        Ok(Self {
            json_writer,
            gps_severed: Mutex::new(false),
            rtb_engaged: Mutex::new(false),
        })
    }
}

#[async_trait]
impl FlightController for ConsoleController {
    async fn on_event(&self, ev: &SpoofingEvent) {
        info!(
            kind = ?ev.kind,
            state = ?ev.state,
            residual_m = ev.residual_m,
            "spoofing event"
        );
        if let Some(writer) = &self.json_writer {
            match serde_json::to_string(ev) {
                Ok(line) => {
                    let mut w = writer.lock().await;
                    let _ = w.write_all(line.as_bytes()).await;
                    let _ = w.write_all(b"\n").await;
                    let _ = w.flush().await;
                }
                Err(e) => warn!(error = %e, "failed to serialize event"),
            }
        }
    }

    async fn sever_gps(&self) -> Result<(), FsError> {
        let mut g = self.gps_severed.lock().unwrap();
        if *g {
            // Audit E-04 fix: idempotent. Previous behavior was to return
            // AlreadyEngaged which fusion treated as a failed action and
            // emitted ActionFailed events — misleading because the action
            // IS engaged. Production wants idempotence; tests that care
            // about latch behavior should observe `gps_severed` directly.
            tracing::debug!("ACTION: sever_gps already engaged; no-op");
            return Ok(());
        }
        *g = true;
        warn!("ACTION: severing GPS link");
        Ok(())
    }

    async fn engage_rtb(&self) -> Result<(), FsError> {
        let mut r = self.rtb_engaged.lock().unwrap();
        if *r {
            tracing::debug!("ACTION: engage_rtb already engaged; no-op");
            return Ok(());
        }
        *r = true;
        warn!("ACTION: engaging Return-to-Base on inertial sensors");
        Ok(())
    }

    /// Audit E-17 fix: the default trait impl returns `Ok(true)` ("trust
    /// the send path"). But a console controller has NO link to a real
    /// autopilot — it can't trust anything. Falsely returning Ok(true)
    /// causes an `ActionAcked` event with no underlying confirmation,
    /// which misleads operators into believing the drone is recovering.
    ///
    /// Honest answer for console mode: "I sent the command (to stderr/log)
    /// but I cannot verify the autopilot transitioned." Return Ok(false)
    /// so fusion emits ActionUnconfirmed with a clear reason.
    async fn verify_rtb_engaged(&self) -> Result<bool, FsError> {
        warn!(
            "MAV VERIFY: console controller cannot observe an autopilot — \
             reporting RTL as UNCONFIRMED. Use --controller mav with an actual \
             MAVLink endpoint for closed-loop verification."
        );
        Ok(false)
    }

    async fn reset(&self) -> Result<(), FsError> {
        let mut g = self.gps_severed.lock().unwrap();
        let mut r = self.rtb_engaged.lock().unwrap();
        let was = (*g, *r);
        *g = false;
        *r = false;
        warn!(
            was_gps_severed = was.0,
            was_rtb_engaged = was.1,
            "ACTION: controller latches cleared (operator reset)"
        );
        Ok(())
    }
}
