//! FlightController abstraction + the v1 console-logging implementation.

pub mod console;

use crate::error::FsError;
use crate::types::SpoofingEvent;
use async_trait::async_trait;

#[async_trait]
pub trait FlightController: Send + Sync + 'static {
    async fn on_event(&self, ev: &SpoofingEvent);
    async fn sever_gps(&self) -> Result<(), FsError>;
    async fn engage_rtb(&self) -> Result<(), FsError>;

    /// Verify after `engage_rtb()` that the autopilot actually transitioned
    /// to an RTL-equivalent state. Implementations that can't observe the
    /// autopilot (e.g. console-only) should return `Ok(true)` and trust the
    /// send path. Implementations with feedback (MAVLink) should poll the
    /// autopilot's mode and return `Ok(true)` only on confirmed transition.
    ///
    /// Returning `Ok(false)` means "command was sent but autopilot did not
    /// engage RTL within the verification window" — operator should know.
    /// Returning `Err` means the verification itself failed.
    async fn verify_rtb_engaged(&self) -> Result<bool, FsError> {
        Ok(true)
    }

    /// Verify after `sever_gps()` that the autopilot actually APPLIED the
    /// GPS-disable parameter. A real autopilot echoes a `PARAM_VALUE` for every
    /// `PARAM_SET` it applies; the MAVLink controller polls for that echo and
    /// returns `Ok(true)` only on a confirmed read-back. The default trusts the
    /// send path and returns `Ok(true)`.
    ///
    /// This complements the existing "did a datagram leave the host" check:
    /// best-effort UDP can deliver the `PARAM_SET` to the wire yet have the
    /// autopilot drop, NAK, or ignore it — in which case the drone keeps
    /// navigating on the spoofed GPS even as RTL fires. An unconfirmed sever is
    /// the single most dangerous silent failure in the action path.
    ///
    /// Note the default stays `Ok(true)` even for non-observing controllers
    /// (unlike `verify_rtb_engaged`, which console overrides to `Ok(false)`).
    /// A false RTL-confirm misleads an operator into believing the drone is
    /// physically returning home; a console `sever_gps` is a local log-only
    /// no-op with no real autopilot GPS to leave active, so there is no
    /// equivalent dangerous false belief. Only the MAVLink controller has a
    /// real read-back to perform.
    async fn verify_sever_engaged(&self) -> Result<bool, FsError> {
        Ok(true)
    }

    /// Operator-driven reset: clear any internal "already engaged" latches so
    /// a subsequent SPOOFED transition can re-fire `sever_gps` / `engage_rtb`.
    /// Without this, the FSM reset (SIGHUP) leaves the controller stuck on
    /// "AlreadyEngaged" from a prior detection, and the next real spoof goes
    /// undefended. Default impl is a no-op for controllers that have no
    /// latches.
    ///
    /// MUST be called from the same code path as `StateMachine::manual_reset_to_normal`.
    async fn reset(&self) -> Result<(), FsError> {
        Ok(())
    }
}
