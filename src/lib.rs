//! FlyingSquirrel — GPS/IMU drift cross-correlator.
//!
//! Detects GPS spoofing by comparing the GPS-reported position against an
//! independently dead-reckoned position from the IMU. Catches both sudden
//! jumps (teleport-style attacks) and slow drift (RQ-170-style smart spoofers).

pub mod action;
pub mod attestation;
pub mod config;
pub mod detect;
pub mod error;
pub mod forensic;
pub mod fusion;
pub mod hw;
pub mod ingest;
pub mod mav;
pub mod nav;
pub mod runtime;
pub mod sim;
pub mod types;

pub use error::FsError;
pub use types::{
    GpsFix, ImuSample, NavStateKind, NedPos, NedVel, SpoofKind, SpoofingEvent, Timestamp,
};
