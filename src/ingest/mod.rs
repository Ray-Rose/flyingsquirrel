//! Source traits. Implementations live in `sim::` for v1; hardware backends
//! are a clean drop-in via the same traits.

pub mod nmea;

use crate::error::FsError;
use crate::types::{GpsFix, ImuSample};
use futures::Stream;
use std::pin::Pin;

pub type BoxStream<'a, T> = Pin<Box<dyn Stream<Item = T> + Send + 'a>>;

pub trait GpsSource: Send + 'static {
    fn into_stream(self: Box<Self>) -> BoxStream<'static, Result<GpsFix, FsError>>;
}

pub trait ImuSource: Send + 'static {
    fn into_stream(self: Box<Self>) -> BoxStream<'static, Result<ImuSample, FsError>>;
}
