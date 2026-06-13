//! `GpsSource` and `ImuSource` impls that consume from the channels owned
//! by a `MavLink` listener.

use crate::error::FsError;
use crate::ingest::{BoxStream, GpsSource, ImuSource};
use crate::types::{GpsFix, ImuSample};
use async_stream::stream;
use tokio::sync::mpsc;

pub struct MavGpsSource {
    pub rx: mpsc::Receiver<GpsFix>,
}

impl GpsSource for MavGpsSource {
    fn into_stream(self: Box<Self>) -> BoxStream<'static, Result<GpsFix, FsError>> {
        let mut rx = self.rx;
        Box::pin(stream! {
            while let Some(fix) = rx.recv().await {
                yield Ok(fix);
            }
        })
    }
}

pub struct MavImuSource {
    pub rx: mpsc::Receiver<ImuSample>,
}

impl ImuSource for MavImuSource {
    fn into_stream(self: Box<Self>) -> BoxStream<'static, Result<ImuSample, FsError>> {
        let mut rx = self.rx;
        Box::pin(stream! {
            while let Some(sample) = rx.recv().await {
                yield Ok(sample);
            }
        })
    }
}
