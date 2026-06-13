use thiserror::Error;

#[derive(Error, Debug)]
pub enum FsError {
    #[error("ingest: {0}")]
    Ingest(#[from] IngestError),
    #[error("nav: {0}")]
    Nav(#[from] NavError),
    #[error("detect: {0}")]
    Detect(#[from] DetectError),
    #[error("action: {0}")]
    Action(#[from] ActionError),
    #[error("channel closed")]
    ChannelClosed,
    /// Operator-facing configuration / CLI-guard error. Distinct from `Io` so
    /// the message reads as a config problem (not a mislabeled "io:" error) and
    /// so the binary can print it cleanly instead of as a Debug-wrapped struct.
    #[error("{0}")]
    Config(String),
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("json: {0}")]
    Json(#[from] serde_json::Error),
}

#[derive(Error, Debug)]
pub enum IngestError {
    #[error("nmea parse: {0}")]
    NmeaParse(String),
    #[error("source ended")]
    SourceEnded,
}

#[derive(Error, Debug)]
pub enum NavError {
    #[error("not initialized")]
    NotInitialized,
    #[error("non-finite math: {0}")]
    NonFinite(&'static str),
}

#[derive(Error, Debug)]
pub enum DetectError {
    #[error("imu buffer empty")]
    BufferEmpty,
    #[error("gps timestamp outside imu buffer (gap={gap_ms} ms)")]
    GpsOutsideBuffer { gap_ms: i64 },
}

#[derive(Error, Debug)]
pub enum ActionError {
    #[error("controller already engaged")]
    AlreadyEngaged,
    #[error("sink write: {0}")]
    SinkWrite(String),
}
