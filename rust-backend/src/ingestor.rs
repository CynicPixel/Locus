// Raw frame ingestion from the Go Unix socket.
// Connects with retry, reads JSON lines, deserializes, and forwards to pipeline.

use std::time::Duration;

use anyhow::Context;
use rustc_hash::FxHasher;
use serde::{Deserialize, Serialize};
use std::hash::{Hash, Hasher};
use tokio::io::AsyncBufReadExt;
use tokio::sync::mpsc;

// ---------------------------------------------------------------------------
// RawFrame — wire type from Go ingestor
// ---------------------------------------------------------------------------

/// A single Mode-S observation from one sensor, as emitted by the Go ingestor.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct RawFrame {
    pub sensor_id: i64,
    pub sensor_lat: f64,
    pub sensor_lon: f64,
    /// Meters above sea level.
    pub sensor_alt: f64,
    /// Unix epoch seconds (Go converts seconds-since-midnight before writing).
    pub timestamp_seconds: u64,
    pub timestamp_nanoseconds: u64,
    /// Hex-encoded raw Mode-S bytes (e.g. "8D4840D6202CC371C32CE0576098").
    pub raw_modes_hex: String,

    // ---- Computed in finalize(), skipped in JSON ----
    /// Pre-computed unix epoch nanoseconds.  All internal code reads this field.
    #[serde(skip)]
    pub timestamp_ns_cache: u64,
    /// Decoded bytes — avoids repeated hex::decode allocations.
    #[serde(skip)]
    pub decoded_bytes: Option<Box<[u8]>>,
    /// FxHash of decoded_bytes; 0 if decode failed.
    #[serde(skip)]
    pub content_hash: u64,
}

impl RawFrame {
    /// Called immediately after deserialization.  Populates all derived fields.
    pub fn finalize(mut self) -> Self {
        self.timestamp_ns_cache =
            self.timestamp_seconds * 1_000_000_000 + self.timestamp_nanoseconds;

        match hex::decode(&self.raw_modes_hex) {
            Ok(b) => {
                let mut hasher = FxHasher::default();
                b.hash(&mut hasher);
                self.content_hash = hasher.finish();
                self.decoded_bytes = Some(b.into_boxed_slice());
            }
            Err(e) => {
                tracing::warn!(
                    sensor_id = self.sensor_id,
                    raw_hex = %self.raw_modes_hex,
                    error = %e,
                    "hex decode failed — frame dropped"
                );
            }
        }
        self
    }

    /// Full timestamp as nanoseconds since Unix epoch.
    #[inline(always)]
    pub fn timestamp_ns(&self) -> u64 {
        self.timestamp_ns_cache
    }

    /// Zero-copy access to decoded frame bytes.  None if hex was invalid.
    #[inline]
    pub fn bytes(&self) -> Option<&[u8]> {
        self.decoded_bytes.as_deref()
    }
}

// ---------------------------------------------------------------------------
// Socket connection with retry
// ---------------------------------------------------------------------------

// Ingestor connection constants are defined in crate::consts.
use crate::consts::{
    INGESTOR_SOCKET_PATH as SOCKET_PATH,
    MAX_CONNECT_RETRIES,
    INGESTOR_RETRY_DELAY_MS as RETRY_DELAY_MS,
};

async fn connect_with_retry() -> anyhow::Result<tokio::net::UnixStream> {
    for attempt in 0..MAX_CONNECT_RETRIES {
        match tokio::net::UnixStream::connect(SOCKET_PATH).await {
            Ok(stream) => {
                tracing::info!(attempt = attempt + 1, "Connected to ingestor socket");
                return Ok(stream);
            }
            Err(e) => {
                if attempt == 0 {
                    tracing::info!(
                        socket = SOCKET_PATH,
                        "Waiting for Go ingestor socket…"
                    );
                }
                tracing::debug!("Connect attempt {} failed: {e}", attempt + 1);
                tokio::time::sleep(Duration::from_millis(RETRY_DELAY_MS)).await;
            }
        }
    }
    anyhow::bail!("Could not connect to {SOCKET_PATH} after {MAX_CONNECT_RETRIES} attempts")
}

async fn read_loop(
    stream: tokio::net::UnixStream,
    tx: &mpsc::Sender<RawFrame>,
) -> anyhow::Result<()> {
    let reader = tokio::io::BufReader::new(stream);
    let mut lines = reader.lines();

    while let Some(line) = lines.next_line().await.context("socket read error")? {
        match serde_json::from_str::<RawFrame>(&line) {
            Ok(frame) => {
                let frame = frame.finalize();
                if tx.send(frame).await.is_err() {
                    anyhow::bail!("downstream receiver dropped — shutting down ingestor");
                }
            }
            Err(e) => {
                tracing::warn!(error = %e, len = line.len(), "JSON parse error — frame dropped");
            }
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------

/// Connect to the Go ingestor Unix socket and forward `RawFrame`s.
/// Reconnects automatically on disconnect.
pub async fn run_ingestor(tx: mpsc::Sender<RawFrame>) -> anyhow::Result<()> {
    loop {
        match connect_with_retry().await {
            Err(e) => {
                tracing::error!("Could not connect to Go ingestor: {e}. Retrying in 5 s.");
                tokio::time::sleep(Duration::from_secs(crate::consts::INGESTOR_RECONNECT_DELAY_SECS)).await;
            }
            Ok(stream) => {
                let result = read_loop(stream, &tx).await;
                match &result {
                    Ok(()) => tracing::warn!("Go ingestor connection closed cleanly. Reconnecting…"),
                    Err(e) => tracing::warn!("Go ingestor connection lost: {e}. Reconnecting…"),
                }
                tokio::time::sleep(Duration::from_millis(crate::consts::INGESTOR_CLOSE_RECONNECT_DELAY_MS)).await;
            }
        }
    }
}
