// WebSocket broadcast server.
// Accepts TCP connections, upgrades to WebSocket, and fans out JSON messages.



use futures_util::{SinkExt, StreamExt};
use serde::{Deserialize, Serialize};
use tokio::net::TcpListener;
use tokio::sync::broadcast;
use tokio_tungstenite::tungstenite::Message;

pub type BroadcastTx = broadcast::Sender<String>;

// ---------------------------------------------------------------------------
// AircraftState — the primary WebSocket payload
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AircraftState {
    pub icao24: String,
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f64,
    /// Degrees per second northward.
    pub vel_lat: f64,
    /// Degrees per second eastward.
    pub vel_lon: f64,
    /// Meters per second vertical.
    pub vel_alt: f64,
    pub gdop: f64,
    /// 2-sigma horizontal confidence ellipse radius (meters).
    pub confidence_ellipse_m: f64,
    pub spoof_flag: bool,
    pub divergence_m: f64,
    /// "normal" | "anomalous" | "unknown"
    pub anomaly_label: String,
    pub anomaly_confidence: f64,
    pub sensor_count: usize,
    pub timestamp_ms: u64,
    /// Degrees of freedom (n_receivers + 1_if_alt_constrained - 4). 0 for ADS-B source.
    pub dof: i32,
    /// IDs of sensors that contributed to this MLAT fix. Empty for ADS-B.
    pub sensor_ids: Vec<i64>,
}

// ---------------------------------------------------------------------------
// Server
// ---------------------------------------------------------------------------

/// Listen on `addr`, accept WebSocket connections, and broadcast messages from `rx`.
pub async fn run_ws_server(addr: String, tx: BroadcastTx) -> anyhow::Result<()> {
    let listener = TcpListener::bind(&addr).await?;
    tracing::info!("WebSocket server listening on {addr}");

    loop {
        let (stream, peer) = listener.accept().await?;
        tracing::debug!("WS peer connected: {peer}");
        let mut rx = tx.subscribe();

        tokio::spawn(async move {
            let ws = match tokio_tungstenite::accept_async(stream).await {
                Ok(ws) => ws,
                Err(e) => {
                    tracing::warn!("WS handshake failed from {peer}: {e}");
                    return;
                }
            };
            let (mut sink, _source) = ws.split();

            loop {
                match rx.recv().await {
                    Ok(msg) => {
                        if sink.send(Message::Text(msg.into())).await.is_err() {
                            break; // client disconnected
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(n)) => {
                        tracing::warn!("WS client {peer} lagged by {n} messages");
                    }
                    Err(broadcast::error::RecvError::Closed) => break,
                }
            }
            tracing::debug!("WS peer disconnected: {peer}");
        });
    }
}
