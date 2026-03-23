// WebSocket broadcast server — bidirectional.
// Accepts TCP connections, upgrades to WebSocket, fans out JSON broadcasts,
// and handles per-client virtual sensor commands for planning-mode GDOP.

use std::net::SocketAddr;
use std::sync::Arc;

use futures_util::stream::SplitSink;
use futures_util::{SinkExt, StreamExt};
use serde::{Deserialize, Serialize};
use tokio::net::TcpListener;
use nalgebra::Vector3;
use tokio::sync::{broadcast, mpsc, RwLock};
use tokio_tungstenite::tungstenite::Message;
use tokio_tungstenite::WebSocketStream;

use crate::sensors::SensorRegistry;
use crate::virtual_sensors::{VirtualSensorId, VirtualSensorRegistry};

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
    /// Vertical DOP - quality indicator for altitude estimate.
    pub vdop: f64,
    /// True if altitude is within valid range (-500m to 15,000m).
    pub altitude_valid: bool,
    /// 2-sigma horizontal confidence ellipse radius (meters).
    pub confidence_ellipse_m: f64,
    pub spoof_flag: bool,
    pub divergence_m: f64,
    /// "normal" | "anomalous" | "unknown"
    pub anomaly_label: String,
    pub anomaly_confidence: f64,
    pub sensor_count: usize,
    /// Sensors with Stable or Marginal clock calibration that contributed to this fix.
    /// May be < sensor_count during warmup. Zero for ADS-B source.
    pub calibrated_sensor_count: usize,
    pub timestamp_ms: u64,
    /// Degrees of freedom (n_receivers + 1_if_alt_constrained - 4). 0 for ADS-B source.
    pub dof: i32,
    /// IDs of sensors that contributed to this MLAT fix. Empty for ADS-B.
    pub sensor_ids: Vec<i64>,
    /// True when this aircraft is a high-accuracy ADS-B source (NUC ≥ 6) actively
    /// used to calibrate sensor pair clocks.
    pub is_clock_beacon: bool,
    /// How many clock-sync observations this aircraft has contributed so far.
    pub beacon_obs_count: u32,
    /// Observation mode: "full_mlat" | "semi_mlat" | "adsb" | "prediction_only"
    #[serde(skip_serializing_if = "Option::is_none")]
    pub observation_mode: Option<String>,
    /// Semi-MLAT DOP (only present for semi-MLAT observations).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sdop: Option<f64>,
    /// Callsign from OpenSky (e.g. "BAW123"). None until enriched.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub callsign: Option<String>,
    /// Transponder squawk code from OpenSky (e.g. "1000"). None until enriched.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub squawk: Option<String>,
    /// Country of registration from OpenSky. None until enriched.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub origin_country: Option<String>,
}

// ---------------------------------------------------------------------------
// Client → Server messages (virtual sensor planning)
// ---------------------------------------------------------------------------

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ClientMessage {
    AddVirtualSensor { lat: f64, lon: f64, alt_m: f64 },
    RemoveVirtualSensor { id: u64 },
    ClearVirtualSensors,
    SetPlanningAltitude { altitude_m: f64 },
    SetLiveHeatmapAltitude { altitude_m: f64 },
    RequestAircraftInfo { icao24: String },
}

// ---------------------------------------------------------------------------
// Server → Client responses (per-client, NOT broadcast)
// ---------------------------------------------------------------------------

#[derive(Debug, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ClientResponse {
    VirtualSensorAdded { id: u64, lat: f64, lon: f64, alt_m: f64 },
    VirtualSensorRemoved { id: u64 },
    VirtualSensorsCleared,
    VirtualSensorError { message: String },
    AircraftInfoResult {
        icao24:         String,
        callsign:       Option<String>,
        squawk:         Option<String>,
        origin_country: Option<String>,
    },
}

// ---------------------------------------------------------------------------
// Server
// ---------------------------------------------------------------------------

/// Listen on `addr`, accept WebSocket connections, broadcast from `rx`,
/// and handle per-client virtual sensor commands.
pub async fn run_ws_server(
    addr: String,
    tx: BroadcastTx,
    cache: Arc<RwLock<Option<String>>>,
    sensor_registry: Arc<RwLock<SensorRegistry>>,
    live_altitude: Arc<RwLock<f64>>,
    heatmap_tx: mpsc::Sender<crate::gdop_heatmap::HeatmapRequest>,
    opensky_cache: crate::opensky_client::OpenSkyCache,
    opensky_http: Arc<reqwest::Client>,
) -> anyhow::Result<()> {
    let listener = TcpListener::bind(&addr).await?;
    tracing::info!("WebSocket server listening on {addr}");

    loop {
        let (stream, peer) = listener.accept().await?;
        tracing::debug!("WS peer connected: {peer}");
        let mut rx = tx.subscribe();
        let cache = Arc::clone(&cache);
        let sensor_registry = Arc::clone(&sensor_registry);
        let live_altitude = Arc::clone(&live_altitude);
        let heatmap_tx = heatmap_tx.clone();
        let opensky_cache = Arc::clone(&opensky_cache);
        let opensky_http  = Arc::clone(&opensky_http);

        tokio::spawn(async move {
            let ws = match tokio_tungstenite::accept_async(stream).await {
                Ok(ws) => ws,
                Err(e) => {
                    tracing::warn!("WS handshake failed from {peer}: {e}");
                    return;
                }
            };

            let (mut sink, mut source) = ws.split();
            let mut virtual_registry = VirtualSensorRegistry::new(20);
            let mut planning_altitude_m: f64 = crate::consts::DEFAULT_HEATMAP_ALTITUDE_M; // default FL300

            // Send cached heatmap immediately — no wait for next 60s tick
            {
                let cached = cache.read().await;
                if let Some(json) = cached.as_ref() {
                    let _ = sink.send(Message::Text(json.clone().into())).await;
                }
            }

            loop {
                tokio::select! {
                    // Outbound: broadcast messages from the live pipeline
                    result = rx.recv() => {
                        match result {
                            Ok(msg) => {
                                if sink.send(Message::Text(msg.into())).await.is_err() {
                                    break;
                                }
                            }
                            Err(broadcast::error::RecvError::Lagged(n)) => {
                                tracing::warn!("WS client {peer} lagged by {n} messages");
                            }
                            Err(broadcast::error::RecvError::Closed) => break,
                        }
                    }

                    // Inbound: virtual sensor commands from this client
                    result = source.next() => {
                        match result {
                            Some(Ok(Message::Text(text))) => {
                                handle_client_msg(
                                    &text,
                                    &mut virtual_registry,
                                    &mut sink,
                                    &sensor_registry,
                                    peer,
                                    &mut planning_altitude_m,
                                    &live_altitude,
                                    &heatmap_tx,
                                    &opensky_cache,
                                    &opensky_http,
                                ).await;
                            }
                            Some(Ok(Message::Close(_))) | None => break,
                            Some(Err(e)) => {
                                tracing::warn!("WS client {peer} error: {e}");
                                break;
                            }
                            _ => {} // Ping/Pong/Binary — ignore
                        }
                    }
                }
            }

            // virtual_registry is dropped here — ephemeral, no shared state
            tracing::debug!("WS peer disconnected: {peer}");
        });
    }
}

// ---------------------------------------------------------------------------
// Per-client message handler
// ---------------------------------------------------------------------------

async fn handle_client_msg(
    text: &str,
    vreg: &mut VirtualSensorRegistry,
    sink: &mut SplitSink<WebSocketStream<tokio::net::TcpStream>, Message>,
    sreg: &Arc<RwLock<SensorRegistry>>,
    peer: SocketAddr,
    planning_alt: &mut f64,
    live_alt: &Arc<RwLock<f64>>,
    heatmap_tx: &mpsc::Sender<crate::gdop_heatmap::HeatmapRequest>,
    opensky_cache: &crate::opensky_client::OpenSkyCache,
    opensky_http: &reqwest::Client,
) {
    let msg: ClientMessage = match serde_json::from_str(text) {
        Ok(m) => m,
        Err(e) => {
            tracing::debug!("unparseable WS message from {peer}: {e}");
            return;
        }
    };

    match msg {
        ClientMessage::AddVirtualSensor { lat, lon, alt_m } => {
            match vreg.add(lat, lon, alt_m) {
                Ok(id) => {
                    send_json(sink, &ClientResponse::VirtualSensorAdded {
                        id: id.0, lat, lon, alt_m,
                    }).await;
                    recompute_planning_gdop(vreg, sink, sreg, *planning_alt).await;
                }
                Err(e) => {
                    send_json(sink, &ClientResponse::VirtualSensorError { message: e }).await;
                }
            }
        }
        ClientMessage::RemoveVirtualSensor { id } => {
            vreg.remove(VirtualSensorId(id));
            send_json(sink, &ClientResponse::VirtualSensorRemoved { id }).await;
            recompute_planning_gdop(vreg, sink, sreg, *planning_alt).await;
        }
        ClientMessage::ClearVirtualSensors => {
            vreg.clear();
            send_json(sink, &ClientResponse::VirtualSensorsCleared).await;
            // No recompute — client exits planning mode, live heatmap resumes
        }
        ClientMessage::SetPlanningAltitude { altitude_m } => {
            if crate::consts::ALLOWED_PLANNING_ALTITUDES.contains(&altitude_m) {
                *planning_alt = altitude_m;
                if vreg.count() > 0 {
                    recompute_planning_gdop(vreg, sink, sreg, *planning_alt).await;
                }
            } else {
                send_json(sink, &ClientResponse::VirtualSensorError {
                    message: format!("Invalid altitude: {altitude_m}m (must be FL050/FL100/FL300)"),
                }).await;
            }
        }
        ClientMessage::RequestAircraftInfo { icao24 } => {
            let key = icao24.to_lowercase();

            // Check TTL cache first
            let cached = {
                let cache = opensky_cache.read().await;
                cache.get(&key).and_then(|c| {
                    if c.fetched_at.elapsed() < crate::opensky_client::CACHE_TTL {
                        Some(c.info.clone())
                    } else {
                        None
                    }
                })
            };

            let info = if let Some(info) = cached {
                info
            } else {
                let fetched = crate::opensky_client::fetch_aircraft_info(opensky_http, &key).await
                    .unwrap_or_default();
                opensky_cache.write().await.insert(key.clone(), crate::opensky_client::CachedInfo {
                    info: fetched.clone(),
                    fetched_at: std::time::Instant::now(),
                });
                fetched
            };

            send_json(sink, &ClientResponse::AircraftInfoResult {
                icao24:         icao24.clone(),
                callsign:       info.callsign,
                squawk:         info.squawk,
                origin_country: info.origin_country,
            }).await;
        }
        ClientMessage::SetLiveHeatmapAltitude { altitude_m } => {
            if crate::consts::ALLOWED_PLANNING_ALTITUDES.contains(&altitude_m) {
                *live_alt.write().await = altitude_m;
                let sensor_ecef: Vec<Vector3<f64>> = {
                    let reg = sreg.read().await;
                    reg.export_wgs84()
                        .iter()
                        .map(|s| {
                            let (x, y, z) = crate::coords::wgs84_to_ecef(s.lat, s.lon, s.alt_m);
                            Vector3::new(x, y, z)
                        })
                        .collect()
                };
                if sensor_ecef.len() >= 3 {
                    let _ = heatmap_tx.try_send(crate::gdop_heatmap::HeatmapRequest::Compute {
                        sensor_ecef,
                        bounds: None,
                        resolution_deg: crate::consts::PLANNING_GDOP_RESOLUTION_DEG,
                        altitude_m,
                    });
                }
            } else {
                tracing::warn!("Invalid live heatmap altitude {altitude_m} from {peer}");
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Planning-mode GDOP recomputation (per-client, never broadcast)
// ---------------------------------------------------------------------------

async fn recompute_planning_gdop(
    vreg: &VirtualSensorRegistry,
    sink: &mut SplitSink<WebSocketStream<tokio::net::TcpStream>, Message>,
    sreg: &Arc<RwLock<SensorRegistry>>,
    altitude_m: f64,
) {
    // Snapshot both registries — release async lock before spawn_blocking.
    // SensorRegistry: #[derive(Clone)], VirtualSensorRegistry: #[derive(Clone)]
    let real_snap = sreg.read().await.clone();
    let virt_snap = vreg.clone();

    let result = tokio::task::spawn_blocking(move || {
        crate::gdop_heatmap::compute_grid_with_virtual(
            &real_snap,
            &virt_snap,
            altitude_m,
            0.1,    // 0.1° resolution — same as live heatmap
        )
    })
    .await;

    match result {
        Ok(Ok(grid)) => {
            if let Ok(json) = serde_json::to_string(&grid) {
                let _ = sink.send(Message::Text(json.into())).await;
            }
        }
        Ok(Err(e)) => {
            send_json(sink, &ClientResponse::VirtualSensorError {
                message: format!("GDOP computation failed: {e}"),
            })
            .await;
        }
        Err(e) => tracing::error!("spawn_blocking panic in planning GDOP: {e}"),
    }
}

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------

async fn send_json<T: Serialize>(
    sink: &mut SplitSink<WebSocketStream<tokio::net::TcpStream>, Message>,
    payload: &T,
) {
    if let Ok(json) = serde_json::to_string(payload) {
        let _ = sink.send(Message::Text(json.into())).await;
    }
}
