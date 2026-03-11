mod adsb_parser;
mod anomaly_client;
mod clock_sync;
mod coords;
mod correlator;
mod ingestor;
mod kalman;
mod mlat_solver;
mod sensors;
mod spoof_detector;
mod ws_server;

use std::time::Duration;

use clap::Parser;
use nalgebra::Vector3;
use tokio::sync::{broadcast, mpsc};
use tracing::info;

use adsb_parser::AdsbParser;
use anomaly_client::AnomalyClient;
use clock_sync::ClockSyncEngine;
use correlator::Correlator;
use ingestor::RawFrame;
use kalman::KalmanRegistry;
use mlat_solver::{GeometryCache, MlatInput};
use sensors::SensorRegistry;
use spoof_detector::SpoofDetector;
use ws_server::{AircraftState, BroadcastTx};

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

#[derive(Parser, Debug)]
#[command(name = "locus-backend", about = "Locus MLAT Engine")]
pub struct Cli {
    #[arg(long, default_value = "0.0.0.0:9001")]
    pub ws_addr: String,

    #[arg(long, default_value = "http://localhost:8000")]
    pub ml_service_url: String,
}

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Speed of light in meters per nanosecond.
const C_M_PER_NS: f64 = 0.299_792_458;
/// Rate-limit ML classify to once per 5 s per aircraft.
const CLASSIFY_INTERVAL_NS: u64 = 5_000_000_000;
/// Eviction tick — run correlator eviction every 10 ms.
const EVICTION_TICK_MS: u64 = 10;
/// Sensor health broadcast every 5 s.
const HEALTH_BROADCAST_SECS: u64 = 5;

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::from_default_env()
                .add_directive("locus_backend=debug".parse()?),
        )
        .init();

    let cli = Cli::parse();
    info!("Locus backend starting");
    info!(ws_addr = %cli.ws_addr, ml_url = %cli.ml_service_url);

    // ---- channels --------------------------------------------------------
    let (frame_tx, mut frame_rx) = mpsc::channel::<RawFrame>(4096);
    let (ws_tx, _) = broadcast::channel::<String>(4096);

    // ---- spawn ingestor (connects to Go Unix socket) ----------------------
    let _ingestor_handle = tokio::spawn(ingestor::run_ingestor(frame_tx));

    // ---- spawn WebSocket server -------------------------------------------
    {
        let ws_tx2 = ws_tx.clone();
        let addr = cli.ws_addr.clone();
        tokio::spawn(async move {
            if let Err(e) = ws_server::run_ws_server(addr, ws_tx2).await {
                tracing::error!("WS server error: {e}");
            }
        });
    }

    // ---- pipeline state --------------------------------------------------
    let mut correlator = Correlator::new();
    let mut clock_sync = ClockSyncEngine::new();
    let mut sensor_registry = SensorRegistry::new();
    let mut kalman_registry = KalmanRegistry::new();
    let mut spoof_detector = SpoofDetector::new();
    let mut adsb_parser = AdsbParser::new();
    let mut geo_cache = GeometryCache::new();
    let anomaly_client = AnomalyClient::new(cli.ml_service_url.clone());

    let mut last_frame_ns: u64 = 0;
    let mut eviction_tick =
        tokio::time::interval(Duration::from_millis(EVICTION_TICK_MS));
    let mut health_tick =
        tokio::time::interval(Duration::from_secs(HEALTH_BROADCAST_SECS));
    let mut frame_count: u64 = 0;

    // ---- main event loop -------------------------------------------------
    loop {
        tokio::select! {
            Some(frame) = frame_rx.recv() => {
                process_frame(
                    frame,
                    &mut last_frame_ns,
                    &mut frame_count,
                    &mut sensor_registry,
                    &mut correlator,
                );
            }

            _ = eviction_tick.tick() => {
                if last_frame_ns == 0 { continue; }
                let groups = correlator.evict_stale(last_frame_ns);
                for group in groups {
                    solve_and_broadcast(
                        group,
                        &mut clock_sync,
                        &sensor_registry,
                        &mut kalman_registry,
                        &mut spoof_detector,
                        &mut adsb_parser,
                        &mut geo_cache,
                        &anomaly_client,
                        &ws_tx,
                    ).await;
                }
            }

            _ = health_tick.tick() => {
                let offsets = clock_sync.export_offsets();
                let msg = serde_json::json!({
                    "type": "sensor_health",
                    "offsets": offsets,
                    "frame_count": frame_count,
                    "live_groups": correlator.live_group_count(),
                });
                let _ = ws_tx.send(msg.to_string());
            }

            _ = tokio::signal::ctrl_c() => {
                info!("Shutting down");
                break;
            }
        }
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Frame ingestion (hot path — runs for every raw frame)
// ---------------------------------------------------------------------------

fn process_frame(
    frame: RawFrame,
    last_frame_ns: &mut u64,
    frame_count: &mut u64,
    sensor_registry: &mut SensorRegistry,
    correlator: &mut Correlator,
) {
    sensor_registry.register(&frame);
    *last_frame_ns = frame.timestamp_ns();
    *frame_count += 1;

    if *frame_count % 1000 == 0 {
        tracing::debug!(
            frame_count,
            "pipeline heartbeat (last sensor {})",
            frame.sensor_id
        );
    }

    correlator.ingest(frame);
}

// ---------------------------------------------------------------------------
// Per-group MLAT → Kalman → spoof → broadcast
// ---------------------------------------------------------------------------

#[allow(clippy::too_many_arguments)]
async fn solve_and_broadcast(
    mut group: correlator::CorrelationGroup,
    clock_sync: &mut ClockSyncEngine,
    sensor_registry: &SensorRegistry,
    kalman_registry: &mut KalmanRegistry,
    spoof_detector: &mut SpoofDetector,
    adsb_parser: &mut AdsbParser,
    geo_cache: &mut GeometryCache,
    anomaly_client: &AnomalyClient,
    ws_tx: &BroadcastTx,
) {
    if group.frames.len() < 3 {
        return;
    }

    // 1. Apply clock corrections (no-op until clock sync warms up)
    clock_sync.apply_corrections(&mut group.frames);

    // 2. Collect sensor ECEF positions from registry
    let sensor_vecs: Vec<Vector3<f64>> = group
        .frames
        .iter()
        .filter_map(|f| sensor_registry.get(f.sensor_id).copied())
        .collect();
    if sensor_vecs.len() < 3 {
        return;
    }

    // 3. Assemble TDOAs (meters) relative to sensor[0]
    let ref_ts_ns = group.frames[0].timestamp_ns() as i64;
    let observed_tdoa_m: Vec<f64> = group.frames[1..]
        .iter()
        .map(|f| (f.timestamp_ns() as i64 - ref_ts_ns) as f64 * C_M_PER_NS)
        .collect();

    let icao24 = format!(
        "{:02X}{:02X}{:02X}",
        group.key.icao24[0], group.key.icao24[1], group.key.icao24[2]
    );

    // 4. Build sensor ID list for geometry cache key
    let mut sensor_ids: Vec<i64> = group.frames.iter().map(|f| f.sensor_id).collect();
    sensor_ids.sort_unstable();

    let input = MlatInput {
        sensor_positions: sensor_vecs.clone(),
        observed_tdoa_m,
    };

    // 5. Solve — use Kalman prior if available, else sensor centroid
    let prior_ecef = kalman_registry
        .get_ecef(&icao24)
        .or(sensor_registry.initial_guess_ecef);

    let solution = match mlat_solver::solve(&input, prior_ecef, geo_cache, &sensor_ids) {
        Some(s) => s,
        None => return,
    };

    tracing::debug!(
        icao24,
        gdop = solution.gdop,
        lat = solution.lat,
        lon = solution.lon,
        alt_m = solution.alt_m,
        "MLAT solution"
    );

    // 6. Kalman update
    let now_ns = group.first_seen_ns;
    kalman_registry.update(&icao24, &solution, now_ns);

    let (lat, lon, alt_m) = kalman_registry
        .get_wgs84(&icao24)
        .unwrap_or((solution.lat, solution.lon, solution.alt_m));

    // 7. Velocity (ENU → deg/s for frontend)
    let (east, north, up) = kalman_registry
        .get_velocity_enu(&icao24, lat, lon)
        .unwrap_or((0.0, 0.0, 0.0));
    let cos_lat = lat.to_radians().cos().abs().max(1e-9);
    let vel_lat = north / 111_320.0;
    let vel_lon = east / (111_320.0 * cos_lat);
    let vel_alt = up;

    // 8. Spoof detection
    let (spoof_flag, divergence_m) = spoof_detector.check(
        &icao24,
        lat,
        lon,
        alt_m,
        solution.accuracy_m,
    );

    // 9. ADS-B parsing — feed claimed position to spoof detector + clock sync beacons
    if let Some(bytes) = group.frames[0].bytes() {
        if let Some(adsb) = adsb_parser.parse(bytes, group.frames[0].timestamp_ns()) {
            spoof_detector.record_adsb(&adsb.icao24, adsb.lat, adsb.lon, adsb.alt_m);

            let beacon_ecef = coords::wgs84_to_ecef(adsb.lat, adsb.lon, adsb.alt_m);
            for i in 0..group.frames.len() {
                for j in (i + 1)..group.frames.len() {
                    if let (Some(pos_i), Some(pos_j)) = (
                        sensor_registry.get(group.frames[i].sensor_id),
                        sensor_registry.get(group.frames[j].sensor_id),
                    ) {
                        clock_sync.update_from_beacon(
                            group.frames[i].sensor_id,
                            (pos_i[0], pos_i[1], pos_i[2]),
                            group.frames[j].sensor_id,
                            (pos_j[0], pos_j[1], pos_j[2]),
                            group.frames[i].timestamp_ns(),
                            group.frames[j].timestamp_ns(),
                            beacon_ecef,
                        );
                    }
                }
            }
        }
    }

    // 10. Build AircraftState and broadcast
    let aircraft_state = AircraftState {
        icao24: icao24.clone(),
        lat,
        lon,
        alt_m,
        vel_lat,
        vel_lon,
        vel_alt,
        gdop: solution.gdop,
        confidence_ellipse_m: solution.accuracy_m,
        spoof_flag,
        divergence_m,
        anomaly_label: "unknown".to_string(),
        anomaly_confidence: 0.0,
        sensor_count: group.frames.len(),
        timestamp_ms: now_ns / 1_000_000,
    };

    let msg = serde_json::to_string(&aircraft_state).expect("AircraftState serialization");
    let _ = ws_tx.send(msg);

    // 11. Rate-limited anomaly classification (async, non-blocking)
    // Collect data under the immutable borrow, then drop it before get_mut.
    let classify_data = kalman_registry.histories.get(&icao24).and_then(|hist| {
        let elapsed = now_ns.saturating_sub(hist.last_classify_ns);
        if elapsed > CLASSIFY_INTERVAL_NS && hist.states.len() >= 5 {
            Some(hist.states.iter().copied().collect::<Vec<_>>())
        } else {
            None
        }
    });
    if let Some(hist_snap) = classify_data {
        // Mark timestamp now (immutable borrow is gone) to avoid double-fire.
        if let Some(h) = kalman_registry.histories.get_mut(&icao24) {
            h.last_classify_ns = now_ns;
        }
        let client = anomaly_client.client.clone();
        let base_url = anomaly_client.base_url.clone();
        let icao_clone = icao24.clone();
        let ws_tx2 = ws_tx.clone();
        let state_clone = aircraft_state.clone();

        tokio::spawn(async move {
            let ac = AnomalyClient { client, base_url };
            match ac.classify(&icao_clone, &hist_snap).await {
                Ok(resp) => {
                    let updated = AircraftState {
                        anomaly_label: resp.anomaly_label,
                        anomaly_confidence: resp.confidence,
                        ..state_clone
                    };
                    let msg = serde_json::to_string(&updated)
                        .expect("AircraftState serialization");
                    let _ = ws_tx2.send(msg);
                }
                Err(e) => {
                    tracing::debug!("anomaly classify error for {icao_clone}: {e}");
                }
            }
        });
    }
}
