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

/// Speed of light **in air** (m/ns). c_vacuum/1.0003 — matches mlat-server Cair constant.
/// Using vacuum speed introduces ~0.03% systematic TDOA error (~90 m at 300 km range).
const C_M_PER_NS: f64 = 0.299_702_547;
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

    // FIX-14: Per-aircraft rate limiting — skip if last solve < 1 s ago.
    let mut last_solve_ns: std::collections::HashMap<String, u64> = std::collections::HashMap::new();
    // FIX-16: Track when altitude was last observed per aircraft for degraded weighting.
    let mut last_alt_time_ns: std::collections::HashMap<String, u64> = std::collections::HashMap::new();
    // Clock beacon tracking: icao24 → cumulative obs count contributed to clock sync.
    let mut beacon_obs: std::collections::HashMap<String, u32> = std::collections::HashMap::new();

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
                // Attempt ADS-B decode on every frame.  When a valid position
                // is returned we broadcast it immediately as the primary track.
                // MLAT (via the correlator/eviction path) remains active as a
                // secondary, independent position source for spoofing detection.
                if let Some(bytes) = frame.bytes() {
                    if let Some(adsb) = adsb_parser.parse(bytes, frame.timestamp_ns()) {
                        broadcast_adsb(
                            &adsb,
                            &mut kalman_registry,
                            &mut spoof_detector,
                            &anomaly_client,
                            &ws_tx,
                            frame.timestamp_ns(),
                            1,
                            &mut beacon_obs,
                        ).await;
                    }
                }
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
                // Evict stale clock sync pairs (FIX-11).
                clock_sync.evict_stale_pairs(last_frame_ns);
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
                        &mut last_solve_ns,
                        &mut last_alt_time_ns,
                        &mut beacon_obs,
                    ).await;
                }
            }

            _ = health_tick.tick() => {
                let offsets = clock_sync.export_offsets();
                let sensors = sensor_registry.export_wgs84();
                let msg = serde_json::json!({
                    "type": "sensor_health",
                    "offsets": offsets,
                    "sensors": sensors,
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
    last_solve_ns: &mut std::collections::HashMap<String, u64>,
    last_alt_time_ns: &mut std::collections::HashMap<String, u64>,
    beacon_obs: &mut std::collections::HashMap<String, u32>,
) {
    // Step A: parse ADS-B from frame[0] early (shared by clock sync AND altitude constraint)
    let adsb_parsed = group.frames[0].bytes().and_then(|bytes| {
        adsb_parser.parse(bytes, group.frames[0].timestamp_ns())
    });

    // Step B: clock sync for ANY group >= 2 sensors (was previously gated by >= 3)
    let mut used_as_beacon = false;
    if group.frames.len() >= 2 {
        if let Some(adsb) = &adsb_parsed {
            if adsb.nuc >= 6 {
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
                            used_as_beacon = true;
                        }
                    }
                }
            }
        }
    }

    // MLAT still requires >= 3
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

    // 3. Timestamp feasibility check (FIX-13): reject groups where any pair's
    //    TDOA exceeds the physical bound imposed by their separation distance.
    //    |t_i - t_j| > d(i,j) * 1.05 / C_air + 1µs guard → impossible.
    const TDOA_GUARD_NS: f64 = 1_000.0; // 1 µs guard
    for (fi, pi) in group.frames.iter().zip(sensor_vecs.iter()) {
        for (fj, pj) in group.frames.iter().zip(sensor_vecs.iter()) {
            if fi.sensor_id >= fj.sensor_id { continue; }
            let max_tdoa_ns = (pi - pj).norm() * 1.05 / C_M_PER_NS + TDOA_GUARD_NS;
            let obs_ns = (fi.timestamp_ns() as i64 - fj.timestamp_ns() as i64).unsigned_abs() as f64;
            if obs_ns > max_tdoa_ns {
                tracing::debug!(
                    "feasibility check failed: sensors {}/{} tdoa={:.0}ns max={:.0}ns",
                    fi.sensor_id, fj.sensor_id, obs_ns, max_tdoa_ns
                );
                return;
            }
        }
    }

    // 4. Assemble TDOAs (meters) relative to sensor[0]
    let ref_ts_ns = group.frames[0].timestamp_ns() as i64;
    let observed_tdoa_m: Vec<f64> = group.frames[1..]
        .iter()
        .map(|f| (f.timestamp_ns() as i64 - ref_ts_ns) as f64 * C_M_PER_NS)
        .collect();

    let icao24 = format!(
        "{:02X}{:02X}{:02X}",
        group.key.icao24[0], group.key.icao24[1], group.key.icao24[2]
    );

    // Track beacon obs count for UI display.
    if used_as_beacon {
        *beacon_obs.entry(icao24.clone()).or_insert(0) += 1;
    }

    // FIX-14: Per-aircraft rate limiting — skip if < 1 s since last solve.
    let now_ns = group.first_seen_ns;
    if let Some(&last) = last_solve_ns.get(&icao24) {
        if now_ns.saturating_sub(last) < 1_000_000_000 {
            return;
        }
    }

    // 5. Build sensor ID list for geometry cache key
    let mut sensor_ids: Vec<i64> = group.frames.iter().map(|f| f.sensor_id).collect();
    sensor_ids.sort_unstable();

    // 5b. ADS-B altitude constraint (adsb_parsed already computed above).
    let adsb_alt_m = adsb_parsed.as_ref().map(|a| a.alt_m);

    // FIX-16: Altitude age degradation — compute age of the altitude report.
    let alt_age_seconds = if adsb_alt_m.is_some() {
        // Fresh altitude in this frame — update tracking and report age = 0.
        last_alt_time_ns.insert(icao24.clone(), now_ns);
        0.0
    } else {
        last_alt_time_ns.get(&icao24)
            .map(|&last| now_ns.saturating_sub(last) as f64 / 1e9)
            .unwrap_or(999.0) // Unknown → very stale → altitude constraint disabled
    };

    // FIX-4: Per-measurement variance from clock sync (meters²).
    // Reference sensor (index 0) uses a fixed 50 ns timing uncertainty.
    let ref_id = group.frames[0].sensor_id;
    let ref_sigma_m = 50.0 * C_M_PER_NS; // 50 ns * C_air ≈ 15 m
    let mut tdoa_variance_m2 = vec![ref_sigma_m * ref_sigma_m]; // sensor 0
    for f in &group.frames[1..] {
        let var_ns2 = clock_sync.get_pair_variance_ns2(f.sensor_id, ref_id);
        tdoa_variance_m2.push(var_ns2 * C_M_PER_NS * C_M_PER_NS);
    }

    let input = MlatInput {
        sensor_positions: sensor_vecs.clone(),
        observed_tdoa_m,
        tdoa_variance_m2,
        adsb_alt_m,
        alt_age_seconds,
    };

    // 6. Solve — use Kalman prior if available, else sensor centroid
    let prior_ecef = kalman_registry
        .get_ecef(&icao24)
        .or(sensor_registry.initial_guess_ecef);

    let solution = match mlat_solver::solve(&input, prior_ecef, geo_cache, &sensor_ids) {
        Some(s) => s,
        None => return,
    };

    // Record this solve (FIX-14).
    last_solve_ns.insert(icao24.clone(), now_ns);

    tracing::debug!(
        icao24,
        gdop = solution.gdop,
        dof = solution.dof,
        lat = solution.lat,
        lon = solution.lon,
        alt_m = solution.alt_m,
        "MLAT solution"
    );

    // 7. Kalman update
    kalman_registry.update(&icao24, &solution, now_ns);

    let (lat, lon, alt_m) = kalman_registry
        .get_wgs84(&icao24)
        .unwrap_or((solution.lat, solution.lon, solution.alt_m));

    // 8. Velocity (ENU → deg/s for frontend)
    let (east, north, up) = kalman_registry
        .get_velocity_enu(&icao24, lat, lon)
        .unwrap_or((0.0, 0.0, 0.0));
    let cos_lat = lat.to_radians().cos().abs().max(1e-9);
    let vel_lat = north / 111_320.0;
    let vel_lon = east / (111_320.0 * cos_lat);
    let vel_alt = up;

    // 9. Spoof detection
    let (spoof_flag, divergence_m) = spoof_detector.check(
        &icao24,
        lat,
        lon,
        alt_m,
        solution.accuracy_m,
    );

    // 10. Record ADS-B position for spoof detection.
    if let Some(adsb) = &adsb_parsed {
        spoof_detector.record_adsb(&adsb.icao24, adsb.lat, adsb.lon, adsb.alt_m);
    }

    // 11. Build AircraftState and broadcast
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
        dof: solution.dof,
        sensor_ids: sensor_ids.clone(),
        is_clock_beacon: false, // MLAT track — beacon flag carried by ADS-B broadcast
        beacon_obs_count: beacon_obs.get(&icao24).copied().unwrap_or(0),
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

// ---------------------------------------------------------------------------
// ADS-B primary track broadcast
// ---------------------------------------------------------------------------
// Called on every frame that yields a valid CPR-decoded position.
// Produces a Kalman-smoothed AircraftState with gdop=0 (indicating ADS-B source).
// MLAT-sourced updates (from solve_and_broadcast) will naturally override these
// when MLAT solutions are available, since both share the same Kalman registry.

#[allow(clippy::too_many_arguments)]
async fn broadcast_adsb(
    adsb: &adsb_parser::AdsbPosition,
    kalman_registry: &mut KalmanRegistry,
    spoof_detector: &mut SpoofDetector,
    anomaly_client: &AnomalyClient,
    ws_tx: &BroadcastTx,
    now_ns: u64,
    sensor_count: usize,
    beacon_obs: &mut std::collections::HashMap<String, u32>,
) {
    let icao24 = &adsb.icao24;

    // Build a synthetic MlatSolution so we can reuse the Kalman registry.
    let solution = mlat_solver::MlatSolution {
        lat: adsb.lat,
        lon: adsb.lon,
        alt_m: adsb.alt_m,
        ecef: {
            let (x, y, z) = coords::wgs84_to_ecef(adsb.lat, adsb.lon, adsb.alt_m);
            nalgebra::Vector3::new(x, y, z)
        },
        gdop: 0.0,          // 0.0 signals ADS-B source (not MLAT)
        accuracy_m: 500.0,  // ADS-B typical horizontal accuracy (~500 m NUC=5)
        dof: 0,             // not applicable for ADS-B
        covariance: nalgebra::Matrix3::identity() * (500.0_f64 * 500.0),
    };

    kalman_registry.update(icao24, &solution, now_ns);

    let (lat, lon, alt_m) = kalman_registry
        .get_wgs84(icao24)
        .unwrap_or((adsb.lat, adsb.lon, adsb.alt_m));

    let (east, north, up) = kalman_registry
        .get_velocity_enu(icao24, lat, lon)
        .unwrap_or((0.0, 0.0, 0.0));
    let cos_lat = lat.to_radians().cos().abs().max(1e-9);
    let vel_lat = north / 111_320.0;
    let vel_lon = east / (111_320.0 * cos_lat);
    let vel_alt = up;

    let (spoof_flag, divergence_m) = spoof_detector.check(icao24, lat, lon, alt_m, 500.0);
    // Record this ADS-B position as the "claimed" position for spoof detection.
    spoof_detector.record_adsb(icao24, adsb.lat, adsb.lon, adsb.alt_m);

    let aircraft_state = AircraftState {
        icao24: icao24.clone(),
        lat,
        lon,
        alt_m,
        vel_lat,
        vel_lon,
        vel_alt,
        gdop: 0.0,
        confidence_ellipse_m: 500.0,
        spoof_flag,
        divergence_m,
        anomaly_label: "unknown".to_string(),
        anomaly_confidence: 0.0,
        sensor_count,
        timestamp_ms: now_ns / 1_000_000,
        dof: 0,
        sensor_ids: vec![],
        is_clock_beacon: adsb.nuc >= 6 && beacon_obs.contains_key(icao24.as_str()),
        beacon_obs_count: beacon_obs.get(icao24.as_str()).copied().unwrap_or(0),
    };

    tracing::debug!(
        icao24,
        lat,
        lon,
        alt_m,
        "ADS-B position broadcast"
    );

    let msg = serde_json::to_string(&aircraft_state).expect("AircraftState serialization");
    let _ = ws_tx.send(msg);

    // Rate-limited anomaly classification
    let classify_data = kalman_registry.histories.get(icao24).and_then(|hist| {
        let elapsed = now_ns.saturating_sub(hist.last_classify_ns);
        if elapsed > CLASSIFY_INTERVAL_NS && hist.states.len() >= 5 {
            Some(hist.states.iter().copied().collect::<Vec<_>>())
        } else {
            None
        }
    });
    if let Some(hist_snap) = classify_data {
        if let Some(h) = kalman_registry.histories.get_mut(icao24) {
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
