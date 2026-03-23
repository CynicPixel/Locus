mod adsb_parser;
mod anomaly_client;
mod clock_sync;
pub mod consts;
mod coords;
mod correlator;
mod gdop_heatmap;
mod global_clock_solver;
mod ingestor;
mod kalman;
mod mlat_solver;
mod opensky_client;
mod semi_mlat_solver;
mod sensors;
mod spoof_detector;
mod virtual_sensors;
mod ws_server;

use std::sync::Arc;
use std::time::Duration;

use clap::Parser;
use nalgebra::Vector3;
use tokio::sync::{broadcast, mpsc, RwLock};
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

// Speed of light, timing intervals, and broadcast periods are defined in crate::consts.

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Classify topology health based on global metrics.
///
/// Returns: "excellent" | "good" | "marginal" | "poor"
fn classify_topology_health(
    global_rms_ns: f64,
    connectivity: f64,
    condition_number: f64,
) -> &'static str {
    if condition_number > crate::consts::MAX_CONDITION_NUMBER {
        return "poor"; // Ill-conditioned system
    }
    if global_rms_ns < 50.0 && connectivity > 0.7 {
        "excellent"
    } else if global_rms_ns < 100.0 && connectivity > 0.5 {
        "good"
    } else if global_rms_ns < 300.0 && connectivity > 0.3 {
        "marginal"
    } else {
        "poor"
    }
}

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
    let (frame_tx, mut frame_rx) = mpsc::channel::<RawFrame>(crate::consts::FRAME_CHANNEL_CAPACITY);
    let (ws_tx, _) = broadcast::channel::<String>(crate::consts::WS_BROADCAST_CAPACITY);

    // ---- heatmap cache: last computed result served instantly to new clients
    let heatmap_cache: Arc<RwLock<Option<String>>> = Arc::new(RwLock::new(None));

    // ---- live heatmap altitude: shared between main loop and WS server ------
    let live_heatmap_altitude_m: Arc<RwLock<f64>> =
        Arc::new(RwLock::new(crate::consts::DEFAULT_HEATMAP_ALTITUDE_M)); // FL300

    // ---- OpenSky enrichment: on-demand per-aircraft, TTL-cached ---------------
    let opensky_cache = opensky_client::new_cache();
    let opensky_http = Arc::new(opensky_client::new_client());

    // ---- spawn ingestor (connects to Go Unix socket) ----------------------
    let _ingestor_handle = tokio::spawn(ingestor::run_ingestor(frame_tx));

    // ---- pipeline state --------------------------------------------------
    let mut correlator = Correlator::new();
    let mut clock_sync = ClockSyncEngine::new();
    let sensor_registry = Arc::new(RwLock::new(SensorRegistry::new()));

    // ---- spawn GDOP heatmap background task -------------------------------
    let heatmap_tx = gdop_heatmap::spawn_heatmap_task(ws_tx.clone(), Arc::clone(&heatmap_cache));
    let heatmap_tx_for_ws = heatmap_tx.clone();

    // ---- spawn WebSocket server -------------------------------------------
    {
        let ws_tx2 = ws_tx.clone();
        let addr = cli.ws_addr.clone();
        let cache = Arc::clone(&heatmap_cache);
        let registry_clone = Arc::clone(&sensor_registry);
        let live_alt = Arc::clone(&live_heatmap_altitude_m);
        let htx = heatmap_tx_for_ws;
        let osky_cache = Arc::clone(&opensky_cache);
        let osky_http = Arc::clone(&opensky_http);
        tokio::spawn(async move {
            if let Err(e) = ws_server::run_ws_server(
                addr,
                ws_tx2,
                cache,
                registry_clone,
                live_alt,
                htx,
                osky_cache,
                osky_http,
            )
            .await
            {
                tracing::error!("WS server error: {e}");
            }
        });
    }

    let mut kalman_registry = KalmanRegistry::new();
    let mut spoof_detector = SpoofDetector::new();
    let mut adsb_parser = AdsbParser::new();
    let mut geo_cache = GeometryCache::new();
    let anomaly_client = AnomalyClient::new(cli.ml_service_url.clone());

    // FIX-14: Per-aircraft rate limiting — skip if last solve < 1 s ago.
    let mut last_solve_ns: std::collections::HashMap<String, u64> =
        std::collections::HashMap::new();
    // FIX-16: Track when altitude was last observed per aircraft for degraded weighting.
    let mut last_alt_time_ns: std::collections::HashMap<String, u64> =
        std::collections::HashMap::new();
    // Clock beacon tracking: icao24 → cumulative obs count contributed to clock sync.
    let mut beacon_obs: std::collections::HashMap<String, u32> = std::collections::HashMap::new();

    let mut last_frame_ns: u64 = 0;
    let mut eviction_tick =
        tokio::time::interval(Duration::from_millis(crate::consts::EVICTION_TICK_MS));
    let mut health_tick =
        tokio::time::interval(Duration::from_secs(crate::consts::HEALTH_BROADCAST_SECS));
    let mut heatmap_tick =
        tokio::time::interval(Duration::from_secs(crate::consts::HEATMAP_TICK_SECS));
    let mut frame_count: u64 = 0;
    let mut last_sensor_count: usize = 0;
    let mut first_heatmap_triggered = false;

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
                            &opensky_cache,
                        ).await;
                    }
                }
                {
                    let mut registry = sensor_registry.write().await;
                    process_frame(
                        frame,
                        &mut last_frame_ns,
                        &mut frame_count,
                        &mut *registry,
                        &mut correlator,
                    );
                }
            }

            _ = eviction_tick.tick() => {
                if last_frame_ns == 0 { continue; }
                let groups = correlator.evict_stale(last_frame_ns);
                // Evict stale clock sync pairs (FIX-11).
                clock_sync.evict_stale_pairs(last_frame_ns);
                let registry_read = sensor_registry.read().await;
                for group in groups {
                    solve_and_broadcast(
                        group,
                        &mut clock_sync,
                        &*registry_read,
                        &mut kalman_registry,
                        &mut spoof_detector,
                        &mut adsb_parser,
                        &mut geo_cache,
                        &anomaly_client,
                        &ws_tx,
                        &mut last_solve_ns,
                        &mut last_alt_time_ns,
                        &mut beacon_obs,
                        &opensky_cache,
                    ).await;
                }
            }

            _ = health_tick.tick() => {
                // Trigger global clock solve (async, non-blocking).
                clock_sync.trigger_global_solve(last_frame_ns, sensor_registry.clone());

                // Update cached global result.
                clock_sync.update_cached_result().await;

                // Export legacy pairwise offsets.
                let offsets = clock_sync.export_offsets();

                // Export global solver result (if available).
                let global_result = clock_sync.export_global_result();

                let sensors = sensor_registry.read().await.export_wgs84();

                // Legacy sensor_health message.
                let msg = serde_json::json!({
                    "type": "sensor_health",
                    "offsets": offsets,
                    "sensors": sensors,
                    "frame_count": frame_count,
                    "live_groups": correlator.live_group_count(),
                });
                let _ = ws_tx.send(msg.to_string());

                // New global clock sync message (with Phase 3 analytics).
                if let Some(result) = global_result {
                    let global_msg = serde_json::json!({
                        "type": "clock_sync_global",
                        "success": result.success,
                        "num_sensors": result.topology.num_sensors,
                        "num_edges": result.topology.num_edges,
                        "global_rms_ns": result.topology.global_rms_ns,
                        "connectivity": result.topology.connectivity,
                        "condition_number": result.topology.condition_number,
                        "health": classify_topology_health(
                            result.topology.global_rms_ns,
                            result.topology.connectivity,
                            result.topology.condition_number
                        ),
                        "sensors": result.offsets.iter().map(|(&sid, &offset)| {
                            serde_json::json!({
                                "sensor_id": sid,
                                "offset_ns": offset,
                                "uncertainty_ns": result.uncertainties.get(&sid).copied().unwrap_or(0.0),
                                "drift_ns_per_s": result.drift_rates.get(&sid).copied().unwrap_or(0.0),
                                "status": result.statuses.get(&sid).copied().unwrap_or(crate::global_clock_solver::SensorStatus::Disconnected),
                                "allan_deviation": result.allan_deviations.get(&sid).cloned(),
                            })
                        }).collect::<Vec<_>>(),
                        "edges": result.edges.iter().map(|edge| {
                            serde_json::json!({
                                "sensor_i": edge.sensor_i,
                                "sensor_j": edge.sensor_j,
                                "weight": edge.weight,
                                "obs_count": edge.obs_count,
                                "mad_ns": edge.mad_ns,
                                "baseline_m": edge.baseline_m,
                            })
                        }).collect::<Vec<_>>(),
                    });
                    let _ = ws_tx.send(global_msg.to_string());
                }
            }

            _ = heatmap_tick.tick() => {
                // GDOP heatmap: compute theoretical coverage quality field.
                // Triggers every 60s OR on significant sensor topology change.
                tracing::debug!("GDOP heatmap tick fired");

                let sensors_lock = sensor_registry.read().await;
                let sensor_ecef: Vec<Vector3<f64>> = sensors_lock
                    .export_wgs84()
                    .iter()
                    .map(|s| {
                        let (x, y, z) = crate::coords::wgs84_to_ecef(s.lat, s.lon, s.alt_m);
                        Vector3::new(x, y, z)
                    })
                    .collect();
                let count = sensor_ecef.len();
                drop(sensors_lock);

                tracing::debug!(count, last_sensor_count, "checking sensor count");

                // Only recompute if significant topology change (>20% sensor count change)
                let change_ratio = if last_sensor_count > 0 {
                    (count as f64 - last_sensor_count as f64).abs() / last_sensor_count as f64
                } else {
                    1.0
                };

                let should_compute = count >= crate::consts::MIN_SENSORS_FIRST_HEATMAP && (
                    change_ratio > 0.2
                    || last_sensor_count == 0
                    || (!first_heatmap_triggered && count >= crate::consts::MIN_SENSORS_FIRST_HEATMAP)  // Immediate first computation
                );

                if should_compute {
                    tracing::info!(
                        num_sensors = count,
                        change_ratio,
                        first_run = !first_heatmap_triggered,
                        "triggering GDOP heatmap computation"
                    );

                    let _ = heatmap_tx.try_send(gdop_heatmap::HeatmapRequest::Compute {
                        sensor_ecef,
                        bounds: None,  // Auto-compute from sensors
                        resolution_deg: 0.1,
                        altitude_m: *live_heatmap_altitude_m.read().await,
                    });

                    last_sensor_count = count;
                    first_heatmap_triggered = true;  // Mark as triggered
                } else {
                    tracing::debug!(count, change_ratio, "GDOP heatmap skipped");
                }
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
// Semi-MLAT solver for 2-sensor observations (requires existing Kalman track)
// ---------------------------------------------------------------------------

async fn solve_semi_mlat_and_broadcast(
    mut group: correlator::CorrelationGroup,
    clock_sync: &mut ClockSyncEngine,
    sensor_registry: &SensorRegistry,
    kalman_registry: &mut KalmanRegistry,
    adsb_parser: &mut AdsbParser,
    ws_tx: &BroadcastTx,
) {
    // Extract ICAO24 for track lookup
    let icao24 = format!(
        "{:02X}{:02X}{:02X}",
        group.key.icao24[0], group.key.icao24[1], group.key.icao24[2]
    );

    // Require existing Kalman track (no cold starts from 2-sensor data)
    let kalman_prior = match kalman_registry.get_prior(&icao24) {
        Some(prior) => prior,
        None => {
            tracing::debug!(icao24 = %icao24, "2-sensor obs discarded: no existing track");
            return;
        }
    };

    // Parse ADS-B to extract altitude (CRITICAL FIX: enable altitude constraint)
    let adsb_parsed = group.frames[0]
        .bytes()
        .and_then(|bytes| adsb_parser.parse(bytes, group.frames[0].timestamp_ns()));

    // Apply clock corrections
    clock_sync.apply_corrections(&mut group.frames);

    // Collect sensor ECEF positions (should be exactly 2)
    let sensor_vecs: Vec<Vector3<f64>> = group
        .frames
        .iter()
        .filter_map(|f| sensor_registry.get(f.sensor_id).copied())
        .collect();
    if sensor_vecs.len() != 2 {
        tracing::debug!(icao24 = %icao24, "2-sensor group corrupted (missing sensor positions)");
        return;
    }

    // Compute TDOA (meters) relative to sensor[0]
    let ref_ts_ns = group.frames[0].timestamp_ns() as i64;
    let tdoa_ns = group.frames[1].timestamp_ns() as i64 - ref_ts_ns;
    let observed_tdoa_m = tdoa_ns as f64 * crate::consts::C_M_PER_NS;

    // TDOA variance from clock sync (or fallback to 300m if unavailable)
    let ref_id = group.frames[0].sensor_id;
    let other_id = group.frames[1].sensor_id;
    let var_ns2 = clock_sync.get_pair_variance_ns2(other_id, ref_id);
    let tdoa_variance_m2 = var_ns2 * crate::consts::C_M_PER_NS * crate::consts::C_M_PER_NS;

    // Extract altitude from ADS-B (if present, it's fresh from current frame)
    let adsb_alt_m = adsb_parsed.as_ref().map(|a| a.alt_m);
    let alt_age_seconds = if adsb_alt_m.is_some() {
        0.0 // Fresh altitude from current frame
    } else {
        999.0 // No altitude → constraint disabled (age > 926m threshold)
    };

    // Build semi-MLAT input (now with altitude constraint enabled)
    let input = semi_mlat_solver::SemiMlatInput {
        sensor_positions: [sensor_vecs[0], sensor_vecs[1]],
        observed_tdoa_m,
        tdoa_variance_m2,
        kalman_prior,
        adsb_alt_m,
        alt_age_seconds,
    };

    // Solve
    let solution = match semi_mlat_solver::solve_semi_mlat(&input) {
        Some(s) => s,
        None => {
            tracing::debug!(icao24 = %icao24, "semi-MLAT solve failed");
            return;
        }
    };

    tracing::debug!(
        icao24 = %icao24,
        sdop = solution.sdop,
        accuracy_m = solution.accuracy_m,
        lat = solution.lat,
        lon = solution.lon,
        alt_m = solution.alt_m,
        "semi-MLAT solution"
    );

    // Update Kalman filter
    let now_ns = group.first_seen_ns;
    kalman_registry.update_from_semi_mlat(&icao24, &solution, now_ns);

    // Get smoothed position from Kalman
    let (lat, lon, alt_m) =
        kalman_registry
            .get_wgs84(&icao24)
            .unwrap_or((solution.lat, solution.lon, solution.alt_m));

    // Velocity (ENU → deg/s for frontend)
    let (east, north, up) = kalman_registry
        .get_velocity_enu(&icao24, lat, lon)
        .unwrap_or((0.0, 0.0, 0.0));
    let cos_lat = lat.to_radians().cos().abs().max(1e-9);
    let vel_lat = north / crate::consts::METERS_PER_DEGREE_LAT;
    let vel_lon = east / (crate::consts::METERS_PER_DEGREE_LAT * cos_lat);
    let vel_alt = up;

    // Build AircraftState (observation_mode = "semi_mlat")
    let sensor_ids: Vec<i64> = group.frames.iter().map(|f| f.sensor_id).collect();
    let aircraft_state = AircraftState {
        icao24: icao24.clone(),
        lat,
        lon,
        alt_m,
        vel_lat,
        vel_lon,
        vel_alt,
        gdop: 0.0,            // N/A for semi-MLAT
        vdop: 0.0,            // N/A for semi-MLAT
        altitude_valid: true, // semi-MLAT uses Kalman prior, assumed valid
        confidence_ellipse_m: solution.accuracy_m,
        spoof_flag: false, // Don't run spoof detector on semi-MLAT (lower accuracy)
        divergence_m: 0.0, // N/A
        anomaly_label: "unknown".to_string(), // Don't classify on semi-MLAT
        anomaly_confidence: 0.0,
        sensor_count: 2,
        timestamp_ms: now_ns / 1_000_000,
        dof: 1, // 4 residuals - 3 unknowns = 1 DOF
        sensor_ids,
        is_clock_beacon: false,
        beacon_obs_count: 0,
        calibrated_sensor_count: 0, // semi-MLAT: no full clock calibration check
        observation_mode: Some("semi_mlat".to_string()),
        sdop: Some(solution.sdop),
        callsign: None,
        squawk: None,
        origin_country: None,
    };

    // Broadcast
    let msg = serde_json::to_string(&aircraft_state).expect("AircraftState serialization");
    let _ = ws_tx.send(msg);
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
    opensky_cache: &opensky_client::OpenSkyCache,
) {
    // Step A: parse ADS-B from frame[0] early (shared by clock sync AND altitude constraint)
    let adsb_parsed = group.frames[0]
        .bytes()
        .and_then(|bytes| adsb_parser.parse(bytes, group.frames[0].timestamp_ns()));

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
                                adsb.nuc, // FIX-2: Pass NUC for quality filtering/weighting.
                            );
                            used_as_beacon = true;
                        }
                    }
                }
            }
        }
    }

    // Branch: 2-sensor semi-MLAT vs 3+ sensor full MLAT
    if group.frames.len() == 2 {
        // Semi-MLAT path: requires existing Kalman track (now with altitude constraint)
        solve_semi_mlat_and_broadcast(group, clock_sync, sensor_registry, kalman_registry, adsb_parser, ws_tx)
            .await;
        return;
    } else if group.frames.len() < 2 {
        // Reject single-sensor observations (cannot constrain position)
        return;
    }

    // Full MLAT path (3+ sensors) continues below...

    // 1. Apply clock corrections (no-op until clock sync warms up)
    clock_sync.apply_corrections(&mut group.frames);

    // Diagnostic: Log corrected timestamps for debugging
    if tracing::enabled!(tracing::Level::TRACE) {
        let timestamps: Vec<u64> = group.frames.iter().map(|f| f.timestamp_ns()).collect();
        tracing::trace!(
            "MLAT timestamps after correction: ref={}, others={:?}",
            timestamps[0],
            &timestamps[1..]
        );
    }

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
    for (fi, pi) in group.frames.iter().zip(sensor_vecs.iter()) {
        for (fj, pj) in group.frames.iter().zip(sensor_vecs.iter()) {
            if fi.sensor_id >= fj.sensor_id {
                continue;
            }
            let max_tdoa_ns =
                (pi - pj).norm() * 1.05 / crate::consts::C_M_PER_NS + crate::consts::TDOA_GUARD_NS;
            let obs_ns =
                (fi.timestamp_ns() as i64 - fj.timestamp_ns() as i64).unsigned_abs() as f64;
            if obs_ns > max_tdoa_ns {
                tracing::debug!(
                    "feasibility check failed: sensors {}/{} tdoa={:.0}ns max={:.0}ns",
                    fi.sensor_id,
                    fj.sensor_id,
                    obs_ns,
                    max_tdoa_ns
                );
                return;
            }
        }
    }

    // 4. Assemble TDOAs (meters) relative to sensor[0]
    let ref_ts_ns = group.frames[0].timestamp_ns() as i64;
    let observed_tdoa_m: Vec<f64> = group.frames[1..]
        .iter()
        .map(|f| (f.timestamp_ns() as i64 - ref_ts_ns) as f64 * crate::consts::C_M_PER_NS)
        .collect();

    let icao24 = format!(
        "{:02X}{:02X}{:02X}",
        group.key.icao24[0], group.key.icao24[1], group.key.icao24[2]
    );

    // Diagnostic: Log TDOA values
    if tracing::enabled!(tracing::Level::TRACE) {
        let tdoa_ns: Vec<i64> = group.frames[1..]
            .iter()
            .map(|f| f.timestamp_ns() as i64 - ref_ts_ns)
            .collect();
        tracing::trace!(
            "ICAO={} TDOA: ns={:?}, m={:?}",
            icao24,
            tdoa_ns,
            observed_tdoa_m
        );
    }

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

    // Count sensors with valid clock calibration (Stable or Marginal status).
    let calibrated_sensor_count = sensor_ids
        .iter()
        .filter(|&&sid| clock_sync.is_calibrated(sid, now_ns))
        .count();

    // 5b. ADS-B altitude constraint (adsb_parsed already computed above).
    let adsb_alt_m = adsb_parsed.as_ref().map(|a| a.alt_m);

    // FIX-16: Altitude age degradation — compute age of the altitude report.
    let alt_age_seconds = if adsb_alt_m.is_some() {
        // Fresh altitude in this frame — update tracking and report age = 0.
        last_alt_time_ns.insert(icao24.clone(), now_ns);
        0.0
    } else {
        last_alt_time_ns
            .get(&icao24)
            .map(|&last| now_ns.saturating_sub(last) as f64 / 1e9)
            .unwrap_or(999.0) // Unknown → very stale → altitude constraint disabled
    };

    // FIX-4: Per-measurement variance from clock sync (meters²).
    // Reference sensor (index 0) uses a fixed 50 ns timing uncertainty.
    let ref_id = group.frames[0].sensor_id;
    let ref_sigma_m = 50.0 * crate::consts::C_M_PER_NS; // 50 ns * C_air ≈ 15 m
    let mut tdoa_variance_m2 = vec![ref_sigma_m * ref_sigma_m]; // sensor 0
    for f in &group.frames[1..] {
        let var_ns2 = clock_sync.get_pair_variance_ns2(f.sensor_id, ref_id);
        tdoa_variance_m2.push(var_ns2 * crate::consts::C_M_PER_NS * crate::consts::C_M_PER_NS);
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

    let mut solution = match mlat_solver::solve(&input, prior_ecef, geo_cache, &sensor_ids) {
        Some(s) => s,
        None => return,
    };

    // Record this solve (FIX-14).
    last_solve_ns.insert(icao24.clone(), now_ns);

    tracing::debug!(
        icao24,
        gdop = solution.gdop,
        vdop = solution.vdop,
        altitude_valid = solution.altitude_valid,
        dof = solution.dof,
        lat = solution.lat,
        lon = solution.lon,
        alt_m = solution.alt_m,
        "MLAT solution"
    );

    // Altitude fallback: If MLAT altitude is invalid (negative or too high),
    // use last valid altitude from Kalman > ADS-B > conservative default
    if !solution.altitude_valid {
        let fallback_altitude = kalman_registry
            .get_last_valid_altitude(&icao24)
            .or(adsb_parsed.as_ref().and_then(|a| {
                if a.alt_m >= -500.0 && a.alt_m <= 15_000.0 { Some(a.alt_m) } else { None }
            }))
            .unwrap_or(5000.0); // FL164 - conservative cruise altitude

        tracing::info!(
            icao24,
            mlat_alt_m = solution.alt_m,
            fallback_alt_m = fallback_altitude,
            gdop = solution.gdop,
            vdop = solution.vdop,
            "Using fallback altitude - MLAT altitude invalid"
        );

        // Update solution with fallback altitude
        let (x, y, z) = coords::wgs84_to_ecef(solution.lat, solution.lon, fallback_altitude);
        solution.alt_m = fallback_altitude;
        solution.ecef = nalgebra::Vector3::new(x, y, z);
    }

    // 7. Kalman update
    kalman_registry.update(&icao24, &solution, now_ns);

    let (lat, lon, alt_m) =
        kalman_registry
            .get_wgs84(&icao24)
            .unwrap_or((solution.lat, solution.lon, solution.alt_m));

    // 8. Velocity (ENU → deg/s for frontend)
    let (east, north, up) = kalman_registry
        .get_velocity_enu(&icao24, lat, lon)
        .unwrap_or((0.0, 0.0, 0.0));
    let cos_lat = lat.to_radians().cos().abs().max(1e-9);
    let vel_lat = north / crate::consts::METERS_PER_DEGREE_LAT;
    let vel_lon = east / (crate::consts::METERS_PER_DEGREE_LAT * cos_lat);
    let vel_alt = up;

    // 9. Spoof detection
    let (spoof_flag, divergence_m) =
        spoof_detector.check(&icao24, lat, lon, alt_m, solution.accuracy_m);

    // Push Kalman history now that gdop and divergence_m are both available.
    kalman_registry.push_history(&icao24, solution.gdop, divergence_m);

    // 10. Record ADS-B position for spoof detection.
    if let Some(adsb) = &adsb_parsed {
        spoof_detector.record_adsb(&adsb.icao24, adsb.lat, adsb.lon, adsb.alt_m);
    }

    // 11. Build AircraftState and broadcast
    // Enrich with callsign/squawk from OpenSky cache (populated on-demand by WS handler).
    let (callsign, squawk, origin_country) = {
        let cache = opensky_cache.read().await;
        if let Some(cached) = cache.get(&icao24.to_lowercase()) {
            (
                cached.info.callsign.clone(),
                cached.info.squawk.clone(),
                cached.info.origin_country.clone(),
            )
        } else {
            (None, None, None)
        }
    };

    let aircraft_state = AircraftState {
        icao24: icao24.clone(),
        lat,
        lon,
        alt_m,
        vel_lat,
        vel_lon,
        vel_alt,
        gdop: solution.gdop,
        vdop: solution.vdop,
        altitude_valid: solution.altitude_valid,
        confidence_ellipse_m: solution.accuracy_m,
        spoof_flag,
        divergence_m,
        anomaly_label: "unknown".to_string(),
        anomaly_confidence: 0.0,
        sensor_count: group.frames.len(),
        calibrated_sensor_count,
        timestamp_ms: now_ns / 1_000_000,
        dof: solution.dof,
        sensor_ids: sensor_ids.clone(),
        is_clock_beacon: false, // MLAT track — beacon flag carried by ADS-B broadcast
        beacon_obs_count: beacon_obs.get(&icao24).copied().unwrap_or(0),
        observation_mode: Some("full_mlat".to_string()), // Full MLAT with 3+ sensors
        sdop: None,                                      // SDOP only applies to semi-MLAT
        callsign,
        squawk,
        origin_country,
    };

    let msg = serde_json::to_string(&aircraft_state).expect("AircraftState serialization");
    let _ = ws_tx.send(msg);

    // 11. Rate-limited anomaly classification (async, non-blocking)
    // Collect data under the immutable borrow, then drop it before get_mut.
    let classify_data = kalman_registry.histories.get(&icao24).and_then(|hist| {
        let elapsed = now_ns.saturating_sub(hist.last_classify_ns);
        if elapsed > crate::consts::CLASSIFY_INTERVAL_NS && hist.states.len() >= 5 {
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
                    let msg = serde_json::to_string(&updated).expect("AircraftState serialization");
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
    opensky_cache: &opensky_client::OpenSkyCache,
) {
    let icao24 = &adsb.icao24;

    // Validate ADS-B altitude range. Barometric alt can be negative (Dead Sea ~-430m)
    // but values outside [-500, 15000] indicate a bad decode — fall back to last known.
    let adsb_alt_valid = adsb.alt_m >= -500.0 && adsb.alt_m <= 15_000.0;
    let alt_to_use = if adsb_alt_valid {
        adsb.alt_m
    } else {
        tracing::warn!(
            icao24,
            adsb_alt_m = adsb.alt_m,
            "ADS-B altitude out of range, using fallback"
        );
        kalman_registry.get_last_valid_altitude(icao24).unwrap_or(5000.0)
    };

    // Build a synthetic MlatSolution so we can reuse the Kalman registry.
    let solution = mlat_solver::MlatSolution {
        lat: adsb.lat,
        lon: adsb.lon,
        alt_m: alt_to_use,
        ecef: {
            let (x, y, z) = coords::wgs84_to_ecef(adsb.lat, adsb.lon, alt_to_use);
            nalgebra::Vector3::new(x, y, z)
        },

        gdop: 0.0,            // 0.0 signals ADS-B source (not MLAT)
        altitude_valid: adsb_alt_valid,
        vdop: 0.0,            // N/A for ADS-B
        accuracy_m: crate::consts::ADSB_HORIZONTAL_ACCURACY_M, // ADS-B typical horizontal accuracy (~500 m NUC=5)
        dof: 0,                                                // not applicable for ADS-B
        covariance: nalgebra::Matrix3::identity()
            * (crate::consts::ADSB_HORIZONTAL_ACCURACY_M
                * crate::consts::ADSB_HORIZONTAL_ACCURACY_M),
    };

    kalman_registry.update(icao24, &solution, now_ns);

    let (lat, lon, alt_m) = kalman_registry
        .get_wgs84(icao24)
        .unwrap_or((adsb.lat, adsb.lon, adsb.alt_m));

    let (east, north, up) = kalman_registry
        .get_velocity_enu(icao24, lat, lon)
        .unwrap_or((0.0, 0.0, 0.0));
    let cos_lat = lat.to_radians().cos().abs().max(1e-9);
    let vel_lat = north / crate::consts::METERS_PER_DEGREE_LAT;
    let vel_lon = east / (crate::consts::METERS_PER_DEGREE_LAT * cos_lat);
    let vel_alt = up;

    let (spoof_flag, divergence_m) = spoof_detector.check(icao24, lat, lon, alt_m, 500.0);

    // Push Kalman history now that divergence_m is available (gdop=0.0 for ADS-B).
    kalman_registry.push_history(icao24, 0.0, divergence_m);

    // Record this ADS-B position as the "claimed" position for spoof detection.
    spoof_detector.record_adsb(icao24, adsb.lat, adsb.lon, adsb.alt_m);

    // Enrich with callsign/squawk from OpenSky cache (populated on-demand by WS handler).
    let (callsign, squawk, origin_country) = {
        let cache = opensky_cache.read().await;
        if let Some(cached) = cache.get(&icao24.to_lowercase()) {
            (
                cached.info.callsign.clone(),
                cached.info.squawk.clone(),
                cached.info.origin_country.clone(),
            )
        } else {
            (None, None, None)
        }
    };

    let aircraft_state = AircraftState {
        icao24: icao24.clone(),
        lat,
        lon,
        alt_m,
        vel_lat,
        vel_lon,
        vel_alt,
        gdop: 0.0,
        vdop: 0.0,
        altitude_valid: adsb_alt_valid,
        confidence_ellipse_m: 500.0,
        spoof_flag,
        divergence_m,
        anomaly_label: "unknown".to_string(),
        anomaly_confidence: 0.0,
        sensor_count,
        calibrated_sensor_count: 0, // ADS-B — not applicable
        timestamp_ms: now_ns / 1_000_000,
        dof: 0,
        sensor_ids: vec![],
        is_clock_beacon: adsb.nuc >= 6 && beacon_obs.contains_key(icao24.as_str()),
        beacon_obs_count: beacon_obs.get(icao24.as_str()).copied().unwrap_or(0),
        observation_mode: Some("adsb".to_string()), // ADS-B source
        sdop: None,                                 // Not applicable for ADS-B
        callsign,
        squawk,
        origin_country,
    };

    tracing::debug!(icao24, lat, lon, alt_m, "ADS-B position broadcast");

    let msg = serde_json::to_string(&aircraft_state).expect("AircraftState serialization");
    let _ = ws_tx.send(msg);

    // Rate-limited anomaly classification
    let classify_data = kalman_registry.histories.get(icao24).and_then(|hist| {
        let elapsed = now_ns.saturating_sub(hist.last_classify_ns);
        if elapsed > crate::consts::CLASSIFY_INTERVAL_NS && hist.states.len() >= 5 {
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
                    let msg = serde_json::to_string(&updated).expect("AircraftState serialization");
                    let _ = ws_tx2.send(msg);
                }
                Err(e) => {
                    tracing::debug!("anomaly classify error for {icao_clone}: {e}");
                }
            }
        });
    }
}
