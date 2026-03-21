// Real-time GDOP (Geometric Dilution of Precision) heatmap computation.
//
// Computes theoretical positioning quality across geographic space independent
// of aircraft positions. Reveals sensor coverage blind zones and enables
// proactive infrastructure planning.
//
// MATHEMATICAL FOUNDATION:
//   GDOP = √(trace((H^T H)^-1))
//   where H is the (n-1)×3 Jacobian matrix of TDOA residuals.
//
// QUALITY THRESHOLDS (based on GNSS positioning theory):
//   Excellent:     GDOP < 2.0    (±5m expected error)
//   Good:          2.0 - 5.0     (±12m expected error)
//   Moderate:      5.0 - 10.0    (±25m expected error)
//   Poor:          10.0 - 20.0   (±50m expected error)
//   Unacceptable:  ≥ 20.0        (>50m expected error)
//   Degenerate:    ∞             (unsolvable geometry)
//
// Assumes 2.5m (1σ) TDOA accuracy from clock synchronization.

use nalgebra::{DMatrix, Vector3};
use serde::Serialize;
use tokio::sync::{broadcast, mpsc};

use crate::coords::{ecef_to_wgs84, wgs84_to_ecef};

// ---------------------------------------------------------------------------
// Data Structures
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum GdopQuality {
    Excellent,
    Good,
    Moderate,
    Poor,
    Unacceptable,
    Degenerate,
}

impl GdopQuality {
    /// Classify GDOP value into quality level.
    pub fn from_gdop(gdop: f64) -> Self {
        if !gdop.is_finite() {
            Self::Degenerate
        } else if gdop < 2.0 {
            Self::Excellent
        } else if gdop < 5.0 {
            Self::Good
        } else if gdop < 10.0 {
            Self::Moderate
        } else if gdop < 20.0 {
            Self::Poor
        } else {
            Self::Unacceptable
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct GdopPoint {
    pub lat: f64,
    pub lon: f64,
    pub gdop: f64,
    pub quality: GdopQuality,
    pub num_sensors: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub condition_number: Option<f64>,
}

#[derive(Debug, Clone, Serialize)]
pub struct GridBounds {
    pub north: f64,
    pub south: f64,
    pub east: f64,
    pub west: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct CoverageStats {
    pub total_points: usize,
    pub excellent: usize,
    pub good: usize,
    pub moderate: usize,
    pub poor: usize,
    pub unacceptable: usize,
    pub degenerate: usize,
    pub mean_gdop: f64,
    pub median_gdop: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct GdopGrid {
    #[serde(rename = "type")]
    pub msg_type: String,
    pub timestamp_ms: u64,
    pub bounds: GridBounds,
    pub resolution_deg: f64,
    pub altitude_m: f64,
    pub points: Vec<GdopPoint>,
    pub stats: CoverageStats,
}

// ---------------------------------------------------------------------------
// GDOP Computation Engine
// ---------------------------------------------------------------------------

pub struct GdopHeatmapEngine;

impl GdopHeatmapEngine {
    /// Compute GDOP grid in parallel using Rayon.
    ///
    /// # Parameters
    /// - `sensor_ecef`: Sensor positions in ECEF (meters)
    /// - `bounds`: Geographic bounding box (or None for auto-compute)
    /// - `resolution_deg`: Grid cell size in degrees (e.g., 0.1° ≈ 11 km)
    /// - `altitude_m`: Altitude above WGS84 ellipsoid for grid points
    ///
    /// # Returns
    /// Complete grid with GDOP values and coverage statistics.
    pub fn compute_grid(
        sensor_ecef: &[Vector3<f64>],
        bounds: Option<GridBounds>,
        resolution_deg: f64,
        altitude_m: f64,
    ) -> GdopGrid {
        use rayon::prelude::*;

        let num_sensors = sensor_ecef.len();

        // Auto-compute bounds if not provided
        let bounds = bounds.unwrap_or_else(|| Self::auto_bounds(sensor_ecef));

        // Generate grid coordinates
        let coords = Self::generate_grid_coords(&bounds, resolution_deg, altitude_m);

        tracing::info!(
            num_sensors,
            num_points = coords.len(),
            resolution_deg,
            altitude_m,
            "computing GDOP heatmap"
        );

        // Compute GDOP for each grid point in parallel
        let points: Vec<GdopPoint> = coords
            .par_iter()
            .map(|&(lat, lon, alt)| {
                let (gdop, condition_number) =
                    Self::compute_gdop_diagnostics(lat, lon, alt, sensor_ecef);
                GdopPoint {
                    lat,
                    lon,
                    gdop,
                    quality: GdopQuality::from_gdop(gdop),
                    num_sensors,
                    condition_number,
                }
            })
            .collect();

        // Compute coverage statistics
        let stats = Self::compute_stats(&points);

        let timestamp_ms = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64;

        GdopGrid {
            msg_type: "gdop_heatmap".to_string(),
            timestamp_ms,
            bounds,
            resolution_deg,
            altitude_m,
            points,
            stats,
        }
    }

    /// Compute GDOP for a single point with numerical stability checks.
    ///
    /// Returns (gdop, condition_number).
    /// GDOP = ∞ if geometry is degenerate or insufficient.
    fn compute_gdop_diagnostics(
        lat: f64,
        lon: f64,
        alt: f64,
        sensors: &[Vector3<f64>],
    ) -> (f64, Option<f64>) {
        if sensors.len() < 3 {
            return (f64::INFINITY, None);
        }

        // Convert grid point to ECEF
        let (x, y, z) = wgs84_to_ecef(lat, lon, alt);
        let solution_ecef = Vector3::new(x, y, z);

        // Compute unit vectors from point to each sensor
        let ref_dist = (&solution_ecef - &sensors[0]).norm();
        if ref_dist < 1.0 {
            return (f64::INFINITY, None);
        }
        let ref_dir = (&solution_ecef - &sensors[0]) / ref_dist;

        let n_tdoa = sensors.len() - 1;
        let mut h = DMatrix::<f64>::zeros(n_tdoa, 3);

        for (k, s_k1) in sensors[1..].iter().enumerate() {
            let d_k1 = (&solution_ecef - s_k1).norm();
            if d_k1 < 1.0 {
                return (f64::INFINITY, None);
            }
            let dir_k1 = (&solution_ecef - s_k1) / d_k1;
            for i in 0..3 {
                h[(k, i)] = dir_k1[i] - ref_dir[i];
            }
        }

        // Compute H^T H
        let ht_h = h.transpose() * &h;

        // SVD for condition number check (clone ht_h since SVD consumes it)
        let svd = ht_h.clone().svd(false, false);
        let sv = svd.singular_values;
        let min_sv = sv[sv.len().saturating_sub(1)];
        let max_sv = sv[0];
        let condition_number = max_sv / min_sv.max(f64::EPSILON);

        // Degeneracy checks
        if min_sv < 1e-6 {
            return (f64::INFINITY, Some(condition_number));
        }
        if condition_number > 1e8 {
            return (f64::INFINITY, Some(condition_number));
        }

        // Compute GDOP = √(trace((H^T H)^-1))
        match ht_h.try_inverse() {
            Some(inv) => {
                let gdop = inv.trace().sqrt();
                (gdop, Some(condition_number))
            }
            None => (f64::INFINITY, Some(condition_number)),
        }
    }

    /// Auto-compute geographic bounds from sensor positions with padding.
    ///
    /// Adds ±300 km padding around sensor convex hull.
    fn auto_bounds(sensor_ecef: &[Vector3<f64>]) -> GridBounds {
        if sensor_ecef.is_empty() {
            return GridBounds {
                north: 55.0,
                south: 45.0,
                east: 5.0,
                west: -5.0,
            };
        }

        // Convert sensors to WGS84
        let mut lats = Vec::new();
        let mut lons = Vec::new();

        for s in sensor_ecef {
            let (lat, lon, _) = ecef_to_wgs84(s.x, s.y, s.z);
            lats.push(lat);
            lons.push(lon);
        }

        let min_lat = lats.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_lat = lats.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min_lon = lons.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_lon = lons.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        // Add padding: 300 km ≈ 2.7° latitude, varies by longitude
        let padding_lat = 2.7;
        let padding_lon = 3.5; // Approximate for mid-latitudes

        GridBounds {
            north: (max_lat + padding_lat).min(85.0),
            south: (min_lat - padding_lat).max(-85.0),
            east: (max_lon + padding_lon).min(180.0),
            west: (min_lon - padding_lon).max(-180.0),
        }
    }

    /// Generate grid coordinates (lat, lon, alt) with given resolution.
    fn generate_grid_coords(
        bounds: &GridBounds,
        resolution_deg: f64,
        altitude_m: f64,
    ) -> Vec<(f64, f64, f64)> {
        let mut coords = Vec::new();

        let mut lat = bounds.south;
        while lat <= bounds.north {
            let mut lon = bounds.west;
            while lon <= bounds.east {
                coords.push((lat, lon, altitude_m));
                lon += resolution_deg;
            }
            lat += resolution_deg;
        }

        coords
    }

    /// Compute coverage statistics from grid points.
    fn compute_stats(points: &[GdopPoint]) -> CoverageStats {
        let mut excellent = 0;
        let mut good = 0;
        let mut moderate = 0;
        let mut poor = 0;
        let mut unacceptable = 0;
        let mut degenerate = 0;

        let mut finite_gdops = Vec::new();

        for p in points {
            match p.quality {
                GdopQuality::Excellent => excellent += 1,
                GdopQuality::Good => good += 1,
                GdopQuality::Moderate => moderate += 1,
                GdopQuality::Poor => poor += 1,
                GdopQuality::Unacceptable => unacceptable += 1,
                GdopQuality::Degenerate => degenerate += 1,
            }

            if p.gdop.is_finite() {
                finite_gdops.push(p.gdop);
            }
        }

        let mean_gdop = if !finite_gdops.is_empty() {
            finite_gdops.iter().sum::<f64>() / finite_gdops.len() as f64
        } else {
            f64::INFINITY
        };

        let median_gdop = if !finite_gdops.is_empty() {
            finite_gdops.sort_by(|a, b| a.partial_cmp(b).unwrap());
            finite_gdops[finite_gdops.len() / 2]
        } else {
            f64::INFINITY
        };

        CoverageStats {
            total_points: points.len(),
            excellent,
            good,
            moderate,
            poor,
            unacceptable,
            degenerate,
            mean_gdop,
            median_gdop,
        }
    }
}

// ---------------------------------------------------------------------------
// Background Task
// ---------------------------------------------------------------------------

pub enum HeatmapRequest {
    Compute {
        sensor_ecef: Vec<Vector3<f64>>,
        bounds: Option<GridBounds>,
        resolution_deg: f64,
        altitude_m: f64,
    },
}

/// Spawn background task for GDOP heatmap computation.
///
/// Computation happens in `spawn_blocking` to avoid blocking async runtime.
/// Results are automatically broadcast via WebSocket.
///
/// Returns channel sender for compute requests.
pub fn spawn_heatmap_task(
    ws_tx: broadcast::Sender<String>,
) -> mpsc::Sender<HeatmapRequest> {
    let (req_tx, mut req_rx) = mpsc::channel::<HeatmapRequest>(16);

    tokio::spawn(async move {
        while let Some(req) = req_rx.recv().await {
            match req {
                HeatmapRequest::Compute {
                    sensor_ecef,
                    bounds,
                    resolution_deg,
                    altitude_m,
                } => {
                    tracing::debug!("GDOP heatmap compute request received");

                    // CPU-intensive work in blocking task
                    let grid = tokio::task::spawn_blocking(move || {
                        GdopHeatmapEngine::compute_grid(
                            &sensor_ecef,
                            bounds,
                            resolution_deg,
                            altitude_m,
                        )
                    })
                    .await;

                    match grid {
                        Ok(grid) => {
                            tracing::info!(
                                num_points = grid.points.len(),
                                mean_gdop = grid.stats.mean_gdop,
                                excellent_pct = (grid.stats.excellent as f64
                                    / grid.stats.total_points as f64
                                    * 100.0),
                                "GDOP heatmap computed"
                            );

                            // Broadcast to WebSocket clients
                            match serde_json::to_string(&grid) {
                                Ok(json) => {
                                    tracing::debug!(
                                        msg_size = json.len(),
                                        num_points = grid.points.len(),
                                        "broadcasting GDOP heatmap to WebSocket clients"
                                    );
                                    if let Err(e) = ws_tx.send(json) {
                                        tracing::warn!("failed to broadcast GDOP heatmap: {e}");
                                    }
                                }
                                Err(e) => {
                                    tracing::error!("failed to serialize GDOP heatmap: {e}");
                                }
                            }
                        }
                        Err(e) => {
                            tracing::error!("GDOP heatmap task error: {e}");
                        }
                    }
                }
            }
        }
    });

    req_tx
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quality_classification() {
        assert_eq!(GdopQuality::from_gdop(1.5), GdopQuality::Excellent);
        assert_eq!(GdopQuality::from_gdop(3.0), GdopQuality::Good);
        assert_eq!(GdopQuality::from_gdop(7.5), GdopQuality::Moderate);
        assert_eq!(GdopQuality::from_gdop(15.0), GdopQuality::Poor);
        assert_eq!(GdopQuality::from_gdop(25.0), GdopQuality::Unacceptable);
        assert_eq!(GdopQuality::from_gdop(f64::INFINITY), GdopQuality::Degenerate);
    }

    #[test]
    fn test_gdop_computation_square() {
        use crate::coords::wgs84_to_ecef;

        // 4 sensors in square around UK (100 km baselines)
        let s1 = wgs84_to_ecef(51.5, -1.0, 100.0);  // North
        let s2 = wgs84_to_ecef(50.5, -1.0, 100.0);  // South
        let s3 = wgs84_to_ecef(51.0, -0.5, 100.0);  // East
        let s4 = wgs84_to_ecef(51.0, -1.5, 100.0);  // West

        let sensors = vec![
            Vector3::new(s1.0, s1.1, s1.2),
            Vector3::new(s2.0, s2.1, s2.2),
            Vector3::new(s3.0, s3.1, s3.2),
            Vector3::new(s4.0, s4.1, s4.2),
        ];

        // Point at center, 10 km altitude
        let (gdop, cond) = GdopHeatmapEngine::compute_gdop_diagnostics(
            51.0,
            -1.0,
            10_000.0,
            &sensors,
        );

        assert!(gdop.is_finite(), "GDOP should be finite for square geometry");
        assert!(gdop < 20.0, "GDOP should be reasonable (<20) for centered point");
        assert!(cond.is_some());
        assert!(cond.unwrap() < 1e6, "Condition number should be reasonable");
    }

    #[test]
    fn test_gdop_collinear_sensors() {
        // 3 sensors in a line (degenerate geometry)
        let sensors = vec![
            Vector3::new(0.0, 0.0, 6_371_000.0),
            Vector3::new(50_000.0, 0.0, 6_371_000.0),
            Vector3::new(100_000.0, 0.0, 6_371_000.0),
        ];

        let (gdop, _) =
            GdopHeatmapEngine::compute_gdop_diagnostics(0.0, 0.0, 10_000.0, &sensors);

        assert!(
            !gdop.is_finite(),
            "GDOP should be infinite for collinear sensors"
        );
    }

    #[test]
    fn test_gdop_insufficient_sensors() {
        let sensors = vec![
            Vector3::new(0.0, 0.0, 6_371_000.0),
            Vector3::new(50_000.0, 0.0, 6_371_000.0),
        ];

        let (gdop, _) =
            GdopHeatmapEngine::compute_gdop_diagnostics(0.0, 0.0, 10_000.0, &sensors);

        assert_eq!(gdop, f64::INFINITY, "GDOP should be infinite for <3 sensors");
    }

    #[test]
    fn test_grid_generation() {
        let bounds = GridBounds {
            north: 51.0,
            south: 50.0,
            east: 1.0,
            west: 0.0,
        };

        let coords =
            GdopHeatmapEngine::generate_grid_coords(&bounds, 0.5, 10_000.0);

        // Should have (1.0/0.5 + 1) × (1.0/0.5 + 1) = 3×3 = 9 points
        assert_eq!(coords.len(), 9, "Grid should have 9 points");
        assert_eq!(coords[0], (50.0, 0.0, 10_000.0), "First point");
        assert_eq!(coords[8], (51.0, 1.0, 10_000.0), "Last point");
    }

    #[test]
    fn test_auto_bounds() {
        // UK sensor network example
        let sensor1 = wgs84_to_ecef(51.5, -0.1, 100.0); // London
        let sensor2 = wgs84_to_ecef(53.5, -2.2, 100.0); // Manchester
        let sensor3 = wgs84_to_ecef(52.5, 1.9, 100.0);  // Norwich

        let sensors = vec![
            Vector3::new(sensor1.0, sensor1.1, sensor1.2),
            Vector3::new(sensor2.0, sensor2.1, sensor2.2),
            Vector3::new(sensor3.0, sensor3.1, sensor3.2),
        ];

        let bounds = GdopHeatmapEngine::auto_bounds(&sensors);

        // Bounds should cover UK with padding
        assert!(bounds.north > 53.5, "North bound includes Manchester + padding");
        assert!(bounds.south < 51.5, "South bound includes London - padding");
        assert!(bounds.east > 1.9, "East bound includes Norwich + padding");
        assert!(bounds.west < -2.2, "West bound includes Manchester - padding");
    }

    #[test]
    fn test_coverage_stats() {
        let points = vec![
            GdopPoint {
                lat: 50.0,
                lon: 0.0,
                gdop: 1.5,
                quality: GdopQuality::Excellent,
                num_sensors: 4,
                condition_number: Some(10.0),
            },
            GdopPoint {
                lat: 50.1,
                lon: 0.0,
                gdop: 3.0,
                quality: GdopQuality::Good,
                num_sensors: 4,
                condition_number: Some(20.0),
            },
            GdopPoint {
                lat: 50.2,
                lon: 0.0,
                gdop: f64::INFINITY,
                quality: GdopQuality::Degenerate,
                num_sensors: 4,
                condition_number: None,
            },
        ];

        let stats = GdopHeatmapEngine::compute_stats(&points);

        assert_eq!(stats.total_points, 3);
        assert_eq!(stats.excellent, 1);
        assert_eq!(stats.good, 1);
        assert_eq!(stats.degenerate, 1);
        assert_eq!(stats.mean_gdop, (1.5 + 3.0) / 2.0);
    }
}
