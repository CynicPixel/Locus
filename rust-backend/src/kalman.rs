// Extended Kalman Filter (EKF) per aircraft — 6D ECEF state [x, y, z, vx, vy, vz].
// All computations in meters / meters-per-second; WGS84 only used for output.
//
// FIX-8: Process noise sigma_a reduced from 50 m/s² to 1 m/s² (commercial aviation).
//   mlat-server uses accel_noise = 0.10 m/s² for 9-state CA model; 1 m/s² is appropriate
//   for a 6-state CV model to handle typical manoeuvres.
// FIX-8: Mahalanobis outlier gating rejects bad MLAT solves before they corrupt the filter.
//   Threshold M = 15 matches mlat-server. Three consecutive outliers trigger a filter reset.

use std::collections::HashMap;
use std::sync::OnceLock;

use nalgebra::{Matrix3, Matrix3x6, Matrix6, Vector3, Vector6};

use crate::coords::{ecef_to_wgs84, wgs84_to_ecef};
use crate::mlat_solver::MlatSolution;

// ---------------------------------------------------------------------------
// Altitude and velocity bounds for aircraft tracking
// ---------------------------------------------------------------------------

/// Vertical velocity bounds for aircraft (m/s).
/// 50 m/s = ~9,840 ft/min - extreme but physically possible (emergency descent, fighter jets).
const VERTICAL_VELOCITY_MAX_MPS: f64 = 50.0;

/// Altitude floor (meters MSL). Dead Sea: -430m. Allows low-elevation operations.
const ALTITUDE_FLOOR_M: f64 = -500.0;

/// Altitude ceiling (meters MSL). Typical MLAT operational ceiling.
const ALTITUDE_CEILING_M: f64 = 15_000.0;

// ---------------------------------------------------------------------------
// Shared constant — H matrix never changes (Improvement 9)
// ---------------------------------------------------------------------------

static H_MATRIX: OnceLock<Matrix3x6<f64>> = OnceLock::new();

fn h_matrix() -> &'static Matrix3x6<f64> {
    H_MATRIX.get_or_init(|| {
        Matrix3x6::new(
            1., 0., 0., 0., 0., 0.,
            0., 1., 0., 0., 0., 0.,
            0., 0., 1., 0., 0., 0.,
        )
    })
}

// ---------------------------------------------------------------------------
// Per-aircraft Kalman filter
// ---------------------------------------------------------------------------

/// State: `[x, y, z, vx, vy, vz]` — all ECEF meters / m·s⁻¹.
pub struct AircraftKalman {
    pub icao24: String,
    pub x: Vector6<f64>,
    pub p: Matrix6<f64>,
    pub last_update_ns: u64,
    /// Consecutive Mahalanobis outlier count; filter reset after 3 (FIX-8).
    outlier_count: u32,
}

impl AircraftKalman {
    pub fn new(icao24: String, x_ecef: f64, y_ecef: f64, z_ecef: f64) -> Self {
        let x = Vector6::new(x_ecef, y_ecef, z_ecef, 0.0, 0.0, 0.0);
        let mut p = Matrix6::zeros();
        // Initial uncertainty: 1000 m position, 200 m/s velocity (matching mlat-server UKF init).
        p[(0, 0)] = 1_000_000.0; // 1 km
        p[(1, 1)] = 1_000_000.0;
        p[(2, 2)] = 1_000_000.0;
        p[(3, 3)] = 40_000.0; // 200 m/s
        p[(4, 4)] = 40_000.0;
        p[(5, 5)] = 40_000.0;
        Self {
            icao24,
            x,
            p,
            last_update_ns: 0,
            outlier_count: 0,
        }
    }

    /// Predict step — uses exploited sparsity of F (Improvement 10).
    pub fn predict(&mut self, dt: f64) {
        // State: x += vx*dt, y += vy*dt, z += vz*dt
        self.x[0] += dt * self.x[3];
        self.x[1] += dt * self.x[4];
        self.x[2] += dt * self.x[5];

        // Covariance: P_new = F*P*F^T + Q, exploiting F = I + dt*B sparsity
        let dt2 = dt * dt;
        // FIX-8: 1 m/s² process noise — appropriate for commercial aviation.
        // mlat-server uses 0.10 m/s² for a 9-state CA model; 1 m/s² is used here
        // for the 6-state CV model to cover typical manoeuvres without over-smoothing.
        let sigma_a2: f64 = 1.0; // (1 m/s²)²
        for i in 0..3 {
            let j = i + 3;
            for k in 0..6 {
                self.p[(i, k)] += dt * self.p[(j, k)];
            }
            for k in 0..6 {
                self.p[(k, i)] += dt * self.p[(k, j)];
            }
            self.p[(i, i)] += dt2 * self.p[(j, j)];
        }
        // Add Q (sparse)
        let dt3 = dt2 * dt;
        let s3 = sigma_a2 * dt3 / 3.0;
        let s2 = sigma_a2 * dt2 / 2.0;
        let s1 = sigma_a2 * dt;
        for i in 0..3 {
            let j = i + 3;
            self.p[(i, i)] += s3;
            self.p[(j, j)] += s1;
            self.p[(i, j)] += s2;
            self.p[(j, i)] += s2;
        }

        // Clamp position to physically plausible altitudes
        let (lat, lon, alt_m) = ecef_to_wgs84(self.x[0], self.x[1], self.x[2]);
        if alt_m < ALTITUDE_FLOOR_M {
            tracing::warn!(
                icao24 = %self.icao24,
                alt_m,
                "Kalman prediction below altitude floor, clamping to {ALTITUDE_FLOOR_M}m"
            );
            // Reproject to ECEF with clamped altitude
            let (x_new, y_new, z_new) = wgs84_to_ecef(lat, lon, ALTITUDE_FLOOR_M);
            self.x[0] = x_new;
            self.x[1] = y_new;
            self.x[2] = z_new;

            // Inflate vertical uncertainty to signal low confidence
            self.p[(2, 2)] = (500.0_f64).powi(2); // 500m std dev in Z
        }
        if alt_m > ALTITUDE_CEILING_M {
            tracing::warn!(
                icao24 = %self.icao24,
                alt_m,
                "Kalman prediction above altitude ceiling, clamping to {ALTITUDE_CEILING_M}m"
            );
            let (x_new, y_new, z_new) = wgs84_to_ecef(lat, lon, ALTITUDE_CEILING_M);
            self.x[0] = x_new;
            self.x[1] = y_new;
            self.x[2] = z_new;
            self.p[(2, 2)] = (500.0_f64).powi(2);
        }
    }

    /// Update step with MLAT-derived ECEF position.
    /// Includes Mahalanobis outlier gating (FIX-8): rejects observations with
    /// innovation distance > 15 (mlat-server threshold). Three consecutive outliers
    /// trigger a filter reset so the track can re-acquire.
    pub fn update(&mut self, x_m: f64, y_m: f64, z_m: f64, accuracy_m: f64) {
        let h = h_matrix();
        // Measurement noise: MLAT accuracy² (floor 10 000 m²); Z 4× larger (approx.)
        let r_val = accuracy_m.powi(2).max(10_000.0);
        let r = Matrix3::from_diagonal(&Vector3::new(r_val, r_val, r_val * 4.0));

        let z = Vector3::new(x_m, y_m, z_m);
        let y = z - h * &self.x; // innovation
        let s = h * &self.p * h.transpose() + r;

        // Mahalanobis distance: y^T S^{-1} y (FIX-8).
        if let Some(s_inv) = s.try_inverse() {
            let maha2 = (y.transpose() * s_inv * y)[(0, 0)];
            const MAHALANOBIS_GATE: f64 = 15.0 * 15.0; // mlat-server threshold = 15
            if maha2 > MAHALANOBIS_GATE {
                self.outlier_count += 1;
                tracing::debug!(
                    icao24 = %self.icao24,
                    maha = maha2.sqrt(),
                    outlier_count = self.outlier_count,
                    "Kalman outlier rejected"
                );
                if self.outlier_count >= 3 {
                    // Three consecutive outliers: reset position from measurement.
                    self.x[0] = x_m;
                    self.x[1] = y_m;
                    self.x[2] = z_m;
                    self.x[3] = 0.0;
                    self.x[4] = 0.0;
                    self.x[5] = 0.0;
                    self.p = Matrix6::from_diagonal(&Vector6::new(
                        r_val, r_val, r_val * 4.0, 40_000.0, 40_000.0, 40_000.0,
                    ));
                    self.outlier_count = 0;
                }
                return;
            }
        }
        self.outlier_count = 0;

        let k = &self.p
            * h.transpose()
            * s.try_inverse().unwrap_or(Matrix3::zeros());
        self.x = &self.x + &k * y;
        self.p = (Matrix6::identity() - k * h) * &self.p;

        // Clamp horizontal velocity to physically plausible range
        let v_mag = (self.x[3].powi(2) + self.x[4].powi(2) + self.x[5].powi(2)).sqrt();
        const MAX_SPEED_MS: f64 = 350.0;
        if v_mag > MAX_SPEED_MS {
            let scale = MAX_SPEED_MS / v_mag;
            self.x[3] *= scale;
            self.x[4] *= scale;
            self.x[5] *= scale;
            tracing::debug!(icao24 = %self.icao24, v_mag, "velocity clamped");
        }

        // Clamp vertical velocity to realistic aircraft performance
        // Typical max climb: 25 m/s (4900 ft/min), max descent: 40 m/s (7900 ft/min)
        // Use 50 m/s as absolute limit to allow for transients
        if self.x[5].abs() > VERTICAL_VELOCITY_MAX_MPS {
            tracing::warn!(
                icao24 = %self.icao24,
                vz = self.x[5],
                "Vertical velocity exceeds physical limit, clamping to ±{VERTICAL_VELOCITY_MAX_MPS} m/s"
            );
            self.x[5] = self.x[5].signum() * VERTICAL_VELOCITY_MAX_MPS;
        }
    }

    /// WGS84 position for WebSocket output.
    pub fn position_wgs84(&self) -> (f64, f64, f64) {
        ecef_to_wgs84(self.x[0], self.x[1], self.x[2])
    }

    /// ECEF velocity (m/s) for ECEF → ENU conversion upstream.
    pub fn velocity_ms(&self) -> (f64, f64, f64) {
        (self.x[3], self.x[4], self.x[5])
    }

    /// Extract Kalman prior for semi-MLAT (position + covariance).
    pub fn get_prior(&self) -> crate::semi_mlat_solver::KalmanPrior {
        crate::semi_mlat_solver::KalmanPrior {
            position: Vector3::new(self.x[0], self.x[1], self.x[2]),
            covariance: self.p.fixed_view::<3, 3>(0, 0).into_owned(),
        }
    }
}

// ---------------------------------------------------------------------------
// Per-aircraft ML classification history
// ---------------------------------------------------------------------------

/// Stores the last 20 Kalman track states for the LSTM classifier.
/// Units: (lat°, lon°, alt_m, vel_lat °/s, vel_lon °/s, vel_alt m/s)
pub struct AircraftHistory {
    pub states: std::collections::VecDeque<(f64, f64, f64, f64, f64, f64)>,
    pub last_classify_ns: u64,
}

impl AircraftHistory {
    pub fn new() -> Self {
        Self {
            states: std::collections::VecDeque::new(),
            last_classify_ns: 0,
        }
    }

    /// Push a new state after each Kalman update (ECEF velocity converted to ENU °/s).
    pub fn push(&mut self, kalman: &AircraftKalman) {
        let (lat, lon, alt_m) = kalman.position_wgs84();
        let (vx, vy, vz) = kalman.velocity_ms();
        let (lat_r, lon_r) = (lat.to_radians(), lon.to_radians());

        // Full ECEF → ENU rotation
        let east = -lon_r.sin() * vx + lon_r.cos() * vy;
        let north = -lat_r.sin() * lon_r.cos() * vx
            - lat_r.sin() * lon_r.sin() * vy
            + lat_r.cos() * vz;
        let up = lat_r.cos() * lon_r.cos() * vx
            + lat_r.cos() * lon_r.sin() * vy
            + lat_r.sin() * vz;

        let vel_lat = north / 111_320.0;
        let cos_lat = lat_r.cos().abs().max(1e-9);
        let vel_lon = east / (111_320.0 * cos_lat);
        let vel_alt = up;

        self.states
            .push_back((lat, lon, alt_m, vel_lat, vel_lon, vel_alt));
        if self.states.len() > 20 {
            self.states.pop_front();
        }
    }
}

// ---------------------------------------------------------------------------
// Kalman registry
// ---------------------------------------------------------------------------

pub struct KalmanRegistry {
    pub filters: HashMap<String, AircraftKalman>,
    pub histories: HashMap<String, AircraftHistory>,
}

impl KalmanRegistry {
    pub fn new() -> Self {
        Self {
            filters: HashMap::new(),
            histories: HashMap::new(),
        }
    }

    /// Feed an MLAT solution into the Kalman filter for the given aircraft.
    pub fn update(&mut self, icao24: &str, mlat: &MlatSolution, now_ns: u64) {
        let (x_ecef, y_ecef, z_ecef) = (mlat.ecef[0], mlat.ecef[1], mlat.ecef[2]);

        let entry = self
            .filters
            .entry(icao24.to_string())
            .or_insert_with(|| AircraftKalman::new(icao24.to_string(), x_ecef, y_ecef, z_ecef));

        let dt = if entry.last_update_ns == 0 {
            0.1
        } else {
            (now_ns.saturating_sub(entry.last_update_ns)) as f64 / 1e9
        };
        // Clamp dt to avoid numerical blow-up during data gaps
        let dt = dt.clamp(0.0, 30.0);
        entry.predict(dt);
        entry.update(x_ecef, y_ecef, z_ecef, mlat.accuracy_m);
        entry.last_update_ns = now_ns;

        let hist = self
            .histories
            .entry(icao24.to_string())
            .or_insert_with(AircraftHistory::new);
        hist.push(entry);
    }

    /// Current ECEF position for use as LM solver prior.
    pub fn get_ecef(&self, icao24: &str) -> Option<Vector3<f64>> {
        self.filters.get(icao24).map(|k| Vector3::new(k.x[0], k.x[1], k.x[2]))
    }

    /// Smoothed WGS84 position.
    pub fn get_wgs84(&self, icao24: &str) -> Option<(f64, f64, f64)> {
        self.filters.get(icao24).map(|k| k.position_wgs84())
    }

    /// ENU velocity: (east m/s, north m/s, up m/s).
    pub fn get_velocity_enu(&self, icao24: &str, lat_deg: f64, lon_deg: f64) -> Option<(f64, f64, f64)> {
        let k = self.filters.get(icao24)?;
        let (vx, vy, vz) = k.velocity_ms();
        let lat_r = lat_deg.to_radians();
        let lon_r = lon_deg.to_radians();
        let east = -lon_r.sin() * vx + lon_r.cos() * vy;
        let north = -lat_r.sin() * lon_r.cos() * vx
            - lat_r.sin() * lon_r.sin() * vy
            + lat_r.cos() * vz;
        let up = lat_r.cos() * lon_r.cos() * vx
            + lat_r.cos() * lon_r.sin() * vy
            + lat_r.sin() * vz;
        Some((east, north, up))
    }

    /// Get Kalman prior for semi-MLAT (if track exists).
    pub fn get_prior(&self, icao24: &str) -> Option<crate::semi_mlat_solver::KalmanPrior> {
        self.filters.get(icao24).map(|k| k.get_prior())
    }

    /// Update Kalman filter from semi-MLAT solution.
    /// Identical to regular update() but logs observation mode.
    pub fn update_from_semi_mlat(
        &mut self,
        icao24: &str,
        solution: &crate::semi_mlat_solver::SemiMlatSolution,
        now_ns: u64,
    ) {
        let (x_ecef, y_ecef, z_ecef) = (solution.ecef[0], solution.ecef[1], solution.ecef[2]);

        let entry = self
            .filters
            .entry(icao24.to_string())
            .or_insert_with(|| AircraftKalman::new(icao24.to_string(), x_ecef, y_ecef, z_ecef));

        let dt = if entry.last_update_ns == 0 {
            0.1
        } else {
            (now_ns.saturating_sub(entry.last_update_ns)) as f64 / 1e9
        };
        let dt = dt.clamp(0.0, 30.0);
        entry.predict(dt);
        entry.update(x_ecef, y_ecef, z_ecef, solution.accuracy_m);
        entry.last_update_ns = now_ns;

        let hist = self
            .histories
            .entry(icao24.to_string())
            .or_insert_with(AircraftHistory::new);
        hist.push(entry);

        tracing::debug!(
            icao24 = %icao24,
            sdop = solution.sdop,
            "Kalman updated from semi-MLAT"
        );
    }
}
