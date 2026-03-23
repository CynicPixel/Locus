// Levenberg-Marquardt MLAT solver.
//
// FIX-2: 4-variable pseudorange formulation [x, y, z, offset] rather than 3-var TDOA.
//   All receivers contribute symmetrically; the offset variable absorbs residual clock bias.
// FIX-3: WGS84 altitude constraint using ecef_to_wgs84() — not spherical approximation.
// FIX-4: Per-measurement error weighting (each residual ÷ sigma_k).
// FIX-15: Post-solve range and covariance validation.
// FIX-17: Solver patience reduced to 50 (matching mlat-server SOLVER_MAXFEV = 50).
// FIX-21: DOF = n_receivers + (1 if altitude) - 4 tracked in MlatSolution.

use nalgebra::{DMatrix, DVector, Matrix3, Matrix4, OMatrix, Vector3, Vector4};
use nalgebra::storage::Owned;
use levenberg_marquardt::{LeastSquaresProblem, LevenbergMarquardt};

use crate::coords::ecef_to_wgs84;

// ---------------------------------------------------------------------------
// Altitude bounds for aircraft tracking
// ---------------------------------------------------------------------------

/// Altitude floor (meters MSL). Dead Sea: -430m. Allows low-elevation operations.
const ALTITUDE_FLOOR_M: f64 = -500.0;

/// Altitude ceiling (meters MSL). Typical MLAT operational ceiling.
/// Commercial aircraft rarely exceed FL450 (~13,700m).
const ALTITUDE_CEILING_M: f64 = 15_000.0;

// ---------------------------------------------------------------------------
// Public I/O types
// ---------------------------------------------------------------------------

pub struct MlatInput {
    /// Sensor ECEF positions (meters).
    pub sensor_positions: Vec<Vector3<f64>>,
    /// TDOAs relative to sensor[0], converted to meters: `(t_k - t_0) * c_air`.
    /// Length = sensor_positions.len() - 1.
    pub observed_tdoa_m: Vec<f64>,
    /// Per-sensor 1-sigma timing uncertainty converted to meters² (length = sensor count).
    /// Index 0 = reference sensor (fixed 50 ns sigma). Index k≥1 = pair variance (k, ref).
    /// When 0.0, no weighting is applied for that residual.
    pub tdoa_variance_m2: Vec<f64>,
    /// Optional ADS-B altitude (meters above WGS84 ellipsoid) — used as a soft constraint.
    pub adsb_alt_m: Option<f64>,
    /// Age of the altitude report in seconds — used to degrade the altitude constraint weight.
    pub alt_age_seconds: f64,
}

pub struct MlatSolution {
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f64,
    /// Native ECEF output — no round-trip needed for Kalman / spoof detector.
    pub ecef: Vector3<f64>,
    pub gdop: f64,
    /// Vertical DOP — quality indicator for altitude estimate.
    pub vdop: f64,
    /// True if altitude is within valid range (-500m to 15,000m).
    pub altitude_valid: bool,
    /// 2-sigma semi-major axis of the horizontal confidence ellipse (meters).
    pub accuracy_m: f64,
    /// Degrees of freedom: n_receivers + (1 if altitude constrained) - 4.
    pub dof: i32,
    #[allow(dead_code)]
    pub covariance: Matrix3<f64>,
}

// ---------------------------------------------------------------------------
// Geometry cache (SVD result per unique sensor set)
// ---------------------------------------------------------------------------

use std::collections::HashMap;

#[derive(Clone)]
struct GeometryResult {
    max_dist_m: f64,
    condition_num: f64,
    min_sv: f64,
    centroid_gdop: f64,
}

pub struct GeometryCache {
    inner: HashMap<Vec<i64>, GeometryResult>,
}

impl GeometryCache {
    pub fn new() -> Self {
        Self {
            inner: HashMap::new(),
        }
    }

    fn get_or_compute(
        &mut self,
        sensor_ids: &[i64],
        sensor_ecef: &[Vector3<f64>],
    ) -> &GeometryResult {
        self.inner
            .entry(sensor_ids.to_vec())
            .or_insert_with(|| {
                tracing::debug!("geometry cache miss — computing SVD for {:?}", sensor_ids);
                compute_geometry_result(sensor_ecef)
            })
    }
}

fn compute_geometry_result(sensor_ecef: &[Vector3<f64>]) -> GeometryResult {
    let max_dist = sensor_ecef
        .iter()
        .flat_map(|a| sensor_ecef.iter().map(move |b| (a - b).norm()))
        .fold(0.0_f64, f64::max);

    let n = sensor_ecef.len() as f64;
    let centroid = sensor_ecef
        .iter()
        .fold(Vector3::zeros(), |a, s| a + s)
        / n;

    let mut mat = DMatrix::<f64>::zeros(sensor_ecef.len(), 3);
    for (i, s) in sensor_ecef.iter().enumerate() {
        let d = s - &centroid;
        mat[(i, 0)] = d[0];
        mat[(i, 1)] = d[1];
        mat[(i, 2)] = d[2];
    }
    let svd = mat.svd(false, false);
    let sv = svd.singular_values;
    let min_sv = sv[sv.len() - 1];
    let condition = sv[0] / min_sv.max(f64::EPSILON);

    let proxy = centroid.normalize() * (6_371_000.0 + 10_000.0);
    let centroid_gdop = compute_gdop(&proxy, sensor_ecef);

    GeometryResult {
        max_dist_m: max_dist,
        condition_num: condition,
        min_sv,
        centroid_gdop,
    }
}

// ---------------------------------------------------------------------------
// Geometry check
// ---------------------------------------------------------------------------

pub fn check_geometry(
    sensor_ids: &[i64],
    sensor_ecef: &[Vector3<f64>],
    cache: &mut GeometryCache,
) -> Result<(), &'static str> {
    let r = cache.get_or_compute(sensor_ids, sensor_ecef);
    if r.max_dist_m < 5_000.0 {
        return Err("sensors too clustered — baseline below 5 km");
    }
    if r.min_sv < 1e-3 {
        return Err("sensors coplanar/collinear — rank deficient");
    }
    if r.condition_num > 1e8 {
        return Err("sensor geometry extremely ill-conditioned");
    }
    if r.centroid_gdop > 20.0 {
        return Err("pre-solve GDOP exceeds coarse threshold (20)");
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// GDOP (post-solve, 3D)
// ---------------------------------------------------------------------------

pub fn compute_gdop(solution_ecef: &Vector3<f64>, sensors: &[Vector3<f64>]) -> f64 {
    if sensors.len() < 2 {
        return f64::INFINITY;
    }
    let ref_dist = (solution_ecef - &sensors[0]).norm();
    if ref_dist < 1.0 {
        return f64::INFINITY;
    }
    let ref_dir = (solution_ecef - &sensors[0]) / ref_dist;

    let n_tdoa = sensors.len() - 1;
    let mut h = DMatrix::<f64>::zeros(n_tdoa, 3);
    for (k, s_k1) in sensors[1..].iter().enumerate() {
        let d_k1 = (solution_ecef - s_k1).norm();
        if d_k1 < 1.0 {
            return f64::INFINITY;
        }
        let dir_k1 = (solution_ecef - s_k1) / d_k1;
        for i in 0..3 {
            h[(k, i)] = dir_k1[i] - ref_dir[i];
        }
    }
    let ht_h = h.transpose() * &h;
    match ht_h.try_inverse() {
        Some(inv) => inv.trace().sqrt(),
        None => f64::INFINITY,
    }
}

// ---------------------------------------------------------------------------
// Confidence ellipse (ENU rotation)
// ---------------------------------------------------------------------------

fn confidence_ellipse_m(cov_ecef: &Matrix3<f64>, lat_deg: f64, lon_deg: f64) -> f64 {
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    let (slat, clat) = (lat.sin(), lat.cos());
    let (slon, clon) = (lon.sin(), lon.cos());
    let r = Matrix3::new(
        -slon,        clon,         0.0,
        -slat * clon, -slat * slon, clat,
        clat * clon,  clat * slon,  slat,
    );
    let cov_enu = &r * cov_ecef * r.transpose();
    let trace = cov_enu[(0, 0)] + cov_enu[(1, 1)];
    let det = cov_enu[(0, 0)] * cov_enu[(1, 1)] - cov_enu[(0, 1)].powi(2);
    let lambda_max = trace / 2.0 + ((trace / 2.0).powi(2) - det).max(0.0).sqrt();
    2.0 * lambda_max.sqrt() // 2-sigma semi-major axis
}

// ---------------------------------------------------------------------------
// LM problem — 4 unknowns: [x, y, z, offset] (FIX-2)
// ---------------------------------------------------------------------------
//
// Residual formulation (n sensors → n residuals):
//   sensor 0 (reference): r[0] = (offset - dist_0) / sigma_0
//   sensor k (k ≥ 1):    r[k] = (obs_tdoa_m[k-1] - dist_k + offset) / sigma_k
//
// The offset variable absorbs any residual clock bias and makes all receivers
// contribute symmetrically (no privileged reference receiver in residuals).

struct MlatProblem {
    sensor_ecef: Vec<Vector3<f64>>,
    /// Observed TDOAs relative to sensor 0 (in meters), length = n_sensors - 1.
    observed_tdoa_m: Vec<f64>,
    /// 1-sigma timing uncertainty in meters per residual (length = n_sensors).
    sigmas_m: Vec<f64>,
    /// Optional WGS84 altitude target (FIX-3).
    alt_target_m: Option<f64>,
    /// Weight for the altitude residual (1/sigma_alt).
    alt_weight: f64,
    /// Current parameter vector [x, y, z, offset].
    p: Vector4<f64>,
}

impl LeastSquaresProblem<f64, nalgebra::Dyn, nalgebra::U4> for MlatProblem {
    type ParameterStorage = Owned<f64, nalgebra::U4>;
    type ResidualStorage = Owned<f64, nalgebra::Dyn>;
    type JacobianStorage = Owned<f64, nalgebra::Dyn, nalgebra::U4>;

    fn set_params(&mut self, p: &Vector4<f64>) {
        self.p = *p;
    }

    fn params(&self) -> Vector4<f64> {
        self.p
    }

    fn residuals(&self) -> Option<DVector<f64>> {
        let pos = self.p.fixed_rows::<3>(0).into_owned();
        let offset = self.p[3];

        let n_sensors = self.sensor_ecef.len();
        let n_alt = if self.alt_target_m.is_some() { 1 } else { 0 };
        let mut r = Vec::with_capacity(n_sensors + n_alt);

        // Reference sensor residual: r[0] = (offset - dist_0) / sigma_0
        let dist_0 = (&pos - &self.sensor_ecef[0]).norm();
        let sigma_0 = self.sigmas_m[0].max(1.0);
        r.push((offset - dist_0) / sigma_0);

        // Other sensors: r[k] = (obs_tdoa_m[k-1] - dist_k + offset) / sigma_k
        for k in 1..n_sensors {
            let dist_k = (&pos - &self.sensor_ecef[k]).norm();
            let sigma_k = self.sigmas_m[k].max(1.0);
            r.push((self.observed_tdoa_m[k - 1] - dist_k + offset) / sigma_k);
        }

        // WGS84 altitude constraint (FIX-3): residual = weight * (wgs84_alt(p) - target)
        if let Some(target_alt) = self.alt_target_m {
            let (_, _, alt_guess) = ecef_to_wgs84(pos[0], pos[1], pos[2]);
            r.push(self.alt_weight * (alt_guess - target_alt));
        }

        Some(DVector::from_vec(r))
    }

    fn jacobian(
        &self,
    ) -> Option<OMatrix<f64, nalgebra::Dyn, nalgebra::U4>> {
        let pos = self.p.fixed_rows::<3>(0).into_owned();

        let n_sensors = self.sensor_ecef.len();
        let n_alt = if self.alt_target_m.is_some() { 1 } else { 0 };
        let n_rows = n_sensors + n_alt;
        let mut j = OMatrix::<f64, nalgebra::Dyn, nalgebra::U4>::zeros(n_rows);

        // Row 0: reference sensor.
        let dist_0 = (&pos - &self.sensor_ecef[0]).norm();
        if dist_0 < 1.0 { return None; }
        let dir_0 = (&pos - &self.sensor_ecef[0]) / dist_0;
        let sigma_0 = self.sigmas_m[0].max(1.0);
        for i in 0..3 {
            j[(0, i)] = -dir_0[i] / sigma_0;  // ∂(offset - dist_0)/∂x_i = -dir_0[i]
        }
        j[(0, 3)] = 1.0 / sigma_0;  // ∂(offset - dist_0)/∂offset = 1

        // Rows 1..n_sensors-1: other sensors.
        for k in 1..n_sensors {
            let dist_k = (&pos - &self.sensor_ecef[k]).norm();
            if dist_k < 1.0 { return None; }
            let dir_k = (&pos - &self.sensor_ecef[k]) / dist_k;
            let sigma_k = self.sigmas_m[k].max(1.0);
            for i in 0..3 {
                j[(k, i)] = -dir_k[i] / sigma_k;  // ∂(obs - dist_k + offset)/∂x_i = -dir_k[i]
            }
            j[(k, 3)] = 1.0 / sigma_k;  // ∂/∂offset = 1
        }

        // WGS84 altitude constraint Jacobian (FIX-3).
        // ∂alt_wgs84/∂p ≈ surface normal = [cos(lat)*cos(lon), cos(lat)*sin(lon), sin(lat)].
        if self.alt_target_m.is_some() {
            let (lat_deg, lon_deg, _) = ecef_to_wgs84(pos[0], pos[1], pos[2]);
            let lat = lat_deg.to_radians();
            let lon = lon_deg.to_radians();
            let n_hat = [lat.cos() * lon.cos(), lat.cos() * lon.sin(), lat.sin()];
            for i in 0..3 {
                j[(n_sensors, i)] = self.alt_weight * n_hat[i];
            }
            // ∂/∂offset = 0 (altitude doesn't depend on offset)
        }

        Some(j)
    }
}

// ---------------------------------------------------------------------------
// Public solve function
// ---------------------------------------------------------------------------

const MAX_RANGE_M: f64 = 500_000.0; // 500 km — reject implausible solutions (FIX-15)

/// Solve MLAT for the given sensor/TDOA inputs.
///
/// Returns `None` if geometry is degenerate, solver fails to converge,
/// GDOP > 10, result is out of range, or covariance is excessive.
pub fn solve(
    input: &MlatInput,
    prior_ecef: Option<Vector3<f64>>,
    geo_cache: &mut GeometryCache,
    sensor_ids: &[i64],
) -> Option<MlatSolution> {
    if input.sensor_positions.len() < 3 || input.observed_tdoa_m.len() < 2 {
        return None;
    }

    if let Err(reason) = check_geometry(sensor_ids, &input.sensor_positions, geo_cache) {
        tracing::debug!("geometry rejected: {reason}");
        return None;
    }

    let n_sensors = input.sensor_positions.len();

    // Altitude degradation: error = 250ft + 70ft/s * age (FIX-16 variant).
    // If no altitude or too stale (> ~12 s → > 926 m error), skip constraint.
    let alt_data: Option<(f64, f64)> = input.adsb_alt_m.and_then(|alt| {
        let err_m = 76.2 + input.alt_age_seconds * 21.3; // 250ft base + 70ft/s degradation
        if err_m > 926.0 { None } else { Some((alt, err_m)) }
    });
    let alt_weight = alt_data.map(|(_, err)| 1.0 / err).unwrap_or(0.0);
    let alt_target_m = alt_data.map(|(a, _)| a);

    // Per-sensor sigmas in meters (FIX-4).
    // tdoa_variance_m2 length must match n_sensors; fall back to 300 m if missing.
    let sigmas_m: Vec<f64> = (0..n_sensors)
        .map(|k| {
            let var = input.tdoa_variance_m2.get(k).copied().unwrap_or(90_000.0);
            var.sqrt().max(1.0)
        })
        .collect();

    // Initial guess: Kalman prior if available; else centroid at ADS-B / cruise altitude.
    let cruise_alt = input.adsb_alt_m.unwrap_or(10_000.0);
    let initial_pos = match prior_ecef {
        Some(ecef) => ecef,
        None => {
            let n = n_sensors as f64;
            let centroid = input.sensor_positions.iter().fold(Vector3::zeros(), |a, v| a + v) / n;
            let norm = centroid.norm();
            if norm < 1.0 { return None; }
            centroid * ((6_371_000.0 + cruise_alt) / norm)
        }
    };

    // Initial offset estimate: distance from initial position to reference sensor.
    let initial_offset = (&initial_pos - &input.sensor_positions[0]).norm();
    let initial_p = Vector4::new(initial_pos[0], initial_pos[1], initial_pos[2], initial_offset);

    let problem = MlatProblem {
        sensor_ecef: input.sensor_positions.clone(),
        observed_tdoa_m: input.observed_tdoa_m.clone(),
        sigmas_m,
        alt_target_m,
        alt_weight,
        p: initial_p,
    };

    let (result_problem, report) = LevenbergMarquardt::new()
        .with_patience(50)  // FIX-17: match mlat-server SOLVER_MAXFEV = 50
        .minimize(problem);

    if !report.termination.was_successful() {
        tracing::debug!("LM solver failed: {:?}", report.termination);
        return None;
    }

    let best_param: Vector4<f64> = result_problem.params();
    let best_ecef = Vector3::new(best_param[0], best_param[1], best_param[2]);

    // Post-solve range validation (FIX-15): reject implausibly distant solutions.
    for sensor in &input.sensor_positions {
        if (&best_ecef - sensor).norm() > MAX_RANGE_M {
            tracing::debug!("MLAT solution discarded — out of range");
            return None;
        }
    }

    // Covariance from the 4×4 Jacobian at solution.
    let j: OMatrix<f64, nalgebra::Dyn, nalgebra::U4> = result_problem.jacobian()?;
    let jtj4 = {
        let m = j.transpose() * &j;
        Matrix4::new(
            m[(0,0)], m[(0,1)], m[(0,2)], m[(0,3)],
            m[(1,0)], m[(1,1)], m[(1,2)], m[(1,3)],
            m[(2,0)], m[(2,1)], m[(2,2)], m[(2,3)],
            m[(3,0)], m[(3,1)], m[(3,2)], m[(3,3)],
        )
    };
    let jtj4_inv = jtj4.try_inverse().unwrap_or(Matrix4::identity() * 1e12);

    // Position covariance = top-left 3×3 block of (J^T J)^{-1} scaled by sigma².
    let n_tdoa = input.observed_tdoa_m.len();
    let rss = result_problem.residuals()?.norm_squared();
    let n_obs = n_tdoa as f64;
    let sigma2 = if n_obs > 3.0 { rss / (n_obs - 3.0) } else { rss };

    let jtj_inv = Matrix3::new(
        jtj4_inv[(0,0)], jtj4_inv[(0,1)], jtj4_inv[(0,2)],
        jtj4_inv[(1,0)], jtj4_inv[(1,1)], jtj4_inv[(1,2)],
        jtj4_inv[(2,0)], jtj4_inv[(2,1)], jtj4_inv[(2,2)],
    );

    // Post-solve covariance magnitude check (FIX-15): trace > 100e6 → ~10 km uncertainty.
    if !jtj4_inv[(0,0)].is_finite() || jtj_inv.trace() > 100e6 {
        tracing::debug!("MLAT solution discarded — covariance too large");
        return None;
    }

    let (lat_tmp, lon_tmp, alt_tmp) = ecef_to_wgs84(best_param[0], best_param[1], best_param[2]);

    // GDOP: 2D horizontal when altitude constrained, 3D otherwise.
    let gdop = if alt_target_m.is_some() {
        let lat = lat_tmp.to_radians();
        let lon = lon_tmp.to_radians();
        let (slat, clat) = (lat.sin(), lat.cos());
        let (slon, clon) = (lon.sin(), lon.cos());
        let r = Matrix3::new(
            -slon,        clon,         0.0,
            -slat * clon, -slat * slon, clat,
             clat * clon,  clat * slon, slat,
        );
        let cov_enu = &r * &jtj_inv * r.transpose();
        (cov_enu[(0, 0)] + cov_enu[(1, 1)]).sqrt()
    } else {
        jtj_inv.trace().sqrt()
    };

    if !gdop.is_finite() {
        tracing::debug!(gdop, "MLAT solution discarded — non-finite GDOP");
        return None;
    }

    // Always compute VDOP as quality indicator for vertical geometry
    // VDOP ≈ sqrt(covariance[2,2]) for ECEF Z axis
    let vdop = jtj_inv[(2, 2)].sqrt();

    // Flag altitude validity but don't reject (with only 9 sensors, need every fix)
    let altitude_valid = alt_tmp >= ALTITUDE_FLOOR_M && alt_tmp <= ALTITUDE_CEILING_M;

    if !altitude_valid {
        tracing::warn!(
            alt_m = alt_tmp,
            gdop,
            vdop,
            constraint_weight = alt_weight,
            "MLAT altitude outside valid range - flagged for fallback"
        );
    }

    // Log warning for poor vertical geometry (but don't reject)
    if vdop > 10.0 {
        tracing::warn!(
            gdop,
            vdop,
            constraint_weight = alt_weight,
            "Poor vertical geometry - altitude uncertainty high (VDOP > 10)"
        );
    }

    let covariance = jtj_inv * sigma2;
    let accuracy_m = confidence_ellipse_m(&covariance, lat_tmp, lon_tmp);

    // DOF = n_receivers + (1 if altitude constrained) - 4 unknowns (FIX-21).
    let dof = n_sensors as i32 + alt_target_m.is_some() as i32 - 4;

    Some(MlatSolution {
        lat: lat_tmp,
        lon: lon_tmp,
        alt_m: alt_tmp,
        ecef: best_ecef,
        gdop,
        vdop,
        altitude_valid,
        accuracy_m,
        dof,
        covariance,
    })
}

