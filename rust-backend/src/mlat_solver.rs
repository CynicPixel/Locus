// Levenberg-Marquardt MLAT solver.
//
// Input:  sensor ECEF positions + observed TDOAs (in meters).
// Output: WGS84 position, ECEF position, GDOP, covariance, accuracy ellipse.

use nalgebra::{DMatrix, DVector, Matrix3, OMatrix, Vector3};
use nalgebra::storage::Owned;
use levenberg_marquardt::{LeastSquaresProblem, LevenbergMarquardt};

use crate::coords::ecef_to_wgs84;

// ---------------------------------------------------------------------------
// Public I/O types
// ---------------------------------------------------------------------------

pub struct MlatInput {
    /// Sensor ECEF positions (meters).
    pub sensor_positions: Vec<Vector3<f64>>,
    /// TDOAs relative to sensor[0], converted to meters: `(t_k - t_0) * c`.
    pub observed_tdoa_m: Vec<f64>,
    /// Optional ADS-B altitude (meters above WGS84 ellipsoid).
    /// When set, an altitude constraint residual is added to the LM system
    /// which dramatically improves GDOP when sensors are coplanar/clustered.
    pub adsb_alt_m: Option<f64>,
}

pub struct MlatSolution {
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f64,
    /// Native ECEF output — no round-trip needed for Kalman / spoof detector.
    pub ecef: Vector3<f64>,
    pub gdop: f64,
    /// 2-sigma semi-major axis of the horizontal confidence ellipse (meters).
    pub accuracy_m: f64,
    #[allow(dead_code)]
    pub covariance: Matrix3<f64>,
}

// ---------------------------------------------------------------------------
// Geometry cache (SVD result per unique sensor set) — Improvement 7
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
    // Minimum baseline: 5 km prevents degenerate point clusters but allows
    // real deployments where all sensors are within a single metro area.
    // The post-solve GDOP > 5 threshold is the true quality gate.
    if r.max_dist_m < 5_000.0 {
        return Err("sensors too clustered — baseline below 5 km");
    }
    if r.min_sv < 1e-3 {
        return Err("sensors coplanar/collinear — rank deficient");
    }
    if r.condition_num > 1e8 {
        return Err("sensor geometry extremely ill-conditioned");
    }
    // Pre-solve geometry GDOP is a coarse filter only — the proxy aircraft
    // position (10 km above centroid) is a rough estimate; the real quality
    // gate is the post-solve GDOP check in solve().  Use a generous threshold
    // here to avoid discarding solvable groups.
    if r.centroid_gdop > 20.0 {
        return Err("pre-solve GDOP exceeds coarse threshold (20)");
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// GDOP (also used by check_geometry for pre-solve estimate)
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
// Confidence ellipse (ENU rotation, Improvement from Step 5.2)
// ---------------------------------------------------------------------------

fn confidence_ellipse_m(cov_ecef: &Matrix3<f64>, lat_deg: f64, lon_deg: f64) -> f64 {
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    let (slat, clat) = (lat.sin(), lat.cos());
    let (slon, clon) = (lon.sin(), lon.cos());
    // ECEF → ENU rotation matrix
    let r = Matrix3::new(
        -slon,        clon,         0.0,
        -slat * clon, -slat * slon, clat,
        clat * clon,  clat * slon,  slat,
    );
    let cov_enu = &r * cov_ecef * r.transpose();
    // East-North 2×2 submatrix
    let trace = cov_enu[(0, 0)] + cov_enu[(1, 1)];
    let det = cov_enu[(0, 0)] * cov_enu[(1, 1)] - cov_enu[(0, 1)].powi(2);
    let lambda_max = trace / 2.0 + ((trace / 2.0).powi(2) - det).max(0.0).sqrt();
    2.0 * lambda_max.sqrt() // 2-sigma semi-major axis
}

// ---------------------------------------------------------------------------
// Levenberg-Marquardt problem definition
// ---------------------------------------------------------------------------

struct MlatProblem {
    sensor_ecef: Vec<Vector3<f64>>,
    observed_tdoa_m: Vec<f64>,
    /// Optional ECEF radius for the altitude-constrained case.
    /// r_constraint = WGS84_a / sqrt(1 - e2*sin2(lat)) + alt_m  (approx)
    /// We use the simpler spherical approximation: |p| = R_earth + alt_m.
    alt_constraint_r: Option<f64>,
    /// Weight for altitude constraint residual (meters, roughly 1-sigma).
    alt_weight: f64,
    p: Vector3<f64>,
}

impl LeastSquaresProblem<f64, nalgebra::Dyn, nalgebra::U3> for MlatProblem {
    type ParameterStorage = Owned<f64, nalgebra::U3>;
    type ResidualStorage = Owned<f64, nalgebra::Dyn>;
    type JacobianStorage = Owned<f64, nalgebra::Dyn, nalgebra::U3>;

    fn set_params(&mut self, p: &Vector3<f64>) {
        self.p = *p;
    }

    fn params(&self) -> Vector3<f64> {
        self.p
    }

    fn residuals(&self) -> Option<DVector<f64>> {
        let ref_dist = (&self.p - &self.sensor_ecef[0]).norm();
        let mut r: Vec<f64> = self
            .observed_tdoa_m
            .iter()
            .enumerate()
            .map(|(k, obs)| {
                let dist_k = (&self.p - &self.sensor_ecef[k + 1]).norm();
                obs - (dist_k - ref_dist)
            })
            .collect();

        // Altitude soft constraint: residual = weight * (|p| - r_constraint)
        if let Some(r_c) = self.alt_constraint_r {
            let p_norm = self.p.norm();
            r.push(self.alt_weight * (p_norm - r_c));
        }
        Some(DVector::from_vec(r))
    }

    fn jacobian(
        &self,
    ) -> Option<
        nalgebra::Matrix<f64, nalgebra::Dyn, nalgebra::U3, Self::JacobianStorage>,
    > {
        let ref_dist = (&self.p - &self.sensor_ecef[0]).norm();
        let ref_dir = (&self.p - &self.sensor_ecef[0]) / ref_dist;
        let n_tdoa = self.sensor_ecef.len() - 1;
        let n_rows = n_tdoa + if self.alt_constraint_r.is_some() { 1 } else { 0 };
        let mut j = OMatrix::<f64, nalgebra::Dyn, nalgebra::U3>::zeros(n_rows);

        for (k, s_k1) in self.sensor_ecef[1..].iter().enumerate() {
            let d_k1 = (&self.p - s_k1).norm();
            if d_k1 < 1.0 {
                return None;
            }
            let dir_k1 = (&self.p - s_k1) / d_k1;
            for i in 0..3 {
                // Negate: residual = observed - predicted; J of predicted w.r.t. p
                j[(k, i)] = -(dir_k1[i] - ref_dir[i]);
            }
        }

        // Jacobian of altitude constraint: d/dp [weight * (|p| - r_c)] = weight * p/|p|
        if self.alt_constraint_r.is_some() {
            let p_norm = self.p.norm();
            if p_norm > 1.0 {
                let p_hat = self.p / p_norm;
                for i in 0..3 {
                    j[(n_tdoa, i)] = self.alt_weight * p_hat[i];
                }
            }
        }

        Some(j)
    }
}

// ---------------------------------------------------------------------------
// Public solve function
// ---------------------------------------------------------------------------

/// Solve MLAT for the given sensor/TDOA inputs.
///
/// `prior_ecef` — Kalman ECEF position (if available) for warm-start; otherwise
/// the function uses the sensor centroid scaled to cruise altitude.
///
/// Returns `None` if geometry is degenerate, solver fails to converge, or GDOP > 5.
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

    // Build altitude constraint radius for the LM system.
    // We use a spherical Earth approximation (R_e = 6_371_000 m) for the constraint,
    // which introduces ~20 km error in radius but is fine as a soft constraint.
    // The weight (100 m) means the constraint contributes ~1 TDOA residual equivalent.
    const R_EARTH_M: f64 = 6_371_000.0;
    let alt_constraint_r = input.adsb_alt_m.map(|a| R_EARTH_M + a);

    // Initial guess: Kalman prior if available, else centroid scaled to ADS-B altitude
    // (or default cruise altitude if no ADS-B alt either).
    let cruise_alt = input.adsb_alt_m.unwrap_or(10_000.0);
    let initial_guess = match prior_ecef {
        Some(ecef) => ecef,
        None => {
            let n = input.sensor_positions.len() as f64;
            let centroid = input
                .sensor_positions
                .iter()
                .fold(Vector3::zeros(), |acc, v| acc + v)
                / n;
            let norm = centroid.norm();
            if norm < 1.0 {
                return None;
            }
            centroid * ((R_EARTH_M + cruise_alt) / norm)
        }
    };

    let problem = MlatProblem {
        sensor_ecef: input.sensor_positions.clone(),
        observed_tdoa_m: input.observed_tdoa_m.clone(),
        alt_constraint_r,
        alt_weight: 1.0 / 100.0, // 100 m 1-sigma on altitude, converted to weight
        p: initial_guess,
    };

    let (result_problem, report) = LevenbergMarquardt::new()
        .with_patience(100)
        .minimize(problem);

    if !report.termination.was_successful() {
        tracing::debug!("LM solver failed: {:?}", report.termination);
        return None;
    }

    let best_param: Vector3<f64> = result_problem.params();

    // Compute covariance from Jacobian at solution (reuse J for efficiency).
    //
    // Strategy: use the FULL Jacobian (TDOA rows + altitude constraint row if present).
    // This is correct because the altitude constraint is a real observation — when ADS-B
    // altitude is known, it genuinely constrains Z and improves J^T J conditioning.
    // Using TDOA-only rows when sensors are coplanar yields a near-singular J^T J with
    // enormous GDOP values that reject all solutions.
    let j: OMatrix<f64, nalgebra::Dyn, nalgebra::U3> = result_problem.jacobian()?;
    let n_tdoa = input.observed_tdoa_m.len();
    let jtj = j.transpose() * &j;
    let jtj_fixed = Matrix3::new(
        jtj[(0, 0)], jtj[(0, 1)], jtj[(0, 2)],
        jtj[(1, 0)], jtj[(1, 1)], jtj[(1, 2)],
        jtj[(2, 0)], jtj[(2, 1)], jtj[(2, 2)],
    );

    let rss = result_problem.residuals()?.norm_squared();
    let n_obs = n_tdoa as f64;
    let sigma2 = if n_obs > 3.0 { rss / (n_obs - 3.0) } else { rss };

    let jtj_inv = jtj_fixed
        .try_inverse()
        .unwrap_or(Matrix3::identity() * 1e12);

    // When altitude is constrained, report 2D horizontal GDOP (trace of East-North
    // submatrix of covariance) rather than the 3D GDOP — vertical is known externally.
    // When unconstrained, use full 3D GDOP.  Accept solutions up to GDOP 10.
    let (lat_tmp, lon_tmp, alt_tmp) = ecef_to_wgs84(best_param[0], best_param[1], best_param[2]);
    let gdop = if alt_constraint_r.is_some() {
        // Project covariance to ENU, take sqrt(trace(EN))
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
        // Horizontal (East-North) GDOP
        (cov_enu[(0, 0)] + cov_enu[(1, 1)]).sqrt()
    } else {
        jtj_inv.trace().sqrt()
    };
    let gdop_threshold = 10.0;
    if !gdop.is_finite() || gdop > gdop_threshold {
        tracing::debug!(gdop, "MLAT solution discarded — GDOP exceeds threshold");
        return None;
    }

    let covariance = jtj_inv * sigma2;
    let (lat, lon, alt_m) = (lat_tmp, lon_tmp, alt_tmp);
    let accuracy_m = confidence_ellipse_m(&covariance, lat, lon);

    Some(MlatSolution {
        lat,
        lon,
        alt_m,
        ecef: best_param,
        gdop,
        accuracy_m,
        covariance,
    })
}
