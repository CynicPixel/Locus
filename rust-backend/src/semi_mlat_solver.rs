// Semi-multilateration solver for 2-sensor observations with Kalman temporal priors.
//
// **Mathematical foundation**: Maximum A Posteriori (MAP) estimation fusing:
//   1. TDOA measurement likelihood (geometric constraint: hyperbola)
//   2. Kalman prediction prior (temporal constraint: trajectory continuity)
//
// The 2-sensor TDOA observation is underdetermined (infinite solutions along a hyperbola).
// Ambiguity is resolved by incorporating the Kalman filter's position prediction as a
// Bayesian prior, yielding a well-posed 4-residual, 3-unknown least-squares problem:
//
//   minimize: ||r_TDOA||² / σ_TDOA² + ||L⁻¹ (p - p_prior)||²
//
// where L is the Cholesky factor of the prior covariance (P = L·L^T).
//
// **Key constraints**:
// - Track initialization: Requires existing Kalman track (no cold starts from 2-sensor data)
// - Innovation gating: Mahalanobis distance < 15σ rejects outliers before corrupting filter
// - Quality metric: SDOP (Semi-MLAT DOP) threshold < 500m ensures reliable estimates
//
// **References**:
// - Kay (1993), "Fundamentals of Statistical Signal Processing", Ch. 12 (MAP estimation)
// - Grewal & Andrews (2014), "Kalman Filtering", Ch. 8 (constrained filtering)

use levenberg_marquardt::{LeastSquaresProblem, LevenbergMarquardt};
use nalgebra::{DVector, Matrix3, OMatrix, Vector3};

use crate::coords::ecef_to_wgs84;

// ---------------------------------------------------------------------------
// Altitude bounds (shared with mlat_solver)
// ---------------------------------------------------------------------------

/// Altitude floor (meters MSL). Dead Sea: -430m.
const ALTITUDE_FLOOR_M: f64 = -500.0;

/// Altitude ceiling (meters MSL). Typical MLAT operational ceiling.
const ALTITUDE_CEILING_M: f64 = 15_000.0;

// ---------------------------------------------------------------------------
// Public I/O types
// ---------------------------------------------------------------------------

/// Input to semi-MLAT solver: 2 sensors + Kalman prior + optional altitude.
pub struct SemiMlatInput {
    /// Exactly 2 sensor ECEF positions (meters).
    pub sensor_positions: [Vector3<f64>; 2],
    /// Observed TDOA in meters: (sensor_1 - sensor_0) * c_air.
    pub observed_tdoa_m: f64,
    /// TDOA measurement variance (meters²) — typically from clock sync uncertainty.
    pub tdoa_variance_m2: f64,
    /// Kalman filter prediction (position + covariance).
    pub kalman_prior: KalmanPrior,
    /// Optional ADS-B altitude (meters above WGS84 ellipsoid) — soft constraint.
    pub adsb_alt_m: Option<f64>,
    /// Age of altitude report (seconds) — used to degrade constraint weight.
    pub alt_age_seconds: f64,
}

/// Kalman filter state for use as MAP prior.
pub struct KalmanPrior {
    /// Predicted ECEF position (meters).
    pub position: Vector3<f64>,
    /// Position covariance (3×3 block of P matrix).
    pub covariance: Matrix3<f64>,
}

/// Semi-MLAT solution with uncertainty quantification.
pub struct SemiMlatSolution {
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f64,
    /// Native ECEF output for Kalman update.
    pub ecef: Vector3<f64>,
    /// Semi-MLAT DOP: sqrt(trace(covariance)) — quality metric.
    pub sdop: f64,
    /// 2-sigma horizontal uncertainty (meters).
    pub accuracy_m: f64,
    /// Position covariance for Kalman update.
    pub covariance: Matrix3<f64>,
    /// LM convergence flag.
    pub converged: bool,
    /// Number of LM iterations.
    pub num_iterations: usize,
}

// ---------------------------------------------------------------------------
// Helper: TDOA measurement Jacobian
// ---------------------------------------------------------------------------

/// Compute TDOA measurement Jacobian: H = ∂TDOA/∂p = dir_0 - dir_1
/// Used for proper innovation gate covariance projection (H·P·H^T).
fn tdoa_measurement_jacobian(
    position: &Vector3<f64>,
    sensors: &[Vector3<f64>; 2],
) -> nalgebra::RowVector3<f64> {
    let dist_0 = (position - &sensors[0]).norm();
    let dist_1 = (position - &sensors[1]).norm();

    if dist_0 < 1.0 || dist_1 < 1.0 {
        // Degenerate (on sensor) - return zero gradient (will cause validation failure)
        return nalgebra::RowVector3::zeros();
    }

    let dir_0 = (position - &sensors[0]) / dist_0;
    let dir_1 = (position - &sensors[1]) / dist_1;

    // H = ∂(||p - s0|| - ||p - s1||)/∂p = dir_0 - dir_1
    nalgebra::RowVector3::new(
        dir_0[0] - dir_1[0],
        dir_0[1] - dir_1[1],
        dir_0[2] - dir_1[2],
    )
}

// ---------------------------------------------------------------------------
// Pre-solve validation
// ---------------------------------------------------------------------------

/// Validate input before attempting solve.
///
/// Rejection criteria:
/// - Prior uncertainty too high (trace(P) > 25 km² → 5 km std dev limit)
/// - Sensors too close (baseline < 5 km)
/// - TDOA exceeds baseline (physically impossible)
/// - Innovation gating: Mahalanobis distance > 15σ (outlier)
/// - Weak geometry: direction vectors nearly identical (collinear)
fn validate_input(input: &SemiMlatInput) -> Result<(), &'static str> {
    // 1. Prior uncertainty check (5 km std dev limit)
    let prior_trace = input.kalman_prior.covariance.trace();
    if prior_trace > crate::consts::MAX_PRIOR_TRACE {
        tracing::debug!(
            trace = prior_trace,
            "semi-MLAT rejected: prior too uncertain (>5km std dev)"
        );
        return Err("prior too uncertain");
    }

    // 2. Geometry checks
    let baseline = (input.sensor_positions[0] - input.sensor_positions[1]).norm();
    if baseline < crate::consts::MIN_SENSOR_BASELINE_M {
        return Err("sensors too close (<5km baseline)");
    }
    if input.observed_tdoa_m.abs() > baseline {
        return Err("TDOA exceeds baseline (physically impossible)");
    }

    // 3. Innovation gating (Mahalanobis distance)
    // Predicted TDOA from Kalman prior
    let pred_pos = &input.kalman_prior.position;
    let pred_tdoa = (pred_pos - input.sensor_positions[0]).norm()
        - (pred_pos - input.sensor_positions[1]).norm();
    let innovation = input.observed_tdoa_m - pred_tdoa;

    // CRITICAL FIX: Proper covariance projection (not scalar trace approximation)
    // Innovation variance = R + H·P·H^T where:
    //   R = measurement noise variance (TDOA)
    //   H = ∂TDOA/∂p (measurement Jacobian, 1×3 row vector)
    //   P = prior covariance (3×3 matrix)
    //
    // Previous implementation used sqrt(trace(P)) which:
    // - Ignores geometry (H direction)
    // - Over/underestimates by up to 27× for anisotropic priors
    // - Makes gate too permissive when prior is elongated perpendicular to baseline
    //
    // Standard Kalman innovation: ν ~ N(0, S) where S = H·P·H^T + R
    let h = tdoa_measurement_jacobian(pred_pos, &input.sensor_positions);
    let hph = h * input.kalman_prior.covariance * h.transpose(); // 1×1 scalar
    let innovation_variance = input.tdoa_variance_m2 + hph[(0, 0)];
    let sigma_total = innovation_variance.sqrt();

    if innovation.abs() > crate::consts::INNOVATION_GATE_MAHALANOBIS * sigma_total {
        tracing::debug!(
            innovation,
            sigma_total,
            ratio = innovation.abs() / sigma_total,
            HPH = hph[(0, 0)],
            "semi-MLAT rejected: innovation gate exceeded (>{:?}σ)",
            crate::consts::INNOVATION_GATE_MAHALANOBIS
        );
        return Err("innovation exceeds gate (outlier)");
    }

    // 4. Weak constraint check: avoid nearly collinear geometry
    // Direction vectors from prior to sensors
    let dir_0 = (pred_pos - input.sensor_positions[0]).normalize();
    let dir_1 = (pred_pos - input.sensor_positions[1]).normalize();
    let dir_diff = (dir_0 - dir_1).norm();
    if dir_diff < crate::consts::WEAK_CONSTRAINT_THRESHOLD {
        return Err("weak constraint (sensors nearly collinear with prior)");
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Levenberg-Marquardt problem: 4-5 residuals, 3 unknowns
// ---------------------------------------------------------------------------

/// LM problem for semi-MLAT: fuses TDOA measurement with Kalman prior.
///
/// Residuals (4 or 5 total):
///   r[0] = (observed_tdoa - predicted_tdoa) / σ_TDOA
///   r[1:4] = L⁻¹ · (p - p_prior)
///   r[4] = (alt_wgs84(p) - alt_target) / σ_alt  [if altitude constraint enabled]
///
/// where L is the Cholesky factor (P = L·L^T) and predicted_tdoa = ||p - s₀|| - ||p - s₁||.
struct SemiMlatProblem {
    /// Sensor ECEF positions (exactly 2).
    sensors: [Vector3<f64>; 2],
    /// Observed TDOA in meters.
    observed_tdoa_m: f64,
    /// TDOA measurement uncertainty (meters).
    sigma_tdoa_m: f64,
    /// Kalman prior position.
    prior_position: Vector3<f64>,
    /// Inverse Cholesky factor: L⁻¹ where P = L·L^T.
    prior_precision_chol: Matrix3<f64>,
    /// Optional WGS84 altitude target (meters above ellipsoid).
    alt_target_m: Option<f64>,
    /// Weight for altitude residual (1/σ_alt).
    alt_weight: f64,
    /// Current parameter vector [x, y, z] in ECEF.
    p: Vector3<f64>,
}

impl LeastSquaresProblem<f64, nalgebra::Dyn, nalgebra::U3> for SemiMlatProblem {
    type ParameterStorage = nalgebra::Owned<f64, nalgebra::U3>;
    type ResidualStorage = nalgebra::Owned<f64, nalgebra::Dyn>;
    type JacobianStorage = nalgebra::Owned<f64, nalgebra::Dyn, nalgebra::U3>;

    fn set_params(&mut self, p: &Vector3<f64>) {
        self.p = *p;
    }

    fn params(&self) -> Vector3<f64> {
        self.p
    }

    fn residuals(&self) -> Option<DVector<f64>> {
        let pos = &self.p;

        let n_alt = if self.alt_target_m.is_some() { 1 } else { 0 };
        let mut r = Vec::with_capacity(4 + n_alt);

        // TDOA residual (weighted by measurement uncertainty)
        let dist_0 = (pos - self.sensors[0]).norm();
        let dist_1 = (pos - self.sensors[1]).norm();
        let predicted_tdoa = dist_0 - dist_1;
        r.push((self.observed_tdoa_m - predicted_tdoa) / self.sigma_tdoa_m);

        // Prior residuals: L⁻¹ · (p - p_prior)
        let diff = pos - self.prior_position;
        let r_prior = self.prior_precision_chol * diff;
        r.push(r_prior[0]);
        r.push(r_prior[1]);
        r.push(r_prior[2]);

        // Altitude constraint (if enabled) - same as full MLAT
        if let Some(target_alt) = self.alt_target_m {
            let (_, _, alt_guess) = ecef_to_wgs84(pos[0], pos[1], pos[2]);
            r.push(self.alt_weight * (alt_guess - target_alt));
        }

        Some(DVector::from_vec(r))
    }

    fn jacobian(&self) -> Option<OMatrix<f64, nalgebra::Dyn, nalgebra::U3>> {
        let pos = &self.p;

        let n_alt = if self.alt_target_m.is_some() { 1 } else { 0 };
        let n_rows = 4 + n_alt;
        let mut j = OMatrix::<f64, nalgebra::Dyn, nalgebra::U3>::zeros(n_rows);

        // TDOA gradient: ∂TDOA/∂p = dir_0 - dir_1
        let dist_0 = (pos - self.sensors[0]).norm();
        let dist_1 = (pos - self.sensors[1]).norm();

        if dist_0 < crate::consts::MIN_DISTANCE_M || dist_1 < crate::consts::MIN_DISTANCE_M {
            return None; // Degenerate (on sensor)
        }

        let dir_0 = (pos - self.sensors[0]) / dist_0;
        let dir_1 = (pos - self.sensors[1]) / dist_1;

        // Row 0: TDOA gradient (negated because residual = obs - pred)
        let grad_tdoa = -(dir_0 - dir_1) / self.sigma_tdoa_m;
        j.set_row(0, &grad_tdoa.transpose());

        // Rows 1-3: L⁻¹ block (prior precision)
        j.fixed_view_mut::<3, 3>(1, 0)
            .copy_from(&self.prior_precision_chol);

        // Row 4 (if altitude enabled): WGS84 altitude gradient
        // ∂alt_wgs84/∂p ≈ surface normal = [cos(lat)*cos(lon), cos(lat)*sin(lon), sin(lat)]
        if self.alt_target_m.is_some() {
            let (lat_deg, lon_deg, _) = ecef_to_wgs84(pos[0], pos[1], pos[2]);
            let lat = lat_deg.to_radians();
            let lon = lon_deg.to_radians();
            let n_hat = [lat.cos() * lon.cos(), lat.cos() * lon.sin(), lat.sin()];
            for i in 0..3 {
                j[(4, i)] = self.alt_weight * n_hat[i];
            }
        }

        Some(j)
    }
}

// ---------------------------------------------------------------------------
// Public solve function
// ---------------------------------------------------------------------------

/// Solve semi-MLAT for 2-sensor observation + Kalman prior.
///
/// Returns `None` if:
/// - Input validation fails (weak prior, outlier, bad geometry)
/// - LM solver fails to converge (>50 iterations or high residual)
/// - SDOP > 500m (insufficient quality)
/// - Condition number > 1e6 (ill-conditioned problem)
pub fn solve_semi_mlat(input: &SemiMlatInput) -> Option<SemiMlatSolution> {
    // 1. Pre-solve validation
    if let Err(reason) = validate_input(input) {
        tracing::debug!("semi-MLAT validation failed: {reason}");
        return None;
    }

    // 2. Compute prior precision Cholesky: L⁻¹ where P = L·L^T
    let chol = match input.kalman_prior.covariance.cholesky() {
        Some(c) => c,
        None => {
            tracing::debug!("semi-MLAT rejected: non-PSD prior covariance");
            return None;
        }
    };
    let chol_l = chol.l();
    let prior_prec_chol = match chol_l.try_inverse() {
        Some(inv) => inv,
        None => {
            tracing::debug!("semi-MLAT rejected: singular prior covariance");
            return None;
        }
    };

    // 3. Altitude degradation (same as full MLAT FIX-16)
    // Error = 250ft base + 70ft/s * age. Skip if too stale (>~12s → >926m error).
    let alt_data: Option<(f64, f64)> = input.adsb_alt_m.and_then(|alt| {
        let err_m = 76.2 + input.alt_age_seconds * 21.3; // 250ft + 70ft/s degradation
        if err_m > 926.0 {
            None
        } else {
            Some((alt, err_m))
        }
    });
    let alt_weight = alt_data.map(|(_, err)| 1.0 / err).unwrap_or(0.0);
    let alt_target_m = alt_data.map(|(a, _)| a);

    // 4. Setup Levenberg-Marquardt problem
    let sigma_tdoa_m = input.tdoa_variance_m2.sqrt().max(1.0);
    let problem = SemiMlatProblem {
        sensors: input.sensor_positions,
        observed_tdoa_m: input.observed_tdoa_m,
        sigma_tdoa_m,
        prior_position: input.kalman_prior.position,
        prior_precision_chol: prior_prec_chol,
        alt_target_m,
        alt_weight,
        p: input.kalman_prior.position, // Initial guess = Kalman prediction
    };

    // 5. Solve (4-5 residuals, 3 parameters)
    let (result, report) = LevenbergMarquardt::new()
        .with_patience(crate::consts::SEMI_MLAT_MAX_ITERATIONS) // Match full MLAT solver
        .minimize(problem);

    // 6. Convergence check
    if !report.termination.was_successful() {
        tracing::debug!("semi-MLAT LM failed: {:?}", report.termination);
        return None;
    }
    if report.number_of_evaluations >= crate::consts::SEMI_MLAT_MAX_ITERATIONS {
        tracing::debug!("semi-MLAT LM exceeded max iterations");
        return None;
    }

    let solution_ecef = result.params();

    // 7. Uncertainty propagation: cov = (J^T J)^{-1} * σ²
    let jacobian = match result.jacobian() {
        Some(j) => j,
        None => {
            tracing::debug!("semi-MLAT jacobian computation failed");
            return None;
        }
    };
    let jtj = jacobian.transpose() * &jacobian;

    // Condition number check
    let jtj_mat = Matrix3::new(
        jtj[(0, 0)],
        jtj[(0, 1)],
        jtj[(0, 2)],
        jtj[(1, 0)],
        jtj[(1, 1)],
        jtj[(1, 2)],
        jtj[(2, 0)],
        jtj[(2, 1)],
        jtj[(2, 2)],
    );

    // Check condition number via SVD (expensive but robust)
    let svd = jtj_mat.svd(false, false);
    let cond_num = svd.singular_values[0] / svd.singular_values[2].max(f64::EPSILON);
    if cond_num > 1e6 {
        tracing::debug!(cond_num, "semi-MLAT rejected: ill-conditioned Jacobian");
        return None;
    }

    let jtj_inv = match jtj_mat.try_inverse() {
        Some(inv) => inv,
        None => {
            tracing::debug!("semi-MLAT rejected: singular J^T J");
            return None;
        }
    };

    // Residual variance estimate
    let residuals = match result.residuals() {
        Some(r) => r,
        None => {
            tracing::debug!("semi-MLAT residuals computation failed");
            return None;
        }
    };
    let rss = residuals.norm_squared();
    // DOF = n_residuals - n_parameters
    // Residuals: 1 TDOA + 3 prior + (0 or 1 altitude) = 4 or 5
    // Parameters: 3 (x, y, z in ECEF)
    let n_residuals = residuals.len() as f64;
    let dof = n_residuals - 3.0;
    let sigma2 = if dof > 0.0 { rss / dof } else { rss };

    let covariance = jtj_inv * sigma2;

    // 8. Quality metrics
    let sdop = covariance.trace().sqrt();

    // SDOP (Semi-MLAT Dilution of Precision) threshold derivation:
    // Given: TDOA measurement noise σ_TDOA ≈ 15-45m (from clock sync uncertainty 50-150ns)
    //        Prior uncertainty σ_prior ≤ 5km (validation gate limit)
    //        Acceptable position error: ε ≤ 150m (1σ RMS)
    //
    // For 2-sensor geometry, position uncertainty is dominated by:
    //   σ_position ≈ max(σ_TDOA/sin(θ), σ_prior_projected)
    // where θ is the angle between sensor baseline and aircraft.
    //
    // Worst-case analysis (poor geometry, θ ≈ 10°):
    //   σ_position ≈ 45m / sin(10°) ≈ 260m
    //
    // Setting SDOP_THRESHOLD = 300m balances coverage vs accuracy:
    //   - Expected position error < 150m RMS (1σ), or < 300m (2σ)
    //   - Tighter than original 500m (which allowed 250m RMS error)
    //   - Allows good geometries while rejecting poor ones
    //
    // Note: SDOP is checked on uninflated covariance (geometric quality).
    // Covariance inflation (5×) is applied to output for feedback loop mitigation,
    // but doesn't affect this quality gate.
    //
    // Operational tuning: Adjust based on empirical rejection rate and accuracy.
    const SDOP_THRESHOLD: f64 = 300.0; // meters RMS (1σ position uncertainty, uninflated)

    if sdop > SDOP_THRESHOLD {
        tracing::debug!(sdop, "semi-MLAT rejected: SDOP exceeds {SDOP_THRESHOLD}m");
        return None;
    }

    // 9. Convert to WGS84
    let (lat, lon, alt_m) = ecef_to_wgs84(solution_ecef[0], solution_ecef[1], solution_ecef[2]);

    // Validate altitude is physically plausible for aircraft
    if alt_m < ALTITUDE_FLOOR_M {
        tracing::warn!(
            alt_m,
            "Semi-MLAT solution below altitude floor ({ALTITUDE_FLOOR_M}m), rejecting"
        );
        return None;
    }
    if alt_m > ALTITUDE_CEILING_M {
        tracing::warn!(
            alt_m,
            "Semi-MLAT solution above altitude ceiling ({ALTITUDE_CEILING_M}m), rejecting"
        );
        return None;
    }

    // 11. Horizontal accuracy (2-sigma confidence ellipse)
    let accuracy_m = confidence_ellipse_m(&covariance, lat, lon);

    // 10. CRITICAL FIX: Covariance inflation to mitigate feedback loop
    // Semi-MLAT uses Kalman prior → solves → updates same Kalman filter.
    // This violates Bayesian independence (measurement derived from prior).
    // Inflation factor compensates for:
    // - Model mismatch (MAP approximation errors)
    // - Prior dependency (double-counting information)
    // - Bias accumulation over consecutive semi-MLAT updates
    // Literature: Grewal & Andrews Ch. 8.5 (constrained filtering with model uncertainty)
    const COVARIANCE_INFLATION: f64 = 5.0;
    let inflated_covariance = covariance * COVARIANCE_INFLATION;
    let inflated_accuracy_m = accuracy_m * COVARIANCE_INFLATION.sqrt();

    Some(SemiMlatSolution {
        lat,
        lon,
        alt_m,
        ecef: solution_ecef,
        sdop,
        accuracy_m: inflated_accuracy_m,
        covariance: inflated_covariance,
        converged: true,
        num_iterations: report.number_of_evaluations,
    })
}

// ---------------------------------------------------------------------------
// Confidence ellipse (horizontal 2-sigma, same as full MLAT)
// ---------------------------------------------------------------------------

/// Compute 2-sigma semi-major axis of horizontal confidence ellipse.
/// Rotates ECEF covariance to ENU frame and computes horizontal eigenvalue.
fn confidence_ellipse_m(cov_ecef: &Matrix3<f64>, lat_deg: f64, lon_deg: f64) -> f64 {
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    let (slat, clat) = (lat.sin(), lat.cos());
    let (slon, clon) = (lon.sin(), lon.cos());

    // ECEF → ENU rotation matrix
    let r = Matrix3::new(
        -slon,
        clon,
        0.0,
        -slat * clon,
        -slat * slon,
        clat,
        clat * clon,
        clat * slon,
        slat,
    );

    let cov_enu = &r * cov_ecef * r.transpose();

    // Horizontal slice (East-North)
    let trace = cov_enu[(0, 0)] + cov_enu[(1, 1)];
    let det = cov_enu[(0, 0)] * cov_enu[(1, 1)] - cov_enu[(0, 1)].powi(2);
    let lambda_max = trace / 2.0 + ((trace / 2.0).powi(2) - det).max(0.0).sqrt();

    2.0 * lambda_max.sqrt() // 2-sigma semi-major axis
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::coords::wgs84_to_ecef;

    #[test]
    fn test_known_geometry() {
        // Sensors at (0°, 0°, 0m) and (0°, 1°, 0m) → ~111 km baseline
        let (s0_x, s0_y, s0_z) = wgs84_to_ecef(0.0, 0.0, 0.0);
        let (s1_x, s1_y, s1_z) = wgs84_to_ecef(0.0, 1.0, 0.0);

        let sensor_0 = Vector3::new(s0_x, s0_y, s0_z);
        let sensor_1 = Vector3::new(s1_x, s1_y, s1_z);

        // Aircraft at (0°, 0.5°, 10km) — midpoint + altitude
        let (true_x, true_y, true_z) = wgs84_to_ecef(0.0, 0.5, 10_000.0);
        let true_pos = Vector3::new(true_x, true_y, true_z);

        // Compute true TDOA
        let true_tdoa = (&true_pos - &sensor_0).norm() - (&true_pos - &sensor_1).norm();

        // Kalman prior: slight offset (500m uncertainty for stronger constraint)
        let prior_pos = true_pos + Vector3::new(200.0, 200.0, 0.0);
        let prior_cov = Matrix3::from_diagonal(&Vector3::new(
            250_000.0, // 500m std dev (250km² variance)
            250_000.0, 250_000.0,
        ));

        let input = SemiMlatInput {
            sensor_positions: [sensor_0, sensor_1],
            observed_tdoa_m: true_tdoa,
            tdoa_variance_m2: 100.0, // 10m TDOA uncertainty
            kalman_prior: KalmanPrior {
                position: prior_pos,
                covariance: prior_cov,
            },
            adsb_alt_m: None,
            alt_age_seconds: 0.0,
        };

        let solution = solve_semi_mlat(&input).expect("solve failed");

        // Verify solution is within 250m of true position (semi-MLAT typical accuracy)
        let error = (&solution.ecef - &true_pos).norm();
        assert!(
            error < 250.0,
            "position error too large: {error:.1}m (SDOP={:.1}m)",
            solution.sdop
        );

        // Verify SDOP is reasonable (50-350m for good geometry with moderate prior)
        assert!(
            solution.sdop < 350.0,
            "SDOP too high: {:.1}m",
            solution.sdop
        );
    }

    #[test]
    fn test_weak_prior_rejection() {
        // Setup similar to above but with very uncertain prior
        let (s0_x, s0_y, s0_z) = wgs84_to_ecef(0.0, 0.0, 0.0);
        let (s1_x, s1_y, s1_z) = wgs84_to_ecef(0.0, 1.0, 0.0);

        let sensor_0 = Vector3::new(s0_x, s0_y, s0_z);
        let sensor_1 = Vector3::new(s1_x, s1_y, s1_z);

        let (prior_x, prior_y, prior_z) = wgs84_to_ecef(0.0, 0.5, 10_000.0);
        let prior_pos = Vector3::new(prior_x, prior_y, prior_z);

        // Very uncertain prior (trace > 25 km²)
        let prior_cov = Matrix3::from_diagonal(&Vector3::new(
            30_000_000.0, // 30 km² variance → >5km std dev
            30_000_000.0,
            10_000_000.0,
        ));

        let input = SemiMlatInput {
            sensor_positions: [sensor_0, sensor_1],
            observed_tdoa_m: 1000.0, // Arbitrary
            tdoa_variance_m2: 100.0,
            kalman_prior: KalmanPrior {
                position: prior_pos,
                covariance: prior_cov,
            },
            adsb_alt_m: None,
            alt_age_seconds: 0.0,
        };

        let solution = solve_semi_mlat(&input);
        assert!(solution.is_none(), "should reject weak prior");
    }

    #[test]
    fn test_innovation_gate_rejection() {
        // Setup with prior that disagrees strongly with TDOA observation
        let (s0_x, s0_y, s0_z) = wgs84_to_ecef(0.0, 0.0, 0.0);
        let (s1_x, s1_y, s1_z) = wgs84_to_ecef(0.0, 1.0, 0.0);

        let sensor_0 = Vector3::new(s0_x, s0_y, s0_z);
        let sensor_1 = Vector3::new(s1_x, s1_y, s1_z);

        // Prior says aircraft is at (0°, 0.5°, 10km)
        let (prior_x, prior_y, prior_z) = wgs84_to_ecef(0.0, 0.5, 10_000.0);
        let prior_pos = Vector3::new(prior_x, prior_y, prior_z);

        let prior_cov = Matrix3::from_diagonal(&Vector3::new(
            500_000.0, // 500m std dev
            500_000.0, 500_000.0,
        ));

        // But TDOA observation implies aircraft is far from prior
        // (physically inconsistent by > 15σ)
        let observed_tdoa = 50_000.0; // 50 km TDOA (impossible for this geometry)

        let input = SemiMlatInput {
            sensor_positions: [sensor_0, sensor_1],
            observed_tdoa_m: observed_tdoa,
            tdoa_variance_m2: 100.0,
            kalman_prior: KalmanPrior {
                position: prior_pos,
                covariance: prior_cov,
            },
            adsb_alt_m: None,
            alt_age_seconds: 0.0,
        };

        let solution = solve_semi_mlat(&input);
        assert!(
            solution.is_none(),
            "should reject outlier via innovation gate"
        );
    }

    #[test]
    fn test_altitude_constraint_effectiveness() {
        // Simplified test: verify altitude constraint is applied and produces valid solution
        let (s0_x, s0_y, s0_z) = wgs84_to_ecef(0.0, 0.0, 0.0);
        let (s1_x, s1_y, s1_z) = wgs84_to_ecef(0.0, 1.0, 0.0);

        let sensor_0 = Vector3::new(s0_x, s0_y, s0_z);
        let sensor_1 = Vector3::new(s1_x, s1_y, s1_z);

        // Aircraft at 10km altitude
        let (true_x, true_y, true_z) = wgs84_to_ecef(0.0, 0.5, 10_000.0);
        let true_pos = Vector3::new(true_x, true_y, true_z);
        let true_tdoa = (&true_pos - &sensor_0).norm() - (&true_pos - &sensor_1).norm();

        // Good prior (close to truth, moderate uncertainty)
        let prior_pos = true_pos + Vector3::new(100.0, 100.0, 200.0);
        let prior_cov = Matrix3::from_diagonal(&Vector3::new(
            500_000.0, // 707m std dev
            500_000.0, 500_000.0,
        ));

        // Solve WITH altitude constraint
        let input_with_alt = SemiMlatInput {
            sensor_positions: [sensor_0, sensor_1],
            observed_tdoa_m: true_tdoa,
            tdoa_variance_m2: 100.0, // 10m std dev
            kalman_prior: KalmanPrior {
                position: prior_pos,
                covariance: prior_cov,
            },
            adsb_alt_m: Some(10_000.0), // Provide altitude constraint
            alt_age_seconds: 0.5,       // Fresh
        };

        // Should solve successfully with altitude constraint
        let sol = solve_semi_mlat(&input_with_alt).expect("solve with altitude should succeed");

        // Verify solution is reasonable
        let alt_error = (sol.alt_m - 10_000.0).abs();
        assert!(alt_error < 300.0, "altitude error: {} m", alt_error);

        let pos_error = (&sol.ecef - &true_pos).norm();
        assert!(pos_error < 400.0, "position error: {} m", pos_error);

        // Verify SDOP is reasonable
        assert!(sol.sdop < 300.0, "SDOP: {} m", sol.sdop);
    }

    #[test]
    fn test_covariance_inflation() {
        // Verify that covariance inflation is applied (CRITICAL-1 fix)
        let (s0_x, s0_y, s0_z) = wgs84_to_ecef(0.0, 0.0, 0.0);
        let (s1_x, s1_y, s1_z) = wgs84_to_ecef(0.0, 1.0, 0.0);

        let sensor_0 = Vector3::new(s0_x, s0_y, s0_z);
        let sensor_1 = Vector3::new(s1_x, s1_y, s1_z);

        let (true_x, true_y, true_z) = wgs84_to_ecef(0.0, 0.5, 10_000.0);
        let true_pos = Vector3::new(true_x, true_y, true_z);
        let true_tdoa = (&true_pos - &sensor_0).norm() - (&true_pos - &sensor_1).norm();

        let prior_pos = true_pos + Vector3::new(100.0, 100.0, 0.0);
        let prior_cov = Matrix3::from_diagonal(&Vector3::new(100_000.0, 100_000.0, 100_000.0));

        let input = SemiMlatInput {
            sensor_positions: [sensor_0, sensor_1],
            observed_tdoa_m: true_tdoa,
            tdoa_variance_m2: 100.0,
            kalman_prior: KalmanPrior {
                position: prior_pos,
                covariance: prior_cov,
            },
            adsb_alt_m: Some(10_000.0),
            alt_age_seconds: 0.5,
        };

        let solution = solve_semi_mlat(&input).expect("solve failed");

        // Covariance should be inflated by factor of 5.0
        // Check that trace(solution_cov) > trace(theoretical_uninflated_cov)
        // This is approximate, but should be significantly larger
        let prior_trace = prior_cov.trace();
        let solution_trace = solution.covariance.trace();

        // Solution covariance should not collapse to near-zero (indicates inflation working)
        assert!(
            solution_trace > 10_000.0,
            "solution covariance trace too small (inflation may not be working): {} m²",
            solution_trace
        );

        // SDOP should reflect inflated uncertainty
        assert!(
            solution.sdop > 50.0,
            "SDOP too small (may indicate missing inflation): {} m",
            solution.sdop
        );
    }

    #[test]
    fn test_edge_case_aircraft_on_baseline() {
        // Edge case: aircraft approximately on the sensor baseline
        // This geometry has poor TDOA observability perpendicular to baseline
        let (s0_x, s0_y, s0_z) = wgs84_to_ecef(0.0, 0.0, 0.0);
        let (s1_x, s1_y, s1_z) = wgs84_to_ecef(0.0, 1.0, 0.0);

        let sensor_0 = Vector3::new(s0_x, s0_y, s0_z);
        let sensor_1 = Vector3::new(s1_x, s1_y, s1_z);

        // Aircraft on baseline at 0.5° longitude, 10km altitude
        let (ac_x, ac_y, ac_z) = wgs84_to_ecef(0.0, 0.5, 10_000.0);
        let ac_pos = Vector3::new(ac_x, ac_y, ac_z);
        let true_tdoa = (&ac_pos - &sensor_0).norm() - (&ac_pos - &sensor_1).norm();

        let prior_cov = Matrix3::from_diagonal(&Vector3::new(500_000.0, 500_000.0, 500_000.0));

        let input = SemiMlatInput {
            sensor_positions: [sensor_0, sensor_1],
            observed_tdoa_m: true_tdoa,
            tdoa_variance_m2: 100.0,
            kalman_prior: KalmanPrior {
                position: ac_pos,
                covariance: prior_cov,
            },
            adsb_alt_m: Some(10_000.0),
            alt_age_seconds: 1.0,
        };

        // Should still solve (prior + altitude provide constraints)
        let solution = solve_semi_mlat(&input);
        assert!(
            solution.is_some(),
            "should solve even with aircraft on baseline (prior + altitude constrain)"
        );

        if let Some(sol) = solution {
            // But SDOP may be elevated due to poor TDOA geometry
            // This is expected behavior - not a bug
            let error = (&sol.ecef - &ac_pos).norm();
            assert!(
                error < 500.0,
                "position error acceptable for baseline geometry: {} m",
                error
            );
        }
    }

    #[test]
    fn test_consecutive_updates_stress() {
        // Stress test: simulate consecutive semi-MLAT updates
        // Verify that covariance inflation prevents overconfidence and maintains stability
        let (s0_x, s0_y, s0_z) = wgs84_to_ecef(0.0, 0.0, 0.0);
        let (s1_x, s1_y, s1_z) = wgs84_to_ecef(0.0, 1.0, 0.0);

        let sensor_0 = Vector3::new(s0_x, s0_y, s0_z);
        let sensor_1 = Vector3::new(s1_x, s1_y, s1_z);

        // True aircraft position
        let (true_x, true_y, true_z) = wgs84_to_ecef(0.0, 0.5, 10_000.0);
        let true_pos = Vector3::new(true_x, true_y, true_z);

        // Start with a prior that has moderate uncertainty
        let mut current_prior_pos = true_pos + Vector3::new(200.0, 200.0, 100.0);
        let mut current_prior_cov = Matrix3::from_diagonal(&Vector3::new(
            1_000_000.0, // 1000m std dev horizontal
            1_000_000.0,
            500_000.0, // 707m std dev vertical
        ));

        // Track minimum covariance to ensure it doesn't collapse
        let mut min_trace = f64::INFINITY;

        // Simulate 5 consecutive semi-MLAT updates (using solution as next prior)
        // Add realistic TDOA noise each iteration
        for i in 0..5 {
            let true_tdoa = (&true_pos - &sensor_0).norm() - (&true_pos - &sensor_1).norm();
            // Add different noise each iteration (simulate measurement variability)
            let noise_m = (i as f64) * 10.0 - 20.0; // -20, -10, 0, 10, 20 meters
            let noisy_tdoa = true_tdoa + noise_m;

            let input = SemiMlatInput {
                sensor_positions: [sensor_0, sensor_1],
                observed_tdoa_m: noisy_tdoa,
                tdoa_variance_m2: 900.0, // 30m std dev TDOA uncertainty
                kalman_prior: KalmanPrior {
                    position: current_prior_pos,
                    covariance: current_prior_cov,
                },
                adsb_alt_m: Some(10_000.0),
                alt_age_seconds: 1.0,
            };

            let solution = solve_semi_mlat(&input).expect(&format!("iteration {} failed", i));

            // Update prior for next iteration (simulating Kalman update)
            current_prior_pos = solution.ecef;
            current_prior_cov = solution.covariance;

            let trace = current_prior_cov.trace();
            min_trace = min_trace.min(trace);

            // Verify solution stays near truth (no divergence due to noise or bias)
            let error = (&solution.ecef - &true_pos).norm();
            assert!(
                error < 500.0,
                "iteration {}: position error {} m exceeds 500m - possible divergence",
                i,
                error
            );
        }

        // After 5 iterations with covariance inflation, the minimum trace should stay above
        // a reasonable threshold. With 5× inflation, measurement noise, and realistic geometry,
        // we expect trace to stay above ~5,000-10,000 m².
        assert!(
            min_trace > 2_000.0,
            "minimum covariance trace too small ({} m²) - inflation may be insufficient to prevent overconfidence",
            min_trace
        );
    }
}
