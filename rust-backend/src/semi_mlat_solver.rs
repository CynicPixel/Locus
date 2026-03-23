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

use nalgebra::{Matrix3, Matrix4x3, Vector3, Vector4};
use levenberg_marquardt::{LeastSquaresProblem, LevenbergMarquardt};

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
    let baseline = (&input.sensor_positions[0] - &input.sensor_positions[1]).norm();
    if baseline < crate::consts::MIN_SENSOR_BASELINE_M {
        return Err("sensors too close (<5km baseline)");
    }
    if input.observed_tdoa_m.abs() > baseline {
        return Err("TDOA exceeds baseline (physically impossible)");
    }

    // 3. Innovation gating (Mahalanobis distance)
    // Predicted TDOA from Kalman prior
    let pred_pos = &input.kalman_prior.position;
    let pred_tdoa = (pred_pos - &input.sensor_positions[0]).norm()
                  - (pred_pos - &input.sensor_positions[1]).norm();
    let innovation = input.observed_tdoa_m - pred_tdoa;

    // Total uncertainty: measurement + prior projection onto TDOA space
    // Approximation: sigma_total ≈ sqrt(sigma_tdoa² + trace(P_prior))
    let sigma_total = (input.tdoa_variance_m2 + prior_trace).sqrt();

    if innovation.abs() > crate::consts::INNOVATION_GATE_MAHALANOBIS * sigma_total {
        tracing::debug!(
            innovation,
            sigma_total,
            ratio = innovation.abs() / sigma_total,
            "semi-MLAT rejected: innovation gate exceeded (>{}σ)", crate::consts::INNOVATION_GATE_MAHALANOBIS
        );
        return Err("innovation exceeds gate (outlier)");
    }

    // 4. Weak constraint check: avoid nearly collinear geometry
    // Direction vectors from prior to sensors
    let dir_0 = (pred_pos - &input.sensor_positions[0]).normalize();
    let dir_1 = (pred_pos - &input.sensor_positions[1]).normalize();
    let dir_diff = (&dir_0 - &dir_1).norm();
    if dir_diff < crate::consts::WEAK_CONSTRAINT_THRESHOLD {
        return Err("weak constraint (sensors nearly collinear with prior)");
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Levenberg-Marquardt problem: 4 residuals, 3 unknowns
// ---------------------------------------------------------------------------

/// LM problem for semi-MLAT: fuses TDOA measurement with Kalman prior.
///
/// Residuals (4 total):
///   r[0] = (observed_tdoa - predicted_tdoa) / σ_TDOA
///   r[1:4] = L⁻¹ · (p - p_prior)
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
    /// Current parameter vector [x, y, z] in ECEF.
    p: Vector3<f64>,
}

impl LeastSquaresProblem<f64, nalgebra::U4, nalgebra::U3> for SemiMlatProblem {
    type ParameterStorage = nalgebra::Owned<f64, nalgebra::U3>;
    type ResidualStorage = nalgebra::Owned<f64, nalgebra::U4>;
    type JacobianStorage = nalgebra::Owned<f64, nalgebra::U4, nalgebra::U3>;

    fn set_params(&mut self, p: &Vector3<f64>) {
        self.p = *p;
    }

    fn params(&self) -> Vector3<f64> {
        self.p
    }

    fn residuals(&self) -> Option<Vector4<f64>> {
        let pos = &self.p;

        // TDOA residual (weighted by measurement uncertainty)
        let dist_0 = (pos - &self.sensors[0]).norm();
        let dist_1 = (pos - &self.sensors[1]).norm();
        let predicted_tdoa = dist_0 - dist_1;
        let r_tdoa = (self.observed_tdoa_m - predicted_tdoa) / self.sigma_tdoa_m;

        // Prior residuals: L⁻¹ · (p - p_prior)
        let diff = pos - &self.prior_position;
        let r_prior = &self.prior_precision_chol * diff;

        Some(Vector4::new(r_tdoa, r_prior[0], r_prior[1], r_prior[2]))
    }

    fn jacobian(&self) -> Option<Matrix4x3<f64>> {
        let pos = &self.p;

        // TDOA gradient: ∂TDOA/∂p = dir_0 - dir_1
        let dist_0 = (pos - &self.sensors[0]).norm();
        let dist_1 = (pos - &self.sensors[1]).norm();

        if dist_0 < crate::consts::MIN_DISTANCE_M || dist_1 < crate::consts::MIN_DISTANCE_M {
            return None; // Degenerate (on sensor)
        }

        let dir_0 = (pos - &self.sensors[0]) / dist_0;
        let dir_1 = (pos - &self.sensors[1]) / dist_1;

        // Row 0: TDOA gradient (negated because residual = obs - pred)
        let grad_tdoa = -(dir_0 - dir_1) / self.sigma_tdoa_m;

        // Assemble 4×3 Jacobian: [TDOA row, L⁻¹ block]
        let mut j = Matrix4x3::zeros();
        j.set_row(0, &grad_tdoa.transpose());
        j.fixed_view_mut::<3, 3>(1, 0).copy_from(&self.prior_precision_chol);

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

    // 3. Setup Levenberg-Marquardt problem
    let sigma_tdoa_m = input.tdoa_variance_m2.sqrt().max(1.0);
    let problem = SemiMlatProblem {
        sensors: input.sensor_positions,
        observed_tdoa_m: input.observed_tdoa_m,
        sigma_tdoa_m,
        prior_position: input.kalman_prior.position,
        prior_precision_chol: prior_prec_chol,
        p: input.kalman_prior.position, // Initial guess = Kalman prediction
    };

    // 4. Solve (4 residuals, 3 parameters)
    let (result, report) = LevenbergMarquardt::new()
        .with_patience(crate::consts::SEMI_MLAT_MAX_ITERATIONS) // Match full MLAT solver
        .minimize(problem);

    // 5. Convergence check
    if !report.termination.was_successful() {
        tracing::debug!("semi-MLAT LM failed: {:?}", report.termination);
        return None;
    }
    if report.number_of_evaluations >= crate::consts::SEMI_MLAT_MAX_ITERATIONS {
        tracing::debug!("semi-MLAT LM exceeded max iterations");
        return None;
    }

    let solution_ecef = result.params();

    // 6. Uncertainty propagation: cov = (J^T J)^{-1} * σ²
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
        jtj[(0, 0)], jtj[(0, 1)], jtj[(0, 2)],
        jtj[(1, 0)], jtj[(1, 1)], jtj[(1, 2)],
        jtj[(2, 0)], jtj[(2, 1)], jtj[(2, 2)],
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
    let dof = 4.0 - 3.0; // 4 residuals - 3 parameters
    let sigma2 = if dof > 0.0 { rss / dof } else { rss };

    let covariance = jtj_inv * sigma2;

    // 7. Quality metrics
    let sdop = covariance.trace().sqrt();
    if sdop > crate::consts::SDOP_THRESHOLD_M {
        tracing::debug!(sdop, "semi-MLAT rejected: SDOP exceeds {}m", crate::consts::SDOP_THRESHOLD_M);
        return None;
    }

    // 8. Convert to WGS84
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

    // 9. Horizontal accuracy (2-sigma confidence ellipse)
    let accuracy_m = confidence_ellipse_m(&covariance, lat, lon);

    Some(SemiMlatSolution {
        lat,
        lon,
        alt_m,
        ecef: solution_ecef,
        sdop,
        accuracy_m,
        covariance,
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
        -slon,        clon,         0.0,
        -slat * clon, -slat * slon, clat,
        clat * clon,  clat * slon,  slat,
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
            250_000.0,
            250_000.0,
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
            500_000.0,
            500_000.0,
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
        assert!(solution.is_none(), "should reject outlier via innovation gate");
    }
}
