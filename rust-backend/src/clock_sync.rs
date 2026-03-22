// Self-calibrating clock synchronisation engine.
//
// **Hybrid architecture**: Maintains both pairwise regression (legacy) and global
// solver (new) for gradual migration. The global solver provides globally consistent
// offsets with transitivity guarantees, while pairwise fallback ensures robustness.
//
// FIX-6: Linear regression on (timestamp, offset) pairs tracks clock drift
//   (frequency error) so the correction tracks real receiver clock behaviour.
//   Replaces simple windowed median which cannot account for drift.
// FIX-11: Pairs expire after 120 s without update; corrections become invalid
//   after 30 s of inactivity, falling back to zero correction gracefully.
// FIX-GLOBAL: Global least-squares solver replaces pairwise regression for
//   transitivity-preserving clock synchronization with unstable sensor detection.

use std::collections::{HashMap, VecDeque};
use std::sync::Arc;

use serde::Serialize;
use tokio::sync::{mpsc, RwLock};

use crate::coords::ecef_dist;
use crate::global_clock_solver::{ClockObservation, GlobalClockSolver, SolveResult};
use crate::ingestor::RawFrame;
use crate::sensors::SensorRegistry;

const WINDOW_SIZE: usize = 50; // ~5 s at 10 beacon obs/s
/// Speed of light in air (m/ns). c_vacuum/1.0003 — matches mlat-server Cair.
const C_M_PER_NS: f64 = 0.299_702_547;

/// Pair validity window: corrections become zero if not updated for this long.
const PAIR_VALIDITY_NS: u64 = 30_000_000_000; // 30 seconds
/// Pair expiry: remove pair state if silent for this long.
const PAIR_EXPIRY_NS: u64 = 120_000_000_000; // 120 seconds
/// Minimum observations before using a pair's estimate.
const MIN_SAMPLE_COUNT: usize = 2;
/// Maximum age of a global result before drift extrapolation is disabled (Audit Bug #3).
/// Beyond this, the raw offset is used without drift correction to prevent unbounded error growth.
const MAX_DRIFT_EXTRAPOLATION_NS: u64 = 120_000_000_000; // 120 seconds

// ---------------------------------------------------------------------------
// Per-pair state — OLS regression replaces windowed median (FIX-6)
// ---------------------------------------------------------------------------

struct PairState {
    /// Observed offset errors in nanoseconds (circular window).
    samples: VecDeque<f64>,
    /// Relative timestamps in fractional seconds from `base_ts_ns` for each sample.
    timestamps: VecDeque<f64>,
    /// Timestamp of the first observation (regression origin).
    base_ts_ns: u64,
    /// Timestamp of the most recent observation (for expiry checks).
    last_update_ns: u64,
    // OLS regression coefficients (evaluated at the most recent timestamp for outlier gating).
    slope_ns_per_s: f64,   // drift rate (nanoseconds per second)
    intercept_ns: f64,     // offset at base_ts_ns
    /// MAD of regression residuals — used for outlier gating and variance export.
    cached_mad: f64,
    dirty: bool,
}

impl PairState {
    fn new(first_ts_ns: u64) -> Self {
        Self {
            samples: VecDeque::new(),
            timestamps: VecDeque::new(),
            base_ts_ns: first_ts_ns,
            last_update_ns: first_ts_ns,
            slope_ns_per_s: 0.0,
            intercept_ns: 0.0,
            cached_mad: 0.0,
            dirty: false,
        }
    }

    fn push(&mut self, error_ns: f64, at_ns: u64) {
        let t_rel = at_ns.saturating_sub(self.base_ts_ns) as f64 / 1e9;
        self.samples.push_back(error_ns);
        self.timestamps.push_back(t_rel);
        if self.samples.len() > WINDOW_SIZE {
            self.samples.pop_front();
            self.timestamps.pop_front();
        }
        self.last_update_ns = at_ns;
        self.dirty = true;
    }

    /// Ordinary least-squares regression of offset(t) = intercept + slope * t.
    fn recompute_if_dirty(&mut self) {
        if !self.dirty || self.samples.len() < 2 {
            return;
        }
        let n = self.samples.len() as f64;
        let sum_x: f64 = self.timestamps.iter().sum();
        let sum_y: f64 = self.samples.iter().sum();
        let sum_xy: f64 = self.timestamps.iter().zip(self.samples.iter()).map(|(&x, &y)| x * y).sum();
        let sum_xx: f64 = self.timestamps.iter().map(|&x| x * x).sum();

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-12 {
            // Degenerate (all samples at same time): fall back to mean.
            self.slope_ns_per_s = 0.0;
            self.intercept_ns = sum_y / n;
        } else {
            self.slope_ns_per_s = (n * sum_xy - sum_x * sum_y) / denom;
            self.intercept_ns = (sum_y - self.slope_ns_per_s * sum_x) / n;
        }

        // MAD of regression residuals (for outlier gating and variance export).
        let mut residuals: Vec<f64> = self.timestamps.iter().zip(self.samples.iter())
            .map(|(&x, &y)| (y - (self.intercept_ns + self.slope_ns_per_s * x)).abs())
            .collect();
        residuals.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
        self.cached_mad = residuals[residuals.len() / 2];
        self.dirty = false;
    }

    /// Predict offset at the given absolute timestamp (nanoseconds).
    fn estimate_at(&mut self, at_ns: u64) -> f64 {
        self.recompute_if_dirty();
        if self.samples.is_empty() {
            return 0.0;
        }
        let t_rel = at_ns.saturating_sub(self.base_ts_ns) as f64 / 1e9;
        self.intercept_ns + self.slope_ns_per_s * t_rel
    }

    /// Simple estimate used for outlier gating before the regression is stable.
    fn mean(&self) -> f64 {
        if self.samples.is_empty() { 0.0 } else { self.samples.iter().sum::<f64>() / self.samples.len() as f64 }
    }

    /// Return regression-residual MAD for outlier gating (nanoseconds).
    fn mad(&mut self) -> f64 {
        self.recompute_if_dirty();
        self.cached_mad
    }

    /// Variance of the timing error (nanoseconds²).
    /// Uses the regression-residual MAD converted to sigma: σ ≈ MAD / 0.6745 (Gaussian).
    fn variance_ns2(&mut self) -> f64 {
        self.recompute_if_dirty();
        let sigma = self.cached_mad.max(50.0) / 0.6745;
        sigma * sigma
    }

    fn len(&self) -> usize {
        self.samples.len()
    }

    /// True if the pair has enough observations and was updated recently.
    fn is_valid(&self, now_ns: u64) -> bool {
        self.len() >= MIN_SAMPLE_COUNT
            && now_ns.saturating_sub(self.last_update_ns) < PAIR_VALIDITY_NS
    }
}

// ---------------------------------------------------------------------------
// Public export type
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize)]
pub struct SensorOffsetReport {
    pub sensor_i: i64,
    pub sensor_j: i64,
    /// Latest regression-predicted offset (nanoseconds).
    pub offset_ns: f64,
    pub sample_count: usize,
    pub is_converged: bool,
}

// ---------------------------------------------------------------------------
// Clock sync engine
// ---------------------------------------------------------------------------

/// Message to trigger a global solve.
pub enum SolveRequest {
    Solve(u64, Arc<RwLock<SensorRegistry>>), // Solve at this timestamp (ns) with sensor registry
}

pub struct ClockSyncEngine {
    // Legacy pairwise regression (fallback).
    pairs: HashMap<(i64, i64), PairState>,

    // Global solver integration.
    global_solver: Arc<RwLock<GlobalClockSolver>>,
    solve_tx: mpsc::Sender<SolveRequest>,

    // Cached global solution.
    cached_global_result: Option<SolveResult>,

    // Enable global solver (false = use pairwise only).
    use_global_solver: bool,
}

impl ClockSyncEngine {
    pub fn new() -> Self {
        let global_solver = Arc::new(RwLock::new(GlobalClockSolver::new()));
        let (solve_tx, mut solve_rx) = mpsc::channel::<SolveRequest>(16);

        // Spawn background solver task.
        let solver_clone = global_solver.clone();
        tokio::spawn(async move {
            while let Some(req) = solve_rx.recv().await {
                match req {
                    SolveRequest::Solve(now_ns, sensor_registry) => {
                        let mut solver = solver_clone.write().await;

                        // Read sensor registry (acquires read lock).
                        let registry_read = sensor_registry.read().await;
                        let result = solver.solve(now_ns, Some(&*registry_read));
                        drop(registry_read); // Release registry read lock.
                        drop(solver); // Release solver write lock.

                        if result.success {
                            tracing::debug!(
                                "Global clock solve: {} sensors, {} edges, RMS={:.1}ns",
                                result.topology.num_sensors,
                                result.topology.num_edges,
                                result.topology.global_rms_ns
                            );
                        }
                    }
                }
            }
        });

        Self {
            pairs: HashMap::new(),
            global_solver,
            solve_tx,
            cached_global_result: None,
            use_global_solver: true, // Enable by default.
        }
    }

    /// Enable or disable global solver.
    #[allow(dead_code)]
    pub fn set_use_global_solver(&mut self, enabled: bool) {
        self.use_global_solver = enabled;
        tracing::info!("Global clock solver: {}", if enabled { "enabled" } else { "disabled" });
    }

    /// Trigger a global solve (non-blocking).
    pub fn trigger_global_solve(&self, now_ns: u64, sensor_registry: Arc<RwLock<SensorRegistry>>) {
        if self.use_global_solver {
            let _ = self.solve_tx.try_send(SolveRequest::Solve(now_ns, sensor_registry));
        }
    }

    /// Update cached global result from solver (called periodically from main loop).
    pub async fn update_cached_result(&mut self) {
        if !self.use_global_solver {
            return;
        }

        let solver = self.global_solver.read().await;
        if let Some(result) = solver.get_cached_result() {
            self.cached_global_result = Some(result.clone());
        }
    }

    /// Update clock offset using a beacon aircraft whose ECEF position is known.
    ///
    /// `pos_i` / `pos_j` are ECEF tuples (meters) — pass from `SensorRegistry`
    /// to avoid repeated `wgs84_to_ecef` calls.
    ///
    /// **Hybrid approach**: Pushes to global solver AND maintains pairwise fallback.
    pub fn update_from_beacon(
        &mut self,
        sensor_i: i64,
        pos_i: (f64, f64, f64),
        sensor_j: i64,
        pos_j: (f64, f64, f64),
        t_i_ns: u64,
        t_j_ns: u64,
        beacon_ecef: (f64, f64, f64),
        beacon_nuc: u8,
    ) {
        let dist_i = ecef_dist(pos_i, beacon_ecef);
        let dist_j = ecef_dist(pos_j, beacon_ecef);
        let expected_tdoa_ns = (dist_i - dist_j) / C_M_PER_NS;
        let observed_tdoa_ns = t_i_ns as f64 - t_j_ns as f64;
        let error_ns = observed_tdoa_ns - expected_tdoa_ns;

        // Quality gate - only accept beacons NUC ≥ 5.
        // NUC 5 = HPL <370m, NUC 6 = HPL <185m, NUC 7 = HPL <75m, NUC 8 = HPL <25m, NUC 9 = HPL <7.5m
        // NUC 5 included but down-weighted; keeps observation rate high enough to fill buffer in regions
        // with few high-accuracy beacons.
        if beacon_nuc < 5 {
            tracing::trace!(
                "beacon rejected: NUC={} (threshold=5)",
                beacon_nuc
            );
            return;
        }

        // Weight observations by NUC and baseline distance.
        let baseline_m = ecef_dist(pos_i, pos_j);
        let nuc_weight = match beacon_nuc {
            5 => 0.5,
            6 => 1.0,
            7 => 2.0,
            8..=9 => 4.0,
            _ => 0.5, // Fallback (should not reach here due to NUC < 6 gate).
        };
        let baseline_weight = (baseline_m / 100_000.0).min(2.0); // Favor long baselines.
        let weight = nuc_weight * baseline_weight;

        // Push to global solver (synchronous, O(1) - just a VecDeque push).
        if self.use_global_solver {
            let obs = ClockObservation {
                sensor_i,
                sensor_j,
                residual_ns: error_ns,
                weight,
                timestamp_ns: t_i_ns,
                beacon_ecef,
            };

            // Non-blocking: try to get write lock, but don't block if solver is busy.
            if let Ok(mut solver) = self.global_solver.try_write() {
                solver.add_observation(obs);
            }
            // If lock is held (solver is running), skip this observation.
            // This is acceptable since we have many observations per second.
        }

        // Maintain pairwise regression (fallback).
        let pair = self.pairs
            .entry((sensor_i, sensor_j))
            .or_insert_with(|| PairState::new(t_i_ns));

        // Outlier gate: reject if > 3× MAD from regression estimate.
        // During warm-up (< 5 samples) use mean instead.
        if pair.len() >= 5 {
            let estimate = pair.estimate_at(t_i_ns);
            let mad = pair.mad().max(50.0); // floor at 50 ns
            if (error_ns - estimate).abs() > 3.0 * mad {
                tracing::debug!(
                    "clock obs rejected: error={:.1}ns estimate={:.1}ns mad={:.1}ns",
                    error_ns, estimate, mad
                );
                return;
            }
        } else if pair.len() >= 2 {
            let mean = pair.mean();
            // FIX-3: Tightened from 3000ns (3µs) to 1000ns (1µs) to prevent bad warm-up data.
            if (error_ns - mean).abs() > 1000.0 {
                return;
            }
        }

        pair.push(error_ns, t_i_ns);

        if pair.len() % 10 == 0 {
            let est = pair.estimate_at(t_i_ns);
            tracing::debug!(
                "clock[{}→{}] offset={:.1}ns drift={:.3}ns/s n={}",
                sensor_i, sensor_j, est, pair.slope_ns_per_s, pair.len()
            );
        }
    }

    /// Return the regression-predicted offset for a sensor pair at `at_ns` (nanoseconds).
    /// Returns 0.0 if no valid observations.
    ///
    /// **Hybrid**: Uses global solver if available, falls back to pairwise.
    pub fn get_offset_at(&mut self, sensor_i: i64, sensor_j: i64, at_ns: u64) -> f64 {
        // Try global solver first.
        if self.use_global_solver {
            if let Some(ref result) = self.cached_global_result {
                if result.success {
                    // Get global offsets for both sensors.
                    let offset_i = result.offsets.get(&sensor_i).copied().unwrap_or(0.0);
                    let offset_j = result.offsets.get(&sensor_j).copied().unwrap_or(0.0);

                    // Apply drift extrapolation.
                    let drift_i = result.drift_rates.get(&sensor_i).copied().unwrap_or(0.0);
                    let drift_j = result.drift_rates.get(&sensor_j).copied().unwrap_or(0.0);

                    // Audit Bug #3: cap drift extrapolation to 120s; beyond that use static offset.
                    let age_ns = at_ns.saturating_sub(result.solve_timestamp_ns);
                    let (extrapolated_i, extrapolated_j) = if age_ns > MAX_DRIFT_EXTRAPOLATION_NS {
                        tracing::debug!(
                            "clock result stale ({:.0}s) — drift extrapolation disabled for {}/{}",
                            age_ns as f64 / 1e9, sensor_i, sensor_j
                        );
                        (offset_i, offset_j)
                    } else {
                        let dt_s = age_ns as f64 / 1e9;
                        (offset_i + drift_i * dt_s, offset_j + drift_j * dt_s)
                    };

                    // Return i-j offset (convention: sensor_i - sensor_j).
                    return extrapolated_i - extrapolated_j;
                }
            }
        }

        // Fallback to pairwise regression.
        let pair = self.pairs
            .entry((sensor_i, sensor_j))
            .or_insert_with(|| PairState::new(at_ns));
        if pair.is_valid(at_ns) { pair.estimate_at(at_ns) } else { 0.0 }
    }

    /// Variance of TDOA error for the pair (nanoseconds²).
    /// Used by mlat_solver for per-measurement weighting (FIX-4).
    /// Returns a conservative default if pair not yet converged.
    ///
    /// **Hybrid**: Uses global uncertainties if available, falls back to pairwise.
    pub fn get_pair_variance_ns2(&mut self, sensor_i: i64, sensor_j: i64) -> f64 {
        // Try global solver first: σ²_pair = σ²_i + σ²_j.
        if self.use_global_solver {
            if let Some(ref result) = self.cached_global_result {
                if result.success {
                    let sigma_i = result.uncertainties.get(&sensor_i).copied().unwrap_or(300.0);
                    let sigma_j = result.uncertainties.get(&sensor_j).copied().unwrap_or(300.0);
                    return sigma_i * sigma_i + sigma_j * sigma_j;
                }
            }
        }

        // Fallback to pairwise regression.
        match self.pairs.get_mut(&(sensor_i, sensor_j)) {
            Some(p) if p.len() >= MIN_SAMPLE_COUNT => p.variance_ns2(),
            _ => 300.0 * 300.0, // 300 ns default sigma → 90_000 ns²
        }
    }

    #[allow(dead_code)]
    pub fn is_converged(&self, sensor_i: i64, sensor_j: i64) -> bool {
        self.pairs
            .get(&(sensor_i, sensor_j))
            .map(|p| p.len() >= WINDOW_SIZE / 2)
            .unwrap_or(false)
    }

    /// Remove pairs that have not been updated for more than PAIR_EXPIRY_NS (FIX-11).
    pub fn evict_stale_pairs(&mut self, now_ns: u64) {
        self.pairs.retain(|_, state| {
            now_ns.saturating_sub(state.last_update_ns) < PAIR_EXPIRY_NS
        });
    }

    /// Apply per-sensor clock corrections to a frame slice **in-place**.
    /// Uses regression-predicted offset at the reference frame's timestamp (FIX-6).
    /// Frames with invalid/expired pairs receive zero correction (safe fallback).
    pub fn apply_corrections(&mut self, frames: &mut [RawFrame]) {
        if frames.len() < 2 {
            return;
        }
        let ref_id = frames[0].sensor_id;
        let ref_ts_ns = frames[0].timestamp_ns();
        // Build corrections first to avoid borrow conflict.
        let corrections: Vec<f64> = frames[1..]
            .iter()
            .map(|f| self.get_offset_at(f.sensor_id, ref_id, ref_ts_ns))
            .collect();

        // Diagnostic: Log corrections if any are non-zero (trace level only)
        if tracing::enabled!(tracing::Level::TRACE) {
            let has_corrections = corrections.iter().any(|&c| c.abs() > 1.0);
            if has_corrections {
                let sensor_corrections: Vec<(i64, f64)> = frames[1..]
                    .iter()
                    .zip(corrections.iter())
                    .map(|(f, &c)| (f.sensor_id, c))
                    .collect();
                tracing::trace!(
                    "Clock corrections applied (ref={}): {:?}",
                    ref_id,
                    sensor_corrections
                );
            }
        }

        for (frame, correction_ns) in frames[1..].iter_mut().zip(corrections) {
            let raw_ns = frame.timestamp_ns() as i64;
            let corrected_ns = (raw_ns - correction_ns.round() as i64).max(0) as u64;
            frame.timestamp_seconds = corrected_ns / 1_000_000_000;
            frame.timestamp_nanoseconds = corrected_ns % 1_000_000_000;
            frame.timestamp_ns_cache = corrected_ns;
        }
    }

    /// Snapshot of all pair offsets for WebSocket broadcast (legacy).
    pub fn export_offsets(&mut self) -> Vec<SensorOffsetReport> {
        self.pairs
            .iter_mut()
            .map(|(&(si, sj), p)| {
                let current_offset = p.estimate_at(p.last_update_ns);
                SensorOffsetReport {
                    sensor_i: si,
                    sensor_j: sj,
                    offset_ns: current_offset,
                    sample_count: p.len(),
                    is_converged: p.len() >= 5,
                }
            })
            .collect()
    }

    /// Export global solver result for WebSocket broadcast (new).
    pub fn export_global_result(&self) -> Option<SolveResult> {
        self.cached_global_result.clone()
    }

    /// Returns true if the sensor has a valid, non-stale global clock calibration.
    ///
    /// Stable and Marginal count as calibrated.
    /// Unstable, Disconnected, and Unknown do not.
    /// Also returns false if the global result is older than MAX_DRIFT_EXTRAPOLATION_NS.
    pub fn is_calibrated(&self, sensor_id: i64, now_ns: u64) -> bool {
        use crate::global_clock_solver::SensorStatus;
        let result = match &self.cached_global_result {
            Some(r) if r.success => r,
            _ => return false,
        };
        if now_ns.saturating_sub(result.solve_timestamp_ns) > MAX_DRIFT_EXTRAPOLATION_NS {
            return false;
        }
        matches!(
            result.statuses.get(&sensor_id),
            Some(SensorStatus::Stable) | Some(SensorStatus::Marginal)
        )
    }
}
