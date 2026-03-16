// Self-calibrating clock synchronisation engine.
//
// Uses beacon aircraft (any ADS-B position reporter) to measure per-sensor-pair
// clock offsets via the difference between expected and observed TDOA.
//
// FIX-6: Linear regression on (timestamp, offset) pairs tracks clock drift
//   (frequency error) so the correction tracks real receiver clock behaviour.
//   Replaces simple windowed median which cannot account for drift.
// FIX-11: Pairs expire after 120 s without update; corrections become invalid
//   after 30 s of inactivity, falling back to zero correction gracefully.

use std::collections::{HashMap, VecDeque};

use serde::Serialize;

use crate::coords::ecef_dist;
use crate::ingestor::RawFrame;

const WINDOW_SIZE: usize = 50; // ~5 s at 10 beacon obs/s
/// Speed of light in air (m/ns). c_vacuum/1.0003 — matches mlat-server Cair.
const C_M_PER_NS: f64 = 0.299_702_547;

/// Pair validity window: corrections become zero if not updated for this long.
const PAIR_VALIDITY_NS: u64 = 30_000_000_000; // 30 seconds
/// Pair expiry: remove pair state if silent for this long.
const PAIR_EXPIRY_NS: u64 = 120_000_000_000; // 120 seconds
/// Minimum observations before using a pair's estimate.
const MIN_SAMPLE_COUNT: usize = 2;

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

pub struct ClockSyncEngine {
    pairs: HashMap<(i64, i64), PairState>,
}

impl ClockSyncEngine {
    pub fn new() -> Self {
        Self {
            pairs: HashMap::new(),
        }
    }

    /// Update clock offset using a beacon aircraft whose ECEF position is known.
    ///
    /// `pos_i` / `pos_j` are ECEF tuples (meters) — pass from `SensorRegistry`
    /// to avoid repeated `wgs84_to_ecef` calls.
    pub fn update_from_beacon(
        &mut self,
        sensor_i: i64,
        pos_i: (f64, f64, f64),
        sensor_j: i64,
        pos_j: (f64, f64, f64),
        t_i_ns: u64,
        t_j_ns: u64,
        beacon_ecef: (f64, f64, f64),
    ) {
        let dist_i = ecef_dist(pos_i, beacon_ecef);
        let dist_j = ecef_dist(pos_j, beacon_ecef);
        let expected_tdoa_ns = (dist_i - dist_j) / C_M_PER_NS;
        let observed_tdoa_ns = t_i_ns as f64 - t_j_ns as f64;
        let error_ns = observed_tdoa_ns - expected_tdoa_ns;

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
            if (error_ns - mean).abs() > 3000.0 { // 3 µs gross outlier gate during warmup
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
    pub fn get_offset_at(&mut self, sensor_i: i64, sensor_j: i64, at_ns: u64) -> f64 {
        let pair = self.pairs
            .entry((sensor_i, sensor_j))
            .or_insert_with(|| PairState::new(at_ns));
        if pair.is_valid(at_ns) { pair.estimate_at(at_ns) } else { 0.0 }
    }

    /// Variance of TDOA error for the pair (nanoseconds²).
    /// Used by mlat_solver for per-measurement weighting (FIX-4).
    /// Returns a conservative default if pair not yet converged.
    pub fn get_pair_variance_ns2(&mut self, sensor_i: i64, sensor_j: i64) -> f64 {
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

        for (frame, correction_ns) in frames[1..].iter_mut().zip(corrections) {
            let raw_ns = frame.timestamp_ns() as i64;
            let corrected_ns = (raw_ns - correction_ns.round() as i64).max(0) as u64;
            frame.timestamp_seconds = corrected_ns / 1_000_000_000;
            frame.timestamp_nanoseconds = corrected_ns % 1_000_000_000;
            frame.timestamp_ns_cache = corrected_ns;
        }
    }

    /// Snapshot of all pair offsets for WebSocket broadcast.
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
}
