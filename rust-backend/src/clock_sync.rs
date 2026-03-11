// Self-calibrating clock synchronisation engine.
//
// Uses beacon aircraft (any ADS-B position reporter) to measure per-sensor-pair
// clock offsets via the difference between expected and observed TDOA.
// Windowed median with MAD outlier gating converges in ~25 observations
// and is immune to multipath spikes.

use std::collections::{HashMap, VecDeque};

use serde::Serialize;

use crate::coords::ecef_dist;
use crate::ingestor::RawFrame;

const WINDOW_SIZE: usize = 50; // ~5 s at 10 beacon obs/s
const C_M_PER_NS: f64 = 0.299_792_458; // speed of light (m/ns)

// ---------------------------------------------------------------------------
// Per-pair state with dirty-flag cached median/MAD (Improvement 5)
// ---------------------------------------------------------------------------

struct PairState {
    samples: VecDeque<f64>,
    cached_median: f64,
    cached_mad: f64,
    dirty: bool,
}

impl PairState {
    fn new() -> Self {
        Self {
            samples: VecDeque::new(),
            cached_median: 0.0,
            cached_mad: 0.0,
            dirty: false,
        }
    }

    fn push(&mut self, value: f64) {
        self.samples.push_back(value);
        if self.samples.len() > WINDOW_SIZE {
            self.samples.pop_front();
        }
        self.dirty = true;
    }

    fn recompute_if_dirty(&mut self) {
        if !self.dirty || self.samples.is_empty() {
            return;
        }
        let mut v: Vec<f64> = self.samples.iter().copied().collect();
        v.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
        let mid = v.len() / 2;
        self.cached_median = if v.len() % 2 == 0 {
            (v[mid - 1] + v[mid]) / 2.0
        } else {
            v[mid]
        };
        let med = self.cached_median;
        let mut devs: Vec<f64> = v.iter().map(|x| (x - med).abs()).collect();
        devs.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
        self.cached_mad = devs[devs.len() / 2];
        self.dirty = false;
    }

    fn median(&mut self) -> f64 {
        self.recompute_if_dirty();
        self.cached_median
    }

    fn mad(&mut self) -> f64 {
        self.recompute_if_dirty();
        self.cached_mad
    }

    fn len(&self) -> usize {
        self.samples.len()
    }
}

// ---------------------------------------------------------------------------
// Public export type
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize)]
pub struct SensorOffsetReport {
    pub sensor_i: i64,
    pub sensor_j: i64,
    /// Windowed median of observed TDOA error (nanoseconds).
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

        let pair = self
            .pairs
            .entry((sensor_i, sensor_j))
            .or_insert_with(PairState::new);

        // Outlier gate: reject if > 3× MAD from current median
        if pair.len() >= 5 {
            let med = pair.median();
            let mad = pair.mad().max(50.0); // floor at 50 ns
            if (error_ns - med).abs() > 3.0 * mad {
                tracing::debug!(
                    "clock obs rejected: error={:.1}ns median={:.1}ns mad={:.1}ns",
                    error_ns,
                    med,
                    mad
                );
                return;
            }
        }

        pair.push(error_ns);

        if pair.len() % 10 == 0 {
            tracing::debug!(
                "clock[{}→{}] offset={:.1}ns n={}",
                sensor_i,
                sensor_j,
                pair.median(),
                pair.len()
            );
        }
    }

    /// Return the current median offset for a sensor pair (nanoseconds).
    /// Returns 0.0 if no observations yet.
    pub fn get_offset(&mut self, sensor_i: i64, sensor_j: i64) -> f64 {
        self.pairs
            .entry((sensor_i, sensor_j))
            .or_insert_with(PairState::new)
            .median()
    }

    #[allow(dead_code)]
    pub fn is_converged(&self, sensor_i: i64, sensor_j: i64) -> bool {
        self.pairs
            .get(&(sensor_i, sensor_j))
            .map(|p| p.len() >= WINDOW_SIZE / 2)
            .unwrap_or(false)
    }

    /// Apply per-sensor clock corrections to a frame slice **in-place**.
    /// Must be called before TDOA assembly. `frames[0]` is the reference.
    pub fn apply_corrections(&mut self, frames: &mut [RawFrame]) {
        if frames.len() < 2 {
            return;
        }
        let ref_id = frames[0].sensor_id;
        // Build corrections first to avoid borrow conflict
        let corrections: Vec<f64> = frames[1..]
            .iter()
            .map(|f| self.get_offset(f.sensor_id, ref_id))
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
    pub fn export_offsets(&self) -> Vec<SensorOffsetReport> {
        self.pairs
            .iter()
            .map(|(&(si, sj), p)| SensorOffsetReport {
                sensor_i: si,
                sensor_j: sj,
                offset_ns: p.cached_median,
                sample_count: p.len(),
                is_converged: p.len() >= WINDOW_SIZE / 2,
            })
            .collect()
    }
}
