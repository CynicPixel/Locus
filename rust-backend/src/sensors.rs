// Sensor position registry — caches ECEF positions so wgs84_to_ecef() is
// only called once per sensor (sensors don't move).
// Also caches the pre-computed LM solver initial guess (sensor centroid at cruise altitude).

use std::collections::HashMap;

use nalgebra::Vector3;

use crate::coords::wgs84_to_ecef;
use crate::ingestor::RawFrame;

pub struct SensorRegistry {
    ecef: HashMap<i64, Vector3<f64>>,
    /// Sensor centroid scaled to typical cruise altitude — stable after warmup.
    /// Used as the cold-start initial guess for the LM solver.
    pub initial_guess_ecef: Option<Vector3<f64>>,
}

impl SensorRegistry {
    pub fn new() -> Self {
        Self {
            ecef: HashMap::new(),
            initial_guess_ecef: None,
        }
    }

    /// Register a sensor on first sight; no-op thereafter.
    pub fn register(&mut self, frame: &RawFrame) {
        if self.ecef.contains_key(&frame.sensor_id) {
            return; // already registered — fast path
        }
        let (x, y, z) = wgs84_to_ecef(frame.sensor_lat, frame.sensor_lon, frame.sensor_alt);
        tracing::info!(
            sensor_id = frame.sensor_id,
            lat = frame.sensor_lat,
            lon = frame.sensor_lon,
            alt = frame.sensor_alt,
            "sensor registered"
        );
        self.ecef.insert(frame.sensor_id, Vector3::new(x, y, z));
        self.recompute_initial_guess();
    }

    fn recompute_initial_guess(&mut self) {
        if self.ecef.is_empty() {
            return;
        }
        let n = self.ecef.len() as f64;
        let centroid = self.ecef.values().fold(Vector3::zeros(), |a, s| a + s) / n;
        let norm = centroid.norm();
        if norm > 1.0 {
            self.initial_guess_ecef = Some(centroid * ((6_371_000.0 + 10_000.0) / norm));
        }
    }

    pub fn get(&self, sensor_id: i64) -> Option<&Vector3<f64>> {
        self.ecef.get(&sensor_id)
    }

    #[allow(dead_code)]
    pub fn all_ecef(&self) -> Vec<Vector3<f64>> {
        self.ecef.values().copied().collect()
    }

    #[allow(dead_code)]
    pub fn sensor_ids_sorted(&self) -> Vec<i64> {
        let mut ids: Vec<i64> = self.ecef.keys().copied().collect();
        ids.sort_unstable();
        ids
    }
}
