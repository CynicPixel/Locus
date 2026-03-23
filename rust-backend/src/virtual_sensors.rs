// Virtual sensor registry for interactive GDOP heatmap planning.
//
// Provides per-client virtual sensor management with:
// - Transient state (cleared on disconnect, never persisted to disk)
// - Input validation (coordinate bounds, altitude limits, DoS protection)
// - ECEF caching (computed once on insert)
// - Zero contamination of production MLAT pipeline

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};

// ---------------------------------------------------------------------------
// Data Structures
// ---------------------------------------------------------------------------

/// Unique identifier for virtual sensors (monotonic counter).
/// Starts at 1,000,000 to avoid collision with real sensor IDs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct VirtualSensorId(pub u64);

impl VirtualSensorId {
    pub fn new() -> Self {
        static COUNTER: AtomicU64 = AtomicU64::new(crate::consts::VIRTUAL_SENSOR_ID_START as u64);
        Self(COUNTER.fetch_add(1, Ordering::Relaxed))
    }
}

impl Default for VirtualSensorId {
    fn default() -> Self {
        Self::new()
    }
}

/// Virtual sensor (transient, planning-only).
///
/// Represents a hypothetical sensor placement for what-if analysis.
/// Never participates in production MLAT solving.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VirtualSensor {
    pub id: VirtualSensorId,
    pub lat: f64,        // WGS84 latitude (decimal degrees)
    pub lon: f64,        // WGS84 longitude (decimal degrees)
    pub alt_m: f64,      // MSL altitude (meters)
    #[serde(skip)]
    pub ecef: Vector3<f64>,  // Cached ECEF position (computed on insert)
}

/// Per-client virtual sensor collection.
///
/// Each WebSocket client maintains its own isolated registry.
/// Sensors are ephemeral - automatically dropped on client disconnect.
#[derive(Clone)]
pub struct VirtualSensorRegistry {
    sensors: HashMap<VirtualSensorId, VirtualSensor>,
    max_sensors: usize,  // DoS protection
}

impl VirtualSensorRegistry {
    /// Create new registry with DoS limit.
    pub fn new(max_sensors: usize) -> Self {
        Self {
            sensors: HashMap::new(),
            max_sensors,
        }
    }

    /// Add virtual sensor with validation.
    ///
    /// # Validation
    /// - Enforces max_sensors capacity limit
    /// - Validates WGS84 coordinate bounds (lat: [-90, 90], lon: [-180, 180])
    /// - Validates altitude range (0m - FL500 / 15,240m MSL)
    /// - Computes and caches ECEF position
    ///
    /// # Returns
    /// - `Ok(VirtualSensorId)` on success
    /// - `Err(String)` with human-readable error message on validation failure
    pub fn add(&mut self, lat: f64, lon: f64, alt_m: f64) -> Result<VirtualSensorId, String> {
        // Capacity check (DoS protection)
        if self.sensors.len() >= self.max_sensors {
            return Err(format!(
                "Maximum {} virtual sensors exceeded",
                self.max_sensors
            ));
        }

        // Coordinate validation
        if !lat.is_finite() || !lon.is_finite() || !alt_m.is_finite() {
            return Err("Coordinates must be finite numbers".to_string());
        }

        if !(-90.0..=90.0).contains(&lat) {
            return Err(format!("Latitude {} out of range [-90, 90]", lat));
        }

        if !(-180.0..=180.0).contains(&lon) {
            return Err(format!("Longitude {} out of range [-180, 180]", lon));
        }

        // Altitude validation (0m - FL500 = 15,240m MSL)
        if !(0.0..=crate::consts::MAX_VIRTUAL_SENSOR_ALTITUDE_M).contains(&alt_m) {
            return Err(format!(
                "Altitude {} out of range [0, 15240]m MSL",
                alt_m
            ));
        }

        // Compute ECEF position (cached for fast GDOP computation)
        let (x, y, z) = crate::coords::wgs84_to_ecef(lat, lon, alt_m);

        let id = VirtualSensorId::new();
        self.sensors.insert(
            id,
            VirtualSensor {
                id,
                lat,
                lon,
                alt_m,
                ecef: Vector3::new(x, y, z),
            },
        );

        tracing::debug!(
            sensor_id = ?id,
            lat,
            lon,
            alt_m,
            "virtual sensor added"
        );

        Ok(id)
    }

    /// Remove virtual sensor by ID.
    ///
    /// # Returns
    /// `true` if sensor was removed, `false` if ID not found.
    pub fn remove(&mut self, id: VirtualSensorId) -> bool {
        let removed = self.sensors.remove(&id).is_some();
        if removed {
            tracing::debug!(sensor_id = ?id, "virtual sensor removed");
        }
        removed
    }

    /// Clear all virtual sensors.
    ///
    /// Typically called when client exits planning mode.
    pub fn clear(&mut self) {
        let count = self.sensors.len();
        self.sensors.clear();
        if count > 0 {
            tracing::debug!(count, "virtual sensors cleared");
        }
    }

    /// Get ECEF positions for all virtual sensors.
    ///
    /// Used to merge with real sensors for GDOP computation.
    pub fn get_ecef_positions(&self) -> Vec<Vector3<f64>> {
        self.sensors.values().map(|s| s.ecef).collect()
    }

    /// Export all virtual sensors for serialization.
    ///
    /// Used for WebSocket broadcast to client.
    pub fn export_all(&self) -> Vec<VirtualSensor> {
        self.sensors.values().cloned().collect()
    }

    /// Get virtual sensor count.
    pub fn count(&self) -> usize {
        self.sensors.len()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_add_virtual_sensor_valid() {
        let mut registry = VirtualSensorRegistry::new(10);
        let id = registry.add(37.7749, -122.4194, 9144.0).unwrap();
        assert_eq!(registry.count(), 1);

        let sensors = registry.export_all();
        assert_eq!(sensors.len(), 1);
        assert_eq!(sensors[0].lat, 37.7749);
        assert_eq!(sensors[0].lon, -122.4194);
        assert_eq!(sensors[0].alt_m, 9144.0);
        assert_eq!(sensors[0].id, id);
    }

    #[test]
    fn test_max_sensors_limit() {
        let mut registry = VirtualSensorRegistry::new(2);
        registry.add(37.0, -122.0, 1000.0).unwrap();
        registry.add(38.0, -122.0, 1000.0).unwrap();

        // 3rd sensor should fail
        let result = registry.add(39.0, -122.0, 1000.0);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Maximum 2"));
        assert_eq!(registry.count(), 2);
    }

    #[test]
    fn test_coordinate_validation_latitude() {
        let mut registry = VirtualSensorRegistry::new(10);

        // Invalid latitude (too high)
        assert!(registry.add(91.0, 0.0, 1000.0).is_err());
        // Invalid latitude (too low)
        assert!(registry.add(-91.0, 0.0, 1000.0).is_err());
        // Invalid latitude (NaN)
        assert!(registry.add(f64::NAN, 0.0, 1000.0).is_err());
        // Invalid latitude (infinity)
        assert!(registry.add(f64::INFINITY, 0.0, 1000.0).is_err());

        // Valid edge cases
        assert!(registry.add(90.0, 0.0, 1000.0).is_ok());
        assert!(registry.add(-90.0, 0.0, 1000.0).is_ok());
    }

    #[test]
    fn test_coordinate_validation_longitude() {
        let mut registry = VirtualSensorRegistry::new(10);

        // Invalid longitude (too high)
        assert!(registry.add(0.0, 181.0, 1000.0).is_err());
        // Invalid longitude (too low)
        assert!(registry.add(0.0, -181.0, 1000.0).is_err());
        // Invalid longitude (NaN)
        assert!(registry.add(0.0, f64::NAN, 1000.0).is_err());

        // Valid edge cases
        assert!(registry.add(0.0, 180.0, 1000.0).is_ok());
        assert!(registry.add(0.0, -180.0, 1000.0).is_ok());
    }

    #[test]
    fn test_altitude_validation() {
        let mut registry = VirtualSensorRegistry::new(10);

        // Invalid altitude (negative)
        assert!(registry.add(0.0, 0.0, -1.0).is_err());
        // Invalid altitude (too high)
        assert!(registry.add(0.0, 0.0, 20000.0).is_err());
        // Invalid altitude (NaN)
        assert!(registry.add(0.0, 0.0, f64::NAN).is_err());

        // Valid edge cases
        assert!(registry.add(0.0, 0.0, 0.0).is_ok());       // Sea level
        assert!(registry.add(0.0, 0.0, 15240.0).is_ok());   // FL500
    }

    #[test]
    fn test_ecef_caching() {
        let mut registry = VirtualSensorRegistry::new(10);
        registry.add(0.0, 0.0, 0.0).unwrap(); // Equator, sea level

        let positions = registry.get_ecef_positions();
        assert_eq!(positions.len(), 1);

        let expected = crate::coords::wgs84_to_ecef(0.0, 0.0, 0.0);

        assert_relative_eq!(positions[0].x, expected.0, epsilon = 0.01);
        assert_relative_eq!(positions[0].y, expected.1, epsilon = 0.01);
        assert_relative_eq!(positions[0].z, expected.2, epsilon = 0.01);
    }

    #[test]
    fn test_remove_sensor() {
        let mut registry = VirtualSensorRegistry::new(10);
        let id1 = registry.add(37.0, -122.0, 1000.0).unwrap();
        let id2 = registry.add(38.0, -122.0, 1000.0).unwrap();

        assert_eq!(registry.count(), 2);

        // Remove first sensor
        assert!(registry.remove(id1));
        assert_eq!(registry.count(), 1);

        // Try to remove already-removed sensor
        assert!(!registry.remove(id1));
        assert_eq!(registry.count(), 1);

        // Remove second sensor
        assert!(registry.remove(id2));
        assert_eq!(registry.count(), 0);
    }

    #[test]
    fn test_clear_sensors() {
        let mut registry = VirtualSensorRegistry::new(10);
        registry.add(37.0, -122.0, 1000.0).unwrap();
        registry.add(38.0, -122.0, 1000.0).unwrap();
        registry.add(39.0, -122.0, 1000.0).unwrap();

        assert_eq!(registry.count(), 3);

        registry.clear();
        assert_eq!(registry.count(), 0);
        assert_eq!(registry.export_all().len(), 0);

        // Clear empty registry (no-op)
        registry.clear();
        assert_eq!(registry.count(), 0);
    }

    #[test]
    fn test_unique_ids() {
        let mut registry = VirtualSensorRegistry::new(10);
        let id1 = registry.add(37.0, -122.0, 1000.0).unwrap();
        let id2 = registry.add(38.0, -122.0, 1000.0).unwrap();
        let id3 = registry.add(39.0, -122.0, 1000.0).unwrap();

        // IDs should be unique
        assert_ne!(id1, id2);
        assert_ne!(id2, id3);
        assert_ne!(id1, id3);

        // IDs should be monotonically increasing
        assert!(id2.0 > id1.0);
        assert!(id3.0 > id2.0);
    }

    #[test]
    fn test_export_all() {
        let mut registry = VirtualSensorRegistry::new(10);
        let id1 = registry.add(37.0, -122.0, 1000.0).unwrap();
        let id2 = registry.add(38.0, -123.0, 2000.0).unwrap();

        let sensors = registry.export_all();
        assert_eq!(sensors.len(), 2);

        // Find sensors by ID (order not guaranteed from HashMap)
        let s1 = sensors.iter().find(|s| s.id == id1).unwrap();
        let s2 = sensors.iter().find(|s| s.id == id2).unwrap();

        assert_eq!(s1.lat, 37.0);
        assert_eq!(s1.lon, -122.0);
        assert_eq!(s1.alt_m, 1000.0);

        assert_eq!(s2.lat, 38.0);
        assert_eq!(s2.lon, -123.0);
        assert_eq!(s2.alt_m, 2000.0);
    }
}
