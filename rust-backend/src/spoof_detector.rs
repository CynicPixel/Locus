// GPS spoof detector.
//
// Compares MLAT-derived 3D position against ADS-B claimed position using ECEF
// Euclidean distance. The threshold is GDOP-adaptive: only flags when divergence
// exceeds 3σ of MLAT's own uncertainty, avoiding false positives under poor geometry.

use std::collections::HashMap;

use crate::coords::wgs84_to_ecef;

// Spoof threshold is defined in crate::consts::SPOOF_THRESHOLD_M

pub struct SpoofDetector {
    /// Cached ADS-B claimed position in both WGS84 and ECEF.
    /// Tuple: (lat°, lon°, alt_m, ecef_x, ecef_y, ecef_z)
    claimed: HashMap<String, (f64, f64, f64, f64, f64, f64)>,
}

impl SpoofDetector {
    pub fn new() -> Self {
        Self {
            claimed: HashMap::new(),
        }
    }

    /// Record (or update) an ADS-B self-reported position.
    /// ECEF is computed once here and cached (Improvement 6).
    pub fn record_adsb(&mut self, icao24: &str, lat: f64, lon: f64, alt_m: f64) {
        let (cx, cy, cz) = wgs84_to_ecef(lat, lon, alt_m);
        self.claimed
            .insert(icao24.to_string(), (lat, lon, alt_m, cx, cy, cz));
    }

    /// Compare MLAT position against ADS-B claim.
    ///
    /// Returns `(spoofed, divergence_m)`.  `spoofed` is `true` when the 3D
    /// divergence exceeds `max(2 km, 3 × accuracy_m)`.
    pub fn check(
        &self,
        icao24: &str,
        mlat_lat: f64,
        mlat_lon: f64,
        mlat_alt_m: f64,
        accuracy_m: f64,
    ) -> (bool, f64) {
        match self.claimed.get(icao24) {
            None => (false, 0.0),
            Some(&(_, _, _, cx, cy, cz)) => {
                let (mx, my, mz) = wgs84_to_ecef(mlat_lat, mlat_lon, mlat_alt_m);
                let divergence_m =
                    ((mx - cx).powi(2) + (my - cy).powi(2) + (mz - cz).powi(2)).sqrt();

                // GDOP-adaptive threshold: only flag when divergence exceeds MLAT uncertainty
                let threshold = crate::consts::SPOOF_THRESHOLD_M.max(3.0 * accuracy_m);
                let flag = divergence_m > threshold;

                if flag {
                    tracing::warn!(
                        icao24,
                        divergence_m = divergence_m as u64,
                        threshold = threshold as u64,
                        accuracy_m = accuracy_m as u64,
                        "SPOOF DETECTED"
                    );
                }
                (flag, divergence_m)
            }
        }
    }
}
