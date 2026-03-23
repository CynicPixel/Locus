// ECEF ↔ WGS84 coordinate conversion
// Used by: mlat_solver, kalman, spoof_detector, clock_sync

/// WGS84 ellipsoid parameters — canonical values in crate::consts.
use crate::consts::{WGS84_A as A, WGS84_E2 as E2};

/// Convert WGS84 geodetic coordinates to ECEF Cartesian (meters).
#[inline]
pub fn wgs84_to_ecef(lat_deg: f64, lon_deg: f64, alt_m: f64) -> (f64, f64, f64) {
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    let (slat, clat) = (lat.sin(), lat.cos());
    let (slon, clon) = (lon.sin(), lon.cos());
    let n = A / (1.0 - E2 * slat * slat).sqrt();
    let x = (n + alt_m) * clat * clon;
    let y = (n + alt_m) * clat * slon;
    let z = (n * (1.0 - E2) + alt_m) * slat;
    (x, y, z)
}

/// Convert ECEF Cartesian (meters) to WGS84 geodetic (deg, deg, m).
/// Uses Bowring's iterative method — converges to sub-millimetre in 3–4 iterations.
pub fn ecef_to_wgs84(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    let lon = y.atan2(x);
    let p = (x * x + y * y).sqrt();
    let mut lat = z.atan2(p * (1.0 - E2));
    for _ in 0..crate::consts::BOWRING_ITERATIONS {
        let (slat, _clat) = (lat.sin(), lat.cos());
        let n = A / (1.0 - E2 * slat * slat).sqrt();
        lat = (z + E2 * n * lat.sin()).atan2(p);
    }
    let slat = lat.sin();
    let n = A / (1.0 - E2 * slat * slat).sqrt();
    let alt = if lat.cos().abs() > crate::consts::LAT_COS_THRESHOLD {
        p / lat.cos() - n
    } else {
        z / slat - n * (1.0 - E2)
    };
    (lat.to_degrees(), lon.to_degrees(), alt)
}

/// Euclidean distance between two ECEF points (meters).
#[inline]
pub fn ecef_dist(a: (f64, f64, f64), b: (f64, f64, f64)) -> f64 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    let dz = a.2 - b.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_london() {
        let (lat, lon, alt) = (51.5074_f64, -0.1278_f64, 15.0_f64);
        let (x, y, z) = wgs84_to_ecef(lat, lon, alt);
        let (lat2, lon2, alt2) = ecef_to_wgs84(x, y, z);
        assert!((lat - lat2).abs() < 1e-9, "lat roundtrip failed: {lat} vs {lat2}");
        assert!((lon - lon2).abs() < 1e-9, "lon roundtrip failed: {lon} vs {lon2}");
        assert!((alt - alt2).abs() < 1e-3, "alt roundtrip failed: {alt} vs {alt2}");
    }

    #[test]
    fn ecef_values_london() {
        // Approximate expected ECEF for London (51.5074, -0.1278, 15m)
        let (x, y, z) = wgs84_to_ecef(51.5074, -0.1278, 15.0);
        assert!((x - 3_978_553.0).abs() < 100.0, "x={x}");
        assert!((y - (-8_837.0)).abs() < 100.0, "y={y}");
        assert!((z - 4_968_195.0).abs() < 100.0, "z={z}");
    }
}
