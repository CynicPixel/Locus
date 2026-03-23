// Message correlator — groups RawFrames by (ICAO24, content_hash) within a 200 ms window.
// Emits groups with >= min_sensors observations via timed eviction.
//
// FIX-9: Accepts all Mode S message types (DF 4/5/11/17/18/20/21) — not just DF17/18.
//   Non-extended-squitter messages (DF4/5/20/21) don't carry position, but CAN be
//   multilaterated to locate non-ADS-B aircraft (requires ≥4 receivers since no altitude).
//   ICAO24 is extracted via CRC residual for surveillance replies.
// FIX-19: Correlation window increased from 50 ms to 200 ms to capture late-arriving frames.

use std::collections::HashMap;

use crate::adsb_parser::crc24_residual;
use crate::ingestor::RawFrame;

// ---------------------------------------------------------------------------
// Correlation key
// ---------------------------------------------------------------------------

/// Identifies a unique Mode-S transmission: same aircraft, same payload.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct CorrelationKey {
    pub icao24: [u8; 3],
    pub content_hash: u64, // FxHash pre-computed in RawFrame::finalize()
}

impl CorrelationKey {
    /// Build a correlation key from a finalized `RawFrame`.
    ///
    /// Supported DF types (FIX-9):
    /// - DF 17/18/11: CRC must be 0; ICAO24 in bytes[1:4].
    /// - DF 4/5/20/21: CRC residual = ICAO24 (address/parity field).
    /// - Other DFs and malformed frames return `None`.
    pub fn from_frame(frame: &RawFrame) -> Option<Self> {
        let bytes = frame.bytes()?;
        if bytes.len() < 7 {
            return None;
        }
        let df = (bytes[0] >> 3) & 0x1F;

        match df {
            // Extended squitters and all-call — ICAO in bytes 1:4, CRC must be 0.
            11 if bytes.len() >= 7 => {
                let icao24 = [bytes[1], bytes[2], bytes[3]];
                Some(CorrelationKey { icao24, content_hash: frame.content_hash })
            }
            17 | 18 if bytes.len() >= 14 => {
                let icao24 = [bytes[1], bytes[2], bytes[3]];
                Some(CorrelationKey { icao24, content_hash: frame.content_hash })
            }
            // Surveillance replies — ICAO = CRC24 residual (address/parity).
            4 | 5 if bytes.len() >= 7 => {
                let icao_u32 = crc24_residual(bytes);
                let icao24 = [
                    ((icao_u32 >> 16) & 0xFF) as u8,
                    ((icao_u32 >> 8) & 0xFF) as u8,
                    (icao_u32 & 0xFF) as u8,
                ];
                Some(CorrelationKey { icao24, content_hash: frame.content_hash })
            }
            20 | 21 if bytes.len() >= 14 => {
                let icao_u32 = crc24_residual(bytes);
                let icao24 = [
                    ((icao_u32 >> 16) & 0xFF) as u8,
                    ((icao_u32 >> 8) & 0xFF) as u8,
                    (icao_u32 & 0xFF) as u8,
                ];
                Some(CorrelationKey { icao24, content_hash: frame.content_hash })
            }
            _ => None,
        }
    }
}

// ---------------------------------------------------------------------------
// Correlation group
// ---------------------------------------------------------------------------

/// Accumulated observations of the same Mode-S transmission from multiple sensors.
#[derive(Debug, Clone)]
pub struct CorrelationGroup {
    pub key: CorrelationKey,
    /// One entry per sensor (deduplicated by sensor_id).
    pub frames: Vec<RawFrame>,
    /// Timestamp of the first frame received into this group.
    pub first_seen_ns: u64,
}

// ---------------------------------------------------------------------------
// Correlator
// ---------------------------------------------------------------------------

/// Time-windowed message correlator.
///
/// Frames are accumulated via [`Correlator::ingest`].
/// Completed groups (age > window, sensor count >= min) are emitted by
/// [`Correlator::evict_stale`], which should be called every ~10 ms.
pub struct Correlator {
    groups: HashMap<CorrelationKey, CorrelationGroup>,
    /// Minimum number of sensors required to emit a group (default 3).
    min_sensors: usize,
    /// Eviction window in nanoseconds (default 50 ms).
    window_ns: u64,
}

impl Correlator {
    pub fn new() -> Self {
        Self {
            groups: HashMap::new(),
            min_sensors: 2, // Changed from 3 to enable semi-MLAT (2-sensor observations)
            window_ns: 200_000_000, // FIX-19: 200 ms (was 50 ms) — captures late-arriving frames
        }
    }

    /// Feed one frame into the correlator.  Never emits early — emission is
    /// exclusively via [`evict_stale`], ensuring the full 50 ms window is used.
    pub fn ingest(&mut self, frame: RawFrame) {
        let key = match CorrelationKey::from_frame(&frame) {
            Some(k) => k,
            None => return,
        };
        let now_ns = frame.timestamp_ns();
        let group = self.groups.entry(key.clone()).or_insert_with(|| CorrelationGroup {
            key: key.clone(),
            frames: Vec::new(),
            first_seen_ns: now_ns,
        });
        // Deduplicate: accept at most one frame per sensor
        if !group.frames.iter().any(|f| f.sensor_id == frame.sensor_id) {
            group.frames.push(frame);
        }
    }

    /// Evict groups older than the window relative to `now_ns`.
    /// Groups with >= min_sensors frames are returned for MLAT solving;
    /// under-populated groups are silently dropped.
    ///
    /// `now_ns` should be set to the timestamp of the most-recently received
    /// frame — **not** the wall clock — to remain stable during data gaps.
    pub fn evict_stale(&mut self, now_ns: u64) -> Vec<CorrelationGroup> {
        let window = self.window_ns;
        let min = self.min_sensors;
        let mut emitted = Vec::new();

        self.groups.retain(|_, group| {
            let age = now_ns.saturating_sub(group.first_seen_ns);
            if age > window {
                if group.frames.len() >= min {
                    emitted.push(group.clone());
                } else {
                    tracing::debug!(
                        icao24 = ?group.key.icao24,
                        n_sensors = group.frames.len(),
                        "evicting incomplete group (insufficient sensors)"
                    );
                }
                false // remove
            } else {
                true // keep
            }
        });

        emitted
    }

    /// Current number of live groups (diagnostic / health metric).
    pub fn live_group_count(&self) -> usize {
        self.groups.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_frame(sensor_id: i64, icao: [u8; 3], ts_ns: u64) -> RawFrame {
        // Construct a minimal DF17 frame
        let mut bytes = vec![0u8; 14];
        bytes[0] = 0x8D; // DF17 = 17 << 3 | capability
        bytes[1] = icao[0];
        bytes[2] = icao[1];
        bytes[3] = icao[2];
        let hex = hex::encode(&bytes);
        let mut frame = RawFrame {
            sensor_id,
            sensor_lat: 50.0,
            sensor_lon: -5.0,
            sensor_alt: 10.0,
            timestamp_seconds: ts_ns / 1_000_000_000,
            timestamp_nanoseconds: ts_ns % 1_000_000_000,
            raw_modes_hex: hex,
            timestamp_ns_cache: 0,
            decoded_bytes: None,
            content_hash: 0,
        };
        frame = frame.finalize();
        frame
    }

    #[test]
    fn groups_by_icao_and_hash() {
        let mut c = Correlator::new();
        let icao = [0x48, 0x40, 0xD6];
        let base_ns = 1_700_000_000_000_000_000u64;

        // 4 sensors see the same frame
        for sensor in 1..=4 {
            c.ingest(make_frame(sensor, icao, base_ns + sensor as u64 * 1_000_000));
        }

        // Not evicted yet — window is 200 ms
        assert!(c.evict_stale(base_ns + 10_000_000).is_empty());

        // After 210 ms, should emit one group with 4 sensors
        let emitted = c.evict_stale(base_ns + 210_000_000);
        assert_eq!(emitted.len(), 1);
        assert_eq!(emitted[0].frames.len(), 4);
    }

    #[test]
    fn drops_insufficient_sensors() {
        let mut c = Correlator::new();
        let icao = [0x01, 0x02, 0x03];
        let base_ns = 1_700_000_000_000_000_000u64;

        // Only 1 sensor (below min_sensors=2) — semi-MLAT requires at least 2
        c.ingest(make_frame(1, icao, base_ns));

        let emitted = c.evict_stale(base_ns + 210_000_000);
        assert!(emitted.is_empty());
    }
}
