// ADS-B (DF17/18) position parsing + CPR decoding.
// Used by: correlator (clock sync beacons), spoof_detector (claimed position feed).

use std::collections::HashMap;

// ---------------------------------------------------------------------------
// ADS-B decoded position
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct AdsbPosition {
    pub icao24: String,
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f64,
}

// ---------------------------------------------------------------------------
// NL lookup table (ICAO Doc 9684 Table C-7)
// Binary search replacing 4× transcendental calls per CPR decode.
// ---------------------------------------------------------------------------

/// NL breakpoints: NL_BREAKS[i] is the minimum |lat| (°) at which NL drops to (59–i).
static NL_BREAKS: [f64; 58] = [
    10.470_471_30,
    14.828_174_37,
    18.186_263_57,
    21.029_394_97,
    23.545_044_87,
    25.829_247_07,
    27.938_987_10,
    29.911_356_86,
    31.772_097_08,
    33.539_934_36,
    35.228_995_98,
    36.850_251_08,
    38.412_418_92,
    39.922_566_84,
    41.386_518_32,
    42.809_140_12,
    44.194_549_51,
    45.546_267_23,
    46.867_332_52,
    48.160_391_28,
    49.427_764_39,
    50.671_501_66,
    51.893_424_69,
    53.095_161_53,
    54.278_174_72,
    55.443_784_44,
    56.593_187_56,
    57.727_473_54,
    58.847_637_76,
    59.954_592_77,
    61.049_177_74,
    62.132_166_59,
    63.204_274_79,
    64.266_165_23,
    65.318_453_10,
    66.361_710_08,
    67.396_467_74,
    68.423_220_22,
    69.442_426_31,
    70.454_510_75,
    71.459_864_73,
    72.458_845_45,
    73.451_774_42,
    74.438_934_16,
    75.420_562_57,
    76.396_843_91,
    77.367_894_61,
    78.333_740_83,
    79.294_282_25,
    80.249_232_13,
    81.198_013_49,
    82.139_569_81,
    83.071_994_45,
    84.000_000_00,
    84.891_661_91,
    85.755_416_21,
    86.535_369_98,
    87.000_000_00,
];

/// Number of longitude zones at the given latitude (ICAO Doc 9684).
fn nl(lat: f64) -> f64 {
    let lat = lat.abs();
    if lat >= 87.0 {
        return 1.0;
    }
    let breaks_below = NL_BREAKS.partition_point(|&b| b <= lat);
    (59 - breaks_below) as f64
}

// ---------------------------------------------------------------------------
// Altitude decode (Gillham code)
// ---------------------------------------------------------------------------

fn decode_altitude_m(raw: u16) -> Option<f64> {
    // Q-bit (bit 4 of the 13-bit field) selects encoding.
    let q_bit = (raw >> 4) & 1;
    if q_bit == 1 {
        // 25 ft increments, offset -1000 ft
        let n = ((raw & !0x10) as i32) - 13;
        Some((n as f64 * 25.0 - 1000.0) * 0.304_8)
    } else {
        None // Gillham (Gray code) — skip for now
    }
}

// ---------------------------------------------------------------------------
// CPR decoder — per-ICAO24 stateful even/odd pair accumulator
// ---------------------------------------------------------------------------

#[derive(Default)]
struct CprDecoder {
    even: Option<(u32, u32, u64)>, // (lat_cpr, lon_cpr, timestamp_ns)
    odd: Option<(u32, u32, u64)>,
}

const MAX_PAIR_AGE_NS: u64 = 10_000_000_000; // 10 s (ICAO Doc 9684 §C.2.6)
const CPR_BITS: f64 = (1u64 << 17) as f64; // 2^17

impl CprDecoder {
    fn feed(
        &mut self,
        lat_cpr: u32,
        lon_cpr: u32,
        f_flag: u8,
        timestamp_ns: u64,
    ) -> Option<(f64, f64)> {
        if f_flag == 0 {
            self.even = Some((lat_cpr, lon_cpr, timestamp_ns));
        } else {
            self.odd = Some((lat_cpr, lon_cpr, timestamp_ns));
        }

        let (even_lat, even_lon, even_ts) = self.even?;
        let (odd_lat, odd_lon, odd_ts) = self.odd?;

        // Reject stale pairs
        let age_diff = even_ts.abs_diff(odd_ts);
        if age_diff > MAX_PAIR_AGE_NS {
            if even_ts < odd_ts {
                self.even = None;
            } else {
                self.odd = None;
            }
            return None;
        }

        let dlat_e = 360.0 / 60.0_f64;
        let dlat_o = 360.0 / 59.0_f64;

        let j = ((59.0 * even_lat as f64 - 60.0 * odd_lat as f64) / CPR_BITS + 0.5).floor();
        let lat_e = dlat_e * (j % 60.0 + even_lat as f64 / CPR_BITS);
        let lat_o = dlat_o * (j % 59.0 + odd_lat as f64 / CPR_BITS);

        // Normalize per ICAO Doc 9684
        let lat_e = if lat_e >= 270.0 {
            lat_e - 360.0
        } else if lat_e > 90.0 {
            return None; // invalid zone
        } else {
            lat_e
        };
        let lat_o = if lat_o >= 270.0 {
            lat_o - 360.0
        } else if lat_o > 90.0 {
            return None;
        } else {
            lat_o
        };

        // Zone consistency check
        if nl(lat_e) as u32 != nl(lat_o) as u32 {
            return None;
        }

        // Use most-recent frame for longitude
        let (lat, m, cpr_lon) = if f_flag == 1 {
            (lat_o, (nl(lat_o) - 1.0).max(1.0), odd_lon)
        } else {
            (lat_e, nl(lat_e).max(1.0), even_lon)
        };

        let dlon = 360.0 / m;
        let mi = ((even_lon as f64 * (nl(lat_e) - 1.0)
            - odd_lon as f64 * nl(lat_e))
            / CPR_BITS
            + 0.5)
            .floor();
        let lon = dlon * (mi % m + cpr_lon as f64 / CPR_BITS);
        let lon = if lon >= 180.0 { lon - 360.0 } else { lon };

        Some((lat, lon))
    }
}

// ---------------------------------------------------------------------------
// ADS-B parser
// ---------------------------------------------------------------------------

/// Per-ICAO24 CPR decoder state (retained across frames).
pub struct AdsbParser {
    cpr_state: HashMap<[u8; 3], CprDecoder>,
}

impl AdsbParser {
    pub fn new() -> Self {
        Self {
            cpr_state: HashMap::new(),
        }
    }

    /// Attempt to decode an airborne position from raw Mode-S bytes.
    /// Returns `Some(AdsbPosition)` when a valid even/odd CPR pair is available.
    pub fn parse(&mut self, bytes: &[u8], timestamp_ns: u64) -> Option<AdsbPosition> {
        if bytes.len() < 14 {
            return None;
        }
        let df = (bytes[0] >> 3) & 0x1F;
        if df != 17 && df != 18 {
            return None;
        }
        let tc = (bytes[4] >> 3) & 0x1F;
        if !(9..=18).contains(&tc) {
            return None; // airborne position TCs only
        }

        let icao = [bytes[1], bytes[2], bytes[3]];
        let icao24 = format!("{:02X}{:02X}{:02X}", icao[0], icao[1], icao[2]);

        // Altitude (13-bit Gillham/Q-bit field across bytes 5–6)
        let alt_raw = (((bytes[5] as u16) & 0xFF) << 4) | ((bytes[6] as u16) >> 4);
        let alt_m = decode_altitude_m(alt_raw)?;

        // CPR lat/lon
        let lat_cpr = ((bytes[6] as u32 & 0x03) << 15)
            | ((bytes[7] as u32) << 7)
            | (bytes[8] as u32 >> 1);
        let lon_cpr = ((bytes[8] as u32 & 0x01) << 16)
            | ((bytes[9] as u32) << 8)
            | bytes[10] as u32;
        let f_flag = (bytes[6] >> 2) & 0x01;

        let decoder = self.cpr_state.entry(icao).or_default();
        let (lat, lon) = decoder.feed(lat_cpr, lon_cpr, f_flag, timestamp_ns)?;

        Some(AdsbPosition {
            icao24,
            lat,
            lon,
            alt_m,
        })
    }
}
