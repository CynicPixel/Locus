// Named constants for the Locus MLAT backend.
//
// All magic numbers that appear in the codebase are defined here with doc
// comments explaining their meaning and provenance.  Every file in this crate
// references these via `crate::consts::CONSTANT_NAME`.

// ===== PHYSICS =====

/// Speed of light in air (m/ns) — converts TDOA nanoseconds to metres
pub const C_M_PER_NS: f64 = 0.299_702_547;
/// Feet to metres unit conversion factor
pub const FEET_TO_METERS: f64 = 0.3048;

// ===== WGS84 / GEODESY =====

/// WGS84 ellipsoid semi-major axis (metres)
pub const WGS84_A: f64 = 6_378_137.0;
/// WGS84 first eccentricity squared
pub const WGS84_E2: f64 = 6.694_379_990_14e-3;
/// Approximate metres per degree of latitude (equatorial)
pub const METERS_PER_DEGREE_LAT: f64 = 111_320.0;
/// Mean Earth radius (metres)
pub const EARTH_RADIUS_M: f64 = 6_371_000.0;
/// Earth radius plus typical cruise altitude — used as initial ECEF guess (metres)
pub const EARTH_RADIUS_CRUISE_ALT_M: f64 = 6_381_000.0;
/// Bowring iteration count for ECEF → WGS84 conversion
pub const BOWRING_ITERATIONS: usize = 10;
/// Cosine floor near poles for numerical stability
pub const LAT_COS_THRESHOLD: f64 = 1e-10;

// ===== TIMING & INTERVALS =====

/// Minimum time between ML anomaly classifications per aircraft (nanoseconds = 5 s)
pub const CLASSIFY_INTERVAL_NS: u64 = 5_000_000_000;
/// How often the correlator evicts stale entries (milliseconds)
pub const EVICTION_TICK_MS: u64 = 10;
/// Sensor health broadcast period (seconds)
pub const HEALTH_BROADCAST_SECS: u64 = 5;
/// How often the GDOP heatmap is recomputed (seconds)
pub const HEATMAP_TICK_SECS: u64 = 60;
/// Maximum TDOA age for a valid correlation group (nanoseconds = 200 ms)
pub const CORRELATION_WINDOW_NS: u64 = 200_000_000;
/// Minimum believable TDOA — guards against same-sensor echoes (nanoseconds = 1 µs)
pub const TDOA_GUARD_NS: f64 = 1_000.0;
/// Clock pair considered "fresh" for this long (nanoseconds = 30 s)
pub const PAIR_VALIDITY_NS: u64 = 30_000_000_000;
/// Clock pair discarded after this age (nanoseconds = 120 s)
pub const PAIR_EXPIRY_NS: u64 = 120_000_000_000;
/// Maximum window over which clock drift is extrapolated (nanoseconds = 120 s)
pub const MAX_DRIFT_EXTRAPOLATION_NS: u64 = 120_000_000_000;
/// How long OpenSky position data is cached before re-fetching (seconds = 5 min)
pub const OPENSKY_CACHE_TTL_SECS: u64 = 300;
/// HTTP timeout for OpenSky API requests (seconds)
pub const OPENSKY_REQUEST_TIMEOUT_SECS: u64 = 10;
/// ML service HTTP request timeout (milliseconds)
pub const ML_SERVICE_TIMEOUT_MS: u64 = 500;
/// Reconnect pause after ingestor socket error (seconds)
pub const INGESTOR_RECONNECT_DELAY_SECS: u64 = 5;
/// Retry delay between ingestor socket connection attempts (milliseconds)
pub const INGESTOR_RETRY_DELAY_MS: u64 = 500;
/// Reconnect pause after ingestor clean close (milliseconds)
pub const INGESTOR_CLOSE_RECONNECT_DELAY_MS: u64 = 500;

// ===== CHANNEL & BUFFER CAPACITIES =====

/// Async channel depth for raw ADS-B frames between tasks
pub const FRAME_CHANNEL_CAPACITY: usize = 4096;
/// WebSocket broadcast ring-buffer depth
pub const WS_BROADCAST_CAPACITY: usize = 4096;
/// OLS regression window size for clock-sync (~5 s at 10 observations/s)
pub const CLOCK_WINDOW_SIZE: usize = 50;
/// Global clock solver observation buffer capacity
pub const BUFFER_CAPACITY: usize = 30_000;
/// LSTM classifier state history window per track (steps)
pub const MAX_HISTORY_STATES: usize = 20;
/// Maximum connection retries for the ingestor socket (120 s budget at 500 ms/retry)
pub const MAX_CONNECT_RETRIES: u32 = 240;
/// Minimum sensors needed before rendering the first GDOP heatmap
pub const MIN_SENSORS_FIRST_HEATMAP: usize = 3;

// ===== SOLVER THRESHOLDS & LIMITS =====

/// Minimum sensors required to attempt semi-MLAT positioning
pub const MIN_SENSORS_SEMI_MLAT: usize = 2;
/// Minimum inter-sensor baseline for a geometrically valid solve (metres)
pub const MIN_SENSOR_BASELINE_M: f64 = 5_000.0;
/// Singular value below which the matrix is rank-deficient
pub const MIN_SINGULAR_VALUE: f64 = 1e-3;
/// Condition number above which the system is considered ill-conditioned
pub const MAX_CONDITION_NUMBER: f64 = 1e8;
/// GDOP threshold above which the pre-solver skips LM (poor geometry)
pub const PRESOLVER_GDOP_THRESHOLD: f64 = 20.0;
/// Maximum plausible MLAT solution range from sensor centroid (metres = 500 km)
pub const MAX_RANGE_M: f64 = 500_000.0;
/// Levenberg–Marquardt iteration limit before declaring no-convergence
pub const SOLVER_PATIENCE: usize = 50;
/// Fallback covariance value when the solution is degenerate
pub const LARGE_COVARIANCE: f64 = 1e12;
/// Degrees-of-freedom divisor for post-solve covariance scaling
pub const DOF_CORRECTION: f64 = 3.0;
/// Post-solve covariance trace above which the fix is discarded (~10 km uncertainty)
pub const MAX_COVARIANCE_TRACE: f64 = 100e6;
/// Default position variance used when no ADS-B altitude is available (metres²)
pub const DEFAULT_VARIANCE_M2: f64 = 90_000.0;
/// Default cruise altitude for altitude-constrained solver (metres)
pub const DEFAULT_CRUISE_ALT_M: f64 = 10_000.0;
/// Numerical stability floor for distance checks inside solvers (metres)
pub const MIN_DISTANCE_M: f64 = 1.0;
/// Minimum residual sigma clamp during weighted LM (metres)
pub const MIN_SIGMA_M: f64 = 1.0;

// ===== KALMAN FILTER =====

/// Initial position variance on filter creation (metres² = 1 km²)
pub const INITIAL_POS_VARIANCE_M2: f64 = 1_000_000.0;
/// Initial velocity variance on filter creation (m²/s² ≈ 200 m/s spread)
pub const INITIAL_VEL_VARIANCE_M2: f64 = 40_000.0;
/// Process noise modelled as constant acceleration (m/s²)
/// FIX: Increased from 1.0 to 100.0 for 6-state CV model with sparse MLAT measurements.
/// With sparse measurements (5-30s intervals), velocity uncertainty must grow by ~32 m/s
/// over 10s to cover aircraft maneuvers (±3σ = ±96 m/s). Based on Bar-Shalom (2001)
/// recommendation for CV models with irregular measurements.
pub const PROCESS_NOISE_ACCEL_MS2: f64 = 0.5;
/// Minimum measurement noise variance — floor on sensor noise (metres²)
pub const MEASUREMENT_NOISE_FLOOR_M2: f64 = 10_000.0;
/// Squared Mahalanobis gate for outlier rejection (= 15²)
pub const MAHALANOBIS_GATE_SQ: f64 = 225.0;
/// Consecutive outliers before the Kalman filter is hard-reset
pub const OUTLIER_COUNT_RESET: u32 = 3;
/// Velocity variance reset to this value after a hard filter reset (m²/s²)
pub const RESET_VEL_VARIANCE_M2: f64 = 40_000.0;
/// Maximum plausible aircraft speed — updates beyond this are rejected (m/s)
/// FIX: Reduced from 350.0 (680 knots, supersonic) to 300.0 (583 knots).
/// Commercial aircraft max cruise ~280 m/s (Mach 0.85 at FL350).
pub const MAX_SPEED_MS: f64 = 583.0;

// ===== CLOCK SYNC & QUALITY =====

/// MAD-to-Gaussian-sigma conversion factor (robust statistics)
pub const MAD_TO_SIGMA: f64 = 0.6745;
/// Minimum clock observations before an offset estimate is trusted
pub const MIN_SAMPLE_COUNT: usize = 2;
/// Minimum graph edges per sensor for the global clock solve
pub const MIN_EDGES_PER_SENSOR: usize = 2;
/// Sensor covariance above which it is classified as Unstable (nanoseconds)
pub const UNSTABLE_SENSOR_THRESHOLD_NS: f64 = 500.0;
/// MAD multiplier for IRLS outlier rejection
pub const OUTLIER_MAD_THRESHOLD: f64 = 2.0;
/// Maximum IRLS iterations for global clock solve
pub const MAX_IRLS_ITERATIONS: usize = 5;
/// IRLS convergence: stop when relative weight change drops below this
pub const IRLS_CONVERGENCE_THRESHOLD: f64 = 0.05;
/// Clock uncertainty to enter Marginal quality state (nanoseconds)
pub const MARGINAL_ENTER_NS: f64 = 350.0;
/// Clock uncertainty to exit Marginal quality state (nanoseconds)
pub const MARGINAL_EXIT_NS: f64 = 200.0;
/// Clock uncertainty to exit Unstable quality state (nanoseconds)
pub const UNSTABLE_EXIT_NS: f64 = 400.0;
/// Zero-edge solves in a row before a sensor is declared Disconnected
pub const DISCONNECTED_COUNT: u32 = 3;
/// Rolling window for drift-rate tracking (seconds)
pub const DRIFT_WINDOW_S: f64 = 60.0;

// ===== GDOP HEATMAP =====

/// GDOP ≤ this value is rated Excellent coverage
pub const GDOP_EXCELLENT: f64 = 2.0;
/// GDOP ≤ this value is rated Good coverage
pub const GDOP_GOOD: f64 = 5.0;
/// GDOP ≤ this value is rated Moderate coverage
pub const GDOP_MODERATE: f64 = 10.0;
/// GDOP ≤ this value is rated Poor coverage (above = Very Poor)
pub const GDOP_POOR: f64 = 20.0;
/// Default heatmap north bound when no sensors are known (degrees latitude)
pub const DEFAULT_BOUNDS_NORTH: f64 = 55.0;
/// Default heatmap south bound when no sensors are known (degrees latitude)
pub const DEFAULT_BOUNDS_SOUTH: f64 = 45.0;
/// Default heatmap east bound when no sensors are known (degrees longitude)
pub const DEFAULT_BOUNDS_EAST: f64 = 5.0;
/// Default heatmap west bound when no sensors are known (degrees longitude)
pub const DEFAULT_BOUNDS_WEST: f64 = -5.0;
/// Auto-fit padding added around the sensor cluster in latitude (~300 km)
pub const PADDING_LAT_DEG: f64 = 2.7;
/// Auto-fit padding added around the sensor cluster in longitude (~300 km at mid-lat)
pub const PADDING_LON_DEG: f64 = 3.5;
/// Hard clamp: maximum north bound for heatmap extent (degrees)
pub const MAX_NORTH_BOUND: f64 = 85.0;
/// Hard clamp: minimum south bound for heatmap extent (degrees)
pub const MIN_SOUTH_BOUND: f64 = -85.0;
/// Hard clamp: maximum east bound for heatmap extent (degrees)
pub const MAX_EAST_BOUND: f64 = 180.0;
/// Hard clamp: minimum west bound for heatmap extent (degrees)
pub const MIN_WEST_BOUND: f64 = -180.0;
/// Grid resolution for the planning-tool GDOP overlay (degrees per cell)
pub const PLANNING_GDOP_RESOLUTION_DEG: f64 = 0.1;
/// Minimum singular value for GDOP degeneracy check
pub const GDOP_MIN_SINGULAR_VALUE: f64 = 1e-6;

// ===== ALTITUDES & AIRSPACE =====

/// Default heatmap display altitude — FL300 (metres)
pub const DEFAULT_HEATMAP_ALTITUDE_M: f64 = 9144.0;
/// Valid planning/heatmap flight levels: FL050, FL100, FL300 (metres)
pub const ALLOWED_PLANNING_ALTITUDES: [f64; 3] = [1524.0, 3048.0, 9144.0];
/// Typical ADS-B horizontal accuracy budget (metres)
pub const ADSB_HORIZONTAL_ACCURACY_M: f64 = 500.0;
/// Altitude constraint base error below which constraint is active (metres = 250 ft)
pub const ALT_ERROR_BASE_M: f64 = 76.2;
/// Altitude constraint error growth rate with vertical speed (m/s = 70 ft/s)
pub const ALT_DEGRADATION_RATE: f64 = 21.3;
/// Altitude constraint is disabled above this total error (metres)
pub const ALT_CONSTRAINT_MAX_ERROR_M: f64 = 926.0;

// ===== ADS-B DECODING =====

/// ICAO Doc 9684 CRC-24 generator polynomial
pub const CRC24_POLY: u32 = 0xFFF409;
/// Latitude above which the NL lookup returns 1 (polar region, degrees)
pub const NL_POLAR_LAT: f64 = 87.0;
/// Gillham altitude code: 500 ft multiplier
pub const GILLHAM_ALT_MULTIPLIER: i32 = 500;
/// Gillham altitude code: 100 ft resolution step
pub const GILLHAM_ALT_RESOLUTION: i32 = 100;
/// Gillham altitude code: 1300 ft offset bias
pub const GILLHAM_ALT_OFFSET: i32 = 1300;
/// Q-bit altitude encoding: 25 ft resolution step
pub const QBIT_ALT_RESOLUTION: f64 = 25.0;
/// Q-bit altitude encoding: 1000 ft bias offset
pub const QBIT_ALT_OFFSET: f64 = 1000.0;
/// Maximum age of a CPR even/odd pair before it is discarded (nanoseconds = 10 s)
pub const MAX_PAIR_AGE_NS: u64 = 10_000_000_000;
/// CPR coordinate resolution: 2^17 zones per 360°
pub const CPR_BITS: f64 = 131_072.0;
/// CPR even-frame zone latitude spacing (degrees)
pub const CPR_DLAT_EVEN: f64 = 360.0 / 60.0;
/// CPR odd-frame zone latitude spacing (degrees)
pub const CPR_DLAT_ODD: f64 = 360.0 / 59.0;
/// CPR latitude normalisation threshold (degrees)
pub const LAT_NORM_THRESHOLD: f64 = 270.0;
/// CPR decoded latitude sanity bound (degrees)
pub const LAT_VALIDITY_THRESHOLD: f64 = 90.0;

// ===== INGESTOR =====

/// Unix domain socket path for the ingestor process
pub const INGESTOR_SOCKET_PATH: &str = "/tmp/locus/ingestor.sock";

// ===== VIRTUAL SENSORS =====

/// First ID reserved for virtual sensors — avoids collision with real hardware IDs
pub const VIRTUAL_SENSOR_ID_START: u32 = 1_000_000;
/// Maximum allowed altitude for a virtual sensor placement — FL500 (metres)
pub const MAX_VIRTUAL_SENSOR_ALTITUDE_M: f64 = 15240.0;

// ===== SEMI-MLAT =====

/// Maximum prior covariance trace before the semi-MLAT prior is discarded (metres² = 5 km std)
pub const MAX_PRIOR_TRACE: f64 = 25_000_000.0;
/// Mahalanobis innovation gate for semi-MLAT update acceptance (σ)
pub const INNOVATION_GATE_MAHALANOBIS: f64 = 15.0;
/// Direction cosine difference below which a TDOA constraint is considered weak
pub const WEAK_CONSTRAINT_THRESHOLD: f64 = 0.1;
/// SDOP above which a semi-MLAT fix is rated Poor quality (metres)
pub const SDOP_THRESHOLD_M: f64 = 500.0;
/// Maximum LM iterations for semi-MLAT solver
pub const SEMI_MLAT_MAX_ITERATIONS: usize = 50;

// ===== SPOOF DETECTION =====

/// MLAT vs ADS-B horizontal divergence above which GPS spoofing is flagged (metres = 2 km)
pub const SPOOF_THRESHOLD_M: f64 = 2_000.0;

// ===== MAIN / CORRELATOR =====

/// ADS-B typical horizontal position accuracy used when no covariance is available (metres)
pub const ADSB_DEFAULT_ACCURACY_M: f64 = 500.0;
