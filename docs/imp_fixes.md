Locus Implementation Plan — Correction Patches
Feed this document to the agent tasked with updating the implementation plan. Each section is self-contained and references the exact location in the plan that needs changing.

Patch 1: Silent Error Drops via unwrap_or_default()
Location: rust-backend/src/ingestor.rs — RawFrame::raw_bytes() method, and all call sites in correlator.rs
Problem:
The current implementation silently discards malformed frames with no log output:
rust// CURRENT — WRONG
pub fn raw_bytes(&self) -> Vec<u8> {
    hex::decode(&self.raw_modes_hex).unwrap_or_default()
}
If the Go ingestor emits malformed hex (odd-length string, null bytes, truncated frame), hex::decode returns Err(...) and unwrap_or_default() substitutes an empty Vec<u8>. The empty vec propagates to the correlator where bytes.len() < 7 silently drops the frame. No log, no counter, no indication that data is being lost. This is undetectable during debugging and will manifest as mysteriously low MLAT output.
Fix — ingestor.rs:
rust// CORRECTED
pub fn raw_bytes(&self) -> Result<Vec<u8>, hex::FromHexError> {
    hex::decode(&self.raw_modes_hex)
}
Fix — correlator.rs, inside CorrelationKey::from_frame and everywhere raw_bytes() is called:
rust// CORRECTED — every call site must handle the error explicitly
let bytes = match frame.raw_bytes() {
    Ok(b) => b,
    Err(e) => {
        tracing::warn!(
            sensor_id = frame.sensor_id,
            raw_hex = %frame.raw_modes_hex,
            error = %e,
            "hex decode failed — dropping frame"
        );
        return None;
    }
};

if bytes.len() < 7 {
    tracing::debug!(
        sensor_id = frame.sensor_id,
        len = bytes.len(),
        "frame too short — dropping"
    );
    return None;
}
Additional requirement: Add a per-sensor drop counter using an Arc<DashMap<i64, u64>> passed into the correlator. Increment on every dropped frame. Expose this counter in the sensor_health WebSocket broadcast so drops are visible in the UI.
Applies to all unwrap(), unwrap_or_default(), unwrap_or(vec![]) calls throughout the codebase. Every such call on a data path must be replaced with explicit error handling and a tracing::warn! or tracing::debug! log. Reserve unwrap() only for cases that are truly programmer errors (e.g. mutex poisoning), not data errors.

Patch 2: Correlator Time-Bucket Bug + SHA256 Replacement
Location: rust-backend/src/correlator.rs — CorrelationKey struct and Correlator::ingest()
Problem A — Time bucket splits valid groups:
The current key includes time_bucket = floor(timestamp_ns / 5_000_000). Since each sensor's timestamp is its own local reception time (not a shared reference), two sensors receiving the same aircraft message will have different timestamps by design — that difference is the TDOA being measured. If those timestamps straddle a 5ms bucket boundary, they hash to different keys and are never correlated. For sensors 1,800km apart (London–Vienna), propagation spread is ~6ms, meaning every single message will split across buckets. This produces zero MLAT output with no error.
Problem B — SHA256 is cryptographic overkill on a hot path:
SHA256 on 7–14 bytes costs ~300ns per call. At 20,000 frames/second this wastes 6ms/second of CPU for no security benefit. FxHash on the same data costs ~3ns — 100x faster.
Fix — replace CorrelationKey entirely:
rust// CORRECTED — remove time_bucket, replace SHA256 with FxHash
// Add to Cargo.toml: rustc-hash = "1.1"
use rustc_hash::FxHasher;
use std::hash::{Hash, Hasher};

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct CorrelationKey {
    pub icao24: [u8; 3],    // fixed array, no heap allocation
    pub content_hash: u64,  // FxHash of raw frame bytes
}

impl CorrelationKey {
    pub fn from_frame(frame: &RawFrame) -> Option<Self> {
        let bytes = frame.raw_bytes().ok()?;
        if bytes.len() < 7 { return None; }

        // Only correlate DF17/DF18 (ADS-B extended squitter)
        let df = (bytes[0] >> 3) & 0x1F;
        if df != 17 && df != 18 { return None; }

        let icao24 = [bytes[1], bytes[2], bytes[3]];

        let mut hasher = FxHasher::default();
        bytes.hash(&mut hasher);
        let content_hash = hasher.finish();

        Some(CorrelationKey { icao24, content_hash })
    }
}
Fix — update Correlator to use time-based eviction instead of time-based keying:
rustpub struct CorrelationGroup {
    pub key: CorrelationKey,
    pub frames: Vec<RawFrame>,
    pub first_seen_ns: u64,   // timestamp of first frame received
}

pub struct Correlator {
    groups: HashMap<CorrelationKey, CorrelationGroup>,
    min_sensors: usize,   // 3
    window_ns: u64,       // 50_000_000 (50ms) — larger than max EU propagation spread of 6ms
}

impl Correlator {
    pub fn ingest(&mut self, frame: RawFrame) -> Option<CorrelationGroup> {
        let key = CorrelationKey::from_frame(&frame)?;
        let now_ns = frame.timestamp_ns();

        let group = self.groups.entry(key.clone()).or_insert_with(|| {
            CorrelationGroup {
                key: key.clone(),
                frames: Vec::new(),
                first_seen_ns: now_ns,
            }
        });

        // Deduplicate by sensor_id — one observation per sensor per group
        let already_seen = group.frames.iter()
            .any(|f| f.sensor_id == frame.sensor_id);
        if !already_seen {
            group.frames.push(frame);
        }

        // Emit immediately when minimum sensor count reached
        if group.frames.len() >= self.min_sensors {
            return Some(self.groups.remove(&key).unwrap());
        }

        None
    }

    // Call every 10ms from the main pipeline loop
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
                false
            } else {
                true
            }
        });

        emitted
    }
}
Add to Cargo.toml:
tomlrustc-hash = "1.1"
Remove from Cargo.toml:
tomlsha2 = "0.10"   # DELETE THIS LINE
hex = "0.4"     # keep hex for the Go→Rust IPC decoding

Patch 3: MLAT Solver — Missing Initial Guess
Location: rust-backend/src/mlat_solver.rs — solve() function
Problem:
The plan specifies the LM solver call but never defines the initial guess parameter. Levenberg-Marquardt is highly sensitive to initialization — a bad initial guess causes divergence or convergence to a physically impossible local minimum. Without an explicit initial guess strategy, the solver will fail silently on first-seen aircraft (returning None) and may produce wrong positions when the guess is arbitrary.
Fix — add initial guess logic to solve():
rustpub fn solve(input: &MlatInput, prior: Option<Vector3<f64>>) -> Option<MlatSolution> {
    let initial_guess = match prior {
        // Use last known ECEF position from Kalman filter if available
        Some(ecef) => ecef,

        // First observation: use centroid of all sensors + typical cruise altitude
        None => {
            let n = input.sensor_positions.len() as f64;
            let centroid = input.sensor_positions.iter()
                .fold(Vector3::zeros(), |acc, &(x, y, z)| {
                    acc + Vector3::new(x, y, z)
                }) / n;

            // Offset centroid outward from Earth center by ~10,000m
            // (typical cruise altitude above the sensor plane)
            let centroid_norm = centroid.norm();
            let earth_radius = 6_371_000.0_f64;
            let scale = (earth_radius + 10_000.0) / centroid_norm;
            centroid * scale
        }
    };

    // Pass initial_guess to the argmin Executor:
    let result = Executor::new(problem, solver)
        .configure(|state| {
            state
                .param(initial_guess)
                .max_iters(100)
                .target_cost(1e-6)
        })
        .run()
        .ok()?;

    // ... rest of solve()
}
Update KalmanRegistry::update() to pass the current Kalman ECEF position as the prior:
rust// In main pipeline, before calling solve():
let prior_ecef = kalman_registry
    .get_ecef(&icao24)  // returns Option<Vector3<f64>>
    .map(|state| Vector3::new(state[0], state[1], state[2]));

let solution = mlat_solver::solve(&input, prior_ecef)?;

Patch 4: GDOP Formula — Remove GPS Clock Bias Column
Location: rust-backend/src/mlat_solver.rs — compute_gdop() function
Problem:
The current GDOP formula uses a 4-column geometry matrix (Nx4) with a -1 fourth column, which is the GPS clock bias term. GPS needs this because GPS receivers solve for an unknown clock offset as a fourth unknown alongside (x,y,z). MLAT does not — TDOA differencing eliminates the transmission time, so the unknowns are only (x,y,z). The GPS GDOP formula applied to MLAT produces incorrect dilution-of-precision values, causing either over-rejection (discarding valid solutions) or under-rejection (accepting poor-geometry solutions) depending on sensor configuration.
Fix — replace compute_gdop() with correct MLAT HDOP:
rustfn compute_gdop(
    solution_ecef: &Vector3<f64>,
    sensors: &[Vector3<f64>],
) -> f64 {
    let n = sensors.len();

    // MLAT geometry matrix: Nx3 (no clock bias column)
    // Each row: unit vector from solution to sensor
    let mut h = nalgebra::DMatrix::<f64>::zeros(n, 3);

    for (i, s) in sensors.iter().enumerate() {
        let diff = solution_ecef - s;
        let r = diff.norm();
        if r < 1.0 { return f64::INFINITY; }  // degenerate: solution at sensor location
        h[(i, 0)] = diff[0] / r;
        h[(i, 1)] = diff[1] / r;
        h[(i, 2)] = diff[2] / r;
        // NO fourth column
    }

    let ht_h = h.transpose() * &h;
    match ht_h.try_inverse() {
        Some(inv) => inv.trace().sqrt(),
        None => f64::INFINITY,  // singular matrix — collinear sensors
    }
}
The GDOP threshold check (gdop > 5.0) remains unchanged — the threshold value is still appropriate, only the computation was wrong.

Patch 5: Clock Sync — Replace EMA With Windowed Median + Outlier Gate
Location: rust-backend/src/clock_sync.rs — ClockSyncEngine struct and update_from_beacon()
Problem:
EMA with fixed alpha=0.1 has two failure modes. First, slow convergence — it takes ~45 observations to reach 99% of the true value, meaning the first minute of operation uses inaccurate clock corrections. Second, spike sensitivity — a single multipath observation (common near terrain and buildings) biases the estimate for dozens of subsequent observations. One bad reading at 5000ns when the true offset is 100ns shifts the EMA to ~580ns, causing ~145 meters of systematic position error until it recovers.
Fix — replace EMA with windowed median:
rustuse std::collections::{HashMap, VecDeque};

const WINDOW_SIZE: usize = 50;  // ~5 seconds at 10 beacon observations/second
const C_M_PER_NS: f64 = 0.299_792_458;

pub struct ClockSyncEngine {
    // Store raw observations per sensor pair — compute median on demand
    samples: HashMap<(i64, i64), VecDeque<f64>>,
}

impl ClockSyncEngine {
    pub fn new() -> Self {
        Self { samples: HashMap::new() }
    }

    pub fn update_from_beacon(
        &mut self,
        sensor_i: i64, pos_i: (f64, f64, f64),
        sensor_j: i64, pos_j: (f64, f64, f64),
        t_i_ns: u64, t_j_ns: u64,
        beacon_ecef: (f64, f64, f64),
    ) {
        let dist_i = ecef_dist(pos_i, beacon_ecef);
        let dist_j = ecef_dist(pos_j, beacon_ecef);
        let expected_tdoa_ns = (dist_i - dist_j) / C_M_PER_NS;
        let observed_tdoa_ns = t_i_ns as f64 - t_j_ns as f64;
        let error_ns = observed_tdoa_ns - expected_tdoa_ns;

        // Outlier gate: reject if >3x current spread from current median
        // Prevents multipath spikes from contaminating the window
        let buf = self.samples
            .entry((sensor_i, sensor_j))
            .or_insert_with(VecDeque::new);

        if buf.len() >= 5 {
            let current_median = median_of_deque(buf);
            let current_spread = spread_of_deque(buf);  // median absolute deviation
            if (error_ns - current_median).abs() > 3.0 * current_spread.max(50.0) {
                tracing::debug!(
                    "clock obs rejected: error={:.1}ns median={:.1}ns spread={:.1}ns",
                    error_ns, current_median, current_spread
                );
                return;
            }
        }

        buf.push_back(error_ns);
        if buf.len() > WINDOW_SIZE {
            buf.pop_front();
        }

        tracing::debug!(
            "clock[{}→{}] offset={:.1}ns n={}",
            sensor_i, sensor_j,
            self.get_offset(sensor_i, sensor_j),
            buf.len()
        );
    }

    pub fn get_offset(&self, sensor_i: i64, sensor_j: i64) -> f64 {
        match self.samples.get(&(sensor_i, sensor_j)) {
            None => 0.0,
            Some(buf) if buf.is_empty() => 0.0,
            Some(buf) => median_of_deque(buf),
        }
    }

    pub fn is_converged(&self, sensor_i: i64, sensor_j: i64) -> bool {
        self.samples
            .get(&(sensor_i, sensor_j))
            .map(|buf| buf.len() >= WINDOW_SIZE / 2)
            .unwrap_or(false)
    }
}

fn median_of_deque(buf: &VecDeque<f64>) -> f64 {
    let mut v: Vec<f64> = buf.iter().copied().collect();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mid = v.len() / 2;
    if v.len() % 2 == 0 {
        (v[mid - 1] + v[mid]) / 2.0
    } else {
        v[mid]
    }
}

fn spread_of_deque(buf: &VecDeque<f64>) -> f64 {
    // Median Absolute Deviation — robust measure of spread
    let med = median_of_deque(buf);
    let mut deviations: Vec<f64> = buf.iter().map(|x| (x - med).abs()).collect();
    deviations.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mid = deviations.len() / 2;
    deviations[mid]
}
Update export_offsets() to also export convergence status and sample count per pair so the UI can show which sensor pairs have reliable calibration versus which are still converging.

Patch 6: Kalman Filter — ECEF State Vector
Location: rust-backend/src/kalman.rs — AircraftKalman struct, predict(), update(), process_noise()
Problem:
The state vector [lat, lon, alt, vel_lat, vel_lon, vel_alt] mixes degrees and meters. The process noise matrix applies sigma_a = 0.1 m/s^2 equally to all components. In degrees, this translates to 0.1 * 111,320 ≈ 11,132 m/s^2 of implied acceleration noise for lat/lon — physically impossible and 100,000x too large. The measurement noise R = diag(1e-8, 1e-8, 100) in degrees^2/m^2 sets lat/lon measurement uncertainty to ~11 meters RMS, far more optimistic than MLAT's actual ~100-500m accuracy. The combined effect: Kalman gain approaches 1.0 (no smoothing) and every noisy MLAT fix passes through directly to the map, causing aircraft to visually teleport.
Fix — replace entire AircraftKalman with ECEF state vector:
rustuse nalgebra::{Matrix6, Vector6, Matrix3x6};

pub struct AircraftKalman {
    pub icao24: String,
    // State: [x, y, z, vx, vy, vz] — ALL in meters and meters/second (ECEF)
    pub x: Vector6<f64>,
    pub p: Matrix6<f64>,
    pub last_update_ns: u64,
}

impl AircraftKalman {
    pub fn new(icao24: String, x_ecef: f64, y_ecef: f64, z_ecef: f64) -> Self {
        let x = Vector6::new(x_ecef, y_ecef, z_ecef, 0.0, 0.0, 0.0);
        // Initial uncertainty: 1000m position, 50m/s velocity
        let mut p = Matrix6::zeros();
        p[(0,0)] = 1_000_000.0;  // 1000m^2
        p[(1,1)] = 1_000_000.0;
        p[(2,2)] = 1_000_000.0;
        p[(3,3)] = 2_500.0;      // 50 m/s ^2
        p[(4,4)] = 2_500.0;
        p[(5,5)] = 2_500.0;
        Self { icao24, x, p, last_update_ns: 0 }
    }

    pub fn predict(&mut self, dt: f64) {
        let f = Self::transition_matrix(dt);
        let q = Self::process_noise(dt);
        self.x = &f * &self.x;
        self.p = &f * &self.p * f.transpose() + q;
    }

    // Input: ECEF position in meters (convert from WGS84 before calling)
    pub fn update(&mut self, x_m: f64, y_m: f64, z_m: f64, accuracy_m: f64) {
        let h = Matrix3x6::new(
            1.,0.,0.,0.,0.,0.,
            0.,1.,0.,0.,0.,0.,
            0.,0.,1.,0.,0.,0.,
        );

        // Measurement noise in meters^2
        // accuracy_m comes from MLAT solver (GDOP-derived)
        // Floor at 100m^2 (10m RMS) — MLAT cannot be more accurate than this
        let r_val = accuracy_m.powi(2).max(10_000.0);  // min 100m RMS
        let r = nalgebra::Matrix3::from_diagonal(
            &nalgebra::Vector3::new(r_val, r_val, r_val * 4.0)
            // Vertical (z) is ~2x less accurate in MLAT geometry
        );

        let z = nalgebra::Vector3::new(x_m, y_m, z_m);
        let y = z - &h * &self.x;
        let s = &h * &self.p * h.transpose() + r;
        let k = &self.p * h.transpose() *
            s.try_inverse().unwrap_or(nalgebra::Matrix3::zeros());
        self.x = &self.x + &k * y;
        self.p = (Matrix6::identity() - k * &h) * &self.p;
    }

    // Output conversion: call only when sending to WebSocket / frontend
    pub fn position_wgs84(&self) -> (f64, f64, f64) {
        ecef_to_wgs84(self.x[0], self.x[1], self.x[2])
    }

    pub fn velocity_ms(&self) -> (f64, f64, f64) {
        (self.x[3], self.x[4], self.x[5])
    }

    fn transition_matrix(dt: f64) -> Matrix6<f64> {
        let mut f = Matrix6::identity();
        f[(0,3)] = dt; f[(1,4)] = dt; f[(2,5)] = dt;
        f
    }

    fn process_noise(dt: f64) -> Matrix6<f64> {
        // sigma_a = 2.0 m/s^2 — realistic for commercial aircraft
        // All components in meters — no unit ambiguity
        let sigma_a2 = 4.0_f64;  // sigma_a^2
        let dt2 = dt * dt / 2.0;
        let dt3 = dt * dt * dt / 3.0;
        let mut q = Matrix6::zeros();
        // Position variance
        q[(0,0)] = dt3 * sigma_a2;
        q[(1,1)] = dt3 * sigma_a2;
        q[(2,2)] = dt3 * sigma_a2;
        // Velocity variance
        q[(3,3)] = dt * sigma_a2;
        q[(4,4)] = dt * sigma_a2;
        q[(5,5)] = dt * sigma_a2;
        // Cross terms
        q[(0,3)] = dt2 * sigma_a2; q[(3,0)] = dt2 * sigma_a2;
        q[(1,4)] = dt2 * sigma_a2; q[(4,1)] = dt2 * sigma_a2;
        q[(2,5)] = dt2 * sigma_a2; q[(5,2)] = dt2 * sigma_a2;
        q
    }
}
Update KalmanRegistry::update() — call site must convert WGS84 MLAT output to ECEF before passing to update(), and convert back to WGS84 for AircraftState:
rust// In KalmanRegistry::update(), before calling entry.update():
let (x_ecef, y_ecef, z_ecef) = wgs84_to_ecef(mlat.lat, mlat.lon, mlat.alt_m);
entry.update(x_ecef, y_ecef, z_ecef, mlat.accuracy_m);

// When building AircraftState for WebSocket broadcast:
let (lat, lon, alt_m) = entry.position_wgs84();

Patch 7: Decouple ML Model Training From Docker Build
Location: ml-service/Dockerfile and docker-compose.yml
Problem:
Training RUN python train.py --epochs 15 --output model.pt inside the Dockerfile bakes a randomly-seeded model into the image at build time. Every docker compose build produces a different model with non-deterministic performance. Build times become unpredictable (~60-120 seconds on CPU). The model cannot be inspected or version-controlled. If training fails mid-build, the entire image build fails with no useful error recovery.
Fix — three-part change:
Part A: Remove training from Dockerfile:
dockerfile# CORRECTED ml-service/Dockerfile
FROM python:3.11-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY . .
EXPOSE 8000
# NO training step — model is either pre-trained or loaded at startup
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
Part B: Add a train.sh script to run once manually before first deploy:
bash#!/usr/bin/env bash
# Run once to generate model.pt before deploying
# Output is committed to the repo (it's a ~1MB file, acceptable)
set -e
cd ml-service
python train.py --epochs 20 --output model.pt
echo "model.pt generated. Commit this file before deploying."
Part C: Update main.py startup to handle missing model gracefully:
python@app.on_event("startup")
async def startup():
    if not MODEL_PATH.exists():
        logger.warning(
            f"model.pt not found at {MODEL_PATH}. "
            f"Run train.sh to generate it. "
            f"/classify will return 'unknown' until model is loaded."
        )
        return
    load_model()
    logger.info(f"Model loaded: {MODEL_PATH} ({MODEL_PATH.stat().st_size} bytes)")
Part D: Add model.pt to the Docker image via COPY (not RUN):
dockerfile# If model.pt exists in the build context, copy it in
# If not, the service starts in degraded mode and logs a warning
COPY model.pt* ./
The model.pt* glob means the COPY succeeds whether or not the file exists (Docker 1.23+). The service degrades gracefully rather than failing to build.
Update .gitignore — remove ml-service/*.pt from gitignore. The trained model should be committed so the build is reproducible.

Patch 8: Unix Socket IPC Race Condition Between Go and Rust
Location: go-ingestor/main.go and rust-backend/src/ingestor.rs
Problem:
The Go ingestor calls os.Remove(socketPath) on startup to clean stale sockets, then calls net.Listen("unix", socketPath) to create a new one. The Rust backend uses connect_with_retry() with 500ms delays. Three race conditions exist:
Race 1 — Startup ordering: Rust starts connecting before Go has created the socket. The retry loop handles this but only if Go starts within MAX_RETRIES * RETRY_DELAY_MS = 15 seconds. If Go takes longer (slow network for SDK initialization), Rust gives up and exits.
Race 2 — Go restart mid-stream: If Go crashes and restarts, it calls os.Remove(socketPath) which deletes the socket file while Rust has an open connection to it. Rust's read loop gets UnexpectedEof or BrokenPipe which currently propagates as an unhandled error, crashing the ingestor task.
Race 3 — Write to disconnected client: Go writes JSON to all connected clients via fmt.Fprintf(conn, ...). If Rust has disconnected (e.g., restarting), this write panics or returns an error that crashes the Go goroutine, potentially taking down the entire ingestor.
Fix — Go side (go-ingestor/main.go):
go// Use a mutex-protected client list, remove clients on write error
var (
    clientsMu sync.Mutex
    clients   = make(map[net.Conn]struct{})
)

// Accept loop (in goroutine):
go func() {
    for {
        conn, err := ln.Accept()
        if err != nil {
            log.Printf("accept error: %v", err)
            continue
        }
        clientsMu.Lock()
        clients[conn] = struct{}{}
        clientsMu.Unlock()
        log.Printf("Rust backend connected")
    }
}()

// Broadcast to all clients (replace fmt.Fprintf direct call):
func broadcast(data []byte) {
    clientsMu.Lock()
    defer clientsMu.Unlock()
    for conn := range clients {
        conn.SetWriteDeadline(time.Now().Add(100 * time.Millisecond))
        _, err := fmt.Fprintf(conn, "%s\n", data)
        if err != nil {
            log.Printf("client write error, removing: %v", err)
            conn.Close()
            delete(clients, conn)
        }
    }
}
Fix — Rust side (rust-backend/src/ingestor.rs):
rust// Increase retry budget and add infinite reconnect on disconnect
const MAX_CONNECT_RETRIES: u32 = 120;   // 60 seconds total
const RETRY_DELAY_MS: u64 = 500;

pub async fn run_ingestor(
    mock: bool,
    tx: tokio::sync::mpsc::Sender<RawFrame>,
) -> anyhow::Result<()> {
    if mock {
        return run_mock_ingestor(tx).await;
    }

    // Outer loop: reconnect on any disconnect
    loop {
        match connect_with_retry().await {
            Err(e) => {
                tracing::error!("Could not connect to Go ingestor: {e}. Retrying in 5s.");
                tokio::time::sleep(Duration::from_secs(5)).await;
                continue;
            }
            Ok(stream) => {
                tracing::info!("Connected to Go ingestor socket");
                let result = read_loop(stream, &tx).await;
                tracing::warn!("Go ingestor connection lost: {result:?}. Reconnecting...");
                // Brief pause before reconnect to avoid tight loop on persistent failure
                tokio::time::sleep(Duration::from_millis(500)).await;
            }
        }
    }
}

async fn read_loop(
    stream: tokio::net::UnixStream,
    tx: &tokio::sync::mpsc::Sender<RawFrame>,
) -> anyhow::Result<()> {
    let reader = tokio::io::BufReader::new(stream);
    let mut lines = tokio::io::AsyncBufReadExt::lines(reader);

    while let Some(line) = lines.next_line().await? {
        match serde_json::from_str::<RawFrame>(&line) {
            Ok(frame) => {
                if tx.send(frame).await.is_err() {
                    anyhow::bail!("downstream receiver dropped — shutting down ingestor");
                }
            }
            Err(e) => {
                tracing::warn!(error = %e, line_len = line.len(), "JSON parse error — dropping frame");
            }
        }
    }

    Ok(())
}
Fix — docker-compose.yml startup ordering:
yaml# Add depends_on with condition to rust-backend
rust-backend:
  depends_on:
    ml-service:
      condition: service_healthy
    go-ingestor:
      condition: service_started  # not service_healthy — Go has no health endpoint
  # Increase startup grace period
  restart: unless-stopped

# Add restart policy to go-ingestor so Docker restarts it on crash
go-ingestor:
  restart: unless-stopped
  # go-ingestor is in 'live' profile — this only applies in live mode
The combination of Go's protected client list, Rust's outer reconnect loop, and Docker's restart policies means any crash in either process results in automatic reconnection within 1-2 seconds rather than requiring manual intervention.
