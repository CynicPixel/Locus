# Locus — Implementation Plan

**Project**: Locus MLAT Aircraft Positioning System
**Hackathon**: 4DSky MLAT Hackathon by Neuron Innovations / Hedera
**Lead Architect**: Locus team
**Working directory**: `/home/psychopunk_sage/dev/tools/Locus/`

---

## Dependency Graph

```
4DSky SDK (Go binary @ port 61339)
         │
         │ JSON lines → Unix socket (/tmp/locus/ingestor.sock)
         ▼
  go-ingestor/main.go
         │
         │ Unix socket client (Rust connects with retry)
         ▼
  rust-backend (tokio async runtime)
    ├── ingestor.rs          ← connects to Go Unix socket OR generates mock frames
    │        │
    │        ▼
    ├── correlator.rs        ← groups frames by (ICAO24, content_hash) with time-based eviction
    │        │
    │        ▼
    ├── clock_sync.rs        ← corrects TDOA using beacon aircraft
    │        │ corrected TDOAs
    │        ▼
    ├── mlat_solver.rs       ← Levenberg-Marquardt via argmin
    │        │ raw (lat,lon,alt) + GDOP
    │        ▼
    ├── kalman.rs            ← EKF per ICAO24, smoothed track
    │        │
    │        ├──────────────────────────────────────────────────►
    │        │                                          anomaly_client.rs
    │        │                                                   │ POST /classify
    │        │                                                   ▼
    │        │                                          ml-service (FastAPI + LSTM)
    │        │                                                   │ anomaly_label
    │        │ ◄─────────────────────────────────────────────────┘
    │        ▼
    ├── spoof_detector.rs    ← compare MLAT pos vs ADS-B claimed pos
    │        │ enriched AircraftState
    │        ▼
    └── ws_server.rs         ← WebSocket broadcast @ port 9001
               │
               │ JSON frames
               ▼
        frontend/index.html  ← Leaflet.js map + Chart.js + Vanilla JS
        (nginx @ port 3000 in Docker)
```

---

## Timeline Estimate

| Phase | Description | Estimated Hours |
|-------|-------------|-----------------|
| 0 | Project scaffold + Docker foundation | 2–3 h |
| 1 | Data pipeline (Go → Rust) + mock mode | 3–4 h |
| 2 | Core MLAT engine + live map dots | 5–7 h |
| 3 | Self-calibrating clock sync + sensor health UI | 3–4 h |
| 4 | Spoof detector + AI anomaly classifier | 4–6 h |
| 5 | Full polish + production Docker Compose | 2–3 h |
| **Total** | | **19–27 h** |

---

## Port Assignments (Reference)

| Service | Port | Protocol |
|---------|------|----------|
| Go ingestor (P2P buyer) | 61339 | TCP (libp2p) |
| Rust backend WebSocket | 9001 | WS |
| Python ML service | 8000 | HTTP |
| Frontend (nginx, Docker only) | 3000 | HTTP |

---

## Phase 0 — Project Scaffold + Docker Foundation

### Goal
Establish the full directory skeleton, initialize all three language runtimes, and prove that `docker compose up` starts four containers without errors.

### Visible Outcome
- Terminal: four services start with no crash loops
- Browser at `http://localhost:3000`: white page reading "Locus — Connecting..."
- `curl http://localhost:8000/health` returns `{"status":"ok"}`
- Rust container logs: "Locus backend starting"

### Estimated time: 2–3 hours

---

### Step 0.1 — Create Directory Structure

```bash
cd /home/psychopunk_sage/dev/tools/Locus

mkdir -p Locus/go-ingestor
mkdir -p Locus/rust-backend/src
mkdir -p Locus/ml-service
mkdir -p Locus/frontend
mkdir -p Locus/docs
```

Verify:
```bash
find Locus -type d
```

Expected output:
```
Locus
Locus/go-ingestor
Locus/rust-backend
Locus/rust-backend/src
Locus/ml-service
Locus/frontend
Locus/docs
```

---

### Step 0.2 — Copy and Configure Go Ingestor

The existing SDK lives at `4dsky-mlat-challenge/`. Copy the structured parser (the `.bak` file) as the base — it already knows how to parse the binary wire format.

```bash
# Copy the structured parser as the new main.go
cp 4dsky-mlat-challenge/main-half-4dsky.go.bak Locus/go-ingestor/main.go

# Copy module files — we will update the module name in a later phase
cp 4dsky-mlat-challenge/go.mod Locus/go-ingestor/go.mod
cp 4dsky-mlat-challenge/go.sum Locus/go-ingestor/go.sum

# Copy env example
cp 4dsky-mlat-challenge/.buyer-env.example Locus/go-ingestor/.buyer-env.example
```

Update the module declaration in `Locus/go-ingestor/go.mod` — change line 1 from `module quickstart` to `module locus-ingestor`. The rest of the file stays identical.

Create `.gitignore` for the ingestor:

**File**: `Locus/go-ingestor/.gitignore`
```
.buyer-env
.seller-env
```

---

### Step 0.3 — Initialize Rust Workspace

```bash
cd /home/psychopunk_sage/dev/tools/Locus/Locus/rust-backend

cargo init --name locus-backend
```

This creates `src/main.rs` and `Cargo.toml`. Replace the generated `Cargo.toml` with:

**File**: `Locus/rust-backend/Cargo.toml`
```toml
[package]
name = "locus-backend"
version = "0.1.0"
edition = "2021"

[[bin]]
name = "locus-backend"
path = "src/main.rs"

[dependencies]
# Async runtime
tokio = { version = "1", features = ["full"] }

# Serialization
serde = { version = "1", features = ["derive"] }
serde_json = "1"

# WebSocket server
tokio-tungstenite = { version = "0.21", features = ["native-tls"] }
futures-util = "0.3"

# HTTP client (for Python ML service)
reqwest = { version = "0.11", features = ["json"] }

# Math / MLAT
nalgebra = "0.33"
argmin = "0.9"
argmin-math = { version = "0.4", features = ["nalgebra_latest"] }

# Geographic utilities
geo = "0.28"

# Logging
tracing = "0.1"
tracing-subscriber = { version = "0.3", features = ["env-filter"] }

# Error handling
thiserror = "1"
anyhow = "1"

# Hashing (for message correlation key)
rustc-hash = "1.1"
hex = "0.4"
dashmap = "5"

# CLI flags
clap = { version = "4", features = ["derive"] }

# Random number generation (mock mode)
rand = "0.8"
rand_distr = "0.4"

# Time
chrono = { version = "0.4", features = ["serde"] }

[profile.release]
opt-level = 3
lto = true
```

**File**: `Locus/rust-backend/src/main.rs` (Phase 0 stub)
```rust
use clap::Parser;
use tracing::info;

#[derive(Parser, Debug)]
#[command(name = "locus-backend", about = "Locus MLAT Engine")]
pub struct Cli {
    /// Run in mock data mode (skip Unix socket connection to Go ingestor)
    #[arg(long, env = "MOCK_DATA")]
    pub mock: bool,

    /// WebSocket bind address
    #[arg(long, default_value = "0.0.0.0:9001")]
    pub ws_addr: String,

    /// Python ML service URL
    #[arg(long, default_value = "http://localhost:8000")]
    pub ml_service_url: String,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::from_default_env()
                .add_directive("locus_backend=debug".parse()?),
        )
        .init();

    let cli = Cli::parse();

    info!("Locus backend starting");
    info!(mock = cli.mock, ws_addr = %cli.ws_addr, "Configuration loaded");

    // Phase 0: just confirm startup
    tokio::signal::ctrl_c().await?;
    info!("Shutting down");
    Ok(())
}
```

Create stub source files so the module tree compiles in later phases:

```bash
touch Locus/rust-backend/src/ingestor.rs
touch Locus/rust-backend/src/correlator.rs
touch Locus/rust-backend/src/clock_sync.rs
touch Locus/rust-backend/src/mlat_solver.rs
touch Locus/rust-backend/src/kalman.rs
touch Locus/rust-backend/src/coords.rs
touch Locus/rust-backend/src/spoof_detector.rs
touch Locus/rust-backend/src/anomaly_client.rs
touch Locus/rust-backend/src/ws_server.rs
```

Verify the stub compiles:
```bash
cd /home/psychopunk_sage/dev/tools/Locus/Locus/rust-backend
cargo build 2>&1 | tail -5
```

---

### Step 0.4 — Python ML Service Skeleton

```bash
cd /home/psychopunk_sage/dev/tools/Locus/Locus/ml-service
python3 -m venv .venv
source .venv/bin/activate
pip install fastapi uvicorn torch numpy pandas requests
pip freeze > requirements.txt
```

**File**: `Locus/ml-service/main.py` (Phase 0 stub)
```python
import logging
from fastapi import FastAPI

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("locus-ml")

app = FastAPI(title="Locus ML Service", version="0.1.0")


@app.get("/health")
async def health():
    return {"status": "ok", "model_loaded": False}
```

**File**: `Locus/ml-service/model.py` (Phase 0 stub)
```python
# LSTM model definition — implemented in Phase 4
import torch
import torch.nn as nn


class AircraftLSTM(nn.Module):
    """Placeholder — full implementation in Phase 4."""
    def __init__(self, input_size: int = 6, hidden_size: int = 64, num_layers: int = 2):
        super().__init__()
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers, batch_first=True)
        self.fc = nn.Linear(hidden_size, 2)  # [normal, anomalous]

    def forward(self, x):
        out, _ = self.lstm(x)
        return self.fc(out[:, -1, :])
```

**File**: `Locus/ml-service/train.py` (Phase 0 stub)
```python
# Training script — implemented in Phase 4
if __name__ == "__main__":
    print("Training not yet implemented. See Phase 4.")
```

**File**: `Locus/ml-service/requirements.txt`
```
fastapi>=0.111.0
uvicorn[standard]>=0.30.0
torch>=2.3.0
numpy>=1.26.0
pandas>=2.2.0
requests>=2.32.0
scikit-learn>=1.5.0
```

---

### Step 0.5 — Frontend Placeholder

**File**: `Locus/frontend/index.html` (Phase 0 stub)
```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Locus — Live MLAT</title>
  <style>
    body { margin: 0; background: #0d1117; color: #e6edf3;
           font-family: 'Segoe UI', sans-serif;
           display: flex; align-items: center; justify-content: center;
           height: 100vh; flex-direction: column; }
    #status { font-size: 1.4rem; color: #58a6ff; }
    #counter { font-size: 1rem; color: #8b949e; margin-top: 0.5rem; }
  </style>
</head>
<body>
  <div id="status">Locus — Connecting...</div>
  <div id="counter">Frames received: 0</div>

  <script>
    const statusEl = document.getElementById('status');
    const counterEl = document.getElementById('counter');
    let frameCount = 0;

    function connect() {
      const ws = new WebSocket('ws://localhost:9001');

      ws.onopen = () => {
        statusEl.textContent = 'Locus — Connected';
        statusEl.style.color = '#3fb950';
      };

      ws.onmessage = (evt) => {
        frameCount++;
        counterEl.textContent = `Frames received: ${frameCount}`;
      };

      ws.onclose = () => {
        statusEl.textContent = 'Locus — Reconnecting...';
        statusEl.style.color = '#f85149';
        setTimeout(connect, 3000);
      };

      ws.onerror = () => ws.close();
    }

    connect();
  </script>
</body>
</html>
```

---

### Step 0.6 — Dockerfiles

**File**: `Locus/go-ingestor/Dockerfile`
```dockerfile
FROM golang:1.24-alpine AS builder
WORKDIR /app
COPY go.mod go.sum ./
RUN go mod download
COPY . .
RUN go build -o ingestor .

FROM alpine:3.20
RUN apk add --no-cache ca-certificates
WORKDIR /app
COPY --from=builder /app/ingestor .
# .buyer-env must be mounted at runtime as /app/.buyer-env
CMD ["./ingestor", "--port=61339", "--mode=peer", \
     "--buyer-or-seller=buyer", \
     "--list-of-sellers-source=env", "--envFile=.buyer-env"]
```

**File**: `Locus/rust-backend/Dockerfile`
```dockerfile
FROM rust:1.78-slim AS builder
RUN apt-get update && apt-get install -y pkg-config libssl-dev && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY Cargo.toml Cargo.lock* ./
# Pre-cache dependencies
RUN mkdir src && echo 'fn main(){}' > src/main.rs && cargo build --release && rm -rf src
COPY src ./src
RUN touch src/main.rs && cargo build --release

FROM debian:bookworm-slim
RUN apt-get update && apt-get install -y ca-certificates && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY --from=builder /app/target/release/locus-backend .
EXPOSE 9001
ENTRYPOINT ["./locus-backend"]
```

**File**: `Locus/ml-service/Dockerfile`
```dockerfile
FROM python:3.11-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY . .
EXPOSE 8000
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

**File**: `Locus/frontend/Dockerfile`
```dockerfile
FROM nginx:alpine
COPY index.html /usr/share/nginx/html/index.html
EXPOSE 80
```

---

### Step 0.7 — Docker Compose (Phase 0 skeleton)

**File**: `Locus/docker-compose.yml`
```yaml
version: "3.9"

services:
  rust-backend:
    build: ./rust-backend
    ports:
      - "9001:9001"
    environment:
      - RUST_LOG=locus_backend=debug
      - MOCK_DATA=true
    command: ["./locus-backend", "--mock", "--ws-addr=0.0.0.0:9001",
              "--ml-service-url=http://ml-service:8000"]
    volumes:
      - locus-ipc:/tmp/locus          # shared socket dir with go-ingestor
    depends_on:
      ml-service:
        condition: service_healthy
    restart: unless-stopped

  ml-service:
    build: ./ml-service
    ports:
      - "8000:8000"
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8000/health"]
      interval: 10s
      timeout: 5s
      retries: 5
    restart: unless-stopped

  frontend:
    build: ./frontend
    ports:
      - "3000:80"
    restart: unless-stopped

  go-ingestor:
    build: ./go-ingestor
    ports:
      - "61339:61339"
    volumes:
      - ./go-ingestor/.buyer-env:/app/.buyer-env:ro
      - ./go-ingestor/location-override.json:/app/location-override.json:ro
      - locus-ipc:/tmp/locus          # exposes ingestor.sock to rust-backend
    profiles:
      - live
    restart: unless-stopped

volumes:
  locus-ipc:
    driver: local
```

Note: `go-ingestor` is in the `live` profile. In mock mode you run:
```bash
docker compose up rust-backend ml-service frontend
```

For live mode with real network data (also applies `docker-compose.live.yml` override for `depends_on: go-ingestor`):
```bash
docker compose -f docker-compose.yml -f docker-compose.live.yml --profile live up
```

**File**: `Locus/docker-compose.dev.yml`
```yaml
version: "3.9"

services:
  rust-backend:
    build:
      context: ./rust-backend
      dockerfile: Dockerfile
    volumes:
      - ./rust-backend/src:/app/src
      - locus-ipc:/tmp/locus          # shared socket dir with go-ingestor
    command: ["cargo", "watch", "-x", "run -- --mock"]
    environment:
      - RUST_LOG=locus_backend=trace

  go-ingestor:
    volumes:
      - ./go-ingestor/.buyer-env:/app/.buyer-env:ro
      - locus-ipc:/tmp/locus          # exposes ingestor.sock to rust-backend

  ml-service:
    volumes:
      - ./ml-service:/app
    command: ["uvicorn", "main:app", "--host", "0.0.0.0",
              "--port", "8000", "--reload"]

  frontend:
    volumes:
      - ./frontend/index.html:/usr/share/nginx/html/index.html:ro

volumes:
  locus-ipc:
    driver: local
```

Use dev overrides with:
```bash
docker compose -f docker-compose.yml -f docker-compose.dev.yml up
```

---

### Phase 0 Verification Checklist

```
[ ] Locus/ directory tree matches the target structure
[ ] cargo build succeeds in rust-backend/
[ ] curl http://localhost:8000/health returns {"status":"ok",...}  (after docker compose up)
[ ] Browser at http://localhost:3000 shows "Locus — Connecting..."
[ ] docker compose up (mock mode) starts 3 containers without restart loops
[ ] go-ingestor container starts cleanly under --profile live (credentials not required to build)
```

---

## Phase 1 — Data Pipeline (Go → Rust) + Mock Mode

### Goal
Wire the Go ingestor to emit structured JSON lines over a Unix domain socket, have Rust consume them by connecting to that socket, and implement mock mode as a full substitute.

### Visible Outcome
- **Terminal**: `RUST_LOG=debug cargo run -- --mock` logs "Received frame from sensor X" at >10 fps
- **Browser**: The counter on the placeholder page increments ("Frames received: N") — this proves the WebSocket pipeline is alive end-to-end even before MLAT

### Estimated time: 3–4 hours

---

### Step 1.1 — Modify Go Ingestor Output Format

**File**: `Locus/go-ingestor/main.go`

The key change from `main-half-4dsky.go.bak`: replace all `fmt.Printf(...)` human-readable lines inside the message loop with a single `json.Marshal` written to a Unix domain socket connection. Everything outside the inner loop (stream setup, error handling) stays identical.

The output JSON schema per line:
```json
{
  "sensor_id": 123456789,
  "sensor_lat": 51.5074,
  "sensor_lon": -0.1278,
  "sensor_alt": 15.0,
  "timestamp_seconds": 43200,
  "timestamp_nanoseconds": 123456789,
  "raw_modes_hex": "8D4840D6202CC371C32CE0576098"
}
```

Imports to add: `"encoding/json"`, `"net"`, `"os"`

**Before the stream handler loop**, set up the Unix socket listener:
```go
const socketPath = "/tmp/locus/ingestor.sock"

os.MkdirAll("/tmp/locus", 0755)
os.Remove(socketPath) // clean stale socket from previous run

ln, err := net.Listen("unix", socketPath)
if err != nil {
    log.Fatalf("Failed to bind Unix socket: %v", err)
}
defer ln.Close()
log.Printf("Ingestor listening on %s", socketPath)
```

Accept connections with a mutex-protected client map — removes disconnected clients on write error (Race 3: silently ignores write errors on dead connections rather than crashing the goroutine):

```go
// Add to imports: "sync", "time"

type RawFrameOut struct {
    SensorID             int64   `json:"sensor_id"`
    SensorLat            float64 `json:"sensor_lat"`
    SensorLon            float64 `json:"sensor_lon"`
    SensorAlt            float64 `json:"sensor_alt"`
    TimestampSeconds     uint64  `json:"timestamp_seconds"`
    TimestampNanoseconds uint64  `json:"timestamp_nanoseconds"`
    RawModesHex          string  `json:"raw_modes_hex"`
}
```

**Location override** — load `location-override.json` at startup:
Some sellers report inaccurate GPS positions. The hackathon team provides exact coordinates
in `location-override.json`, keyed by the seller's libp2p compressed public key (hex).

Load at startup:
```
overrides = load_location_overrides("./location-override.json")  // map[pubkey_hex]→{lat,lon,alt}
```

When populating RawFrameOut per seller stream:
```
if override, ok := overrides[peerPubKeyHex]; ok {
    frame.SensorLat = override.Lat
    frame.SensorLon = override.Lon
    frame.SensorAlt = override.Alt
}
```

Note: Resolving peerPubKeyHex from a libp2p stream requires calling
`stream.Conn().RemotePeer()` to get the `peer.ID`, then extracting the raw public key bytes.
Verify during live testing that the SDK's `peer.ID` encodes the same 33-byte compressed
public key format used in `location-override.json`.

```go
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

// Broadcast to all clients (replace direct fmt.Fprintf call):
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

// In the message loop, replace fmt.Fprintf with:
frame := RawFrameOut{
    SensorID:             sensorID,
    SensorLat:            sensorLatitude,
    SensorLon:            sensorLongitude,
    SensorAlt:            sensorAltitude,
    // TimestampSeconds must be Unix epoch seconds, NOT seconds since midnight.
    // Seconds-since-midnight wraps to 0 at UTC midnight, corrupting TDOA for cross-midnight frames.
    TimestampSeconds:     uint64(time.Now().Unix()),
    TimestampNanoseconds: uint64(time.Now().UnixNano() % 1_000_000_000),
    RawModesHex:          fmt.Sprintf("%x", rawModeS),
}
jsonBytes, err := json.Marshal(frame)
if err != nil {
    log.Printf("JSON marshal error: %v", err)
    continue
}
broadcast(jsonBytes)
```

All `log.Printf` and `fmt.Println` debug lines must write to `os.Stderr` so they don't appear on the socket. All human-readable logging goes to `os.Stderr`.

Verify with two terminals — start Go first, then in a second terminal:
```bash
nc -U /tmp/locus/ingestor.sock | head -5
```

Each line should be a valid JSON object.

---

### Step 1.2 — Rust Data Structures

**File**: `Locus/rust-backend/src/ingestor.rs`

Key types (add these — full implementations follow):

```rust
use serde::{Deserialize, Serialize};

/// Raw frame as received from the Go ingestor via Unix domain socket.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct RawFrame {
    pub sensor_id: i64,
    pub sensor_lat: f64,
    pub sensor_lon: f64,
    pub sensor_alt: f64,          // meters above sea level
    pub timestamp_seconds: u64,   // Unix epoch seconds (NOT seconds since midnight)
    pub timestamp_nanoseconds: u64,
    pub raw_modes_hex: String,    // hex-encoded Mode-S bytes
}
// Note: If the 4DSky SDK provides only seconds-since-midnight, add today's midnight Unix timestamp before writing to the socket.

impl RawFrame {
    /// Full timestamp as nanoseconds since midnight
    pub fn timestamp_ns(&self) -> u64 {
        self.timestamp_seconds * 1_000_000_000 + self.timestamp_nanoseconds
    }

    /// Decoded raw Mode-S bytes.
    /// Reserve `unwrap()` for programmer errors only (e.g. mutex poisoning);
    /// use explicit error handling on all data paths.
    pub fn raw_bytes(&self) -> Result<Vec<u8>, hex::FromHexError> {
        hex::decode(&self.raw_modes_hex)
    }
}
```

The ingestor module exposes one public async function:

```rust
pub async fn run_ingestor(
    mock: bool,
    tx: tokio::sync::mpsc::Sender<RawFrame>,
) -> anyhow::Result<()>
```

**Live mode implementation** inside `run_ingestor` — outer reconnect loop handles Go crashes/restarts (Race 1 and Race 2):
```rust
const SOCKET_PATH: &str = "/tmp/locus/ingestor.sock";
const MAX_CONNECT_RETRIES: u32 = 120;  // 60 seconds total budget
const RETRY_DELAY_MS: u64 = 500;

async fn connect_with_retry() -> anyhow::Result<tokio::net::UnixStream> {
    for attempt in 0..MAX_CONNECT_RETRIES {
        match tokio::net::UnixStream::connect(SOCKET_PATH).await {
            Ok(stream) => {
                tracing::info!("Connected to ingestor socket on attempt {}", attempt + 1);
                return Ok(stream);
            }
            Err(e) => {
                tracing::warn!("Socket connect attempt {} failed: {e}", attempt + 1);
                tokio::time::sleep(Duration::from_millis(RETRY_DELAY_MS)).await;
            }
        }
    }
    anyhow::bail!("Could not connect after {MAX_CONNECT_RETRIES} attempts")
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

// In run_ingestor (live branch) — outer reconnect loop:
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
```

Add `tokio::net::UnixStream` to imports; `tokio` feature `"net"` is already enabled via `"full"`.
The `run_ingestor(mock: bool, tx: Sender<RawFrame>)` signature is unchanged.
The mock branch is unchanged.

---

### Step 1.3 — Mock Mode Implementation

Mock mode generates synthetic frames inside Rust. No Go process is needed and `connect_with_retry()` is never called.

**Sensor positions** (6 European ground stations):

```rust
pub struct SensorPos {
    pub id: i64,
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,   // meters
}

pub const MOCK_SENSORS: &[SensorPos] = &[
    SensorPos { id: 1001, lat: 51.5074, lon: -0.1278, alt: 15.0 },  // London
    SensorPos { id: 1002, lat: 52.3676, lon:  4.9041, alt:  5.0 },  // Amsterdam
    SensorPos { id: 1003, lat: 48.8566, lon:  2.3522, alt: 35.0 },  // Paris
    SensorPos { id: 1004, lat: 50.1109, lon:  8.6821, alt: 110.0 }, // Frankfurt
    SensorPos { id: 1005, lat: 50.8503, lon:  4.3517, alt: 60.0 },  // Brussels
    SensorPos { id: 1006, lat: 53.3498, lon: -6.2603, alt: 20.0 },  // Dublin
    SensorPos { id: 1007, lat: 47.3769, lon:  8.5417, alt: 440.0 }, // Zurich
    SensorPos { id: 1008, lat: 48.2082, lon: 16.3738, alt: 170.0 }, // Vienna
];
```

**Aircraft motion** (three simulated aircraft):

```rust
pub struct MockAircraft {
    pub icao24: &'static str,
    pub is_spoofer: bool,      // ADS-B position diverges after 30s
    pub is_beacon: bool,       // Transmits full ADS-B position for clock sync
    // lat/lon/alt start and velocity (deg/s and m/s)
    pub start_lat: f64,
    pub start_lon: f64,
    pub start_alt: f64,
    pub vel_lat: f64,
    pub vel_lon: f64,
    pub vel_alt: f64,
}

pub const MOCK_AIRCRAFT: &[MockAircraft] = &[
    MockAircraft {
        icao24: "4840D6", is_spoofer: false, is_beacon: true,
        start_lat: 51.0, start_lon: 1.0, start_alt: 10000.0,
        vel_lat: 0.003, vel_lon: 0.005, vel_alt: 0.0,
    },
    MockAircraft {
        icao24: "3C6444", is_spoofer: false, is_beacon: false,
        start_lat: 50.5, start_lon: 3.0, start_alt: 9500.0,
        vel_lat: 0.002, vel_lon: 0.004, vel_alt: -1.0,
    },
    MockAircraft {
        icao24: "AABBCC", is_spoofer: true, is_beacon: false,
        start_lat: 52.0, start_lon: 5.0, start_alt: 8000.0,
        vel_lat: 0.001, vel_lon: 0.003, vel_alt: 0.5,
    },
];
```

**TDOA physics** (inside mock loop):
```
dist = sqrt((sensor_x - ac_x)^2 + (sensor_y - ac_y)^2 + (sensor_z - ac_z)^2)
arrival_time_ns = true_timestamp_ns + (dist / c_m_per_ns)
arrival_time_ns += Gaussian(mean=0, sigma=50)  // clock drift noise
```

Where `c_m_per_ns = 0.299792458` (meters per nanosecond).

Convert lat/lon/alt to ECEF for distance calculation — use `coords::wgs84_to_ecef()` from `src/coords.rs` (see Step 2.3). Do **not** duplicate this function here; import it via `use crate::coords;`.

The mock loop ticks at 20ms intervals (50 Hz) per aircraft, broadcasting to all sensors.

```rust
// Mock beacon aircraft bypass CPR decode entirely:
// When is_beacon=true, inject AdsbPosition directly from the aircraft's
// exact lat/lon/alt — skip parse_adsb_position() for mock frames.
if aircraft.is_beacon {
    Some(AdsbPosition { icao24: aircraft.icao24.to_string(),
                        lat: current_lat, lon: current_lon, alt_m: current_alt })
} else {
    None
}
```

---

### Step 1.4 — Wire WebSocket for Frame Counter

In Phase 1, the WebSocket server is a minimal broadcast. The goal is to prove the UI counter increments.

**File**: `Locus/rust-backend/src/ws_server.rs` (Phase 1 stub)

```rust
use std::sync::Arc;
use tokio::sync::broadcast;
use tokio_tungstenite::tungstenite::Message;

pub type BroadcastTx = broadcast::Sender<String>;

pub async fn run_ws_server(
    addr: &str,
    rx: broadcast::Receiver<String>,
) -> anyhow::Result<()> {
    use tokio::net::TcpListener;
    let listener = TcpListener::bind(addr).await?;
    tracing::info!("WebSocket server listening on {addr}");

    loop {
        let (stream, peer) = listener.accept().await?;
        tracing::debug!("WS peer connected: {peer}");
        let mut rx = rx.resubscribe(); // each connection gets own receiver
        tokio::spawn(async move {
            let ws = tokio_tungstenite::accept_async(stream).await?;
            let (mut sink, _source) = futures_util::StreamExt::split(ws);
            loop {
                match rx.recv().await {
                    Ok(msg) => {
                        use futures_util::SinkExt;
                        if sink.send(Message::Text(msg)).await.is_err() { break; }
                    }
                    Err(broadcast::error::RecvError::Lagged(n)) => {
                        tracing::warn!("WS client lagged by {n} messages");
                    }
                    Err(_) => break,
                }
            }
            anyhow::Ok(())
        });
    }
}
```

Update `src/main.rs` to:
1. Parse CLI args
2. Create an `mpsc::channel` for raw frames
3. Create a `broadcast::channel` for WebSocket messages
4. Spawn `ingestor::run_ingestor` task
5. Spawn `ws_server::run_ws_server` task
6. In a loop: receive raw frames, serialize to JSON, broadcast to WebSocket

```rust
// main.rs Phase 1 additions (inside tokio::main)
let (frame_tx, mut frame_rx) = tokio::sync::mpsc::channel::<RawFrame>(1024);
let (ws_tx, ws_rx) = tokio::sync::broadcast::channel::<String>(1024);

let mock = cli.mock;
let ingestor_handle = tokio::spawn(ingestor::run_ingestor(mock, frame_tx));
let ws_handle = tokio::spawn(ws_server::run_ws_server(cli.ws_addr.clone(), ws_rx));

// Forward raw frames directly to WebSocket (Phase 1 — no MLAT yet)
let forwarder = tokio::spawn(async move {
    let mut count = 0u64;
    while let Some(frame) = frame_rx.recv().await {
        count += 1;
        if count % 100 == 0 {
            tracing::info!("Received frame #{count} from sensor {}", frame.sensor_id);
        }
        let msg = serde_json::json!({ "type": "raw_frame", "sensor_id": frame.sensor_id });
        let _ = ws_tx.send(msg.to_string());
    }
});

tokio::select! {
    _ = tokio::signal::ctrl_c() => { info!("Shutting down"); }
    r = ingestor_handle => { tracing::error!("Ingestor exited: {r:?}"); }
    r = ws_handle => { tracing::error!("WS server exited: {r:?}"); }
    r = forwarder => { tracing::error!("Forwarder exited: {r:?}"); }
}
```

---

### Step 1.5 — Update Frontend Counter

Update `Locus/frontend/index.html` — the WebSocket `onmessage` handler already increments `frameCount`. In Phase 1 we also log the sensor ID received:

```javascript
ws.onmessage = (evt) => {
  frameCount++;
  const data = JSON.parse(evt.data);
  counterEl.textContent =
    `Frames received: ${frameCount} (last sensor: ${data.sensor_id ?? '?'})`;
};
```

---

### Phase 1 Verification Checklist

```
[ ] Go ingestor outputs valid JSON lines to stdout (manually verified with head -5)
[ ] cargo run -- --mock logs frame receipts at >10 fps
[ ] Browser counter at localhost:3000 increments in real time
[ ] No panic or unwrap failures in Rust logs
[ ] docker compose up (mock) shows all 3 services healthy
[ ] ECEF conversion unit test: London (51.5074, -0.1278, 15) → ~(3978553, -8837, 4968195) m
```

---

### Phase 1 Docker Compose Update

No changes needed — Phase 0 `docker-compose.yml` already has `MOCK_DATA=true` set for the Rust backend and the `locus-ipc` volume already mounted on both services. In mock mode `MOCK_DATA=true`, `go-ingestor` is skipped (it's in the `live` profile) and Rust never calls `connect_with_retry()`, so the absence of a socket file causes no crash.

---

## Phase 2 — Core MLAT Engine + Aircraft on Map

### Goal
Implement the correlator, Levenberg-Marquardt MLAT solver, Kalman filter, and broadcast full `AircraftState` objects over WebSocket so aircraft markers appear on the Leaflet map.

### Visible Outcome
- **Terminal**: `GDOP=1.8 lat=51.234 lon=-0.456 alt=10032m icao=4840D6` logged per solution
- **Browser**: Leaflet map appears; aircraft markers move in real time; clicking a marker shows ICAO24, altitude, GDOP

### Estimated time: 5–7 hours

---

### Step 2.1 — Message Correlator

**File**: `Locus/rust-backend/src/correlator.rs`

**Correlation key** — uses FxHash (100× faster than SHA256 on the hot path; `rustc-hash = "1.1"` in Cargo.toml):
```rust
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
```

**Correlation group**:
```rust
#[derive(Debug, Clone)]
pub struct CorrelationGroup {
    pub key: CorrelationKey,
    pub frames: Vec<RawFrame>,   // one per sensor (deduplicated by sensor_id)
    pub first_seen_ns: u64,      // timestamp of first frame received
}
```

**Correlator state** — pass a per-sensor drop counter from `main.rs` for UI visibility:
```rust
use std::sync::Arc;
use dashmap::DashMap;

pub struct Correlator {
    groups: HashMap<CorrelationKey, CorrelationGroup>,
    min_sensors: usize,   // 3
    window_ns: u64,       // 50_000_000 (50ms) — larger than max EU propagation spread of 6ms
    drop_counter: Arc<DashMap<i64, u64>>,  // per-sensor drop counts, broadcast in sensor_health
}

impl Correlator {
    pub fn new(drop_counter: Arc<DashMap<i64, u64>>) -> Self {
        Self {
            groups: HashMap::new(),
            min_sensors: 3,
            window_ns: 50_000_000,
            drop_counter,
        }
    }

    /// Feed a raw frame. Accumulates into the group — never emits early.
    /// Emission happens only via evict_stale(), ensuring all sensors within
    /// the 50ms window contribute to the solution.
    pub fn ingest(&mut self, frame: RawFrame) {
        let key = match CorrelationKey::from_frame(&frame) {
            Some(k) => k,
            None => return,
        };
        let now_ns = frame.timestamp_ns();
        let group = self.groups.entry(key.clone()).or_insert_with(|| {
            CorrelationGroup {
                key: key.clone(),
                frames: Vec::new(),
                first_seen_ns: now_ns,
            }
        });
        if !group.frames.iter().any(|f| f.sensor_id == frame.sensor_id) {
            group.frames.push(frame);
        }
        // No early emit — wait for full eviction window to collect all sensors
    }

    /// Call every 10ms from the main pipeline loop to evict timed-out groups.
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
```

**Main pipeline loop** — accumulate frames and emit only via timed eviction:
```rust
// Every frame:
correlator.ingest(frame);   // () — accumulate only

// Every 10ms tick:
for group in correlator.evict_stale(now_ns) {
    solver_tx.send(group).await?;  // emit all groups that timed out with >= min_sensors
}
```

**Call site in `main.rs`** — create drop counter and pass into `Correlator::new()`:
```rust
let drop_counter: Arc<DashMap<i64, u64>> = Arc::new(DashMap::new());
let mut correlator = Correlator::new(drop_counter.clone());
// Include drop_counter.clone() in the sensor_health WebSocket broadcast (ws_server.rs)
```

---

### Step 2.2 — MLAT Solver

**File**: `Locus/rust-backend/src/mlat_solver.rs`

**Coordinate handling**: The solver works in ECEF (meters). Convert all sensor positions and the initial guess to ECEF before calling. Convert the solution back to WGS84.

**Input type**:
```rust
pub struct MlatInput {
    pub sensor_positions: Vec<(f64, f64, f64)>,  // ECEF meters
    pub tdoa_ns: Vec<Vec<f64>>,     // tdoa_ns[i][j] = t_i - t_j in nanoseconds
                                    // reference sensor is index 0
}

pub struct MlatSolution {
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f64,
    pub gdop: f64,
    pub accuracy_m: f64,         // confidence_ellipse_m() result
    pub covariance: [[f64; 3]; 3],
}
```

**Residual function** (implement `argmin::core::CostFunction`):

```rust
struct MlatResiduals {
    sensor_ecef: Vec<nalgebra::Vector3<f64>>,
    observed_tdoa_m: Vec<f64>,  // TDOAs converted to meters: tdoa_ns * c_m_per_ns
}

impl argmin::core::CostFunction for MlatResiduals {
    type Param = nalgebra::Vector3<f64>;  // [x, y, z] in ECEF meters
    type Output = nalgebra::DVector<f64>;

    fn cost(&self, p: &Self::Param) -> Result<Self::Output, argmin::core::Error> {
        // residuals[k] = observed_tdoa_m[k] - theoretical_tdoa_m(p)
        // theoretical_tdoa_m[k] = dist(p, sensor[k+1]) - dist(p, sensor[0])
        let ref_dist = (p - &self.sensor_ecef[0]).norm();
        let residuals: Vec<f64> = self.observed_tdoa_m.iter().enumerate().map(|(k, obs)| {
            let dist_k = (p - &self.sensor_ecef[k + 1]).norm();
            obs - (dist_k - ref_dist)
        }).collect();
        Ok(nalgebra::DVector::from_vec(residuals))
    }
}
```

Also implement `argmin::core::Jacobian` for the LM solver — compute the Jacobian analytically:
```
J[k][i] = d/dx_i (dist(p, sensor[k+1]) - dist(p, sensor[0]))
         = (p_i - sensor[k+1][i]) / dist(p, sensor[k+1])
           - (p_i - sensor[0][i]) / dist(p, sensor[0])
```

**Solver call**:
```rust
use argmin::solver::leastsquares::GaussNewton;
// or: argmin::solver::levenbergmarquardt::LevenbergMarquardt

let solver = LevenbergMarquardt::new();
let result = Executor::new(problem, solver)
    .configure(|state| state.param(initial_guess).max_iters(100).target_cost(1e-6))
    .run()?;
```

**GDOP computation** — Nx3 matrix (MLAT has no clock bias unknown; TDOA differencing already eliminates transmission time):
```rust
fn compute_gdop(
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
        // NO fourth column — MLAT does not estimate clock bias
    }

    let ht_h = h.transpose() * &h;
    match ht_h.try_inverse() {
        Some(inv) => inv.trace().sqrt(),
        None => f64::INFINITY,  // singular matrix — collinear sensors
    }
}
```

**GDOP filter**: discard solution if `gdop > 5.0`. Log discards at `tracing::debug!` level.

**WGS84 back-conversion** — use `coords::ecef_to_wgs84()` from `src/coords.rs` (see Step 2.3). Do not duplicate; import via `use crate::coords;`.

**Public API**:
```rust
pub fn solve(input: &MlatInput, prior: Option<Vector3<f64>>) -> Option<MlatSolution>
```

Returns `None` if solver fails to converge or GDOP > 5.

> **Note**: Patch 3's Kalman prior is only physically meaningful after Patch 6 converts the
> filter to ECEF state. Implementation order: Patch 6 (ECEF Kalman) first, then Patch 3.

**Initial guess strategy** — LM is sensitive to initialization; always provide a physically meaningful starting point:
```rust
pub fn solve(input: &MlatInput, prior: Option<Vector3<f64>>) -> Option<MlatSolution> {
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

    // compute accuracy_m from confidence_ellipse_m() before returning
    let accuracy_m = confidence_ellipse_m(&covariance);
    // ... build and return MlatSolution
}
```

**Call site in main pipeline** — pass Kalman ECEF state as prior:
```rust
let prior_ecef = kalman_registry
    .get_ecef(&icao24)  // returns Option<Vector3<f64>>
    .map(|state| Vector3::new(state[0], state[1], state[2]));

let solution = mlat_solver::solve(&input, prior_ecef)?;
```

---

### Step 2.3 — Kalman Filter (EKF per Aircraft)

**File**: `Locus/rust-backend/src/kalman.rs`

**File**: `Locus/rust-backend/src/coords.rs` — shared coordinate conversion module (add `mod coords;` to `main.rs`):
```rust
// rust-backend/src/coords.rs
pub fn wgs84_to_ecef(lat_deg: f64, lon_deg: f64, alt_m: f64) -> (f64, f64, f64) {
    let a = 6_378_137.0_f64;
    let e2 = 6.694_379_990_14e-3;
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    let n = a / (1.0 - e2 * lat.sin().powi(2)).sqrt();
    let x = (n + alt_m) * lat.cos() * lon.cos();
    let y = (n + alt_m) * lat.cos() * lon.sin();
    let z = (n * (1.0 - e2) + alt_m) * lat.sin();
    (x, y, z)
}

pub fn ecef_to_wgs84(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    // Bowring's iterative method
    let a = 6_378_137.0_f64;
    let e2 = 6.694_379_990_14e-3;
    let lon = y.atan2(x);
    let p = (x * x + y * y).sqrt();
    let mut lat = z.atan2(p * (1.0 - e2));
    for _ in 0..10 {
        let n = a / (1.0 - e2 * lat.sin().powi(2)).sqrt();
        lat = (z + e2 * n * lat.sin()).atan2(p);
    }
    let n = a / (1.0 - e2 * lat.sin().powi(2)).sqrt();
    let alt = p / lat.cos() - n;
    (lat.to_degrees(), lon.to_degrees(), alt)
}
```

Also add `touch Locus/rust-backend/src/coords.rs` to the Step 0.3 stub creation commands.
The mock ingestor's `to_ecef()` function moves here as `pub fn wgs84_to_ecef(...)` — update all import sites accordingly.

**State vector**: `[x, y, z, vx, vy, vz]` in ECEF meters and m/s — no unit mixing, no degree/meter ambiguity.

```rust
use nalgebra::{Matrix6, Vector6, Matrix3x6};

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
    // accuracy_m comes from MlatSolution — GDOP-derived confidence ellipse
    pub fn update(&mut self, x_m: f64, y_m: f64, z_m: f64, accuracy_m: f64) {
        let h = Matrix3x6::new(
            1.,0.,0.,0.,0.,0.,
            0.,1.,0.,0.,0.,0.,
            0.,0.,1.,0.,0.,0.,
        );

        // Measurement noise in meters^2; floor at 100m RMS (MLAT cannot exceed this)
        let r_val = accuracy_m.powi(2).max(10_000.0);
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

        // Clamp velocity magnitude to physically plausible range
        let v_mag = (self.x[3].powi(2) + self.x[4].powi(2) + self.x[5].powi(2)).sqrt();
        const MAX_SPEED_MS: f64 = 350.0;  // well above any commercial aircraft (~Mach 1)
        if v_mag > MAX_SPEED_MS {
            let scale = MAX_SPEED_MS / v_mag;
            self.x[3] *= scale; self.x[4] *= scale; self.x[5] *= scale;
            tracing::debug!(icao24 = %self.icao24, v_mag, "velocity clamped to {MAX_SPEED_MS} m/s");
        }
    }

    // Output conversion: call only when sending to WebSocket / frontend
    pub fn position_wgs84(&self) -> (f64, f64, f64) {
        coords::ecef_to_wgs84(self.x[0], self.x[1], self.x[2])
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
        // sigma_a = 50 m/s^2 — deliberately high because velocity is inferred from noisy
        // position-only updates (100–500m MLAT accuracy at 100ms → 1000–5000 m/s finite diff).
        // High sigma_a reduces filter confidence in velocity predictions → stable tracks.
        let sigma_a2 = 2500.0_f64;  // 50^2
        let dt2 = dt * dt / 2.0;
        let dt3 = dt * dt * dt / 3.0;
        let mut q = Matrix6::zeros();
        q[(0,0)] = dt3 * sigma_a2;
        q[(1,1)] = dt3 * sigma_a2;
        q[(2,2)] = dt3 * sigma_a2;
        q[(3,3)] = dt * sigma_a2;
        q[(4,4)] = dt * sigma_a2;
        q[(5,5)] = dt * sigma_a2;
        q[(0,3)] = dt2 * sigma_a2; q[(3,0)] = dt2 * sigma_a2;
        q[(1,4)] = dt2 * sigma_a2; q[(4,1)] = dt2 * sigma_a2;
        q[(2,5)] = dt2 * sigma_a2; q[(5,2)] = dt2 * sigma_a2;
        q
    }
}
```

**Kalman registry** in main pipeline — converts WGS84 MLAT output to ECEF before updating, converts back for broadcast:
```rust
pub struct KalmanRegistry {
    filters: HashMap<String, AircraftKalman>,
}

impl KalmanRegistry {
    pub fn update(&mut self, icao24: &str, mlat: &MlatSolution, now_ns: u64) -> AircraftState {
        let (x_ecef, y_ecef, z_ecef) = coords::wgs84_to_ecef(mlat.lat, mlat.lon, mlat.alt_m);
        let entry = self.filters.entry(icao24.to_string()).or_insert_with(|| {
            AircraftKalman::new(icao24.to_string(), x_ecef, y_ecef, z_ecef)
        });
        let dt = if entry.last_update_ns == 0 { 0.1 }
                 else { (now_ns - entry.last_update_ns) as f64 / 1e9 };
        entry.predict(dt);
        entry.update(x_ecef, y_ecef, z_ecef, mlat.accuracy_m);
        entry.last_update_ns = now_ns;
        let (lat, lon, alt_m) = entry.position_wgs84();
        AircraftState::from_kalman(icao24, lat, lon, alt_m, entry, mlat)
    }

    /// Returns current ECEF position for use as LM solver prior (Patch 3).
    pub fn get_ecef(&self, icao24: &str) -> Option<Vector3<f64>> {
        self.filters.get(icao24).map(|k| {
            Vector3::new(k.x[0], k.x[1], k.x[2])
        })
    }
}
```

---

### Step 2.4 — AircraftState and WebSocket Broadcast

**File**: `Locus/rust-backend/src/ws_server.rs` — add the full `AircraftState` struct:

```rust
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AircraftState {
    pub icao24: String,
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f64,
    pub vel_lat: f64,
    pub vel_lon: f64,
    pub vel_alt: f64,
    pub gdop: f64,
    pub confidence_ellipse_m: f64,
    pub spoof_flag: bool,
    pub divergence_m: f64,
    pub anomaly_label: String,       // "normal" | "anomalous" | "unknown"
    pub anomaly_confidence: f64,
    pub sensor_count: usize,
    pub timestamp_ms: u64,
}
```

Serialize to JSON and send over the broadcast channel from `main.rs` after each Kalman update.

---

### Step 2.5 — Frontend Leaflet Map

Replace the placeholder `index.html` with the Phase 2 version. Key additions:

- Leaflet.js map centered on Cornwall/Scilly sensor cluster (lat=50.1, lon=-5.8, zoom=9)
- `aircraft` Map: `icao24 → { marker, track_polyline, last_seen }`
- WebSocket `onmessage` updates marker position and extends polyline
- Marker tooltip: ICAO24, alt, GDOP
- Stale aircraft (no update >30s) get marker removed

```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <title>Locus — Live MLAT</title>
  <link rel="stylesheet"
        href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body { display: flex; height: 100vh; background: #0d1117; color: #e6edf3;
           font-family: 'Segoe UI', sans-serif; }
    #sidebar { width: 300px; background: #161b22; border-right: 1px solid #30363d;
               display: flex; flex-direction: column; overflow: hidden; }
    #sidebar-header { padding: 16px; border-bottom: 1px solid #30363d; }
    #sidebar-header h1 { font-size: 1.2rem; color: #58a6ff; }
    #status-bar { font-size: 0.75rem; color: #8b949e; margin-top: 4px; }
    #aircraft-list { flex: 1; overflow-y: auto; padding: 8px; }
    .aircraft-item { background: #0d1117; border: 1px solid #30363d;
                     border-radius: 6px; padding: 10px; margin-bottom: 8px;
                     cursor: pointer; transition: border-color 0.2s; }
    .aircraft-item:hover { border-color: #58a6ff; }
    .aircraft-item.spoofed { border-color: #f85149; }
    .aircraft-icao { font-weight: bold; font-size: 0.9rem; }
    .aircraft-detail { font-size: 0.75rem; color: #8b949e; margin-top: 4px; }
    #map { flex: 1; }
    #alert-feed { height: 120px; background: #0d1117;
                  border-top: 1px solid #30363d; overflow-y: auto; padding: 8px; }
    .alert { font-size: 0.75rem; padding: 4px;
             border-left: 3px solid #f85149; padding-left: 8px;
             margin-bottom: 4px; color: #ffa657; }
  </style>
</head>
<body>
  <div id="sidebar">
    <div id="sidebar-header">
      <h1>Locus</h1>
      <div id="status-bar">Connecting...</div>
    </div>
    <div id="aircraft-list"></div>
    <div id="alert-feed"></div>
  </div>
  <div id="map"></div>

  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <script>
    // Map setup
    const map = L.map('map').setView([50.1, -5.8], 9);
    L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
      attribution: 'Locus | CartoDB'
    }).addTo(map);

    const aircraft = new Map();
    const statusBar = document.getElementById('status-bar');
    const aircraftList = document.getElementById('aircraft-list');
    const alertFeed = document.getElementById('alert-feed');

    function makeIcon(spoofed, anomalous) {
      const color = spoofed ? '#f85149' : anomalous ? '#ffa657' : '#3fb950';
      return L.divIcon({
        className: '',
        html: `<div style="width:10px;height:10px;border-radius:50%;
                           background:${color};border:2px solid #fff;"></div>`,
        iconSize: [10, 10],
        iconAnchor: [5, 5],
      });
    }

    function updateAircraft(state) {
      const key = state.icao24;
      if (!aircraft.has(key)) {
        const marker = L.marker([state.lat, state.lon], {
          icon: makeIcon(state.spoof_flag, state.anomaly_label === 'anomalous')
        }).addTo(map);
        const track = L.polyline([], { color: '#58a6ff', weight: 1, opacity: 0.6 }).addTo(map);
        aircraft.set(key, { marker, track, last_seen: Date.now() });
      }
      const ac = aircraft.get(key);
      ac.marker.setLatLng([state.lat, state.lon]);
      ac.marker.setIcon(makeIcon(state.spoof_flag, state.anomaly_label === 'anomalous'));
      ac.marker.bindTooltip(
        `<b>${state.icao24}</b><br>` +
        `Alt: ${Math.round(state.alt_m)}m<br>` +
        `GDOP: ${state.gdop.toFixed(2)}<br>` +
        `Sensors: ${state.sensor_count}`
      );
      ac.track.addLatLng([state.lat, state.lon]);
      ac.last_seen = Date.now();

      // Sidebar card
      let card = document.getElementById(`ac-${key}`);
      if (!card) {
        card = document.createElement('div');
        card.id = `ac-${key}`;
        card.className = 'aircraft-item';
        card.onclick = () => map.setView([state.lat, state.lon], 8);
        aircraftList.prepend(card);
      }
      card.className = `aircraft-item${state.spoof_flag ? ' spoofed' : ''}`;
      card.innerHTML = `
        <div class="aircraft-icao">${state.icao24}
          ${state.spoof_flag ? ' <span style="color:#f85149">SPOOF</span>' : ''}
          ${state.anomaly_label === 'anomalous' ? ' <span style="color:#ffa657">ANOMALY</span>' : ''}
        </div>
        <div class="aircraft-detail">
          Alt: ${Math.round(state.alt_m)}m | GDOP: ${state.gdop.toFixed(2)} |
          Sensors: ${state.sensor_count}
        </div>`;

      // Alerts
      if (state.spoof_flag) {
        const alert = document.createElement('div');
        alert.className = 'alert';
        alert.textContent = `${new Date().toLocaleTimeString()} SPOOF: ${state.icao24} divergence ${Math.round(state.divergence_m)}m`;
        alertFeed.prepend(alert);
        if (alertFeed.children.length > 50) alertFeed.lastChild.remove();
      }
    }

    // Stale aircraft cleanup (every 10s)
    setInterval(() => {
      const cutoff = Date.now() - 30000;
      for (const [key, ac] of aircraft.entries()) {
        if (ac.last_seen < cutoff) {
          map.removeLayer(ac.marker);
          map.removeLayer(ac.track);
          aircraft.delete(key);
          document.getElementById(`ac-${key}`)?.remove();
        }
      }
    }, 10000);

    // WebSocket
    let frameCount = 0;
    function connect() {
      const ws = new WebSocket('ws://localhost:9001');
      ws.onopen = () => { statusBar.textContent = 'Connected'; };
      ws.onmessage = (evt) => {
        const state = JSON.parse(evt.data);
        if (state.icao24) {
          frameCount++;
          updateAircraft(state);
          statusBar.textContent = `Connected | ${frameCount} updates | ${aircraft.size} aircraft`;
        }
      };
      ws.onclose = () => {
        statusBar.textContent = 'Reconnecting...';
        setTimeout(connect, 3000);
      };
      ws.onerror = () => ws.close();
    }
    connect();
  </script>
</body>
</html>
```

---

### Phase 2 Verification Checklist

```
[ ] Correlator groups frames correctly in unit test (inject 5 frames same ICAO24 + hash)
[ ] MLAT solver converges on mock data: solution within 500m of true position
[ ] GDOP values logged per solution; solutions with GDOP>5 discarded
[ ] Kalman filter smooths position (no position jumps >2km between 100ms updates)
[ ] Aircraft markers appear on Leaflet map and move continuously
[ ] Clicking a marker shows tooltip with ICAO24, altitude, GDOP
[ ] Track polylines drawn correctly (no wraparound artifacts)
[ ] No memory leak: stale aircraft cleaned up from map after 30s
```

---

### Phase 2 Docker Compose Update

No Compose changes required. The Rust container already builds and runs. Verify:
```bash
docker compose build rust-backend
docker compose up rust-backend ml-service frontend
```

---

## Phase 3 — Self-Calibrating Clock Sync + Sensor Health UI

### Goal
Parse ADS-B position frames from beacon aircraft to compute per-sensor-pair clock offsets, apply them before the MLAT solve, and display sensor health in a Chart.js panel.

### Visible Outcome
- **Terminal**: `clock[1001→1002] offset=+12.3ns n=50` logged, GDOP improving over time
- **Browser**: "Sensor Health" panel shows a per-sensor clock offset drift sparkline (Chart.js)

### Estimated time: 3–4 hours

---

### Step 3.1 — ADS-B Position Parser

Inside `correlator.rs` or a new `adsb_parser.rs`, add a function to extract the ADS-B (DF17) airborne position from raw Mode-S bytes:

```rust
pub struct AdsbPosition {
    pub icao24: String,
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f64,
}

/// Attempt to decode an ADS-B airborne position message (DF17, TC=9..18).
/// Returns None if frame is not a position message or CPR decode fails.
pub fn parse_adsb_position(bytes: &[u8]) -> Option<AdsbPosition> {
    if bytes.len() < 14 { return None; }
    let df = (bytes[0] >> 3) & 0x1F;
    if df != 17 && df != 18 { return None; }
    let tc = (bytes[4] >> 3) & 0x1F;
    if !(9..=18).contains(&tc) { return None; } // airborne position TCs
    let icao24 = format!("{:02X}{:02X}{:02X}", bytes[1], bytes[2], bytes[3]);
    // Altitude (ME bytes 4-5, bits 40-52)
    let alt_raw = ((bytes[5] as u16) << 4) | ((bytes[6] as u16) >> 4);
    let alt_m = decode_altitude_m(alt_raw)?;
    // CPR position (simplified: use globally-unambiguous decode)
    let lat_cpr = ((bytes[6] as u32 & 0x03) << 15)
                | ((bytes[7] as u32) << 7)
                | ((bytes[8] as u32) >> 1);
    let lon_cpr = ((bytes[8] as u32 & 0x01) << 16)
                | ((bytes[9] as u32) << 8)
                | (bytes[10] as u32);
    let f_flag = (bytes[6] >> 2) & 0x01; // 0=even, 1=odd
    // For ground truth use, we do simple surface-relative decode:
    // In production, maintain even/odd pair cache; for clock sync use
    // recent pair to compute precise lat/lon.
    // TODO: implement full CPR decode — see Step 3.1 note
    None // placeholder until CPR decoder implemented
}
```

Note: Full CPR decoding requires accumulating even/odd frame pairs per ICAO24. Implement a `CprDecoder` struct:

```rust
pub struct CprDecoder {
    even: Option<(u32, u32)>,  // (lat_cpr, lon_cpr)
    odd:  Option<(u32, u32)>,
}

impl CprDecoder {
    pub fn feed(&mut self, lat_cpr: u32, lon_cpr: u32, f_flag: u8) -> Option<(f64, f64)> {
        if f_flag == 0 { self.even = Some((lat_cpr, lon_cpr)); }
        else           { self.odd  = Some((lat_cpr, lon_cpr)); }

        let (even_lat, even_lon) = self.even?;
        let (odd_lat,  odd_lon)  = self.odd?;

        let n_bits = (1u64 << 17) as f64;  // 2^17
        let dlat_e = 360.0 / 60.0_f64;
        let dlat_o = 360.0 / 59.0_f64;

        let j = ((59.0 * even_lat as f64 - 60.0 * odd_lat as f64) / n_bits + 0.5).floor();
        let lat_e = dlat_e * (j % 60.0 + even_lat as f64 / n_bits);
        let lat_o = dlat_o * (j % 59.0 + odd_lat  as f64 / n_bits);

        // Normalize to [-90, 90]
        let lat_e = if lat_e >= 270.0 { lat_e - 360.0 } else { lat_e };
        let lat_o = if lat_o >= 270.0 { lat_o - 360.0 } else { lat_o };

        // Zone consistency check
        if nl(lat_e) as u32 != nl(lat_o) as u32 { return None; }

        // Use most-recent frame for longitude
        let (lat, m, cpr_lon) = if f_flag == 1 {
            (lat_o, (nl(lat_o) - 1.0).max(1.0), odd_lon)
        } else {
            (lat_e, nl(lat_e).max(1.0), even_lon)
        };

        let dlon = 360.0 / m;
        let mi = ((even_lon as f64 * (nl(lat_e) - 1.0)
                  - odd_lon as f64 * nl(lat_e)) / n_bits + 0.5).floor();
        let lon = dlon * (mi % m + cpr_lon as f64 / n_bits);
        let lon = if lon >= 180.0 { lon - 360.0 } else { lon };

        Some((lat, lon))
    }
}

/// NL(lat) — number of longitude zones (ICAO Doc 9684 Table)
fn nl(lat: f64) -> f64 {
    let lat = lat.abs();
    if lat >= 87.0 { return 1.0; }
    let a = (1.0 - (std::f64::consts::PI / 60.0).cos().powi(2)
             / lat.to_radians().cos().powi(2)).acos();
    (2.0 * std::f64::consts::PI / a).floor()
}
```

For hackathon purposes, if CPR decode is complex, use the aircraft's self-reported position via the ADS-B velocity message (TC=19) combined with known reference points. An alternative shortcut: use the claimed lat/lon directly from the spoof detector (Step 4.1) and trust it only for beacon aircraft that are validated by two independent ground stations — log a warning if only clock-sync beacons are ADS-B-only.

---

### Step 3.2 — Clock Sync Engine

**File**: `Locus/rust-backend/src/clock_sync.rs`

Uses windowed median instead of EMA — converges in ~5 observations vs. ~45 for EMA, and is immune to multipath spike contamination.

```rust
use std::collections::{HashMap, VecDeque};

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

    /// Update clock offset using a beacon aircraft with known position.
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

    /// Apply corrections to a set of frames before passing TDOAs to the MLAT solver.
    pub fn apply_corrections(&self, frames: &mut [RawFrame]) {
        // For each frame, apply pairwise offset using get_offset(ref_sensor, frame.sensor_id)
        // (existing correction logic, just using get_offset() instead of offsets HashMap)
        let ref_id = frames[0].sensor_id;
        for frame in frames[1..].iter_mut() {
            let correction = self.get_offset(frame.sensor_id, ref_id);
            // correction is applied to frame.timestamp_ns before computing TDOA
            let _ = correction; // applied at TDOA computation site
        }
    }

    // TDOA assembly (main pipeline, after apply_corrections):
    // CRITICAL unit ordering: subtract correction in NANOSECONDS first, then multiply by c.
    //
    // WRONG:  corrected_tdoa_m = (raw_tdoa_ns * C_M_PER_NS) - correction_ns
    //         ^^^ unit mismatch: subtracting nanoseconds from meters
    //
    // CORRECT:
    // let corrected_tdoa_ns = raw_tdoa_ns - clock_sync.get_offset(sensor_i_id, ref_id);
    // let observed_tdoa_m   = corrected_tdoa_ns * C_M_PER_NS;  // now convert to meters

    /// Export current offsets for UI display — includes convergence status.
    pub fn export_offsets(&self) -> Vec<SensorOffsetReport> {
        self.samples.iter().map(|((si, sj), buf)| SensorOffsetReport {
            sensor_pair: (*si, *sj),
            offset_ns: if buf.is_empty() { 0.0 } else { median_of_deque(buf) },
            sample_count: buf.len(),
            is_converged: buf.len() >= WINDOW_SIZE / 2,
        }).collect()
    }
}

#[derive(Serialize)]
pub struct SensorOffsetReport {
    pub sensor_pair: (i64, i64),
    pub offset_ns: f64,         // from median_of_deque
    pub sample_count: usize,    // from buf.len()
    pub is_converged: bool,     // from is_converged()
}

fn median_of_deque(buf: &VecDeque<f64>) -> f64 {
    let mut v: Vec<f64> = buf.iter().copied().collect();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mid = v.len() / 2;
    if v.len() % 2 == 0 { (v[mid - 1] + v[mid]) / 2.0 } else { v[mid] }
}

fn spread_of_deque(buf: &VecDeque<f64>) -> f64 {
    // Median Absolute Deviation — robust measure of spread
    let med = median_of_deque(buf);
    let mut deviations: Vec<f64> = buf.iter().map(|x| (x - med).abs()).collect();
    deviations.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mid = deviations.len() / 2;
    deviations[mid]
}
```

Broadcast a `sensor_health` WebSocket message every 5 seconds:
```json
{
  "type": "sensor_health",
  "offsets": [
    {
      "sensor_pair": [1001, 1002],
      "offset_ns": 12.3,
      "sample_count": 450,
      "is_converged": true
    }
  ]
}
```

---

### Step 3.3 — Sensor Health Panel (Chart.js)

Add to `frontend/index.html` below the alert feed:

```html
<!-- Add to sidebar, below alert-feed -->
<div id="sensor-health" style="height:200px;padding:8px;border-top:1px solid #30363d;">
  <div style="font-size:0.75rem;color:#8b949e;margin-bottom:4px;">Sensor Clock Offsets (ns)</div>
  <canvas id="offset-chart"></canvas>
</div>
```

Add Chart.js CDN in `<head>`:
```html
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
```

In the `<script>` block, add the sensor health chart:

```javascript
const offsetHistory = {}; // sensorPair -> [last 60 values]
let offsetChart = null;

function initOffsetChart() {
  const ctx = document.getElementById('offset-chart').getContext('2d');
  offsetChart = new Chart(ctx, {
    type: 'line',
    data: { labels: Array(60).fill(''), datasets: [] },
    options: {
      animation: false,
      scales: {
        x: { display: false },
        y: { ticks: { color: '#8b949e', font: { size: 10 } },
             grid: { color: '#30363d' } }
      },
      plugins: { legend: { labels: { color: '#8b949e', font: { size: 9 } } } }
    }
  });
}

function updateSensorHealth(msg) {
  if (!offsetChart) initOffsetChart();
  for (const o of msg.offsets) {
    const key = `${o.sensor_i}-${o.sensor_j}`;
    if (!offsetHistory[key]) offsetHistory[key] = [];
    offsetHistory[key].push(o.offset_ns);
    if (offsetHistory[key].length > 60) offsetHistory[key].shift();
  }
  // Rebuild datasets
  const colors = ['#58a6ff','#3fb950','#ffa657','#f85149','#d2a8ff','#79c0ff'];
  offsetChart.data.datasets = Object.entries(offsetHistory).map(([key, vals], i) => ({
    label: key,
    data: vals,
    borderColor: colors[i % colors.length],
    borderWidth: 1,
    pointRadius: 0,
    tension: 0.3,
  }));
  offsetChart.update();
}

// Update WebSocket message handler to dispatch by type:
ws.onmessage = (evt) => {
  const msg = JSON.parse(evt.data);
  if (msg.type === 'sensor_health') {
    updateSensorHealth(msg);
  } else if (msg.icao24) {
    frameCount++;
    updateAircraft(msg);
    statusBar.textContent = `Connected | ${frameCount} updates | ${aircraft.size} aircraft`;
  }
};
```

---

### Phase 3 Verification Checklist

```
[ ] Clock offsets logged per sensor pair after first beacon aircraft observation
[ ] Windowed median converges: offset stabilizes within ~5 beacon observations (WINDOW_SIZE/2 = 25 samples)
[ ] GDOP before correction vs after correction logged — improvement visible
[ ] sensor_health WebSocket message emitted every 5 seconds
[ ] Chart.js panel shows offset sparklines updating in real time
[ ] Beacon aircraft (is_beacon=true in mock) drives clock sync, not the spoofer
```

---

### Phase 3 Docker Compose Update

No additional Compose changes. Verify the 5-second health broadcast does not flood the WebSocket.

---

## Phase 4 — Spoof Detector + AI Anomaly Classifier

### Goal
Flag aircraft whose ADS-B claimed position diverges from MLAT-derived position by >2km, and classify aircraft tracks as normal/anomalous using the LSTM model.

### Visible Outcome
- **Terminal**: `SPOOF DETECTED icao=AABBCC divergence=5241m` for the mock spoofer aircraft
- **Browser**: Spoofer aircraft marker turns red, alert feed shows divergence value; anomaly labels appear on sidebar cards

### Estimated time: 4–6 hours

---

### Step 4.1 — Spoof Detector

**File**: `Locus/rust-backend/src/spoof_detector.rs`

```rust
const SPOOF_THRESHOLD_M: f64 = 2000.0; // 2km divergence triggers flag

pub struct SpoofDetector {
    // Last known ADS-B claimed position per ICAO24
    claimed_positions: HashMap<String, (f64, f64, f64)>,  // lat, lon, alt_m
}

impl SpoofDetector {
    pub fn new() -> Self { Self { claimed_positions: HashMap::new() } }

    /// Record an ADS-B claimed position for an aircraft.
    pub fn record_adsb(&mut self, icao24: &str, lat: f64, lon: f64, alt_m: f64) {
        self.claimed_positions.insert(icao24.to_string(), (lat, lon, alt_m));
    }

    /// Compare MLAT-derived 3D position against ADS-B claim using ECEF Euclidean distance.
    /// Threshold is GDOP-adaptive: flag only when divergence exceeds MLAT's own uncertainty.
    pub fn check(&self, icao24: &str,
                 mlat_lat: f64, mlat_lon: f64, mlat_alt_m: f64,
                 accuracy_m: f64) -> (bool, f64) {
        match self.claimed_positions.get(icao24) {
            None => (false, 0.0),
            Some(&(claimed_lat, claimed_lon, claimed_alt)) => {
                let (mx, my, mz) = crate::coords::wgs84_to_ecef(mlat_lat, mlat_lon, mlat_alt_m);
                let (cx, cy, cz) = crate::coords::wgs84_to_ecef(claimed_lat, claimed_lon, claimed_alt);
                let divergence_m = ((mx-cx).powi(2) + (my-cy).powi(2) + (mz-cz).powi(2)).sqrt();
                // GDOP-adaptive: only flag when divergence exceeds MLAT uncertainty (3-sigma)
                let threshold = SPOOF_THRESHOLD_M.max(3.0 * accuracy_m);
                let flag = divergence_m > threshold;
                if flag {
                    tracing::warn!(
                        "SPOOF DETECTED icao={icao24} divergence={divergence_m:.0}m \
                         threshold={threshold:.0}m (accuracy={accuracy_m:.0}m)"
                    );
                }
                (flag, divergence_m)
            }
        }
    }
}
```

Remove `use geo::{point, HaversineDistance};` import. Update call sites to pass `mlat.accuracy_m`.

In **mock mode**, inject the spoof behavior: after 30 seconds of simulation time, the `AABBCC` aircraft's ADS-B-reported position (fed to `record_adsb`) starts deviating +0.05 degrees lat from its true MLAT-tracked position per second.

---

### Step 4.2 — Python ML Service: Full Implementation

**File**: `Locus/ml-service/model.py`

```python
import torch
import torch.nn as nn


class AircraftLSTM(nn.Module):
    """
    LSTM anomaly classifier for aircraft tracks.
    Input: sequence of 20 Kalman state vectors [lat, lon, alt, vel_lat, vel_lon, vel_alt]
    Output: [p_normal, p_anomalous]
    """
    def __init__(self, input_size: int = 6, hidden_size: int = 128,
                 num_layers: int = 2, dropout: float = 0.3):
        super().__init__()
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        self.lstm = nn.LSTM(
            input_size, hidden_size, num_layers,
            batch_first=True, dropout=dropout if num_layers > 1 else 0.0
        )
        self.dropout = nn.Dropout(dropout)
        self.fc = nn.Linear(hidden_size, 2)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (batch, seq_len=20, features=6)
        out, _ = self.lstm(x)
        out = self.dropout(out[:, -1, :])  # last timestep
        return self.fc(out)   # (batch, 2) — raw logits
```

**File**: `Locus/ml-service/train.py`

```python
"""
Train the LSTM anomaly classifier on OpenSky Network data.
Fetches historical flight data, labels anomalies (spoofed/unusual trajectories),
and trains the model.

Usage:
    python train.py --epochs 20 --output model.pt
"""
import argparse
import logging
import os
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset
from model import AircraftLSTM

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("skymesh-train")

SEQ_LEN = 20
FEATURES = 6   # lat, lon, alt, vel_lat, vel_lon, vel_alt


def fetch_opensky_data(start_ts: int, end_ts: int):
    """
    Fetch flight state vectors from OpenSky REST API.
    GET https://opensky-network.org/api/states/all?time=<ts>&lamin=45&lamax=55&lomin=-5&lomax=15
    Returns list of state vectors as dicts.
    """
    import requests
    url = "https://opensky-network.org/api/states/all"
    params = {"time": start_ts, "lamin": 45, "lamax": 55, "lomin": -5, "lomax": 15}
    try:
        r = requests.get(url, params=params, timeout=30)
        r.raise_for_status()
        data = r.json()
        return data.get("states", []) or []
    except Exception as e:
        logger.warning(f"OpenSky fetch error: {e}")
        return []


def generate_synthetic_dataset(n_normal: int = 800, n_anomalous: int = 200):
    """
    Generate synthetic training data when OpenSky is unavailable.
    Normal: smooth velocity, realistic altitudes.
    Anomalous: sudden position jumps (simulating spoofing).
    """
    def normal_track():
        lat, lon, alt = np.random.uniform(48, 54), np.random.uniform(-2, 12), \
                        np.random.uniform(5000, 12000)
        vl, vlo, va = np.random.uniform(0.001, 0.005), np.random.uniform(0.002, 0.008), \
                      np.random.uniform(-2, 2)
        seq = []
        for _ in range(SEQ_LEN):
            seq.append([lat, lon, alt, vl, vlo, va])
            lat += vl * 0.1; lon += vlo * 0.1; alt += va * 0.1
        return np.array(seq)

    def anomalous_track():
        track = normal_track()
        # Inject jump at random timestep
        jump_idx = np.random.randint(5, 15)
        track[jump_idx:, 0] += np.random.uniform(0.02, 0.08)  # lat jump (spoof)
        track[jump_idx:, 1] += np.random.uniform(0.02, 0.08)
        return track

    X = np.array([normal_track() for _ in range(n_normal)] +
                 [anomalous_track() for _ in range(n_anomalous)])
    y = np.array([0] * n_normal + [1] * n_anomalous)
    idx = np.random.permutation(len(X))
    return X[idx], y[idx]


def train(epochs: int, output_path: str):
    logger.info("Generating training dataset...")
    X, y = generate_synthetic_dataset()

    # Normalize features
    X_mean = X.mean(axis=(0, 1), keepdims=True)
    X_std = X.std(axis=(0, 1), keepdims=True) + 1e-8
    X_norm = (X - X_mean) / X_std

    # Save normalization params alongside model
    np.save(output_path.replace('.pt', '_mean.npy'), X_mean)
    np.save(output_path.replace('.pt', '_std.npy'), X_std)

    split = int(0.8 * len(X_norm))
    X_train, X_val = X_norm[:split], X_norm[split:]
    y_train, y_val = y[:split], y[split:]

    ds_train = TensorDataset(torch.FloatTensor(X_train), torch.LongTensor(y_train))
    ds_val = TensorDataset(torch.FloatTensor(X_val), torch.LongTensor(y_val))
    dl_train = DataLoader(ds_train, batch_size=32, shuffle=True)
    dl_val = DataLoader(ds_val, batch_size=64)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = AircraftLSTM().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-3, weight_decay=1e-4)
    criterion = nn.CrossEntropyLoss()
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, epochs)

    for epoch in range(1, epochs + 1):
        model.train()
        total_loss = 0.0
        for xb, yb in dl_train:
            xb, yb = xb.to(device), yb.to(device)
            optimizer.zero_grad()
            logits = model(xb)
            loss = criterion(logits, yb)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            total_loss += loss.item()
        scheduler.step()

        model.eval()
        correct = total = 0
        with torch.no_grad():
            for xb, yb in dl_val:
                xb, yb = xb.to(device), yb.to(device)
                preds = model(xb).argmax(dim=1)
                correct += (preds == yb).sum().item()
                total += len(yb)
        acc = correct / total
        logger.info(f"Epoch {epoch}/{epochs} loss={total_loss/len(dl_train):.4f} val_acc={acc:.3f}")

    torch.save(model.state_dict(), output_path)
    logger.info(f"Model saved to {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--output", type=str, default="model.pt")
    args = parser.parse_args()
    train(args.epochs, args.output)
```

**File**: `Locus/ml-service/main.py` (Phase 4 full version)

```python
import logging
import os
from pathlib import Path
from typing import List, Optional
import numpy as np
import torch
import torch.nn.functional as F
from fastapi import FastAPI, HTTPException, BackgroundTasks
from pydantic import BaseModel
from model import AircraftLSTM

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("locus-ml")

app = FastAPI(title="Locus ML Service", version="1.0.0")

MODEL_PATH = Path(os.getenv("MODEL_PATH", "model.pt"))
MEAN_PATH = MODEL_PATH.parent / (MODEL_PATH.stem + "_mean.npy")
STD_PATH = MODEL_PATH.parent / (MODEL_PATH.stem + "_std.npy")
SEQ_LEN = 20
FEATURES = 6

device = torch.device("cpu")
model: Optional[AircraftLSTM] = None
X_mean: Optional[np.ndarray] = None
X_std: Optional[np.ndarray] = None


def load_model():
    # Note: model, X_mean, X_std are updated non-atomically.
    # In production, protect with threading.Lock(). For demo, load_model() is called
    # only at startup or after /train completes — never concurrently with classify.
    global model, X_mean, X_std
    if not MODEL_PATH.exists():
        logger.warning(f"Model file not found at {MODEL_PATH}. /classify will return 'unknown'.")
        return
    m = AircraftLSTM()
    m.load_state_dict(torch.load(MODEL_PATH, map_location=device))
    m.eval()
    model = m
    if MEAN_PATH.exists():
        X_mean = np.load(MEAN_PATH)
        X_std = np.load(STD_PATH)
    logger.info("Model loaded successfully")


@app.on_event("startup")
async def startup():
    load_model()


@app.get("/health")
async def health():
    return {"status": "ok", "model_loaded": model is not None}


class KalmanStatePoint(BaseModel):
    lat: float
    lon: float
    alt_m: float
    vel_lat: float
    vel_lon: float
    vel_alt: float


class ClassifyRequest(BaseModel):
    icao24: str
    states: List[KalmanStatePoint]  # last 20 Kalman states


class ClassifyResponse(BaseModel):
    icao24: str
    anomaly_label: str       # "normal" | "anomalous" | "unknown"
    confidence: float


@app.post("/classify", response_model=ClassifyResponse)
async def classify(req: ClassifyRequest):
    if model is None:
        return ClassifyResponse(icao24=req.icao24, anomaly_label="unknown", confidence=0.0)

    if not req.states:
        return ClassifyResponse(icao24=req.icao24, anomaly_label="unknown", confidence=0.0)

    if len(req.states) < SEQ_LEN:
        # Pad with first known state (not zeros — lat=0,lon=0 is outside training distribution)
        first = [req.states[0].lat, req.states[0].lon, req.states[0].alt_m,
                 req.states[0].vel_lat, req.states[0].vel_lon, req.states[0].vel_alt]
        pad = SEQ_LEN - len(req.states)
        seq = [first] * pad + [
            [s.lat, s.lon, s.alt_m, s.vel_lat, s.vel_lon, s.vel_alt]
            for s in req.states
        ]
    else:
        seq = [[s.lat, s.lon, s.alt_m, s.vel_lat, s.vel_lon, s.vel_alt] for s in req.states[-SEQ_LEN:]]

    x = np.array(seq, dtype=np.float32)[np.newaxis]  # (1, 20, 6)
    if X_mean is not None:
        x = (x - X_mean) / X_std

    with torch.no_grad():
        logits = model(torch.tensor(x))
        probs = F.softmax(logits, dim=1).numpy()[0]

    label = "anomalous" if probs[1] > 0.5 else "normal"
    confidence = float(probs[1] if label == "anomalous" else probs[0])

    logger.info(f"classify {req.icao24}: {label} ({confidence:.3f})")
    return ClassifyResponse(icao24=req.icao24, anomaly_label=label, confidence=confidence)


@app.post("/train")
async def trigger_train(background_tasks: BackgroundTasks):
    """Trigger model retraining in the background."""
    def _train():
        import subprocess
        result = subprocess.run(["python", "train.py", "--epochs", "10", "--output", str(MODEL_PATH)],
                                capture_output=True, text=True)
        logger.info(f"Training stdout: {result.stdout}")
        if result.returncode == 0:
            load_model()
            logger.info("Model reloaded after training")
        else:
            logger.error(f"Training failed: {result.stderr}")
    background_tasks.add_task(_train)
    return {"status": "training started"}
```

---

### Step 4.3 — Rust Anomaly Client

**File**: `Locus/rust-backend/src/anomaly_client.rs`

```rust
use reqwest::Client;
use serde::{Deserialize, Serialize};

#[derive(Serialize)]
struct KalmanStatePoint {
    lat: f64, lon: f64, alt_m: f64,
    vel_lat: f64, vel_lon: f64, vel_alt: f64,
}

#[derive(Serialize)]
struct ClassifyRequest<'a> {
    icao24: &'a str,
    states: Vec<KalmanStatePoint>,
}

#[derive(Deserialize)]
pub struct ClassifyResponse {
    pub icao24: String,
    pub anomaly_label: String,
    pub confidence: f64,
}

pub struct AnomalyClient {
    client: Client,
    base_url: String,
}

impl AnomalyClient {
    pub fn new(base_url: String) -> Self {
        Self { client: Client::new(), base_url }
    }

    pub async fn classify(
        &self,
        icao24: &str,
        history: &[(f64, f64, f64, f64, f64, f64)], // (lat,lon,alt,vlat,vlon,valt)
    ) -> anyhow::Result<ClassifyResponse> {
        let states: Vec<KalmanStatePoint> = history.iter().map(|&(lat, lon, alt_m, vel_lat, vel_lon, vel_alt)| {
            KalmanStatePoint { lat, lon, alt_m, vel_lat, vel_lon, vel_alt }
        }).collect();

        let req = ClassifyRequest { icao24, states };
        let resp = self.client
            .post(format!("{}/classify", self.base_url))
            .json(&req)
            .timeout(std::time::Duration::from_millis(500))
            .send()
            .await?
            .json::<ClassifyResponse>()
            .await?;
        Ok(resp)
    }
}
```

In `main.rs` (or `kalman.rs`), maintain an `AircraftHistory` per ICAO24 inside `KalmanRegistry`. The history VecDeque is filled on every Kalman update (20ms), but the ML HTTP call is rate-limited to once per 5 seconds — reducing load from 150 calls/sec to 0.6 calls/sec, well within single-worker FastAPI capacity.

```rust
use std::collections::VecDeque;

/// Per-aircraft ML classification history.
/// Tuple units: (lat°, lon°, alt_m, vel_lat °/s, vel_lon °/s, vel_alt m/s)
///
/// IMPORTANT: vel_lat and vel_lon are in DEGREES/SECOND to match the training
/// data in generate_synthetic_dataset(). Convert from ECEF m/s via full ENU
/// rotation at push time — do NOT pass velocity_ms() ECEF components directly.
pub struct AircraftHistory {
    pub states: VecDeque<(f64, f64, f64, f64, f64, f64)>,
    pub last_classify_ns: u64,   // gating field — limits classify to 5s cadence
}

impl AircraftHistory {
    pub fn new() -> Self {
        Self { states: VecDeque::new(), last_classify_ns: 0 }
    }

    /// Push a new state after each Kalman update.
    /// Converts ECEF velocity to ENU degrees/s before storing.
    pub fn push(&mut self, entry: &AircraftKalman) {
        let (lat, lon, alt_m) = entry.position_wgs84();
        let (vx, vy, vz)      = entry.velocity_ms();  // ECEF m/s
        let (lat_r, lon_r)    = (lat.to_radians(), lon.to_radians());

        // Full ECEF → ENU rotation (correct at all latitudes, including 50°N Europe)
        let east  = -lon_r.sin() * vx + lon_r.cos() * vy;
        let north = -lat_r.sin() * lon_r.cos() * vx
                    - lat_r.sin() * lon_r.sin() * vy
                    + lat_r.cos() * vz;
        let up    =  lat_r.cos() * lon_r.cos() * vx
                   + lat_r.cos() * lon_r.sin() * vy
                   + lat_r.sin() * vz;

        // Convert horizontal to deg/s (consistent with training data units)
        let vel_lat = north / 111_320.0;
        let vel_lon = east  / (111_320.0 * lat_r.cos());
        let vel_alt = up;   // keep in m/s — training va is already in m/s

        self.states.push_back((lat, lon, alt_m, vel_lat, vel_lon, vel_alt));
        if self.states.len() > 20 {
            self.states.pop_front();
        }
    }
}

// Rate-limit constant: classify at most once per 5 seconds per aircraft
const CLASSIFY_INTERVAL_NS: u64 = 5_000_000_000;
```

**In `KalmanRegistry::update()`**, after calling `entry.update(...)`:

```rust
// Add AircraftHistory alongside each AircraftKalman filter
history.push(&entry);   // fills every 20ms — always fresh

// Rate-limited classify dispatch
if now_ns.saturating_sub(history.last_classify_ns) > CLASSIFY_INTERVAL_NS {
    history.last_classify_ns = now_ns;
    let client  = anomaly_client.clone();
    let icao    = icao24.to_string();
    let hist_snap: Vec<_> = history.states.iter().copied().collect();
    let ws_tx2  = ws_tx.clone();
    tokio::spawn(async move {
        match client.classify(&icao, &hist_snap).await {
            Ok(resp) => {
                // update the in-flight AircraftState with anomaly label
                // re-broadcast updated state
            }
            Err(e) => tracing::warn!("anomaly classify error: {e}"),
        }
    });
}
```

---

### Phase 4 Verification Checklist

```
[ ] Mock spoofer (AABBCC) triggers spoof_flag=true after 30s simulation time
[ ] spoof_flag=true appears in WebSocket JSON, marker turns red in browser
[ ] Alert feed shows "SPOOF: AABBCC divergence Xm" with accurate value
[ ] curl -X POST http://localhost:8000/classify -H 'Content-Type: application/json' \
        -d '{"icao24":"TEST","states":[]}' returns {"anomaly_label":"unknown","confidence":0.0}
[ ] After python train.py runs, model.pt exists and /classify returns normal/anomalous
[ ] Anomaly labels appear on sidebar aircraft cards in browser
[ ] ML HTTP calls do not block MLAT pipeline (verify with timing logs)
```

---

### Phase 4 Docker Compose Update

**Do NOT train inside the Dockerfile.** Training produces a non-deterministic model per build, extends build times unpredictably, and cannot be version-controlled. Instead:

**Part A** — Updated `ml-service/Dockerfile` (no training step):
```dockerfile
# model.pt must exist in ml-service/ before docker compose build.
# Run train.sh once to generate it, then commit it.
# The startup handler degrades gracefully if absent.
FROM python:3.11-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY . .
EXPOSE 8000
# NO training step — model is either pre-trained (via train.sh) or service starts degraded
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

**Part B** — Add `Locus/train.sh` (run once before first deploy):
```bash
#!/usr/bin/env bash
# Run once to generate model.pt before deploying
set -e
cd ml-service
python train.py --epochs 20 --output model.pt
echo "model.pt generated. Commit this file before deploying."
```

**Part C** — Updated `main.py` startup handler (graceful degradation):
```python
@app.on_event("startup")
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
```

Add `MODEL_PATH` environment variable to Compose (no volume mount — model is baked into the image via `COPY`):
```yaml
ml-service:
  environment:
    - MODEL_PATH=/app/model.pt
```

---

## Phase 5 — Full Polish + Production Docker Compose

### Goal
Complete the frontend UI, finalize all Docker Compose configuration, add confidence ellipses to the map, and verify the full system starts from a clean clone in under 5 minutes.

### Visible Outcome
- **Browser**: Full professional UI — dark theme map, sidebar with aircraft list, alert feed, sensor health charts, confidence ellipses drawn around aircraft positions
- **Terminal**: `docker compose up --build` from clean clone completes and all services healthy within 5 minutes

### Estimated time: 2–3 hours

---

### Step 5.1 — Confidence Ellipses on Map

In `frontend/index.html`, when receiving an `AircraftState` with `confidence_ellipse_m` > 0, draw an ellipse:

```javascript
function updateAircraft(state) {
  // ... existing marker update code ...

  const ac = aircraft.get(state.icao24);

  // Draw/update confidence ellipse
  if (state.confidence_ellipse_m > 0) {
    const radiusDeg = state.confidence_ellipse_m / 111320; // rough m -> degrees
    if (ac.ellipse) map.removeLayer(ac.ellipse);
    ac.ellipse = L.circle([state.lat, state.lon], {
      radius: state.confidence_ellipse_m,
      color: state.spoof_flag ? '#f85149' : '#58a6ff',
      fillColor: state.spoof_flag ? '#f85149' : '#58a6ff',
      fillOpacity: 0.05,
      weight: 1,
    }).addTo(map);
  }
}
```

---

### Step 5.2 — Compute Confidence Ellipse Radius in Rust

In `mlat_solver.rs`, extract the 2D horizontal standard deviation from the LM covariance matrix:

```rust
// After solver converges, covariance matrix is available from the Jacobian:
// cov = (J^T J)^{-1} * sigma^2
// confidence_ellipse_m = 2-sigma radius of horizontal position uncertainty
// = 2 * sqrt(max eigenvalue of cov_xy)

fn confidence_ellipse_m(covariance: &Matrix3<f64>) -> f64 {
    // NOTE: Takes ECEF XY submatrix as proxy for horizontal uncertainty.
    // Correct approach: rotate full 3×3 ECEF covariance into ENU, take EN 2×2 submatrix.
    // This approximation underestimates horizontal uncertainty ~20–40% at European latitudes
    // due to ECEF-Z coupling into local horizontal plane. Acceptable for visualization.
    let cov_xy = covariance.fixed_slice::<2, 2>(0, 0);
    let trace = cov_xy[(0, 0)] + cov_xy[(1, 1)];
    let det = cov_xy[(0, 0)] * cov_xy[(1, 1)] - cov_xy[(0, 1)].powi(2);
    let lambda_max = trace / 2.0 + ((trace / 2.0).powi(2) - det).max(0.0).sqrt();
    2.0 * lambda_max.sqrt()  // 2-sigma ellipse semi-major axis in meters
}
```

---

### Step 5.3 — Final docker-compose.yml

**File**: `Locus/docker-compose.yml` (final version)

```yaml
version: "3.9"

services:
  rust-backend:
    build:
      context: ./rust-backend
      dockerfile: Dockerfile
    ports:
      - "9001:9001"
    environment:
      - RUST_LOG=locus_backend=info
      - MOCK_DATA=${MOCK_DATA:-true}
    command: >
      ./locus-backend
      --ws-addr=0.0.0.0:9001
      --ml-service-url=http://ml-service:8000
      ${MOCK_DATA:+--mock}
    volumes:
      - locus-ipc:/tmp/locus          # shared socket dir with go-ingestor
    depends_on:
      ml-service:
        condition: service_healthy
    restart: unless-stopped
    healthcheck:
      test: ["CMD-SHELL", "echo > /dev/tcp/localhost/9001 2>/dev/null && echo ok"]
      interval: 10s
      timeout: 5s
      retries: 5

  ml-service:
    build:
      context: ./ml-service
      dockerfile: Dockerfile
    ports:
      - "8000:8000"
    environment:
      - MODEL_PATH=/app/model.pt
    healthcheck:
      test: ["CMD", "curl", "-sf", "http://localhost:8000/health"]
      interval: 10s
      timeout: 5s
      retries: 5
      start_period: 30s
    restart: unless-stopped

  frontend:
    build:
      context: ./frontend
      dockerfile: Dockerfile
    ports:
      - "3000:80"
    restart: unless-stopped
    depends_on:
      - rust-backend

  go-ingestor:
    build:
      context: ./go-ingestor
      dockerfile: Dockerfile
    ports:
      - "61339:61339"
    volumes:
      - ./go-ingestor/.buyer-env:/app/.buyer-env:ro
      - ./go-ingestor/location-override.json:/app/location-override.json:ro
      - locus-ipc:/tmp/locus          # exposes ingestor.sock to rust-backend
    environment:
      - GO_LOG=info
    profiles:
      - live
    restart: unless-stopped

volumes:
  locus-ipc:
    driver: local
```

**File**: `Locus/docker-compose.live.yml` — `depends_on: go-ingestor` must be in an override file, not the base compose (profile-gated services cannot be depended on in the base file):
```yaml
# Use with: docker compose -f docker-compose.yml -f docker-compose.live.yml --profile live up
services:
  rust-backend:
    depends_on:
      go-ingestor:
        condition: service_started  # not service_healthy — Go has no health endpoint
```

**File**: `Locus/docker-compose.dev.yml`

```yaml
version: "3.9"

services:
  rust-backend:
    build:
      context: ./rust-backend
    volumes:
      - ./rust-backend/src:/app/src:rw
      - rust-target:/app/target
      - locus-ipc:/tmp/locus          # shared socket dir with go-ingestor
    environment:
      - RUST_LOG=locus_backend=trace
    command: >
      sh -c "cargo install cargo-watch 2>/dev/null;
             cargo watch -x 'run -- --mock --ws-addr=0.0.0.0:9001
             --ml-service-url=http://ml-service:8000'"

  go-ingestor:
    volumes:
      - ./go-ingestor/.buyer-env:/app/.buyer-env:ro
      - locus-ipc:/tmp/locus          # exposes ingestor.sock to rust-backend

  ml-service:
    volumes:
      - ./ml-service:/app:rw
    command: >
      uvicorn main:app --host 0.0.0.0 --port 8000 --reload

  frontend:
    volumes:
      - ./frontend/index.html:/usr/share/nginx/html/index.html:ro

volumes:
  locus-ipc:
    driver: local
  rust-target:
```

---

### Step 5.4 — .gitignore

**File**: `Locus/.gitignore`
```
# Credentials
go-ingestor/.buyer-env
go-ingestor/.seller-env

# Rust build artifacts
rust-backend/target/

# Python
ml-service/.venv/
ml-service/__pycache__/
ml-service/*_mean.npy
ml-service/*_std.npy
# NOTE: ml-service/*.pt is NOT gitignored — commit the trained model for reproducibility

# Docker volumes
.docker-data/

# IDE
.idea/
.vscode/
*.swp
```

---

### Step 5.5 — Environment and Quick-Start

**File**: `Locus/.env.example`
```env
# Set to true for mock mode (no live 4DSky network required)
MOCK_DATA=true

# Rust log level
RUST_LOG=locus_backend=info
```

**File**: `Locus/README.md` (minimal, not the focus of judging)
```markdown
# Locus

Real-time MLAT aircraft positioning with GPS spoof detection and AI anomaly classification.

## Quick Start (Mock Mode)

    cp .env.example .env
    docker compose up --build

Open http://localhost:3000 — aircraft will appear within 10 seconds.

## Live Mode (requires 4DSky credentials)

    cp go-ingestor/.buyer-env.example go-ingestor/.buyer-env
    # fill in your credentials
    docker compose -f docker-compose.yml -f docker-compose.live.yml --profile live up --build

## Development

    docker compose -f docker-compose.yml -f docker-compose.dev.yml up
```

---

### Phase 5 Verification Checklist

```
[ ] docker compose up --build completes without errors from clean directory
[ ] All 3 services (rust-backend, ml-service, frontend) pass health checks
[ ] Browser at http://localhost:3000 shows full UI within 10s of start
[ ] Aircraft markers appear with confidence ellipses
[ ] Spoofer aircraft (AABBCC) turns red and appears in alert feed after 30s
[ ] Anomaly labels appear on sidebar after ML service classifies tracks
[ ] Sensor health chart updates every 5 seconds
[ ] docker compose down && docker compose up (no --build) starts in <60s
[ ] .buyer-env is NOT committed (confirm with: git status Locus/go-ingestor/)
```

---

## End-to-End Verification Guide

After completing all phases, run this sequence to confirm the full system works:

### 1. Start the stack (mock mode)
```bash
cd /home/psychopunk_sage/dev/tools/Locus/Locus
cp .env.example .env
docker compose up --build -d
docker compose logs -f
```

Expected within 30s:
```
ml-service    | INFO:     Application startup complete.
rust-backend  | INFO Locus backend starting
rust-backend  | INFO WebSocket server listening on 0.0.0.0:9001
rust-backend  | INFO Received frame #100 from sensor 1001
```

### 2. Verify each service
```bash
# ML health
curl http://localhost:8000/health
# Expected: {"status":"ok","model_loaded":true}

# WebSocket output (requires wscat or websocat)
websocat ws://localhost:9001 | head -3
# Expected: JSON AircraftState objects

# Frontend
curl -s http://localhost:3000 | grep -c 'Locus'
# Expected: 1 (or more)
```

### 3. Verify MLAT accuracy (mock mode)
In logs, look for:
```
INFO mlat_solver: icao=4840D6 lat=51.012 lon=1.048 alt=10001m gdop=1.82
```

Compare against `MOCK_AIRCRAFT[0]` expected position for the elapsed time. Error should be <500m with σ=50ns noise.

### 4. Verify clock sync convergence
After ~30 seconds in logs:
```
DEBUG clock[1001→1002] offset=-3.2ns n=50
```

Offset magnitude should stabilize quickly (windowed median converges in ~5 observations). `is_converged=true` appears once `n >= WINDOW_SIZE/2 = 25`.

### 5. Verify spoof detection
After 30 seconds simulation time:
```
WARN spoof_detector: SPOOF DETECTED icao=AABBCC divergence=5241m
```

In browser: AABBCC marker is red, alert feed shows the event.

### 6. Verify ML classification
```bash
curl -X POST http://localhost:8000/classify \
  -H 'Content-Type: application/json' \
  -d '{
    "icao24": "TEST",
    "states": [
      {"lat":51.0,"lon":1.0,"alt_m":10000,"vel_lat":0.003,"vel_lon":0.005,"vel_alt":0.0}
    ]
  }'
```

Expected: `{"icao24":"TEST","anomaly_label":"normal","confidence":0.85}` (approximately)

### 7. Live mode smoke test (if credentials available)
```bash
docker compose --profile live up --build
# Watch for: "Stream established with peer QmXxx..."
# Then: "Received frame #1 from sensor <real_sensor_id>"
```

---

## Key Implementation Notes

### TDOA Sign Convention
Always compute TDOA as `t_sensor_i - t_sensor_ref`. The reference sensor (index 0 in the group) has TDOA = 0 by definition. Never pass the reference sensor's TDOA to the residual function.

### Units Throughout
- Timestamps: stored as nanoseconds (u64)
- TDOA passed to solver: converted to meters (`tdoa_ns * 0.299792458`)
- Solver state vector: ECEF meters
- Final output: WGS84 degrees + meters altitude
- Clock offsets: nanoseconds (f64)

### Numerical Stability in LM Solver
If the Jacobian condition number exceeds 1e10, the solver may diverge. Mitigate by:
1. Centering sensor positions around their centroid before solving (translate origin)
2. Using at least 4 sensors (3 independent TDOA equations for 3 unknowns + 1 redundant)
3. Regularizing: add a small damping factor (argmin LM does this automatically)

### Mock Mode Aircraft Coordinate Conversion
The mock generator computes TDOA using ECEF distances. Do NOT use the Haversine approximation for TDOA generation — it introduces systematic errors that will confuse the clock sync engine.

### CPR Decoding Note
If full CPR decoding is too complex to implement within time constraints, accept a simplified fallback: use the last two consecutive DF17 frames from the same ICAO24 (even/odd pair) and apply the standard CPR global decode. OpenSky's `pyModeS` library has a reference Python implementation to cross-check against.

### Logging Strategy
```
tracing::error! — unrecoverable errors (service will restart)
tracing::warn!  — recoverable anomalies (spoof detected, solver diverged)
tracing::info!  — one-line summary per MLAT solution (every 10th to avoid flood)
tracing::debug! — per-frame events, clock offsets (normal operation detail)
tracing::trace! — raw bytes, Jacobian values (debugging only)
```

Set `RUST_LOG=locus_backend=info` in production, `=debug` for development.

---

*End of Locus Implementation Plan — Version 1.0*
*Lead Architect: Locus Team | Hackathon: 4DSky MLAT by Neuron Innovations / Hedera*
