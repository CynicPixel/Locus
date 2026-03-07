## Features

- Live ingestion of Mode-S streams from multiple sensors via 4DSky/Neuron SDK
- Message correlation engine (group same transmission across sensors by ICAO24 + content hash + time window)
- Self-calibrating clock sync using ADS-B beacon aircraft as ground truth
- MLAT solver (Levenberg-Marquardt) with GDOP filtering
- Kalman filter track smoothing per aircraft
- Confidence ellipse generation (uncertainty visualization)
- GPS spoofing detection (MLAT position vs claimed ADS-B position divergence)
- Real-time trajectory anomaly classification (loitering, unexpected vectors, suspicious approach)
- Live map dashboard with aircraft tracks, sensor positions, confidence ellipses
- Sensor health panel (clock offset drift, signal quality per sensor)
- Alert system for spoofing/anomaly events

---

## Tech Stack by Component

**Stream Ingestion (Go)**

- 4DSky SDK (Go) — mandatory, run as-is
- Stdout pipe to Rust backend

**Core Backend (Rust)**

- `tokio` — async runtime
- `argmin` — Levenberg-Marquardt solver
- `nalgebra` — matrix math for Kalman filter
- `serde` / `serde_json` — message parsing
- `tokio-tungstenite` — WebSocket server to frontend

**ML Anomaly Engine (Python)**

- `PyTorch` — LSTM trajectory model
- `OpenSky Network REST API` — historical training data
- `FastAPI` — exposes inference as HTTP endpoint to Rust backend

**Frontend**

- Leaflet.js — live map
- Chart.js — sensor health / clock sync panel
- Vanilla JS WebSocket client

---

## Component Connections

```
4DSky SDK (Go)
    │ stdout pipe (raw ModeS bytes)
    ▼
Rust Backend
    ├── Message Correlator
    │       │ correlated TDOA groups
    │       ▼
    │   Clock Sync Engine ←── beacon aircraft (known position)
    │       │ corrected timestamps
    │       ▼
    │   MLAT Solver (LM)
    │       │ raw position estimate + GDOP score
    │       ▼
    │   Kalman Filter
    │       │ smoothed track + confidence ellipse
    │       ▼
    │   Spoof Detector ←── ADS-B claimed position (from same stream)
    │       │ spoof flag + divergence score
    │       ▼
    │   HTTP Client ──────────────────────► Python ML Service (FastAPI)
    │                                            │ anomaly classification
    │                ◄───────────────────────────┘
    │       │ enriched aircraft state
    │       ▼
    WebSocket Server
          │
          ▼
    Frontend (Leaflet + Chart.js)
          ├── Live map (tracks, ellipses, spoof alerts, anomaly flags)
          └── Sensor panel (clock drift, GDOP, signal quality)
```
