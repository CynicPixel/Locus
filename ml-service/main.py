"""
FastAPI anomaly-classification service for Locus MLAT.

Endpoints:
    GET  /health          — liveness check
    POST /classify        — returns anomaly label + confidence for an aircraft track
    POST /train           — background retrain trigger
"""

import asyncio
import logging
import os
from pathlib import Path
from typing import List, Optional

import numpy as np
import torch
from fastapi import BackgroundTasks, FastAPI, HTTPException
from model import AircraftLSTM
from pydantic import BaseModel
from constants import (
    SEQ_LEN, NUM_FEATURES, ANOMALY_CLASS_INDEX, ANOMALY_LABEL,
    DEFAULT_RETRAIN_EPOCHS,
)

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
logger = logging.getLogger("locus-ml")

FEATURES = NUM_FEATURES  # lat, lon, alt_m, vel_lat, vel_lon, vel_alt, gdop, divergence_m

MODEL_PATH = Path(os.getenv("MODEL_PATH", "model.pt"))
MEAN_PATH = MODEL_PATH.with_name(MODEL_PATH.stem + "_mean.npy")
STD_PATH = MODEL_PATH.with_name(MODEL_PATH.stem + "_std.npy")

app = FastAPI(title="Locus ML Service")

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model: Optional[AircraftLSTM] = None
model_mean: Optional[np.ndarray] = None
model_std: Optional[np.ndarray] = None


def _load_model() -> None:
    global model, model_mean, model_std
    if not MODEL_PATH.exists():
        logger.warning("model.pt not found — service running in degraded mode")
        return
    m = AircraftLSTM().to(device)
    m.load_state_dict(torch.load(MODEL_PATH, map_location=device))
    m.eval()
    model = m
    if MEAN_PATH.exists() and STD_PATH.exists():
        model_mean = np.load(MEAN_PATH)
        model_std = np.load(STD_PATH)
    logger.info("Model loaded from %s", MODEL_PATH)


@app.on_event("startup")
async def startup_event() -> None:
    _load_model()


# ---------------------------------------------------------------------------
# Schemas
# ---------------------------------------------------------------------------


class KalmanStatePoint(BaseModel):
    lat: float
    lon: float
    alt_m: float
    vel_lat: float
    vel_lon: float
    vel_alt: float
    gdop: float = 0.0
    divergence_m: float = 0.0


class ClassifyRequest(BaseModel):
    icao24: str
    states: List[KalmanStatePoint]


class ClassifyResponse(BaseModel):
    icao24: str
    anomaly_label: str  # "normal" or "anomalous"
    confidence: float


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------


@app.get("/health")
async def health():
    return {"status": "ok", "model_loaded": model is not None}


@app.post("/classify", response_model=ClassifyResponse)
def classify(req: ClassifyRequest):
    if model is None:
        raise HTTPException(status_code=503, detail="Model not loaded")

    raw = np.array(
        [[s.lat, s.lon, s.alt_m, s.vel_lat, s.vel_lon, s.vel_alt, s.gdop, s.divergence_m] for s in req.states],
        dtype=np.float32,
    )

    # Pad or trim to SEQ_LEN
    if len(raw) < SEQ_LEN:
        pad = np.zeros((SEQ_LEN - len(raw), FEATURES), dtype=np.float32)
        raw = np.concatenate([pad, raw], axis=0)
    else:
        raw = raw[-SEQ_LEN:]  # keep the most recent SEQ_LEN steps

    # Normalise
    if model_mean is not None and model_std is not None:
        raw = (raw - model_mean[0]) / model_std[0]

    x = torch.FloatTensor(raw).unsqueeze(0).to(device)  # (1, SEQ_LEN, FEATURES)
    with torch.no_grad():
        logits = model(x)
        probs = torch.softmax(logits, dim=1)[0]
        label_idx = int(probs.argmax().item())
        confidence = float(probs[label_idx].item())
        label = "anomalous" if label_idx == ANOMALY_CLASS_INDEX else "normal"

    return ClassifyResponse(
        icao24=req.icao24, anomaly_label=label, confidence=confidence
    )


@app.post("/train")
async def trigger_train(background_tasks: BackgroundTasks):
    background_tasks.add_task(_run_train)
    return {"status": "training started"}


async def _run_train() -> None:
    proc = await asyncio.create_subprocess_exec(
        "python", "train.py", "--epochs", str(DEFAULT_RETRAIN_EPOCHS), "--output", str(MODEL_PATH)
    )
    await proc.wait()
    _load_model()
    logger.info("Retrain complete")
