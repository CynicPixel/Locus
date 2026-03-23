"""
Train the AircraftLSTM anomaly classifier.

Generates synthetic flight data (normal + spoofed tracks) when OpenSky is
unavailable, then trains and saves model.pt alongside normalisation arrays.

Usage:
    python train.py --epochs 20 --output model.pt
"""
import argparse
import logging

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

from model import AircraftLSTM
from constants import (
    SEQ_LEN, NUM_FEATURES, NUM_CLASSES,
    LAT_MIN, LAT_MAX, LON_MIN, LON_MAX, ALT_MIN, ALT_MAX,
    VEL_LAT_MIN, VEL_LAT_MAX, VEL_LON_MIN, VEL_LON_MAX, VEL_ALT_MIN, VEL_ALT_MAX,
    GDOP_MIN, GDOP_MAX, DIVERGENCE_MIN, DIVERGENCE_MAX, TRACK_STEP_SCALE,
    ANOMALY_WINDOW_MIN, ANOMALY_WINDOW_MAX,
    POSITION_JUMP_MIN, POSITION_JUMP_MAX,
    ANOMALY_GDOP_MIN, ANOMALY_GDOP_MAX, ANOMALY_DIVERGENCE_MIN, ANOMALY_DIVERGENCE_MAX,
    VELOCITY_FLIP_MULTIPLIER,
    ALT_JUMP_MIN, ALT_JUMP_MAX,
    N_NORMAL, N_ANOMALOUS, NUM_ANOMALY_TYPES,
    NORMAL_LABEL, ANOMALY_LABEL,
    STD_EPSILON, TRAIN_SPLIT, TRAIN_BATCH_SIZE, VAL_BATCH_SIZE,
    LEARNING_RATE, WEIGHT_DECAY, NORMAL_CLASS_WEIGHT, ANOMALY_CLASS_WEIGHT,
    GRADIENT_CLIP, DEFAULT_EPOCHS,
)

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
logger = logging.getLogger("locus-train")

FEATURES = NUM_FEATURES  # lat, lon, alt_m, vel_lat, vel_lon, vel_alt, gdop, divergence_m


# ---------------------------------------------------------------------------
# Synthetic dataset
# ---------------------------------------------------------------------------


def _normal_track() -> np.ndarray:
    lat = np.random.uniform(LAT_MIN, LAT_MAX)
    lon = np.random.uniform(LON_MIN, LON_MAX)
    alt = np.random.uniform(ALT_MIN, ALT_MAX)
    vl = np.random.uniform(VEL_LAT_MIN, VEL_LAT_MAX)
    vlo = np.random.uniform(VEL_LON_MIN, VEL_LON_MAX)
    va = np.random.uniform(VEL_ALT_MIN, VEL_ALT_MAX)
    seq = []
    for _ in range(SEQ_LEN):
        seq.append([lat, lon, alt, vl, vlo, va])
        lat += vl * TRACK_STEP_SCALE
        lon += vlo * TRACK_STEP_SCALE
        alt += va * TRACK_STEP_SCALE
    track = np.array(seq, dtype=np.float32)
    gdop = np.random.uniform(GDOP_MIN, GDOP_MAX, (SEQ_LEN, 1)).astype(np.float32)
    divergence = np.random.uniform(DIVERGENCE_MIN, DIVERGENCE_MAX, (SEQ_LEN, 1)).astype(np.float32)
    return np.concatenate([track, gdop, divergence], axis=1)  # (SEQ_LEN, NUM_FEATURES)


def _anomaly_position_jump() -> np.ndarray:
    """GPS spoofing: sudden lat/lon teleport, velocity unchanged."""
    track = _normal_track()
    jump_idx = np.random.randint(ANOMALY_WINDOW_MIN, ANOMALY_WINDOW_MAX)
    track[jump_idx:, 0] += np.random.uniform(POSITION_JUMP_MIN, POSITION_JUMP_MAX)
    track[jump_idx:, 1] += np.random.uniform(POSITION_JUMP_MIN, POSITION_JUMP_MAX)
    track[:, 6] = np.random.uniform(ANOMALY_GDOP_MIN, ANOMALY_GDOP_MAX, SEQ_LEN).astype(np.float32)
    track[:, 7] = np.random.uniform(ANOMALY_DIVERGENCE_MIN, ANOMALY_DIVERGENCE_MAX, SEQ_LEN).astype(np.float32)
    return track


def _anomaly_velocity_flip() -> np.ndarray:
    """Spoofed track: sudden velocity reversal (characteristic of replay attack)."""
    track = _normal_track()
    flip_idx = np.random.randint(ANOMALY_WINDOW_MIN, ANOMALY_WINDOW_MAX)
    track[flip_idx:, 3] *= VELOCITY_FLIP_MULTIPLIER  # negate vel_lat
    track[flip_idx:, 4] *= VELOCITY_FLIP_MULTIPLIER  # negate vel_lon
    track[:, 6] = np.random.uniform(ANOMALY_GDOP_MIN, ANOMALY_GDOP_MAX, SEQ_LEN).astype(np.float32)
    track[:, 7] = np.random.uniform(ANOMALY_DIVERGENCE_MIN, ANOMALY_DIVERGENCE_MAX, SEQ_LEN).astype(np.float32)
    return track


def _anomaly_alt_teleport() -> np.ndarray:
    """ADS-B altitude spoofing: instant vertical jump."""
    track = _normal_track()
    jump_idx = np.random.randint(ANOMALY_WINDOW_MIN, ANOMALY_WINDOW_MAX)
    track[jump_idx:, 2] += np.random.uniform(ALT_JUMP_MIN, ALT_JUMP_MAX)
    track[:, 6] = np.random.uniform(ANOMALY_GDOP_MIN, ANOMALY_GDOP_MAX, SEQ_LEN).astype(np.float32)
    track[:, 7] = np.random.uniform(ANOMALY_DIVERGENCE_MIN, ANOMALY_DIVERGENCE_MAX, SEQ_LEN).astype(np.float32)
    return track


def _anomaly_frozen() -> np.ndarray:
    """Dead ADS-B replay: position freezes while velocity continues."""
    track = _normal_track()
    freeze_idx = np.random.randint(ANOMALY_WINDOW_MIN, ANOMALY_WINDOW_MAX)
    frozen_lat = track[freeze_idx, 0]
    frozen_lon = track[freeze_idx, 1]
    track[freeze_idx:, 0] = frozen_lat
    track[freeze_idx:, 1] = frozen_lon
    track[:, 6] = np.random.uniform(ANOMALY_GDOP_MIN, ANOMALY_GDOP_MAX, SEQ_LEN).astype(np.float32)
    track[:, 7] = np.random.uniform(ANOMALY_DIVERGENCE_MIN, ANOMALY_DIVERGENCE_MAX, SEQ_LEN).astype(np.float32)
    return track


def generate_dataset(n_normal: int = N_NORMAL, n_anomalous: int = N_ANOMALOUS):
    # Split anomalies equally across NUM_ANOMALY_TYPES types
    per_type = n_anomalous // NUM_ANOMALY_TYPES
    anomalies = (
        [_anomaly_position_jump() for _ in range(per_type)]
        + [_anomaly_velocity_flip() for _ in range(per_type)]
        + [_anomaly_alt_teleport() for _ in range(per_type)]
        + [_anomaly_frozen() for _ in range(n_anomalous - 3 * per_type)]
    )
    X = np.array([_normal_track() for _ in range(n_normal)] + anomalies)
    y = np.array([NORMAL_LABEL] * n_normal + [ANOMALY_LABEL] * n_anomalous)
    idx = np.random.permutation(len(X))
    return X[idx], y[idx]


# ---------------------------------------------------------------------------
# Training loop
# ---------------------------------------------------------------------------


def train(epochs: int, output_path: str) -> None:
    logger.info("Generating synthetic training dataset…")
    X, y = generate_dataset()

    X_mean = X.mean(axis=(0, 1), keepdims=True)
    X_std = X.std(axis=(0, 1), keepdims=True) + STD_EPSILON
    X_norm = (X - X_mean) / X_std

    mean_path = output_path.replace(".pt", "_mean.npy")
    std_path = output_path.replace(".pt", "_std.npy")
    np.save(mean_path, X_mean)
    np.save(std_path, X_std)

    split = int(TRAIN_SPLIT * len(X_norm))
    X_train, X_val = X_norm[:split], X_norm[split:]
    y_train, y_val = y[:split], y[split:]

    dl_train = DataLoader(
        TensorDataset(torch.FloatTensor(X_train), torch.LongTensor(y_train)),
        batch_size=TRAIN_BATCH_SIZE,
        shuffle=True,
    )
    dl_val = DataLoader(
        TensorDataset(torch.FloatTensor(X_val), torch.LongTensor(y_val)),
        batch_size=VAL_BATCH_SIZE,
    )

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = AircraftLSTM().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=LEARNING_RATE, weight_decay=WEIGHT_DECAY)
    # Class weights compensate for 80/20 imbalance: penalise missed anomalies 4×
    weight = torch.tensor([NORMAL_CLASS_WEIGHT, ANOMALY_CLASS_WEIGHT], device=device)
    criterion = nn.CrossEntropyLoss(weight=weight)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, epochs)

    for epoch in range(1, epochs + 1):
        model.train()
        total_loss = 0.0
        for xb, yb in dl_train:
            xb, yb = xb.to(device), yb.to(device)
            optimizer.zero_grad()
            loss = criterion(model(xb), yb)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), GRADIENT_CLIP)
            optimizer.step()
            total_loss += loss.item()
        scheduler.step()

        model.eval()
        correct = total = 0
        with torch.no_grad():
            for xb, yb in dl_val:
                xb, yb = xb.to(device), yb.to(device)
                correct += (model(xb).argmax(1) == yb).sum().item()
                total += len(yb)
        logger.info(
            f"Epoch {epoch}/{epochs}  loss={total_loss / len(dl_train):.4f}"
            f"  val_acc={correct / total:.3f}"
        )

    torch.save(model.state_dict(), output_path)
    logger.info(f"Model saved → {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--epochs", type=int, default=DEFAULT_EPOCHS)
    parser.add_argument("--output", type=str, default="model.pt")
    args = parser.parse_args()
    train(args.epochs, args.output)
