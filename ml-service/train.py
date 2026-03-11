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

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
logger = logging.getLogger("locus-train")

SEQ_LEN = 20
FEATURES = 6  # lat, lon, alt_m, vel_lat, vel_lon, vel_alt


# ---------------------------------------------------------------------------
# Synthetic dataset
# ---------------------------------------------------------------------------


def _normal_track() -> np.ndarray:
    lat = np.random.uniform(48.0, 54.0)
    lon = np.random.uniform(-2.0, 12.0)
    alt = np.random.uniform(5000.0, 12000.0)
    vl = np.random.uniform(0.001, 0.005)
    vlo = np.random.uniform(0.002, 0.008)
    va = np.random.uniform(-2.0, 2.0)
    seq = []
    for _ in range(SEQ_LEN):
        seq.append([lat, lon, alt, vl, vlo, va])
        lat += vl * 0.1
        lon += vlo * 0.1
        alt += va * 0.1
    return np.array(seq, dtype=np.float32)


def _anomalous_track() -> np.ndarray:
    track = _normal_track()
    jump_idx = np.random.randint(5, 15)
    track[jump_idx:, 0] += np.random.uniform(0.02, 0.08)  # lat jump
    track[jump_idx:, 1] += np.random.uniform(0.02, 0.08)  # lon jump
    return track


def generate_dataset(n_normal: int = 800, n_anomalous: int = 200):
    X = np.array(
        [_normal_track() for _ in range(n_normal)]
        + [_anomalous_track() for _ in range(n_anomalous)]
    )
    y = np.array([0] * n_normal + [1] * n_anomalous)
    idx = np.random.permutation(len(X))
    return X[idx], y[idx]


# ---------------------------------------------------------------------------
# Training loop
# ---------------------------------------------------------------------------


def train(epochs: int, output_path: str) -> None:
    logger.info("Generating synthetic training dataset…")
    X, y = generate_dataset()

    X_mean = X.mean(axis=(0, 1), keepdims=True)
    X_std = X.std(axis=(0, 1), keepdims=True) + 1e-8
    X_norm = (X - X_mean) / X_std

    mean_path = output_path.replace(".pt", "_mean.npy")
    std_path = output_path.replace(".pt", "_std.npy")
    np.save(mean_path, X_mean)
    np.save(std_path, X_std)

    split = int(0.8 * len(X_norm))
    X_train, X_val = X_norm[:split], X_norm[split:]
    y_train, y_val = y[:split], y[split:]

    dl_train = DataLoader(
        TensorDataset(torch.FloatTensor(X_train), torch.LongTensor(y_train)),
        batch_size=32,
        shuffle=True,
    )
    dl_val = DataLoader(
        TensorDataset(torch.FloatTensor(X_val), torch.LongTensor(y_val)),
        batch_size=64,
    )

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
            loss = criterion(model(xb), yb)
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
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--output", type=str, default="model.pt")
    args = parser.parse_args()
    train(args.epochs, args.output)
