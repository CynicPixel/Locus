import torch
import torch.nn as nn
from constants import NUM_FEATURES, HIDDEN_SIZE, NUM_LSTM_LAYERS, DROPOUT_RATE, NUM_CLASSES


class AircraftLSTM(nn.Module):
    """
    LSTM anomaly classifier for aircraft tracks.

    Input:  (batch, seq_len=20, features=8)
            features = [lat, lon, alt_m, vel_lat, vel_lon, vel_alt, gdop, divergence_m]
    Output: (batch, 2) raw logits  →  softmax gives [p_normal, p_anomalous]
    """

    def __init__(
        self,
        input_size: int = NUM_FEATURES,
        hidden_size: int = HIDDEN_SIZE,
        num_layers: int = NUM_LSTM_LAYERS,
        dropout: float = DROPOUT_RATE,
    ):
        super().__init__()
        self.lstm = nn.LSTM(
            input_size,
            hidden_size,
            num_layers,
            batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0,
        )
        self.dropout = nn.Dropout(dropout)
        self.fc = nn.Linear(hidden_size, NUM_CLASSES)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        out, _ = self.lstm(x)
        out = self.dropout(out[:, -1, :])  # last timestep
        return self.fc(out)
