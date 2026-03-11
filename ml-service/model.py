import torch
import torch.nn as nn


class AircraftLSTM(nn.Module):
    """
    LSTM anomaly classifier for aircraft tracks.

    Input:  (batch, seq_len=20, features=6)
            features = [lat, lon, alt_m, vel_lat, vel_lon, vel_alt]
    Output: (batch, 2) raw logits  →  softmax gives [p_normal, p_anomalous]
    """

    def __init__(
        self,
        input_size: int = 6,
        hidden_size: int = 128,
        num_layers: int = 2,
        dropout: float = 0.3,
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
        self.fc = nn.Linear(hidden_size, 2)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        out, _ = self.lstm(x)
        out = self.dropout(out[:, -1, :])  # last timestep
        return self.fc(out)
