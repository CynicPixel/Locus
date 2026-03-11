#!/usr/bin/env bash
# train.sh — bootstrap ML model inside the ml-service container (or locally).
# Usage: ./train.sh [--epochs N]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ML_DIR="$SCRIPT_DIR/ml-service"

EPOCHS="${1:-20}"
if [[ "$1" == "--epochs" ]]; then
  EPOCHS="$2"
fi

echo "==> Training LSTM model (epochs=$EPOCHS)"
cd "$ML_DIR"

if [[ -f .venv/bin/activate ]]; then
  # shellcheck source=/dev/null
  source .venv/bin/activate
fi

python train.py --epochs "$EPOCHS" --output model.pt
echo "==> Done.  model.pt written to $ML_DIR"
