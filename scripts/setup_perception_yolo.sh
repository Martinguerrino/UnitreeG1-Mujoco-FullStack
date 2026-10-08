#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT_DIR/scripts/python_env.sh"
PERCEPTION_PYTHON="${PERCEPTION_PYTHON:-$G1_PYTHON}"
"$PERCEPTION_PYTHON" -m pip install --upgrade --target "$ROOT_DIR/artifacts/perception/python" \
  --no-deps -r "$ROOT_DIR/requirements-perception.txt"
if [[ "${1:-}" == --train ]]; then
  shift
  "$PERCEPTION_PYTHON" -m pip install torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cpu
  "$PERCEPTION_PYTHON" "$ROOT_DIR/scripts/train_perception_yolo.py" "$@"
elif (($#)); then
  echo 'Uso: setup_perception_yolo.sh [--train [opciones de entrenamiento]]' >&2
  exit 2
else
  "$PERCEPTION_PYTHON" "$ROOT_DIR/scripts/install_perception_model.py"
fi
