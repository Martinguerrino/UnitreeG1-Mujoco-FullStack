#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PERCEPTION_PYTHON="${PERCEPTION_PYTHON:-/home/martin/miniforge3/envs/mujoco-unitree/bin/python}"
"$PERCEPTION_PYTHON" -m pip install --target "$ROOT_DIR/artifacts/perception/python" \
  --no-deps -r "$ROOT_DIR/requirements-perception.txt"
"$PERCEPTION_PYTHON" "$ROOT_DIR/scripts/train_perception_yolo.py" "$@"
