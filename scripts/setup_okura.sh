#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${CONDA_DEFAULT_ENV:-}" != mujoco-unitree ]]; then
  echo 'Primero ejecutá: conda activate mujoco-unitree' >&2
  exit 1
fi
cd "$project_root"
python -m pip install torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r requirements-okura.txt
python -m experiments.okura.checkpoint --download --report artifacts/okura/checkpoint_report.json
