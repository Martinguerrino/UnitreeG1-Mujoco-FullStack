#!/usr/bin/env bash
# Installs optional CUDA packages into the configured project interpreter.
set -euo pipefail
G1_GPU_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$G1_GPU_ROOT/scripts/python_env.sh"
if (($#)); then
  echo 'Uso: bash scripts/setup_gpu.sh (usa G1_PYTHON como entorno CPU base)' >&2
  exit 2
fi
echo "Instalando CUDA en: $G1_PYTHON"
"$G1_PYTHON" -m pip install 'torch==2.7.1+cu128' 'torchvision==0.22.1+cu128' \
  --index-url https://download.pytorch.org/whl/cu128
"$G1_PYTHON" -m pip install -r "$G1_GPU_ROOT/requirements-gpu.txt" \
  -c "$G1_GPU_ROOT/requirements-runtime.txt"
"$G1_PYTHON" "$G1_GPU_ROOT/scripts/check_gpu.py"
echo 'Listo: los lanzadores detectan automáticamente las dependencias GPU del proyecto.'
