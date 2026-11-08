#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$project_root/scripts/python_env.sh"
cd "$project_root"
if ! "$G1_PYTHON" -c 'import torch; raise SystemExit(torch.version.cuda is None)'; then
  "$G1_PYTHON" -m pip install torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cpu
fi
"$G1_PYTHON" -m pip install -r requirements-okura.txt -c requirements-runtime.txt
"$G1_PYTHON" -m experiments.okura.checkpoint --download --report artifacts/okura/checkpoint_report.json
