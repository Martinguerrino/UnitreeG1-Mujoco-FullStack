#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TASK_VENV="$ROOT_DIR/artifacts/slam/tsdf_venv"
/usr/bin/python3 -m venv --system-site-packages "$TASK_VENV"
"$TASK_VENV/bin/python" -m pip install 'numpy<2' 'open3d==0.19.0'
bash "$ROOT_DIR/scripts/build_navigation_mesh.sh"
