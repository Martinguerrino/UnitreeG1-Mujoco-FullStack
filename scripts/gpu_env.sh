#!/usr/bin/env bash
# Source this before launchers. G1_GPU=off disables acceleration; a fresh shell
# restores the configured interpreter. .g1-runtime.json is never rewritten.
G1_GPU_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$G1_GPU_ROOT/scripts/python_env.sh"
G1_GPU_SITE="$("$G1_PYTHON" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
export G1_GPU_PYTHON_PATH="$G1_GPU_SITE"
export PYTHONPATH="$G1_GPU_SITE:${PYTHONPATH:-}"
# Interpreter used by existing ROS camera/perception entry points.
export PERCEPTION_PYTHON="$G1_PYTHON"
