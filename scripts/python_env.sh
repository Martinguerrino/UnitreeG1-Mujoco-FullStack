#!/usr/bin/env bash
# Source before resetting PATH for ROS. No Conda activation is necessary.
G1_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export G1_PYTHON
G1_PYTHON="$(/usr/bin/python3 "$G1_PROJECT_ROOT/g1_backend/python_runtime.py")" || return 1
