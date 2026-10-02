#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TASK_SOURCE="$ROOT_DIR/scripts/native/cloud_mesh.cpp"
TASK_BINARY="$ROOT_DIR/artifacts/slam/bin/g1_cloud_mesh"
if [[ -x "$TASK_BINARY" && "$TASK_BINARY" -nt "$TASK_SOURCE" ]]; then exit 0; fi
mkdir -p "$(dirname "$TASK_BINARY")"
TASK_PCL_CONFIG=$(pkg-config --cflags --libs pcl_surface pcl_features pcl_filters pcl_io)
read -r -a TASK_PCL_FLAGS <<< "$TASK_PCL_CONFIG"
g++ -std=c++17 -O2 "$TASK_SOURCE" "${TASK_PCL_FLAGS[@]}" -o "$TASK_BINARY.tmp"
mv "$TASK_BINARY.tmp" "$TASK_BINARY"
