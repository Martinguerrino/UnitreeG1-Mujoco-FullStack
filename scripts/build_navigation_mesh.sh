#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TASK_SOURCE="$ROOT_DIR/scripts/native/cloud_mesh.cpp"
TASK_BINARY="$ROOT_DIR/artifacts/slam/bin/g1_cloud_mesh"
mkdir -p "$(dirname "$TASK_BINARY")"
if [[ ! -x "$TASK_BINARY" || "$TASK_SOURCE" -nt "$TASK_BINARY" ]]; then
TASK_PCL_CONFIG=$(pkg-config --cflags --libs pcl_surface pcl_features pcl_filters pcl_io)
read -r -a TASK_PCL_FLAGS <<< "$TASK_PCL_CONFIG"
g++ -std=c++17 -O2 "$TASK_SOURCE" "${TASK_PCL_FLAGS[@]}" -o "$TASK_BINARY.tmp"
mv "$TASK_BINARY.tmp" "$TASK_BINARY"
fi
TASK_PREFIX="$ROOT_DIR/artifacts/slam/rootfs/opt/ros/jazzy"
if [[ ! -d "$TASK_PREFIX/include" ]]; then
  TASK_PREFIX=/opt/ros/jazzy
fi
TASK_RTABMAP_HEADERS=("$TASK_PREFIX"/include/rtabmap-*)
TASK_CODEC="$ROOT_DIR/artifacts/slam/bin/librtabmap_image.so"
TASK_CODEC_SOURCE="$ROOT_DIR/scripts/native/rtabmap_image.cpp"
if [[ ! -f "$TASK_CODEC" || "$TASK_CODEC_SOURCE" -nt "$TASK_CODEC" ]]; then
  TASK_OPENCV_CONFIG=$(pkg-config --cflags --libs opencv4)
  read -r -a TASK_OPENCV_FLAGS <<< "$TASK_OPENCV_CONFIG"
  g++ -std=c++17 -O2 -shared -fPIC "$TASK_CODEC_SOURCE" \
    -I"${TASK_RTABMAP_HEADERS[0]}" -L"$TASK_PREFIX/lib/x86_64-linux-gnu" \
    -Wl,-rpath,"$TASK_PREFIX/lib/x86_64-linux-gnu" -lrtabmap_core \
    "${TASK_OPENCV_FLAGS[@]}" -o "$TASK_CODEC.tmp"
  mv "$TASK_CODEC.tmp" "$TASK_CODEC"
fi
