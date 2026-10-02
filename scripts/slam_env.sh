#!/usr/bin/env bash
# Source after /opt/ros/jazzy/setup.bash. Dependencies stay inside the project.
SLAM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../artifacts/slam" && pwd)"
SLAM_PREFIX="$SLAM_ROOT/rootfs/opt/ros/jazzy"
if [[ -d "$SLAM_PREFIX" ]]; then
  export AMENT_PREFIX_PATH="$SLAM_PREFIX:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="$SLAM_PREFIX:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$SLAM_PREFIX/lib:$SLAM_PREFIX/lib/x86_64-linux-gnu:$SLAM_ROOT/rootfs/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$SLAM_PREFIX/lib/python3.12/site-packages:$SLAM_ROOT/rootfs/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
  export PATH="$SLAM_PREFIX/bin:$PATH"
fi
if [[ -f "$SLAM_ROOT/install/local_setup.bash" ]]; then
  source "$SLAM_ROOT/install/local_setup.bash"
fi
