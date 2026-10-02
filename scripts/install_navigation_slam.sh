#!/usr/bin/env bash
# Ubuntu 24.04 / ROS Jazzy; installs SLAM into artifacts/slam, without sudo.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TASK_WS="$ROOT_DIR/artifacts/slam"
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH LD_LIBRARY_PATH
mkdir -p "$TASK_WS/debs" "$TASK_WS/src" "$TASK_WS/vendor"
set +u
source /opt/ros/jazzy/setup.bash
set -u
# Resolve only missing dependencies according to the host package database.
apt-get -s install ros-jazzy-rtabmap-ros python3-scipy python3-opencv libpcl-dev python3-dev | awk '/^Inst / {print $2}' > "$TASK_WS/debs/packages.txt"
mapfile -t TASK_PACKAGES < "$TASK_WS/debs/packages.txt"
if ((${#TASK_PACKAGES[@]})); then
  (cd "$TASK_WS/debs"; apt-get download "${TASK_PACKAGES[@]}")
fi
for deb in "$TASK_WS/debs"/*.deb; do
  [[ -f "$deb" ]] && dpkg-deb -x "$deb" "$TASK_WS/rootfs"
done
if [[ ! -d "$TASK_WS/src/fast_lio/.git" ]]; then
  git clone https://github.com/Ericsii/FAST_LIO_ROS2.git "$TASK_WS/src/fast_lio"
fi
git -C "$TASK_WS/src/fast_lio" checkout 2fffc570a25d0df172720bac034fbdb6a13d2162
git -C "$TASK_WS/src/fast_lio" submodule update --init --recursive
if [[ ! -d "$TASK_WS/vendor/livox_ros_driver2/.git" ]]; then
  git clone https://github.com/Livox-SDK/livox_ros_driver2.git "$TASK_WS/vendor/livox_ros_driver2"
fi
git -C "$TASK_WS/vendor/livox_ros_driver2" checkout 21445540f0d100dc86a7e6df312dd70bbdb4afdf
# FAST-LIO links Livox message types even when receiving PointCloud2. Build the
# official interfaces only: the simulator does not need the hardware SDK/driver.
mkdir -p "$TASK_WS/src/livox_ros_driver2/msg"
cp "$TASK_WS/vendor/livox_ros_driver2/msg/"*.msg "$TASK_WS/src/livox_ros_driver2/msg/"
cp "$TASK_WS/vendor/livox_ros_driver2/LICENSE.txt" "$TASK_WS/src/livox_ros_driver2/LICENSE.txt"
cp "$ROOT_DIR/configs/navigation/livox_interfaces/"* "$TASK_WS/src/livox_ros_driver2/"
/usr/bin/python3 "$ROOT_DIR/scripts/prepare_fast_lio.py" "$TASK_WS/src/fast_lio"
set +u
source "$ROOT_DIR/scripts/slam_env.sh"
set -u
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --base-paths "$TASK_WS/src" \
  --build-base "$TASK_WS/build" --install-base "$TASK_WS/install" \
  --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
bash "$ROOT_DIR/scripts/install_navigation_tsdf.sh"
printf 'SLAM installed in %s\n' "$TASK_WS"
