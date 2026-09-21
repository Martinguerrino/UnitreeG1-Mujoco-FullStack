#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# ROS uses system Python/FastDDS; SDK2 uses Conda in a separate process.
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
unset PYTHONHOME PYTHONPATH LD_LIBRARY_PATH
unset GTK_PATH GTK_EXE_PREFIX GTK_MODULES GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR
set +u
source /opt/ros/jazzy/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=43
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# The navigation backend uses its own Unitree DDS domain (44).  If a previous
# MuJoCo launch survived a terminal/GUI close, stop only that exact process
# before starting a replacement.
bash "$ROOT_DIR/scripts/recover_navigation_lock.sh" /tmp/g1_backend_domain_44.lock
for package in nav2_bringup slam_toolbox; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "Falta $package. Ejecutar: sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox" >&2
    exit 1
  fi
done
cd "$ROOT_DIR"
exec ros2 launch scripts/navigation.launch.py "$@"
