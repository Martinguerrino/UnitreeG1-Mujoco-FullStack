#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAUNCH_ARGS=()
SLAM_MODE=slam_toolbox
while (($#)); do
  case "$1" in
    --slam)
      if (($# < 2)) || [[ "$2" != lidar3d && "$2" != rgb-d ]]; then
        echo "Uso: --slam lidar3d|rgb-d" >&2
        exit 2
      fi
      SLAM_MODE="$2"
      LAUNCH_ARGS+=("slam:=$2")
      shift 2
      ;;
    --slam=*)
      SLAM_MODE="${1#*=}"
      if [[ "$SLAM_MODE" != lidar3d && "$SLAM_MODE" != rgb-d ]]; then
        echo "Uso: --slam lidar3d|rgb-d" >&2
        exit 2
      fi
      LAUNCH_ARGS+=("slam:=$SLAM_MODE")
      shift
      ;;
    --dexterity)
      if (($# < 2)) || [[ "$2" != claw && "$2" != hand ]]; then
        echo "Uso: --dexterity claw|hand" >&2
        exit 2
      fi
      LAUNCH_ARGS+=("dexterity:=$2")
      shift 2
      ;;
    --dexterity=*)
      value="${1#*=}"
      if [[ "$value" != claw && "$value" != hand ]]; then
        echo "Uso: --dexterity claw|hand" >&2
        exit 2
      fi
      LAUNCH_ARGS+=("dexterity:=$value")
      shift
      ;;
    --real-odom) LAUNCH_ARGS+=("real_odom:=true"); shift ;;
    --ideal) LAUNCH_ARGS+=("ideal:=true"); shift ;;
    slam:=*) SLAM_MODE="${1#slam:=}"; LAUNCH_ARGS+=("$1"); shift ;;
    *) LAUNCH_ARGS+=("$1"); shift ;;
  esac
done
# ROS uses system Python/FastDDS; SDK2 uses Conda in a separate process.
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
unset PYTHONHOME PYTHONPATH LD_LIBRARY_PATH
unset GTK_PATH GTK_EXE_PREFIX GTK_MODULES GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR
set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  source "$ROOT_DIR/install/setup.bash"
fi
if [[ -d "$ROOT_DIR/artifacts/slam" ]]; then
  source "$ROOT_DIR/scripts/slam_env.sh"
fi
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=43
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
# The navigation backend uses its own Unitree DDS domain (44).  If a previous
# MuJoCo launch survived a terminal/GUI close, stop only that exact process
# before starting a replacement.
bash "$ROOT_DIR/scripts/recover_navigation_lock.sh" /tmp/g1_backend_domain_44.lock
/usr/bin/python3 "$ROOT_DIR/scripts/recover_navigation_ports.py"
PACKAGES=(nav2_bringup)
case "$SLAM_MODE" in
  slam_toolbox) PACKAGES+=(slam_toolbox) ;;
  lidar3d) PACKAGES+=(fast_lio) ;;
  rgb-d) PACKAGES+=(rtabmap_odom rtabmap_slam) ;;
  *) echo "SLAM inválido: $SLAM_MODE" >&2; exit 2 ;;
esac
for package in "${PACKAGES[@]}"; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "Falta $package. Para SLAM 3D ejecutar: bash scripts/install_navigation_slam.sh" >&2
    exit 1
  fi
done
cd "$ROOT_DIR"
if [[ "$SLAM_MODE" == rgb-d ]]; then
  bash "$ROOT_DIR/scripts/build_navigation_mesh.sh"
  if ! "$ROOT_DIR/artifacts/slam/tsdf_venv/bin/python" -c 'import open3d' >/dev/null 2>&1; then
    echo 'Falta Open3D para TSDF: bash scripts/install_navigation_tsdf.sh' >&2
    exit 1
  fi
fi
exec ros2 launch scripts/navigation.launch.py "${LAUNCH_ARGS[@]}"
