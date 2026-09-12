#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

if [[ ! -f install/setup.bash ]]; then
  echo "El workspace no está compilado. Ejecuta ./scripts/build.sh" >&2
  exit 1
fi
# shellcheck disable=SC1091
set +u
source scripts/activate.sh
set -u

MOVEIT_REQUESTED=false
for argument in "$@"; do
  if [[ "${argument}" == "moveit:=true" ]]; then
    MOVEIT_REQUESTED=true
    break
  fi
done

if [[ "${MOVEIT_REQUESTED}" == true ]] && \
  ! ros2 pkg prefix moveit_ros_move_group >/dev/null 2>&1; then
  echo "MoveIt 2 no está instalado para ROS Jazzy." >&2
  echo "Ejecuta:" >&2
  echo "  sudo apt update" >&2
  echo "  sudo apt install ros-jazzy-moveit" >&2
  echo "  ./scripts/build.sh" >&2
  exit 1
fi

exec ros2 launch g1_bringup simulation.launch.py "$@"
