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

LAUNCH_ARGS=()
while (($#)); do
  case "$1" in
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
    --ideal) LAUNCH_ARGS+=("ideal:=true"); shift ;;
    *) LAUNCH_ARGS+=("$1"); shift ;;
  esac
done
MOVEIT_REQUESTED=false
for argument in "${LAUNCH_ARGS[@]}"; do
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

exec ros2 launch g1_bringup simulation.launch.py "${LAUNCH_ARGS[@]}"
