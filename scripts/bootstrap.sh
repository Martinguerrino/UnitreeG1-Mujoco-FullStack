#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "Falta ROS 2 Jazzy. Consulta exp/01_instalacion.md." >&2
  exit 1
fi

# shellcheck disable=SC1091
set +u
source /opt/ros/jazzy/setup.bash
set -u

if ! command -v vcs >/dev/null 2>&1; then
  echo "Falta vcstool (sudo apt install python3-vcstool)." >&2
  exit 1
fi

if ! ros2 pkg prefix mujoco_ros2_control >/dev/null 2>&1; then
  echo "No hay binario mujoco_ros2_control; importando la fuente fijada."
  mkdir -p src/vendor
  vcs import src < dependencies.repos
else
  echo "Usando mujoco_ros2_control instalado por APT."
fi
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

echo "Dependencias listas. Ejecuta: ./scripts/build.sh"
