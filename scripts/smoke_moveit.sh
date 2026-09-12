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

for package in moveit_ros_move_group moveit_ros_planning_interface; do
  if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    echo "Falta ${package}. Ejecuta ./scripts/install_ros_dependencies.sh" >&2
    exit 1
  fi
done

LOG_FILE="/tmp/g1_moveit_smoke.$$.log"
ros2 launch g1_bringup simulation.launch.py \
  headless:=true rviz:=false moveit:=true >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -INT "${LAUNCH_PID}" >/dev/null 2>&1 || true
  wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

READY=false
for _ in {1..45}; do
  if ros2 node list 2>/dev/null | grep -qx '/move_group' && \
    ros2 action list 2>/dev/null | \
      grep -qx '/whole_body_controller/follow_joint_trajectory'; then
    READY=true
    break
  fi
  if ! kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    echo "El launch terminó antes de estar listo. Log: ${LOG_FILE}" >&2
    tail -n 80 "${LOG_FILE}" >&2
    exit 1
  fi
  sleep 1
done

if [[ "${READY}" != true ]]; then
  echo "MoveIt/controlador no estuvieron listos tras 45 s. Log: ${LOG_FILE}" >&2
  tail -n 80 "${LOG_FILE}" >&2
  exit 1
fi

if ! timeout 30s ros2 run g1_manipulation move_to_pose \
  --group left_arm --named-home --plan-only; then
  echo "Falló la planificación de humo. Log: ${LOG_FILE}" >&2
  tail -n 80 "${LOG_FILE}" >&2
  exit 1
fi

echo "Smoke test correcto: MuJoCo, move_group, IK, colisiones y OMPL responden."
echo "Log: ${LOG_FILE}"
