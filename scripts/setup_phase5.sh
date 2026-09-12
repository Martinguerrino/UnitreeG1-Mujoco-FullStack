#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

echo "[1/5] Instalando dependencias ROS 2 (sudo puede pedir tu contraseña)"
./scripts/install_ros_dependencies.sh

echo "[2/5] Resolviendo dependencias del workspace"
./scripts/bootstrap.sh

echo "[3/5] Compilando"
./scripts/build.sh

echo "[4/5] Ejecutando tests"
# shellcheck disable=SC1091
set +u
source scripts/activate.sh
set -u
./scripts/check.sh

echo "[5/5] Probando MoveIt contra MuJoCo sin interfaz gráfica"
./scripts/smoke_moveit.sh

echo "Fase 5 instalada y validada. Inicia la demo con:"
echo "  ./scripts/sim.sh moveit:=true rviz:=true"
