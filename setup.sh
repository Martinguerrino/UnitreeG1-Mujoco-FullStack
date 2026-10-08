#!/usr/bin/env bash
# Full installation for Ubuntu 24.04 amd64; run as your regular user.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"
DRY_RUN=false
CHECK_ONLY=false
SYSTEM=true
SLAM3D=true
EXPERIMENTS=true
BUILD=true
TASK_PYTHON="${G1_PYTHON:-}"
usage() {
  cat <<'EOF'
Uso: bash setup.sh [opciones]
  --python /ruta/python  Usar un entorno Python 3.12 propio (Conda o venv).
                        Sin opción, usa el entorno activo/configurado o crea .venv.
  --dry-run             Mostrar los pasos sin instalar ni modificar archivos.
  --check               Verificar la instalación existente, sin instalar.
  --skip-system         No instalar paquetes APT (deben estar disponibles).
  --skip-slam3d         Omitir FAST-LIO2, RTAB-Map y TSDF.
  --skip-experiments    Omitir el experimento ACT/Okura y su checkpoint.
  --skip-build          Omitir la compilación del workspace ROS.
  --help                Mostrar esta ayuda.
Por defecto instala ROS/MoveIt/Nav2, simulación DDS, Dex1, SLAM 3D,
percepción YOLO lista para usar, exportación de video y el experimento ACT.
APT puede solicitar sudo. Requiere Ubuntu 24.04 amd64 y conexión a Internet.
EOF
}
while (($#)); do
  case "$1" in
    --python) (($# >= 2)) || { usage >&2; exit 2; }; TASK_PYTHON="$2"; shift 2 ;;
    --dry-run) DRY_RUN=true; shift ;;
    --check) CHECK_ONLY=true; shift ;;
    --skip-system) SYSTEM=false; shift ;;
    --skip-slam3d) SLAM3D=false; shift ;;
    --skip-experiments) EXPERIMENTS=false; shift ;;
    --skip-build) BUILD=false; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Opción desconocida: $1" >&2; usage >&2; exit 2 ;;
  esac
done
run() {
  if $DRY_RUN; then
    printf '[dry-run]'; printf ' %q' "$@"; printf '\n'
  else
    "$@"
  fi
}
if [[ -z "$TASK_PYTHON" ]]; then
  if [[ -n "${CONDA_PREFIX:-${VIRTUAL_ENV:-}}" ]]; then
    TASK_PYTHON="${CONDA_PREFIX:-$VIRTUAL_ENV}/bin/python"
  elif [[ -f .g1-runtime.json || -x .venv/bin/python ]]; then
    TASK_PYTHON="$(/usr/bin/python3 g1_backend/python_runtime.py)"
  else
    TASK_PYTHON="$ROOT_DIR/.venv/bin/python"
  fi
fi
if [[ "$TASK_PYTHON" != /* ]]; then
  TASK_PYTHON="$(command -v "$TASK_PYTHON" || true)"
  [[ -n "$TASK_PYTHON" ]] || { echo 'No se encontró el Python indicado.' >&2; exit 1; }
fi
TASK_PYTHON="$(/usr/bin/python3 -c 'import os,sys;print(os.path.abspath(sys.argv[1]))' "$TASK_PYTHON")"
export G1_PYTHON="$TASK_PYTHON"
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH LD_LIBRARY_PATH LIBRARY_PATH CPATH CPLUS_INCLUDE_PATH PKG_CONFIG_PATH
export OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 OMP_NUM_THREADS=1
export G1_REQUIRE_SLAM3D="$SLAM3D" G1_REQUIRE_EXPERIMENTS="$EXPERIMENTS" G1_REQUIRE_BUILD="$BUILD"
if $CHECK_ONLY; then
  bash scripts/check_installation.sh "$G1_PYTHON"
  exit
fi
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 24.04 || "$(uname -m)" != x86_64 ]]; then
  echo 'El setup completo soporta Ubuntu 24.04 amd64. No se modificó el sistema.' >&2
  exit 1
fi
if [[ -x "$TASK_PYTHON" ]]; then
  "$TASK_PYTHON" -c 'import sys,pathlib; assert sys.version_info[:2] == (3,12), "Se requiere Python 3.12"; assert sys.prefix != sys.base_prefix or (pathlib.Path(sys.prefix)/"conda-meta").is_dir(), "Usá un entorno Conda o venv, no el Python global"'
elif [[ "$TASK_PYTHON" != "$ROOT_DIR/.venv/bin/python" ]]; then
  echo "No existe el Python del entorno: $TASK_PYTHON" >&2; exit 1
fi
if $SYSTEM; then
  run bash scripts/install_host_dependencies.sh
  run bash scripts/install_ros_dependencies.sh
fi
if [[ ! -x "$TASK_PYTHON" ]]; then
  run /usr/bin/python3 -m venv "$ROOT_DIR/.venv"
fi
echo "Python elegido: $TASK_PYTHON"
run /usr/bin/python3 g1_backend/python_runtime.py --save "$TASK_PYTHON"
run "$TASK_PYTHON" -m pip install -r requirements-runtime.txt -r requirements-video.txt
if $SYSTEM; then run bash scripts/bootstrap.sh; else run bash scripts/bootstrap.sh --check-only; fi
run "$TASK_PYTHON" scripts/setup_dds.py
run "$TASK_PYTHON" scripts/setup_dex1.py
run bash scripts/setup_perception_yolo.sh
if $EXPERIMENTS; then
  run bash scripts/setup_okura.sh
fi
if $SLAM3D; then
  run bash scripts/install_navigation_slam.sh
fi
if $BUILD; then
  run bash scripts/build.sh
fi
run bash scripts/check_installation.sh "$TASK_PYTHON"
if $DRY_RUN; then echo 'Plan mostrado; no se instaló ni modificó nada.'; else echo 'Instalación finalizada.'; fi
echo 'Navegación y percepción:'
echo '  bash scripts/navigation.sh perception:=true'
echo 'SLAM 3D: bash scripts/navigation.sh --slam rgb-d perception:=true'
echo 'Manipulación: bash scripts/sim.sh moveit:=true rviz:=true'
