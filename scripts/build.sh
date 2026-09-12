#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

# ROS 2 Jazzy fue compilado contra las librerías de Ubuntu. Si este script se
# ejecuta desde Conda, CMake puede encontrar fmt/OpenSSL de Miniforge y mezclar
# dos ABI incompatibles durante el link. El build C++ usa solo herramientas del
# host; el entorno Conda sigue reservado para MuJoCo y los tests Python.
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE
unset _CE_CONDA _CE_M _CONDA_EXE _CONDA_ROOT
unset LD_LIBRARY_PATH LIBRARY_PATH CPATH CPLUS_INCLUDE_PATH PKG_CONFIG_PATH
unset PYTHONHOME PYTHONPATH

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "Falta /opt/ros/jazzy/setup.bash" >&2
  exit 1
fi
# shellcheck disable=SC1091
set +u
source /opt/ros/jazzy/setup.bash
set -u

colcon build \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
