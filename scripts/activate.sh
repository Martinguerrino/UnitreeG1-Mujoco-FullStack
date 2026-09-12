#!/usr/bin/env bash
set -euo pipefail

CONDA_SH="/home/martin/miniforge3/etc/profile.d/conda.sh"
if [[ ! -f "${CONDA_SH}" ]]; then
  echo "No se encontró Miniforge en /home/martin/miniforge3" >&2
  return 1 2>/dev/null || exit 1
fi

# shellcheck disable=SC1090
set +u
source "${CONDA_SH}"
conda activate mujoco-unitree

# VS Code installed as a Snap exports GTK paths from Ubuntu Core. Native ROS
# GUI programs can then mix Snap's libc with the host libraries and abort. Some
# integrated terminals keep these paths without exporting SNAP, so always clear
# the overrides: this workspace uses the host GUI stack, never Snap's GTK stack.
unset GTK_PATH GTK_EXE_PREFIX GTK_MODULES
unset GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR GTK_IM_MODULE_FILE

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # ROS remains an apt/system dependency; its Python ABI is compatible with 3.12.
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

if [[ -f install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source install/setup.bash
fi
set -u

echo "Entorno mujoco-unitree activo. Python: $(command -v python)"
