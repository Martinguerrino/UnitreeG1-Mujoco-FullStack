#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT_DIR/scripts/python_env.sh"
export PATH="$(dirname "$G1_PYTHON"):$PATH"
set +u

# VS Code installed as a Snap exports GTK paths from Ubuntu Core. Native ROS
# GUI programs can then mix Snap's libc with the host libraries and abort. Some
# integrated terminals keep these paths without exporting SNAP, so always clear
# the overrides: this workspace uses the host GUI stack, never Snap's GTK stack.
unset GTK_PATH GTK_EXE_PREFIX GTK_MODULES
unset GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR GTK_IM_MODULE_FILE
unset GSETTINGS_SCHEMA_DIR GIO_MODULE_DIR GIO_EXTRA_MODULES
if [[ -n "${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-}" ]]; then
  export XDG_DATA_DIRS="$XDG_DATA_DIRS_VSCODE_SNAP_ORIG"
fi

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # ROS remains an apt/system dependency; its Python ABI is compatible with 3.12.
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT_DIR/install/setup.bash"
fi
set -u

echo "Python del proyecto: $G1_PYTHON"
