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
source "$ROOT_DIR/scripts/host_gui_env.sh"

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
