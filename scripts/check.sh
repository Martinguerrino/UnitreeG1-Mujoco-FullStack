#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

bash -n scripts/*.sh
ruff format --check scripts tests experiments src/g1_control/scripts src/g1_bringup src/g1_manipulation/launch
ruff check scripts tests experiments src/g1_control/scripts src/g1_bringup src/g1_manipulation/launch
# ROS installs system-wide pytest plugins. These unit tests do not use them;
# autoloading would couple the Conda test environment to optional ROS modules.
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  colcon list
  if [[ -f install/setup.bash ]]; then
    # shellcheck disable=SC1091
    set +u
    source install/setup.bash
    set -u
    colcon test --packages-select \
      g1_description g1_mujoco g1_control g1_bringup \
      g1_moveit_config g1_manipulation
    colcon test-result --verbose
  fi
fi
