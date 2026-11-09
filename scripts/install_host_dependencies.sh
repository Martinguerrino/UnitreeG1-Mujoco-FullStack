#!/usr/bin/env bash
# ROS repository setup follows the official Jazzy Ubuntu installation guide:
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
set -euo pipefail
if [[ $(id -u) == 0 ]]; then APT=(apt-get); PRIV=(); else APT=(sudo apt-get); PRIV=(sudo); fi
"${APT[@]}" update
"${APT[@]}" install -y curl ca-certificates software-properties-common locales
"${PRIV[@]}" add-apt-repository -y universe
"${APT[@]}" update
"${APT[@]}" install -y \
  python3-venv python3-dev build-essential cmake git pkg-config ffmpeg \
  libgl1 libegl1 libglfw3 libosmesa6 libpcl-dev libopencv-dev qtbase5-dev \
  python3-opencv python3-scipy python3-pytest python3-numpy
"${PRIV[@]}" locale-gen en_US.UTF-8
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  TASK_DEB_DIR="$(mktemp -d)"
  trap 'rm -rf "$TASK_DEB_DIR"' EXIT
  TASK_ROS_VERSION="$(curl -fsSL --retry 3 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | /usr/bin/python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
  [[ "$TASK_ROS_VERSION" =~ ^v?[0-9]+\.[0-9]+\.[0-9]+$ ]] || { echo 'Versión ros2-apt-source inválida' >&2; exit 1; }
  curl -fL --retry 3 -o "$TASK_DEB_DIR/ros2-apt-source.deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${TASK_ROS_VERSION}/ros2-apt-source_${TASK_ROS_VERSION}.noble_all.deb"
  "${PRIV[@]}" dpkg -i "$TASK_DEB_DIR/ros2-apt-source.deb"
fi
"${APT[@]}" update
"${APT[@]}" install -y ros-jazzy-desktop ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
