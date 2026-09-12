#!/usr/bin/env bash
set -euo pipefail

if [[ "$(id -u)" -eq 0 ]]; then
  APT=(apt-get)
else
  APT=(sudo apt-get)
fi

"${APT[@]}" update
"${APT[@]}" install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-jazzy-mujoco-ros2-control \
  ros-jazzy-moveit \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  if [[ "$(id -u)" -eq 0 ]]; then
    rosdep init
  else
    sudo rosdep init
  fi
fi
rosdep update

echo "Dependencias ROS 2 listas. Continúa con ./scripts/build.sh"
