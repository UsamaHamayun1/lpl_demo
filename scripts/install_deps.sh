#!/usr/bin/env bash
# Installs everything lpl_demo needs on Ubuntu 24.04:
# ROS 2 Jazzy, Gazebo Harmonic (via ros_gz), TurtleBot3, Nav2 and the Python deps.
# Usage: sudo bash scripts/install_deps.sh
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "Run with sudo: sudo bash $0" >&2
  exit 1
fi

. /etc/os-release
if [ "$VERSION_CODENAME" != "noble" ]; then
  echo "This script targets Ubuntu 24.04 (noble); found $VERSION_CODENAME." >&2
  exit 1
fi

apt-get update
apt-get install -y software-properties-common curl locales
add-apt-repository -y universe
locale-gen en_US en_US.UTF-8

# ROS 2 apt source (official ros2-apt-source package)
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
  dpkg -i /tmp/ros2-apt-source.deb
  rm -f /tmp/ros2-apt-source.deb
fi

apt-get update
apt-get install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-simulations \
  ros-jazzy-turtlebot3-msgs \
  ros-jazzy-cv-bridge \
  python3-pyqt5 \
  python3-opencv \
  python3-numpy \
  git gh

[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init

echo
echo "Done. Now run (as your normal user, not sudo): rosdep update"
