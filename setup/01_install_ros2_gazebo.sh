#!/usr/bin/env bash
# Installs ROS 2 Jazzy + Gazebo Harmonic (via ros-gz) on Ubuntu 24.04. Run as root.
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y software-properties-common curl gnupg lsb-release locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository -y universe

# ROS 2 apt source
ROS_APT_VER=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_VER}/ros2-apt-source_${ROS_APT_VER}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
apt-get install -y /tmp/ros2-apt-source.deb

# Gazebo (OSRF) apt source, needed for gz-harmonic dev packages used by ardupilot_gazebo
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  > /etc/apt/sources.list.d/gazebo-stable.list

apt-get update
apt-get upgrade -y
apt-get install -y \
  ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-ros-gz ros-jazzy-mavros ros-jazzy-mavros-extras \
  gz-harmonic libgz-sim8-dev rapidjson-dev libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
  git build-essential cmake python3-pip python3-venv python3-numpy python3-scipy python3-matplotlib \
  mesa-utils

# MAVROS needs GeographicLib datasets
/opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh || true

echo "ROS2+GAZEBO INSTALL DONE"
