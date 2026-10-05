#!/bin/bash
#
# One-time installer for the optional PX4 SITL + MAVROS + RTAB-Map backends
# (Ubuntu 24.04 / ROS 2 Jazzy, native or WSL2). Asks for your sudo password once.
#
#   1. MAVROS (+ extras) and RTAB-Map (visual SLAM) for ROS 2 Jazzy
#   2. GeographicLib datasets required by MAVROS
#   3. PX4 toolchain + Gazebo Harmonic dev packages via PX4's own setup script
#
# PX4 itself is cloned to ~/PX4-Autopilot (v1.16.2: first release line with Ubuntu 24.04 support).
# Build it afterwards with:  cd ~/PX4-Autopilot && make px4_sitl
#

set -e
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_TAG="${PX4_TAG:-v1.16.2}"

echo "=== [1/4] ROS 2 Jazzy: MAVROS + RTAB-Map ==="
sudo apt-get update
sudo apt-get install -y ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-rtabmap-ros

echo "=== [2/4] MAVROS GeographicLib datasets ==="
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

echo "=== [3/4] PX4 ${PX4_TAG} source ==="
if [ ! -d "$PX4_DIR" ]; then
    git clone --branch "$PX4_TAG" --depth 1 --recursive --shallow-submodules \
        https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
fi

echo "=== [4/4] PX4 toolchain + Gazebo Harmonic (PX4 Tools/setup/ubuntu.sh --no-nuttx) ==="
cd "$PX4_DIR" && bash ./Tools/setup/ubuntu.sh --no-nuttx

echo "=== ALL INSTALLS DONE. Next: cd $PX4_DIR && make px4_sitl ==="
