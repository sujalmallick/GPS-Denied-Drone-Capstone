#!/bin/bash
#
# Master 3-Drone Gazebo Swarm Simulation Launcher
# GPS-Denied Autonomous Indoor Exploration Stack (ROS 2 Jazzy + Gazebo Harmonic)
#
# Launches:
# 1. Gazebo Harmonic (DART physics) with the indoor arena
# 2. Three rotor-physics quadrotors (drone_1, drone_2, drone_3)
# 3. ros_gz_bridge for /clock, odometry, IMU, depth camera and velocity commands
# 4. Per-drone depth mappers + flight controllers, map_merger, task_allocator
# 5. Swarm overlay inside the Gazebo window (merged map, frontiers, exclusion zones, paths, HUD)
#
# Usage:
#   ./run_gazebo_swarm.sh                              # ONE Gazebo window with the swarm overlay
#   ./run_gazebo_swarm.sh rviz:=true                   # + RViz2
#   ./run_gazebo_swarm.sh headless:=true rviz:=true    # no Gazebo window, RViz2 only
#

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}========================================================================${NC}"
echo -e "${GREEN}🚁 Starting Multi-Drone Gazebo Simulation Stack${NC}"
echo -e "${GREEN}   Engine: Gazebo Harmonic + ROS 2 Jazzy + Swarm Autonomy Pipeline${NC}"
echo -e "${GREEN}========================================================================${NC}"

# Gazebo Harmonic pairs with ROS 2 Jazzy (Humble ships Gazebo Fortress and cannot run this stack)
if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
    echo -e "${RED}Error: ROS 2 Jazzy not found in /opt/ros/jazzy (required for Gazebo Harmonic)${NC}"
    exit 1
fi
source /opt/ros/jazzy/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Only the simulation packages are built here: src/control and src/localization need px4_msgs and
# gazebo_ros, which this stack does not use. Rebuilding is incremental and takes a few seconds.
echo -e "${YELLOW}Building ROS 2 packages: coordination, simulation...${NC}"
colcon build --packages-select coordination simulation
source install/setup.bash

# A Gazebo server left over from an earlier run keeps simulating the same world: it competes for
# CPU, publishes a second /clock and makes the new run crawl. Stop any stale instance first.
if pgrep -f "^gz sim" > /dev/null; then
    echo -e "${YELLOW}Stopping Gazebo processes left over from a previous run...${NC}"
    pkill -INT -f "^gz sim" || true
    sleep 3
    pkill -9 -f "^gz sim" || true
    pkill -9 -f "ros_gz_bridge|parameter_bridge" || true
fi

echo -e "${GREEN}Launching Gazebo Swarm + Coordination Stack...${NC}"
ros2 launch simulation gazebo_swarm.launch.py "$@"
