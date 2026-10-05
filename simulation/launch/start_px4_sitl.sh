#!/bin/bash
#
# Start PX4 SITL with Gazebo Harmonic
#
# Usage: ./start_px4_sitl.sh
#
# This script starts PX4 SITL for use with Gazebo Harmonic.
# Gazebo should already be running before executing this script.
#
# Prerequisites:
#   - Gazebo Harmonic must be running (start with: ros2 launch simulation simple_world.launch.py)
#   - PX4 source code must be cloned and built
#   - PX4_SITL_BUILD_TARGET must be set to point to the build directory
#

set -e

# Configuration
# Adjust these paths to match your PX4 installation
PX4_SOURCE_DIR="${PX4_SOURCE_DIR:=$HOME/src/PX4-Autopilot}"
PX4_BUILD_DIR="${PX4_SOURCE_DIR}/build/px4_sitl_default"

# Color output for clarity
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Starting PX4 SITL with Gazebo Harmonic${NC}"
echo -e "${YELLOW}PX4 Source Dir: $PX4_SOURCE_DIR${NC}"
echo -e "${YELLOW}PX4 Build Dir: $PX4_BUILD_DIR${NC}"

# Check if PX4 source directory exists
if [ ! -d "$PX4_SOURCE_DIR" ]; then
    echo -e "${RED}Error: PX4 source directory not found at $PX4_SOURCE_DIR${NC}"
    echo "Please clone PX4-Autopilot and set PX4_SOURCE_DIR environment variable."
    exit 1
fi

# Check if PX4 build directory exists
if [ ! -d "$PX4_BUILD_DIR" ]; then
    echo -e "${RED}Error: PX4 build directory not found at $PX4_BUILD_DIR${NC}"
    echo "Please build PX4 first:"
    echo "  cd $PX4_SOURCE_DIR"
    echo "  make px4_sitl_default gazebo-classic"
    exit 1
fi

# Navigate to PX4 source directory
cd "$PX4_SOURCE_DIR"

# Export environment variables for PX4 SITL
export PX4_SYS_AUTOSTART=4001  # Generic quadrotor airframe
export PX4_ESTIMATOR=ekf2

# Start PX4 SITL
# Note: The exact command may vary depending on your PX4 version
# For Gazebo Harmonic (not Classic), use:
echo -e "${GREEN}Starting PX4 SITL...${NC}"
./build/px4_sitl_default/bin/px4 -i 1 -d -s px4.commands -m iris

echo -e "${GREEN}PX4 SITL started successfully${NC}"
echo -e "${YELLOW}You can now arm the drone and command takeoff using PX4 shell or QGroundControl${NC}"
