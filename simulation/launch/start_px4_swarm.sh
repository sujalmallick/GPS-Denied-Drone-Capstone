#!/bin/bash
#
# Start 3-Drone PX4 SITL Swarm with Gazebo Harmonic
# Usage: ./start_px4_swarm.sh
#

set -e

PX4_SOURCE_DIR="${PX4_SOURCE_DIR:=$HOME/src/PX4-Autopilot}"
PX4_BUILD_DIR="${PX4_SOURCE_DIR}/build/px4_sitl_default"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}========================================================================${NC}"
echo -e "${GREEN}🚁 Launching 3-Drone PX4 SITL Swarm for Gazebo Harmonic${NC}"
echo -e "${GREEN}========================================================================${NC}"

if [ ! -d "$PX4_BUILD_DIR" ]; then
    echo -e "${RED}Error: PX4 build directory not found at $PX4_BUILD_DIR${NC}"
    echo "Please build PX4 first: cd $PX4_SOURCE_DIR && make px4_sitl_default"
    exit 1
fi

cd "$PX4_SOURCE_DIR"
export PX4_SYS_AUTOSTART=4001
export PX4_ESTIMATOR=ekf2

# Trap signals to kill all child PX4 processes cleanly on exit
trap 'kill $(jobs -p) 2>/dev/null' EXIT

# Instance 0: drone_1 (MAVLink UDP 14540)
echo -e "${YELLOW}Starting PX4 SITL Instance 0 [drone_1] on port 14540...${NC}"
./build/px4_sitl_default/bin/px4 -i 0 -d -s px4.commands -m iris > /tmp/px4_drone_1.log 2>&1 &
sleep 2

# Instance 1: drone_2 (MAVLink UDP 14541)
echo -e "${YELLOW}Starting PX4 SITL Instance 1 [drone_2] on port 14541...${NC}"
./build/px4_sitl_default/bin/px4 -i 1 -d -s px4.commands -m iris > /tmp/px4_drone_2.log 2>&1 &
sleep 2

# Instance 2: drone_3 (MAVLink UDP 14542)
echo -e "${YELLOW}Starting PX4 SITL Instance 2 [drone_3] on port 14542...${NC}"
./build/px4_sitl_default/bin/px4 -i 2 -d -s px4.commands -m iris > /tmp/px4_drone_3.log 2>&1 &

echo -e "${GREEN}========================================================================${NC}"
echo -e "${GREEN}✅ All 3 PX4 SITL instances active:${NC}"
echo -e "   • drone_1: UDP 14540"
echo -e "   • drone_2: UDP 14541"
echo -e "   • drone_3: UDP 14542"
echo -e "${YELLOW}Press Ctrl+C to shut down swarm simulation instances.${NC}"
echo -e "${GREEN}========================================================================${NC}"

wait
