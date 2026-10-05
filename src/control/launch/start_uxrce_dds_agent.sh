#!/bin/bash
#
# Start uXRCE-DDS Agent for PX4 SITL
#
# This script starts the uXRCE-DDS agent that bridges PX4 SITL (via UDP)
# with ROS 2 (via DDS). The agent translates between PX4's internal topic
# structure (uORB) and ROS 2 topics.
#
# Prerequisites:
#   - PX4 SITL must be running (in another terminal)
#   - uXRCE-DDS agent must be installed
#   - ROS 2 environment must be sourced
#
# Usage: ./start_uxrce_dds_agent.sh
#

set -e

# Configuration
AGENT_HOST="127.0.0.1"
AGENT_PORT="2019"
PX4_PORT="2018"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}Starting uXRCE-DDS Agent for PX4 SITL${NC}"
echo -e "${YELLOW}Agent: ${AGENT_HOST}:${AGENT_PORT}${NC}"
echo -e "${YELLOW}PX4: ${AGENT_HOST}:${PX4_PORT}${NC}"

# Check if uXRCE-DDS agent is available
if ! command -v MicroXRCEAgent &> /dev/null; then
    echo -e "${RED}Error: MicroXRCEAgent not found in PATH${NC}"
    echo "Please install uXRCE-DDS agent:"
    echo "  sudo apt install ros-humble-uxrce-agent"
    exit 1
fi

# Start the agent
# The agent listens on UDP port 2019 and connects to PX4 on UDP port 2018
echo -e "${GREEN}Starting MicroXRCEAgent...${NC}"
MicroXRCEAgent udp4 -p ${AGENT_PORT} &
AGENT_PID=$!

echo -e "${GREEN}Agent started with PID ${AGENT_PID}${NC}"
echo -e "${YELLOW}Agent is listening on ${AGENT_HOST}:${AGENT_PORT}${NC}"
echo -e "${YELLOW}Waiting for PX4 SITL to connect...${NC}"

# Keep agent running
wait $AGENT_PID
