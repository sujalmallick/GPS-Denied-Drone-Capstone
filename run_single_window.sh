#!/bin/bash
#
# Single-Window Swarm Simulation
# Opens ONE Gazebo window: the physically simulated drones and arena, with the swarm's merged map
# (coloured by which drone mapped it), frontiers, 2 m exclusion zones, A* paths, trails and a live
# coverage/overlap HUD drawn inside it by coordination/gz_overlay.py. No RViz2.
#
# Usage: ./run_single_window.sh [extra gazebo_swarm.launch.py arguments]
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/run_gazebo_swarm.sh" rviz:=false "$@"
