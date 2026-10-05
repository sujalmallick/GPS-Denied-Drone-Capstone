#!/usr/bin/env bash
# sim/isaac_ros_vslam.sh - Launch VSLAM for a specific drone under its namespace
# Usage: bash isaac_ros_vslam.sh --ns drone_1

DRONE_NS="drone_1"

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --ns) DRONE_NS="$2"; shift ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
    shift
done

echo "Starting Isaac ROS Visual SLAM for namespace: /${DRONE_NS}/"
ros2 launch vslam isaac_ros_vslam_realsense.py drone_ns:=${DRONE_NS}
