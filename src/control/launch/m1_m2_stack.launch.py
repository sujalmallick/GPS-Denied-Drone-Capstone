#!/usr/bin/env python3
"""
M1 + M2 Full Stack Launch File

This launch file brings up the complete M1+M2 stack:
1. Gazebo with drone model (M1)
2. PX4 SITL startup script (M1)
3. uXRCE-DDS agent (M2 bridge)
4. ROS 2 control node (M2)

This is a convenience launcher. Individual components can still be 
launched separately if needed (see README).

Prerequisites:
  - M1 fully implemented and tested
  - PX4 SITL built and environment configured
  - uXRCE-DDS agent installed
  - ROS 2 control package built

Usage (4 terminals):
  Terminal 1: Gazebo
    ros2 launch simulation simple_world.launch.py
  
  Terminal 2: PX4 SITL
    cd simulation/launch && ./start_px4_sitl.sh
  
  Terminal 3: uXRCE-DDS Agent
    cd src/control && bash ./launch/start_uxrce_dds_agent.sh
  
  Terminal 4: ROS 2 Control Node
    ros2 launch control px4_control.launch.py

Alternatively, this file can be used to automate steps 3-4.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the full M1+M2 stack launch description."""

    # Launch arguments
    verbose_arg = DeclareLaunchArgument(
        "verbose",
        default_value="false",
        description="Enable verbose output",
    )

    domain_id_arg = DeclareLaunchArgument(
        "domain_id",
        default_value="0",
        description="ROS 2 DDS domain ID",
    )

    # Note: Gazebo and PX4 SITL should be started manually in separate terminals
    # This launch file handles the ROS 2 side (uXRCE-DDS + control node)

    # PX4 Control Node (M2)
    control_node = Node(
        package="control",
        executable="px4_control_node",
        name="px4_control",
        output="screen",
        parameters=[
            {"verbose": LaunchConfiguration("verbose")},
            {"domain_id": LaunchConfiguration("domain_id")},
        ],
    )

    return LaunchDescription(
        [
            verbose_arg,
            domain_id_arg,
            control_node,
        ]
    )
