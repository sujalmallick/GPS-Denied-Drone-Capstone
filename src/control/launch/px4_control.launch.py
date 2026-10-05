#!/usr/bin/env python3
"""
M2: PX4 Control Node Launch File

This launch file starts the minimal PX4 control node for M2 testing.
The node subscribes to PX4 vehicle status and provides basic arm/disarm commands.

Prerequisites:
  - Gazebo + drone model running (from M1)
  - PX4 SITL running (from M1)
  - uXRCE-DDS agent running and bridging PX4 to ROS 2
  - ROS 2 Humble workspace built with the control package

Usage:
  ros2 launch control px4_control.launch.py verbose:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for M2 control node."""

    # Launch arguments
    verbose_arg = DeclareLaunchArgument(
        "verbose",
        default_value="false",
        description="Enable verbose debug output",
    )

    domain_id_arg = DeclareLaunchArgument(
        "domain_id",
        default_value="0",
        description="ROS 2 DDS domain ID (must match PX4 uXRCE-DDS config)",
    )

    # PX4 Control Node
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
