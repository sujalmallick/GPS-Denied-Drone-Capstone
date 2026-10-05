#!/usr/bin/env python3
"""
Launch file for starting Gazebo with the drone model.

For M1 testing, this launches Gazebo with a simple world and the quadrotor model.
PX4 SITL should be started separately (see docs/simulation.md).

Usage:
    ros2 launch simulation simple_world.launch.py

"""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """Generate the launch description for Gazebo + drone model."""

    # Get the package directory (simulation package)
    # Note: For this to work, we need a proper ROS 2 package structure.
    # In development, we use relative paths.
    simulation_dir = Path(__file__).parent.parent.absolute()
    worlds_dir = simulation_dir / "worlds"
    models_dir = simulation_dir / "models"

    # World file path
    world_file = worlds_dir / "simple_world.sdf"

    # Model path for Gazebo to find the drone model
    gazebo_model_path = str(models_dir)

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=str(world_file),
        description="Path to the Gazebo world file",
    )

    verbose_arg = DeclareLaunchArgument(
        "verbose",
        default_value="false",
        description="Enable verbose output for Gazebo",
    )

    # Gazebo launch command
    # Note: This uses 'gz sim' (Gazebo Harmonic CLI)
    gazebo_launch = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-r",  # Run paused initially (optional)
            LaunchConfiguration("world"),
        ],
        output="screen",
        additional_env={
            "GZ_SIM_RESOURCE_PATH": gazebo_model_path,
        },
    )

    # Return the launch description
    return LaunchDescription(
        [
            world_arg,
            verbose_arg,
            gazebo_launch,
        ]
    )
