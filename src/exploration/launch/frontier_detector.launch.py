#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch M5-M7 frontier detection, clustering, and selection stack.
    
    Prerequisites:
    - Gazebo running with drone model
    - Ground truth pose publisher (M3) running
    - Mapping node (M4) running and publishing /map
    """

    exploration_dir = get_package_share_directory('exploration')

    frontier_node = Node(
        package='exploration',
        executable='frontier_detector_node',
        name='frontier_detector_node',
        output='screen',
        parameters=[
            {
                'distance_cost_weight': 0.1,
                'min_cluster_size': 1,
                'verbose': True,
                'publish_visualization': True,
            }
        ],
    )

    return LaunchDescription([frontier_node])
