#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch M4 mapping node.
    
    Prerequisites:
    - Gazebo running with drone model
    - Ground truth pose publisher (M3) running
    - Depth camera publishing to /depth/image
    """

    mapping_dir = get_package_share_directory('mapping')

    mapping_node = Node(
        package='mapping',
        executable='mapping_node',
        name='mapping_node',
        output='screen',
        parameters=[
            {
                'map_width_m': 40.0,
                'map_height_m': 40.0,
                'map_resolution_m': 0.1,
                'camera_fx': 518.0,
                'camera_fy': 518.0,
                'camera_cx': 320.0,
                'camera_cy': 240.0,
                'depth_max_range': 10.0,
                'publish_rate_hz': 2,
                'verbose': True,
            }
        ],
    )

    return LaunchDescription([mapping_node])
