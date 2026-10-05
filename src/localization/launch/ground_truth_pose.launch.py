#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch M3 localization stack:
    - Ground truth pose publisher
    - Sensor bridge (optional)
    """

    localization_dir = get_package_share_directory('localization')

    # Ground truth pose publisher (required for M3-M10)
    ground_truth_node = Node(
        package='localization',
        executable='ground_truth_pose_publisher',
        name='ground_truth_pose_publisher',
        output='screen',
        parameters=[
            {
                'gazebo_odometry_topic': '/odom',
                'ros_pose_topic': 'ground_truth_pose',
                'publish_tf': True,
                'map_frame': 'map',
                'base_link_frame': 'base_link',
                'verbose': True,
            }
        ],
        remappings=[],
    )

    # Sensor bridge (optional; camera/depth can also be published directly via gazebo_ros)
    sensor_bridge_node = Node(
        package='localization',
        executable='sensor_bridge',
        name='sensor_bridge',
        output='screen',
        parameters=[
            {
                'republish_camera': True,
                'republish_depth': True,
                'republish_imu': False,  # IMU typically direct from Gazebo
                'verbose': False,
            }
        ],
    )

    return LaunchDescription([
        ground_truth_node,
        # Uncomment if sensor bridge is needed:
        # sensor_bridge_node,
    ])
