"""
Hardware MAVROS Launch & VSLAM Relay (vslam/mavrospy.launch.py)
Capstone Technical Report - Section 6.1.2

Hardware-side launch graph per Jetson Orin Nano:
- Launches MAVROS connecting to Pixhawk via serial endpoint (/dev/ttyUSB0 @ 921600 baud)
- Relay node forwards VSLAM pose (/visual_slam/tracking/vo_pose_covariance) to MAVROS vision pose (/mavros/vision_pose/pose)
- Configures PX4 EKF2 parameters for vision-only GPS-denied navigation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyUSB0:921600',
        description='Serial connection to Pixhawk FCU'
    )
    fcu_url = LaunchConfiguration('fcu_url')

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': fcu_url,
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0'
        }]
    )

    # Relay node forwarding Isaac ROS VSLAM pose into MAVROS vision_pose topic
    relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='vslam_pose_relay',
        parameters=[{
            'input_topic': 'visual_slam/tracking/vo_pose_covariance',
            'output_topic': 'mavros/vision_pose/pose_cov'
        }]
    )

    return LaunchDescription([
        fcu_url_arg,
        mavros_node,
        relay_node
    ])
