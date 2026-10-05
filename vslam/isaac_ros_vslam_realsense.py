"""
Isaac ROS Visual SLAM & RealSense Launch Graph (vslam/isaac_ros_vslam_realsense.py)
Capstone Technical Report - Section 6.1.1

Perception pipeline per drone:
- Starts realsense2_camera_node with stereo IR and IMU @ 200 Hz
- Starts Isaac ROS visual_slam_node with IMU EKF fusion enabled
- Applies topic remappings under drone namespace: /<drone_ns>/
- Publishes pose estimate to /<drone_ns>/visual_slam/tracking/vo_pose_covariance
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    drone_ns_arg = DeclareLaunchArgument(
        'drone_ns',
        default_value='drone_1',
        description='Namespace of the drone agent'
    )
    drone_ns = LaunchConfiguration('drone_ns')

    perception_group = GroupAction([
        PushRosNamespace(drone_ns),

        # Intel RealSense D435i Node
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            parameters=[{
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_depth': True,
                'enable_gyro': True,
                'enable_accel': True,
                'gyro_fps': 200,
                'accel_fps': 200,
                'infra_width': 640,
                'infra_height': 480,
                'infra_fps': 30,
                'unite_imu_method': 2 # Linear interpolation
            }]
        ),

        # Isaac ROS Visual SLAM Node
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam_node',
            name='visual_slam_node',
            parameters=[{
                'enable_rectified_pose': True,
                'denoise_input_images': False,
                'enable_imu_fusion': True,
                'gyro_noise_density': 0.000244,
                'gyro_random_walk': 0.000019393,
                'accel_noise_density': 0.001862,
                'accel_random_walk': 0.003,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link'
            }],
            remappings=[
                ('stereo_camera/left/image', 'camera/infra1/image_rect_raw'),
                ('stereo_camera/left/camera_info', 'camera/infra1/camera_info'),
                ('stereo_camera/right/image', 'camera/infra2/image_rect_raw'),
                ('stereo_camera/right/camera_info', 'camera/infra2/camera_info'),
                ('visual_slam/imu', 'camera/imu')
            ]
        )
    ])

    return LaunchDescription([
        drone_ns_arg,
        perception_group
    ])
