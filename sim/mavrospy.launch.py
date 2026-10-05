"""
Multi-Drone MAVROS Launch File (sim/mavrospy.launch.py)
Capstone Technical Report - Section 5.1 & Section 8.2 Listing 1

Launches MAVROS and control node per drone instance:
- Namespace parameterized via launch argument
- FCU URL maps to the respective PX4 SITL UDP port:
  drone_1 -> udp://:14540@localhost:14557
  drone_2 -> udp://:14541@localhost:14558
  drone_3 -> udp://:14542@localhost:14559
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        'drone_ns',
        default_value='drone_1',
        description='Drone namespace'
    )
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@localhost:14557',
        description='PX4 MAVLink endpoint'
    )

    drone_ns = LaunchConfiguration('drone_ns')
    fcu_url = LaunchConfiguration('fcu_url')

    mavros_group = GroupAction([
        PushRosNamespace(drone_ns),
        Node(
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
        ),
        # Control bridge relay node
        Node(
            package='control',
            executable='px4_control_node',
            name='px4_control_node',
            output='screen',
            parameters=[{
                'drone_id': drone_ns,
                'offboard_rate': 20.0
            }]
        )
    ])

    return LaunchDescription([
        ns_arg,
        fcu_url_arg,
        mavros_group
    ])
