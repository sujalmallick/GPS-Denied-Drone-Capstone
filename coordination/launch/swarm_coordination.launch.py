#!/usr/bin/env python3
"""
ROS 2 Swarm Coordination Launch File (Gazebo Simulation Stack)
Capstone Technical Report - Section 6 & 10

Brings up, for every drone namespace:
- depth_mapper:      /{ns}/depth/image + /{ns}/odom -> /{ns}/map (local occupancy grid, 2 Hz)
- flight_controller: /{ns}/cmd_pose + /global_map  -> A* path -> /{ns}/cmd_vel, /{ns}/enable

and once for the swarm:
- map_merger:       /{ns}/map -> /global_map (conservative fusion, 2.0 Hz) + coverage/overlap metrics
- task_allocator:   Algorithm 1 greedy no-overlap frontier assignment -> /{ns}/cmd_pose (1.0 Hz)
- swarm_sim_bridge: Gazebo odometry + world SDF -> RViz2 drone/trail/environment markers and TF (rviz:=true)
- gz_overlay:       draws the merged map, frontiers, exclusion zones and HUD inside the Gazebo GUI
- rviz2:            optional ROS-side mission control (coordination/rviz/swarm_coordination.rviz)

Normally included by simulation/launch/gazebo_swarm.launch.py, which starts Gazebo first.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('coordination')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'swarm_coordination.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    lock_radius = float(LaunchConfiguration('lock_radius').perform(context))
    publish_rate = float(LaunchConfiguration('publish_rate').perform(context))
    altitude = float(LaunchConfiguration('altitude').perform(context))
    world_file = LaunchConfiguration('world_file').perform(context)
    drones = [d.strip() for d in LaunchConfiguration('drones').perform(context).split(',') if d.strip()]

    nodes = []
    for ns in drones:
        nodes.append(Node(
            package='coordination', executable='depth_mapper', name=f'depth_mapper_{ns}',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'namespace': ns, 'drone_namespaces': drones}]))
        nodes.append(Node(
            package='coordination', executable='flight_controller', name=f'flight_controller_{ns}',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'namespace': ns,
                         'drone_namespaces': drones, 'altitude': altitude}]))

    nodes += [
        Node(
            package='coordination', executable='map_merger', name='map_merger', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'publish_rate': publish_rate,
                         'global_frame_id': 'map', 'drone_namespaces': drones}]),
        Node(
            package='coordination', executable='task_allocator', name='task_allocator', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'allocation_frequency': 1.0,
                         'lock_radius': lock_radius, 'min_cluster_size': 5,
                         'target_altitude': altitude, 'drone_namespaces': drones}]),
        Node(
            package='coordination', executable='swarm_sim_bridge', name='swarm_sim_bridge', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'drone_namespaces': drones,
                         'world_file': world_file}],
            condition=IfCondition(LaunchConfiguration('rviz'))),  # RViz-only markers/TF
        Node(
            package='coordination', executable='gz_overlay', name='gz_overlay', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'drone_namespaces': drones}],
            condition=IfCondition(LaunchConfiguration('gz_overlay'))),
        Node(
            package='rviz2', executable='rviz2', name='rviz2_swarm', output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the Gazebo /clock'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2 swarm mission control'),
        DeclareLaunchArgument('gz_overlay', default_value='false',
                              description='Draw the swarm overlay inside the Gazebo GUI'),
        DeclareLaunchArgument('lock_radius', default_value='2.0',
                              description='Algorithm 1 spatial exclusion radius in meters'),
        DeclareLaunchArgument('publish_rate', default_value='2.0',
                              description='Global map merge publication rate in Hz'),
        DeclareLaunchArgument('altitude', default_value='1.5',
                              description='Exploration flight altitude in meters'),
        DeclareLaunchArgument('drones', default_value='drone_1,drone_2,drone_3',
                              description='Comma-separated drone namespaces'),
        DeclareLaunchArgument('world_file', default_value='',
                              description='Gazebo world SDF mirrored into RViz2 as environment markers'),
        OpaqueFunction(function=launch_setup),
    ])
