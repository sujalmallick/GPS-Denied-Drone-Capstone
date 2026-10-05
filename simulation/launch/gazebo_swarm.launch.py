#!/usr/bin/env python3
"""
Master Gazebo Swarm Launch File (simulation/launch/gazebo_swarm.launch.py)
Capstone Technical Report - Section 5 & Section 10

Launches the full real-time pipeline:
1. Gazebo Harmonic (gz sim, DART physics) with the indoor arena (worlds/simple_world.sdf)
2. Three quadrotors rendered from models/quadrotor_px4.sdf (one namespace each):
   - drone_1 @ (-4.0, -6.0)   drone_2 @ (0.0, -6.0)   drone_3 @ (4.0, -6.0), facing +y
3. ros_gz_bridge:
   Gazebo -> ROS: /clock (100 Hz relay), /{ns}/odom, /{ns}/ground_truth_pose, /{ns}/imu,
                  /{ns}/depth/image, /{ns}/depth/camera_info, /{ns}/camera/image (rgb_bridge:=true)
   ROS -> Gazebo: /{ns}/cmd_vel, /{ns}/enable  (MulticopterVelocityControl)
4. Swarm stack (coordination/launch/swarm_coordination.launch.py): depth mappers, flight
   controllers, map_merger, task_allocator, RViz2 bridge and RViz2

Usage:
    ros2 launch simulation gazebo_swarm.launch.py                       # ONE Gazebo window with swarm overlay
    ros2 launch simulation gazebo_swarm.launch.py rviz:=true            # + RViz2
    ros2 launch simulation gazebo_swarm.launch.py headless:=true rviz:=true   # physics server + RViz2 only
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD_NAME = 'simple_world'
DRONES = [
    # namespace, x, y, yaw
    ('drone_1', -4.0, -6.0, 1.5708),
    ('drone_2', 0.0, -6.0, 1.5708),
    ('drone_3', 4.0, -6.0, 1.5708),
]


# Depth camera profiles. 'realtime' keeps a 3-drone swarm at ~1x real time on a laptop (software
# sensor rendering under WSL2); the depth mapper samples the same number of rays in both.
SENSOR_PROFILES = {
    'realtime': {'__DEPTH_W__': '160', '__DEPTH_H__': '120', '__DEPTH_HZ__': '10'},
    'full': {'__DEPTH_W__': '320', '__DEPTH_H__': '240', '__DEPTH_HZ__': '30'},
}


# Identity colours, shared with the swarm overlay / RViz markers (coordination DRONE_COLORS)
DRONE_COLORS = {'drone_1': '0.01 0.52 0.78', 'drone_2': '0.96 0.62 0.04', 'drone_3': '0.55 0.36 0.96'}


def render_model(template, ns, profile):
    sdf = template.replace('__NS__', ns).replace('__COLOR__', DRONE_COLORS.get(ns, '0.8 0.8 0.8'))
    for token, value in SENSOR_PROFILES[profile].items():
        sdf = sdf.replace(token, value)
    return sdf


def launch_setup(context, *args, **kwargs):
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    world_path = LaunchConfiguration('world').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)

    sim_dir = Path(get_package_share_directory('simulation'))
    model_template = (sim_dir / 'models' / 'quadrotor_px4.sdf').read_text()

    profile = LaunchConfiguration('sensor_profile').perform(context)
    if profile not in SENSOR_PROFILES:
        raise RuntimeError(f"sensor_profile must be one of {list(SENSOR_PROFILES)}, got '{profile}'")

    # 1. Gazebo Harmonic. The physics/sensor server always renders sensors off-screen (EGL) in its own
    # process; the GUI is a separate client process, so drawing the 3D view never stalls the physics
    # loop. (Under WSL2 the d3d12 GPU path is slower than llvmpipe for sensor read-back, so it is not
    # forced here; on a native NVIDIA Linux host EGL uses the GPU.)
    server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', world_path],
        output='screen', name='gz_server')
    actions = [server]
    if not headless:
        # Open the GUI only after the drones are spawned (t = 3 s): a GUI client that is still
        # initialising when a model is created can miss it and never draw that drone.
        actions.append(TimerAction(period=7.0, actions=[ExecuteProcess(
            cmd=['gz', 'sim', '-g', '--gui-config', str(sim_dir / 'gui' / 'swarm_gui.config')],
            output='screen', name='gz_gui')]))

    # 2. Spawn one namespaced quadrotor per drone
    spawners = []
    for ns, x, y, yaw in DRONES:
        spawners.append(Node(
            package='ros_gz_sim', executable='create', name=f'spawn_{ns}', output='screen',
            arguments=['-world', WORLD_NAME, '-name', ns,
                       '-string', render_model(model_template, ns, profile),
                       '-x', str(x), '-y', str(y), '-z', '0.1', '-Y', str(yaw)]))

    # 3. ROS <-> Gazebo bridges
    core_topics = []  # /clock is relayed at 100 Hz by coordination/clock_relay.py, not bridged at 500 Hz
    core_remaps = []
    camera_topics = []
    for ns, *_ in DRONES:
        core_topics += [
            f'/model/{ns}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/model/{ns}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            f'/{ns}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            f'/{ns}/depth/image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/{ns}/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/{ns}/enable@std_msgs/msg/Bool]gz.msgs.Boolean',
        ]
        core_remaps += [
            (f'/model/{ns}/odometry', f'/{ns}/odom'),
            (f'/model/{ns}/pose', f'/{ns}/ground_truth_pose'),
        ]
        camera_topics.append(f'/{ns}/camera/image@sensor_msgs/msg/Image[gz.msgs.Image')

    clock_relay = Node(package='coordination', executable='clock_relay', name='clock_relay', output='screen')
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge', output='screen',
        arguments=core_topics, remappings=core_remaps)
    # The 640x480 RGB streams are opt-in: once bridged, Gazebo renders all three cameras at 30 Hz,
    # which alone drops a laptop below real time.
    camera_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_camera_bridge', output='screen',
        arguments=camera_topics,
        condition=IfCondition(LaunchConfiguration('rgb_bridge')))

    # 4. Swarm autonomy + visualization, started once the drones exist
    coord_launch = Path(get_package_share_directory('coordination')) / 'launch' / 'swarm_coordination.launch.py'
    coordination = TimerAction(period=5.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(coord_launch)),
        launch_arguments={
            'use_sim_time': 'true',
            'rviz': rviz,
            'lock_radius': LaunchConfiguration('lock_radius').perform(context),
            'altitude': LaunchConfiguration('altitude').perform(context),
            'drones': ','.join(ns for ns, *_ in DRONES),
            'world_file': world_path,
            'gz_overlay': 'false' if headless else 'true',
        }.items())])

    return actions + [TimerAction(period=3.0, actions=spawners), clock_relay, bridge, camera_bridge, coordination]


def generate_launch_description():
    sim_dir = Path(get_package_share_directory('simulation'))
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=str(sim_dir / 'worlds' / 'simple_world.sdf'),
                              description='Path to the Gazebo world file'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run the Gazebo physics/sensor server without its GUI'),
        DeclareLaunchArgument('rviz', default_value='false', description='Also launch RViz2 mission control'),
        DeclareLaunchArgument('lock_radius', default_value='2.0',
                              description='Algorithm 1 spatial exclusion radius (m)'),
        DeclareLaunchArgument('altitude', default_value='1.5', description='Exploration altitude (m)'),
        DeclareLaunchArgument('rgb_bridge', default_value='false',
                              description='Bridge the 640x480 RGB cameras to /{ns}/camera/image'),
        DeclareLaunchArgument('sensor_profile', default_value='realtime',
                              description="Depth camera profile: 'realtime' (160x120 @ 10 Hz) or "
                                          "'full' (320x240 @ 30 Hz, for GPU hosts)"),
        OpaqueFunction(function=launch_setup),
    ])
