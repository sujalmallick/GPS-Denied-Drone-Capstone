#!/usr/bin/env python3
"""
Gazebo -> RViz2 Swarm Visualization Bridge (coordination/swarm_sim_bridge.py)
Capstone Project - Multi-Drone GPS-Denied Exploration

Mirrors the live Gazebo Harmonic simulation into the single RViz2 mission-control window.
Nothing here simulates motion or sensing: every drone pose comes from Gazebo physics.

- Subscribes to /{ns}/odom (Gazebo OdometryPublisher via ros_gz_bridge)
- Broadcasts TF map -> {ns}/base_link
- Publishes /swarm/markers/drones       3D quadcopter bodies, arms, spinning rotors, labels
- Publishes /swarm/markers/trails       flown trajectory of every drone
- Publishes /swarm/markers/environment  walls, obstacles and pads parsed from the Gazebo world SDF,
                                        so RViz always shows exactly the arena Gazebo simulates
"""

import math
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

DRONE_COLORS = {
    'drone_1': (0.01, 0.52, 0.78),
    'drone_2': (0.96, 0.62, 0.04),
    'drone_3': (0.55, 0.36, 0.96),
}
ROTOR_OFFSETS = [(0.15, 0.15), (0.15, -0.15), (-0.15, 0.15), (-0.15, -0.15)]


def yaw_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def parse_world_environment(world_file):
    """Return [(name, shape, (x, y, z, yaw), (sx, sy, sz), (r, g, b, a))] for every static model
    in the SDF world whose first visual is a box or cylinder."""
    items = []
    root = ET.parse(world_file).getroot()
    for model in root.iter('model'):
        if (model.findtext('static') or '').strip() != 'true':
            continue
        pose = [float(v) for v in (model.findtext('pose') or '0 0 0 0 0 0').split()]
        visual = model.find('./link/visual')
        if visual is None:
            continue
        diffuse = visual.findtext('./material/diffuse') or '0.6 0.6 0.6 1'
        color = tuple(float(v) for v in diffuse.split())
        box = visual.find('./geometry/box/size')
        cyl = visual.find('./geometry/cylinder')
        if box is not None:
            size = tuple(float(v) for v in box.text.split())
            items.append((model.get('name'), Marker.CUBE, (pose[0], pose[1], pose[2], pose[5]), size, color))
        elif cyl is not None:
            r = float(cyl.findtext('radius'))
            length = float(cyl.findtext('length'))
            items.append((model.get('name'), Marker.CYLINDER, (pose[0], pose[1], pose[2], pose[5]),
                          (2 * r, 2 * r, length), color))
    return items


class SwarmSimBridgeNode(Node):
    def __init__(self):
        super().__init__('swarm_sim_bridge')

        self.declare_parameter('drone_namespaces', ['drone_1', 'drone_2', 'drone_3'])
        self.declare_parameter('world_file', '')
        self.declare_parameter('trail_length', 600)

        self.drone_namespaces = self.get_parameter('drone_namespaces').value
        self.trail_length = self.get_parameter('trail_length').value
        world_file = self.get_parameter('world_file').value

        self.poses = {}     # ns -> (x, y, z, qx, qy, qz, qw, yaw)
        self.trails = {ns: [] for ns in self.drone_namespaces}
        self.rotor_angle = 0.0

        self.tf_broadcaster = TransformBroadcaster(self)
        for ns in self.drone_namespaces:
            self.create_subscription(Odometry, f'/{ns}/odom', lambda msg, n=ns: self.odom_callback(msg, n), 10)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.drone_marker_pub = self.create_publisher(MarkerArray, '/swarm/markers/drones', 10)
        self.trail_pub = self.create_publisher(MarkerArray, '/swarm/markers/trails', 10)
        self.env_marker_pub = self.create_publisher(MarkerArray, '/swarm/markers/environment', latched)

        self.environment = []
        if world_file:
            try:
                self.environment = parse_world_environment(world_file)
                self.get_logger().info(f'Loaded {len(self.environment)} environment models from {world_file}')
            except (OSError, ET.ParseError) as exc:
                self.get_logger().error(f'Could not parse world file {world_file}: {exc}')
        else:
            self.get_logger().warn('No world_file parameter given: environment markers disabled')

        self.create_timer(0.05, self.publish_drones)   # 20 Hz
        self.create_timer(0.5, self.publish_trails)
        self.create_timer(2.0, self.publish_environment)
        self.publish_environment()
        self.get_logger().info('SwarmSimBridgeNode mirroring Gazebo drones into RViz2')

    def odom_callback(self, msg: Odometry, ns: str):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.poses[ns] = (p.x, p.y, p.z, q.x, q.y, q.z, q.w, yaw)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = f'{ns}/base_link'
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        trail = self.trails[ns]
        if not trail or math.dist(trail[-1], (p.x, p.y, p.z)) > 0.1:
            trail.append((p.x, p.y, p.z))
            del trail[:-self.trail_length]

    def _marker(self, ns, mid, mtype, stamp, frame='map'):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def publish_drones(self):
        stamp = self.get_clock().now().to_msg()
        self.rotor_angle = (self.rotor_angle + 1.3) % (2.0 * math.pi)
        array = MarkerArray()
        for idx, ns in enumerate(self.drone_namespaces):
            if ns not in self.poses:
                continue
            x, y, z, qx, qy, qz, qw, yaw = self.poses[ns]
            color = DRONE_COLORS.get(ns, (0.8, 0.8, 0.8))
            frame = f'{ns}/base_link'
            base = idx * 20
            airborne = z > 0.25

            body = self._marker('drone_bodies', base, Marker.CUBE, stamp, frame)
            body.scale.x, body.scale.y, body.scale.z = 0.24, 0.24, 0.08
            body.color.r, body.color.g, body.color.b, body.color.a = color[0], color[1], color[2], 0.95
            array.markers.append(body)

            for k, sign in enumerate((1.0, -1.0)):
                arm = self._marker('drone_arms', base + 1 + k, Marker.CUBE, stamp, frame)
                sz, cz = yaw_quaternion(sign * math.pi / 4.0)
                arm.pose.orientation.z, arm.pose.orientation.w = sz, cz
                arm.scale.x, arm.scale.y, arm.scale.z = 0.46, 0.025, 0.02
                arm.color.r = arm.color.g = arm.color.b = 0.18
                arm.color.a = 1.0
                array.markers.append(arm)

            for k, (rx, ry) in enumerate(ROTOR_OFFSETS):
                # Rotor disc plus a blade bar that spins while airborne
                disc = self._marker('drone_rotors', base + 3 + k, Marker.CYLINDER, stamp, frame)
                disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = rx, ry, 0.04
                disc.scale.x = disc.scale.y = 0.2
                disc.scale.z = 0.005
                disc.color.r, disc.color.g, disc.color.b, disc.color.a = 0.85, 0.85, 0.9, 0.35
                array.markers.append(disc)

                blade = self._marker('drone_blades', base + 7 + k, Marker.CUBE, stamp, frame)
                blade.pose.position.x, blade.pose.position.y, blade.pose.position.z = rx, ry, 0.045
                spin = (self.rotor_angle if airborne else 0.0) * (1 if k in (0, 3) else -1)
                blade.pose.orientation.z, blade.pose.orientation.w = yaw_quaternion(spin)
                blade.scale.x, blade.scale.y, blade.scale.z = 0.2, 0.02, 0.006
                blade.color.r = blade.color.g = blade.color.b = 0.1
                blade.color.a = 1.0
                array.markers.append(blade)

            nose = self._marker('drone_heading', base + 11, Marker.ARROW, stamp, frame)
            nose.scale.x, nose.scale.y, nose.scale.z = 0.35, 0.04, 0.04
            nose.color.r, nose.color.g, nose.color.b, nose.color.a = color[0], color[1], color[2], 1.0
            array.markers.append(nose)

            label = self._marker('drone_labels', base + 12, Marker.TEXT_VIEW_FACING, stamp)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, z + 0.5
            label.scale.z = 0.35
            label.text = f'{ns.upper()}  z={z:.1f}m'
            label.color.r, label.color.g, label.color.b, label.color.a = color[0], color[1], color[2], 1.0
            array.markers.append(label)
        self.drone_marker_pub.publish(array)

    def publish_trails(self):
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()
        for idx, ns in enumerate(self.drone_namespaces):
            trail = self.trails[ns]
            if len(trail) < 2:
                continue
            m = self._marker('trails', idx, Marker.LINE_STRIP, stamp)
            m.scale.x = 0.05
            color = DRONE_COLORS.get(ns, (0.8, 0.8, 0.8))
            m.color.r, m.color.g, m.color.b, m.color.a = color[0], color[1], color[2], 0.8
            m.points = [Point(x=px, y=py, z=pz) for px, py, pz in trail]
            array.markers.append(m)
        self.trail_pub.publish(array)

    def publish_environment(self):
        if not self.environment:
            return
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()
        for idx, (name, shape, (x, y, z, yaw), size, color) in enumerate(self.environment):
            m = self._marker('landing_pads' if name.startswith('pad') else 'obstacles', idx, shape, stamp)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.pose.orientation.z, m.pose.orientation.w = yaw_quaternion(yaw)
            m.scale.x, m.scale.y, m.scale.z = size
            alpha = 0.35 if name.startswith('wall') or name.startswith('partition') else 0.9
            m.color.r, m.color.g, m.color.b, m.color.a = color[0], color[1], color[2], alpha
            array.markers.append(m)
        self.env_marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmSimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
