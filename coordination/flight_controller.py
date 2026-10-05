#!/usr/bin/env python3
"""
Per-Drone Offboard Flight Controller (coordination/flight_controller.py)

Closes the loop between the swarm task allocator and the Gazebo rotor-dynamics model:

  /{ns}/cmd_pose (goal) + /global_map --A*--> path --pursuit--> /{ns}/cmd_vel (body-frame twist)
                                                              /{ns}/enable  (arms the controller)

Gazebo's MulticopterVelocityControl turns the twist into motor speeds for the four
MulticopterMotorModel propellers, so every motion seen in the simulation is real physics.

Behaviour:
- ARMING:   publish /{ns}/enable until the vehicle responds
- TAKEOFF:  climb to `altitude` (default 1.5 m)
- MISSION:  follow an A* path (obstacles inflated by `inflation_radius`) to the latest
            cmd_pose; replanned at `replan_rate`. If no cmd_pose arrives for `command_timeout`
            seconds the drone drops the goal and holds position. The drone yaws towards its direction of travel
            so the forward depth camera always looks where it flies, and keeps
            `separation_radius` from the other drones (reactive repulsion).
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from coordination.algorithms import GridGeometry, plan_path, yaw_from_quaternion


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class FlightControllerNode(Node):
    def __init__(self):
        super().__init__('flight_controller')

        self.declare_parameter('namespace', 'drone_1')
        self.declare_parameter('drone_namespaces', ['drone_1', 'drone_2', 'drone_3'])
        self.declare_parameter('altitude', 1.5)
        self.declare_parameter('cruise_speed', 0.8)
        self.declare_parameter('max_yaw_rate', 0.8)
        self.declare_parameter('lookahead', 0.8)
        self.declare_parameter('inflation_radius', 0.5)
        self.declare_parameter('separation_radius', 1.5)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('replan_rate', 1.0)
        self.declare_parameter('command_timeout', 2.0)     # s without cmd_pose -> hold position

        self.ns = self.get_parameter('namespace').value
        self.others = [n for n in self.get_parameter('drone_namespaces').value if n != self.ns]
        self.altitude = self.get_parameter('altitude').value
        self.cruise = self.get_parameter('cruise_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.lookahead = self.get_parameter('lookahead').value
        self.inflation = self.get_parameter('inflation_radius').value
        self.sep_radius = self.get_parameter('separation_radius').value
        self.command_timeout = self.get_parameter('command_timeout').value

        self.state = 'ARMING'
        self.pose = None          # (x, y, z, yaw)
        self.other_xy = {}
        self.goal = None
        self.goal_stamp = None
        self.path = []
        self.global_map = None
        self.arming_ticks = 0

        self.create_subscription(Odometry, f'/{self.ns}/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, f'/{self.ns}/cmd_pose', self.goal_callback, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/global_map', self.map_callback, latched)
        for other in self.others:
            self.create_subscription(
                Odometry, f'/{other}/odom',
                lambda msg, n=other: self.other_xy.__setitem__(
                    n, (msg.pose.pose.position.x, msg.pose.pose.position.y)), 10)

        self.cmd_pub = self.create_publisher(Twist, f'/{self.ns}/cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool, f'/{self.ns}/enable', 10)
        self.path_pub = self.create_publisher(Path, f'/{self.ns}/plan', 10)

        self.create_timer(1.0 / self.get_parameter('control_rate').value, self.control_step)
        self.create_timer(1.0 / self.get_parameter('replan_rate').value, self.replan)
        self.get_logger().info(f'[{self.ns}] flight controller ready (altitude {self.altitude} m)')

    # ------------------------------------------------------------------ inputs
    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, p.z, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def goal_callback(self, msg: PoseStamped):
        goal = (msg.pose.position.x, msg.pose.position.y)
        self.goal_stamp = self.get_clock().now()
        if self.goal is None or math.dist(goal, self.goal) > 0.3:
            self.goal = goal
            self.path = []
            if self.state == 'MISSION':
                self.replan()

    def map_callback(self, msg: OccupancyGrid):
        self.global_map = msg

    # ---------------------------------------------------------------- planning
    def replan(self):
        if self.state != 'MISSION' or self.goal is None or self.pose is None or self.global_map is None:
            return
        geom = GridGeometry.from_info(self.global_map.info)
        grid = np.asarray(self.global_map.data, dtype=np.int8).reshape(geom.height, geom.width)
        path = plan_path(grid, geom, self.pose[:2], self.goal, inflation_radius=self.inflation)
        if path is None:
            self.get_logger().warn(
                f'[{self.ns}] no collision-free path to ({self.goal[0]:.1f}, {self.goal[1]:.1f})',
                throttle_duration_sec=5.0)
            self.path = []
        else:
            self.path = path
        self.publish_path()

    def publish_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        points = ([self.pose[:2]] if self.pose else []) + list(self.path)
        for x, y in points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = self.altitude
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def carrot(self):
        """Pure-pursuit target: first path point beyond the lookahead distance."""
        x, y = self.pose[:2]
        while len(self.path) > 1 and math.dist((x, y), self.path[0]) < self.lookahead:
            self.path.pop(0)
        return self.path[0] if self.path else None

    # ----------------------------------------------------------------- control
    def control_step(self):
        if self.pose is None:
            return
        x, y, z, yaw = self.pose
        vx = vy = 0.0
        yaw_rate = 0.0
        vz = float(np.clip(1.2 * (self.altitude - z), -0.6, 0.6))

        if self.state == 'ARMING':
            # Wait for the ros_gz_bridge to subscribe before arming, otherwise enable is lost
            if self.enable_pub.get_subscription_count() > 0:
                self.enable_pub.publish(Bool(data=True))
                self.arming_ticks += 1
            if self.arming_ticks > 20:
                self.state = 'TAKEOFF'
                self.get_logger().info(f'[{self.ns}] armed, taking off')
        elif self.state == 'TAKEOFF':
            self.enable_pub.publish(Bool(data=True))
            if z > self.altitude - 0.15:
                # No 360 deg scan: three drones spinning 4 m apart all sense the same space, which
                # is pure overlap. Each starts from its own forward view and the allocator spreads them.
                self.state = 'MISSION'
                self.get_logger().info(f'[{self.ns}] at altitude, starting exploration')
                self.replan()
        elif self.state == 'MISSION':
            if self.goal is not None and self.goal_stamp is not None:
                age = (self.get_clock().now() - self.goal_stamp).nanoseconds * 1e-9
            else:
                age = 0.0
            if age > self.command_timeout:
                self.get_logger().info(
                    f'[{self.ns}] no cmd_pose for {age:.1f}s (goal released or allocator silent), holding position')
                self.goal = None
                self.path = []
                self.publish_path()
            target = self.carrot() if self.goal is not None else None
            if target is not None:
                dx, dy = target[0] - x, target[1] - y
                dist_goal = math.dist((x, y), self.goal) if self.goal else 0.0
                speed = min(self.cruise, 0.8 * dist_goal + 0.15)
                heading = math.atan2(dy, dx)
                yaw_err = wrap(heading - yaw)
                yaw_rate = float(np.clip(1.5 * yaw_err, -self.max_yaw_rate, self.max_yaw_rate))
                # Only translate while facing the direction of travel, so the depth camera sees it
                speed *= max(0.0, math.cos(yaw_err)) ** 2
                norm = math.hypot(dx, dy) or 1.0
                vx, vy = speed * dx / norm, speed * dy / norm

        # Reactive inter-drone separation
        for ox, oy in self.other_xy.values():
            d = math.dist((x, y), (ox, oy))
            if 1e-3 < d < self.sep_radius:
                push = 1.0 * (self.sep_radius - d) / self.sep_radius
                vx += push * (x - ox) / d
                vy += push * (y - oy) / d

        speed = math.hypot(vx, vy)
        if speed > self.cruise:
            vx, vy = vx * self.cruise / speed, vy * self.cruise / speed

        # MulticopterVelocityControl expects the twist in the (yaw-rotated) body frame
        cmd = Twist()
        if self.state != 'ARMING':
            cmd.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy
            cmd.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
            cmd.linear.z = vz
            cmd.angular.z = yaw_rate
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FlightControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
