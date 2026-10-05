#!/usr/bin/env python3
"""
Swarm Task Allocator Node (coordination/task_allocator.py)
Capstone Technical Report - Section 6.2.3 & Algorithm 1

Centralized coordination layer assigning non-overlapping exploration regions to 3 drones:
- Subscribes to /global_map (fused occupancy grid)
- Tracks drone poses from /{ns}/odom (Gazebo) or /{ns}/mavros/local_position/pose (hardware)
- Extracts 8-connected frontiers and clusters them
- Executes Algorithm 1: Greedy No-Overlap Frontier Assignment
    U(f) = alpha * InformationGain(f) * discount(f) - beta * ||p_i - f||
    reject f within lock_radius (2.0 m) of any other drone or other drone's locked region;
    discount(f) lowers the gain of frontiers a teammate will sense anyway (spreads the swarm)
- Keeps a target while it is still on a frontier (waypoint persistence), releases the lock on
  arrival (<= 0.8 m), and blacklists targets the drone makes no progress towards
- Publishes waypoints on /{ns}/cmd_pose at 1.0 Hz and RViz2 markers for frontiers and locks
"""

import math
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from coordination.algorithms import (GridGeometry, extract_frontier_clusters, greedy_assign,
                                     inflate_obstacles, region_mask)

DRONE_COLORS = {
    'drone_1': (0.01, 0.52, 0.78),
    'drone_2': (0.96, 0.62, 0.04),
    'drone_3': (0.55, 0.36, 0.96),
}


class TaskAllocatorNode(Node):
    def __init__(self):
        super().__init__('task_allocator')

        self.declare_parameter('drone_namespaces', ['drone_1', 'drone_2', 'drone_3'])
        self.declare_parameter('allocation_frequency', 1.0)
        self.declare_parameter('lock_radius', 2.0)
        self.declare_parameter('arrival_radius', 0.8)
        self.declare_parameter('min_cluster_size', 5)        # cells; smaller frontier crumbs are ignored
        self.declare_parameter('target_altitude', 1.5)
        self.declare_parameter('alpha', 1.0)                 # weight of information gain (per metre of frontier)
        self.declare_parameter('beta', 1.0)                  # weight of travel distance
        self.declare_parameter('min_flight_altitude', 1.0)   # drones below this are not yet tasked
        self.declare_parameter('progress_timeout', 20.0)     # s without getting 0.5 m closer -> blacklist
        self.declare_parameter('blacklist_radius', 1.0)
        # Operating area [xmin, xmax, ymin, ymax]: frontiers outside it (e.g. seen through gaps in a
        # partially mapped wall) are ignored. Frontier cells closer than obstacle_clearance to a mapped
        # obstacle are ignored too: the flight controller could not reach them safely.
        self.declare_parameter('arena_bounds', [-12.0, 12.0, -9.0, 11.0])
        self.declare_parameter('obstacle_clearance', 0.4)
        # Coordinated exploration: discount frontier gain near teammates (Burgard et al. 2005)
        self.declare_parameter('sensor_radius', 5.0)
        # Keep a fresh target at least this long before dropping it because its frontier moved
        self.declare_parameter('min_commit_time', 5.0)

        self.drone_namespaces = self.get_parameter('drone_namespaces').value
        self.allocation_freq = self.get_parameter('allocation_frequency').value
        self.lock_radius = self.get_parameter('lock_radius').value
        self.arrival_radius = self.get_parameter('arrival_radius').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.target_alt = self.get_parameter('target_altitude').value
        self.alpha = self.get_parameter('alpha').value
        self.beta = self.get_parameter('beta').value
        self.min_flight_alt = self.get_parameter('min_flight_altitude').value
        self.progress_timeout = self.get_parameter('progress_timeout').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.arena_bounds = list(self.get_parameter('arena_bounds').value)
        self.clearance = self.get_parameter('obstacle_clearance').value
        self.arena_mask = None
        self.sensor_radius = self.get_parameter('sensor_radius').value
        self.min_commit = self.get_parameter('min_commit_time').value
        self.assigned_at = {ns: 0.0 for ns in self.drone_namespaces}
        self.exploration_complete = False

        self.lock = threading.Lock()
        self.global_map = None
        self.drone_poses = {ns: None for ns in self.drone_namespaces}
        self.current_waypoints = {ns: None for ns in self.drone_namespaces}
        self.progress = {ns: None for ns in self.drone_namespaces}  # (best distance, time of last improvement)
        self.blacklist = []

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, '/global_map', self.map_callback, latched)

        self.pose_subs = []
        self.waypoint_pubs = {}
        for ns in self.drone_namespaces:
            self.pose_subs.append(self.create_subscription(
                Odometry, f'/{ns}/odom', lambda msg, n=ns: self.pose_callback(msg.pose.pose, n), 10))
            self.pose_subs.append(self.create_subscription(
                PoseStamped, f'/{ns}/mavros/local_position/pose',
                lambda msg, n=ns: self.pose_callback(msg.pose, n), 10))
            self.waypoint_pubs[ns] = self.create_publisher(PoseStamped, f'/{ns}/cmd_pose', 10)

        self.frontier_marker_pub = self.create_publisher(MarkerArray, '/swarm/markers/frontiers', 10)
        self.locked_marker_pub = self.create_publisher(MarkerArray, '/swarm/markers/locked_regions', 10)

        self.alloc_timer = self.create_timer(1.0 / self.allocation_freq, self.allocation_cycle)

        self.get_logger().info(
            f'TaskAllocatorNode initialized with {len(self.drone_namespaces)} drones at '
            f'{self.allocation_freq} Hz. Lock radius: {self.lock_radius}m.')

    def map_callback(self, msg: OccupancyGrid):
        with self.lock:
            self.global_map = msg

    def pose_callback(self, pose, namespace: str):
        with self.lock:
            self.drone_poses[namespace] = (pose.position.x, pose.position.y, pose.position.z)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def allocation_cycle(self):
        with self.lock:
            occ = self.global_map
            poses = dict(self.drone_poses)
        if occ is None:
            return
        airborne = {ns: p for ns, p in poses.items() if p is not None and p[2] >= self.min_flight_alt}
        if not airborne:
            return

        geom = GridGeometry.from_info(occ.info)
        grid = np.asarray(occ.data, dtype=np.int8).reshape(geom.height, geom.width)
        if self.arena_mask is None or self.arena_mask.shape != grid.shape:
            b = self.arena_bounds
            margin = self.clearance
            self.arena_mask = region_mask(geom, [b[0] + margin, b[1] - margin, b[2] + margin, b[3] - margin])
        valid = self.arena_mask & ~inflate_obstacles(grid, self.clearance / geom.resolution)
        clusters = extract_frontier_clusters(grid, geom, self.min_cluster_size, valid_mask=valid)
        now = self.now_sec()

        needing = []
        for ns, pos in airborne.items():
            target = self.current_waypoints[ns]
            if target is None:
                needing.append(ns)
                continue
            dist = math.dist(pos[:2], target)

            # Arrival: release the spatial exclusion lock
            if dist <= self.arrival_radius:
                self.release(ns)
                needing.append(ns)
                continue

            # Progress watchdog: unreachable targets are blacklisted and the lock released
            best, since = self.progress[ns]
            if dist < best - 0.5:
                self.progress[ns] = (dist, now)
            elif now - since > self.progress_timeout:
                self.get_logger().warn(
                    f'{ns}: no progress towards ({target[0]:.1f}, {target[1]:.1f}) for '
                    f'{self.progress_timeout:.0f}s, blacklisting target')
                self.blacklist.append(target)
                self.release(ns)
                needing.append(ns)
                continue

            # Waypoint persistence: keep the target while it still borders unexplored space (and for
            # at least min_commit_time, so small frontier shifts do not make the drone zig-zag)
            if now - self.assigned_at[ns] >= self.min_commit and                     not any(math.dist(f['target'], target) < self.lock_radius for f in clusters):
                self.release(ns)
                needing.append(ns)

        # Exploration is complete once no worthwhile frontier is left: drones hold position instead of
        # criss-crossing the explored room for crumbs (which would only add sensor overlap).
        done = not clusters and all(self.current_waypoints[ns] is None for ns in airborne)
        if done and not self.exploration_complete:
            self.get_logger().info('EXPLORATION COMPLETE: no frontiers left, drones holding position')
        elif not done and self.exploration_complete:
            self.get_logger().info('New frontiers appeared, resuming exploration')
        self.exploration_complete = done

        locked = {ns: wp for ns, wp in self.current_waypoints.items() if wp is not None}
        positions = {ns: p[:2] for ns, p in airborne.items()}
        assignments = greedy_assign(
            positions, needing, clusters, locked,
            lock_radius=self.lock_radius, alpha=self.alpha, beta=self.beta,
            blacklist=self.blacklist, blacklist_radius=self.blacklist_radius,
            sensor_radius=self.sensor_radius)

        for ns, target in assignments.items():
            self.current_waypoints[ns] = target
            self.progress[ns] = (math.dist(positions[ns], target), now)
            self.assigned_at[ns] = now
            self.get_logger().info(f'{ns} -> frontier ({target[0]:.2f}, {target[1]:.2f})')

        stamp = self.get_clock().now().to_msg()
        for ns, target in self.current_waypoints.items():
            if target is None:
                continue
            cmd = PoseStamped()
            cmd.header.stamp = stamp
            cmd.header.frame_id = 'map'
            cmd.pose.position.x = target[0]
            cmd.pose.position.y = target[1]
            cmd.pose.position.z = self.target_alt
            cmd.pose.orientation.w = 1.0
            self.waypoint_pubs[ns].publish(cmd)

        self.publish_rviz_markers(clusters, stamp)

    def release(self, ns):
        self.current_waypoints[ns] = None
        self.progress[ns] = None

    def publish_rviz_markers(self, clusters, stamp):
        f_array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        f_array.markers.append(clear)
        for i, c in enumerate(clusters):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = c['target'][0]
            m.pose.position.y = c['target'][1]
            m.pose.position.z = self.target_alt
            m.pose.orientation.w = 1.0
            size = min(0.3 + 0.02 * c['size'], 0.8)
            m.scale.x = m.scale.y = m.scale.z = size
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.8, 0.0, 0.85
            f_array.markers.append(m)
        self.frontier_marker_pub.publish(f_array)

        l_array = MarkerArray()
        for idx, ns in enumerate(self.drone_namespaces):
            target = self.current_waypoints[ns]
            lm = Marker()
            lm.header.frame_id = 'map'
            lm.header.stamp = stamp
            lm.ns = 'locked_zones'
            lm.id = idx
            if target is None:
                lm.action = Marker.DELETE
                l_array.markers.append(lm)
                continue
            lm.type = Marker.CYLINDER
            lm.action = Marker.ADD
            lm.pose.position.x = target[0]
            lm.pose.position.y = target[1]
            lm.pose.position.z = self.target_alt
            lm.pose.orientation.w = 1.0
            lm.scale.x = lm.scale.y = self.lock_radius * 2.0
            lm.scale.z = 2.0
            col = DRONE_COLORS.get(ns, (0.2, 0.2, 0.8))
            lm.color.r, lm.color.g, lm.color.b, lm.color.a = col[0], col[1], col[2], 0.25
            l_array.markers.append(lm)
        self.locked_marker_pub.publish(l_array)


def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
