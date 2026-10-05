#!/usr/bin/env python3
"""
Shared Global Occupancy Grid Node (coordination/map_merger.py)
Capstone Technical Report - Section 6.2.1

Subscribes to the local occupancy grids built from each drone's depth camera:
- /drone_1/map, /drone_2/map, /drone_3/map

Fuses them in real time (2.0 Hz) into a unified global occupancy grid:
- /global_map

Publishes Swarm Metrics (Section 7):
- /swarm/metrics/coverage_pct (std_msgs/Float32)  known cells inside the arena bounds
- /swarm/metrics/overlap_rate (std_msgs/Float32)  explored cells seen by >= 2 drones

Conservative Fusion Rule (coordination.algorithms.fuse_occupancy):
- Occupied (100) if any drone reports >= 65
- Free (0) only if every drone that observed the cell reports free
- Unknown (-1) otherwise
"""

import threading

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32

from coordination.algorithms import (GridGeometry, coverage_pct, fuse_occupancy,
                                     overlap_rate, region_mask)


class MapMergerNode(Node):
    def __init__(self):
        super().__init__('map_merger')

        self.declare_parameter('drone_namespaces', ['drone_1', 'drone_2', 'drone_3'])
        self.declare_parameter('publish_rate', 2.0)  # Hz
        self.declare_parameter('global_frame_id', 'map')
        # Arena extent [xmin, xmax, ymin, ymax] used for the coverage metric
        self.declare_parameter('arena_bounds', [-12.0, 12.0, -9.0, 11.0])

        self.drone_namespaces = self.get_parameter('drone_namespaces').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.global_frame = self.get_parameter('global_frame_id').value
        self.arena_bounds = list(self.get_parameter('arena_bounds').value)

        self.lock = threading.Lock()
        self.local_maps = {ns: None for ns in self.drone_namespaces}
        self.geometry = None
        self.reference_info = None
        self.arena_mask = None

        self.subs = []
        for ns in self.drone_namespaces:
            topic = f'/{ns}/map'
            self.subs.append(self.create_subscription(
                OccupancyGrid, topic, lambda msg, n=ns: self.map_callback(msg, n), 10))
            self.get_logger().info(f'Subscribed to local occupancy grid: {topic}')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.global_map_pub = self.create_publisher(OccupancyGrid, '/global_map', latched)
        self.coverage_pub = self.create_publisher(Float32, '/swarm/metrics/coverage_pct', 10)
        self.overlap_pub = self.create_publisher(Float32, '/swarm/metrics/overlap_rate', 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.merge_and_publish)

        self.get_logger().info(
            f'MapMergerNode initialized for swarm namespaces: {self.drone_namespaces} @ {self.publish_rate} Hz'
        )

    def map_callback(self, msg: OccupancyGrid, namespace: str):
        geom = GridGeometry.from_info(msg.info)
        with self.lock:
            if self.geometry is None:
                self.geometry = geom
                self.reference_info = msg.info
                self.arena_mask = region_mask(geom, self.arena_bounds)
            elif not geom.same_as(self.geometry):
                self.get_logger().warn(
                    f'Ignoring /{namespace}/map: grid geometry differs from the swarm reference grid',
                    throttle_duration_sec=5.0)
                return
            self.local_maps[namespace] = np.asarray(msg.data, dtype=np.int8)

    def merge_and_publish(self):
        with self.lock:
            arrays = [m for m in self.local_maps.values() if m is not None]
            if not arrays:
                return
            geom = self.geometry
            info = self.reference_info
            mask = self.arena_mask

        merged = fuse_occupancy(arrays)

        out_msg = OccupancyGrid()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = self.global_frame
        out_msg.info = info
        out_msg.data = merged.tolist()
        self.global_map_pub.publish(out_msg)

        grid2d = merged.reshape(geom.height, geom.width)
        self.coverage_pub.publish(Float32(data=float(coverage_pct(grid2d, mask))))
        if len(arrays) >= 2:
            self.overlap_pub.publish(Float32(data=float(overlap_rate(arrays))))


def main(args=None):
    rclpy.init(args=args)
    node = MapMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
