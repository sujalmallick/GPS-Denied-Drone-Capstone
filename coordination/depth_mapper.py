#!/usr/bin/env python3
"""
Per-Drone Depth Mapping Node (coordination/depth_mapper.py)

Builds each drone's local occupancy grid from its own simulated sensors:
- /{ns}/depth/image  sensor_msgs/Image (32FC1 metric depth from the Gazebo depth camera)
- /{ns}/odom         nav_msgs/Odometry (Gazebo odometry, world frame)

Every depth frame (processed at `update_rate`) is turned into one 2D ray per sampled image
column: free space up to the nearest return inside the obstacle height band, an obstacle at
that return. Rays are integrated into a log-odds grid that is published as
- /{ns}/map  nav_msgs/OccupancyGrid, 200 x 200 cells at 0.2 m, values {-1, 0, 100}, 2 Hz

All drones share the same grid geometry so map_merger can fuse them cell by cell.
"""

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from coordination.algorithms import (GridGeometry, OccupancyLogOdds, depth_image_to_rays,
                                     quaternion_to_matrix)


class DepthMapperNode(Node):
    def __init__(self):
        super().__init__('depth_mapper')

        self.declare_parameter('namespace', 'drone_1')
        self.declare_parameter('grid_size', 200)
        self.declare_parameter('resolution', 0.2)
        self.declare_parameter('origin', [-20.0, -20.0])
        self.declare_parameter('horizontal_fov', 1.5184)      # must match the SDF depth camera
        self.declare_parameter('camera_offset', [0.13, 0.0, 0.0])  # camera position in base_link
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('min_range', 0.35)   # ignore own propellers / airframe in view
        self.declare_parameter('obstacle_z_min', 0.3)
        self.declare_parameter('obstacle_z_max', 2.7)
        self.declare_parameter('pixel_stride', 0)            # 0 = auto: ~80 rays per frame
        self.declare_parameter('min_mapping_altitude', 0.8)
        self.declare_parameter('drone_namespaces', ['drone_1', 'drone_2', 'drone_3'])
        self.declare_parameter('teammate_radius', 0.6)   # depth hits this close to a teammate are ignored
        self.declare_parameter('update_rate', 5.0)
        self.declare_parameter('publish_rate', 2.0)

        self.ns = self.get_parameter('namespace').value
        size = self.get_parameter('grid_size').value
        res = self.get_parameter('resolution').value
        origin = self.get_parameter('origin').value
        self.hfov = self.get_parameter('horizontal_fov').value
        self.cam_offset = np.array(self.get_parameter('camera_offset').value, dtype=float)
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.z_min = self.get_parameter('obstacle_z_min').value
        self.z_max = self.get_parameter('obstacle_z_max').value
        self.stride = self.get_parameter('pixel_stride').value
        self.min_alt = self.get_parameter('min_mapping_altitude').value

        self.geom = GridGeometry(size, size, res, origin[0], origin[1])
        self.grid = OccupancyLogOdds(self.geom)
        self.latest_depth = None
        self.latest_odom = None
        self.frames_integrated = 0

        self.teammate_radius = self.get_parameter('teammate_radius').value
        self.teammates = {}
        for other in self.get_parameter('drone_namespaces').value:
            if other != self.ns:
                self.create_subscription(
                    Odometry, f'/{other}/odom',
                    lambda m, n=other: self.teammates.__setitem__(n, (m.pose.pose.position.x, m.pose.pose.position.y)),
                    10)
        self.create_subscription(Image, f'/{self.ns}/depth/image', self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, f'/{self.ns}/odom', self.odom_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, f'/{self.ns}/map', 10)

        self.create_timer(1.0 / self.get_parameter('update_rate').value, self.integrate)
        self.create_timer(1.0 / self.get_parameter('publish_rate').value, self.publish_map)
        self.get_logger().info(
            f'[{self.ns}] depth mapper: {size}x{size} @ {res} m, range {self.max_range} m')

    def depth_callback(self, msg: Image):
        self.latest_depth = msg

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def integrate(self):
        msg, odom = self.latest_depth, self.latest_odom
        if msg is None or odom is None:
            return
        self.latest_depth = None
        if msg.encoding != '32FC1':
            self.get_logger().error(f'Unsupported depth encoding {msg.encoding}', throttle_duration_sec=10.0)
            return
        p = odom.pose.pose.position
        if p.z < self.min_alt:
            return
        q = odom.pose.pose.orientation
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        stride = self.stride if self.stride > 0 else max(1, msg.width // 80)
        origin_xy, azimuths, free_lengths, hit_xy = depth_image_to_rays(
            depth, self.hfov, self.cam_offset, quaternion_to_matrix(q.x, q.y, q.z, q.w),
            np.array([p.x, p.y, p.z]), stride=stride, max_range=self.max_range,
            obstacle_z_min=self.z_min, obstacle_z_max=self.z_max, min_range=self.min_range)
        # Teammate filtering: another drone in view is not a static obstacle. Its rays still count
        # as free space up to the teammate, but no occupied cell is written there.
        if len(hit_xy) and self.teammates:
            mates = np.array(list(self.teammates.values()))
            near_mate = (np.linalg.norm(hit_xy[:, None, :] - mates[None, :, :], axis=2)
                         < self.teammate_radius).any(axis=1)
            hit_xy = hit_xy[~near_mate]
        self.grid.integrate_rays(origin_xy, azimuths, free_lengths, hit_xy)
        self.frames_integrated += 1

    def publish_map(self):
        if self.frames_integrated == 0:
            return
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.info.resolution = self.geom.resolution
        out.info.width = self.geom.width
        out.info.height = self.geom.height
        out.info.origin.position.x = self.geom.origin_x
        out.info.origin.position.y = self.geom.origin_y
        out.info.origin.orientation.w = 1.0
        out.data = self.grid.to_occupancy().reshape(-1).tolist()
        self.map_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DepthMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
