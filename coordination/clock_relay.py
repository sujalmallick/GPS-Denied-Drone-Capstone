#!/usr/bin/env python3
"""
Simulation Clock Relay (coordination/clock_relay.py)

Gazebo publishes its clock on every physics step (500 Hz with the 2 ms step). Bridging that
directly wakes every use_sim_time node 500 times per second, and with ~11 Python nodes the CPU
churn starves Gazebo's sensor rendering threads and drags the simulation below real time.

This node subscribes to the Gazebo /clock with gz-transport and republishes ROS /clock
(rosgraph_msgs/Clock) at most `max_rate` Hz (default 100 Hz). All swarm nodes run at 20 Hz or
slower, so 10 ms clock resolution loses nothing.
"""

import threading

import rclpy
from gz.msgs10.clock_pb2 import Clock as GzClock
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class ClockRelayNode(Node):
    def __init__(self):
        super().__init__('clock_relay')
        self.declare_parameter('gz_topic', '/clock')
        self.declare_parameter('max_rate', 100.0)
        self.min_period_ns = int(1e9 / self.get_parameter('max_rate').value)

        # Reliable publisher: compatible with both best-effort (rclcpp ClockQoS) and reliable subscribers
        self.pub = self.create_publisher(Clock, '/clock', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.last_ns = None
        self.lock = threading.Lock()

        self.gz = GzNode()
        topic = self.get_parameter('gz_topic').value
        if not self.gz.subscribe(GzClock, topic, self.on_gz_clock):
            raise RuntimeError(f'could not subscribe to Gazebo clock topic {topic}')
        self.get_logger().info(
            f'Relaying Gazebo {topic} to ROS /clock at <= {self.get_parameter("max_rate").value:.0f} Hz')

    def on_gz_clock(self, msg: GzClock):
        sim_ns = msg.sim.sec * 1_000_000_000 + msg.sim.nsec
        with self.lock:
            if self.last_ns is not None and 0 <= sim_ns - self.last_ns < self.min_period_ns:
                return
            self.last_ns = sim_ns
        out = Clock()
        out.clock.sec = msg.sim.sec
        out.clock.nanosec = msg.sim.nsec
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ClockRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
