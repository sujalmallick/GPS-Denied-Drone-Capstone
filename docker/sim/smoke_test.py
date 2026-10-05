#!/usr/bin/env python3
"""
Swarm simulation smoke test (docker/sim/smoke_test.py, run through smoke_test.sh)

Attaches to a RUNNING simulation and checks that the core topics exist and are publishing:
  /clock, /global_map and per drone /{ns}/odom, /{ns}/imu, /{ns}/depth/image,
  /{ns}/ground_truth_pose, /{ns}/map
For each topic it prints the message rate against wall time and against simulation time (/clock),
then the real-time factor measured from /clock and reported by Gazebo on /stats.

Rates are checked against simulation time. Default minimums (nominal rate in brackets):
  odom 28 (30), imu 190 (200), ground_truth_pose 28 (30), map 1.8 (2), /global_map 1.8 (2),
  depth/image 28 (30) with sensor_profile full or 9 (10) with realtime; real-time factor 0.8.

Exit status: 0 = all topics present, publishing and above their thresholds,
             1 = a topic is missing or silent, or a threshold is not met.

Examples:
    smoke_test.sh                                   # defaults for sensor_profile full
    smoke_test.sh --profile realtime --min-rtf 0    # realtime profile, no RTF check
    smoke_test.sh --min-rate imu=150 --no-default-rates
"""

import os

import argparse
import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message

PER_DRONE_TOPICS = ['odom', 'imu', 'depth/image', 'ground_truth_pose', 'map']

# Minimum sim-time rates, keyed by topic suffix
DEFAULT_MIN_RATES = {'odom': 28.0, 'imu': 190.0, 'ground_truth_pose': 28.0, 'map': 1.8, 'global_map': 1.8}
DEPTH_MIN_RATE = {'full': 28.0, 'realtime': 9.0}
DEFAULT_MIN_RTF = 0.8


def expected_topics(drones):
    topics = ['/clock']
    for ns in drones:
        topics += [f'/{ns}/{name}' for name in PER_DRONE_TOPICS]
    topics.append('/global_map')
    return topics


def parse_min_rates(items):
    """'depth/image=28' -> {'depth/image': 28.0}; a key matches a topic by its trailing path."""
    rates = {}
    for item in items:
        key, sep, value = item.partition('=')
        if not sep:
            raise SystemExit(f'--min-rate expects TOPIC=HZ, got {item!r}')
        rates[key.strip('/')] = float(value)
    return rates


def min_rate_for(topic, *rate_tables):
    """First matching threshold, searching the tables in order (CLI overrides before defaults)."""
    for table in rate_tables:
        for key, rate in table.items():
            if topic.strip('/') == key or topic.endswith('/' + key):
                return rate
    return None


def gz_stats_rtf(samples, timeout):
    """Mean real_time_factor over `samples` messages of Gazebo's /stats topic, or None."""
    try:
        out = subprocess.run(['gz', 'topic', '-e', '-t', '/stats', '-n', str(samples)],
                             capture_output=True, text=True, timeout=timeout).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    values = [float(v) for v in re.findall(r'real_time_factor:\s*([0-9.eE+-]+)', out)]
    return sum(values) / len(values) if values else None


class RateProbe(Node):

    def __init__(self):
        super().__init__('swarm_smoke_test')
        # Best effort + volatile is compatible with every publisher QoS (reliable, transient local)
        self.qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                              reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        self.counts = {}
        self.sim_first = None
        self.sim_last = None

    def discover(self, topics, timeout):
        deadline = time.monotonic() + timeout
        while True:
            types = dict(self.get_topic_names_and_types())
            found = {t: types[t][0] for t in topics if t in types}
            if len(found) == len(topics) or time.monotonic() > deadline:
                return found
            rclpy.spin_once(self, timeout_sec=0.5)

    def subscribe(self, topic_types):
        for topic, type_name in topic_types.items():
            self.counts[topic] = 0
            if topic == '/clock':
                self.create_subscription(Clock, topic, self._on_clock, self.qos)
            else:
                self.create_subscription(get_message(type_name), topic, self._counter(topic), self.qos, raw=True)

    def _counter(self, topic):
        def callback(_msg):
            self.counts[topic] += 1
        return callback

    def _on_clock(self, msg):
        self.counts['/clock'] += 1
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.sim_first is None:
            self.sim_first = t
        self.sim_last = t

    def reset(self):
        for topic in self.counts:
            self.counts[topic] = 0
        self.sim_first = self.sim_last = None


def main():
    parser = argparse.ArgumentParser(description='Smoke test for the running swarm simulation')
    parser.add_argument('--drones', default='drone_1,drone_2,drone_3', help='comma-separated namespaces')
    parser.add_argument('--duration', type=float, default=10.0, help='measurement window, wall seconds')
    parser.add_argument('--discover-timeout', type=float, default=15.0, help='seconds to wait for topics')
    parser.add_argument('--profile', choices=sorted(DEPTH_MIN_RATE),
                        default=os.environ.get('SENSOR_PROFILE', 'full'),
                        help='sensor_profile the simulation runs with (sets the depth/image minimum)')
    parser.add_argument('--min-rate', action='append', default=[], metavar='TOPIC=HZ',
                        help='minimum sim-time rate, matched on the topic suffix (repeatable), e.g. depth/image=28')
    parser.add_argument('--no-default-rates', action='store_true', help='only check --min-rate thresholds')
    parser.add_argument('--min-rtf', type=float, default=DEFAULT_MIN_RTF,
                        help=f'minimum real-time factor (default {DEFAULT_MIN_RTF}; 0 disables)')
    args = parser.parse_args()

    drones = [d.strip() for d in args.drones.split(',') if d.strip()]
    topics = expected_topics(drones)
    min_rates = parse_min_rates(args.min_rate)
    default_rates = {} if args.no_default_rates else \
        {**DEFAULT_MIN_RATES, 'depth/image': DEPTH_MIN_RATE[args.profile]}

    rclpy.init()
    probe = RateProbe()
    failures = []
    try:
        found = probe.discover(topics, args.discover_timeout)
        missing = [t for t in topics if t not in found]
        for topic in missing:
            failures.append(f'missing topic {topic}')

        probe.subscribe(found)
        warmup_end = time.monotonic() + 1.0
        while time.monotonic() < warmup_end:
            rclpy.spin_once(probe, timeout_sec=0.1)
        probe.reset()

        start = time.monotonic()
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(probe, timeout_sec=0.1)
        wall = time.monotonic() - start
        sim = (probe.sim_last - probe.sim_first) if probe.sim_first is not None else 0.0
    finally:
        probe.destroy_node()
        rclpy.shutdown()

    print(f'\nTopic rates over {wall:.1f} s wall / {sim:.1f} s sim time')
    print(f'{"topic":<32}{"msgs":>7}{"wall Hz":>10}{"sim Hz":>10}{"min Hz":>9}  status')
    for topic in topics:
        if topic not in found:
            print(f'{topic:<32}{"-":>7}{"-":>10}{"-":>10}{"":>9}  MISSING')
            continue
        count = probe.counts[topic]
        wall_hz = count / wall if wall > 0 else 0.0
        sim_hz = count / sim if sim > 0 else 0.0
        floor = min_rate_for(topic, min_rates, default_rates)
        status = 'ok'
        if count == 0:
            status = 'SILENT'
            failures.append(f'no messages on {topic}')
        elif floor is not None and sim_hz < floor:
            status = 'LOW'
            failures.append(f'{topic} at {sim_hz:.1f} Hz sim time, below {floor:g} Hz')
        floor_text = f'{floor:g}' if floor is not None else ''
        print(f'{topic:<32}{count:>7}{wall_hz:>10.1f}{sim_hz:>10.1f}{floor_text:>9}  {status}')

    clock_rtf = sim / wall if wall > 0 and sim > 0 else None
    stats_rtf = gz_stats_rtf(samples=10, timeout=10.0)
    print()
    print(f'RTF from /clock:          {clock_rtf:.3f}' if clock_rtf is not None else 'RTF from /clock:          n/a')
    print(f'RTF from Gazebo /stats:   {stats_rtf:.3f}' if stats_rtf is not None else 'RTF from Gazebo /stats:   n/a')
    if args.min_rtf > 0:
        rtf = stats_rtf if stats_rtf is not None else clock_rtf
        if rtf is None or rtf < args.min_rtf:
            failures.append(f'real-time factor {rtf if rtf is not None else "n/a"} below {args.min_rtf}')

    print()
    if failures:
        print('SMOKE TEST FAILED')
        for failure in failures:
            print(f'  - {failure}')
        return 1
    print('SMOKE TEST PASSED')
    return 0


if __name__ == '__main__':
    sys.exit(main())
