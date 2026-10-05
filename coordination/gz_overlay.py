#!/usr/bin/env python3
"""
Swarm Intelligence Overlay for the Gazebo GUI (coordination/gz_overlay.py)

Draws what the swarm knows and decides directly inside the Gazebo Harmonic window, next to the
physically simulated drones, so the whole mission is visible in ONE 3D view:

- Map merging:  floor tiles of /global_map coloured by WHICH drone mapped each free cell
                (drone colour), cells mapped by 2+ drones (overlap, white) and fused obstacles (red)
- Frontiers:    gold spheres (from the task allocator)
- Algorithm 1:  translucent 2.0 m exclusion cylinders around every locked target
- Planning:     each drone's A* path and its flown trail
- Drones:       a beacon per drone (halo, altitude stem, heading) in its identity colour
- Gauges:       live coverage (green) and overlap (orange) bars above the north wall

All markers use GUI visibility: they are drawn in the Gazebo window only and are invisible to the
drones' depth cameras, so the overlay never feeds back into perception.

Requires the Gazebo GUI (the MarkerManager GUI plugin serves /marker_array); with a headless server
the node simply waits.
"""

import math
import threading
import time

import numpy as np
import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.marker_v_pb2 import Marker_V
from gz.transport13 import Node as GzNode
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32
from visualization_msgs.msg import MarkerArray

DRONE_COLORS = {
    'drone_1': (0.01, 0.52, 0.78),
    'drone_2': (0.96, 0.62, 0.04),
    'drone_3': (0.55, 0.36, 0.96),
}
OVERLAP_COLOR = (0.95, 0.95, 0.95)
OBSTACLE_COLOR = (0.9, 0.12, 0.12)
FRONTIER_COLOR = (1.0, 0.8, 0.0)


def merge_rectangles(mask):
    """Cover the True cells of a 2D mask with axis-aligned rectangles (row runs merged vertically).

    Returns a list of (row0, row1, col0, col1) with inclusive-exclusive bounds.
    """
    rects = []
    open_rects = {}  # (c0, c1) -> row0
    for r in range(mask.shape[0] + 1):
        runs = set()
        if r < mask.shape[0]:
            row = mask[r].astype(np.int8)
            edges = np.diff(np.concatenate(([0], row, [0])))
            starts = np.nonzero(edges == 1)[0]
            ends = np.nonzero(edges == -1)[0]
            runs = set(zip(starts.tolist(), ends.tolist()))
        for key in list(open_rects):
            if key not in runs:
                rects.append((open_rects.pop(key), r, key[0], key[1]))
        for key in runs:
            open_rects.setdefault(key, r)
    return rects


class GzOverlayNode(Node):
    def __init__(self):
        super().__init__('gz_overlay')

        self.declare_parameter('drone_namespaces', ['drone_1', 'drone_2', 'drone_3'])
        self.declare_parameter('map_rate', 0.5)
        self.declare_parameter('dynamic_rate', 1.0)
        self.declare_parameter('hud_position', [0.0, 11.6, 3.6])  # gauge centre above the north wall
        self.declare_parameter('trail_length', 400)
        self.declare_parameter('arena_bounds', [-12.0, 12.0, -9.0, 11.0])   # map tiles drawn inside only

        self.drones = self.get_parameter('drone_namespaces').value
        self.hud_position = self.get_parameter('hud_position').value
        self.trail_length = self.get_parameter('trail_length').value

        self.lock = threading.Lock()
        self.global_map = None
        self.local_maps = {}
        self.poses = {}
        self.trails = {ns: [] for ns in self.drones}
        self.plans = {}
        self.frontiers = []
        self.locked = {}
        self.coverage = None
        self.overlap = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/global_map', self.on_global_map, latched)
        self.create_subscription(MarkerArray, '/swarm/markers/frontiers', self.on_frontiers, 10)
        self.create_subscription(MarkerArray, '/swarm/markers/locked_regions', self.on_locked, 10)
        self.create_subscription(Float32, '/swarm/metrics/coverage_pct',
                                 lambda m: setattr(self, 'coverage', m.data), 10)
        self.create_subscription(Float32, '/swarm/metrics/overlap_rate',
                                 lambda m: setattr(self, 'overlap', m.data), 10)
        for ns in self.drones:
            self.create_subscription(OccupancyGrid, f'/{ns}/map',
                                     lambda m, n=ns: self.on_local_map(m, n), 10)
            self.create_subscription(Odometry, f'/{ns}/odom', lambda m, n=ns: self.on_odom(m, n), 10)
            self.create_subscription(Path, f'/{ns}/plan', lambda m, n=ns: self.on_plan(m, n), 10)

        self.gz = GzNode()
        self.running = True
        self.worker = threading.Thread(target=self.draw_loop, daemon=True)
        self.worker.start()
        self.get_logger().info('Gazebo GUI overlay started (waiting for the Gazebo GUI marker service)')

    # ------------------------------------------------------------------ ROS inputs
    def on_global_map(self, msg):
        with self.lock:
            self.global_map = msg

    def on_local_map(self, msg, ns):
        with self.lock:
            self.local_maps[ns] = np.asarray(msg.data, dtype=np.int8)

    def on_odom(self, msg, ns):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.poses[ns] = (p.x, p.y, p.z, yaw)
            trail = self.trails[ns]
            if not trail or math.dist(trail[-1], (p.x, p.y, p.z)) > 0.15:
                trail.append((p.x, p.y, p.z))
                del trail[:-self.trail_length]

    def on_plan(self, msg, ns):
        with self.lock:
            self.plans[ns] = [(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z) for ps in msg.poses]

    def on_frontiers(self, msg):
        with self.lock:
            self.frontiers = [(m.pose.position.x, m.pose.position.y, m.pose.position.z, m.scale.x)
                              for m in msg.markers if m.action == m.ADD]

    def on_locked(self, msg):
        with self.lock:
            for m in msg.markers:
                key = m.id
                if m.action == m.ADD:
                    self.locked[key] = (m.pose.position.x, m.pose.position.y, m.pose.position.z,
                                        m.scale.x / 2.0, (m.color.r, m.color.g, m.color.b))
                else:
                    self.locked.pop(key, None)

    # ------------------------------------------------------------------ marker helpers
    @staticmethod
    def _marker(ns, mid, mtype, color, alpha=1.0):
        m = Marker()
        m.ns = ns
        m.id = mid
        m.action = Marker.ADD_MODIFY
        m.type = mtype
        m.visibility = Marker.GUI
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        for c in (m.material.ambient, m.material.diffuse):
            c.r, c.g, c.b, c.a = color[0], color[1], color[2], alpha
        m.material.emissive.r, m.material.emissive.g, m.material.emissive.b = (v * 0.4 for v in color)
        m.material.emissive.a = alpha
        return m

    @staticmethod
    def _add_point(m, x, y, z):
        p = m.point.add()
        p.x, p.y, p.z = x, y, z

    def _send(self, markers):
        req = Marker_V()
        req.marker.extend(markers)
        ok, _ = self.gz.request('/marker_array', req, Marker_V, Boolean, 3000)
        return ok

    # ------------------------------------------------------------------ drawing
    def build_map_markers(self):
        with self.lock:
            occ = self.global_map
            locals_ = {ns: m for ns, m in self.local_maps.items()}
        if occ is None:
            return []
        h, w, res = occ.info.height, occ.info.width, occ.info.resolution
        ox, oy = occ.info.origin.position.x, occ.info.origin.position.y
        grid = np.asarray(occ.data, dtype=np.int16).reshape(h, w)
        xmin, xmax, ymin, ymax = self.get_parameter('arena_bounds').value
        xs = ox + (np.arange(w) + 0.5) * res
        ys = oy + (np.arange(h) + 0.5) * res
        inside = ((ys[:, None] >= ymin - 0.3) & (ys[:, None] <= ymax + 0.3)
                  & (xs[None, :] >= xmin - 0.3) & (xs[None, :] <= xmax + 0.3))
        grid = np.where(inside, grid, -1)
        free = grid == 0
        observers = np.zeros((h, w), dtype=np.int8)
        owner = np.full((h, w), -1, dtype=np.int8)
        for k, ns in enumerate(self.drones):
            local = locals_.get(ns)
            if local is None or local.size != h * w:
                continue
            seen = local.reshape(h, w) >= 0
            observers += seen
            owner[seen & (owner < 0)] = k

        layers = [('map_obstacles', grid >= 65, OBSTACLE_COLOR, 0.9, 0.04),
                  ('map_overlap', free & (observers >= 2), OVERLAP_COLOR, 0.55, 0.025)]
        for k, ns in enumerate(self.drones):
            layers.append((f'map_{ns}', free & (observers == 1) & (owner == k),
                           DRONE_COLORS.get(ns, (0.6, 0.6, 0.6)), 0.5, 0.02))

        markers = []
        for idx, (name, mask, color, alpha, z) in enumerate(layers):
            m = self._marker('swarm_map', idx, Marker.TRIANGLE_LIST, color, alpha)
            for r0, r1, c0, c1 in merge_rectangles(mask):
                x0, x1 = ox + c0 * res, ox + c1 * res
                y0, y1 = oy + r0 * res, oy + r1 * res
                for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y0), (x1, y1), (x0, y1)):
                    self._add_point(m, x, y, z)
            if len(m.point) == 0:
                m.action = Marker.DELETE_MARKER
            markers.append(m)
        return markers

    def build_dynamic_markers(self):
        with self.lock:
            frontiers = list(self.frontiers)
            locked = dict(self.locked)
            plans = {ns: list(p) for ns, p in self.plans.items()}
            trails = {ns: list(t) for ns, t in self.trails.items()}
            poses = dict(self.poses)
            coverage, overlap = self.coverage, self.overlap
        markers = []

        # Namespaces whose element count changes are cleared and redrawn in the same request, so no
        # stale frontier or zone can survive (DELETE_ALL with a namespace only clears that namespace).
        for ns_name in ('frontiers', 'exclusion_zones'):
            clear = Marker()
            clear.ns = ns_name
            clear.action = Marker.DELETE_ALL
            markers.append(clear)

        for i, (x, y, z, size) in enumerate(frontiers):
            m = self._marker('frontiers', i, Marker.SPHERE, FRONTIER_COLOR, 1.0)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.scale.x = m.scale.y = m.scale.z = 0.35
            markers.append(m)

        # Exclusion zones as outline rings at floor level and at flight altitude (Gazebo markers
        # ignore alpha on solids, so a filled disc/cylinder would hide the map underneath).
        for i, (key, (x, y, z, radius, color)) in enumerate(sorted(locked.items())):
            for j, ring_z in enumerate((0.08, z)):
                ring = self._marker('exclusion_zones', 2 * i + j, Marker.LINE_STRIP, color, 1.0)
                for k in range(49):
                    a = 2.0 * math.pi * k / 48
                    self._add_point(ring, x + radius * math.cos(a), y + radius * math.sin(a), ring_z)
                markers.append(ring)

        for k, ns in enumerate(self.drones):
            color = DRONE_COLORS.get(ns, (0.8, 0.8, 0.8))
            for name, pts in (('plans', plans.get(ns, [])), ('trails', trails.get(ns, []))):
                m = self._marker(name, k, Marker.LINE_STRIP, color, 1.0)
                if len(pts) >= 2:
                    for x, y, z in pts:
                        self._add_point(m, x, y, z)
                else:
                    m.action = Marker.DELETE_MARKER
                markers.append(m)

        # Mission gauges above the north wall (Ogre2 cannot draw text markers): grey track,
        # green coverage bar and orange overlap bar, each 20 m = 100 %.
        gx, gy, gz = self.hud_position
        length = 20.0
        for idx, (value, color, dz) in enumerate(((coverage, (0.15, 0.85, 0.3), 0.6), (overlap, (1.0, 0.5, 0.0), 0.0))):
            track = self._marker('gauges', 2 * idx, Marker.BOX, (0.25, 0.25, 0.28), 1.0)
            track.pose.position.x, track.pose.position.y, track.pose.position.z = gx, gy, gz + dz
            track.scale.x, track.scale.y, track.scale.z = length, 0.3, 0.35
            bar = self._marker('gauges', 2 * idx + 1, Marker.BOX, color, 1.0)
            frac = max(0.0, min(1.0, (value or 0.0) / 100.0))
            if frac <= 0.0:
                bar.action = Marker.DELETE_MARKER
            bar.pose.position.x = gx - length / 2.0 + frac * length / 2.0
            bar.pose.position.y, bar.pose.position.z = gy - 0.05, gz + dz
            bar.scale.x, bar.scale.y, bar.scale.z = max(frac * length, 0.01), 0.35, 0.4
            markers += [track, bar]
        return markers

    def build_beacon_markers(self):
        """Drone beacons attached to each drone's Gazebo visual (marker `parent`), so they move with
        the drone at the GUI frame rate and never need re-sending: a halo ring, a heading line and
        an altitude stem in the drone's identity colour."""
        markers = []
        for k, ns in enumerate(self.drones):
            color = DRONE_COLORS.get(ns, (0.8, 0.8, 0.8))
            halo = self._marker('drone_beacons', 3 * k, Marker.LINE_STRIP, color, 1.0)
            for j in range(33):
                a = 2.0 * math.pi * j / 32
                self._add_point(halo, 0.6 * math.cos(a), 0.6 * math.sin(a), 0.0)
            heading = self._marker('drone_beacons', 3 * k + 1, Marker.LINE_LIST, color, 1.0)
            self._add_point(heading, 0.0, 0.0, 0.0)
            self._add_point(heading, 1.2, 0.0, 0.0)
            stem = self._marker('drone_beacons', 3 * k + 2, Marker.LINE_LIST, color, 1.0)
            self._add_point(stem, 0.0, 0.0, -0.1)
            self._add_point(stem, 0.0, 0.0, -1.4)
            for m in (halo, heading, stem):
                m.parent = ns
            markers += [halo, heading, stem]
        return markers

    def draw_loop(self):
        # One marker request per period: the Gazebo GUI duplicates a marker when two updates for the
        # same id arrive within one rendered frame, and under WSL the GUI may render only a few FPS.
        period = 1.0 / self.get_parameter('dynamic_rate').value
        map_every = max(1, int(round(self.get_parameter('dynamic_rate').value / self.get_parameter('map_rate').value)))
        cycle = 0
        connected = False
        while self.running and rclpy.ok():
            t0 = time.monotonic()
            try:
                markers = self.build_dynamic_markers()
                if not connected or cycle % 10 == 0:   # re-send in case a drone spawned after us
                    markers += self.build_beacon_markers()
                if connected and cycle % map_every == 0:
                    markers += self.build_map_markers()
                ok = self._send(markers)
                if ok and not connected:
                    connected = True
                    self.get_logger().info('Connected to the Gazebo GUI: drawing swarm overlay')
                elif not ok:
                    if connected:
                        self.get_logger().warn('Gazebo GUI marker service not responding')
                    connected = False
                    time.sleep(2.0)
                    continue
                cycle += 1
            except Exception as exc:  # keep the overlay alive through transient transport errors
                self.get_logger().error(f'overlay update failed: {exc}', throttle_duration_sec=10.0)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GzOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
