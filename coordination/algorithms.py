#!/usr/bin/env python3
"""
Swarm algorithms shared by the coordination nodes (coordination/algorithms.py)

Pure numpy, no rclpy, so every formula here is unit-tested directly by
coordination/test_swarm_coordination.py and reused unchanged by the ROS 2 nodes:

- fuse_occupancy:            Section 4.1 conservative multi-robot map fusion
- coverage_pct / overlap_rate: swarm exploration metrics
- extract_frontier_clusters: 8-connected frontier extraction + clustering
- greedy_assign:             Algorithm 1 greedy no-overlap frontier assignment
- inflate_obstacles / plan_path: A* path planning on the fused map
- OccupancyLogOdds:          depth-ray occupancy integration for the local maps

Grid conventions follow nav_msgs/OccupancyGrid: row-major (row = y, col = x),
-1 unknown, 0 free, 100 occupied.
"""

import heapq
import math
from collections import deque

import numpy as np

UNKNOWN = -1
FREE = 0
OCCUPIED = 100
OCCUPIED_THRESHOLD = 65


# --------------------------------------------------------------------------
# Grid geometry
# --------------------------------------------------------------------------
class GridGeometry:
    """Maps between world coordinates (m) and grid indices (row, col)."""

    def __init__(self, width, height, resolution, origin_x, origin_y):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)

    @classmethod
    def from_info(cls, info):
        return cls(info.width, info.height, info.resolution,
                   info.origin.position.x, info.origin.position.y)

    def world_to_cell(self, x, y):
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return row, col

    def cell_to_world(self, row, col):
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def in_bounds(self, row, col):
        return 0 <= row < self.height and 0 <= col < self.width

    def same_as(self, other, tol=1e-6):
        return (self.width == other.width and self.height == other.height
                and abs(self.resolution - other.resolution) < tol
                and abs(self.origin_x - other.origin_x) < tol
                and abs(self.origin_y - other.origin_y) < tol)


# --------------------------------------------------------------------------
# Section 4.1: conservative map fusion and swarm metrics
# --------------------------------------------------------------------------
def fuse_occupancy(arrays, occupied_threshold=OCCUPIED_THRESHOLD):
    """Fuse K local grids (flat or 2D int arrays with identical shape).

    M(c) = 100 if any drone reports >= occupied_threshold
           0   if every drone that observed c reports exactly 0 (and at least one did)
           -1  otherwise
    """
    stack = np.stack([np.asarray(a, dtype=np.int16) for a in arrays], axis=0)
    observed = stack >= 0
    occupied = (stack >= occupied_threshold).any(axis=0)
    all_observers_free = np.where(observed, stack == 0, True).all(axis=0)
    merged = np.full(stack.shape[1:], UNKNOWN, dtype=np.int8)
    merged[observed.any(axis=0) & all_observers_free] = FREE
    merged[occupied] = OCCUPIED
    return merged


def coverage_pct(merged, mask=None):
    """Percentage of cells (optionally inside a boolean region mask) that are known."""
    merged = np.asarray(merged)
    if mask is None:
        mask = np.ones(merged.shape, dtype=bool)
    total = int(np.count_nonzero(mask))
    if total == 0:
        return 0.0
    return 100.0 * np.count_nonzero((merged != UNKNOWN) & mask) / total


def overlap_rate(arrays):
    """Percentage of explored cells that were observed by two or more drones."""
    observed = np.stack([np.asarray(a) >= 0 for a in arrays], axis=0)
    counts = observed.sum(axis=0)
    union = np.count_nonzero(counts >= 1)
    if union == 0:
        return 0.0
    return 100.0 * np.count_nonzero(counts >= 2) / union


def region_mask(geom, bounds):
    """Boolean (height, width) mask of the cells whose centres lie inside [xmin, xmax, ymin, ymax]."""
    xmin, xmax, ymin, ymax = bounds
    xs = geom.origin_x + (np.arange(geom.width) + 0.5) * geom.resolution
    ys = geom.origin_y + (np.arange(geom.height) + 0.5) * geom.resolution
    return ((ys[:, None] >= ymin) & (ys[:, None] <= ymax)
            & (xs[None, :] >= xmin) & (xs[None, :] <= xmax))


# --------------------------------------------------------------------------
# Frontier extraction (Algorithm 1, step 1-2)
# --------------------------------------------------------------------------
_N8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def frontier_mask(grid):
    """Free cells with at least one unknown 8-neighbour. Cells outside the grid are not unknown."""
    grid = np.asarray(grid)
    padded = np.pad(grid, 1, mode='constant', constant_values=OCCUPIED)
    h, w = grid.shape
    unknown_neighbour = np.zeros((h, w), dtype=bool)
    for dr, dc in _N8:
        unknown_neighbour |= padded[1 + dr:1 + dr + h, 1 + dc:1 + dc + w] == UNKNOWN
    return (grid == FREE) & unknown_neighbour


def extract_frontier_clusters(grid, geom, min_cluster_size=3, valid_mask=None):
    """Group frontier cells into 8-connected clusters.

    valid_mask (optional, same shape as grid): only frontier cells where it is True are used, e.g.
    to drop frontiers hugging obstacles or lying outside the known operating area.

    Returns a list of dicts:
      centroid: (x, y) mean of the cluster cells (m)
      target:   (x, y) the cluster cell closest to the centroid, so the goal is always a known free cell
      size:     number of frontier cells
      gain:     information gain proxy = frontier length in metres
    """
    mask = frontier_mask(grid)
    if valid_mask is not None:
        mask &= valid_mask
    visited = np.zeros_like(mask)
    clusters = []
    rows, cols = np.nonzero(mask)
    for r0, c0 in zip(rows, cols):
        if visited[r0, c0]:
            continue
        visited[r0, c0] = True
        queue = deque([(r0, c0)])
        cells = []
        while queue:
            r, c = queue.popleft()
            cells.append((r, c))
            for dr, dc in _N8:
                nr, nc = r + dr, c + dc
                if 0 <= nr < geom.height and 0 <= nc < geom.width and mask[nr, nc] and not visited[nr, nc]:
                    visited[nr, nc] = True
                    queue.append((nr, nc))
        if len(cells) < min_cluster_size:
            continue
        cells = np.array(cells, dtype=float)
        mean_r, mean_c = cells.mean(axis=0)
        nearest = cells[np.argmin((cells[:, 0] - mean_r) ** 2 + (cells[:, 1] - mean_c) ** 2)]
        clusters.append({
            'centroid': geom.cell_to_world(mean_r, mean_c),
            'target': geom.cell_to_world(int(nearest[0]), int(nearest[1])),
            'size': len(cells),
            'gain': len(cells) * geom.resolution,
        })
    return clusters


# --------------------------------------------------------------------------
# Algorithm 1: greedy no-overlap frontier assignment
# --------------------------------------------------------------------------
def frontier_utility(gain, drone_xy, target_xy, alpha=1.0, beta=1.0):
    """U(f) = alpha * InformationGain(f) - beta * ||p_i - f||."""
    return alpha * gain - beta * math.dist(drone_xy, target_xy)


def sensing_discount(target_xy, others_xy, sensor_radius):
    """Fraction of a frontier's information gain still expected after accounting for teammates.

    Coordinated multi-robot exploration (Burgard et al., IEEE T-RO 2005): space around a point that
    another drone occupies or is flying to will be sensed by that drone anyway, so a frontier's
    gain is scaled by min(1, d / sensor_radius) for every teammate position / target at distance d.
    """
    factor = 1.0
    for o in others_xy:
        factor *= min(1.0, math.dist(target_xy, o) / sensor_radius)
    return factor


def greedy_assign(drone_positions, needing_task, frontiers, locked_centres,
                  lock_radius=2.0, alpha=1.0, beta=1.0, blacklist=(), blacklist_radius=1.0,
                  sensor_radius=0.0):
    """Assign frontier targets to the drones in `needing_task`.

    drone_positions: {ns: (x, y)} for every active drone
    needing_task:    drones that need a new target
    frontiers:       clusters from extract_frontier_clusters
    locked_centres:  {ns: (x, y)} lock centres held by drones keeping their current target
    blacklist:       (x, y) points that were found unreachable

    A frontier f is rejected for drone i when it lies within lock_radius of another drone's
    position or of another drone's locked region. The globally best (drone, frontier) utility
    pair is assigned first; its lock then excludes nearby frontiers for everyone else.
    With sensor_radius > 0 the gain is additionally discounted near teammates' positions and
    targets (sensing_discount), which spreads the drones out and reduces sensor overlap.

    Returns {ns: (x, y)} assignments.
    """
    locks = {ns: c for ns, c in locked_centres.items() if c is not None}
    remaining_drones = [ns for ns in needing_task if ns in drone_positions]
    candidates = [f for f in frontiers
                  if all(math.dist(f['target'], b) >= blacklist_radius for b in blacklist)]
    assignments = {}

    while remaining_drones and candidates:
        best = None
        for ns in remaining_drones:
            pos = drone_positions[ns]
            for idx, f in enumerate(candidates):
                t = f['target']
                if any(other != ns and math.dist(t, p) < lock_radius
                       for other, p in drone_positions.items()):
                    continue
                if any(other != ns and math.dist(t, c) < lock_radius
                       for other, c in locks.items()):
                    continue
                gain = f['gain']
                if sensor_radius > 0:
                    others = [p for o, p in drone_positions.items() if o != ns]
                    others += [c for o, c in locks.items() if o != ns]
                    gain *= sensing_discount(t, others, sensor_radius)
                u = frontier_utility(gain, pos, t, alpha, beta)
                if best is None or u > best[0]:
                    best = (u, ns, idx)
        if best is None:
            break
        _, ns, idx = best
        target = candidates[idx]['target']
        assignments[ns] = target
        locks[ns] = target
        remaining_drones.remove(ns)
        candidates = [f for f in candidates if math.dist(f['target'], target) >= lock_radius]
    return assignments


# --------------------------------------------------------------------------
# A* path planning on the fused occupancy grid
# --------------------------------------------------------------------------
def _disk_offsets(radius_cells):
    r = int(math.ceil(radius_cells))
    return [(dr, dc) for dr in range(-r, r + 1) for dc in range(-r, r + 1)
            if dr * dr + dc * dc <= radius_cells * radius_cells]


def inflate_obstacles(grid, radius_cells):
    """Boolean mask of cells within radius_cells of an occupied (>= threshold) cell."""
    occ = np.asarray(grid) >= OCCUPIED_THRESHOLD
    if radius_cells <= 0:
        return occ.copy()
    h, w = occ.shape
    r = int(math.ceil(radius_cells))
    padded = np.pad(occ, r, mode='constant', constant_values=False)
    out = np.zeros_like(occ)
    for dr, dc in _disk_offsets(radius_cells):
        out |= padded[r + dr:r + dr + h, r + dc:r + dc + w]
    return out


def _nearest_unblocked(blocked, start, max_radius):
    """BFS from start to the closest unblocked cell within max_radius cells (or None)."""
    h, w = blocked.shape
    if not blocked[start]:
        return start
    seen = {start}
    queue = deque([start])
    while queue:
        r, c = queue.popleft()
        for dr, dc in _N8:
            n = (r + dr, c + dc)
            if n in seen or not (0 <= n[0] < h and 0 <= n[1] < w):
                continue
            if abs(n[0] - start[0]) > max_radius or abs(n[1] - start[1]) > max_radius:
                continue
            if not blocked[n]:
                return n
            seen.add(n)
            queue.append(n)
    return None


def _line_clear(blocked, a, b):
    """True if the straight segment between cells a and b crosses no blocked cell."""
    (r0, c0), (r1, c1) = a, b
    n = max(abs(r1 - r0), abs(c1 - c0))
    if n == 0:
        return not blocked[a]
    # Sample the continuous segment between cell centres every quarter cell so that
    # diagonal shortcuts cannot clip the corner of a blocked cell.
    ts = np.linspace(0.0, 1.0, 4 * n + 1)
    rs = np.floor(r0 + 0.5 + ts * (r1 - r0)).astype(int)
    cs = np.floor(c0 + 0.5 + ts * (c1 - c0)).astype(int)
    return not blocked[rs, cs].any()


def plan_path(grid, geom, start_xy, goal_xy, inflation_radius=0.5, unknown_cost=2.0,
              goal_snap_radius=1.5):
    """A* over the 8-connected grid. Occupied cells are inflated by inflation_radius (m) and
    are impassable; unknown cells are allowed (exploration) at a higher traversal cost.

    Returns a list of (x, y) waypoints (line-of-sight simplified, start excluded, goal last)
    or None if no path exists.
    """
    grid = np.asarray(grid)
    blocked = inflate_obstacles(grid, inflation_radius / geom.resolution)
    start = geom.world_to_cell(*start_xy)
    goal = geom.world_to_cell(*goal_xy)
    if not (geom.in_bounds(*start) and geom.in_bounds(*goal)):
        return None
    # The drone may legitimately sit inside an inflated band (e.g. right after take-off near a
    # wall): plan from the nearest free cell instead of failing.
    start = _nearest_unblocked(blocked, start, int(1.0 / geom.resolution)) or start
    goal = _nearest_unblocked(blocked, goal, int(goal_snap_radius / geom.resolution))
    if goal is None:
        return None

    cell_cost = np.where(grid == UNKNOWN, unknown_cost, 1.0)
    h, w = grid.shape
    g = {start: 0.0}
    parent = {start: None}
    open_heap = [(0.0, start)]
    closed = set()
    while open_heap:
        _, cur = heapq.heappop(open_heap)
        if cur == goal:
            break
        if cur in closed:
            continue
        closed.add(cur)
        for dr, dc in _N8:
            n = (cur[0] + dr, cur[1] + dc)
            if not (0 <= n[0] < h and 0 <= n[1] < w) or blocked[n] or n in closed:
                continue
            step = (1.41421356 if dr and dc else 1.0) * cell_cost[n]
            ng = g[cur] + step
            if ng < g.get(n, float('inf')):
                g[n] = ng
                parent[n] = cur
                heapq.heappush(open_heap, (ng + math.hypot(goal[0] - n[0], goal[1] - n[1]), n))
    if goal not in parent:
        return None

    cells = []
    node = goal
    while node is not None:
        cells.append(node)
        node = parent[node]
    cells.reverse()

    # Line-of-sight shortcutting
    simplified = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not _line_clear(blocked, cells[i], cells[j]):
            j -= 1
        simplified.append(cells[j])
        i = j
    return [geom.cell_to_world(r, c) for r, c in simplified[1:]] or [geom.cell_to_world(*goal)]


# --------------------------------------------------------------------------
# Local occupancy mapping from depth rays
# --------------------------------------------------------------------------
class OccupancyLogOdds:
    """Log-odds occupancy grid updated with 2D rays (free space along the ray, hit at the end)."""

    def __init__(self, geom, l_free=-0.4, l_occ=0.85, l_min=-2.0, l_max=3.5,
                 free_threshold=-0.3, occ_threshold=0.5):
        self.geom = geom
        self.l_free, self.l_occ = l_free, l_occ
        self.l_min, self.l_max = l_min, l_max
        self.free_threshold, self.occ_threshold = free_threshold, occ_threshold
        self.logodds = np.zeros((geom.height, geom.width), dtype=np.float32)
        self.observed = np.zeros((geom.height, geom.width), dtype=bool)

    def _cells(self, xs, ys):
        cols = np.floor((xs - self.geom.origin_x) / self.geom.resolution).astype(np.int64)
        rows = np.floor((ys - self.geom.origin_y) / self.geom.resolution).astype(np.int64)
        ok = (rows >= 0) & (rows < self.geom.height) & (cols >= 0) & (cols < self.geom.width)
        return rows[ok] * self.geom.width + cols[ok]

    def integrate_rays(self, origin_xy, azimuths, free_lengths, hit_xy):
        """origin_xy: sensor position; azimuths/free_lengths: per-ray direction and free distance;
        hit_xy: (M, 2) obstacle endpoints (may be empty)."""
        step = self.geom.resolution * 0.5
        free_idx = np.empty(0, dtype=np.int64)
        if len(azimuths):
            n_steps = int(np.ceil(np.max(free_lengths) / step)) if np.max(free_lengths) > 0 else 0
            if n_steps > 0:
                ts = (np.arange(n_steps) + 0.5) * step                       # (S,)
                valid = ts[None, :] < (np.asarray(free_lengths)[:, None] - step)  # (R, S)
                xs = origin_xy[0] + np.cos(azimuths)[:, None] * ts[None, :]
                ys = origin_xy[1] + np.sin(azimuths)[:, None] * ts[None, :]
                free_idx = np.unique(self._cells(xs[valid], ys[valid]))
        hit_idx = np.empty(0, dtype=np.int64)
        if len(hit_xy):
            hit_xy = np.asarray(hit_xy)
            hit_idx = np.unique(self._cells(hit_xy[:, 0], hit_xy[:, 1]))
            free_idx = np.setdiff1d(free_idx, hit_idx, assume_unique=True)

        flat = self.logodds.reshape(-1)
        obs = self.observed.reshape(-1)
        flat[free_idx] = np.clip(flat[free_idx] + self.l_free, self.l_min, self.l_max)
        flat[hit_idx] = np.clip(flat[hit_idx] + self.l_occ, self.l_min, self.l_max)
        obs[free_idx] = True
        obs[hit_idx] = True

    def to_occupancy(self):
        out = np.full(self.logodds.shape, UNKNOWN, dtype=np.int8)
        out[self.observed & (self.logodds <= self.free_threshold)] = FREE
        out[self.observed & (self.logodds >= self.occ_threshold)] = OCCUPIED
        return out


def quaternion_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def depth_image_to_rays(depth, hfov, cam_offset_body, rotation, position,
                        stride=4, max_range=8.0, far_clip=20.0,
                        obstacle_z_min=0.3, obstacle_z_max=2.7, min_range=0.0):
    """Convert a planar float32 depth image from a forward-looking camera (Gazebo convention:
    camera looks along body +x) into per-column 2D rays.

    Returns (origin_xy, azimuths, free_lengths, hit_xy).
    Returns closer than min_range (the drone's own propellers in view) are discarded entirely.
    A column's hit is the nearest point inside the obstacle height band; free space extends up
    to that hit, or to max_range when the column sees nothing (or only floor) closer.
    """
    h, w = depth.shape
    fx = (w / 2.0) / math.tan(hfov / 2.0)
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    us = np.arange(stride // 2, w, stride)
    vs = np.arange(stride // 2, h, stride)
    d = depth[np.ix_(vs, us)].astype(np.float64)                    # (V, U)

    # Camera-frame rays (x forward, y left, z up), planar depth scales the x component
    ray_y = -(us - cx) / fx
    ray_z = -(vs - cy) / fx
    no_return = ~np.isfinite(d) | (d >= far_clip * 0.99)
    d_valid = np.where(no_return | (d <= 0) | (d < min_range), np.nan, d)

    pts_body = np.stack([
        d_valid,
        d_valid * ray_y[None, :],
        d_valid * ray_z[:, None],
    ], axis=-1) + np.asarray(cam_offset_body)                       # (V, U, 3)
    pts_world = pts_body @ np.asarray(rotation).T + np.asarray(position)

    sensor_world = np.asarray(rotation) @ np.asarray(cam_offset_body) + np.asarray(position)
    origin_xy = sensor_world[:2]
    horiz = np.hypot(pts_world[..., 0] - origin_xy[0], pts_world[..., 1] - origin_xy[1])

    is_obstacle = (pts_world[..., 2] >= obstacle_z_min) & (pts_world[..., 2] <= obstacle_z_max)
    hit_range = np.where(is_obstacle, horiz, np.inf)
    hit_row = np.argmin(np.nan_to_num(hit_range, nan=np.inf), axis=0)
    col_idx = np.arange(len(us))
    nearest_hit = hit_range[hit_row, col_idx]
    nearest_hit = np.nan_to_num(nearest_hit, nan=np.inf)

    # Column azimuth from the camera ray direction projected on the ground plane
    dirs_body = np.stack([np.ones_like(ray_y), ray_y, np.zeros_like(ray_y)], axis=-1)
    dirs_world = dirs_body @ np.asarray(rotation).T
    azimuths = np.arctan2(dirs_world[:, 1], dirs_world[:, 0])

    has_hit = nearest_hit <= max_range
    free_lengths = np.where(has_hit, nearest_hit, max_range)
    # Columns with neither an obstacle in range nor any open view only reveal floor: limit the
    # free space to the farthest floor point seen.
    floor_range = np.where(pts_world[..., 2] < obstacle_z_min, horiz, np.nan)
    floor_max = np.nan_to_num(np.nanmax(np.where(np.isnan(floor_range), -np.inf, floor_range), axis=0),
                              neginf=0.0)
    sees_open = no_return.any(axis=0) | np.isfinite(nearest_hit)
    free_lengths = np.where(has_hit | sees_open, free_lengths, np.minimum(floor_max, max_range))

    hit_xy = np.stack([pts_world[hit_row, col_idx, 0], pts_world[hit_row, col_idx, 1]], axis=-1)[has_hit]
    return origin_xy, azimuths, free_lengths, hit_xy
