#!/usr/bin/env python3
"""
Unit Tests for the Multi-Drone Swarm Coordination Layer.

Every test exercises the production code in coordination/algorithms.py (the same functions the
map_merger, task_allocator, flight_controller and depth_mapper nodes run):
- Conservative map fusion (Section 4.1) and swarm metrics
- 8-connected frontier extraction and clustering
- Algorithm 1: greedy no-overlap frontier assignment (Section 4.2)
- A* path planning around inflated obstacles
- Depth image -> occupancy rays

Run:  python3 -m pytest coordination/test_swarm_coordination.py   (or execute this file directly)
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coordination.algorithms import (  # noqa: E402
    FREE, OCCUPIED, UNKNOWN, GridGeometry, OccupancyLogOdds, coverage_pct, depth_image_to_rays,
    extract_frontier_clusters, frontier_utility, fuse_occupancy, greedy_assign, inflate_obstacles,
    overlap_rate, plan_path, quaternion_to_matrix, region_mask, sensing_discount)


def test_conservative_map_fusion():
    m1 = np.array([-1, 0, 100, 0, -1, 0, 70, 30], dtype=np.int8)
    m2 = np.array([-1, 0, 0, 100, -1, -1, 0, 0], dtype=np.int8)
    m3 = np.array([-1, -1, 0, 0, -1, -1, -1, -1], dtype=np.int8)
    merged = fuse_occupancy([m1, m2, m3])
    assert merged[0] == UNKNOWN        # nobody observed it
    assert merged[1] == FREE           # every observer says free
    assert merged[2] == OCCUPIED       # obstacle priority
    assert merged[3] == OCCUPIED
    assert merged[4] == UNKNOWN
    assert merged[5] == FREE           # a single observer suffices
    assert merged[6] == OCCUPIED       # >= 65 counts as occupied
    assert merged[7] == UNKNOWN        # an uncertain (30) observer prevents "free"


def test_coverage_and_overlap_metrics():
    a = np.array([0, 0, 0, -1, -1, -1, -1, -1, -1, -1])
    b = np.array([-1, -1, 0, 0, 0, -1, -1, -1, -1, -1])
    merged = fuse_occupancy([a, b])
    assert math.isclose(coverage_pct(merged), 50.0)
    assert math.isclose(overlap_rate([a, b]), 100.0 / 5.0)   # 1 shared cell of 5 explored
    mask = np.zeros(10, dtype=bool)
    mask[:4] = True
    assert math.isclose(coverage_pct(merged, mask), 100.0)


def test_region_mask_matches_bounds():
    geom = GridGeometry(10, 10, 1.0, -5.0, -5.0)
    mask = region_mask(geom, [-2.0, 2.0, -1.0, 1.0])
    assert mask.sum() == 4 * 2       # cell centres -1.5..1.5 in x, -0.5..0.5 in y


def test_frontier_extraction_uses_8_neighbourhood():
    geom = GridGeometry(7, 7, 1.0, 0.0, 0.0)
    grid = np.full((7, 7), UNKNOWN, dtype=np.int8)
    grid[1:6, 1:4] = FREE            # free block; its right edge borders unknown
    grid[1:6, 0] = OCCUPIED
    clusters = extract_frontier_clusters(grid, geom, min_cluster_size=3)
    assert len(clusters) == 1
    c = clusters[0]
    tx, ty = c['target']
    row, col = geom.world_to_cell(tx, ty)
    assert grid[row, col] == FREE     # the target is always a known free cell
    assert math.isclose(c['gain'], c['size'] * geom.resolution)


def test_algorithm_1_greedy_no_overlap():
    frontiers = [
        {'target': (5.0, 5.0), 'gain': 2.0},
        {'target': (5.5, 5.2), 'gain': 2.0},   # inside the 2 m lock of the first frontier
        {'target': (-5.0, 5.0), 'gain': 2.0},
        {'target': (0.0, 10.0), 'gain': 2.0},
    ]
    drones = {'drone_1': (-4.0, 0.0), 'drone_2': (0.0, 0.0), 'drone_3': (4.0, 0.0)}
    assignments = greedy_assign(drones, list(drones), frontiers, {}, lock_radius=2.0)
    assert len(assignments) == 3
    targets = list(assignments.values())
    for i in range(len(targets)):
        for j in range(i + 1, len(targets)):
            assert math.dist(targets[i], targets[j]) >= 2.0
    assert assignments['drone_1'] == (-5.0, 5.0)
    assert assignments['drone_3'] == (5.0, 5.0)


def test_algorithm_1_rejects_frontiers_near_other_drones_and_locks():
    frontiers = [{'target': (1.0, 0.0), 'gain': 10.0}, {'target': (8.0, 0.0), 'gain': 1.0}]
    drones = {'drone_1': (0.0, 0.0), 'drone_2': (1.5, 0.5)}
    # drone_2 is within 2 m of the high-gain frontier, so drone_1 must not take it
    a = greedy_assign(drones, ['drone_1'], frontiers, {}, lock_radius=2.0)
    assert a == {'drone_1': (8.0, 0.0)}
    # a locked region held by another drone excludes frontiers the same way
    a = greedy_assign({'drone_1': (0.0, 0.0), 'drone_2': (20.0, 0.0)}, ['drone_1'], frontiers,
                      {'drone_2': (1.2, 0.0)}, lock_radius=2.0)
    assert a == {'drone_1': (8.0, 0.0)}
    # blacklisted (unreachable) targets are skipped
    a = greedy_assign({'drone_1': (0.0, 0.0)}, ['drone_1'], frontiers, {}, blacklist=[(1.0, 0.0)])
    assert a == {'drone_1': (8.0, 0.0)}


def test_sensing_discount_spreads_the_swarm():
    # Two equal frontiers: one right next to where drone_2 is already heading, one far away.
    frontiers = [{'target': (3.0, 0.0), 'gain': 6.0}, {'target': (-3.0, 0.0), 'gain': 6.0}]
    drones = {'drone_1': (0.5, 0.0), 'drone_2': (8.0, 0.0)}
    locked = {'drone_2': (5.5, 0.0)}
    plain = greedy_assign(drones, ['drone_1'], frontiers, locked, lock_radius=2.0)
    assert plain == {'drone_1': (3.0, 0.0)}           # nearest wins without coordination
    spread = greedy_assign(drones, ['drone_1'], frontiers, locked, lock_radius=2.0, sensor_radius=5.0)
    assert spread == {'drone_1': (-3.0, 0.0)}         # away from the region drone_2 will sense
    assert sensing_discount((0.0, 0.0), [(10.0, 0.0)], 5.0) == 1.0
    assert math.isclose(sensing_discount((0.0, 0.0), [(2.5, 0.0)], 5.0), 0.5)


def test_utility_trades_gain_against_distance():
    near_small = frontier_utility(1.0, (0.0, 0.0), (1.0, 0.0))
    far_large = frontier_utility(10.0, (0.0, 0.0), (5.0, 0.0))
    assert far_large > near_small
    assert math.isclose(frontier_utility(3.0, (0, 0), (3, 4), alpha=2.0, beta=1.0), 1.0)


def test_astar_avoids_inflated_wall():
    geom = GridGeometry(40, 40, 0.2, 0.0, 0.0)
    grid = np.zeros((40, 40), dtype=np.int8)
    grid[0:30, 20] = OCCUPIED        # wall at x = 4.0 m with a gap at the top (y > 6 m)
    path = plan_path(grid, geom, (1.0, 1.0), (7.0, 1.0), inflation_radius=0.4)
    assert path is not None
    assert math.dist(path[-1], (7.0, 1.0)) < 0.3
    blocked = inflate_obstacles(grid, 0.4 / geom.resolution)
    pts = [(1.0, 1.0)] + path
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        for t in np.linspace(0.0, 1.0, 50):
            r, c = geom.world_to_cell(x0 + t * (x1 - x0), y0 + t * (y1 - y0))
            assert not blocked[r, c], 'path crosses an inflated obstacle'
    grid[:, 20] = OCCUPIED            # close the gap -> no path
    assert plan_path(grid, geom, (1.0, 1.0), (7.0, 1.0), inflation_radius=0.4) is None


def test_depth_rays_map_wall_ahead():
    # Synthetic depth camera 1.5 m above the floor looking along +x at a wall 4 m away
    w, h, hfov = 64, 48, math.radians(87.0)
    fx = (w / 2) / math.tan(hfov / 2)
    vs = np.arange(h)[:, None]
    us = np.arange(w)[None, :]
    ray_z = -(vs - (h - 1) / 2) / fx
    depth = np.full((h, w), 4.0, dtype=np.float32)
    floor_d = np.where(ray_z < 0, 1.5 / np.maximum(-ray_z, 1e-6), np.inf)
    depth = np.minimum(depth, floor_d).astype(np.float32) + 0 * us
    origin, az, free_len, hits = depth_image_to_rays(
        depth, hfov, [0.0, 0.0, 0.0], quaternion_to_matrix(0, 0, 0, 1), [0.0, 0.0, 1.5], stride=4)
    assert np.allclose(origin, [0.0, 0.0])
    assert len(hits) == len(az)                        # every column sees the wall
    assert np.allclose(hits[:, 0], 4.0, atol=1e-6)     # planar depth -> wall plane x = 4
    geom = GridGeometry(60, 60, 0.2, -6.0, -6.0)
    grid = OccupancyLogOdds(geom)
    grid.integrate_rays(origin, az, free_len, hits)
    occ = grid.to_occupancy()
    assert occ[geom.world_to_cell(4.05, 0.0)] == OCCUPIED
    assert occ[geom.world_to_cell(2.0, 0.0)] == FREE
    assert occ[geom.world_to_cell(-2.0, 0.0)] == UNKNOWN   # behind the camera


def test_depth_rays_ignore_own_propellers():
    # A propeller blade 0.15 m in front of the lens must not become an obstacle
    depth = np.full((48, 64), 4.0, dtype=np.float32)
    depth[20:28, 40:56] = 0.15
    rot = quaternion_to_matrix(0, 0, 0, 1)
    _, _, _, hits_raw = depth_image_to_rays(depth, math.radians(87.0), [0, 0, 0], rot, [0, 0, 1.5], stride=4)
    _, _, _, hits = depth_image_to_rays(depth, math.radians(87.0), [0, 0, 0], rot, [0, 0, 1.5], stride=4,
                                        min_range=0.35)
    assert (np.hypot(hits_raw[:, 0], hits_raw[:, 1]) < 0.5).any()
    assert (np.hypot(hits[:, 0], hits[:, 1]) > 3.9).all()


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_') and callable(v)]
    for t in tests:
        t()
        print(f'[PASS] {t.__name__}')
    print(f'\nALL {len(tests)} SWARM COORDINATION UNIT TESTS PASSED')
