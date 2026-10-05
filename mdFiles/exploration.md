# Exploration

This is one of the most important documents in the project — it fully
specifies the frontier exploration algorithm that drives the drone's
autonomous behavior.

## 1. Frontier Definition

> A frontier is a boundary between known free space and unknown space.

Concretely: a **frontier cell** is a FREE cell in the occupancy grid that
has at least one directly adjacent UNKNOWN cell. See
[`mapping.md`](mapping.md) for the FREE/OCCUPIED/UNKNOWN cell model this
builds on.

## 2. Frontier Detection

**Input:** Occupancy grid.
**Output:** Set of frontier cells.

```text
frontier_cells = {}
for cell in occupancy_grid:
    if cell.state == FREE:
        if any(neighbor.state == UNKNOWN for neighbor in adjacent(cell)):
            frontier_cells.add(cell)
```

`adjacent(cell)` should use a consistent connectivity definition (4- or
8-connected) with the clustering step below — using different connectivity
for detection vs. clustering is a common source of bugs.

## 3. Frontier Clustering

**Input:** Set of frontier cells.
**Output:** List of clusters, each a connected group of frontier cells.

```text
clusters = connected_components(frontier_cells, connectivity=8)
clusters = [c for c in clusters if size(c) >= min_cluster_size]
```

`min_cluster_size` filters out noise-sized frontier fragments (e.g., single
isolated cells caused by sensor noise at map edges) and is a configurable
parameter, not a hard-coded constant.

## 4. Frontier Scoring

**Input:** List of clusters, current drone pose.
**Output:** Each cluster annotated with a score.

Initial scoring model (must remain swappable — see
[`system-design.md`](system-design.md) NFR-3):

```text
score(cluster) = information_gain(cluster) - distance_cost(cluster, drone_pose)
```

### 4.1 Information Gain

An estimate of how much new (currently UNKNOWN) area would likely become
observable by exploring this cluster. A simple, well-understood starting
point is cluster size (number of frontier cells, optionally weighted by the
amount of adjacent UNKNOWN space), since larger frontiers generally border
larger unexplored regions.

```text
information_gain(cluster) = weight_gain * estimated_unknown_area_near(cluster)
```

### 4.2 Distance Cost

The cost of traveling from the drone's current position to the cluster,
discouraging selection of far-away frontiers when a closer one offers
comparable gain.

```text
distance_cost(cluster, drone_pose) = weight_dist * distance(drone_pose, representative_point(cluster))
```

`distance` should initially be straight-line (Euclidean) distance in the
map frame for simplicity; a path-length-aware cost (e.g., from a planned
path) is a reasonable future improvement but is not required for Phase 1.

### 4.3 Safety Cost

Not part of the initial scoring formula, but should be documented as a
planned extension point: a term penalizing clusters whose representative
point/approach path has low obstacle clearance. This can be added as an
additional subtracted term once the basic scoring model (§4) is validated,
without changing the interface of the scoring function (it still returns
one scalar score per cluster).

```text
score(cluster) = information_gain(cluster) - distance_cost(cluster, drone_pose) [- safety_cost(cluster)]
```

## 5. Frontier Selection

**Input:** Scored clusters.
**Output:** A single selected cluster (or `None` if no valid cluster
exists).

```text
valid_clusters = [c for c in clusters if is_valid(c)]
if not valid_clusters:
    return None
target = max(valid_clusters, key=lambda c: c.score)
return target
```

`is_valid(c)` should filter out clusters that cannot plausibly yield a safe
waypoint (e.g., entirely surrounded by OCCUPIED cells) before scoring
effort is spent on them, and/or after waypoint generation fails (see §6 and
[`algorithms.md`](algorithms.md) for the retry relationship between
selection and waypoint generation).

## 6. Waypoint Generation

A frontier cluster is **not** automatically a valid waypoint — it is a
region of the map, not a flight target. Converting it into one requires
explicit validation.

```text
Selected Frontier
      ↓
Representative Target        (e.g., centroid, or nearest frontier cell to the drone)
      ↓
Check map validity           (is the representative point itself FREE, not OCCUPIED/UNKNOWN?)
      ↓
Check obstacle clearance     (is there sufficient clearance from nearby OCCUPIED cells?)
      ↓
Generate waypoint            (position expressed in the documented map frame)
      ↓
Send to PX4
```

Pseudocode:

```text
target_point = representative_point(selected_cluster)
if not is_free(target_point):
    target_point = nearest_free_point(target_point)
if clearance(target_point) < min_clearance:
    reject()   # frontier selector must supply the next-best candidate
waypoint = Waypoint(position=target_point, frame=MAP_FRAME)
send_to_px4(waypoint)
```

If validation fails, the Exploration Manager should return to frontier
selection (§5) with this cluster excluded, rather than treating it as a
hard failure of the whole loop — see [`algorithms.md`](algorithms.md).

The waypoint's coordinate frame must match the map frame documented in
[`system-design.md`](system-design.md) §6 and
[`localization.md`](localization.md) §7; any transform into a PX4-expected
frame happens in the PX4 Interface Node (see
[`ros2-architecture.md`](ros2-architecture.md) §2.8), not inside waypoint
generation itself.

## 7. Exploration Completion

"Exploration complete" must be explicitly defined and configurable, using
one or more of the following criteria (any can trigger completion; which
ones are active is a configuration choice, not a fixed algorithm decision):

* **No valid frontiers remain** — frontier detection + clustering + `is_valid`
  filtering yields an empty set.
* **Coverage threshold reached** — the fraction of the map that is no
  longer UNKNOWN exceeds a configured threshold.
* **Maximum flight time reached** — a configured wall-clock or
  simulated-time budget has elapsed.
* **Maximum distance reached** — the drone's cumulative traveled distance
  exceeds a configured budget.
* **Safety condition triggered** — e.g., battery/energy proxy exhausted,
  repeated navigation failures, or any other configured safety stop.

```text
def exploration_complete(state, config):
    if config.stop_on_no_frontiers and not state.valid_frontiers:
        return True
    if state.coverage >= config.coverage_threshold:
        return True
    if state.elapsed_time >= config.max_time:
        return True
    if state.distance_traveled >= config.max_distance:
        return True
    if state.safety_triggered:
        return True
    return False
```

## 8. Exploration Loop

```text
while not exploration_complete(state, config):

    update_map()

    frontiers = detect_frontiers(map)

    clusters = cluster_frontiers(frontiers)

    candidates = score_frontiers(clusters)

    target = select_best_frontier(candidates)
    if target is None:
        break  # no valid frontiers — see completion criteria above

    waypoint = generate_waypoint(target)
    if waypoint is None:
        candidates.remove(target)
        continue  # retry with next-best candidate

    navigate_to(waypoint)

    update_map()
```

This loop is owned by the Exploration Manager (see
[`ros2-architecture.md`](ros2-architecture.md) §2.7) and is implemented at
M10, after each stage (M4–M9) is individually validated.

## 9. Modularity Requirement

Per [`system-design.md`](system-design.md) NFR-3, each stage above must be
independently replaceable:

* The clustering method (connected-component labeling initially) could be
  swapped for a different clustering approach.
* The scoring function (§4) could be replaced with a more sophisticated
  model without changing the selection/waypoint-generation interfaces.
* The pose source feeding this entire pipeline can change (ground truth →
  VSLAM/VIO, per [`localization.md`](localization.md)) without any change
  to this file's algorithm.
