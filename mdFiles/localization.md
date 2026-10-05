# Localization

## 1. Two Stages

### Development stage

```text
Gazebo Ground Truth
```

### Final stage (target autonomy input)

```text
Camera + IMU
      ↓
VSLAM / VIO
      ↓
Pose
```

Both stages publish pose on the same documented topic/frame contract, so
the mapping and exploration components never need to know which stage is
active (see [`ros2-architecture.md`](ros2-architecture.md) §2.1).

## 2. Why Localization Is Required

The drone needs to know where it is, relative to what it has already
mapped, in order to:

* Correctly place new sensor observations into the occupancy grid.
* Know its own position relative to a selected frontier for distance-cost
  scoring.
* Know when it has arrived at a commanded waypoint.

Without a pose estimate, none of the mapping or exploration pipeline can
function, regardless of how that pose estimate is produced.

## 3. Why GPS Is Not Used

This project's defining constraint is GPS-denied operation: the target
deployment scenarios (indoor, urban canyon, underground, disaster sites)
are exactly the situations where GPS is degraded or unavailable. Building
the exploration system around a GPS dependency would make it inapplicable
to the scenarios this project is meant to address. GPS is therefore
excluded from the autonomy pipeline entirely — it is not used as a fallback
or a sanity check within the autonomy stack. PX4 itself may still have GPS
support available internally as a general-purpose flight-controller
feature, but the exploration autonomy this project builds does not consume
GPS data.

## 4. Why Ground Truth First

This is the most important methodological decision in the project and is
repeated here deliberately (it also appears in
[`development-roadmap.md`](development-roadmap.md)).

Localization (state estimation from noisy sensors) and exploration (deciding
where to go given a map) are two largely independent problems. If both are
implemented and debugged simultaneously, a failure in the exploration loop
is hard to distinguish from a failure caused by pose drift or VSLAM
tracking loss.

By using Gazebo's ground-truth pose first (M4–M10), the mapping and
exploration algorithms can be fully validated against a perfect pose
source. Only once that pipeline is known to work (M10) is ground truth
replaced with VSLAM/VIO (M11) — at which point any new problems that
appear can be attributed to localization, not to the exploration logic.

```text
Gazebo Ground Truth Pose
        ↓
Mapping
        ↓
Frontier Detection
        ↓
Frontier Selection
        ↓
Waypoint
        ↓
PX4
```

is replaced, once validated, with:

```text
Camera + IMU
      ↓
VSLAM / VIO
      ↓
Estimated Pose
        ↓
Mapping
        ↓
Frontier Detection
        ↓
Frontier Selection
        ↓
Waypoint
        ↓
PX4
```

Ground-truth pose must never be used in the final Phase-1 demonstration
(M12) — it is a development and debugging aid only.

## 5. VSLAM/VIO Candidates

* **ORB-SLAM3** — full-featured, well-documented, supports monocular,
  stereo, and monocular-inertial/stereo-inertial modes; a reasonable
  default candidate given the "or another well-supported VIO/VSLAM
  solution" allowance in the technology stack.
* **Other well-supported alternatives** may be substituted if there is a
  strong technical reason (e.g., better ROS 2 integration available at
  implementation time, licensing considerations, or better performance in
  the chosen simulated environments). Any substitution must be documented
  here along with the reason, per the "never silently replace a major
  technology" rule in [`development-roadmap.md`](development-roadmap.md).

This project does not fix the exact VSLAM/VIO package at the documentation
stage — that decision is deferred to M11 implementation time, informed by
what integrates most cleanly with ROS 2 Humble and the chosen camera/IMU
setup.

## 6. Interface Between Localization and Mapping

* Localization publishes a single pose estimate (position + orientation)
  in the map frame, on a fixed, documented topic — identical in shape
  whether sourced from ground truth or VSLAM/VIO.
* Mapping consumes only this pose topic; it does not know or care which
  underlying method produced it.
* This contract must not change between M4 (ground truth) and M11
  (VSLAM/VIO) — if it needs to change, that is a signal the interface was
  under-specified and should be fixed rather than worked around.

## 7. Coordinate Frames

* **Map frame:** fixed, global frame the occupancy grid and all frontier/
  waypoint computations are expressed in.
* **Odometry frame:** the frame VSLAM/VIO naturally publishes in before any
  correction; may drift relative to the map frame over time.
* **Body frame:** drone-fixed frame.

Exact frame names and the specific transform tree should follow standard
ROS conventions (e.g., `REP-105`-style `map` → `odom` → `base_link`) and be
finalized in [`ros2-architecture.md`](ros2-architecture.md) once M3/M11 are
implemented. This document intentionally does not fix exact frame names to
avoid duplicated, potentially conflicting definitions across documents.

## 8. Drift Considerations

* VSLAM/VIO pose estimates drift over time and distance traveled, unlike
  Gazebo ground truth, which is always exact.
* The exploration algorithm should not assume perfect pose once M11 is
  implemented; excessive drift can cause the drone to select/generate
  waypoints based on an inaccurate map alignment.
* Loop closure (recognizing a previously visited area) is a standard
  mitigation in full SLAM systems (as opposed to pure VIO) and should be
  considered during M11 if drift proves significant in the target
  environments, but is not a hard Phase-1 requirement beyond what the
  chosen VSLAM/VIO library provides out of the box.
* Drift should be quantified during evaluation (M12) by comparing
  VSLAM/VIO pose against Gazebo ground truth, which remains available in
  simulation purely as an evaluation reference, never as an autonomy input,
  post-M11 (see [`evaluation.md`](evaluation.md), localization error
  metric).

## 9. How to Evaluate Localization

* Log both the VSLAM/VIO pose estimate and the Gazebo ground-truth pose
  during a run (ground truth used only for offline comparison, never fed
  back into the autonomy pipeline).
* Compute pose error (e.g., position error over time, drift rate) as
  described in [`evaluation.md`](evaluation.md).
* Correlate localization error against exploration outcomes (e.g., did
  drift cause a bad waypoint or map misalignment) to distinguish
  localization failures from exploration-algorithm failures — this is the
  entire point of the ground-truth-first strategy in §4.

## 10. Scope Note

This document intentionally keeps the localization discussion at the level
needed to integrate a VSLAM/VIO system into the exploration pipeline. It is
not a tutorial on SLAM/VIO theory, and it does not attempt to pick or tune
a specific VSLAM configuration ahead of M11 implementation — doing so here
would risk over-specifying a decision that should be informed by actual
integration experience.
