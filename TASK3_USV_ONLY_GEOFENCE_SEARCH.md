# Task 3 USV-only geofence search

This document describes the USV-only search path kept on the
`feature/task3-usv-only-geofence-search` branch.  The earlier UAV-seeded path
remains available through `detector_mode:=seeded_lines`; this branch adds the
independent `unseeded_lines` mode instead of replacing that implementation.

## Mission path

1. Convert the four official geofence GPS corners into `robotx_rtk_ned`.
2. Generate an inset midpoint-to-midpoint search line and one orthogonal
   fallback line.
3. Stop at the line stations and accumulate Mid-360 observations.
4. Confirm three parallel entrances in at least three frames.
5. Resolve the inward heading and assign Bay 1/2/3 from left to right while
   facing the dock.
6. Publish the recovered coarse centre, entrance observations, and one camera
   observation pose for every bay.
7. Hand the selected entrance to the existing colour and terminal docking
   flow.

The current implementation generates the search stations and perception
targets.  Connecting those search stations to the guarded waypoint controller
is a separate vehicle-control step and is not simulated by this branch.

## RViz simulation

Build `robotx_perception_common` and `usv_task3_executor`, source the resulting
overlay, then run:

```bash
ros2 launch robotx_perception_common task3_geofence_search_rviz.launch.py
```

RViz displays the geofence, primary and fallback search lines, observation
stations, synthetic Mid-360 cloud, recovered coarse centre, Bay 1/2/3
entrances, and camera observation points.  The expected terminal confirmation
is:

```text
UNSEEDED DOCK CONFIRMED: 3 entrances
```

The simulation exercises the production PointCloud2 perception node.  It does
not model water dynamics, propulsion, or collision avoidance.

## Real perception launch

After Livox, RTK pose, and the common TF are running:

```bash
ros2 launch robotx_perception_common task3_lidar_perception.launch.py \
  detector_mode:=unseeded_lines \
  local_max_candidates:=6 \
  local_max_range_m:=10.0 \
  local_lateral_half_width_m:=12.0 \
  unseeded_required_detections:=3 \
  unseeded_min_entrances:=3
```
