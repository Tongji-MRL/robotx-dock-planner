# Gazebo dock geometry experiments

This directory is isolated from the Task 3 planner and executor.

## RobotX 2026 geometry prototype

`models/robotx_2026_task3_dock/model.sdf` encodes the dimensions visible in
the official top-view drawing supplied by the team:

- overall width: 6.596 m
- overall depth: 3.000 m
- main deck: 13 cube positions by 2 cube positions
- four fingers, each 1 cube by 4 cubes
- three cube positions between adjacent fingers

The model uses one box for the main deck and one box for each finger. It is a
LiDAR collision-geometry prototype, not an official competition CAD model.
Connector protrusions, cube seams, material response, target panels, exact
freeboard, and flotation dynamics are not inferred from the drawing.

`robotx_2026_dock_lidar_test.sdf` places a static 3D LiDAR 6 m in front of the
three bay openings. It publishes Gazebo point-cloud data on:

`/robotx_2026_dock_lidar/scan/points`

## First static-view result

The simulated cloud contains all three physical bays, but the front-center
view provides a clean line of sight mainly into the middle bay. The current
prototype generated several geometric candidates and ranked an internal gap
above the true front entrance. This is a known algorithm limitation, not a
model or sensor failure.

Before integration, candidate ranking should require an externally visible
entrance boundary and multi-view observations should be associated while the
USV moves along the coarse-dock safety arc.
