"""Run from the project root with python3 -m examples.basic."""

import math

from dock_planner import DockPlanner, DockSpec, PlannerConfig, Pose2D


def main():
    usv = Pose2D(2.0, 3.0, 0.0)
    dock = DockSpec(10.0, 20.0, math.pi / 2, bay_width=3.0, bay_depth=5.0)
    config = PlannerConfig(approach_distance=4.0, stop_distance_inside=2.0)
    result = DockPlanner(config).plan(usv, dock)
    print(f"USV: {usv}")
    print(f"Dock: {dock}")
    print(f"Config: {config}")
    print(f"status: {result.status.value}")
    print(f"reason: {result.reason}")
    print(f"approach_pose: {result.approach_pose}")
    for label, pose in zip(("usv", "approach", "entrance", "stop"), result.waypoints):
        print(f"{label:8s}: x={pose.x:.3f}, y={pose.y:.3f}, yaw={pose.yaw:.6f} rad")


if __name__ == "__main__":
    main()
