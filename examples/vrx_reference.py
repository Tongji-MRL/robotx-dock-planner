"""Print reference geometry and planner poses: python3 -m examples.vrx_reference."""

from dock_planner import DockPlanner, PlannerConfig, PlanStatus, Pose2D
from dock_planner.vrx_reference import (
    VRX_REFERENCE_GEOMETRY, vrx_dock_2022_bay_to_dockspec,
)


def main():
    print(f"{VRX_REFERENCE_GEOMETRY}: bay width=4 m, depth=6 m")
    print("Not RobotX 2026 official dimensions.")
    print("VRX world dock pose: x=-480 m, y=190 m, yaw=1.0 rad")
    print("Test USV pose: (-500, 180, 1.0); approach=4 m; stop_inside=2 m")
    print("The USV pose is synthetic, not the official world's spawn pose.")
    print("bay    point       world_x       world_y    heading_rad")
    planner = DockPlanner(PlannerConfig(4.0, 2.0))
    for index in (1, 2, 3):
        dock = vrx_dock_2022_bay_to_dockspec(-480, 190, 1.0, index)
        result = planner.plan(Pose2D(-500, 180, 1.0), dock)
        if result.status != PlanStatus.SUCCESS:
            raise RuntimeError(result.reason)
        for label, pose in zip(("approach", "entrance", "stop"), result.waypoints[1:]):
            print(f"{dock.dock_id:6s} {label:8s} {pose.x:13.6f} {pose.y:13.6f} {pose.yaw:12.6f}")
    print("SUCCESS means geometry only; collision and motion feasibility are not checked.")


if __name__ == "__main__":
    main()
