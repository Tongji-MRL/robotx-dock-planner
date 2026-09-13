"""Generate the Phase 1 target through the existing VRX adapter and planner."""

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from dock_planner import DockPlanner, PlannerConfig, PlanStatus, Pose2D
from dock_planner.vrx_reference import (
    VRX_REFERENCE_GEOMETRY,
    vrx_dock_2022_bay_to_dockspec,
)


DOCK_WORLD_POSE = (-480.0, 190.0, 1.0)
BAY_INDEX = 2
APPROACH_DISTANCE = 12.0
STOP_DISTANCE_INSIDE = 2.0


def build_phase1_target() -> dict:
    dock = vrx_dock_2022_bay_to_dockspec(*DOCK_WORLD_POSE, BAY_INDEX)
    # The dynamic controllers consume the planner-generated approach, entrance,
    # and stop poses. The runtime initial pose does not affect their geometry.
    planning_api_pose = Pose2D(0.0, 0.0, 0.0)
    result = DockPlanner(
        PlannerConfig(APPROACH_DISTANCE, STOP_DISTANCE_INSIDE)
    ).plan(planning_api_pose, dock)
    if result.status != PlanStatus.SUCCESS or result.approach_pose is None:
        raise RuntimeError(result.reason)
    return {
        "geometry_source": VRX_REFERENCE_GEOMETRY,
        "world": "scan_dock_deliver_task",
        "dock_world_pose": {
            "x": DOCK_WORLD_POSE[0],
            "y": DOCK_WORLD_POSE[1],
            "yaw": DOCK_WORLD_POSE[2],
        },
        "bay_index": BAY_INDEX,
        "planner_config": {
            "approach_distance": APPROACH_DISTANCE,
            "stop_distance_inside": STOP_DISTANCE_INSIDE,
        },
        "dock_spec": asdict(dock),
        "approach_pose": asdict(result.approach_pose),
        "entrance_pose": asdict(result.waypoints[2]),
        "stop_pose": asdict(result.waypoints[3]),
        "planner_status": result.status.value,
        "planner_reason": result.reason,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "generated/phase1_target.json",
    )
    args = parser.parse_args()
    record = build_phase1_target()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    pose = record["approach_pose"]
    print(f"Generated {args.output}")
    print(f"approach=({pose['x']:.9f}, {pose['y']:.9f}, {pose['yaw']:.9f})")


if __name__ == "__main__":
    main()
