"""Dry-run UAV semantic-map conversion through the coarse dock approach input."""

import json

from dock_planner import (
    CoarseDockSearchConfig,
    GpsOrigin,
    Pose2D,
    coarse_dock_staging_pose,
    semantic_map_to_coarse_docks,
    uav_origin_in_usv_ned,
)


def main() -> None:
    # These values are synthetic and exercise only the coordinate/data flow.
    uav_mapping_origin = GpsOrigin(31.230410, 121.473710, 15.2)
    usv_task_origin = GpsOrigin(31.230400, 121.473700, 14.7)
    semantic_map_json = json.dumps({
        "objects": [
            {
                "type": "dock",
                "id": "dock_mock",
                "state": "static",
                "NED_position": [20.0, 35.0, 0.0],
            },
            {
                "type": "buoy",
                "id": "buoy_mock",
                "NED_position": [8.0, 2.0, 0.0],
            },
        ]
    })

    offset = uav_origin_in_usv_ned(uav_mapping_origin, usv_task_origin)
    docks = semantic_map_to_coarse_docks(
        semantic_map_json,
        uav_mapping_origin,
        usv_task_origin,
        timestamp=100.0,
    )
    assert len(docks) == 1
    dock = docks[0]
    staging = coarse_dock_staging_pose(
        Pose2D(0.0, 0.0, 0.0),
        dock,
        CoarseDockSearchConfig(),
    )

    print("offset north/east/down = " + ", ".join(f"{value:.3f}" for value in offset))
    print(
        "USV dock center         = "
        f"({dock.center_ned.north:.3f}, {dock.center_ned.east:.3f}, "
        f"{dock.center_ned.down:.3f})"
    )
    print(f"staging pose            = ({staging.x:.3f}, {staging.y:.3f}, {staging.yaw:.3f})")
    print("UAV MAP -> USV NED -> STAGING DRY RUN PASS")


if __name__ == "__main__":
    main()
