"""Run the complete center-to-arc-to-terminal Task 3 logic with mock data.

This script is ROS independent and never publishes a vehicle command.  Pose
arrival events are simulated, while the entrance observation is produced by
the NumPy dock-perception prototype from a deterministic synthetic point
cloud.
"""

import math

from dock_planner import (
    BayColorObservation,
    CircularDockSearchConfig,
    CoarseDockCenterObservationNED,
    CoarseDockSearchConfig,
    DataFreshnessConfig,
    DockPerceptionConfig,
    DockPerceptionResult,
    DockPlanner,
    NEDPoint,
    NumpyDockPerception,
    PlannerConfig,
    Pose2D,
    SyntheticDockConfig,
    SprayerTargetObservation,
    Task3FlowManager,
    Task3FlowState,
    WateringCompletionTracker,
    WateringVerificationConfig,
    generate_synthetic_dock_point_cloud,
    parse_logistics_request_json,
)


def _show(manager: Task3FlowManager, event: str) -> None:
    print(f"{event:<28} -> {manager.state.value}")


def _empty_result(timestamp: float) -> DockPerceptionResult:
    return DockPerceptionResult(
        dock_id="dock_mock",
        entrance_observations=(),
        timestamp=timestamp,
        valid_until=timestamp + 2.0,
        source="mock_lidar_no_detection",
    )


def main() -> None:
    current_pose = Pose2D(0.0, 0.0, 0.0)
    center = NEDPoint(20.0, 35.0, 0.0)
    coarse_center = CoarseDockCenterObservationNED(
        dock_id="dock_mock",
        center_ned=center,
        timestamp=100.0,
        valid=True,
        confidence=0.90,
        source="mock_uav_usv_ned",
    )
    search_config = CoarseDockSearchConfig()
    arc_config = CircularDockSearchConfig(
        arc_span_rad=math.pi,
        waypoint_spacing_m=1.5,
        clockwise=True,
    )
    manager = Task3FlowManager(
        DockPlanner(PlannerConfig(approach_distance=4.0, stop_distance_inside=2.0)),
        target_color="GREEN",
        bay_width=3.0,
        bay_depth=5.0,
        freshness=DataFreshnessConfig(
            coarse_dock_timeout_s=30.0,
            perception_timeout_s=2.0,
        ),
    )

    search_plan = manager.begin_center_search(
        coarse_center,
        current_pose,
        approach_config=search_config,
        arc_config=arc_config,
        now=100.5,
    )
    if search_plan is None:
        raise RuntimeError(manager.error or "search plan was not produced")
    print(f"search radius               = {search_plan.radius_m:.3f} m")
    print(
        "staging pose                = "
        f"({search_plan.staging_pose.x:.3f}, {search_plan.staging_pose.y:.3f}, "
        f"{search_plan.staging_pose.yaw:.3f} rad)"
    )
    print(f"arc waypoint count          = {len(search_plan.arc_waypoints)}")
    _show(manager, "center-only coarse accepted")

    manager.approach_reached()
    _show(manager, "staging pose reached")

    # Simulate two arc goals where a 360-degree LiDAR has no usable entrance.
    now = 101.0
    for index in range(2):
        pose = manager.current_search_goal
        if pose is None:
            raise RuntimeError("search arc ended before the mock detection")
        accepted = manager.accept_perception_result(
            _empty_result(now), pose, now=now + 0.1,
        )
        if accepted:
            raise RuntimeError("empty perception result was unexpectedly accepted")
        print(f"arc waypoint {index + 1:02d}          = no entrance")
        manager.search_waypoint_reached()
        now += 0.4

    # The synthetic point cloud stands in for one transformed LiDAR scan.
    cloud = generate_synthetic_dock_point_cloud(SyntheticDockConfig(
        center_north_m=center.north,
        center_east_m=center.east,
        heading_rad=0.35,
        noise_std_m=0.03,
        dropout_probability=0.10,
        seed=23,
    ))
    active_search_pose = manager.current_search_goal
    if active_search_pose is None:
        raise RuntimeError("search arc ended before perception")
    detector = NumpyDockPerception(DockPerceptionConfig(
        range_origin_ned=(active_search_pose.x, active_search_pose.y, 0.0),
    ))
    first_result = detector.detect(cloud, coarse_center, timestamp=102.0)
    if not manager.accept_perception_result(
        first_result, active_search_pose, now=102.1,
    ):
        raise RuntimeError("synthetic entrance was not accepted")
    _show(manager, "synthetic entrance observed")

    manager.accept_color_observation(
        BayColorObservation("RED", 0.95, 102.2, "mock_color"), now=102.3,
    )
    _show(manager, "red bay rejected")
    exit_plan = manager.prepare_non_target_bay_exit()
    print(
        "non-target exit goal        = "
        f"({exit_plan.exit_goal.x:.3f}, {exit_plan.exit_goal.y:.3f})"
    )
    manager.next_bay()
    manager.search_waypoint_reached()
    _show(manager, "arc search resumed")

    selected_pose = manager.current_search_goal
    if selected_pose is None:
        raise RuntimeError("search arc ended before selected bay")
    selected_result = detector.detect(cloud, coarse_center, timestamp=102.7)
    if not manager.accept_perception_result(
        selected_result, selected_pose, now=102.8,
    ):
        raise RuntimeError("selected synthetic entrance was not accepted")
    plan = manager.accept_color_observation(
        BayColorObservation("GREEN", 0.98, 102.9, "mock_color"), now=103.0,
    )
    _show(manager, "green bay selected")
    if plan is None:
        raise RuntimeError("terminal plan was not produced")
    for index, waypoint in enumerate(plan.waypoints):
        print(
            f"terminal waypoint[{index}]        = "
            f"({waypoint.x:.3f}, {waypoint.y:.3f}, {waypoint.yaw:.3f} rad)"
        )

    manager.docking_complete()
    _show(manager, "terminal docking done")
    manager.docked_confirmed()
    _show(manager, "docked confirmed")

    watering = WateringCompletionTracker(WateringVerificationConfig(
        green_confirmation_s=5.0,
        observation_timeout_s=1.1,
    ))
    red = watering.update(SprayerTargetObservation(
        target_valid=True,
        delivery_confirmed=False,
        timestamp=103.5,
        source="mock_sprayer_perception",
    ))
    print(f"red window / stop pump      = {red.stop_pump_requested}")
    green = None
    for timestamp in (104.0, 105.0, 106.0, 107.0, 108.0, 109.0):
        green = watering.update(SprayerTargetObservation(
            target_valid=False,
            delivery_confirmed=True,
            timestamp=timestamp,
            source="mock_sprayer_perception",
        ))
    if green is None or not green.watering_complete:
        raise RuntimeError("five-second GREEN confirmation did not complete")
    print(f"green confirmed             = {green.confirmed_green_s:.1f} s")

    sequence = parse_logistics_request_json(
        '{"valid":true,"rule":"DISRUPTIVE",'
        '"resource_color":"BLUE","destination_color":"GREEN"}',
        timestamp=109.1,
        source="mock_logistics_request",
    )
    if not sequence.is_actionable:
        raise RuntimeError("mock logistics sequence is not actionable")
    print(
        "resource sequence            = "
        f"({sequence.resource_color}, {sequence.destination_color})"
    )
    manager.watering_complete()
    _show(manager, "watering complete")

    if manager.state is not Task3FlowState.FINISHED:
        raise RuntimeError(f"dry run ended in {manager.state.value}")
    print("FULL TASK3 CENTER/ARC DRY RUN PASS")


if __name__ == "__main__":
    main()
