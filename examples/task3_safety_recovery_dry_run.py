"""Exercise random mock boundaries, retry limits, and return goals offline."""

import math
import random

from dock_planner import (
    CircularDockSearchConfig,
    CoarseDockCenterObservationNED,
    CoarseDockSearchConfig,
    DockAttemptSupervisor,
    DockRetryConfig,
    NEDPoint,
    NavigableBoundaryNED,
    Pose2D,
    PreciseEntrancePoseNED,
    SearchBoundaryConfig,
    SafeReturnGoalConfig,
    plan_boundary_safe_circular_search,
    plan_exit_and_return,
    select_safe_return_goal,
)


def main() -> None:
    rng = random.Random(20261006)
    shore_north = rng.uniform(4.0, 7.0)
    boundary = NavigableBoundaryNED((
        NEDPoint(-15.0, -12.0, 0.0),
        NEDPoint(shore_north, -12.0, 0.0),
        NEDPoint(shore_north, 12.0, 0.0),
        NEDPoint(-15.0, 12.0, 0.0),
    ))
    coarse = CoarseDockCenterObservationNED(
        "dock_random_mock", NEDPoint(0.0, 0.0, 0.0), 100.0, True,
    )
    checked = plan_boundary_safe_circular_search(
        Pose2D(-12.0, 0.0, 0.0),
        coarse,
        boundary,
        approach_config=CoarseDockSearchConfig(
            dock_length_m=6.596,
            dock_width_m=3.0,
            stand_off_distance_m=2.0,
            position_uncertainty_m=0.5,
        ),
        arc_config=CircularDockSearchConfig(
            arc_span_rad=math.pi,
            waypoint_spacing_m=1.0,
        ),
        boundary_config=SearchBoundaryConfig(
            min_boundary_clearance_m=1.0,
            min_safe_arc_length_m=4.0,
        ),
    )
    print(f"mock shore north limit      = {shore_north:.2f} m")
    print(f"safe search available       = {checked.valid}")
    print(f"boundary decision           = {checked.reason}")
    if checked.valid:
        print(f"safe arc direction clockwise= {checked.plan.clockwise}")
        print(f"safe arc waypoints          = {len(checked.plan.arc_waypoints)}")
        print(f"safe arc length             = {checked.safe_arc_length_m:.2f} m")

    supervisor = DockAttemptSupervisor(DockRetryConfig(
        max_attempts=3,
        attempt_timeout_s=60.0,
        total_timeout_s=150.0,
    ))
    supervisor.begin(100.0)
    for timestamp, reason in (
        (125.0, "entrance not stable"),
        (165.0, "bay color uncertain"),
        (205.0, "terminal alignment timeout"),
    ):
        decision = supervisor.record_failure(timestamp, reason)
        print(
            f"attempt decision            = {decision.action.value} "
            f"(attempt {decision.attempt_number}: {decision.reason})"
        )

    entrance = PreciseEntrancePoseNED(
        0.0, 0.0, 0.0, 205.0, True, dock_id="dock_random_mock"
    )
    preferred_return_goal = Pose2D(
        rng.uniform(-12.0, -9.0),
        rng.uniform(-5.0, 5.0),
        0.0,
    )
    return_goal = select_safe_return_goal(
        boundary,
        coarse.center_ned,
        SafeReturnGoalConfig(
            min_dock_distance_m=8.0,
            min_boundary_clearance_m=1.0,
            grid_spacing_m=0.5,
        ),
        preferred_goal=preferred_return_goal,
    )
    recovery = plan_exit_and_return(
        entrance,
        return_goal,
        exit_distance_m=4.0,
        boundary=boundary,
        min_boundary_clearance_m=1.0,
    )
    print(f"exit goal                   = {recovery.exit_goal}")
    print(f"mock shore wait goal        = {recovery.return_goal}")
    print("TASK3 SAFETY/RECOVERY DRY RUN PASS")


if __name__ == "__main__":
    main()
