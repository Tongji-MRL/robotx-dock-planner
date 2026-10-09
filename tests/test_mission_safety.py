import math
import random
import unittest

from dock_planner import (
    AdaptiveSearchRadiusConfig,
    BoundaryPathValidation,
    CircularDockSearchConfig,
    CircularDockSearchPlan,
    CoarseDockCenterObservationNED,
    CoarseDockSearchConfig,
    DockAttemptSupervisor,
    DockRetryConfig,
    NEDPoint,
    NavigableBoundaryNED,
    PlanResult,
    PlanStatus,
    Pose2D,
    PreciseEntrancePoseNED,
    RecoveryAction,
    SafeReturnGoalConfig,
    SearchBoundaryConfig,
    constrain_search_plan_to_boundary,
    plan_adaptive_boundary_safe_circular_search,
    plan_boundary_safe_circular_search,
    plan_exit_and_return,
    select_safe_return_goal,
    validate_pose_path_to_boundary,
    validate_terminal_plan_to_boundary,
)


def rectangle(min_north, max_north, min_east, max_east):
    return NavigableBoundaryNED((
        NEDPoint(min_north, min_east, 0.0),
        NEDPoint(max_north, min_east, 0.0),
        NEDPoint(max_north, max_east, 0.0),
        NEDPoint(min_north, max_east, 0.0),
    ))


def boundary_with_notch():
    return NavigableBoundaryNED((
        NEDPoint(0.0, -3.0, 0.0),
        NEDPoint(2.0, -3.0, 0.0),
        NEDPoint(2.0, 1.0, 0.0),
        NEDPoint(3.0, 1.0, 0.0),
        NEDPoint(3.0, -3.0, 0.0),
        NEDPoint(6.0, -3.0, 0.0),
        NEDPoint(6.0, 3.0, 0.0),
        NEDPoint(0.0, 3.0, 0.0),
    ))


class SearchBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.coarse = CoarseDockCenterObservationNED(
            "dock_mock", NEDPoint(0.0, 0.0, 0.0), 10.0, True,
        )
        self.approach = CoarseDockSearchConfig(
            dock_length_m=4.0,
            dock_width_m=2.0,
            vessel_envelope_radius_m=0.0,
            dimension_measurement_margin_m=0.0,
            stand_off_distance_m=1.0,
            position_uncertainty_m=0.0,
        )
        self.arc = CircularDockSearchConfig(
            arc_span_rad=math.pi,
            waypoint_spacing_m=0.5,
        )
        self.safety = SearchBoundaryConfig(
            min_boundary_clearance_m=0.5,
            min_safe_arc_length_m=2.0,
        )

    def test_large_water_area_keeps_full_arc(self):
        result = plan_boundary_safe_circular_search(
            Pose2D(-10.0, 0.0, 0.0),
            self.coarse,
            rectangle(-20.0, 20.0, -20.0, 20.0),
            approach_config=self.approach,
            arc_config=self.arc,
            boundary_config=self.safety,
        )
        self.assertTrue(result.valid)
        self.assertTrue(result.full_requested_arc_safe)
        self.assertEqual(result.rejected_waypoint_count, 0)

    def test_shoreline_truncates_arc_and_selects_safe_direction(self):
        result = plan_boundary_safe_circular_search(
            Pose2D(-10.0, 0.0, 0.0),
            self.coarse,
            rectangle(-12.0, 2.7, -8.0, 8.0),
            approach_config=self.approach,
            arc_config=self.arc,
            boundary_config=self.safety,
        )
        self.assertTrue(result.valid)
        self.assertFalse(result.full_requested_arc_safe)
        self.assertGreater(result.rejected_waypoint_count, 0)
        for waypoint in result.plan.arc_waypoints:
            self.assertTrue(
                rectangle(-12.0, 2.7, -8.0, 8.0).contains_with_clearance(
                    waypoint.x, waypoint.y, 0.5
                )
            )

    def test_no_safe_staging_is_rejected(self):
        result = plan_boundary_safe_circular_search(
            Pose2D(-10.0, 0.0, 0.0),
            self.coarse,
            rectangle(-1.0, 1.0, -1.0, 1.0),
            approach_config=self.approach,
            arc_config=self.arc,
            boundary_config=self.safety,
        )
        self.assertFalse(result.valid)
        self.assertIsNone(result.plan)

    def test_self_intersecting_boundary_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "self-intersect"):
            NavigableBoundaryNED((
                NEDPoint(0.0, 0.0, 0.0),
                NEDPoint(2.0, 2.0, 0.0),
                NEDPoint(0.0, 2.0, 0.0),
                NEDPoint(2.0, 0.0, 0.0),
            ))

    def test_segment_rejects_concave_notch_between_safe_endpoints(self):
        boundary = boundary_with_notch()
        start = Pose2D(1.0, 0.0, 0.0)
        end = Pose2D(4.0, 0.0, 0.0)
        self.assertTrue(boundary.contains(start.x, start.y))
        self.assertTrue(boundary.contains(end.x, end.y))
        self.assertFalse(boundary.segment_with_clearance(start, end, 0.0))
        validation = validate_pose_path_to_boundary((start, end), boundary)
        self.assertIsInstance(validation, BoundaryPathValidation)
        self.assertFalse(validation.valid)
        self.assertEqual(validation.failed_segment_index, 0)

    def test_inside_circle_outward_transition_must_remain_in_boundary(self):
        result = plan_boundary_safe_circular_search(
            Pose2D(1.0, 0.0, 0.0),
            CoarseDockCenterObservationNED(
                "dock_mock", NEDPoint(0.0, 0.0, 0.0), 10.0, True,
            ),
            boundary_with_notch(),
            approach_config=CoarseDockSearchConfig(
                dock_length_m=1.0,
                dock_width_m=1.0,
                vessel_envelope_radius_m=0.1,
                dimension_measurement_margin_m=0.0,
                stand_off_distance_m=3.2,
                position_uncertainty_m=0.0,
            ),
            arc_config=CircularDockSearchConfig(
                arc_span_rad=0.4,
                waypoint_spacing_m=0.2,
            ),
            boundary_config=SearchBoundaryConfig(
                min_boundary_clearance_m=0.05,
                min_safe_arc_length_m=0.5,
            ),
        )
        self.assertFalse(result.valid)
        self.assertIn("current-to-staging transition", result.reason)

    def test_search_arc_rejects_unsafe_chord_between_safe_waypoints(self):
        boundary = boundary_with_notch()
        raw = CircularDockSearchPlan(
            dock_id="dock_mock",
            center_ned=NEDPoint(0.0, 0.0, 0.0),
            radius_m=4.0,
            staging_pose=Pose2D(1.0, 0.0, 0.0),
            arc_waypoints=(Pose2D(4.0, 0.0, 0.0),),
            clockwise=True,
            arc_span_rad=0.5,
        )
        result = constrain_search_plan_to_boundary(
            raw,
            boundary,
            SearchBoundaryConfig(
                min_boundary_clearance_m=0.0,
                min_safe_arc_length_m=0.5,
            ),
        )
        self.assertFalse(result.valid)
        self.assertEqual(result.safe_arc_length_m, 0.0)

    def test_terminal_plan_checks_every_segment(self):
        safe = PlanResult(
            Pose2D(1.0, 2.0, 0.0),
            waypoints=(Pose2D(1.0, 2.0, 0.0), Pose2D(4.0, 2.0, 0.0)),
            status=PlanStatus.SUCCESS,
        )
        unsafe = PlanResult(
            Pose2D(1.0, 0.0, 0.0),
            waypoints=(Pose2D(1.0, 0.0, 0.0), Pose2D(4.0, 0.0, 0.0)),
            status=PlanStatus.SUCCESS,
        )
        self.assertTrue(validate_terminal_plan_to_boundary(
            safe,
            boundary_with_notch(),
        ).valid)
        self.assertFalse(validate_terminal_plan_to_boundary(
            unsafe,
            boundary_with_notch(),
        ).valid)

    def test_random_rectangular_fields_never_return_unsafe_waypoints(self):
        rng = random.Random(42)
        for _ in range(40):
            west = rng.uniform(4.0, 12.0)
            east = rng.uniform(3.0, 12.0)
            south = rng.uniform(3.0, 12.0)
            north = rng.uniform(3.0, 12.0)
            boundary = rectangle(-west, east, -south, north)
            result = plan_boundary_safe_circular_search(
                Pose2D(-10.0, 0.0, 0.0),
                self.coarse,
                boundary,
                approach_config=self.approach,
                arc_config=self.arc,
                boundary_config=self.safety,
            )
            if result.valid:
                self.assertTrue(boundary.contains_with_clearance(
                    result.plan.staging_pose.x, result.plan.staging_pose.y, 0.5
                ))
                self.assertTrue(all(
                    boundary.contains_with_clearance(point.x, point.y, 0.5)
                    for point in result.plan.arc_waypoints
                ))

    def test_adaptive_search_reduces_radius_only_after_both_directions_fail(self):
        result = plan_adaptive_boundary_safe_circular_search(
            Pose2D(-5.5, 0.0, 0.0),
            self.coarse,
            rectangle(-6.0, 4.0, -3.5, 3.5),
            approach_config=CoarseDockSearchConfig(
                dock_length_m=4.0,
                dock_width_m=2.0,
                vessel_envelope_radius_m=0.0,
                dimension_measurement_margin_m=0.0,
                stand_off_distance_m=2.0,
                position_uncertainty_m=0.0,
            ),
            arc_config=self.arc,
            boundary_config=SearchBoundaryConfig(
                min_boundary_clearance_m=0.1,
                min_safe_arc_length_m=5.0,
            ),
            radius_config=AdaptiveSearchRadiusConfig(
                min_stand_off_distance_m=0.5,
                radius_step_m=0.5,
                max_radius_reductions=3,
            ),
        )
        self.assertTrue(result.valid)
        self.assertGreater(result.radius_reductions, 0)
        self.assertEqual(len(result.attempted_radii_m), result.radius_reductions + 1)
        self.assertLess(result.plan.radius_m, result.attempted_radii_m[0])
        self.assertGreaterEqual(result.plan.radius_m, math.hypot(2.0, 1.0) + 0.5)

    def test_adaptive_search_keeps_initial_radius_when_it_is_safe(self):
        result = plan_adaptive_boundary_safe_circular_search(
            Pose2D(-10.0, 0.0, 0.0),
            self.coarse,
            rectangle(-20.0, 20.0, -20.0, 20.0),
            approach_config=self.approach,
            arc_config=self.arc,
            boundary_config=self.safety,
            radius_config=AdaptiveSearchRadiusConfig(
                min_stand_off_distance_m=0.25,
            ),
        )
        self.assertTrue(result.valid)
        self.assertEqual(result.radius_reductions, 0)
        self.assertEqual(result.attempted_radii_m, (self.approach.search_radius_m,))

    def test_adaptive_search_stops_at_configured_minimum_radius(self):
        approach = CoarseDockSearchConfig(
            dock_length_m=4.0,
            dock_width_m=2.0,
            vessel_envelope_radius_m=0.0,
            dimension_measurement_margin_m=0.0,
            stand_off_distance_m=2.0,
            position_uncertainty_m=0.25,
        )
        result = plan_adaptive_boundary_safe_circular_search(
            Pose2D(-10.0, 0.0, 0.0),
            self.coarse,
            rectangle(-1.0, 1.0, -1.0, 1.0),
            approach_config=approach,
            arc_config=self.arc,
            boundary_config=self.safety,
            radius_config=AdaptiveSearchRadiusConfig(
                min_stand_off_distance_m=0.75,
                radius_step_m=0.5,
                max_radius_reductions=10,
            ),
        )
        minimum_radius = math.hypot(2.0, 1.0) + 0.25 + 0.75
        self.assertFalse(result.valid)
        self.assertAlmostEqual(result.attempted_radii_m[-1], minimum_radius)
        self.assertTrue(all(radius >= minimum_radius for radius in result.attempted_radii_m))


class RecoveryTests(unittest.TestCase):
    def test_attempt_limit_leads_to_exit_and_return(self):
        supervisor = DockAttemptSupervisor(DockRetryConfig(
            max_attempts=3,
            attempt_timeout_s=10.0,
            total_timeout_s=40.0,
        ))
        self.assertEqual(supervisor.begin(0.0).action, RecoveryAction.CONTINUE)
        self.assertEqual(
            supervisor.record_failure(3.0, "no entrance").action,
            RecoveryAction.RETRY,
        )
        self.assertEqual(
            supervisor.record_failure(6.0, "color uncertain").action,
            RecoveryAction.RETRY,
        )
        final = supervisor.record_failure(9.0, "alignment failed")
        self.assertEqual(final.action, RecoveryAction.EXIT_AND_RETURN)
        self.assertEqual(final.attempt_number, 3)

    def test_attempt_and_total_timeouts_are_enforced(self):
        per_attempt = DockAttemptSupervisor(DockRetryConfig(3, 10.0, 40.0))
        per_attempt.begin(0.0)
        self.assertEqual(per_attempt.check_time(10.0).action, RecoveryAction.RETRY)

        total = DockAttemptSupervisor(DockRetryConfig(3, 20.0, 25.0))
        total.begin(0.0)
        total.record_failure(5.0, "first failure")
        self.assertEqual(total.check_time(25.0).action, RecoveryAction.EXIT_AND_RETURN)

    def test_exit_plan_moves_outward_then_to_wait_goal(self):
        entrance = PreciseEntrancePoseNED(
            x_ned=2.0,
            y_ned=3.0,
            heading_rad=0.0,
            timestamp=10.0,
            valid=True,
        )
        boundary = rectangle(-10.0, 10.0, -10.0, 10.0)
        wait_goal = Pose2D(-7.0, 5.0, math.pi / 2)
        plan = plan_exit_and_return(
            entrance,
            wait_goal,
            exit_distance_m=4.0,
            boundary=boundary,
            min_boundary_clearance_m=0.5,
        )
        self.assertEqual(plan.exit_goal.x, -2.0)
        self.assertEqual(plan.exit_goal.y, 3.0)
        self.assertAlmostEqual(abs(plan.exit_goal.yaw), math.pi)
        self.assertEqual(plan.return_goal, wait_goal)
        self.assertIsNotNone(plan.minimum_boundary_clearance_m)

    def test_exit_plan_rejects_safe_endpoints_connected_through_shore(self):
        entrance = PreciseEntrancePoseNED(
            x_ned=1.0,
            y_ned=0.0,
            heading_rad=math.pi,
            timestamp=10.0,
            valid=True,
        )
        with self.assertRaisesRegex(ValueError, "exit/return path is unsafe"):
            plan_exit_and_return(
                entrance,
                Pose2D(5.0, 0.0, 0.0),
                current_pose=Pose2D(1.0, 0.0, math.pi),
                exit_distance_m=3.0,
                boundary=boundary_with_notch(),
            )

    def test_safe_return_goal_preserves_safe_preference_or_repairs_it(self):
        boundary = rectangle(-10.0, 10.0, -10.0, 10.0)
        config = SafeReturnGoalConfig(
            min_dock_distance_m=4.0,
            min_boundary_clearance_m=1.0,
            grid_spacing_m=1.0,
        )
        preferred = Pose2D(-7.0, 5.0, 0.3)
        self.assertEqual(
            select_safe_return_goal(
                boundary,
                NEDPoint(0.0, 0.0, 0.0),
                config,
                preferred_goal=preferred,
            ),
            preferred,
        )
        repaired = select_safe_return_goal(
            boundary,
            NEDPoint(0.0, 0.0, 0.0),
            config,
            preferred_goal=Pose2D(0.0, 0.0, 0.0),
        )
        self.assertTrue(boundary.contains_with_clearance(repaired.x, repaired.y, 1.0))
        self.assertGreaterEqual(math.hypot(repaired.x, repaired.y), 4.0)

    def test_safe_return_goal_allows_zero_boundary_clearance(self):
        config = SafeReturnGoalConfig(
            min_dock_distance_m=2.0,
            min_boundary_clearance_m=0.0,
        )

        self.assertEqual(config.min_boundary_clearance_m, 0.0)

    def test_safe_return_goal_rejects_impossible_field(self):
        with self.assertRaisesRegex(ValueError, "no safe return goal"):
            select_safe_return_goal(
                rectangle(-1.0, 1.0, -1.0, 1.0),
                NEDPoint(0.0, 0.0, 0.0),
                SafeReturnGoalConfig(
                    min_dock_distance_m=10.0,
                    min_boundary_clearance_m=0.1,
                    grid_spacing_m=0.25,
                ),
            )


if __name__ == "__main__":
    unittest.main()
