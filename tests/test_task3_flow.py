import math
import unittest

from dock_planner import (
    BayColorObservation,
    CircularDockSearchConfig,
    CoarseDockCenterObservationNED,
    CoarseDockCirclePosition,
    CoarseDockInfo,
    CoarseDockSearchConfig,
    DockEntranceObservationNED,
    DockPlanner,
    NEDPoint,
    PlannerConfig,
    Pose2D,
    PlanStatus,
    Task3FlowManager,
    Task3FlowState,
)


class Task3FlowTests(unittest.TestCase):
    def setUp(self):
        self.coarse = CoarseDockInfo(
            "dock_1",
            NEDPoint(10, 20, 0),
            (NEDPoint(8, 18, 0), NEDPoint(12, 18, 0),
             NEDPoint(12, 22, 0), NEDPoint(8, 22, 0)),
            10.0,
            True,
        )
        self.manager = Task3FlowManager(
            DockPlanner(PlannerConfig(4, 2)),
            target_color="GREEN",
            bay_width=3,
            bay_depth=5,
        )
        self.entrance = DockEntranceObservationNED(
            9.0, 20.0, math.pi / 2, 0.9, 11.0, "mid360", dock_id="dock_1"
        )

    def test_state_vocabulary_matches_orin_contract(self):
        self.assertEqual(
            [state.value for state in Task3FlowState],
            [
                "IDLE",
                "APPROACH_DOCK",
                "SEARCH_DOCK",
                "CHECK_BAY_COLOR",
                "NEXT_BAY",
                "TERMINAL_DOCKING",
                "DOCKED",
                "WATERING",
                "FINISHED",
                "ERROR",
            ],
        )

    def test_confirmed_flow_reaches_docked_watering_and_finished(self):
        self.manager.begin(self.coarse)
        self.assertEqual(self.manager.state, Task3FlowState.APPROACH_DOCK)
        self.manager.approach_reached()
        self.manager.accept_entrance_observation(self.entrance, Pose2D(0, 0, 0))
        self.assertEqual(self.manager.state, Task3FlowState.CHECK_BAY_COLOR)
        plan = self.manager.accept_color_observation(
            BayColorObservation("green", 0.95, 12.0, "perception_group")
        )
        self.assertEqual(plan.status, PlanStatus.SUCCESS)
        self.assertEqual(self.manager.state, Task3FlowState.TERMINAL_DOCKING)
        self.assertEqual(plan.waypoints[2], Pose2D(9, 20, math.pi / 2))
        self.manager.docking_complete()
        self.assertEqual(self.manager.state, Task3FlowState.DOCKED)
        self.manager.docked_confirmed()
        self.assertEqual(self.manager.state, Task3FlowState.WATERING)
        self.manager.watering_complete()
        self.assertEqual(self.manager.state, Task3FlowState.FINISHED)

    def test_wrong_color_visits_next_bay_before_search(self):
        self.manager.begin(self.coarse)
        self.manager.approach_reached()
        self.manager.accept_entrance_observation(self.entrance, Pose2D(0, 0, 0))
        self.assertIsNone(self.manager.accept_color_observation(
            BayColorObservation("RED", 0.9, 12.0, "perception_group")
        ))
        self.assertEqual(self.manager.state, Task3FlowState.NEXT_BAY)
        self.manager.next_bay()
        self.assertEqual(self.manager.state, Task3FlowState.SEARCH_DOCK)
        self.assertIsNone(self.manager.plan)

    def test_invalid_input_enters_error_without_point_cloud_processing(self):
        self.manager.begin(CoarseDockInfo(
            "dock_1", self.coarse.center_ned, self.coarse.approximate_corners_ned, 10, False
        ))
        self.assertEqual(self.manager.state, Task3FlowState.ERROR)
        self.assertIn("coarse dock", self.manager.error)

    def test_center_only_flow_advances_along_arc_and_stops_on_entrance(self):
        center_only = CoarseDockCenterObservationNED(
            "dock_1", NEDPoint(10.0, 20.0, 0.0), 10.0, True,
        )
        search_plan = self.manager.begin_center_search(
            center_only,
            Pose2D(0.0, 20.0, 0.0),
            approach_config=CoarseDockSearchConfig(
                dock_length_m=2.0,
                dock_width_m=2.0,
                stand_off_distance_m=1.0,
                position_uncertainty_m=0.0,
            ),
            arc_config=CircularDockSearchConfig(
                arc_span_rad=math.pi / 2,
                waypoint_spacing_m=1.0,
            ),
            now=10.5,
        )
        self.assertIsNotNone(search_plan)
        self.assertEqual(self.manager.state, Task3FlowState.APPROACH_DOCK)
        self.assertEqual(self.manager.approach_goal, search_plan.staging_pose)

        self.manager.approach_reached()
        first_goal = self.manager.current_search_goal
        self.assertIsNotNone(first_goal)
        next_goal = self.manager.search_waypoint_reached()
        self.assertIsNotNone(next_goal)
        self.assertNotEqual(first_goal, next_goal)

        self.manager.accept_entrance_observation(
            self.entrance,
            next_goal,
            now=11.5,
        )
        self.assertEqual(self.manager.state, Task3FlowState.CHECK_BAY_COLOR)
        self.assertIsNone(self.manager.current_search_goal)
        plan = self.manager.accept_color_observation(
            BayColorObservation("GREEN", 0.95, 11.6, "mock_color"),
            now=11.7,
        )
        self.assertEqual(plan.status, PlanStatus.SUCCESS)
        self.assertEqual(self.manager.state, Task3FlowState.TERMINAL_DOCKING)

    def test_center_only_arc_exhaustion_enters_error(self):
        center_only = CoarseDockCenterObservationNED(
            "dock_1", NEDPoint(10.0, 20.0, 0.0), 10.0, True,
        )
        search_plan = self.manager.begin_center_search(
            center_only,
            Pose2D(0.0, 20.0, 0.0),
            arc_config=CircularDockSearchConfig(
                arc_span_rad=0.2,
                waypoint_spacing_m=100.0,
            ),
            now=10.5,
        )
        self.assertEqual(len(search_plan.arc_waypoints), 1)
        self.manager.approach_reached()
        self.assertIsNone(self.manager.search_waypoint_reached())
        self.assertEqual(self.manager.state, Task3FlowState.ERROR)
        self.assertIn("arc exhausted", self.manager.error)

    def test_center_search_handles_inside_on_circle_and_undefined_center(self):
        center = CoarseDockCenterObservationNED(
            "dock_1", NEDPoint(10.0, 20.0, 0.0), 10.0, True,
        )
        config = CoarseDockSearchConfig(
            dock_length_m=2.0,
            dock_width_m=2.0,
            vessel_envelope_radius_m=0.0,
            dimension_measurement_margin_m=0.0,
            stand_off_distance_m=1.0,
            position_uncertainty_m=0.0,
            circle_reached_tolerance_m=0.1,
        )

        inside_manager = Task3FlowManager(
            DockPlanner(PlannerConfig(4, 2)),
            target_color="GREEN",
            bay_width=3,
            bay_depth=5,
        )
        inside_plan = inside_manager.begin_center_search(
            center,
            Pose2D(9.0, 20.0, 0.0),
            approach_config=config,
        )
        self.assertIsNotNone(inside_plan)
        self.assertEqual(
            inside_manager.coarse_circle_position,
            CoarseDockCirclePosition.INSIDE_MINIMUM_CLEARANCE,
        )
        self.assertEqual(inside_manager.state, Task3FlowState.APPROACH_DOCK)
        self.assertLess(inside_plan.staging_pose.x, 9.0)

        radius = config.search_radius_m
        on_circle_manager = Task3FlowManager(
            DockPlanner(PlannerConfig(4, 2)),
            target_color="GREEN",
            bay_width=3,
            bay_depth=5,
        )
        on_circle_manager.begin_center_search(
            center,
            Pose2D(10.0 - radius, 20.0, 0.0),
            approach_config=config,
        )
        self.assertEqual(
            on_circle_manager.coarse_circle_position,
            CoarseDockCirclePosition.ON_CIRCLE,
        )
        self.assertEqual(on_circle_manager.state, Task3FlowState.SEARCH_DOCK)

        center_manager = Task3FlowManager(
            DockPlanner(PlannerConfig(4, 2)),
            target_color="GREEN",
            bay_width=3,
            bay_depth=5,
        )
        self.assertIsNone(center_manager.begin_center_search(
            center,
            Pose2D(10.0, 20.0, 0.0),
            approach_config=config,
        ))
        self.assertEqual(center_manager.state, Task3FlowState.ERROR)
        self.assertIn("escape direction", center_manager.error)


if __name__ == "__main__":
    unittest.main()
