import unittest

from dock_planner import (
    BayColorObservation,
    CoarseDockInfo,
    DataFreshnessConfig,
    DockEntranceObservationNED,
    DockPerceptionResult,
    DockPlanner,
    NEDPoint,
    PlannerConfig,
    PlanStatus,
    Pose2D,
    Task3FlowManager,
    Task3FlowState,
)


class Task3IntegrationFlowTests(unittest.TestCase):
    def setUp(self):
        self.coarse = CoarseDockInfo(
            "dock_mock",
            NEDPoint(10.0, 20.0, 0.0),
            (
                NEDPoint(8.0, 18.0, 0.0),
                NEDPoint(12.0, 18.0, 0.0),
                NEDPoint(12.0, 22.0, 0.0),
                NEDPoint(8.0, 22.0, 0.0),
            ),
            timestamp=10.0,
            valid=True,
        )
        self.freshness = DataFreshnessConfig(
            coarse_dock_timeout_s=5.0,
            perception_timeout_s=2.0,
        )
        self.manager = Task3FlowManager(
            DockPlanner(PlannerConfig(approach_distance=4.0, stop_distance_inside=2.0)),
            target_color="GREEN",
            bay_width=3.0,
            bay_depth=5.0,
            freshness=self.freshness,
        )

    def _result(self, timestamp=12.0, valid_until=14.0, observations=None):
        if observations is None:
            observations = (
                DockEntranceObservationNED(
                    9.0, 20.0, 1.5707963267948966, 0.9, timestamp,
                    "mock_perception", dock_id="dock_mock",
                ),
            )
        return DockPerceptionResult(
            dock_id="dock_mock",
            entrance_observations=observations,
            timestamp=timestamp,
            valid_until=valid_until,
            source="mock_adapter",
        )

    def test_mock_coarse_perception_and_planner_chain(self):
        self.manager.begin(self.coarse, now=12.0)
        self.assertEqual(self.manager.state, Task3FlowState.APPROACH_DOCK)
        self.manager.approach_reached()
        accepted = self.manager.accept_perception_result(
            self._result(), Pose2D(0.0, 0.0, 0.0), now=12.5
        )
        self.assertTrue(accepted)
        self.assertEqual(self.manager.state, Task3FlowState.CHECK_BAY_COLOR)
        plan = self.manager.accept_color_observation(
            BayColorObservation("GREEN", 0.95, 12.5, "mock_color"), now=12.6
        )
        self.assertIsNotNone(plan)
        self.assertEqual(plan.status, PlanStatus.SUCCESS)
        self.assertEqual(self.manager.state, Task3FlowState.TERMINAL_DOCKING)
        self.assertEqual(plan.waypoints[2], Pose2D(9.0, 20.0, 1.5707963267948966))

    def test_stale_coarse_dock_fails_before_approach(self):
        self.manager.begin(self.coarse, now=20.1)
        self.assertEqual(self.manager.state, Task3FlowState.ERROR)
        self.assertIn("stale", self.manager.error)

    def test_stale_perception_result_fails_search(self):
        self.manager.begin(self.coarse, now=12.0)
        self.manager.approach_reached()
        for _ in range(3):
            accepted = self.manager.accept_perception_result(
                self._result(timestamp=10.0, valid_until=20.0),
                Pose2D(0.0, 0.0, 0.0),
                now=13.0,
            )
            self.assertFalse(accepted)
        self.assertEqual(self.manager.state, Task3FlowState.ERROR)
        self.assertIn("stale", self.manager.error)

    def test_result_valid_until_expiry_fails_search(self):
        self.manager.begin(self.coarse, now=12.0)
        self.manager.approach_reached()
        for _ in range(3):
            accepted = self.manager.accept_perception_result(
                self._result(timestamp=12.0, valid_until=12.4),
                Pose2D(0.0, 0.0, 0.0),
                now=12.5,
            )
            self.assertFalse(accepted)
        self.assertEqual(self.manager.state, Task3FlowState.ERROR)
        self.assertIn("stale", self.manager.error)

    def test_fresh_result_without_candidates_keeps_searching(self):
        self.manager.begin(self.coarse, now=12.0)
        self.manager.approach_reached()
        self.assertFalse(self.manager.accept_perception_result(
            self._result(observations=()), Pose2D(0.0, 0.0, 0.0), now=12.5
        ))
        self.assertEqual(self.manager.state, Task3FlowState.SEARCH_DOCK)


if __name__ == "__main__":
    unittest.main()
