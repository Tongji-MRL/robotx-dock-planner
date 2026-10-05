import math
import unittest

from dock_planner import (
    CircularDockSearchConfig,
    CoarseDockCenterObservationNED,
    CoarseDockSearchConfig,
    NEDPoint,
    Pose2D,
    plan_circular_dock_search,
)


class CircularDockSearchTests(unittest.TestCase):
    def setUp(self):
        self.coarse = CoarseDockCenterObservationNED(
            "dock_arc",
            NEDPoint(10.0, 0.0, 0.0),
            timestamp=5.0,
            valid=True,
            source="mock_uav",
        )
        self.approach = CoarseDockSearchConfig(
            dock_length_m=2.0,
            dock_width_m=2.0,
            stand_off_distance_m=1.0,
            position_uncertainty_m=0.0,
        )

    def test_plan_starts_on_near_edge_and_samples_clockwise_arc(self):
        arc = CircularDockSearchConfig(
            arc_span_rad=math.pi / 2,
            waypoint_spacing_m=0.75,
            clockwise=True,
        )
        plan = plan_circular_dock_search(
            Pose2D(0.0, 0.0, 0.0), self.coarse, self.approach, arc,
        )
        radius = self.approach.search_radius_m
        self.assertAlmostEqual(plan.staging_pose.x, 10.0 - radius)
        self.assertAlmostEqual(plan.staging_pose.y, 0.0)
        self.assertTrue(plan.arc_waypoints)
        for waypoint in plan.arc_waypoints:
            self.assertAlmostEqual(
                math.hypot(waypoint.x - 10.0, waypoint.y), radius,
            )
        endpoint = plan.arc_waypoints[-1]
        self.assertAlmostEqual(endpoint.x, 10.0, places=7)
        self.assertAlmostEqual(endpoint.y, -radius, places=7)
        self.assertAlmostEqual(endpoint.yaw, 0.0, places=7)

    def test_counterclockwise_arc_uses_opposite_side_and_tangent(self):
        plan = plan_circular_dock_search(
            Pose2D(0.0, 0.0, 0.0),
            self.coarse,
            self.approach,
            CircularDockSearchConfig(
                arc_span_rad=math.pi / 2,
                waypoint_spacing_m=100.0,
                clockwise=False,
            ),
        )
        endpoint = plan.arc_waypoints[-1]
        self.assertAlmostEqual(endpoint.x, 10.0, places=7)
        self.assertAlmostEqual(endpoint.y, self.approach.search_radius_m, places=7)
        self.assertAlmostEqual(abs(endpoint.yaw), 0.0, places=7)

    def test_config_rejects_invalid_arc_policy(self):
        with self.assertRaises(ValueError):
            CircularDockSearchConfig(arc_span_rad=0.0)
        with self.assertRaises(ValueError):
            CircularDockSearchConfig(arc_span_rad=2 * math.pi + 0.1)
        with self.assertRaises(ValueError):
            CircularDockSearchConfig(waypoint_spacing_m=0.0)

    def test_plan_rejects_invalid_center_observation(self):
        invalid = CoarseDockCenterObservationNED(
            "dock_arc", NEDPoint(10.0, 0.0, 0.0), 5.0, False,
        )
        with self.assertRaises(ValueError):
            plan_circular_dock_search(
                Pose2D(0.0, 0.0, 0.0), invalid, self.approach,
            )


if __name__ == "__main__":
    unittest.main()
