import math
import unittest

from dock_planner import (
    CoarseDockInfo, DockPlanner, NEDPoint, ObstaclePointNED, PlannerConfig,
    PlanStatus, Pose2D, PreciseEntrancePoseNED, prepare_terminal_plan_input,
)


class IntegrationTests(unittest.TestCase):
    def setUp(self):
        self.corners = (
            NEDPoint(8, 18, 0), NEDPoint(12, 18, 0),
            NEDPoint(12, 22, 0), NEDPoint(8, 22, 0),
        )
        self.coarse = CoarseDockInfo("dock_1", NEDPoint(10, 20, 0), self.corners, 12.5, True)
        self.entrance = PreciseEntrancePoseNED(9, 20, math.pi / 2, 15.0, True, 0.9, "dock_1")

    def test_ned_point_and_coarse_observation(self):
        self.assertEqual(self.coarse.center_ned.north, 10)
        self.assertEqual(self.coarse.center_ned.east, 20)
        self.assertEqual(len(self.coarse.approximate_corners_ned), 4)
        with self.assertRaises(ValueError):
            NEDPoint(math.nan, 0, 0)
        with self.assertRaises(ValueError):
            NEDPoint(0, math.inf, 0)
        with self.assertRaises(ValueError):
            NEDPoint(True, 0, 0)
        with self.assertRaises(ValueError):
            CoarseDockInfo("dock_1", self.coarse.center_ned, self.corners, math.inf, True)

    def test_approximate_corners_count_and_validity(self):
        for corners in (self.corners[:3], self.corners + self.corners[:1],
                        self.corners[:3] + self.corners[:1],
                        self.corners[:3] + ((1, 2, 3),)):
            with self.subTest(corners=corners), self.assertRaises(ValueError):
                CoarseDockInfo("dock_1", self.coarse.center_ned, corners, 12.5, True)
        # Approximate corners need no inferred order or rectangular geometry.
        reordered = CoarseDockInfo("dock_1", self.coarse.center_ned,
                                   list(reversed(self.corners)), 12.5, True)
        self.assertEqual(reordered.approximate_corners_ned, tuple(reversed(self.corners)))

    def test_obstacle_is_center_only(self):
        obstacle = ObstaclePointNED(NEDPoint(2, 3, 0.4))
        self.assertEqual(obstacle.center_ned, NEDPoint(2, 3, 0.4))
        self.assertFalse(hasattr(obstacle, "radius"))
        with self.assertRaises(ValueError):
            ObstaclePointNED((2, 3, 0.4))

    def test_precise_entrance_validity(self):
        invalid_coarse = CoarseDockInfo("dock_1", self.coarse.center_ned,
                                        self.corners, 12.5, False)
        with self.assertRaisesRegex(ValueError, "valid coarse dock"):
            prepare_terminal_plan_input(invalid_coarse, self.entrance, 3, 5)
        with self.assertRaisesRegex(ValueError, "valid precise entrance"):
            prepare_terminal_plan_input(
                self.coarse,
                PreciseEntrancePoseNED(9, 20, 0, 15, False), 3, 5,
            )
        with self.assertRaises(ValueError):
            PreciseEntrancePoseNED(9, 20, 0, 15, True, 1.1)
        with self.assertRaises(ValueError):
            PreciseEntrancePoseNED(9, 20, math.nan, 15, True)
        with self.assertRaises(ValueError):
            prepare_terminal_plan_input(
                self.coarse, PreciseEntrancePoseNED(9, 20, 0, 15, True, dock_id="dock_2"), 3, 5,
            )
        with self.assertRaises(ValueError):
            prepare_terminal_plan_input(self.coarse, self.entrance, 0, 5)

    def test_adapter_uses_precise_entrance_not_coarse_corners(self):
        request = prepare_terminal_plan_input(self.coarse, self.entrance, 3, 5)
        dock = request.dock
        self.assertEqual((dock.entrance_center_x, dock.entrance_center_y), (9, 20))
        self.assertEqual(dock.entrance_heading, math.pi / 2)
        self.assertEqual((dock.bay_width, dock.bay_depth, dock.confidence, dock.dock_id),
                         (3, 5, 0.9, "dock_1"))
        self.assertEqual(request.obstacles, ())
        result = DockPlanner(PlannerConfig(4, 2)).plan(
            Pose2D(0, 0, 0), dock, obstacles=request.obstacles,
        )
        self.assertEqual(result.status, PlanStatus.SUCCESS)
        self.assertEqual(result.waypoints[2], Pose2D(9, 20, math.pi / 2))

    def test_nonempty_obstacles_remain_unsupported(self):
        point = ObstaclePointNED(NEDPoint(5, 6, 0))
        request = prepare_terminal_plan_input(self.coarse, self.entrance, 3, 5, [point])
        self.assertEqual(request.obstacles, (point,))
        result = DockPlanner(PlannerConfig(4, 2)).plan(
            Pose2D(0, 0, 0), request.dock, obstacles=request.obstacles,
        )
        self.assertEqual(result.status, PlanStatus.UNSUPPORTED_INPUT)
        self.assertEqual(result.waypoints, [])


if __name__ == "__main__":
    unittest.main()
