import math
import unittest

from dock_planner import DockPlanner, DockSpec, PlannerConfig, PlanStatus, Pose2D


class DockPlannerTests(unittest.TestCase):
    def setUp(self):
        self.planner = DockPlanner(PlannerConfig(4.0, 2.0))
        self.usv = Pose2D(-8.0, -3.0, 0.7)

    def assert_pose(self, pose, x, y, yaw):
        self.assertAlmostEqual(pose.x, x)
        self.assertAlmostEqual(pose.y, y)
        self.assertAlmostEqual(pose.yaw, yaw)

    def test_translation(self):
        base = self.planner.plan(self.usv, DockSpec(0, 0, 0, 3, 5))
        shifted_usv = Pose2D(self.usv.x + 12, self.usv.y - 7, self.usv.yaw)
        moved = self.planner.plan(shifted_usv, DockSpec(12, -7, 0, 3, 5))
        self.assertEqual(moved.status, PlanStatus.SUCCESS)
        for p, q in zip(base.waypoints, moved.waypoints):
            self.assert_pose(q, p.x + 12, p.y - 7, p.yaw)

    def test_rotation_90_degrees(self):
        result = self.planner.plan(self.usv, DockSpec(10, 20, math.pi / 2, 3, 5))
        self.assertEqual(result.status, PlanStatus.SUCCESS)
        self.assert_pose(result.approach_pose, 10, 16, math.pi / 2)
        self.assert_pose(result.waypoints[2], 10, 20, math.pi / 2)
        self.assert_pose(result.waypoints[3], 10, 22, math.pi / 2)

    def test_rotation_180_degrees(self):
        result = self.planner.plan(self.usv, DockSpec(10, 20, math.pi, 3, 5))
        self.assertEqual(result.status, PlanStatus.SUCCESS)
        self.assert_pose(result.approach_pose, 14, 20, math.pi)
        self.assert_pose(result.waypoints[2], 10, 20, math.pi)
        self.assert_pose(result.waypoints[3], 8, 20, math.pi)

    def test_arbitrary_rigid_transform(self):
        angle = 0.63
        c, s = math.cos(angle), math.sin(angle)

        def transform(p):
            return Pose2D(c * p.x - s * p.y + 7, s * p.x + c * p.y - 11, p.yaw + angle)

        dock = DockSpec(3, 8, -0.4, 3, 5)
        base = self.planner.plan(self.usv, dock)
        entrance = transform(Pose2D(3, 8, -0.4))
        moved = self.planner.plan(transform(self.usv), DockSpec(
            entrance.x, entrance.y, entrance.yaw, 3, 5))
        self.assertEqual(moved.status, PlanStatus.SUCCESS)
        for p, q in zip(base.waypoints, moved.waypoints):
            expected = transform(p)
            self.assert_pose(q, expected.x, expected.y, expected.yaw)

    def test_different_usv_starts(self):
        dock = DockSpec(10, 20, 0, 3, 5)
        for usv in (self.usv, Pose2D(100, -50, -2), Pose2D(6, 20, 0)):
            with self.subTest(usv=usv):
                result = self.planner.plan(usv, dock)
                self.assertEqual(result.status, PlanStatus.SUCCESS)
                self.assertEqual(len(result.waypoints), 4)
                self.assertEqual(result.waypoints[0], usv)
                self.assert_pose(result.approach_pose, 6, 20, 0)
                self.assert_pose(result.waypoints[-1], 12, 20, 0)

    def test_invalid_bay_dimensions(self):
        for width, depth in ((0, 5), (-1, 5), (3, 0), (3, -1),
                             (math.nan, 5), (3, math.inf)):
            with self.subTest(width=width, depth=depth):
                self.assert_invalid(self.planner.plan(self.usv, DockSpec(0, 0, 0, width, depth)))

    def assert_invalid(self, result):
        self.assertEqual(result.status, PlanStatus.INVALID_INPUT)
        self.assertIsNone(result.approach_pose)
        self.assertEqual(result.waypoints, [])
        self.assertTrue(result.reason)

    def test_invalid_distances(self):
        for approach, stop in ((0, 2), (-1, 2), (4, 0), (4, -1),
                               (4, 5), (4, 6), (math.inf, 2)):
            with self.subTest(approach=approach, stop=stop):
                result = DockPlanner(PlannerConfig(approach, stop)).plan(
                    self.usv, DockSpec(0, 0, 0, 3, 5))
                self.assert_invalid(result)

    def test_nonfinite_pose_or_heading(self):
        self.assert_invalid(self.planner.plan(Pose2D(math.nan, 0, 0), DockSpec(0, 0, 0, 3, 5)))
        self.assert_invalid(self.planner.plan(self.usv, DockSpec(0, 0, math.inf, 3, 5)))

    def test_optional_inputs(self):
        dock = DockSpec(0, 0, 0, 3, 5, confidence=0.8, dock_id="candidate-a")
        for obstacles in (None, [], ()):
            self.assertEqual(self.planner.plan(self.usv, dock, obstacles).status, PlanStatus.SUCCESS)
        blocked = self.planner.plan(self.usv, dock, obstacles=[object()])
        self.assertEqual(blocked.status, PlanStatus.UNSUPPORTED_INPUT)
        self.assertEqual(blocked.waypoints, [])


if __name__ == "__main__":
    unittest.main()
