import math
import unittest

from dock_planner import DockPlanner, PlannerConfig, PlanStatus, Pose2D
from dock_planner.vrx_reference import vrx_dock_2022_bay_to_dockspec as adapt


class VRXReferenceTests(unittest.TestCase):
    def test_zero_yaw(self):
        for index, y in enumerate((3, 9, 15), 1):
            dock = adapt(0, 0, 0, index)
            self.assertEqual((dock.entrance_center_x, dock.entrance_center_y), (-1, y))
            self.assertEqual(dock.entrance_heading, 0)
            self.assertEqual((dock.bay_width, dock.bay_depth), (4, 6))

    def test_ninety_degrees(self):
        for index, y in enumerate((3, 9, 15), 1):
            dock = adapt(0, 0, math.pi / 2, index)
            self.assertAlmostEqual(dock.entrance_center_x, -y)
            self.assertAlmostEqual(dock.entrance_center_y, -1)
            self.assertAlmostEqual(dock.entrance_heading, math.pi / 2)

    def test_one_eighty_degrees(self):
        for index, y in enumerate((3, 9, 15), 1):
            dock = adapt(0, 0, math.pi, index)
            self.assertAlmostEqual(dock.entrance_center_x, 1)
            self.assertAlmostEqual(dock.entrance_center_y, -y)
            self.assertAlmostEqual(dock.entrance_heading, -math.pi)

    def test_translation(self):
        for index in (1, 2, 3):
            base = adapt(0, 0, 0.73, index)
            shifted = adapt(12.3, -41.7, 0.73, index)
            self.assertAlmostEqual(shifted.entrance_center_x - base.entrance_center_x, 12.3)
            self.assertAlmostEqual(shifted.entrance_center_y - base.entrance_center_y, -41.7)
            self.assertEqual(shifted.entrance_heading, base.entrance_heading)

    def test_relative_positions(self):
        for yaw in (0, math.pi / 2, math.pi, -0.73):
            bays = [adapt(23, -19, yaw, i) for i in (1, 2, 3)]
            for a, b in zip(bays, bays[1:]):
                dx = b.entrance_center_x - a.entrance_center_x
                dy = b.entrance_center_y - a.entrance_center_y
                self.assertAlmostEqual(math.hypot(dx, dy), 6)
                self.assertAlmostEqual(dx * math.cos(yaw) + dy * math.sin(yaw), 0)
                self.assertAlmostEqual(-dx * math.sin(yaw) + dy * math.cos(yaw), 6)

    def test_heading_wrap_and_rotation(self):
        for yaw in (-5 * math.pi, -2.8, 0.73, 2 * math.pi + 0.4, 5 * math.pi):
            for index in (1, 2, 3):
                h = adapt(0, 0, yaw, index).entrance_heading
                self.assertGreaterEqual(h, -math.pi)
                self.assertLess(h, math.pi)
                self.assertAlmostEqual(math.cos(h), math.cos(yaw))
                self.assertAlmostEqual(math.sin(h), math.sin(yaw))

    def test_official_world_coordinates(self):
        expected = ((-483.064715, 190.779436),
                    (-488.113541, 194.021250),
                    (-493.162367, 197.263064))
        for index, (x, y) in enumerate(expected, 1):
            dock = adapt(-480, 190, 1.0, index)
            self.assertAlmostEqual(dock.entrance_center_x, x, delta=1e-6)
            self.assertAlmostEqual(dock.entrance_center_y, y, delta=1e-6)
            self.assertAlmostEqual(dock.entrance_heading, 1.0)

    def test_planner_order_and_local_free_space(self):
        planner = DockPlanner(PlannerConfig(4, 2))
        for yaw in (0, math.pi / 2, math.pi, 1.0, -2.3):
            c, s = math.cos(yaw), math.sin(yaw)
            for index, local_y in enumerate((3, 9, 15), 1):
                with self.subTest(yaw=yaw, bay=index):
                    dock = adapt(-480, 190, yaw, index)
                    result = planner.plan(Pose2D(-500, 180, 0), dock)
                    self.assertEqual(result.status, PlanStatus.SUCCESS)
                    self.assertEqual(result.approach_pose, result.waypoints[1])
                    for pose, expected_x in zip(result.waypoints[1:], (-5, -1, 1)):
                        dx, dy = pose.x + 480, pose.y - 190
                        self.assertAlmostEqual(c * dx + s * dy, expected_x)
                        self.assertAlmostEqual(-s * dx + c * dy, local_y)
                        self.assertAlmostEqual(math.cos(pose.yaw), c)
                        self.assertAlmostEqual(math.sin(pose.yaw), s)
                    # Stop reference point lies between entrance (-1) and back wall (5).
                    stop = result.waypoints[-1]
                    sx = c * (stop.x + 480) + s * (stop.y - 190)
                    self.assertTrue(-1 < sx < 5)

    def test_invalid_inputs(self):
        for index in (0, 4, -1, True, 1.5, "1"):
            with self.subTest(index=index), self.assertRaises(ValueError):
                adapt(0, 0, 0, index)
        for args in ((math.nan, 0, 0), (0, math.inf, 0), (0, 0, math.nan), (0, 0, True)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                adapt(*args, 1)
