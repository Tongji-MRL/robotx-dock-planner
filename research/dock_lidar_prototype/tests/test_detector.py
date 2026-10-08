import math
import unittest

import numpy as np

from dock_lidar_prototype import (
    DockLidarConfig,
    SyntheticDockConfig,
    detect_entrance,
    detect_entrance_debug,
    generate_synthetic_dock_point_cloud,
)


class DockEntrancePrototypeTests(unittest.TestCase):
    def assert_heading_close(self, actual, expected, tolerance=0.12):
        error = math.atan2(math.sin(actual - expected), math.cos(actual - expected))
        self.assertLessEqual(abs(error), tolerance)

    def test_front_facing_dock(self):
        cloud = generate_synthetic_dock_point_cloud(SyntheticDockConfig(seed=1))
        result = detect_entrance(cloud, (0.0, 0.0))
        self.assertIsNotNone(result)
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.center_x, -2.0, delta=0.16)
        self.assertAlmostEqual(result.center_y, 0.0, delta=0.16)
        self.assertAlmostEqual(result.width_m, 2.0, delta=0.18)
        self.assert_heading_close(result.heading_rad, 0.0)

    def test_oblique_dock(self):
        heading = 0.53
        cloud = generate_synthetic_dock_point_cloud(SyntheticDockConfig(heading_rad=heading, seed=2))
        result = detect_entrance(cloud, (0.0, 0.0))
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.center_x, -2.0 * math.cos(heading), delta=0.20)
        self.assertAlmostEqual(result.center_y, -2.0 * math.sin(heading), delta=0.20)
        self.assert_heading_close(result.heading_rad, heading, 0.14)

    def test_noise_and_dropout(self):
        cloud = generate_synthetic_dock_point_cloud(SyntheticDockConfig(
            noise_std_m=0.035, dropout_probability=0.22, outlier_count=10, seed=3,
        ))
        result = detect_entrance(cloud, (0.0, 0.0))
        self.assertIsNotNone(result)
        self.assertTrue(result.valid)
        self.assertGreater(result.confidence, 0.45)

    def test_no_valid_entrance_when_gap_is_too_wide(self):
        cloud = generate_synthetic_dock_point_cloud(SyntheticDockConfig(entrance_width_m=4.5, seed=4))
        result = detect_entrance(cloud, (0.0, 0.0))
        self.assertIsNone(result)

    def test_no_valid_entrance_when_gap_is_too_narrow(self):
        cloud = generate_synthetic_dock_point_cloud(SyntheticDockConfig(entrance_width_m=0.45, seed=5))
        result = detect_entrance(cloud, (0.0, 0.0))
        self.assertIsNone(result)

    def test_no_valid_entrance_for_unrelated_points(self):
        rng = np.random.default_rng(6)
        points = np.column_stack((rng.uniform(-4, 4, 300), rng.uniform(-4, 4, 300), np.zeros(300)))
        result = detect_entrance(points, (0.0, 0.0))
        self.assertIsNone(result)

    def test_multiple_candidates_are_ranked(self):
        primary = generate_synthetic_dock_point_cloud(SyntheticDockConfig(seed=7))
        decoy = generate_synthetic_dock_point_cloud(SyntheticDockConfig(
            length_m=2.4, entrance_width_m=1.2, center_x=0.0, center_y=4.8, heading_rad=math.pi, seed=8,
        ))
        debug = detect_entrance_debug(np.vstack((primary, decoy)), (0.0, 0.0))
        self.assertGreaterEqual(len(debug.candidates), 2)
        self.assertIsNotNone(debug.selected)
        self.assertAlmostEqual(debug.selected.width_m, 2.0, delta=0.2)

    def test_nonfinite_and_height_points_are_filtered(self):
        cloud = generate_synthetic_dock_point_cloud(SyntheticDockConfig(seed=9))
        extra = np.asarray([[np.nan, 0, 0], [0, np.inf, 0], [0, 0, 2.0]])
        debug = detect_entrance_debug(np.vstack((cloud, extra)), (0.0, 0.0))
        self.assertTrue(np.isfinite(debug.filtered_xy).all())
        self.assertIsNotNone(debug.selected)

    def test_observer_rejects_gap_behind_coarse_center(self):
        real = generate_synthetic_dock_point_cloud(SyntheticDockConfig(
            center_x=4.0, length_m=2.0, entrance_width_m=2.0, seed=10,
        ))
        internal_decoy = generate_synthetic_dock_point_cloud(SyntheticDockConfig(
            center_x=7.0, length_m=2.0, entrance_width_m=2.0,
            heading_rad=math.pi, seed=11,
        ))
        result = detect_entrance(
            np.vstack((real, internal_decoy)),
            (5.0, 0.0),
            observer_origin_xy=(0.0, 0.0),
        )
        self.assertIsNotNone(result)
        self.assertLess(result.center_x, 5.0)
        self.assert_heading_close(result.heading_rad, 0.0)


if __name__ == "__main__":
    unittest.main()
