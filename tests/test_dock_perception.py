import math
import unittest

import numpy as np

from dock_planner import (
    CoarseDockCenterObservationNED,
    CoarseDockInfo,
    DockPerceptionConfig,
    NEDPoint,
    NumpyDockPerception,
    SyntheticDockConfig,
    generate_synthetic_dock_point_cloud,
)


def _angle_error(first, second):
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


class DockPerceptionPrototypeTests(unittest.TestCase):
    def setUp(self):
        self.coarse = CoarseDockInfo(
            "dock_synthetic",
            NEDPoint(0.0, 0.0, 0.0),
            (
                NEDPoint(-4.0, -4.0, 0.0),
                NEDPoint(-4.0, 4.0, 0.0),
                NEDPoint(4.0, 4.0, 0.0),
                NEDPoint(4.0, -4.0, 0.0),
            ),
            10.0,
            True,
        )
        self.detector = NumpyDockPerception()

    def _best_for_heading(self, observations, heading):
        self.assertTrue(observations)
        return min(observations, key=lambda item: _angle_error(item.entrance_heading_rad, heading))

    def test_front_facing_and_nx4_intensity_is_ignored(self):
        cloud = generate_synthetic_dock_point_cloud(
            SyntheticDockConfig(heading_rad=0.0, include_intensity=True)
        )
        result = self.detector.detect(cloud, self.coarse, timestamp=12.5)
        best = self._best_for_heading(result.entrance_observations, 0.0)
        self.assertAlmostEqual(best.entrance_center_north_m, -3.0, delta=0.15)
        self.assertAlmostEqual(best.entrance_center_east_m, 0.0, delta=0.15)
        self.assertAlmostEqual(best.entrance_heading_rad, 0.0, delta=0.05)
        self.assertEqual(best.timestamp, 12.5)
        self.assertEqual(best.dock_id, "dock_synthetic")
        self.assertGreater(best.confidence, 0.2)

    def test_oblique_view_estimates_non_axis_aligned_heading(self):
        heading = 0.63
        cloud = generate_synthetic_dock_point_cloud(
            SyntheticDockConfig(heading_rad=heading)
        )
        result = self.detector.detect(cloud, self.coarse)
        best = self._best_for_heading(result.entrance_observations, heading)
        self.assertLess(_angle_error(best.entrance_heading_rad, heading), 0.08)

    def test_noise_and_dropout_keep_a_candidate(self):
        cloud = generate_synthetic_dock_point_cloud(
            SyntheticDockConfig(
                heading_rad=-0.45,
                noise_std_m=0.06,
                dropout_probability=0.25,
                seed=22,
            )
        )
        result = self.detector.detect(cloud, self.coarse)
        best = self._best_for_heading(result.entrance_observations, -0.45)
        self.assertLess(_angle_error(best.entrance_heading_rad, -0.45), 0.12)
        self.assertGreater(best.confidence, 0.2)

    def test_partial_occlusion_keeps_the_visible_structure_pair(self):
        cloud = generate_synthetic_dock_point_cloud(
            SyntheticDockConfig(
                heading_rad=1.05,
                occlusion_depth_fraction=0.5,
                occlusion_side=0,
            )
        )
        result = self.detector.detect(cloud, self.coarse)
        best = self._best_for_heading(result.entrance_observations, 1.05)
        self.assertLess(_angle_error(best.entrance_heading_rad, 1.05), 0.08)
        self.assertGreaterEqual(best.confidence, 0.2)

    def test_no_valid_gap_returns_no_observation(self):
        detector = NumpyDockPerception(DockPerceptionConfig(min_gap_m=3.5))
        cloud = generate_synthetic_dock_point_cloud(SyntheticDockConfig(opening_width_m=3.0))
        result = detector.detect(cloud, self.coarse)
        self.assertEqual(result.entrance_observations, ())

    def test_nonfinite_points_and_bad_shape_are_handled(self):
        cloud = generate_synthetic_dock_point_cloud()
        cloud = np.vstack([cloud, [math.nan, 0.0, 0.0], [math.inf, 0.0, 0.0]])
        result = self.detector.detect_with_debug(cloud, self.coarse)
        self.assertTrue(np.isfinite(result.filtered_points).all())
        with self.assertRaises(ValueError):
            self.detector.detect(np.zeros((3, 2)), self.coarse)

    def test_center_only_coarse_observation_is_supported(self):
        center_only = CoarseDockCenterObservationNED(
            "dock_synthetic", NEDPoint(0.0, 0.0, 0.0), 10.0, True,
        )
        result = self.detector.detect(
            generate_synthetic_dock_point_cloud(), center_only, timestamp=12.5,
        )
        self.assertTrue(result.entrance_observations)
        self.assertEqual(result.dock_id, "dock_synthetic")


if __name__ == "__main__":
    unittest.main()
