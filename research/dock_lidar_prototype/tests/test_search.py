import math
import unittest

import numpy as np

from dock_lidar_prototype import (
    DockLidarConfig,
    SyntheticDockConfig,
    UnseededSearchConfig,
    generate_synthetic_dock_point_cloud,
    search_entrance,
    search_entrances_debug,
)


class UnseededSearchTests(unittest.TestCase):
    def _config(self, **kwargs):
        detector = DockLidarConfig(
            z_min=-0.25,
            z_max=0.25,
            roi_radius_m=5.0,
            ransac_iterations=120,
        )
        return UnseededSearchConfig(
            detector_config=detector,
            min_range_m=0.5,
            max_range_m=10.0,
            grid_spacing_m=1.25,
            max_grid_centers=128,
            **kwargs,
        )

    def test_finds_dock_without_coarse_center(self):
        cloud = generate_synthetic_dock_point_cloud(SyntheticDockConfig(
            center_x=5.0, heading_rad=0.0, seed=21,
        ))
        result = search_entrance(cloud, self._config())
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.center_x, 3.0, delta=0.35)
        self.assertAlmostEqual(result.center_y, 0.0, delta=0.35)
        self.assertAlmostEqual(result.width_m, 2.0, delta=0.25)

    def test_near_origin_points_are_removed(self):
        dock = generate_synthetic_dock_point_cloud(SyntheticDockConfig(
            center_x=5.0, heading_rad=0.0, seed=22,
        ))
        self_body = np.column_stack((
            np.linspace(-0.4, 0.4, 60),
            np.linspace(-0.2, 0.2, 60),
            np.zeros(60),
        ))
        debug = search_entrances_debug(np.vstack((dock, self_body)), self._config())
        self.assertEqual(debug.removed_near_points, len(self_body))
        self.assertTrue(all(np.linalg.norm(point[:2]) >= 0.5 for point in debug.filtered_xyz))

    def test_empty_or_unrelated_cloud_returns_no_candidate(self):
        rng = np.random.default_rng(23)
        points = np.column_stack((rng.uniform(2, 8, 150), rng.uniform(4, 8, 150), np.zeros(150)))
        debug = search_entrances_debug(points, self._config())
        self.assertIsNone(debug.selected)

    def test_candidates_are_deduplicated_and_ranked(self):
        cloud = generate_synthetic_dock_point_cloud(SyntheticDockConfig(
            center_x=5.0, heading_rad=0.0, seed=24,
        ))
        debug = search_entrances_debug(cloud, self._config())
        self.assertLessEqual(len(debug.candidates), self._config().max_candidates)
        self.assertEqual(
            list(debug.candidates),
            sorted(debug.candidates, key=lambda candidate: candidate.confidence, reverse=True),
        )


if __name__ == "__main__":
    unittest.main()
