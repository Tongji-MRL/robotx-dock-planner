import math
import unittest

import numpy as np

from dock_planner import SyntheticDockConfig, generate_synthetic_dock_point_cloud
from ros2.robotx_perception_common.robotx_perception_common.local_dock_detector import (
    LocalDockDetectorConfig,
    detect_local_entrances,
)


class LocalDockDetectorTests(unittest.TestCase):
    def test_coarse_seed_selects_the_external_entrance(self):
        points = generate_synthetic_dock_point_cloud(
            SyntheticDockConfig(center_north_m=4.0, depth_m=6.0, opening_width_m=3.0)
        )
        candidates = detect_local_entrances(points, coarse_center_xy=(4.0, 0.0))
        self.assertTrue(candidates)
        self.assertGreaterEqual(candidates[0].confidence, 0.70)
        self.assertAlmostEqual(candidates[0].center_x_m, 1.0, delta=0.5)
        self.assertAlmostEqual(candidates[0].center_y_m, 0.0, delta=0.25)
        self.assertLess(abs(candidates[0].heading_rad), 0.15)

    def test_low_dock_survives_dropout_water_returns_and_side_clutter(self):
        rng = np.random.default_rng(12)
        parts = []
        for y_m in (-1.25, 1.25):
            x_m = np.linspace(1.0, 6.5, 90)
            for z_m in (-0.22, -0.08, 0.04):
                keep = rng.random(len(x_m)) > 0.30
                parts.append(np.column_stack((
                    x_m[keep] + rng.normal(0.0, 0.018, keep.sum()),
                    np.full(keep.sum(), y_m) + rng.normal(0.0, 0.018, keep.sum()),
                    np.full(keep.sum(), z_m) + rng.normal(0.0, 0.015, keep.sum()),
                )))
        water = np.column_stack((
            rng.uniform(0.9, 7.0, 100),
            rng.uniform(-3.0, 3.0, 100),
            rng.normal(-0.28, 0.025, 100),
        ))
        side_clutter = []
        for y_m in (-3.8, 3.8):
            x_m = np.linspace(0.8, 7.0, 50)
            side_clutter.append(np.column_stack((
                x_m,
                np.full(50, y_m),
                rng.normal(0.10, 0.03, 50),
            )))

        candidates = detect_local_entrances(
            np.vstack(parts + [water] + side_clutter),
            coarse_center_xy=(4.0, 0.0),
        )

        self.assertTrue(candidates)
        self.assertAlmostEqual(candidates[0].center_x_m, 1.0, delta=0.20)
        self.assertAlmostEqual(candidates[0].center_y_m, 0.0, delta=0.20)
        self.assertGreaterEqual(candidates[0].confidence, 0.70)

    def test_oblique_dock_uses_coarse_direction_instead_of_boat_axis(self):
        heading = math.radians(35.0)
        coarse = (4.0 * math.cos(heading), 4.0 * math.sin(heading))
        points = generate_synthetic_dock_point_cloud(SyntheticDockConfig(
            center_north_m=coarse[0],
            center_east_m=coarse[1],
            heading_rad=heading,
            depth_m=6.0,
            opening_width_m=3.0,
            noise_std_m=0.02,
            dropout_probability=0.25,
        ))

        candidates = detect_local_entrances(points, coarse_center_xy=coarse)

        self.assertTrue(candidates)
        self.assertAlmostEqual(candidates[0].center_x_m, math.cos(heading), delta=0.25)
        self.assertAlmostEqual(candidates[0].center_y_m, math.sin(heading), delta=0.25)
        self.assertAlmostEqual(candidates[0].heading_rad, heading, delta=0.15)

    def test_rejects_an_entrance_with_a_dense_blockage(self):
        points = generate_synthetic_dock_point_cloud(
            SyntheticDockConfig(center_north_m=4.0, depth_m=6.0, opening_width_m=3.0)
        )
        blockage_x = np.linspace(0.85, 1.15, 8)
        blockage_y = np.linspace(-1.2, 1.2, 40)
        xx, yy = np.meshgrid(blockage_x, blockage_y)
        blockage = np.column_stack((
            xx.ravel(), yy.ravel(), np.zeros(xx.size),
        ))
        config = LocalDockDetectorConfig(max_gap_interior_points=2)

        candidates = detect_local_entrances(
            np.vstack((points, blockage)),
            coarse_center_xy=(4.0, 0.0),
            config=config,
        )

        self.assertFalse(any(candidate.center_x_m < 1.5 for candidate in candidates))

    def test_finds_gap_from_low_transverse_front_faces(self):
        rng = np.random.default_rng(21)
        parts = []
        for y_start, y_end in ((-3.0, -1.1), (1.1, 3.0)):
            y_m = np.linspace(y_start, y_end, 55)
            for z_m in (-0.20, -0.06, 0.08):
                keep = rng.random(len(y_m)) > 0.25
                parts.append(np.column_stack((
                    np.full(keep.sum(), 1.25) + rng.normal(0.0, 0.02, keep.sum()),
                    y_m[keep] + rng.normal(0.0, 0.015, keep.sum()),
                    np.full(keep.sum(), z_m) + rng.normal(0.0, 0.015, keep.sum()),
                )))

        candidates = detect_local_entrances(
            np.vstack(parts),
            coarse_center_xy=(4.0, 0.0),
        )

        self.assertTrue(candidates)
        self.assertAlmostEqual(candidates[0].center_x_m, 1.25, delta=0.20)
        self.assertAlmostEqual(candidates[0].center_y_m, 0.0, delta=0.20)
        self.assertAlmostEqual(candidates[0].heading_rad, 0.0, delta=0.15)
        self.assertGreaterEqual(candidates[0].confidence, 0.70)

    def test_rejects_empty_cloud(self):
        self.assertEqual(detect_local_entrances([]), ())


if __name__ == "__main__":
    unittest.main()
