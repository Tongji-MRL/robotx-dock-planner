import math
import unittest

from dock_planner import (
    CoarseDockCenterObservationNED,
    CoarseDockCirclePosition,
    CoarseDockSearchConfig,
    NEDPoint,
    Pose2D,
    classify_coarse_dock_circle_position,
    coarse_dock_staging_pose,
    coarse_dock_search_radius_m,
)


class CoarseDockSearchTests(unittest.TestCase):
    def test_center_only_observation_is_valid_and_fresh(self):
        observation = CoarseDockCenterObservationNED(
            dock_id="dock_1",
            center_ned=NEDPoint(20.0, 35.0, 0.0),
            timestamp=10.0,
            valid=True,
            confidence=0.9,
        )
        self.assertTrue(observation.is_fresh_at(12.0, 5.0))
        self.assertFalse(observation.is_fresh_at(15.1, 5.0))

    def test_center_only_observation_rejects_invalid_values(self):
        with self.assertRaises(ValueError):
            CoarseDockCenterObservationNED(
                "dock_1", NEDPoint(20.0, 35.0, 0.0), math.nan, True,
            )
        with self.assertRaises(ValueError):
            CoarseDockCenterObservationNED(
                "dock_1", NEDPoint(20.0, 35.0, 0.0), 10.0, True, confidence=1.1,
            )
        with self.assertRaises(ValueError):
            CoarseDockCenterObservationNED(
                "dock_1", NEDPoint(20.0, 35.0, 0.0), 10.0, True, source="",
            )

    def test_search_radius_uses_known_footprint_and_margins(self):
        config = CoarseDockSearchConfig(
            dock_length_m=6.596,
            dock_width_m=3.0,
            stand_off_distance_m=2.0,
            position_uncertainty_m=1.0,
        )
        expected = math.hypot(6.596 / 2.0, 3.0 / 2.0) + 0.92 + 0.10 + 3.0
        self.assertAlmostEqual(config.search_radius_m, expected)
        self.assertAlmostEqual(coarse_dock_search_radius_m(config), expected)

    def test_search_config_rejects_invalid_dimensions(self):
        with self.assertRaises(ValueError):
            CoarseDockSearchConfig(dock_length_m=0.0)
        with self.assertRaises(ValueError):
            CoarseDockSearchConfig(position_uncertainty_m=-1.0)
        with self.assertRaises(TypeError):
            coarse_dock_search_radius_m(object())

    def test_staging_pose_is_on_near_side_of_search_circle(self):
        config = CoarseDockSearchConfig(
            dock_length_m=2.0,
            dock_width_m=2.0,
            stand_off_distance_m=1.0,
            position_uncertainty_m=0.0,
        )
        pose = coarse_dock_staging_pose(
            Pose2D(0.0, 0.0, 0.0), NEDPoint(10.0, 0.0, 0.0), config,
        )
        self.assertAlmostEqual(pose.x, 10.0 - config.search_radius_m)
        self.assertAlmostEqual(pose.y, 0.0)
        self.assertAlmostEqual(pose.yaw, 0.0)

    def test_staging_pose_moves_outward_when_current_pose_is_inside_circle(self):
        config = CoarseDockSearchConfig(
            dock_length_m=2.0,
            dock_width_m=2.0,
            stand_off_distance_m=1.0,
            position_uncertainty_m=0.0,
        )
        pose = coarse_dock_staging_pose(
            Pose2D(9.0, 0.0, 1.0), NEDPoint(10.0, 0.0, 0.0), config,
        )
        self.assertAlmostEqual(pose.x, 10.0 - config.search_radius_m)
        self.assertAlmostEqual(pose.y, 0.0)
        self.assertAlmostEqual(pose.yaw, 0.0)

    def test_circle_position_distinguishes_inside_on_circle_and_outside(self):
        config = CoarseDockSearchConfig(
            dock_length_m=2.0,
            dock_width_m=2.0,
            vessel_envelope_radius_m=0.5,
            dimension_measurement_margin_m=0.1,
            stand_off_distance_m=1.0,
            position_uncertainty_m=0.0,
            circle_reached_tolerance_m=0.2,
        )
        center = NEDPoint(10.0, 0.0, 0.0)
        radius = config.search_radius_m
        self.assertEqual(
            classify_coarse_dock_circle_position(
                Pose2D(10.0 - radius - 1.0, 0.0, 0.0), center, config,
            ),
            CoarseDockCirclePosition.OUTSIDE,
        )
        self.assertEqual(
            classify_coarse_dock_circle_position(
                Pose2D(10.0 - radius + 0.1, 0.0, 0.0), center, config,
            ),
            CoarseDockCirclePosition.ON_CIRCLE,
        )
        self.assertEqual(
            classify_coarse_dock_circle_position(
                Pose2D(10.0 - config.minimum_center_distance_m - 0.2, 0.0, 0.0),
                center,
                config,
            ),
            CoarseDockCirclePosition.INSIDE,
        )
        self.assertEqual(
            classify_coarse_dock_circle_position(
                Pose2D(10.0 - config.minimum_center_distance_m + 0.1, 0.0, 0.0),
                center,
                config,
            ),
            CoarseDockCirclePosition.INSIDE_MINIMUM_CLEARANCE,
        )
        self.assertEqual(
            classify_coarse_dock_circle_position(Pose2D(10.0, 0.0, 0.0), center, config),
            CoarseDockCirclePosition.CENTER_UNDEFINED,
        )

    def test_staging_pose_rejects_coincident_or_invalid_center(self):
        config = CoarseDockSearchConfig()
        with self.assertRaises(ValueError):
            coarse_dock_staging_pose(
                Pose2D(1.0, 2.0, 0.0), NEDPoint(1.0, 2.0, 0.0), config,
            )
        invalid = CoarseDockCenterObservationNED(
            "dock_1", NEDPoint(5.0, 5.0, 0.0), 10.0, False,
        )
        with self.assertRaises(ValueError):
            coarse_dock_staging_pose(Pose2D(0.0, 0.0, 0.0), invalid, config)


if __name__ == "__main__":
    unittest.main()
