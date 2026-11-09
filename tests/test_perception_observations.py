import math
import unittest

from dock_planner import (
    BayColorObservation,
    CoarseDockInfo,
    DockEntranceObservationNED,
    NEDPoint,
    PreciseEntrancePoseNED,
    entrance_observation_to_precise_pose,
    sort_entrances_left_to_right,
)


class PerceptionObservationTests(unittest.TestCase):
    def make_entrance(self, **overrides):
        values = dict(
            entrance_center_north_m=12.0,
            entrance_center_east_m=-4.5,
            entrance_heading_rad=1.2,
            confidence=0.86,
            timestamp=25.0,
            source="mid360_dock_perception",
            dock_id="dock_1",
        )
        values.update(overrides)
        return DockEntranceObservationNED(**values)

    def test_valid_entrance_observation(self):
        observation = self.make_entrance()
        self.assertEqual(observation.entrance_center_north_m, 12.0)
        self.assertEqual(observation.entrance_center_east_m, -4.5)
        self.assertEqual(observation.source, "mid360_dock_perception")
        self.assertTrue(observation.valid)

    def test_entrances_are_ordered_from_dock_left_to_right(self):
        # Facing north into the dock, west/east are viewer-left/viewer-right.
        unordered = (
            self.make_entrance(entrance_center_east_m=2.0, entrance_heading_rad=0.0),
            self.make_entrance(entrance_center_east_m=-2.0, entrance_heading_rad=0.02),
            self.make_entrance(entrance_center_east_m=0.0, entrance_heading_rad=-0.02),
        )
        ordered = sort_entrances_left_to_right(unordered)
        self.assertEqual(
            [item.entrance_center_east_m for item in ordered],
            [-2.0, 0.0, 2.0],
        )

        # Facing east, north/south are viewer-left/viewer-right.
        east_facing = (
            self.make_entrance(entrance_center_north_m=-2.0, entrance_heading_rad=math.pi / 2),
            self.make_entrance(entrance_center_north_m=2.0, entrance_heading_rad=math.pi / 2),
            self.make_entrance(entrance_center_north_m=0.0, entrance_heading_rad=math.pi / 2),
        )
        ordered = sort_entrances_left_to_right(east_facing)
        self.assertEqual(
            [item.entrance_center_north_m for item in ordered],
            [2.0, 0.0, -2.0],
        )

    def test_entrance_observation_rejects_bad_values(self):
        for field, value in (
            ("entrance_center_north_m", math.nan),
            ("entrance_center_east_m", math.inf),
            ("entrance_heading_rad", -math.inf),
            ("confidence", -0.1),
            ("confidence", 1.1),
            ("timestamp", math.nan),
        ):
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                self.make_entrance(**{field: value})

    def test_valid_color_observation(self):
        observation = BayColorObservation(
            color="GREEN",
            confidence=0.93,
            timestamp=26.0,
            source="bay_color_model",
        )
        self.assertEqual((observation.color, observation.confidence), ("GREEN", 0.93))
        self.assertTrue(observation.valid)

    def test_entrance_observation_converts_to_precise_pose(self):
        precise = entrance_observation_to_precise_pose(self.make_entrance())
        self.assertIsInstance(precise, PreciseEntrancePoseNED)
        self.assertEqual(
            (precise.x_ned, precise.y_ned, precise.heading_rad),
            (12.0, -4.5, 1.2),
        )
        self.assertEqual((precise.confidence, precise.timestamp, precise.dock_id), (0.86, 25.0, "dock_1"))

        with self.assertRaises(TypeError):
            entrance_observation_to_precise_pose(
                CoarseDockInfo(
                    "dock_1",
                    NEDPoint(10, 20, 0),
                    (NEDPoint(8, 18, 0), NEDPoint(12, 18, 0),
                     NEDPoint(12, 22, 0), NEDPoint(8, 22, 0)),
                    12.0,
                    True,
                )
            )

        with self.assertRaises(ValueError):
            entrance_observation_to_precise_pose(self.make_entrance(valid=False))


if __name__ == "__main__":
    unittest.main()
