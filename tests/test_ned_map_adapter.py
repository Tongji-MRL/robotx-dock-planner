import json
import math
import unittest

from dock_planner import (
    GpsOrigin,
    NEDPoint,
    add_ned_offset,
    semantic_map_to_coarse_docks,
    transform_semantic_map,
    uav_origin_in_usv_ned,
)


class NedMapAdapterTests(unittest.TestCase):
    def setUp(self):
        self.origin = GpsOrigin(31.2304, 121.4737, 10.0)

    def test_same_origins_have_zero_offset(self):
        offset = uav_origin_in_usv_ned(self.origin, self.origin)
        self.assertTrue(all(abs(value) < 1e-9 for value in offset))

    def test_gps_axes_have_expected_ned_signs(self):
        north_origin = GpsOrigin(31.23041, 121.4737, 10.0)
        east_origin = GpsOrigin(31.2304, 121.47371, 10.0)
        higher_origin = GpsOrigin(31.2304, 121.4737, 15.0)
        north = uav_origin_in_usv_ned(north_origin, self.origin)
        east = uav_origin_in_usv_ned(east_origin, self.origin)
        higher = uav_origin_in_usv_ned(higher_origin, self.origin)
        self.assertGreater(north[0], 1.0)
        self.assertAlmostEqual(north[1], 0.0, delta=0.01)
        self.assertGreater(east[1], 0.8)
        self.assertAlmostEqual(east[0], 0.0, delta=0.01)
        self.assertAlmostEqual(higher[2], -5.0, delta=1e-5)

    def test_transformed_map_is_copied_and_labelled(self):
        source = {"objects": [{"type": "dock", "id": 3, "NED_position": [5, 2, -0.5]}]}
        result = transform_semantic_map(json.dumps(source), self.origin, self.origin)
        self.assertEqual(result["objects"][0]["NED_position"], [5.0, 2.0, -0.5])
        self.assertEqual(result["source_coordinate_frame"], "uav_ned")
        self.assertEqual(result["coordinate_frame"], "usv_ned")
        self.assertEqual(source["objects"][0]["NED_position"], [5, 2, -0.5])

    def test_dock_objects_become_existing_coarse_center_contract(self):
        payload = {
            "objects": [
                {"type": "buoy", "id": "b1", "NED_position": [1, 2, 0]},
                {
                    "type": "dock", "id": "dock_7", "NED_position": [5, 2, -0.5],
                    "confidence": 0.8,
                },
            ]
        }
        observations = semantic_map_to_coarse_docks(
            payload, self.origin, self.origin, timestamp=20.0,
        )
        self.assertEqual(len(observations), 1)
        observation = observations[0]
        self.assertEqual(observation.dock_id, "dock_7")
        self.assertEqual(observation.center_ned, NEDPoint(5.0, 2.0, -0.5))
        self.assertEqual(observation.confidence, 0.8)
        self.assertTrue(observation.is_fresh_at(21.0, 2.0))

    def test_live_map_updates_reuse_fixed_origin_offset(self):
        uav_origin = GpsOrigin(31.23041, 121.47371, 10.0)
        first = semantic_map_to_coarse_docks(
            {"objects": [{"type": "dock", "id": 1, "NED_position": [5, 2, 0]}]},
            uav_origin, self.origin, timestamp=1.0,
        )[0]
        second = semantic_map_to_coarse_docks(
            {"objects": [{"type": "dock", "id": 1, "NED_position": [6, 4, 0]}]},
            uav_origin, self.origin, timestamp=2.0,
        )[0]
        self.assertAlmostEqual(second.center_ned.north - first.center_ned.north, 1.0)
        self.assertAlmostEqual(second.center_ned.east - first.center_ned.east, 2.0)

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            GpsOrigin(math.nan, 121.0, 0.0)
        with self.assertRaises(ValueError):
            add_ned_offset([1, 2], (0.0, 0.0, 0.0))
        with self.assertRaises(ValueError):
            transform_semantic_map({"objects": [{"type": "dock"}]}, self.origin, self.origin)
        with self.assertRaisesRegex(ValueError, "missing id"):
            semantic_map_to_coarse_docks(
                {"objects": [{"type": "dock", "NED_position": [1, 2, 0]}]},
                self.origin, self.origin, timestamp=1.0,
            )


if __name__ == "__main__":
    unittest.main()
