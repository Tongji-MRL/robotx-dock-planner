import json
import math
import unittest

from usv_task3_executor.semantic_map_adapter import parse_semantic_map_dock


def semantic_map(*objects):
    return json.dumps({"objects": list(objects)})


class SemanticMapDockParserTest(unittest.TestCase):
    def test_selects_dock_id_zero(self):
        result = parse_semantic_map_dock(
            semantic_map(
                {"id": 0, "type": "dock", "NED_position": [1, 2, 3]},
                {"id": 1, "type": "dock", "NED_position": [4, 5, 6]},
            ),
            dock_id=0,
            timestamp=12.5,
        )

        self.assertEqual(result["dock_id"], "0")
        self.assertEqual(result["center_ned"], {"north": 1.0, "east": 2.0, "down": 3.0})
        self.assertEqual(result["timestamp"], 12.5)
        self.assertTrue(result["valid"])
        self.assertEqual(result["source"], "semantic_map_mock")
        self.assertTrue(result["mock"])
        self.assertEqual(result["frame_id"], "mock_ned")
        self.assertNotIn("confidence", result)

    def test_selects_configured_dock_id(self):
        result = parse_semantic_map_dock(
            semantic_map(
                {"id": 0, "type": "dock", "NED_position": [1, 2, 3]},
                {"id": 7, "type": "dock", "NED_position": [4, 5, 6]},
            ),
            dock_id=7,
            timestamp=3.0,
        )

        self.assertEqual(result["dock_id"], "7")
        self.assertEqual(result["center_ned"]["north"], 4.0)

    def test_rejects_missing_dock(self):
        with self.assertRaisesRegex(ValueError, "was not found"):
            parse_semantic_map_dock(
                semantic_map({"id": 1, "type": "buoy", "NED_position": [1, 2, 3]}),
                dock_id=0,
                timestamp=1.0,
            )

    def test_rejects_invalid_json(self):
        with self.assertRaisesRegex(ValueError, "invalid semantic map JSON"):
            parse_semantic_map_dock("{", dock_id=0, timestamp=1.0)

    def test_rejects_non_finite_coordinate(self):
        with self.assertRaisesRegex(ValueError, "finite number"):
            parse_semantic_map_dock(
                '{"objects":[{"id":0,"type":"dock","NED_position":[1,NaN,3]}]}',
                dock_id=0,
                timestamp=1.0,
            )


if __name__ == "__main__":
    unittest.main()
