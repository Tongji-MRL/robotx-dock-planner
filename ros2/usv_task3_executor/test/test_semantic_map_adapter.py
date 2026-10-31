import json
import unittest

from usv_task3_executor.semantic_map_adapter import (
    build_platform_report,
    parse_semantic_map_dock,
)


def semantic_map(*objects):
    return json.dumps({
        "map_epoch": "run-3",
        "version": 7,
        "objects": list(objects),
    })


class SemanticMapAdapterTests(unittest.TestCase):
    def test_selects_platforms_for_state_machine(self):
        report = build_platform_report(semantic_map(
            {"id": 0, "type": "platform", "NED_position": [1, 2, 3]},
            {"id": 1, "type": "buoy", "NED_position": [4, 5, 6]},
        ))
        self.assertEqual(report["task_id"], 3)
        self.assertEqual(report["data_kind"], "platforms")
        self.assertEqual(report["map_epoch"], "run-3")
        self.assertEqual([item["id"] for item in report["objects"]], [0])

    def test_builds_real_ned_coarse_center(self):
        result = parse_semantic_map_dock(
            semantic_map({
                "id": 0,
                "type": "platform",
                "NED_position": [1, 2, 3],
                "confidence": 0.9,
            }),
            dock_id=0,
            timestamp=12.5,
        )
        self.assertEqual(result["center_ned"], {"north": 1.0, "east": 2.0, "down": 3.0})
        self.assertEqual(result["source"], "semantic_map")
        self.assertFalse(result["mock"])
        self.assertEqual(result["frame_id"], "ned")
        self.assertEqual(result["confidence"], 0.9)

    def test_rejects_missing_platform(self):
        with self.assertRaisesRegex(ValueError, "was not found"):
            parse_semantic_map_dock(
                semantic_map({"id": 1, "type": "buoy", "NED_position": [1, 2, 3]}),
                dock_id=0,
                timestamp=1.0,
            )


if __name__ == "__main__":
    unittest.main()
