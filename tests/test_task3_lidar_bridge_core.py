import json
import math
import unittest

import numpy as np

from robotx_perception_common.task3_lidar_bridge_core import (
    USVPoseNED,
    dock_perception_result_to_json,
    parse_coarse_center_json,
    parse_usv_pose_json,
    transform_base_link_points_to_ned,
)
from dock_planner.integration import (
    DockEntranceObservationNED,
    DockPerceptionResult,
)


class Task3LidarBridgeCoreTests(unittest.TestCase):
    def test_base_link_to_ned_rotation_and_translation(self):
        pose = USVPoseNED(10.0, 20.0, -1.0, math.pi / 2.0, 100.0)
        points = transform_base_link_points_to_ned([[1.0, 0.0, 2.0]], pose)
        np.testing.assert_allclose(points, [[10.0, 21.0, -3.0]])

    def test_pose_requires_heading(self):
        with self.assertRaises(ValueError):
            parse_usv_pose_json({
                "valid": True,
                "position_ned_m": {"north": 1, "east": 2, "down": 0},
                "stamp_unix_sec": 3,
            })

    def test_contract_round_trip_shape(self):
        coarse = parse_coarse_center_json({
            "dock_id": "dock_a",
            "center_ned": {"north": 10, "east": 20, "down": 0},
            "timestamp": 4,
            "valid": True,
        })
        self.assertEqual(coarse.dock_id, "dock_a")
        result = DockPerceptionResult(
            dock_id="dock_a",
            entrance_observations=(DockEntranceObservationNED(
                entrance_center_north_m=10.0,
                entrance_center_east_m=20.0,
                entrance_heading_rad=0.5,
                confidence=0.8,
                timestamp=5.0,
                source="test",
                dock_id="dock_a",
            ),),
            timestamp=5.0,
            source="test",
            valid=True,
            valid_until=7.0,
        )
        payload = json.loads(dock_perception_result_to_json(result))
        self.assertEqual(payload["dock_id"], "dock_a")
        self.assertEqual(len(payload["entrance_observations"]), 1)
        self.assertEqual(payload["valid_until"], 7.0)


if __name__ == "__main__":
    unittest.main()
