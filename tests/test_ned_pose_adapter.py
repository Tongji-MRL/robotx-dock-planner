import json
import math
import unittest

from usv_task3_executor.ned_pose_adapter import (
    enu_odometry_to_ned_payload,
    parse_ned_pose_payload,
)


class NEDPoseAdapterTests(unittest.TestCase):
    def test_enu_position_and_east_facing_yaw_convert_to_ned(self):
        payload = enu_odometry_to_ned_payload(
            east_m=2.0,
            north_m=3.0,
            up_m=0.5,
            quaternion_x=0.0,
            quaternion_y=0.0,
            quaternion_z=0.0,
            quaternion_w=1.0,
            timestamp=10.0,
            sequence=4,
        )
        self.assertEqual(
            payload["position_ned_m"],
            {"north": 3.0, "east": 2.0, "down": -0.5},
        )
        self.assertAlmostEqual(payload["heading_rad"], math.pi / 2.0)

    def test_north_facing_enu_yaw_becomes_zero_ned_heading(self):
        half = math.sqrt(0.5)
        payload = enu_odometry_to_ned_payload(
            east_m=0.0,
            north_m=0.0,
            up_m=0.0,
            quaternion_x=0.0,
            quaternion_y=0.0,
            quaternion_z=half,
            quaternion_w=half,
            timestamp=10.0,
            sequence=0,
        )
        self.assertAlmostEqual(payload["heading_rad"], 0.0)

    def test_parser_round_trip(self):
        payload = enu_odometry_to_ned_payload(
            east_m=2.0,
            north_m=3.0,
            up_m=0.5,
            quaternion_x=0.0,
            quaternion_y=0.0,
            quaternion_z=0.0,
            quaternion_w=1.0,
            timestamp=10.0,
            sequence=4,
        )
        pose = parse_ned_pose_payload(json.loads(json.dumps(payload)))
        self.assertEqual(pose.frame_id, "usv_fcu_local_ned")
        self.assertEqual(pose.sequence, 4)
        self.assertAlmostEqual(pose.north_m, 3.0)
        self.assertAlmostEqual(pose.heading_rad, math.pi / 2.0)

    def test_parser_rejects_position_without_heading(self):
        with self.assertRaisesRegex(ValueError, "heading_rad"):
            parse_ned_pose_payload(
                {
                    "valid": True,
                    "position_ned_m": {"north": 1.0, "east": 2.0, "down": 0.0},
                    "stamp_unix_sec": 3.0,
                    "sequence": 0,
                    "frame_id": "test_ned",
                }
            )


if __name__ == "__main__":
    unittest.main()
