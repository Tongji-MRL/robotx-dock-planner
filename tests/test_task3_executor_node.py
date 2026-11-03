import json
import time
import unittest

import rclpy
from std_msgs.msg import Empty, String

from dock_planner import Task3FlowState
from usv_task3_executor.node import Task3Executor


class Task3ExecutorNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Task3Executor()

    def tearDown(self):
        self.node.destroy_node()

    @staticmethod
    def message(payload):
        message = String()
        message.data = json.dumps(payload)
        return message

    def test_center_search_and_perception_use_shared_pose(self):
        now = time.time()
        self.node._on_ned_position(self.message({
            "valid": True,
            "position_ned_m": {"north": 0.0, "east": 0.0, "down": 0.0},
            "heading_rad": 0.0,
            "stamp_unix_sec": now,
            "sequence": 1,
            "frame_id": "test_ned",
        }))
        self.node._on_coarse_dock(self.message({
            "dock_id": "dock_0",
            "center_ned": {"north": 20.0, "east": 0.0, "down": 0.0},
            "timestamp": now,
            "valid": True,
            "source": "semantic_map_mock",
            "mock": True,
            "frame_id": "test_ned",
        }))
        self.assertEqual(self.node.flow_manager.state, Task3FlowState.APPROACH_DOCK)

        self.node._on_approach_reached(Empty())
        self.assertIsNotNone(self.node.flow_manager.current_search_goal)
        perception_time = time.time()
        self.node._on_dock_perception(self.message({
            "dock_id": "dock_0",
            "entrance_observations": [{
                "entrance_center_north_m": 18.0,
                "entrance_center_east_m": 0.0,
                "entrance_heading_rad": 0.0,
                "confidence": 0.9,
                "timestamp": perception_time,
                "source": "test",
                "valid": True,
                "dock_id": "dock_0",
            }],
            "timestamp": perception_time,
            "source": "test",
            "valid": True,
        }))
        self.assertEqual(self.node.flow_manager.state, Task3FlowState.CHECK_BAY_COLOR)

    def test_mismatched_frames_do_not_start(self):
        now = time.time()
        self.node._on_ned_position(self.message({
            "valid": True,
            "position_ned_m": {"north": 0.0, "east": 0.0, "down": 0.0},
            "heading_rad": 0.0,
            "stamp_unix_sec": now,
            "sequence": 1,
            "frame_id": "usv_ned",
        }))
        self.node._on_coarse_dock(self.message({
            "dock_id": "dock_0",
            "center_ned": {"north": 20.0, "east": 0.0, "down": 0.0},
            "timestamp": now,
            "valid": True,
            "source": "semantic_map_mock",
            "mock": True,
            "frame_id": "uav_ned",
        }))
        self.assertEqual(self.node.flow_manager.state, Task3FlowState.IDLE)


if __name__ == "__main__":
    unittest.main()
