import unittest

from dock_planner import DockPlanner, PlannerConfig, PlanStatus, Pose2D
from dock_planner.vrx_reference import vrx_dock_2022_bay_to_dockspec
from examples.phase1_target import (
    APPROACH_DISTANCE,
    BAY_INDEX,
    DOCK_WORLD_POSE,
    STOP_DISTANCE_INSIDE,
    build_phase1_target,
)


class Phase1TargetTests(unittest.TestCase):
    def test_target_is_existing_adapter_and_planner_output(self):
        record = build_phase1_target()
        dock = vrx_dock_2022_bay_to_dockspec(*DOCK_WORLD_POSE, BAY_INDEX)
        expected = DockPlanner(
            PlannerConfig(APPROACH_DISTANCE, STOP_DISTANCE_INSIDE)
        ).plan(Pose2D(0.0, 0.0, 0.0), dock)
        self.assertEqual(expected.status, PlanStatus.SUCCESS)
        self.assertEqual(record["approach_pose"]["x"], expected.approach_pose.x)
        self.assertEqual(record["approach_pose"]["y"], expected.approach_pose.y)
        self.assertEqual(record["approach_pose"]["yaw"], expected.approach_pose.yaw)
        self.assertEqual(record["entrance_pose"]["x"], expected.waypoints[2].x)
        self.assertEqual(record["entrance_pose"]["y"], expected.waypoints[2].y)
        self.assertEqual(record["entrance_pose"]["yaw"], expected.waypoints[2].yaw)
        self.assertEqual(record["stop_pose"]["x"], expected.waypoints[3].x)
        self.assertEqual(record["stop_pose"]["y"], expected.waypoints[3].y)
        self.assertEqual(record["stop_pose"]["yaw"], expected.waypoints[3].yaw)
        self.assertEqual(
            record["stop_pose"]["yaw"], record["entrance_pose"]["yaw"]
        )
        self.assertEqual(record["bay_index"], 2)
        self.assertEqual(record["planner_config"]["approach_distance"], 12.0)


if __name__ == "__main__":
    unittest.main()
