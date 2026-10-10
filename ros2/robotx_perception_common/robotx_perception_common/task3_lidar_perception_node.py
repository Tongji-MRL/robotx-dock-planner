"""ROS 2 bridge from Mid-360 PointCloud2 to Task3 dock perception JSON."""

from __future__ import annotations

import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from dock_planner import DockPerceptionConfig, NumpyDockPerception
from robotx_perception_common.lidar_transform import pointcloud_message_to_base_link_xyz

from .task3_lidar_bridge_core import (
    USVPoseNED,
    dock_perception_result_to_json,
    parse_coarse_center_json,
    parse_usv_pose_json,
    transform_base_link_points_to_ned,
)


class Task3LidarPerceptionNode(Node):
    """Consume point clouds and publish the existing Task3 JSON contract."""

    def __init__(self) -> None:
        super().__init__("task3_lidar_perception")
        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("coarse_dock_topic", "/tongji_mrl/twave/task3/coarse_dock")
        self.declare_parameter("usv_pose_topic", "/tongji_mrl/twave/ned_position")
        self.declare_parameter("output_topic", "/tongji_mrl/twave/task3/dock_perception")
        self.declare_parameter("process_period_s", 0.5)
        self.declare_parameter("coarse_timeout_s", 30.0)
        self.declare_parameter("pose_timeout_s", 2.0)
        self.declare_parameter("roi_radius_m", 25.0)
        self.declare_parameter("min_confidence", 0.20)

        self._coarse = None
        self._pose: Optional[USVPoseNED] = None
        self._last_process_time = 0.0
        self._detector = NumpyDockPerception(DockPerceptionConfig(
            roi_radius_m=float(self.get_parameter("roi_radius_m").value),
            min_confidence=float(self.get_parameter("min_confidence").value),
            source="mid360_task3_lidar_perception",
        ))

        self._result_pub = self.create_publisher(
            String,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self._cloud_sub = self.create_subscription(
            PointCloud2,
            str(self.get_parameter("input_topic").value),
            self._on_cloud,
            10,
        )
        self._coarse_sub = self.create_subscription(
            String,
            str(self.get_parameter("coarse_dock_topic").value),
            self._on_coarse,
            10,
        )
        self._pose_sub = self.create_subscription(
            String,
            str(self.get_parameter("usv_pose_topic").value),
            self._on_pose,
            10,
        )
        self.get_logger().info(
            "Task3 LiDAR perception bridge ready; waiting for coarse dock and "
            "NED pose with heading_rad."
        )

    def _on_coarse(self, message: String) -> None:
        try:
            self._coarse = parse_coarse_center_json(json.loads(message.data))
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignoring invalid coarse dock JSON: {exc}")

    def _on_pose(self, message: String) -> None:
        try:
            self._pose = parse_usv_pose_json(json.loads(message.data))
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignoring NED pose without explicit heading: {exc}")

    @staticmethod
    def _message_timestamp(message: PointCloud2) -> float:
        stamp = message.header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return value if value > 0.0 else time.time()

    def _on_cloud(self, message: PointCloud2) -> None:
        now = time.monotonic()
        period = float(self.get_parameter("process_period_s").value)
        if period > 0.0 and now - self._last_process_time < period:
            return
        self._last_process_time = now
        if self._coarse is None or self._pose is None:
            return
        wall_time = time.time()
        coarse_timeout = float(self.get_parameter("coarse_timeout_s").value)
        pose_timeout = float(self.get_parameter("pose_timeout_s").value)
        if coarse_timeout >= 0.0 and wall_time - self._coarse.timestamp > coarse_timeout:
            return
        if pose_timeout >= 0.0 and wall_time - self._pose.timestamp > pose_timeout:
            return
        try:
            points_base = pointcloud_message_to_base_link_xyz(message)
            points_ned = transform_base_link_points_to_ned(points_base, self._pose)
            timestamp = self._message_timestamp(message)
            result = self._detector.detect(points_ned, self._coarse, timestamp=timestamp)
            output = String()
            output.data = dock_perception_result_to_json(result)
            self._result_pub.publish(output)
            self.get_logger().debug(
                f"Published {len(result.entrance_observations)} entrance candidates"
            )
        except (TypeError, ValueError, RuntimeError) as exc:
            self.get_logger().warning(f"LiDAR perception update failed: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Task3LidarPerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
