"""ROS 2 bridge from Mid-360 PointCloud2 to Task3 dock perception JSON."""

from __future__ import annotations

import json
import math
import time
from typing import Any, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError:  # pragma: no cover - available on the Orin ROS overlay
    CustomMsg = None

from dock_planner import DockPerceptionConfig, NumpyDockPerception
from dock_planner.integration import DockEntranceObservationNED, DockPerceptionResult
from robotx_perception_common.local_dock_detector import (
    LocalDockDetectorConfig,
    detect_local_entrances,
    detect_local_entrances_debug,
)
from robotx_perception_common.lidar_transform import (
    custom_msg_to_base_link_xyz,
    pointcloud_message_to_base_link_xyz,
)

from .task3_lidar_bridge_core import (
    USVPoseNED,
    dock_perception_result_to_json,
    parse_coarse_center_json,
    parse_usv_pose_json,
    transform_base_link_points_to_ned,
    transform_base_link_yaw_to_ned_heading,
    transform_ned_point_to_base_link,
)


class Task3LidarPerceptionNode(Node):
    """Consume point clouds and publish the existing Task3 JSON contract."""

    def __init__(self) -> None:
        super().__init__("task3_lidar_perception")
        self.declare_parameter("input_topic", "/livox/lidar")
        # Orin's current official Mid-360 launch publishes PointCloud2.  Keep
        # CustomMsg support available as an explicit override for older driver
        # configurations, but make the observed runtime format the default.
        self.declare_parameter("input_format", "pointcloud2")
        self.declare_parameter("coarse_dock_topic", "/tongji_mrl/twave/task3/coarse_dock")
        self.declare_parameter("usv_pose_topic", "/tongji_mrl/twave/ned_position")
        self.declare_parameter("output_topic", "/tongji_mrl/twave/task3/dock_perception")
        self.declare_parameter("process_period_s", 0.5)
        self.declare_parameter("coarse_timeout_s", 30.0)
        self.declare_parameter("pose_timeout_s", 2.0)
        self.declare_parameter("roi_radius_m", 25.0)
        self.declare_parameter("min_confidence", 0.20)
        self.declare_parameter("detector_mode", "seeded_lines")
        self.declare_parameter("enable_local_fallback", False)
        self.declare_parameter("local_min_confidence", 0.30)
        self.declare_parameter("local_coarse_gate_m", 8.0)
        self.declare_parameter("local_max_candidates", 3)
        self.declare_parameter("local_z_min_m", -0.35)
        self.declare_parameter("local_z_max_m", 0.85)
        self.declare_parameter("local_cluster_distance_m", 0.28)
        self.declare_parameter("local_min_wall_length_m", 0.65)
        self.declare_parameter("local_lateral_half_width_m", 3.0)
        self.declare_parameter("local_coarse_roi_radius_m", 6.0)
        self.declare_parameter("diagnostic_log_period_s", 2.0)

        self._coarse = None
        self._pose: Optional[USVPoseNED] = None
        self._last_process_time = 0.0
        self._last_diagnostic_log_time = 0.0
        self._detector = NumpyDockPerception(DockPerceptionConfig(
            roi_radius_m=float(self.get_parameter("roi_radius_m").value),
            min_confidence=float(self.get_parameter("min_confidence").value),
            source="mid360_task3_lidar_perception",
        ))
        self._local_detector_config = LocalDockDetectorConfig(
            z_min_m=float(self.get_parameter("local_z_min_m").value),
            z_max_m=float(self.get_parameter("local_z_max_m").value),
            cluster_distance_m=float(
                self.get_parameter("local_cluster_distance_m").value
            ),
            min_wall_length_m=float(
                self.get_parameter("local_min_wall_length_m").value
            ),
            lateral_half_width_m=float(
                self.get_parameter("local_lateral_half_width_m").value
            ),
            coarse_roi_radius_m=float(
                self.get_parameter("local_coarse_roi_radius_m").value
            ),
            min_confidence=float(
                self.get_parameter("local_min_confidence").value
            ),
            max_candidates=max(
                1, int(self.get_parameter("local_max_candidates").value)
            ),
        )

        self._result_pub = self.create_publisher(
            String,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self._cloud_sub = self._create_cloud_subscription()
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
            "Task3 LiDAR perception bridge ready; input_format=%s; "
            "detector_mode=%s; waiting for "
            "coarse dock and NED pose with heading_rad."
            % (
                str(self.get_parameter("input_format").value),
                str(self.get_parameter("detector_mode").value),
            )
        )

    def _create_cloud_subscription(self):
        input_format = str(self.get_parameter("input_format").value).strip().lower()
        topic = str(self.get_parameter("input_topic").value)
        if input_format in {"custom", "custom_msg", "livox_custom"}:
            if CustomMsg is None:
                raise RuntimeError(
                    "input_format=custom_msg requires livox_ros_driver2/msg/CustomMsg"
                )
            return self.create_subscription(CustomMsg, topic, self._on_cloud, 10)
        if input_format in {"pointcloud2", "point_cloud2"}:
            return self.create_subscription(PointCloud2, topic, self._on_cloud, 10)
        raise ValueError("input_format must be custom_msg or pointcloud2")

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
    def _message_timestamp(message: Any) -> float:
        stamp = message.header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return value if value > 0.0 else time.time()

    def _coarse_center_in_base(self) -> Tuple[float, float]:
        """Express the current coarse NED centre in ``base_link`` FLU."""

        if self._coarse is None or self._pose is None:
            raise RuntimeError("coarse centre and pose are required")
        return transform_ned_point_to_base_link(
            self._coarse.center_ned.north,
            self._coarse.center_ned.east,
            self._pose,
        )

    def _local_candidates_to_result(
        self,
        candidates,
        *,
        timestamp: float,
        source: str,
    ) -> DockPerceptionResult:
        dock_id = self._coarse.dock_id if self._coarse is not None else "local_test"
        observations = []
        for candidate in candidates:
            local_point = np.array(
                [[candidate.center_x_m, candidate.center_y_m, 0.0]],
                dtype=float,
            )
            if self._pose is not None:
                converted = transform_base_link_points_to_ned(
                    local_point, self._pose,
                )[0]
                north, east = float(converted[0]), float(converted[1])
                heading = transform_base_link_yaw_to_ned_heading(
                    candidate.heading_rad, self._pose,
                )
            else:
                north, east = candidate.center_x_m, candidate.center_y_m
                heading = -candidate.heading_rad
            heading = math.atan2(math.sin(heading), math.cos(heading))
            observations.append(DockEntranceObservationNED(
                entrance_center_north_m=north,
                entrance_center_east_m=east,
                entrance_heading_rad=heading,
                confidence=candidate.confidence,
                timestamp=timestamp,
                source=source,
                valid=True,
                dock_id=dock_id,
            ))
        return DockPerceptionResult(
            dock_id=dock_id,
            entrance_observations=tuple(observations),
            timestamp=timestamp,
            source=source,
            valid=True,
        )

    def _on_cloud(self, message: Any) -> None:
        now = time.monotonic()
        period = float(self.get_parameter("process_period_s").value)
        if period > 0.0 and now - self._last_process_time < period:
            return
        self._last_process_time = now
        local_fallback = bool(self.get_parameter("enable_local_fallback").value)
        if (self._coarse is None or self._pose is None) and not local_fallback:
            return
        wall_time = time.time()
        coarse_timeout = float(self.get_parameter("coarse_timeout_s").value)
        pose_timeout = float(self.get_parameter("pose_timeout_s").value)
        if self._coarse is not None and coarse_timeout >= 0.0 and wall_time - self._coarse.timestamp > coarse_timeout:
            return
        if self._pose is not None and pose_timeout >= 0.0 and wall_time - self._pose.timestamp > pose_timeout:
            return
        try:
            input_format = str(self.get_parameter("input_format").value).strip().lower()
            if input_format in {"custom", "custom_msg", "livox_custom"}:
                points_base = custom_msg_to_base_link_xyz(message)
            else:
                points_base = pointcloud_message_to_base_link_xyz(message)
            timestamp = self._message_timestamp(message)
            result = None
            if self._coarse is not None and self._pose is not None:
                detector_mode = str(
                    self.get_parameter("detector_mode").value
                ).strip().lower()
                if detector_mode == "seeded_lines":
                    debug = detect_local_entrances_debug(
                        points_base,
                        coarse_center_xy=self._coarse_center_in_base(),
                        config=self._local_detector_config,
                    )
                    candidates = debug.candidates
                    diagnostic_period = float(
                        self.get_parameter("diagnostic_log_period_s").value
                    )
                    if (
                        diagnostic_period >= 0.0
                        and now - self._last_diagnostic_log_time >= diagnostic_period
                    ):
                        self._last_diagnostic_log_time = now
                        best = candidates[0] if candidates else None
                        best_text = (
                            "none"
                            if best is None
                            else "x=%.2f y=%.2f gap=%.2f confidence=%.2f"
                            % (
                                best.center_x_m,
                                best.center_y_m,
                                best.gap_m,
                                best.confidence,
                            )
                        )
                        self.get_logger().info(
                            "Dock detector: input=%d filtered=%d clusters=%d "
                            "segments=%d candidates=%d best=%s"
                            % (
                                len(points_base),
                                len(debug.filtered_xyz),
                                len(debug.clusters),
                                len(debug.segments),
                                len(candidates),
                                best_text,
                            )
                        )
                    result = self._local_candidates_to_result(
                        candidates,
                        timestamp=timestamp,
                        source="mid360_task3_seeded_lines",
                    )
                elif detector_mode == "legacy_numpy":
                    points_ned = transform_base_link_points_to_ned(
                        points_base, self._pose,
                    )
                    result = self._detector.detect(
                        points_ned, self._coarse, timestamp=timestamp,
                    )
                else:
                    raise ValueError(
                        "detector_mode must be seeded_lines or legacy_numpy"
                    )
            # The field test can run before the UAV coarse centre is available.
            # Search in the measured base_link cloud and convert candidates with
            # the available heading/position when one exists.  This branch is
            # disabled by default and is intended for recognition bring-up only.
            if local_fallback:
                # In explicit field-test mode the local detector is the
                # authority.  Do not let a far-away candidate from the seeded
                # prototype bypass the local coarse gate.
                result = None
                candidates = detect_local_entrances(
                    points_base,
                    min_confidence=float(self.get_parameter("local_min_confidence").value),
                )
                # If a rough centre is available, use it only as a local
                # ranking/gate.  This prevents shoreline/boat structures far
                # outside the target bay from winning the fallback search.
                if self._coarse is not None and self._pose is not None:
                    reference_x, reference_y = self._coarse_center_in_base()
                    gate = float(self.get_parameter("local_coarse_gate_m").value)
                    candidates = tuple(
                        sorted(
                            (
                                candidate for candidate in candidates
                                if math.hypot(
                                    candidate.center_x_m - reference_x,
                                    candidate.center_y_m - reference_y,
                                ) <= gate
                            ),
                            key=lambda candidate: (
                                math.hypot(
                                    candidate.center_x_m - reference_x,
                                    candidate.center_y_m - reference_y,
                                ),
                                -candidate.confidence,
                            ),
                        )
                    )
                max_candidates = int(self.get_parameter("local_max_candidates").value)
                candidates = candidates[:max(1, max_candidates)]
                result = self._local_candidates_to_result(
                    candidates,
                    timestamp=timestamp,
                    source="mid360_task3_lidar_local_fallback",
                )
            if result is None:
                return
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
