"""ROS 2 bridge from Mid-360 PointCloud2 to Task3 dock perception JSON."""

from __future__ import annotations

from collections import deque
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

from dock_planner import (
    DockPerceptionConfig,
    NumpyDockPerception,
    Pose2D,
    StableEntranceConfig,
    stabilize_entrance_layout,
)
from dock_planner.integration import DockEntranceObservationNED, DockPerceptionResult
from robotx_perception_common.local_dock_detector import (
    LocalDockDetectorConfig,
    detect_local_entrances,
    detect_local_entrances_debug,
)
from robotx_perception_common.lidar_transform import (
    LidarExtrinsics,
    custom_msg_to_base_link_xyz,
    pointcloud_message_to_base_link_xyz,
)
from robotx_perception_common.pointcloud_adapter import pointcloud2_message_to_xyz

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
        # ``sensor`` means points are expressed in the Mid-360/LIO body frame
        # and still need the measured installation transform. ``base_link`` is
        # reserved for an upstream component that already performed it.
        self.declare_parameter("input_frame_mode", "sensor")
        self.declare_parameter("expected_input_frame", "")
        self.declare_parameter("input_to_base_x_m", 0.15)
        self.declare_parameter("input_to_base_y_m", 0.10)
        self.declare_parameter("input_to_base_z_m", 0.22)
        self.declare_parameter("input_to_base_roll_rad", 0.0)
        self.declare_parameter("input_to_base_pitch_rad", 0.0)
        self.declare_parameter("input_to_base_yaw_rad", 0.0)
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
        self.declare_parameter("local_max_range_m", 10.0)
        self.declare_parameter("local_z_min_m", -0.35)
        self.declare_parameter("local_z_max_m", 0.85)
        self.declare_parameter("local_cluster_distance_m", 0.28)
        self.declare_parameter("local_min_wall_length_m", 0.65)
        self.declare_parameter("local_lateral_half_width_m", 3.0)
        self.declare_parameter("local_coarse_roi_radius_m", 6.0)
        self.declare_parameter("unseeded_history_frames", 6)
        self.declare_parameter("unseeded_required_detections", 3)
        self.declare_parameter("unseeded_min_entrances", 3)
        self.declare_parameter("unseeded_association_distance_m", 0.75)
        self.declare_parameter("unseeded_heading_tolerance_rad", 0.35)
        self.declare_parameter("unseeded_dock_depth_m", 6.0)
        self.declare_parameter("unseeded_observation_standoff_m", 4.0)
        self.declare_parameter("required_ned_frame_id", "robotx_rtk_ned")
        self.declare_parameter("diagnostic_log_period_s", 2.0)

        self._coarse = None
        self._pose: Optional[USVPoseNED] = None
        self._last_process_time = 0.0
        self._last_diagnostic_log_time = 0.0
        self._input_frame_mode = str(
            self.get_parameter("input_frame_mode").value
        ).strip().lower()
        if self._input_frame_mode not in {"sensor", "base_link"}:
            raise ValueError("input_frame_mode must be sensor or base_link")
        self._input_extrinsics = LidarExtrinsics(
            x_m=float(self.get_parameter("input_to_base_x_m").value),
            y_m=float(self.get_parameter("input_to_base_y_m").value),
            z_m=float(self.get_parameter("input_to_base_z_m").value),
            roll_rad=float(self.get_parameter("input_to_base_roll_rad").value),
            pitch_rad=float(self.get_parameter("input_to_base_pitch_rad").value),
            yaw_rad=float(self.get_parameter("input_to_base_yaw_rad").value),
            child_frame="input_sensor_frame",
        )
        self._unseeded_frames = deque(maxlen=max(
            3, int(self.get_parameter("unseeded_history_frames").value)
        ))
        self._detector = NumpyDockPerception(DockPerceptionConfig(
            roi_radius_m=float(self.get_parameter("roi_radius_m").value),
            min_confidence=float(self.get_parameter("min_confidence").value),
            source="mid360_task3_lidar_perception",
        ))
        self._local_detector_config = LocalDockDetectorConfig(
            max_range_m=float(self.get_parameter("local_max_range_m").value),
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
        self._coarse_pub = self.create_publisher(
            String,
            str(self.get_parameter("coarse_dock_topic").value),
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
            "input_frame_mode=%s; "
            "detector_mode=%s; waiting for "
            "inputs required by the selected mode."
            % (
                str(self.get_parameter("input_format").value),
                self._input_frame_mode,
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

    def _cloud_points_in_base_link(self, message: Any) -> np.ndarray:
        """Extract one cloud and return points in the USV ``base_link`` frame."""

        expected = str(
            self.get_parameter("expected_input_frame").value
        ).strip().lstrip("/")
        actual = str(getattr(getattr(message, "header", None), "frame_id", ""))
        actual = actual.strip().lstrip("/")
        if expected and actual != expected:
            raise ValueError(
                f"input frame mismatch: expected {expected!r}, received {actual!r}"
            )

        input_format = str(self.get_parameter("input_format").value).strip().lower()
        if self._input_frame_mode == "base_link":
            if input_format in {"custom", "custom_msg", "livox_custom"}:
                raise ValueError(
                    "base_link input_frame_mode requires PointCloud2 input"
                )
            return pointcloud2_message_to_xyz(message)
        if input_format in {"custom", "custom_msg", "livox_custom"}:
            return custom_msg_to_base_link_xyz(message, self._input_extrinsics)
        return pointcloud_message_to_base_link_xyz(message, self._input_extrinsics)

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

    def _stabilize_unseeded_result(
        self,
        result: DockPerceptionResult,
        *,
        timestamp: float,
    ) -> Optional[DockPerceptionResult]:
        """Require repeated three-bay geometry and publish its coarse centre."""

        if self._pose is None:
            return None
        self._unseeded_frames.append(result.entrance_observations)
        layout = stabilize_entrance_layout(
            tuple(self._unseeded_frames),
            Pose2D(
                self._pose.north_m,
                self._pose.east_m,
                self._pose.heading_rad,
            ),
            StableEntranceConfig(
                required_frame_hits=max(
                    1,
                    int(self.get_parameter("unseeded_required_detections").value),
                ),
                min_entrances=max(
                    1,
                    int(self.get_parameter("unseeded_min_entrances").value),
                ),
                association_distance_m=float(
                    self.get_parameter("unseeded_association_distance_m").value
                ),
                association_heading_rad=float(
                    self.get_parameter("unseeded_heading_tolerance_rad").value
                ),
                dock_depth_m=float(
                    self.get_parameter("unseeded_dock_depth_m").value
                ),
                bay_observation_standoff_m=float(
                    self.get_parameter("unseeded_observation_standoff_m").value
                ),
            ),
        )
        if layout is None:
            return None
        dock_id = layout.entrances_left_to_right[0].dock_id or "field_search"
        frame_id = str(self.get_parameter("required_ned_frame_id").value).strip()
        coarse_payload = {
            "dock_id": dock_id,
            "center_ned": {
                "north": layout.coarse_center_ned.north,
                "east": layout.coarse_center_ned.east,
                "down": layout.coarse_center_ned.down,
            },
            "timestamp": timestamp,
            "valid": True,
            "confidence": float(sum(
                item.confidence for item in layout.entrances_left_to_right
            ) / len(layout.entrances_left_to_right)),
            "source": "mid360_task3_unseeded_multiframe",
            "frame_id": frame_id,
            "entrance_count": len(layout.entrances_left_to_right),
            "entrance_heading_rad": layout.inward_heading_rad,
            "bay_observation_poses": [
                {
                    "bay_id": item.bay_id,
                    "north": item.observation_pose.x,
                    "east": item.observation_pose.y,
                    "heading_rad": item.observation_pose.yaw,
                }
                for item in layout.bays
            ],
        }
        coarse_message = String()
        coarse_message.data = json.dumps(
            coarse_payload, separators=(",", ":"), sort_keys=True
        )
        self._coarse_pub.publish(coarse_message)
        self.get_logger().warning(
            "UNSEEDED DOCK CONFIRMED: %d entrances; coarse centre N=%.2f E=%.2f; "
            "left-to-right hits=%s"
            % (
                len(layout.entrances_left_to_right),
                layout.coarse_center_ned.north,
                layout.coarse_center_ned.east,
                layout.frame_hits,
            )
        )
        return DockPerceptionResult(
            dock_id=dock_id,
            entrance_observations=layout.entrances_left_to_right,
            timestamp=timestamp,
            source="mid360_task3_unseeded_multiframe",
            valid=True,
        )

    def _on_cloud(self, message: Any) -> None:
        now = time.monotonic()
        period = float(self.get_parameter("process_period_s").value)
        if period > 0.0 and now - self._last_process_time < period:
            return
        self._last_process_time = now
        local_fallback = bool(self.get_parameter("enable_local_fallback").value)
        detector_mode = str(
            self.get_parameter("detector_mode").value
        ).strip().lower()
        unseeded_mode = detector_mode == "unseeded_lines"
        if self._pose is None:
            return
        if self._coarse is None and not (local_fallback or unseeded_mode):
            return
        wall_time = time.time()
        coarse_timeout = float(self.get_parameter("coarse_timeout_s").value)
        pose_timeout = float(self.get_parameter("pose_timeout_s").value)
        if self._coarse is not None and coarse_timeout >= 0.0 and wall_time - self._coarse.timestamp > coarse_timeout:
            return
        if self._pose is not None and pose_timeout >= 0.0 and wall_time - self._pose.timestamp > pose_timeout:
            return
        try:
            points_base = self._cloud_points_in_base_link(message)
            timestamp = self._message_timestamp(message)
            result = None
            if unseeded_mode:
                debug = detect_local_entrances_debug(
                    points_base,
                    coarse_center_xy=None,
                    config=self._local_detector_config,
                )
                diagnostic_period = float(
                    self.get_parameter("diagnostic_log_period_s").value
                )
                if (
                    diagnostic_period >= 0.0
                    and now - self._last_diagnostic_log_time >= diagnostic_period
                ):
                    self._last_diagnostic_log_time = now
                    self.get_logger().info(
                        "Unseeded 360 detector: input=%d filtered=%d clusters=%d "
                        "segments=%d candidates=%d stable_frames=%d"
                        % (
                            len(points_base),
                            len(debug.filtered_xyz),
                            len(debug.clusters),
                            len(debug.segments),
                            len(debug.candidates),
                            len(self._unseeded_frames),
                        )
                    )
                raw_result = self._local_candidates_to_result(
                    debug.candidates,
                    timestamp=timestamp,
                    source="mid360_task3_unseeded_lines",
                )
                result = self._stabilize_unseeded_result(
                    raw_result,
                    timestamp=timestamp,
                )
            elif self._coarse is not None and self._pose is not None:
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
                        "detector_mode must be seeded_lines, unseeded_lines, "
                        "or legacy_numpy"
                    )
            # The field test can run before the UAV coarse centre is available.
            # Search in the measured base_link cloud and convert candidates with
            # the available heading/position when one exists.  This branch is
            # disabled by default and is intended for recognition bring-up only.
            if local_fallback and not unseeded_mode:
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
