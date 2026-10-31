"""Structural entry point for the future USV Task 3 implementation."""

import json
import math
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

from dock_planner.integration import CoarseDockCenterObservationNED
from dock_planner import (
    CoarseDockInfo,
    DockEntranceObservationNED,
    DockPerceptionResult,
    DockPlanner,
    NEDPoint,
    PlannerConfig,
    Pose2D,
    Task3FlowManager,
    Task3RetryConfig,
    Task3FlowState,
)


def _finite_json_number(value: Any, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field_name} must be a finite number")
    try:
        numeric_value = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{field_name} must be a finite number") from exc
    if not math.isfinite(numeric_value):
        raise ValueError(f"{field_name} must be a finite number")
    return numeric_value


class Task3Executor(Node):
    def __init__(self) -> None:
        super().__init__("usv_task3_executor")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("target_color", "RED")
        self.declare_parameter("bay_width_m", 3.0)
        self.declare_parameter("bay_depth_m", 6.0)
        self.declare_parameter("approach_distance_m", 5.0)
        self.declare_parameter("stop_distance_inside_m", 1.0)
        self.declare_parameter("retry_max_attempts", 3)
        self.declare_parameter("retry_operation_timeout_s", 30.0)
        self.flow_manager = Task3FlowManager(
            DockPlanner(
                PlannerConfig(
                    approach_distance=float(self.get_parameter("approach_distance_m").value),
                    stop_distance_inside=float(self.get_parameter("stop_distance_inside_m").value),
                )
            ),
            target_color=str(self.get_parameter("target_color").value),
            bay_width=float(self.get_parameter("bay_width_m").value),
            bay_depth=float(self.get_parameter("bay_depth_m").value),
            retry_config=Task3RetryConfig(
                max_attempts=int(self.get_parameter("retry_max_attempts").value),
                operation_timeout_s=float(
                    self.get_parameter("retry_operation_timeout_s").value
                ),
            ),
        )
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.latest_coarse_dock_center: Optional[CoarseDockCenterObservationNED] = None
        self.latest_coarse_dock_frame_id: Optional[str] = None
        self.latest_coarse_dock_is_mock = False
        self.latest_position_ned_m: Optional[dict[str, float]] = None
        self.latest_frame_id: Optional[str] = None
        self.latest_position_message_time: Optional[float] = None
        self.latest_position_received_time: Optional[float] = None
        self.position_received = False
        self.position_valid = False

        self.position_sub = self.create_subscription(
            String,
            "/tongji_mrl/twave/ned_position",
            self._on_ned_position,
            10,
        )
        self.coarse_dock_sub = self.create_subscription(
            String,
            "/tongji_mrl/twave/task3/coarse_dock",
            self._on_coarse_dock,
            10,
        )
        self.dock_perception_sub = self.create_subscription(
            String,
            "/tongji_mrl/twave/task3/dock_perception",
            self._on_dock_perception,
            10,
        )
        self.approach_reached_sub = self.create_subscription(
            Empty,
            "/tongji_mrl/twave/task3/approach_reached",
            self._on_approach_reached,
            10,
        )
        self.state_pub = self.create_publisher(String, "/tongji_mrl/twave/task3/state", 10)
        self.timer = self.create_timer(1.0, self._publish_state)
        self.get_logger().info("USV Task 3 executor started in IDLE state.")

    def _on_ned_position(self, msg: String) -> None:
        receive_time = time.time()
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("message JSON must be an object")
            if payload.get("valid") is not True:
                raise ValueError("position message is not valid")

            position = payload["position_ned_m"]
            if not isinstance(position, dict):
                raise ValueError("position_ned_m must be an object")
            position_ned_m = {
                axis: float(position[axis]) for axis in ("north", "east", "down")
            }
            if not all(math.isfinite(value) for value in position_ned_m.values()):
                raise ValueError("position_ned_m contains a non-finite value")

            frame_id = payload["frame_id"]
            if not isinstance(frame_id, str) or not frame_id:
                raise ValueError("frame_id must be a non-empty string")

            message_time = payload["stamp_unix_sec"]
            sequence = payload["sequence"]
            if not isinstance(message_time, (int, float)) or not math.isfinite(message_time):
                raise ValueError("stamp_unix_sec must be finite")
            if not isinstance(sequence, int):
                raise ValueError("sequence must be an integer")
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignoring invalid NED position message: {exc}")
            return

        self.latest_position_ned_m = position_ned_m
        self.latest_frame_id = frame_id
        self.latest_position_message_time = float(message_time)
        self.latest_position_received_time = receive_time
        self.position_received = True
        self.position_valid = True

    @staticmethod
    def _parse_ned_point(payload: Any, field_name: str) -> NEDPoint:
        if not isinstance(payload, dict):
            raise ValueError(f"{field_name} must be an object")
        return NEDPoint(
            north=_finite_json_number(payload.get("north"), f"{field_name}.north"),
            east=_finite_json_number(payload.get("east"), f"{field_name}.east"),
            down=_finite_json_number(payload.get("down"), f"{field_name}.down"),
        )

    def _on_coarse_dock(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("message JSON must be an object")
            if "approximate_corners_ned" not in payload:
                is_mock = payload.get("mock") is True
                if is_mock:
                    if not self.dry_run:
                        raise ValueError("mock coarse dock input requires dry_run=true")
                    if payload.get("source") != "semantic_map_mock":
                        raise ValueError("unsupported mock coarse dock source")
                elif payload.get("source") != "semantic_map":
                    raise ValueError("unsupported center-only coarse dock source")
                frame_id = payload.get("frame_id")
                expected_frame = "mock_ned" if is_mock else "ned"
                if frame_id != expected_frame:
                    raise ValueError(
                        f"center-only semantic-map frame_id must be {expected_frame}"
                    )
                if payload.get("valid") is not True:
                    raise ValueError("coarse dock center message is not valid")
                observation = CoarseDockCenterObservationNED(
                    dock_id=str(payload["dock_id"]),
                    center_ned=self._parse_ned_point(payload["center_ned"], "center_ned"),
                    timestamp=_finite_json_number(payload["timestamp"], "timestamp"),
                    valid=True,
                    confidence=(
                        None
                        if payload.get("confidence") is None
                        else _finite_json_number(payload["confidence"], "confidence")
                    ),
                    source=payload["source"],
                )
                self.latest_coarse_dock_center = observation
                self.latest_coarse_dock_frame_id = frame_id
                self.latest_coarse_dock_is_mock = is_mock
                self.get_logger().info(
                    "Recorded center-only semantic-map dock observation; search start "
                    "waits for a current pose that includes heading."
                )
                return

            corners = payload["approximate_corners_ned"]
            if not isinstance(corners, list):
                raise ValueError("approximate_corners_ned must be an array")
            coarse_dock = CoarseDockInfo(
                dock_id=payload["dock_id"],
                center_ned=self._parse_ned_point(payload["center_ned"], "center_ned"),
                approximate_corners_ned=tuple(
                    self._parse_ned_point(point, f"approximate_corners_ned[{index}]")
                    for index, point in enumerate(corners)
                ),
                timestamp=_finite_json_number(payload["timestamp"], "timestamp"),
                valid=payload["valid"],
            )
            self.flow_manager.begin(coarse_dock, now=time.time())
        except (KeyError, TypeError, ValueError, RuntimeError) as exc:
            self.get_logger().warning(f"Ignoring invalid coarse dock message: {exc}")

    def _on_dock_perception(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("message JSON must be an object")
            raw_observations = payload["entrance_observations"]
            if not isinstance(raw_observations, list):
                raise ValueError("entrance_observations must be an array")

            observations = []
            for index, item in enumerate(raw_observations):
                if not isinstance(item, dict):
                    raise ValueError(f"entrance_observations[{index}] must be an object")
                observations.append(
                    DockEntranceObservationNED(
                        entrance_center_north_m=_finite_json_number(
                            item["entrance_center_north_m"],
                            f"entrance_observations[{index}].entrance_center_north_m",
                        ),
                        entrance_center_east_m=_finite_json_number(
                            item["entrance_center_east_m"],
                            f"entrance_observations[{index}].entrance_center_east_m",
                        ),
                        entrance_heading_rad=_finite_json_number(
                            item["entrance_heading_rad"],
                            f"entrance_observations[{index}].entrance_heading_rad",
                        ),
                        confidence=_finite_json_number(
                            item["confidence"], f"entrance_observations[{index}].confidence"
                        ),
                        timestamp=_finite_json_number(
                            item["timestamp"], f"entrance_observations[{index}].timestamp"
                        ),
                        source=item["source"],
                        valid=item.get("valid", True),
                        dock_id=item.get("dock_id"),
                    )
                )

            result = DockPerceptionResult(
                dock_id=payload["dock_id"],
                entrance_observations=tuple(observations),
                timestamp=_finite_json_number(payload["timestamp"], "timestamp"),
                source=payload["source"],
                valid=payload.get("valid", True),
                valid_until=(
                    None
                    if payload.get("valid_until") is None
                    else _finite_json_number(payload["valid_until"], "valid_until")
                ),
            )
            current_pose_payload = payload["current_pose"]
            if not isinstance(current_pose_payload, dict):
                raise ValueError("current_pose must be an object")
            current_pose = Pose2D(
                x=_finite_json_number(current_pose_payload.get("x"), "current_pose.x"),
                y=_finite_json_number(current_pose_payload.get("y"), "current_pose.y"),
                yaw=_finite_json_number(current_pose_payload.get("yaw"), "current_pose.yaw"),
            )
            self.flow_manager.accept_perception_result(
                result,
                current_pose,
                now=time.time(),
            )
        except (KeyError, TypeError, ValueError, RuntimeError) as exc:
            self.get_logger().warning(f"Ignoring invalid dock perception message: {exc}")

    def _on_approach_reached(self, _msg: Empty) -> None:
        try:
            self.flow_manager.approach_reached()
        except RuntimeError as exc:
            self.get_logger().warning(f"Ignoring approach-reached event: {exc}")

    def _publish_state(self) -> None:
        current_state: Task3FlowState = self.flow_manager.state
        msg = String()
        msg.data = json.dumps(
            {
                "state": current_state.value,
                "ready_for_task_logic": self.position_valid,
                "position_received": self.position_received,
                "position_valid": self.position_valid,
                "position_ned_m": self.latest_position_ned_m,
                "frame_id": self.latest_frame_id,
                "dry_run": self.dry_run,
                "error": self.flow_manager.error,
                "retry": self.flow_manager.retry_status,
                "coarse_dock_center": (
                    None
                    if self.latest_coarse_dock_center is None
                    else {
                        "dock_id": self.latest_coarse_dock_center.dock_id,
                        "center_ned": {
                            "north": self.latest_coarse_dock_center.center_ned.north,
                            "east": self.latest_coarse_dock_center.center_ned.east,
                            "down": self.latest_coarse_dock_center.center_ned.down,
                        },
                        "timestamp": self.latest_coarse_dock_center.timestamp,
                        "valid": self.latest_coarse_dock_center.valid,
                        "confidence": self.latest_coarse_dock_center.confidence,
                        "source": self.latest_coarse_dock_center.source,
                        "mock": self.latest_coarse_dock_is_mock,
                        "frame_id": self.latest_coarse_dock_frame_id,
                    }
                ),
            },
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.state_pub.publish(msg)

    # TODO: Add bay color recognition result input.
    # TODO: Add DockPlanner integration.
    # TODO: Add PX4 velocity/control adapter.
    # TODO: Add gimbal and water pump control.


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Task3Executor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
