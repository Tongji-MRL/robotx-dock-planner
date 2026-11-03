"""Structural entry point for the future USV Task 3 implementation."""

import json
import math
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String

from dock_planner.integration import CoarseDockCenterObservationNED
from usv_task3_executor.ned_pose_adapter import parse_ned_pose_payload
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
    GotoReferenceSample,
    GotoStatusKind,
    ned_goal_to_goto_enu,
    parse_goto_status,
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
        self.declare_parameter("position_timeout_s", 2.0)
        # Motion output is deliberately opt-in.  A dry run or a missing
        # explicit enable flag must never hand a target to the boat controller.
        self.declare_parameter("enable_goto_output", False)
        self.declare_parameter("goto_target_topic", "/safe_passage/goto/target")
        self.declare_parameter("goto_status_topic", "/safe_passage/goto/status")
        self.declare_parameter("goto_preview_topic", "/tongji_mrl/twave/task3/goto_preview")
        self.declare_parameter("goto_frame_id", "map")
        self.declare_parameter("goto_max_reference_skew_s", 0.2)
        self.declare_parameter("goto_odom_topic", "/mavros/local_position/odom")
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
        self.latest_heading_rad: Optional[float] = None
        self.latest_current_pose: Optional[Pose2D] = None
        self.latest_frame_id: Optional[str] = None
        self.latest_position_message_time: Optional[float] = None
        self.latest_position_received_time: Optional[float] = None
        self.position_received = False
        self.position_valid = False
        self.enable_goto_output = bool(self.get_parameter("enable_goto_output").value)
        self.latest_odom_east_m: Optional[float] = None
        self.latest_odom_north_m: Optional[float] = None
        self.latest_odom_timestamp: Optional[float] = None
        self.active_goto_phase: Optional[str] = None
        self.last_goto_target: Optional[dict[str, object]] = None

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
        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("goto_odom_topic").value),
            self._on_odom,
            10,
        )
        self.goto_status_sub = self.create_subscription(
            String,
            str(self.get_parameter("goto_status_topic").value),
            self._on_goto_status,
            10,
        )
        self.state_pub = self.create_publisher(String, "/tongji_mrl/twave/task3/state", 10)
        self.goto_target_pub = self.create_publisher(
            PointStamped,
            str(self.get_parameter("goto_target_topic").value),
            10,
        )
        self.goto_preview_pub = self.create_publisher(
            String,
            str(self.get_parameter("goto_preview_topic").value),
            10,
        )
        self.timer = self.create_timer(1.0, self._publish_state)
        self.get_logger().info("USV Task 3 executor started in IDLE state.")

    def _on_ned_position(self, msg: String) -> None:
        receive_time = time.time()
        try:
            payload = json.loads(msg.data)
            pose = parse_ned_pose_payload(payload)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignoring invalid NED position message: {exc}")
            return

        self.latest_position_ned_m = {
            "north": pose.north_m,
            "east": pose.east_m,
            "down": pose.down_m,
        }
        self.latest_heading_rad = pose.heading_rad
        self.latest_current_pose = Pose2D(
            x=pose.north_m,
            y=pose.east_m,
            yaw=pose.heading_rad,
        )
        self.latest_frame_id = pose.frame_id
        self.latest_position_message_time = pose.timestamp
        self.latest_position_received_time = receive_time
        self.position_received = True
        self.position_valid = True
        self._try_begin_center_search()

    def _on_odom(self, msg: Odometry) -> None:
        """Cache the ENU map sample used to translate a Task3 NED goal."""

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        values = (msg.pose.pose.position.x, msg.pose.pose.position.y, stamp)
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().warning("Ignoring non-finite odometry sample")
            return
        self.latest_odom_east_m = float(msg.pose.pose.position.x)
        self.latest_odom_north_m = float(msg.pose.pose.position.y)
        self.latest_odom_timestamp = float(stamp)

    def _try_begin_center_search(self) -> None:
        """Start the center-only flow once a fresh pose in the same frame exists."""

        if self.flow_manager.state is not Task3FlowState.IDLE:
            return
        if self.latest_coarse_dock_center is None or self.latest_current_pose is None:
            return
        if self.latest_coarse_dock_frame_id != self.latest_frame_id:
            self.get_logger().warning(
                "Cannot start center search: coarse dock frame %r does not match "
                "USV pose frame %r."
                % (self.latest_coarse_dock_frame_id, self.latest_frame_id)
            )
            return
        timeout = float(self.get_parameter("position_timeout_s").value)
        if (
            timeout >= 0.0
            and self.latest_position_received_time is not None
            and time.time() - self.latest_position_received_time > timeout
        ):
            return
        plan = self.flow_manager.begin_center_search(
            self.latest_coarse_dock_center,
            self.latest_current_pose,
            now=time.time(),
        )
        if plan is not None:
            self.get_logger().info(
                "Started center-only dock search with %d arc waypoints."
                % len(plan.arc_waypoints)
            )
            if self.flow_manager.state is Task3FlowState.SEARCH_DOCK:
                self._publish_current_goto_goal("SEARCH_DOCK")
            else:
                self._publish_current_goto_goal("APPROACH_DOCK")

    def _publish_current_goto_goal(self, phase: str) -> None:
        """Translate and publish the flow's current goal, or only preview it.

        The preview topic is always emitted for dry-run and field diagnostics.
        The real ``PointStamped`` is emitted only when ``enable_goto_output`` is
        true and the executor is not in dry-run mode.
        """

        goal = (
            self.flow_manager.approach_goal
            if phase == "APPROACH_DOCK"
            else self.flow_manager.current_search_goal
        )
        if goal is None:
            self.get_logger().warning(f"No {phase} goal is available")
            return
        if (
            self.latest_position_ned_m is None
            or self.latest_position_message_time is None
            or self.latest_odom_east_m is None
            or self.latest_odom_north_m is None
            or self.latest_odom_timestamp is None
        ):
            self.get_logger().warning(
                f"Cannot convert {phase} goal: synchronized NED and odom samples are required",
            )
            return
        try:
            reference = GotoReferenceSample(
                current_ned=NEDPoint(
                    north=self.latest_position_ned_m["north"],
                    east=self.latest_position_ned_m["east"],
                    down=self.latest_position_ned_m["down"],
                ),
                odom_east_m=self.latest_odom_east_m,
                odom_north_m=self.latest_odom_north_m,
                ned_timestamp=self.latest_position_message_time,
                odom_timestamp=self.latest_odom_timestamp,
            )
            target = ned_goal_to_goto_enu(
                goal,
                reference,
                max_reference_skew_s=float(
                    self.get_parameter("goto_max_reference_skew_s").value
                ),
            )
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Cannot convert {phase} goal: {exc}")
            return

        self.active_goto_phase = phase
        self.last_goto_target = {
            "phase": phase,
            "east_m": target.east_m,
            "north_m": target.north_m,
            "up_m": target.up_m,
            "frame_id": target.frame_id,
        }
        preview = String()
        preview.data = json.dumps(self.last_goto_target, separators=(",", ":"))
        self.goto_preview_pub.publish(preview)
        if self.dry_run or not self.enable_goto_output:
            self.get_logger().info(
                "Goto preview (%s): east=%.3f north=%.3f frame=%s; motion output disabled"
                % (phase, target.east_m, target.north_m, target.frame_id)
            )
            return
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("goto_frame_id").value)
        message.point.x = target.east_m
        message.point.y = target.north_m
        message.point.z = target.up_m
        self.goto_target_pub.publish(message)
        self.get_logger().warning(
            "GOTO target published (%s): east=%.3f north=%.3f frame=%s"
            % (phase, target.east_m, target.north_m, message.header.frame_id)
        )

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
            is_mock = payload.get("mock") is True
            if is_mock:
                if not self.dry_run:
                    raise ValueError("mock coarse dock input requires dry_run=true")
                if payload.get("source") != "semantic_map_mock":
                    raise ValueError("unsupported mock coarse dock source")
            if "approximate_corners_ned" not in payload:
                frame_id = payload.get("frame_id")
                if not isinstance(frame_id, str) or not frame_id.strip():
                    raise ValueError("center-only coarse dock requires frame_id")
                if payload.get("valid") is not True:
                    raise ValueError("center-only coarse dock message is not valid")
                observation = CoarseDockCenterObservationNED(
                    dock_id=payload["dock_id"],
                    center_ned=self._parse_ned_point(payload["center_ned"], "center_ned"),
                    timestamp=_finite_json_number(payload["timestamp"], "timestamp"),
                    valid=True,
                    source=payload["source"],
                )
                self.latest_coarse_dock_center = observation
                self.latest_coarse_dock_frame_id = frame_id.strip()
                self.latest_coarse_dock_is_mock = is_mock
                self.get_logger().info(
                    "Recorded center-only coarse dock; waiting for a fresh "
                    "USV pose in the same NED frame."
                )
                self._try_begin_center_search()
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
            current_pose_payload = payload.get("current_pose")
            if current_pose_payload is None:
                if self.latest_current_pose is None:
                    raise ValueError("no current USV NED pose is available")
                current_pose = self.latest_current_pose
            else:
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
            self._publish_current_goto_goal("SEARCH_DOCK")
        except RuntimeError as exc:
            self.get_logger().warning(f"Ignoring approach-reached event: {exc}")

    def _on_goto_status(self, msg: String) -> None:
        try:
            status = parse_goto_status(msg.data)
        except ValueError as exc:
            self.get_logger().warning(f"Ignoring invalid goto status: {exc}")
            return
        if status.kind is GotoStatusKind.ABORTED:
            self.flow_manager.fail("goto aborted: %s" % (status.reason or "UNKNOWN"))
            self.active_goto_phase = None
            return
        if status.kind is not GotoStatusKind.DONE:
            return
        if self.active_goto_phase == "APPROACH_DOCK":
            try:
                self.flow_manager.approach_reached()
            except RuntimeError as exc:
                self.get_logger().warning(f"Ignoring goto DONE for approach: {exc}")
                return
            self._publish_current_goto_goal("SEARCH_DOCK")
        elif self.active_goto_phase == "SEARCH_DOCK":
            try:
                next_goal = self.flow_manager.search_waypoint_reached()
            except RuntimeError as exc:
                self.get_logger().warning(f"Ignoring goto DONE for search: {exc}")
                return
            if next_goal is not None:
                self._publish_current_goto_goal("SEARCH_DOCK")
            else:
                self.active_goto_phase = None

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
                "heading_rad": self.latest_heading_rad,
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
                        "source": self.latest_coarse_dock_center.source,
                        "mock": self.latest_coarse_dock_is_mock,
                        "frame_id": self.latest_coarse_dock_frame_id,
                    }
                ),
                # Retain the old diagnostic key for existing observers while
                # exposing the general center-only contract above.
                "mock_coarse_dock_center": (
                    None
                    if (
                        self.latest_coarse_dock_center is None
                        or not self.latest_coarse_dock_is_mock
                    )
                    else {
                        "dock_id": self.latest_coarse_dock_center.dock_id,
                        "center_ned": {
                            "north": self.latest_coarse_dock_center.center_ned.north,
                            "east": self.latest_coarse_dock_center.center_ned.east,
                            "down": self.latest_coarse_dock_center.center_ned.down,
                        },
                        "timestamp": self.latest_coarse_dock_center.timestamp,
                        "valid": self.latest_coarse_dock_center.valid,
                        "source": self.latest_coarse_dock_center.source,
                        "mock": True,
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
    # Goto target output is guarded by ``enable_goto_output`` and dry_run.
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
