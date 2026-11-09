"""Structural entry point for the future USV Task 3 implementation."""

import json
import math
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, String
from robotx_msgs.msg import LEDObservation

from dock_planner.integration import CoarseDockCenterObservationNED
from usv_task3_executor.ned_pose_adapter import parse_ned_pose_payload
from dock_planner import (
    BayColorObservation,
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
        self.declare_parameter("target_color", "GREEN")
        self.declare_parameter("bay_width_m", 3.0)
        self.declare_parameter("bay_depth_m", 6.0)
        self.declare_parameter("approach_distance_m", 5.0)
        self.declare_parameter("stop_distance_inside_m", 1.0)
        self.declare_parameter("retry_max_attempts", 3)
        self.declare_parameter("retry_operation_timeout_s", 30.0)
        self.declare_parameter("position_timeout_s", 2.0)
        # The formal launch sets this to robotx_rtk_ned.  An empty library
        # default keeps isolated unit/replay tests able to use explicit test
        # frames without pretending those frames are formal RTK coordinates.
        self.declare_parameter("required_ned_frame_id", "")
        # Motion output is deliberately opt-in.  A dry run or a missing
        # explicit enable flag must never hand a target to the boat controller.
        self.declare_parameter("enable_goto_output", False)
        self.declare_parameter("goto_target_topic", "/safe_passage/goto/target")
        self.declare_parameter("goto_status_topic", "/safe_passage/goto/status")
        self.declare_parameter("goto_preview_topic", "/tongji_mrl/twave/task3/goto_preview")
        self.declare_parameter("goto_frame_id", "map")
        self.declare_parameter("goto_max_reference_skew_s", 0.2)
        self.declare_parameter("goto_odom_topic", "/mavros/local_position/odom")
        self.declare_parameter("field_direct_dock", False)
        self.declare_parameter("field_coarse_center_ahead_m", 4.0)
        self.declare_parameter("field_dock_id", "field_dock")
        self.declare_parameter("field_required_detections", 3)
        self.declare_parameter("field_detection_position_tolerance_m", 1.5)
        self.declare_parameter("field_detection_heading_tolerance_rad", 1.0)
        self.declare_parameter("field_detection_window_s", 10.0)
        self.declare_parameter("field_max_entrance_distance_m", 7.0)
        self.declare_parameter("field_max_lateral_offset_m", 2.0)
        self.declare_parameter("field_max_coarse_offset_m", 5.0)
        self.declare_parameter("dock_led_topic", "/robotx/perception/leds")
        self.declare_parameter("dock_led_timeout_s", 0.75)
        self.declare_parameter("dock_led_required_detections", 3)
        self.declare_parameter("dock_led_sequence_window_s", 1.5)
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
        self.field_direct_dock = bool(self.get_parameter("field_direct_dock").value)
        self.field_coarse_published = False
        self.field_detection_count = 0
        self.field_last_candidate: Optional[DockEntranceObservationNED] = None
        self.field_last_candidate_time: Optional[float] = None
        self.field_coarse_payload: Optional[dict[str, object]] = None
        self.dock_led_candidate_color: Optional[str] = None
        self.dock_led_detection_count = 0
        self.dock_led_last_timestamp: Optional[float] = None

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
            qos_profile_sensor_data,
        )
        self.goto_status_sub = self.create_subscription(
            String,
            str(self.get_parameter("goto_status_topic").value),
            self._on_goto_status,
            10,
        )
        self.dock_led_sub = self.create_subscription(
            LEDObservation,
            str(self.get_parameter("dock_led_topic").value),
            self._on_dock_led,
            10,
        )
        self.state_pub = self.create_publisher(String, "/tongji_mrl/twave/task3/state", 10)
        self.coarse_dock_pub = self.create_publisher(
            String, "/tongji_mrl/twave/task3/coarse_dock", 10
        )
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
        self.docked_confirmed_pub = self.create_publisher(
            Bool, "/robotx/task3/docked_confirmed", 10
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

        required_frame = str(
            self.get_parameter("required_ned_frame_id").value
        ).strip()
        if required_frame and pose.frame_id != required_frame:
            self.get_logger().warning(
                "Ignoring NED position in frame %r; required formal frame is %r."
                % (pose.frame_id, required_frame)
            )
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
        self._publish_field_coarse_center()
        self._try_begin_center_search()

    def _publish_field_coarse_center(self) -> None:
        """Publish one real, pose-relative coarse center for the field run."""

        if not self.field_direct_dock or self.field_coarse_published:
            return
        if self.latest_current_pose is None or self.latest_frame_id is None:
            return
        distance = float(self.get_parameter("field_coarse_center_ahead_m").value)
        if not math.isfinite(distance) or distance <= 0.0:
            self.get_logger().error("field_coarse_center_ahead_m must be positive")
            return
        pose = self.latest_current_pose
        payload = {
            "dock_id": str(self.get_parameter("field_dock_id").value),
            "center_ned": {
                "north": pose.x + distance * math.cos(pose.yaw),
                "east": pose.y + distance * math.sin(pose.yaw),
                "down": self.latest_position_ned_m["down"],
            },
            "timestamp": time.time(),
            "valid": True,
            "source": "field_relative_center",
            "mock": False,
            "frame_id": self.latest_frame_id,
        }
        self.field_coarse_payload = payload
        self._publish_field_coarse_payload()
        self.field_coarse_published = True
        self.get_logger().warning(
            "FIELD DIRECT DOCK: coarse center fixed %.2f m ahead of the initial pose"
            % distance
        )

    def _publish_field_coarse_payload(self) -> None:
        """Refresh the fixed field center so the perception lease stays valid."""

        if self.field_coarse_payload is None:
            return
        self.field_coarse_payload["timestamp"] = time.time()
        message = String()
        message.data = json.dumps(self.field_coarse_payload, separators=(",", ":"))
        self.coarse_dock_pub.publish(message)

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
            if self.field_direct_dock:
                if self.flow_manager.state is Task3FlowState.APPROACH_DOCK:
                    self.flow_manager.approach_reached()
                self.get_logger().warning(
                    "FIELD DIRECT DOCK: waiting stationary for a stable lidar entrance"
                )
            elif self.flow_manager.state is Task3FlowState.SEARCH_DOCK:
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
        self._publish_goto_goal(phase, goal)

    def _publish_goto_goal(self, phase: str, goal: Pose2D) -> None:
        """Convert one NED planner goal and hand it to the goto controller."""
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
            required_frame = str(
                self.get_parameter("required_ned_frame_id").value
            ).strip()
            payload_frame = payload.get("frame_id")
            if required_frame and payload_frame != required_frame:
                raise ValueError(
                    "coarse dock frame %r does not match required formal frame %r"
                    % (payload_frame, required_frame)
                )
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
                if (
                    self.field_direct_dock
                    and self.flow_manager.state is not Task3FlowState.IDLE
                ):
                    return
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
            if self.field_direct_dock:
                if self.flow_manager.state is not Task3FlowState.SEARCH_DOCK:
                    return
                stable = self._stable_field_candidate(result)
                if stable is None:
                    return
                result = DockPerceptionResult(
                    dock_id=result.dock_id,
                    entrance_observations=(stable,),
                    timestamp=result.timestamp,
                    source=result.source,
                    valid=result.valid,
                    valid_until=result.valid_until,
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
            accepted = self.flow_manager.accept_perception_result(
                result,
                current_pose,
                now=time.time(),
            )
            if accepted and self.field_direct_dock:
                plan = self.flow_manager.accept_color_observation(
                    BayColorObservation(
                        str(self.get_parameter("target_color").value),
                        1.0,
                        time.time(),
                        "single_dock_field_mode",
                    ),
                    now=time.time(),
                )
                if plan is None or not plan.waypoints:
                    raise RuntimeError("terminal dock plan was not generated")
                self.get_logger().warning(
                    "FIELD DIRECT DOCK: stable entrance accepted; publishing final stop point"
                )
                self._publish_goto_goal("TERMINAL_DOCKING", plan.waypoints[-1])
        except (KeyError, TypeError, ValueError, RuntimeError) as exc:
            self.get_logger().warning(f"Ignoring invalid dock perception message: {exc}")

    def _on_dock_led(self, msg: LEDObservation) -> None:
        """Use the camera indicator only for the bay currently under inspection.

        Entrance geometry remains owned by the lidar flow.  The image-space
        position in ``LEDObservation.pose`` is deliberately ignored because it
        is not a navigation-frame coordinate.
        """

        if self.flow_manager.state is not Task3FlowState.CHECK_BAY_COLOR:
            return
        color_names = {
            LEDObservation.COLOR_RED: "RED",
            LEDObservation.COLOR_GREEN: "GREEN",
        }
        color = color_names.get(int(msg.color))
        if color is None:
            return
        timestamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        now = time.time()
        timeout = float(self.get_parameter("dock_led_timeout_s").value)
        if timestamp <= 0.0 or now - timestamp > timeout or timestamp - now > 0.1:
            self.get_logger().warning("Ignoring stale or invalid dock indicator observation")
            return
        sequence_window = float(
            self.get_parameter("dock_led_sequence_window_s").value
        )
        if (
            self.dock_led_candidate_color != color
            or self.dock_led_last_timestamp is None
            or timestamp - self.dock_led_last_timestamp > sequence_window
            or timestamp < self.dock_led_last_timestamp
        ):
            self.dock_led_candidate_color = color
            self.dock_led_detection_count = 1
        else:
            self.dock_led_detection_count += 1
        self.dock_led_last_timestamp = timestamp
        required = max(
            1, int(self.get_parameter("dock_led_required_detections").value)
        )
        self.get_logger().info(
            "Dock indicator stable samples %d/%d: %s"
            % (self.dock_led_detection_count, required, color)
        )
        if self.dock_led_detection_count < required:
            return
        self.dock_led_candidate_color = None
        self.dock_led_detection_count = 0
        self.dock_led_last_timestamp = None
        try:
            plan = self.flow_manager.accept_color_observation(
                BayColorObservation(
                    color=color,
                    confidence=float(msg.confidence),
                    timestamp=timestamp,
                    source=str(self.get_parameter("dock_led_topic").value),
                ),
                now=now,
            )
        except (TypeError, ValueError, RuntimeError) as exc:
            self.get_logger().warning(f"Ignoring invalid dock indicator observation: {exc}")
            return
        if plan is not None and plan.waypoints:
            self.get_logger().warning(
                "Target bay indicator accepted; publishing terminal docking stop point"
            )
            self._publish_goto_goal("TERMINAL_DOCKING", plan.waypoints[-1])
        elif self.flow_manager.state is Task3FlowState.NEXT_BAY:
            self.get_logger().info(
                "Non-target bay indicator observed; exit/next-bay motion remains with the planner"
            )

    def _stable_field_candidate(
        self, result: DockPerceptionResult
    ) -> Optional[DockEntranceObservationNED]:
        """Require repeated geometry and orient the entrance into the bay."""

        candidates = [
            item for item in result.entrance_observations
            if item.valid and item.confidence >= 0.20
        ]
        now = time.time()
        window = float(self.get_parameter("field_detection_window_s").value)
        if not candidates or self.latest_current_pose is None:
            if (
                self.field_last_candidate_time is not None
                and now - self.field_last_candidate_time > window
            ):
                self.field_detection_count = 0
                self.field_last_candidate = None
                self.field_last_candidate_time = None
            return None
        candidate = max(candidates, key=lambda item: item.confidence)
        pose = self.latest_current_pose
        delta_north = candidate.entrance_center_north_m - pose.x
        delta_east = candidate.entrance_center_east_m - pose.y
        distance = math.hypot(delta_north, delta_east)
        forward = delta_north * math.cos(pose.yaw) + delta_east * math.sin(pose.yaw)
        lateral = -delta_north * math.sin(pose.yaw) + delta_east * math.cos(pose.yaw)
        coarse_offset = math.inf
        if self.latest_coarse_dock_center is not None:
            coarse_offset = math.hypot(
                candidate.entrance_center_north_m
                - self.latest_coarse_dock_center.center_ned.north,
                candidate.entrance_center_east_m
                - self.latest_coarse_dock_center.center_ned.east,
            )
        if (
            forward <= 0.25
            or distance > float(
                self.get_parameter("field_max_entrance_distance_m").value
            )
            or abs(lateral) > float(
                self.get_parameter("field_max_lateral_offset_m").value
            )
            or coarse_offset > float(
                self.get_parameter("field_max_coarse_offset_m").value
            )
        ):
            self.field_detection_count = 0
            self.field_last_candidate = None
            self.field_last_candidate_time = None
            self.get_logger().warning(
                "FIELD DIRECT DOCK: rejected implausible entrance "
                "distance=%.2f forward=%.2f lateral=%.2f coarse_offset=%.2f"
                % (distance, forward, lateral, coarse_offset)
            )
            return None
        travel_heading = math.atan2(
            candidate.entrance_center_east_m - pose.y,
            candidate.entrance_center_north_m - pose.x,
        )
        heading = candidate.entrance_heading_rad
        if math.cos(heading - travel_heading) < 0.0:
            heading = (heading + math.pi + math.pi) % (2.0 * math.pi) - math.pi
        candidate = DockEntranceObservationNED(
            candidate.entrance_center_north_m,
            candidate.entrance_center_east_m,
            heading,
            candidate.confidence,
            candidate.timestamp,
            candidate.source,
            candidate.valid,
            candidate.dock_id,
        )
        previous = self.field_last_candidate
        if previous is None:
            self.field_detection_count = 1
        else:
            position_delta = math.hypot(
                candidate.entrance_center_north_m - previous.entrance_center_north_m,
                candidate.entrance_center_east_m - previous.entrance_center_east_m,
            )
            heading_delta = abs(
                (candidate.entrance_heading_rad - previous.entrance_heading_rad + math.pi)
                % (2.0 * math.pi) - math.pi
            )
            if (
                position_delta <= float(
                    self.get_parameter("field_detection_position_tolerance_m").value
                )
                and heading_delta <= float(
                    self.get_parameter("field_detection_heading_tolerance_rad").value
                )
            ):
                self.field_detection_count += 1
            else:
                self.field_detection_count = 1
        self.field_last_candidate = candidate
        self.field_last_candidate_time = now
        required = int(self.get_parameter("field_required_detections").value)
        self.get_logger().info(
            "FIELD DIRECT DOCK: stable entrance samples %d/%d"
            % (self.field_detection_count, required)
        )
        return candidate if self.field_detection_count >= max(1, required) else None

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
        elif self.active_goto_phase == "TERMINAL_DOCKING":
            try:
                self.flow_manager.docking_complete()
                self.get_logger().warning(
                    "FIELD DIRECT DOCK: final stop reached; state is DOCKED"
                )
            except RuntimeError as exc:
                self.get_logger().warning(f"Ignoring goto DONE for docking: {exc}")
            self.active_goto_phase = None

    def _publish_state(self) -> None:
        if self.field_direct_dock and self.field_coarse_payload is not None:
            self._publish_field_coarse_payload()
        current_state: Task3FlowState = self.flow_manager.state
        self.docked_confirmed_pub.publish(
            Bool(
                data=current_state
                in (Task3FlowState.DOCKED, Task3FlowState.WATERING, Task3FlowState.FINISHED)
            )
        )
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
