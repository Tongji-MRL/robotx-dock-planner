"""Thin ROS 2 adapter for the USV-only Task 3 flow.

All external payloads are JSON String messages so this adapter can be wired to
the existing shared-position, perception, safe-passage and communication
bridges without changing those packages.  Hardware output remains gated by
both ``dry_run`` and ``enable_goto_output``.
"""

from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    from mavros_msgs.msg import State as MavrosState
except ImportError:  # Allows pure/unit-test environments without MAVROS.
    MavrosState = None

from dock_planner import (
    DockEntranceObservationNED,
    GpsOrigin,
    NEDPoint,
    Pose2D,
    geodetic_point_to_ned,
    parse_goto_status,
)
from usv_only_task3.flow import (
    UsvHealth,
    UsvOnlyTask3Config,
    UsvOnlyTask3FlowManager,
    UsvOnlyTask3State,
)


def _object(raw: str) -> dict:
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError("payload must be a JSON object")
    return value


def _number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


class UsvOnlyTask3Executor(Node):
    def __init__(self) -> None:
        super().__init__("usv_task3_executor")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("enable_goto_output", False)
        self.declare_parameter("required_ned_frame_id", "robotx_rtk_ned")
        self.declare_parameter("pose_timeout_s", 2.0)
        self.declare_parameter("max_retries_per_state", 2)
        self.declare_parameter("max_search_passes", 2)
        self.declare_parameter("goto_target_topic", "/safe_passage/goto/target")
        self.declare_parameter("goto_status_topic", "/safe_passage/goto/status")

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.enable_goto_output = bool(self.get_parameter("enable_goto_output").value)
        self.manager = UsvOnlyTask3FlowManager(UsvOnlyTask3Config(
            required_frame_id=str(self.get_parameter("required_ned_frame_id").value),
            pose_timeout_s=float(self.get_parameter("pose_timeout_s").value),
            max_retries_per_state=int(self.get_parameter("max_retries_per_state").value),
            max_search_passes=int(self.get_parameter("max_search_passes").value),
        ))
        self._corners: tuple[NEDPoint, ...] | None = None
        self._pose: Pose2D | None = None
        self._health: UsvHealth | None = None
        self._last_state = None
        self._last_goal = None

        self.create_subscription(String, "/tongji_mrl/twave/task_cmd", self._on_command, 10)
        self.create_subscription(String, "/tongji_mrl/twave/task3/geofence", self._on_geofence, 10)
        self.create_subscription(String, "/tongji_mrl/twave/ned_position", self._on_pose, 10)
        self.create_subscription(String, "/tongji_mrl/twave/task3/vehicle_health", self._on_health, 10)
        if MavrosState is not None:
            self.create_subscription(MavrosState, "/mavros/state", self._on_mavros_state, 10)
        else:
            self.get_logger().warning(
                "mavros_msgs unavailable; vehicle_health must provide MAVROS connection state"
            )
        self.create_subscription(String, "/tongji_mrl/twave/task3/dock_perception", self._on_dock, 10)
        self.create_subscription(String, str(self.get_parameter("goto_status_topic").value), self._on_goto, 10)
        self.create_subscription(String, "/tongji_mrl/twave/task3/bay_observation", self._on_bay, 10)
        self.create_subscription(Bool, "/robotx/task3/hold_confirmed", self._on_hold, 10)
        self.create_subscription(String, "/tongji_mrl/twave/task3/gimbal_ready", self._on_gimbal, 10)
        self.create_subscription(Bool, "/tongji_mrl/twave/task3/spray_complete", self._on_spray, 10)
        self.create_subscription(Bool, "/tongji_mrl/twave/task3/result_ack", self._on_result_ack, 10)
        self.create_subscription(Bool, "/tongji_mrl/twave/task3/dock_verified", self._on_dock_verified, 10)

        self.state_pub = self.create_publisher(String, "/tongji_mrl/twave/task3/state", 10)
        self.preview_pub = self.create_publisher(String, "/tongji_mrl/twave/task3/goto_preview", 10)
        self.goto_pub = self.create_publisher(
            String, str(self.get_parameter("goto_target_topic").value), 10
        )
        self.release_pub = self.create_publisher(Bool, "/tongji_mrl/twave/task3/release_control", 10)
        self.mode_pub = self.create_publisher(String, "/tongji_mrl/twave/task3/perception_mode", 10)
        self.create_timer(0.1, self._tick)

    def _warn(self, label: str, exc: Exception) -> None:
        self.get_logger().warning(f"Ignoring invalid {label}: {exc}")

    def _on_command(self, msg: String) -> None:
        try:
            value = _object(msg.data)
            if str(value.get("task", "TASK3")).upper() != "TASK3":
                return
            command = str(value.get("command", value.get("type", ""))).upper()
            now = time.time()
            if command in {"START", "START_TASK3", "TASK3_BEGIN"}:
                self.manager.start(now)
                self._try_begin(now)
            elif command in {"PAUSE", "PAUSE_TASK3"}:
                self.manager.pause("PAUSED_BY_MASTER", now=now)
            elif command in {"RESUME", "RESUME_TASK3"}:
                self.manager.resume(now=now)
            elif command in {"ABORT", "SAFE_STOP", "END"}:
                self.manager.abort("ABORTED_BY_MASTER", now=now)
            elif command == "RESET":
                self.manager = UsvOnlyTask3FlowManager(self.manager.config)
            else:
                raise ValueError(f"unsupported command {command!r}")
            self._publish()
        except (TypeError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
            self._warn("task command", exc)

    def _on_geofence(self, msg: String) -> None:
        try:
            value = _object(msg.data)
            frame = str(value.get("frame_id", ""))
            if frame != self.manager.config.required_frame_id:
                raise ValueError(f"geofence frame must be {self.manager.config.required_frame_id}")
            raw = value.get("corners_ned")
            if isinstance(raw, list):
                if len(raw) != 4:
                    raise ValueError("corners_ned must contain four objects")
                self._corners = tuple(NEDPoint(
                    _number(item.get("north"), "north"),
                    _number(item.get("east"), "east"),
                    _number(item.get("down", 0.0), "down"),
                ) for item in raw)
            else:
                raw = value.get("corners_gps")
                origin_raw = value.get("rtk_origin_gps")
                if not isinstance(raw, list) or len(raw) != 4 or not isinstance(origin_raw, dict):
                    raise ValueError("provide corners_ned or four corners_gps plus rtk_origin_gps")
                origin = GpsOrigin.from_mapping(origin_raw)
                self._corners = tuple(
                    geodetic_point_to_ned(GpsOrigin.from_mapping(item), origin)
                    for item in raw
                )
            self._try_begin(time.time())
        except (TypeError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
            self._warn("geofence", exc)

    def _on_pose(self, msg: String) -> None:
        try:
            value = _object(msg.data)
            raw = value.get("position_ned_m", value.get("position_ned"))
            if not isinstance(raw, dict):
                raise ValueError("position_ned_m must be an object")
            self._pose = Pose2D(
                _number(raw.get("north"), "north"),
                _number(raw.get("east"), "east"),
                _number(value.get("heading_rad"), "heading_rad"),
            )
            previous = self._health
            self._health = UsvHealth(
                pose_timestamp=_number(value.get("stamp_unix_sec", value.get("timestamp")), "timestamp"),
                frame_id=str(value.get("frame_id", "")),
                mavros_connected=False if previous is None else previous.mavros_connected,
                manual_takeover=False if previous is None else previous.manual_takeover,
            )
            now = time.time()
            if self.manager.state is UsvOnlyTask3State.WAIT_INPUTS:
                self._try_begin(now)
            else:
                self.manager.update_health(self._health, self._pose, now=now)
        except (TypeError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
            self._warn("NED pose", exc)

    def _on_health(self, msg: String) -> None:
        try:
            value = _object(msg.data)
            if self._health is None:
                return
            self._health = UsvHealth(
                self._health.pose_timestamp,
                self._health.frame_id,
                bool(value.get("mavros_connected", False)),
                bool(value.get("manual_takeover", False)),
            )
            if self._pose is not None:
                self.manager.update_health(self._health, self._pose, now=time.time())
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self._warn("vehicle health", exc)

    def _on_mavros_state(self, msg) -> None:
        previous = self._health
        if previous is None:
            return
        self._health = UsvHealth(
            previous.pose_timestamp,
            previous.frame_id,
            bool(msg.connected),
            previous.manual_takeover,
        )
        if self._pose is not None:
            self.manager.update_health(self._health, self._pose, now=time.time())

    def _try_begin(self, now: float) -> None:
        if (
            self.manager.state is UsvOnlyTask3State.WAIT_INPUTS
            and self._corners is not None and self._pose is not None and self._health is not None
        ):
            self.manager.begin(self._corners, self._pose, self._health, now=now)

    def _on_dock(self, msg: String) -> None:
        try:
            if self._pose is None:
                return
            value = _object(msg.data)
            raw = value.get("entrance_observations", [])
            observations = tuple(DockEntranceObservationNED(
                _number(item.get("entrance_center_north_m"), "entrance north"),
                _number(item.get("entrance_center_east_m"), "entrance east"),
                _number(item.get("entrance_heading_rad"), "entrance heading"),
                _number(item.get("confidence"), "confidence"),
                _number(item.get("timestamp", value.get("timestamp")), "timestamp"),
                str(item.get("source", value.get("source", "mid360"))),
                bool(item.get("valid", True)),
                item.get("dock_id", value.get("dock_id")),
            ) for item in raw)
            self.manager.accept_dock_frame(observations, self._pose, now=time.time())
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self._warn("dock perception", exc)

    def _on_goto(self, msg: String) -> None:
        try:
            status = parse_goto_status(msg.data)
            if status.kind.value == "DONE":
                self.manager.navigation_reached(now=time.time())
            elif status.kind.value == "ABORTED":
                self.manager.pause(f"GOTO_ABORTED:{status.reason}", now=time.time())
        except (TypeError, ValueError, RuntimeError) as exc:
            self._warn("goto status", exc)

    def _on_bay(self, msg: String) -> None:
        try:
            value = _object(msg.data)
            self.manager.accept_bay_color(
                str(value.get("color", "UNKNOWN")),
                _number(value.get("confidence", 0.0), "confidence"),
                now=time.time(),
            )
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self._warn("bay observation", exc)

    def _on_hold(self, msg: Bool) -> None:
        if msg.data:
            try:
                self.manager.hold_confirmed(now=time.time())
            except RuntimeError as exc:
                self._warn("hold confirmation", exc)

    def _on_gimbal(self, msg: String) -> None:
        try:
            value = _object(msg.data)
            self.manager.aiming_ready(
                gimbal_ready=bool(value.get("ready", False)),
                window_id=value.get("window_id"),
                now=time.time(),
            )
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self._warn("gimbal status", exc)

    def _on_spray(self, msg: Bool) -> None:
        if msg.data:
            try:
                self.manager.spray_complete(now=time.time())
            except RuntimeError as exc:
                self._warn("spray completion", exc)

    def _on_result_ack(self, msg: Bool) -> None:
        if msg.data:
            try:
                self.manager.result_reported(now=time.time())
            except RuntimeError as exc:
                self._warn("result ACK", exc)

    def _on_dock_verified(self, msg: Bool) -> None:
        if msg.data:
            try:
                self.manager.dock_verified(now=time.time())
            except RuntimeError as exc:
                self._warn("dock verification", exc)

    def _tick(self) -> None:
        now = time.time()
        self.manager.tick(now=now)
        if (
            self.manager.state is UsvOnlyTask3State.RETRY
            and now - self.manager.state_entered_at >= 0.5
        ):
            self.manager.retry(now=now)
        if self.manager.state is UsvOnlyTask3State.WAIT_INPUTS:
            self._try_begin(now)
        self._publish()

    def _publish(self) -> None:
        status = self.manager.status()
        state_key = json.dumps(status, sort_keys=True)
        if state_key != self._last_state:
            msg = String(); msg.data = json.dumps(status, separators=(",", ":"))
            self.state_pub.publish(msg)
            self._last_state = state_key
        release = Bool(); release.data = self.manager.releases_control
        self.release_pub.publish(release)
        mode = String()
        mode.data = "STOP_SCAN" if self.manager.state in {
            UsvOnlyTask3State.SEARCH_PRIMARY, UsvOnlyTask3State.SEARCH_SECONDARY
        } else "TRACK"
        self.mode_pub.publish(mode)
        goal = self.manager.current_goal
        goal_key = None if goal is None else (goal.x, goal.y, goal.yaw, self.manager.state.value)
        if goal_key == self._last_goal:
            return
        self._last_goal = goal_key
        if goal is None:
            return
        payload = {
            "frame_id": self.manager.config.required_frame_id,
            "north_m": goal.x, "east_m": goal.y, "heading_rad": goal.yaw,
            "phase": self.manager.state.value,
            "avoidance_enabled": self.manager.state is not UsvOnlyTask3State.ENTER_BAY,
        }
        msg = String(); msg.data = json.dumps(payload, separators=(",", ":"))
        self.preview_pub.publish(msg)
        if not self.dry_run and self.enable_goto_output:
            self.goto_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UsvOnlyTask3Executor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ["UsvOnlyTask3Executor", "main"]
