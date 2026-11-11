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
from usv_only_task3.reports import Task3ResultFactory


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
        self.declare_parameter("keep_out_detour_margin_m", 0.5)
        self.declare_parameter("keep_out_arc_spacing_m", 1.0)
        self.declare_parameter("auto_verify_stable_dock", True)
        self.declare_parameter("enable_aux_output", False)
        self.declare_parameter("team_id", "TONG")
        self.declare_parameter("vehicle_id", "usv")
        self.declare_parameter("report_seq_start", 0)
        self.declare_parameter("goto_target_topic", "/safe_passage/goto/target")
        self.declare_parameter("goto_status_topic", "/safe_passage/goto/status")
        self.declare_parameter("goto_cancel_topic", "/safe_passage/goto/cancel")
        self.declare_parameter(
            "gimbal_request_topic", "/tongji_mrl/twave/task3/gimbal_request"
        )
        self.declare_parameter(
            "spray_request_topic", "/tongji_mrl/twave/task3/spray_request"
        )

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.enable_goto_output = bool(self.get_parameter("enable_goto_output").value)
        self.enable_aux_output = bool(self.get_parameter("enable_aux_output").value)
        self.auto_verify_stable_dock = bool(
            self.get_parameter("auto_verify_stable_dock").value
        )
        self.manager = UsvOnlyTask3FlowManager(UsvOnlyTask3Config(
            required_frame_id=str(self.get_parameter("required_ned_frame_id").value),
            pose_timeout_s=float(self.get_parameter("pose_timeout_s").value),
            max_retries_per_state=int(self.get_parameter("max_retries_per_state").value),
            max_search_passes=int(self.get_parameter("max_search_passes").value),
            keep_out_detour_margin_m=float(
                self.get_parameter("keep_out_detour_margin_m").value
            ),
            keep_out_arc_spacing_m=float(
                self.get_parameter("keep_out_arc_spacing_m").value
            ),
        ))
        self._corners: tuple[NEDPoint, ...] | None = None
        self._pose: Pose2D | None = None
        self._health: UsvHealth | None = None
        self._last_state = None
        self._last_goal = None
        self._goal_running = False
        self._navigation_active = False
        self._navigation_cancel_pending = False
        self._pump_started = False
        self._gimbal_active = False
        self._docking_reported = False
        self._firefighting_reported = False
        self._result_factory = Task3ResultFactory(
            team_id=str(self.get_parameter("team_id").value),
            vehicle_id=str(self.get_parameter("vehicle_id").value),
            initial_seq=int(self.get_parameter("report_seq_start").value),
        )

        self.create_subscription(String, "/tongji_mrl/twave/task_cmd", self._on_command, 10)
        self.create_subscription(String, "/tongji_mrl/twave/task3/navigation_constraint",
                                 self._on_navigation_constraint, 10)
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
        self.create_subscription(Bool, "/tongji_mrl/twave/task3/dock_verified", self._on_dock_verified, 10)

        self.state_pub = self.create_publisher(String, "/tongji_mrl/twave/task3/state", 10)
        self.checkpoint_pub = self.create_publisher(String, "/tongji_mrl/twave/checkpoint", 10)
        self.progress_pub = self.create_publisher(String, "/tongji_mrl/twave/progress", 10)
        self.result_pub = self.create_publisher(String, "/tongji_mrl/twave/result", 10)
        self.preview_pub = self.create_publisher(String, "/tongji_mrl/twave/task3/goto_preview", 10)
        self.goto_pub = self.create_publisher(
            String, str(self.get_parameter("goto_target_topic").value), 10
        )
        self.goto_cancel_pub = self.create_publisher(
            Bool, str(self.get_parameter("goto_cancel_topic").value), 10
        )
        self.action_preview_pub = self.create_publisher(
            String, "/tongji_mrl/twave/task3/action_preview", 10
        )
        self.gimbal_request_pub = self.create_publisher(
            String, str(self.get_parameter("gimbal_request_topic").value), 10
        )
        self.spray_request_pub = self.create_publisher(
            String, str(self.get_parameter("spray_request_topic").value), 10
        )
        self.release_pub = self.create_publisher(Bool, "/tongji_mrl/twave/task3/release_control", 10)
        self.mode_pub = self.create_publisher(String, "/tongji_mrl/twave/task3/perception_mode", 10)
        self.create_timer(0.1, self._tick)

    def _warn(self, label: str, exc: Exception) -> None:
        self.get_logger().warning(f"Ignoring invalid {label}: {exc}")

    def _on_command(self, msg: String) -> None:
        try:
            value = _object(msg.data)
            if str(value.get("vehicle", "usv")).lower() != "usv":
                return
            task = str(value.get("task", "TASK3")).upper()
            now = time.time()
            if task == "TASK4":
                raw = value.get("command", {})
                if not isinstance(raw, dict):
                    raise ValueError("TASK4 command must be an object")
                kind = raw.get("type")
                if kind == "TASK4_BEGIN":
                    self._pause_core(now)
                elif kind in {"TASK4_KEEP_OUT", "TASK4_ALL_CLEAR"}:
                    constraint = dict(raw)
                    constraint.pop("type")
                    constraint["active"] = kind == "TASK4_KEEP_OUT"
                    self.manager.apply_navigation_constraint(constraint, now=now)
                    self._last_goal = None
                    self._goal_running = False
                else:
                    return
                self._publish()
                return
            if task != "TASK3":
                return
            command = str(value.get("command", value.get("type", ""))).upper()
            if command in {"START", "BEGIN", "START_TASK3", "TASK3_BEGIN"}:
                if self.manager.state not in {UsvOnlyTask3State.IDLE, UsvOnlyTask3State.COMPLETE}:
                    raise ValueError("active mission requires RESUME or explicit RESET")
                self._docking_reported = False
                self._firefighting_reported = False
                self.manager.start(now)
                self._try_begin(now)
            elif command in {"PAUSE", "PAUSE_TASK3"}:
                self._pause_core(now)
            elif command in {"RESUME", "RESUME_TASK3"}:
                if self.manager.status()["core_paused"]:
                    self.manager.resume_core(value.get("checkpoint_id"),
                        motion_stopped=value.get("motion_stopped"),
                        dock_exit_completed=value.get("dock_exit_completed", False), now=now)
                else:
                    self.manager.resume(now=now)
                self._last_goal = None
                self._goal_running = False
            elif command in {"ABORT", "SAFE_STOP", "END"}:
                self.manager.abort("ABORTED_BY_MASTER", now=now)
            elif command == "RESET":
                self.manager.start(now)
                self.manager._enter(UsvOnlyTask3State.IDLE, now)
                self._last_goal = None
                self._goal_running = False
                self._docking_reported = False
                self._firefighting_reported = False
            else:
                raise ValueError(f"unsupported command {command!r}")
            self._publish()
        except (TypeError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
            self._warn("task command", exc)

    def _pause_core(self, now: float) -> None:
        checkpoint = self.manager.pause_for_core(now=now)
        self._last_goal = None
        self._goal_running = False
        message = String()
        message.data = json.dumps({
            "vehicle": "usv", "task": "TASK3", "command": "checkpoint",
            "progress": checkpoint["saved_step"], "error": None,
            **checkpoint,
        }, separators=(",", ":"))
        self.checkpoint_pub.publish(message)

    def _on_navigation_constraint(self, msg: String) -> None:
        try:
            if self.manager.apply_navigation_constraint(_object(msg.data), now=time.time()):
                self._last_goal = None
                self._goal_running = False
        except (TypeError, ValueError) as exc:
            self.manager.hold_navigation("INVALID_NAVIGATION_CONSTRAINT", now=time.time())
            self._warn("navigation constraint", exc)
        self._publish()

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
            if status.kind.value == "ABORTED" and self._navigation_cancel_pending:
                self._navigation_active = False
                self._navigation_cancel_pending = False
                return
            if self.manager.releases_control or self._last_goal is None:
                return
            if status.kind.value == "RUNNING":
                self._goal_running = True
            if status.kind.value == "DONE" and self._goal_running:
                self._goal_running = False
                self._last_goal = None
                self._navigation_active = False
                self._navigation_cancel_pending = False
                self.manager.navigation_reached(now=time.time())
            elif status.kind.value == "ABORTED":
                self._navigation_active = False
                if self._navigation_cancel_pending:
                    self._navigation_cancel_pending = False
                else:
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
                now = time.time()
                self.manager.hold_confirmed(now=now)
                if not self._docking_reported:
                    self._publish_result(self._result_factory.docking(
                        self.manager.bay_id, now=now
                    ))
                    self._docking_reported = True
                self._publish()
            except RuntimeError as exc:
                self._warn("hold confirmation", exc)
            except ValueError as exc:
                self.manager.fail(f"DOCKING_REPORT_INVALID: {exc}", time.time())

    def _on_gimbal(self, msg: String) -> None:
        try:
            value = _object(msg.data)
            changed = self.manager.aiming_ready(
                gimbal_ready=bool(value.get("ready", False)),
                window_id=value.get("window_id"),
                now=time.time(),
            )
            if changed:
                self._publish()
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self._warn("gimbal status", exc)

    def _on_spray(self, msg: Bool) -> None:
        if msg.data:
            try:
                now = time.time()
                self.manager.spray_complete(now=now)
                if not self._firefighting_reported:
                    self._publish_result(self._result_factory.firefighting(
                        self.manager.window_id, now=now
                    ))
                    self._firefighting_reported = True
                self._publish()
            except RuntimeError as exc:
                self._warn("spray completion", exc)
            except ValueError as exc:
                self.manager.fail(f"FIREFIGHTING_REPORT_INVALID: {exc}", time.time())

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
            self.auto_verify_stable_dock
            and self.manager.dock_verification_ready
        ):
            try:
                self.manager.dock_verified(now=now)
            except RuntimeError as exc:
                self.manager.fail(f"DOCK_LAYOUT_VERIFICATION_FAILED: {exc}", now)
        if (
            self.manager.state is UsvOnlyTask3State.RETRY
            and now - self.manager.state_entered_at >= 0.5
        ):
            self.manager.retry(now=now)
        if self.manager.state is UsvOnlyTask3State.WAIT_INPUTS:
            self._try_begin(now)
        if self.manager.state is UsvOnlyTask3State.REPORT_RESULT:
            if not self._firefighting_reported:
                self.manager.fail("FIREFIGHTING_REPORT_NOT_PUBLISHED", now)
            else:
                try:
                    self.manager.result_reported(now=now)
                except RuntimeError as exc:
                    self.manager.fail(f"FINAL_REPORT_INVALID: {exc}", now)
        self._publish()

    def _publish_result(self, value: dict) -> None:
        message = String()
        message.data = json.dumps(value, separators=(",", ":"))
        self.result_pub.publish(message)

    def _publish_action(self, publisher, payload: dict) -> None:
        preview = String()
        preview.data = json.dumps(payload, separators=(",", ":"))
        self.action_preview_pub.publish(preview)
        if self.dry_run or not self.enable_aux_output:
            return
        message = String()
        message.data = preview.data
        publisher.publish(message)

    def _publish_auxiliary_actions(self) -> None:
        state = self.manager.state
        if state is UsvOnlyTask3State.AIM_WINDOW and not self._gimbal_active:
            self._publish_action(self.gimbal_request_pub, {
                "action": "AIM_WINDOW",
                "bay_id": self.manager.bay_id,
            })
            self._gimbal_active = True
        if state is UsvOnlyTask3State.SPRAY_WATER and not self._pump_started:
            self._publish_action(self.spray_request_pub, {
                "action": "START",
                "bay_id": self.manager.bay_id,
                "window_id": self.manager.window_id,
            })
            self._pump_started = True
        if state is not UsvOnlyTask3State.SPRAY_WATER and self._pump_started:
            self._publish_action(self.spray_request_pub, {"action": "STOP"})
            self._pump_started = False
        if state not in {
            UsvOnlyTask3State.AIM_WINDOW,
            UsvOnlyTask3State.SPRAY_WATER,
        } and self._gimbal_active:
            self._publish_action(self.gimbal_request_pub, {"action": "STOP"})
            self._gimbal_active = False

    def _publish(self) -> None:
        if self.manager.current_goal is not None:
            self.manager.guard_current_goal(now=time.time())
        status = self.manager.status()
        diagnostic = String()
        diagnostic.data = json.dumps(status, separators=(",", ":"))
        self.state_pub.publish(diagnostic)
        state_key = json.dumps(status, sort_keys=True)
        if state_key != self._last_state:
            progress = String()
            report = self.manager.progress_report()
            for key in ("resume_phase", "core_checkpoint_id", "core_paused", "retry_counts",
                        "navigation_hold_reason", "navigation_constraint",
                        "keep_out_detour_side", "keep_out_detour_waypoints_remaining"):
                report[key] = status[key]
            progress.data = json.dumps(report, separators=(",", ":"))
            self.progress_pub.publish(progress)
            self._last_state = state_key
        release = Bool(); release.data = self.manager.releases_control
        self.release_pub.publish(release)
        mode = String()
        mode.data = "STOP_SCAN" if self.manager.state in {
            UsvOnlyTask3State.SEARCH_PRIMARY, UsvOnlyTask3State.SEARCH_SECONDARY
        } else "TRACK"
        self.mode_pub.publish(mode)
        self._publish_auxiliary_actions()
        goal = self.manager.current_goal
        goal_key = None if goal is None else (goal.x, goal.y, goal.yaw, self.manager.state.value)
        if goal is None and self._navigation_active and not self._navigation_cancel_pending:
            cancel = Bool(); cancel.data = True
            self.goto_cancel_pub.publish(cancel)
            self._navigation_cancel_pending = True
            self._navigation_active = False
        if goal_key == self._last_goal:
            return
        self._last_goal = goal_key
        self._goal_running = False
        if goal is None:
            return
        payload = {
            "frame_id": self.manager.config.required_frame_id,
            "north_m": goal.x, "east_m": goal.y, "heading_rad": goal.yaw,
            "phase": ("KEEP_OUT_DETOUR" if status["keep_out_detour_side"]
                      else self.manager.state.value),
            "task3_phase": self.manager.state.value,
            "keep_out_detour_side": status["keep_out_detour_side"],
            "avoidance_enabled": (
                status["keep_out_detour_side"] is not None
                or self.manager.state is not UsvOnlyTask3State.ENTER_BAY
            ),
            "navigation_constraint": self.manager.navigation_constraint.to_dict(),
        }
        msg = String(); msg.data = json.dumps(payload, separators=(",", ":"))
        self.preview_pub.publish(msg)
        if not self.dry_run and self.enable_goto_output:
            self.goto_pub.publish(msg)
            self._navigation_active = True
            self._navigation_cancel_pending = False


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
