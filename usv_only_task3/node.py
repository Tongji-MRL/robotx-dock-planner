"""Thin ROS 2 adapter for the USV-only Task 3 flow.

Mission, perception and communication payloads use JSON String messages.  The
team's existing single-waypoint controller is an exception: it consumes a
``geometry_msgs/PointStamped`` in local map/ENU coordinates.  Hardware output
remains gated by both ``dry_run`` and ``enable_goto_output``.
"""

from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String

try:
    from mavros_msgs.msg import State as MavrosState
except ImportError:  # Allows pure/unit-test environments without MAVROS.
    MavrosState = None

from dock_planner import (
    DockEntranceObservationNED,
    GeofenceSearchConfig,
    GpsOrigin,
    GotoReferenceSample,
    NEDPoint,
    Pose2D,
    geodetic_point_to_ned,
    ned_goal_to_goto_enu,
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


def _course_gps_point(value: dict) -> GpsOrigin:
    """Parse one official course-config GPS point."""

    if not isinstance(value, dict):
        raise ValueError("GPS point must be an object")
    latitude = value.get("latitude")
    longitude = value.get("longitude")
    return GpsOrigin(
        _number(latitude, "latitude"),
        _number(longitude, "longitude"),
        0.0,
    )


def _course_corners(value: dict) -> tuple[GpsOrigin, ...]:
    """Return the four distinct GPS corners from the official course config."""

    raw = value.get("corners")
    if not isinstance(raw, list):
        raise ValueError("course_config.corners must be a list")
    if not all(isinstance(item, dict) for item in raw):
        raise ValueError("course_config.corners must contain GPS objects")
    points = tuple(_course_gps_point(item) for item in raw)
    if len(points) != 4 or len(set(points)) != 4:
        raise ValueError("course configuration must contain four distinct corners")
    return points


def _ned_heading_from_enu_yaw(yaw_enu_rad: float) -> float:
    """Convert ROS map/ENU yaw into clockwise-from-north NED heading."""

    return (math.pi / 2.0 - yaw_enu_rad + math.pi) % (2.0 * math.pi) - math.pi


class UsvOnlyTask3Executor(Node):
    def __init__(self) -> None:
        super().__init__("usv_task3_executor")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("enable_goto_output", False)
        self.declare_parameter("required_ned_frame_id", "robotx_rtk_ned")
        self.declare_parameter("pose_timeout_s", 3.0)
        self.declare_parameter("max_retries_per_state", 2)
        self.declare_parameter("max_search_passes", 2)
        # Three metres keeps the default course search and keep-out decisions
        # away from nominal GPS boundaries when RTK corrections are absent.
        # Both remain launch parameters because a small course may require a
        # measured, explicitly reviewed value.
        self.declare_parameter("geofence_safety_inset_m", 3.0)
        self.declare_parameter("keep_out_safety_clearance_m", 3.0)
        self.declare_parameter("keep_out_detour_margin_m", 1.0)
        self.declare_parameter("keep_out_arc_spacing_m", 1.0)
        self.declare_parameter("auto_verify_stable_dock", True)
        self.declare_parameter("enable_aux_output", False)
        self.declare_parameter("team_id", "TONG")
        self.declare_parameter("vehicle_id", "usv")
        self.declare_parameter("report_seq_start", 0)
        self.declare_parameter("goto_target_topic", "/safe_passage/goto/target")
        self.declare_parameter("goto_status_topic", "/safe_passage/goto/status")
        self.declare_parameter("goto_odom_topic", "/mavros/local_position/odom")
        self.declare_parameter("goto_frame_id", "map")
        self.declare_parameter("goto_max_reference_skew_s", 0.5)
        self.declare_parameter("goto_cancel_topic", "/safe_passage/goto/abort")
        self.declare_parameter("mavros_global_fix_topic", "/mavros/global_position/global")
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
            geofence=GeofenceSearchConfig(
                safety_inset_m=float(
                    self.get_parameter("geofence_safety_inset_m").value
                )
            ),
            keep_out_safety_clearance_m=float(
                self.get_parameter("keep_out_safety_clearance_m").value
            ),
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
        self._odom_east_m: float | None = None
        self._odom_north_m: float | None = None
        self._odom_timestamp: float | None = None
        self._mavros_fix: GpsOrigin | None = None
        self._mavros_fix_timestamp: float | None = None
        self._mavros_heading_ned: float | None = None
        self._mavros_connected = False
        self._active_task4_command_seq: int | None = None
        self._last_task4_command_seq: int | None = None
        self._active_task4_run_id: int | None = None
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
        self._rtk_origin: GpsOrigin | None = None
        self._pending_keep_out: dict | None = None
        self._keep_out_revision = 0

        self.create_subscription(String, "/tongji_mrl/twave/task_cmd", self._on_command, 10)
        self.create_subscription(String, "/tongji_mrl/twave/task3/navigation_constraint",
                                 self._on_navigation_constraint, 10)
        self.create_subscription(
            String, "/tongji_mrl/twave/course_config", self._on_course_config, 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("goto_odom_topic").value),
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix,
            str(self.get_parameter("mavros_global_fix_topic").value),
            self._on_mavros_global_fix,
            qos_profile_sensor_data,
        )
        if MavrosState is not None:
            self.create_subscription(MavrosState, "/mavros/state", self._on_mavros_state, 10)
        else:
            self.get_logger().warning(
                "mavros_msgs unavailable; Task3 cannot verify the FCU connection"
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
        self.ned_pose_pub = self.create_publisher(String, "/tongji_mrl/twave/ned_position", 10)
        self.goto_pub = self.create_publisher(
            PointStamped, str(self.get_parameter("goto_target_topic").value), 10
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
        # The competition launch itself is the Task3 start signal.  It waits
        # safely here until course corners, MAVROS GPS, odometry and FCU state
        # have all arrived.
        self.manager.start(time.time())
        self.create_timer(0.1, self._tick)

    def _warn(self, label: str, exc: Exception) -> None:
        self.get_logger().warning(f"Ignoring invalid {label}: {exc}")

    def _on_command(self, msg: String) -> None:
        try:
            value = _object(msg.data)
            if str(value.get("vehicle", "usv")).lower() != "usv":
                return
            task = str(value.get("task", "TASK3")).upper()
            if isinstance(value.get("command"), str) and str(value["command"]).upper() in {
                "TASK4_BEGIN", "TASK4_KEEP_OUT", "TASK4_KEEP_OUT_ZONE", "TASK4_ALL_CLEAR"
            }:
                task = "TASK4"
            now = time.time()
            if task == "TASK4":
                raw = value.get("command", {})
                parameters = value.get("parameters", {})
                if parameters is None:
                    parameters = {}
                if not isinstance(parameters, dict):
                    raise ValueError("TASK4 parameters must be an object")
                if isinstance(raw, str):
                    kind = raw.upper()
                    command = dict(parameters)
                elif isinstance(raw, dict):
                    kind = str(raw.get("type", raw.get("command", ""))).upper()
                    command = dict(parameters)
                    command.update(raw)
                else:
                    raise ValueError("TASK4 command must be a string or object")
                if kind == "TASK4_BEGIN":
                    if not any(
                        key in command for key in ("command_seq", "center", "radius_m")
                    ):
                        # Retain the older Task4 Core checkpoint handshake.  It
                        # has no keep-out geometry or official command sequence.
                        self._pause_core(now)
                        self._publish()
                        return
                    command_seq = command.get("command_seq")
                    if (isinstance(command_seq, bool)
                            or not isinstance(command_seq, int)
                            or command_seq < 1):
                        raise ValueError("TASK4_BEGIN command_seq must be a positive integer")
                    if (self._last_task4_command_seq is not None
                            and command_seq < self._last_task4_command_seq):
                        raise ValueError("TASK4_BEGIN command_seq is older than the last command")
                    if (self._active_task4_command_seq is None
                            and command_seq == self._last_task4_command_seq):
                        # A delayed duplicate after resume must not reactivate
                        # a zone which has already been cleared.
                        self._publish_incident_ack(command_seq)
                        self._publish()
                        return
                    accepted = self._accept_keep_out(command, value, now=now)
                    if accepted is None:
                        return
                    run_id, _revision = accepted
                    self._active_task4_command_seq = command_seq
                    self._last_task4_command_seq = command_seq
                    self._active_task4_run_id = run_id
                    self._last_goal = None
                    self._goal_running = False
                    self._publish_incident_ack(command_seq)
                elif kind in {"TASK4_KEEP_OUT", "TASK4_KEEP_OUT_ZONE"}:
                    self._accept_keep_out(command, value, now=now)
                    self._last_goal = None
                    self._goal_running = False
                elif kind == "TASK4_ALL_CLEAR":
                    self._apply_all_clear(command, value, now=now)
                    self._last_goal = None
                    self._goal_running = False
                elif kind in {"RESUME", "TASK4_RESUME"}:
                    command_seq = (
                        self._active_task4_command_seq
                        if self._active_task4_command_seq is not None
                        else self._last_task4_command_seq
                    )
                    if command_seq is None:
                        raise ValueError("TASK4 resume received before TASK4_BEGIN")
                    if self._active_task4_command_seq is not None:
                        resume_command = {
                            "run_id": self._active_task4_run_id or 1,
                            "vehicle_type": "USV",
                        }
                        self._apply_all_clear(resume_command, {}, now=now)
                        self._last_goal = None
                        self._goal_running = False
                        self._active_task4_command_seq = None
                        self._active_task4_run_id = None
                    # Resume carries no new sequence in the agreed interface,
                    # so acknowledge it with the TASK4_BEGIN sequence.
                    self._publish_incident_ack(command_seq)
                else:
                    return
                self._publish()
                return
            if task != "TASK3":
                return
            command = str(value.get("command", value.get("type", ""))).upper()
            if command in {"START", "BEGIN", "START_TASK3", "TASK3_BEGIN"}:
                if self.manager.state not in {UsvOnlyTask3State.IDLE, UsvOnlyTask3State.FINISHED}:
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

    @staticmethod
    def _is_usv_vehicle_type(value) -> bool:
        return value is None or value == 1 or str(value).strip().upper() in {
            "1", "USV", "TYPE_USV", "T_WAVE", "T-WAVE"
        }

    def _next_constraint_version(self, command: dict, envelope: dict) -> tuple[int, int]:
        run_id = command.get("run_id", envelope.get("run_id", 1))
        if isinstance(run_id, bool) or not isinstance(run_id, int) or run_id < 1:
            raise ValueError("run_id must be a positive integer")
        supplied = command.get(
            "revision",
            command.get("official_command_seq", command.get("command_seq")),
        )
        if supplied is None:
            self._keep_out_revision += 1
            revision = self._keep_out_revision
        else:
            if isinstance(supplied, bool) or not isinstance(supplied, int) or supplied < 1:
                raise ValueError("official command sequence must be a positive integer")
            self._keep_out_revision = max(self._keep_out_revision, supplied)
            revision = supplied
        return run_id, revision

    def _accept_keep_out(
        self, command: dict, envelope: dict, *, now: float
    ) -> tuple[int, int] | None:
        if not self._is_usv_vehicle_type(command.get("vehicle_type")):
            return None
        center = command.get("center")
        radius = _number(command.get("radius_m"), "radius_m")
        if not isinstance(center, dict) or radius <= 0.0:
            raise ValueError("KeepOutZone requires GPS center and positive radius_m")
        run_id, revision = self._next_constraint_version(command, envelope)
        pending = {
            "run_id": run_id,
            "revision": revision,
            "center": dict(center),
            "radius_m": radius,
        }
        self._pending_keep_out = pending
        if self._rtk_origin is None:
            self.manager.hold_navigation("WAIT_RTK_ORIGIN_FOR_KEEP_OUT", now=now)
            return run_id, revision
        self._apply_pending_keep_out(now=now)
        return run_id, revision

    def _publish_incident_ack(self, command_seq: int) -> None:
        """Return the exact report body forwarded by the main FSM to OCS."""

        message = String()
        message.data = json.dumps({
            "incident_ack": {"command_seq": command_seq},
        }, separators=(",", ":"))
        self.result_pub.publish(message)

    def _apply_pending_keep_out(self, *, now: float) -> None:
        if self._pending_keep_out is None or self._rtk_origin is None:
            return
        value = self._pending_keep_out
        center = value["center"]
        latitude = center.get("latitude", center.get("latitude_deg"))
        longitude = center.get("longitude", center.get("longitude_deg"))
        point = GpsOrigin(
            _number(latitude, "center latitude"),
            _number(longitude, "center longitude"),
            self._rtk_origin.altitude_ellipsoid_m,
        )
        center_ned = geodetic_point_to_ned(point, self._rtk_origin)
        normalized = {
            "run_id": value["run_id"],
            "revision": value["revision"],
            "active": True,
            "frame_id": self.manager.config.required_frame_id,
            "center_ned": {"north": center_ned.north, "east": center_ned.east},
            "radius_m": value["radius_m"],
            "clearance_m": self.manager.config.keep_out_safety_clearance_m,
        }
        self.manager.apply_navigation_constraint(normalized, now=now)
        self._pending_keep_out = None

    def _apply_all_clear(self, command: dict, envelope: dict, *, now: float) -> None:
        if not self._is_usv_vehicle_type(command.get("vehicle_type")):
            return
        run_id, revision = self._next_constraint_version(command, envelope)
        self._pending_keep_out = None
        self.manager.apply_navigation_constraint({
            "run_id": run_id,
            "revision": revision,
            "active": False,
        }, now=now)

    def _on_navigation_constraint(self, msg: String) -> None:
        try:
            if self.manager.apply_navigation_constraint(_object(msg.data), now=time.time()):
                self._last_goal = None
                self._goal_running = False
        except (TypeError, ValueError) as exc:
            self.manager.hold_navigation("INVALID_NAVIGATION_CONSTRAINT", now=time.time())
            self._warn("navigation constraint", exc)
        self._publish()

    def _apply_course_config(self, value: dict) -> None:
        """Use the official GPS course_config as the Task3 geofence."""

        gps_corners = _course_corners(value)
        # Task3 is USV-only: the first official corner establishes one stable
        # NED reference for the current run, shared by local GPS and Task4 GPS.
        self._rtk_origin = gps_corners[0]
        self._corners = tuple(
            geodetic_point_to_ned(point, self._rtk_origin)
            for point in gps_corners
        )
        now = time.time()
        self._update_pose_from_mavros()
        self._try_begin(now)
        self._apply_pending_keep_out(now=now)

    def _on_course_config(self, msg: String) -> None:
        try:
            self._apply_course_config(_object(msg.data))
        except (TypeError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
            self._warn("course config", exc)

    def _on_mavros_global_fix(self, msg: NavSatFix) -> None:
        """Use the USV RTK/GNSS solution locally; master never relays FCU pose."""

        try:
            # RTK is preferred but deliberately not required.  Accept every
            # valid GNSS fix (standalone/SBAS/GBAS/RTK as represented by
            # MAVROS), while refusing an explicit no-fix sample.
            if int(msg.status.status) == int(NavSatStatus.STATUS_NO_FIX):
                self.get_logger().warning("Ignoring MAVROS GPS sample without a fix")
                return
            altitude = float(msg.altitude)
            if not math.isfinite(altitude):
                altitude = 0.0 if self._rtk_origin is None else self._rtk_origin.altitude_ellipsoid_m
            self._mavros_fix = GpsOrigin(
                _number(msg.latitude, "MAVROS latitude"),
                _number(msg.longitude, "MAVROS longitude"),
                altitude,
            )
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            self._mavros_fix_timestamp = stamp if stamp > 0.0 else time.time()
            self._update_pose_from_mavros()
        except (TypeError, ValueError) as exc:
            self._warn("MAVROS global position", exc)

    def _update_pose_from_mavros(self) -> None:
        if (
            self._rtk_origin is None
            or self._mavros_fix is None
            or self._mavros_fix_timestamp is None
            or self._mavros_heading_ned is None
        ):
            return
        position = geodetic_point_to_ned(self._mavros_fix, self._rtk_origin)
        self._pose = Pose2D(position.north, position.east, self._mavros_heading_ned)
        previous = self._health
        self._health = UsvHealth(
            pose_timestamp=self._mavros_fix_timestamp,
            frame_id=self.manager.config.required_frame_id,
            mavros_connected=self._mavros_connected,
            manual_takeover=False if previous is None else previous.manual_takeover,
        )
        message = String()
        message.data = json.dumps({
            "position_ned_m": {
                "north": position.north,
                "east": position.east,
                "down": position.down,
            },
            "heading_rad": self._mavros_heading_ned,
            "timestamp": self._mavros_fix_timestamp,
            "frame_id": self.manager.config.required_frame_id,
            "source": "mavros_global_position_and_odom",
        }, separators=(",", ":"))
        self.ned_pose_pub.publish(message)
        now = time.time()
        if self.manager.state is UsvOnlyTask3State.WAIT_INPUTS:
            self._try_begin(now)
        else:
            self.manager.update_health(self._health, self._pose, now=now)

    def _on_odom(self, msg: Odometry) -> None:
        """Cache the local map/ENU position paired with the RTK NED pose."""

        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        orientation = msg.pose.pose.orientation
        values = (
            msg.pose.pose.position.x, msg.pose.pose.position.y, stamp,
            orientation.x, orientation.y, orientation.z, orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().warning("Ignoring non-finite MAVROS odometry sample")
            return
        self._odom_east_m = float(msg.pose.pose.position.x)
        self._odom_north_m = float(msg.pose.pose.position.y)
        self._odom_timestamp = stamp if stamp > 0.0 else time.time()
        # ROS map is ENU: x east, y north, yaw CCW from east.  Task3 uses NED
        # heading: clockwise from north.
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self._mavros_heading_ned = _ned_heading_from_enu_yaw(math.atan2(siny_cosp, cosy_cosp))
        self._update_pose_from_mavros()

    def _on_mavros_state(self, msg) -> None:
        self._mavros_connected = bool(msg.connected)
        previous = self._health
        if previous is None:
            return
        self._health = UsvHealth(
            previous.pose_timestamp,
            previous.frame_id,
            self._mavros_connected,
            previous.manual_takeover,
        )
        if self._pose is not None:
            now = time.time()
            if self.manager.state is UsvOnlyTask3State.WAIT_INPUTS:
                self._try_begin(now)
            else:
                self.manager.update_health(self._health, self._pose, now=now)

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
        keep_out_stopped = self.manager.keep_out_operation_suspended
        if (state is UsvOnlyTask3State.AIM_WINDOW
                and not keep_out_stopped and not self._gimbal_active):
            self._publish_action(self.gimbal_request_pub, {
                "action": "AIM_WINDOW",
                "bay_id": self.manager.bay_id,
            })
            self._gimbal_active = True
        if (state is UsvOnlyTask3State.SPRAY_WATER
                and not keep_out_stopped and not self._pump_started):
            self._publish_action(self.spray_request_pub, {
                "action": "START",
                "bay_id": self.manager.bay_id,
                "window_id": self.manager.window_id,
            })
            self._pump_started = True
        if ((state is not UsvOnlyTask3State.SPRAY_WATER or keep_out_stopped)
                and self._pump_started):
            self._publish_action(self.spray_request_pub, {"action": "STOP"})
            self._pump_started = False
        if (state not in {
            UsvOnlyTask3State.AIM_WINDOW,
            UsvOnlyTask3State.SPRAY_WATER,
        } or keep_out_stopped) and self._gimbal_active:
            self._publish_action(self.gimbal_request_pub, {"action": "STOP"})
            self._gimbal_active = False

    def _publish(self) -> None:
        if self.manager.current_goal is not None:
            self.manager.guard_current_goal(now=time.time())
        status = self.manager.status()
        status["task4_command_seq"] = (
            self._active_task4_command_seq
            if self._active_task4_command_seq is not None
            else self._last_task4_command_seq
        )
        status["task4_keep_out_active"] = self._active_task4_command_seq is not None
        diagnostic = String()
        diagnostic.data = json.dumps(status, separators=(",", ":"))
        self.state_pub.publish(diagnostic)
        state_key = json.dumps(status, sort_keys=True)
        if state_key != self._last_state:
            progress = String()
            report = self.manager.progress_report()
            for key in ("resume_phase", "core_checkpoint_id", "core_paused", "retry_counts",
                        "navigation_hold_reason", "navigation_constraint",
                        "keep_out_detour_side", "keep_out_detour_waypoints_remaining",
                        "keep_out_inside_official_zone", "keep_out_inside_protected_zone",
                        "keep_out_suspended_state", "keep_out_checkpoint_id",
                        "keep_out_mode", "task4_command_seq", "task4_keep_out_active"):
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
        self._goal_running = False
        if goal is None:
            self._last_goal = None
            return
        payload = {
            "frame_id": self.manager.config.required_frame_id,
            "north_m": goal.x, "east_m": goal.y, "heading_rad": goal.yaw,
            "phase": ("KEEP_OUT_EXIT" if status["keep_out_mode"] == "EXITING"
                      else "KEEP_OUT_DETOUR" if status["keep_out_detour_side"]
                      else self.manager.state.value),
            "task3_phase": self.manager.state.value,
            "keep_out_detour_side": status["keep_out_detour_side"],
            "avoidance_enabled": (
                status["keep_out_mode"] == "EXITING"
                or status["keep_out_detour_side"] is not None
                or self.manager.state is not UsvOnlyTask3State.ENTER_BAY
            ),
            "navigation_constraint": self.manager.navigation_constraint.to_dict(),
        }
        msg = String(); msg.data = json.dumps(payload, separators=(",", ":"))
        self.preview_pub.publish(msg)
        if self.dry_run or not self.enable_goto_output:
            self._last_goal = goal_key
            return
        if (
            self._pose is None
            or self._health is None
            or self._odom_east_m is None
            or self._odom_north_m is None
            or self._odom_timestamp is None
        ):
            self.get_logger().warning(
                "Cannot publish goto target: synchronized NED and MAVROS odom samples are required"
            )
            return
        try:
            target = ned_goal_to_goto_enu(
                goal,
                GotoReferenceSample(
                    current_ned=NEDPoint(self._pose.x, self._pose.y, 0.0),
                    odom_east_m=self._odom_east_m,
                    odom_north_m=self._odom_north_m,
                    ned_timestamp=self._health.pose_timestamp,
                    odom_timestamp=self._odom_timestamp,
                ),
                max_reference_skew_s=float(
                    self.get_parameter("goto_max_reference_skew_s").value
                ),
            )
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Cannot publish goto target: {exc}")
            return
        target_msg = PointStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = str(self.get_parameter("goto_frame_id").value)
        target_msg.point.x = target.east_m
        target_msg.point.y = target.north_m
        target_msg.point.z = target.up_m
        self.goto_pub.publish(target_msg)
        self._last_goal = goal_key
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
