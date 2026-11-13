"""Connect the deployed Task 3 perception/actuator topics to the USV-only FSM.

This node deliberately contains no mission planning.  It translates the
messages already produced by the OAK, waypoint controller, gimbal and pump
stacks into the small completion inputs consumed by ``usv_only_task3``.
"""

from __future__ import annotations

import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String

from robotx_gimbal_interfaces.msg import GimbalStatus
from robotx_msgs.msg import LEDObservation


DOCKED_STATES = {"HOLD_POSITION", "AIM_WINDOW", "SPRAY_WATER", "REPORT_RESULT"}


def _state_fields(raw: str) -> dict[str, str]:
    return {
        key.strip().upper(): value.strip().upper()
        for part in raw.split(";")
        if "=" in part
        for key, value in [part.split("=", 1)]
        if key.strip()
    }


class UsvOnlyContractAdapter(Node):
    """Publish the four completion inputs required by the USV-only executor."""

    def __init__(self) -> None:
        super().__init__("task3_usv_only_contract_adapter")

        self.stop_linear_speed_mps = float(
            self.declare_parameter("stop_linear_speed_mps", 0.15).value
        )
        self.stop_angular_speed_rps = float(
            self.declare_parameter("stop_angular_speed_rps", 0.10).value
        )
        self.stop_confirmation_s = float(
            self.declare_parameter("stop_confirmation_s", 2.0).value
        )
        self.odom_timeout_s = float(
            self.declare_parameter("odom_timeout_s", 0.75).value
        )
        self.aim_error_tolerance_m = float(
            self.declare_parameter("aim_error_tolerance_m", 0.10).value
        )
        self.aim_lock_frames = int(
            self.declare_parameter("aim_lock_frames", 10).value
        )
        if min(
            self.stop_linear_speed_mps,
            self.stop_angular_speed_rps,
            self.stop_confirmation_s,
            self.odom_timeout_s,
            self.aim_error_tolerance_m,
        ) < 0.0 or self.aim_lock_frames < 1:
            raise ValueError("Task3 contract adapter parameters must be non-negative")

        self.task3_state = ""
        self.task4_active = False
        self.last_odom_at: float | None = None
        self.linear_speed_mps = math.inf
        self.angular_speed_rps = math.inf
        self.stopped_since: float | None = None
        self.hold_sent = False
        self.docked = False
        self.aim_lock_count = 0
        self.gimbal_ready_sent = False
        self.window_id: int | None = None
        self.spray_complete_sent = False

        self.bay_pub = self.create_publisher(
            String, "/tongji_mrl/twave/task3/bay_observation", 10
        )
        self.hold_pub = self.create_publisher(
            Bool, "/robotx/task3/hold_confirmed", 10
        )
        self.docked_pub = self.create_publisher(
            Bool, "/robotx/task3/docked_confirmed", 10
        )
        self.gimbal_ready_pub = self.create_publisher(
            String, "/tongji_mrl/twave/task3/gimbal_ready", 10
        )
        self.spray_complete_pub = self.create_publisher(
            Bool, "/tongji_mrl/twave/task3/spray_complete", 10
        )

        self.create_subscription(
            String, "/tongji_mrl/twave/task3/state", self._on_task3_state, 10
        )
        self.create_subscription(
            LEDObservation, "/robotx/perception/leds", self._on_bay_led, 10
        )
        self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(GimbalStatus, "/gimbal/status", self._on_gimbal, 10)
        self.create_subscription(
            String,
            "/robotx/perception/sprayer/task_state",
            self._on_window_task_state,
            10,
        )
        self.create_subscription(
            String,
            "/tongji_mrl/twave/task3/sprayer_window",
            self._on_sprayer_window,
            10,
        )
        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._publish_docked_gate)
        self.get_logger().info(
            "Task3 contract adapter ready: bay, hold, gimbal and spray completion"
        )

    def _on_task3_state(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            state = str(value.get("state", "")).upper()
        except (AttributeError, json.JSONDecodeError):
            return
        task4_active = bool(value.get("task4_keep_out_active", False))
        if state == self.task3_state and task4_active == self.task4_active:
            return
        self.task3_state = state
        self.task4_active = task4_active
        if state != "HOLD_POSITION":
            self.stopped_since = None
        if state not in DOCKED_STATES or task4_active:
            self.docked = False
            self.hold_sent = False
            self.gimbal_ready_sent = False
            self.spray_complete_sent = False
            self.aim_lock_count = 0

    def _on_bay_led(self, message: LEDObservation) -> None:
        if self.task4_active or self.task3_state != "INSPECT_BAYS":
            return
        names = {
            int(LEDObservation.COLOR_RED): "RED",
            int(LEDObservation.COLOR_GREEN): "GREEN",
        }
        color = names.get(int(message.color))
        confidence = float(message.confidence)
        if color is None or not math.isfinite(confidence):
            return
        confidence = min(1.0, max(0.0, confidence))
        self.bay_pub.publish(String(data=json.dumps({
            "color": color,
            "confidence": confidence,
            "source": "robotx_perception_leds",
        }, separators=(",", ":"))))

    def _on_odom(self, message: Odometry) -> None:
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        values = (linear.x, linear.y, linear.z, angular.x, angular.y, angular.z)
        if not all(math.isfinite(float(value)) for value in values):
            return
        self.linear_speed_mps = math.sqrt(
            float(linear.x) ** 2 + float(linear.y) ** 2 + float(linear.z) ** 2
        )
        self.angular_speed_rps = math.sqrt(
            float(angular.x) ** 2 + float(angular.y) ** 2 + float(angular.z) ** 2
        )
        self.last_odom_at = time.monotonic()

    def _on_window_task_state(self, message: String) -> None:
        active = _state_fields(message.data).get("ACTIVE")
        if active == "LEFT":
            self.window_id = 1
        elif active == "RIGHT":
            self.window_id = 2

    def _on_gimbal(self, message: GimbalStatus) -> None:
        if self.task4_active or self.task3_state != "AIM_WINDOW" or self.gimbal_ready_sent:
            self.aim_lock_count = 0
            return
        ready = (
            bool(message.online)
            and not bool(message.emergency_stop)
            and bool(message.target_valid)
            and bool(message.depth_valid)
            and bool(message.ballistic_reachable)
            and bool(message.aim_error_valid)
            and bool(message.angle_sync_valid)
            and abs(float(message.aim_error_x_m)) <= self.aim_error_tolerance_m
            and abs(float(message.aim_error_y_m)) <= self.aim_error_tolerance_m
        )
        self.aim_lock_count = self.aim_lock_count + 1 if ready else 0
        if self.aim_lock_count < self.aim_lock_frames:
            return
        payload: dict[str, object] = {
            "ready": True,
            "source": "gimbal_status_stable_lock",
        }
        if self.window_id in (1, 2):
            payload["window_id"] = self.window_id
        self.gimbal_ready_pub.publish(
            String(data=json.dumps(payload, separators=(",", ":")))
        )
        self.gimbal_ready_sent = True
        self.get_logger().info("Stable gimbal lock confirmed for Task3")

    def _on_sprayer_window(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        observed_window = value.get("window_id")
        if observed_window in (1, 2):
            self.window_id = int(observed_window)
        try:
            green_duration_s = float(value.get("green_confirmed_duration_s", 0.0))
        except (TypeError, ValueError):
            return
        complete = (
            not self.task4_active
            and self.task3_state == "SPRAY_WATER"
            and not self.spray_complete_sent
            and str(value.get("color", "")).upper() == "GREEN"
            and value.get("pump_off") is True
            and value.get("completion_confirmed") is True
            and green_duration_s >= 5.0
        )
        if complete:
            self.spray_complete_pub.publish(Bool(data=True))
            self.spray_complete_sent = True
            self.get_logger().info("Validated Task3 spray completion published")

    def _tick(self) -> None:
        if self.task4_active or self.task3_state != "HOLD_POSITION" or self.hold_sent:
            return
        now = time.monotonic()
        fresh = self.last_odom_at is not None and now - self.last_odom_at <= self.odom_timeout_s
        stopped = (
            fresh
            and self.linear_speed_mps <= self.stop_linear_speed_mps
            and self.angular_speed_rps <= self.stop_angular_speed_rps
        )
        if not stopped:
            self.stopped_since = None
            return
        if self.stopped_since is None:
            self.stopped_since = now
            return
        if now - self.stopped_since < self.stop_confirmation_s:
            return
        self.hold_pub.publish(Bool(data=True))
        self.docked = True
        self.hold_sent = True
        self._publish_docked_gate()
        self.get_logger().info("USV stop confirmed; DockingReport gate released")

    def _publish_docked_gate(self) -> None:
        self.docked_pub.publish(Bool(data=self.docked))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UsvOnlyContractAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
