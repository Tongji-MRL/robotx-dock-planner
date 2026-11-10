"""Publish reusable reactive-avoidance decisions from the shared LaserScan."""

from dataclasses import asdict
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from usv_reactive_avoidance import ReactiveAvoidanceConfig, ReactiveAvoidanceCore


TERMINAL_STATES = {"TERMINAL_DOCKING", "DOCKED", "AIM_AND_SPRAY", "VERIFY"}


class ReactiveAvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("usv_reactive_avoidance")
        defaults = asdict(ReactiveAvoidanceConfig())
        values = {name: self.declare_parameter(name, value).value for name, value in defaults.items()}
        self.core = ReactiveAvoidanceCore(ReactiveAvoidanceConfig(**values))
        self.scan_timeout_s = float(self.declare_parameter("scan_timeout_s", 0.5).value)
        self.scan_topic = str(self.declare_parameter("scan_topic", "/mavros/obstacle/send").value)
        self.state_topic = str(self.declare_parameter(
            "task_state_topic", "/tongji_mrl/twave/task3/state"
        ).value)
        output_topic = str(self.declare_parameter(
            "output_topic", "/tongji_mrl/twave/navigation/avoidance"
        ).value)
        self.terminal_docking = False
        self.last_scan_at = None
        self.publisher = self.create_publisher(String, output_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, self.state_topic, self._on_state, 10)
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            f"Reactive avoidance monitor: {self.scan_topic} -> {output_topic}; control output disabled"
        )

    def _on_state(self, message: String) -> None:
        try:
            state = json.loads(message.data).get("state")
        except (AttributeError, json.JSONDecodeError):
            return
        self.terminal_docking = state in TERMINAL_STATES

    def _on_scan(self, message: LaserScan) -> None:
        now = time.time()
        self.last_scan_at = time.monotonic()
        result = self.core.evaluate(
            message.ranges,
            angle_min=message.angle_min,
            angle_increment=message.angle_increment,
            range_min=message.range_min,
            range_max=message.range_max,
            timestamp=now,
            terminal_docking=self.terminal_docking,
        )
        payload = asdict(result)
        payload["action"] = result.action.value
        payload["steering_side"] = (
            None if result.steering_side is None else result.steering_side.value
        )
        payload["terminal_docking"] = self.terminal_docking
        self.publisher.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def _watchdog(self) -> None:
        if self.terminal_docking:
            return
        if self.last_scan_at is None or time.monotonic() - self.last_scan_at > self.scan_timeout_s:
            now = time.time()
            self.publisher.publish(String(data=json.dumps({
                "action": "HOLD",
                "timestamp": now,
                "valid_until": now + 0.35,
                "reason": "LaserScan missing or stale",
                "nearest_forward_m": None,
                "steering_side": None,
                "speed_scale": 0.0,
                "obstacle_points": 0,
                "terminal_docking": False,
            }, separators=(",", ":"))))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReactiveAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
