"""ROS 2 adapter for Task3 progress, reports, commands, and heartbeats.

This node transports validated JSON only.  It does not command navigation,
the pump, or the gimbal.  Accepted commands are forwarded to the Task3
executor on a separate topic and acknowledged as ``DISPATCHED``.
"""

from __future__ import annotations

import json
import time
from typing import Any, Mapping

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from dock_planner.task3_ros_contract import (
    CorrectBayInfo,
    HeartbeatMonitor,
    SprayerWindowInfo,
    TASK3_CONTROL_COMMAND_TOPIC,
    TASK3_CORRECT_BAY_TOPIC,
    TASK3_EXECUTOR_STATE_TOPIC,
    TASK3_SPRAYER_WINDOW_TOPIC,
    TASK_ACK_TOPIC,
    TASK_COMMAND_TOPIC,
    TASK_FAULT_TOPIC,
    TASK_PROGRESS_TOPIC,
    TASK_REPORT_TOPIC,
    Task3RosMessageFactory,
    UAV_HEARTBEAT_TOPIC,
    USV_HEARTBEAT_TOPIC,
    decode_json_object,
    encode_json_object,
    normalize_task_command,
)


class Task3CommunicationAdapter(Node):
    """Expose the frozen Task3 communication contract over ROS 2 String topics."""

    def __init__(self) -> None:
        super().__init__("task3_communication_adapter")
        self.declare_parameter("run_id", 0)
        self.declare_parameter("heartbeat_timeout_s", 3.0)
        self.declare_parameter("heartbeat_startup_grace_s", 5.0)

        configured_run_id = int(self.get_parameter("run_id").value)
        self.run_id: int | None = configured_run_id if configured_run_id > 0 else None
        self.factory = (
            Task3RosMessageFactory(self.run_id) if self.run_id is not None else None
        )
        self.heartbeat_monitor = HeartbeatMonitor(
            float(self.get_parameter("heartbeat_timeout_s").value)
        )
        self.heartbeat_startup_grace_s = float(
            self.get_parameter("heartbeat_startup_grace_s").value
        )
        if self.heartbeat_startup_grace_s < 0:
            raise ValueError("heartbeat_startup_grace_s must be non-negative")
        self.started_monotonic_s = time.monotonic()
        self._heartbeat_stale_state: dict[str, bool | None] = {
            "T-Wave": None,
            "T-Sky": None,
        }
        self._processed_command_ids: set[str] = set()
        self._published_report_keys: set[tuple[Any, ...]] = set()

        self.progress_pub = self.create_publisher(String, TASK_PROGRESS_TOPIC, 10)
        self.report_pub = self.create_publisher(String, TASK_REPORT_TOPIC, 10)
        self.fault_pub = self.create_publisher(String, TASK_FAULT_TOPIC, 10)
        self.ack_pub = self.create_publisher(String, TASK_ACK_TOPIC, 10)
        self.control_command_pub = self.create_publisher(
            String, TASK3_CONTROL_COMMAND_TOPIC, 10
        )

        self.create_subscription(
            String, TASK3_EXECUTOR_STATE_TOPIC, self._on_executor_state, 10
        )
        self.create_subscription(
            String, TASK3_CORRECT_BAY_TOPIC, self._on_correct_bay, 10
        )
        self.create_subscription(
            String, TASK3_SPRAYER_WINDOW_TOPIC, self._on_sprayer_window, 10
        )
        self.create_subscription(String, TASK_COMMAND_TOPIC, self._on_task_command, 10)
        self.create_subscription(
            String,
            USV_HEARTBEAT_TOPIC,
            lambda msg: self._on_heartbeat("T-Wave", msg),
            10,
        )
        self.create_subscription(
            String,
            UAV_HEARTBEAT_TOPIC,
            lambda msg: self._on_heartbeat("T-Sky", msg),
            10,
        )
        self.heartbeat_timer = self.create_timer(0.5, self._check_heartbeats)
        self.get_logger().info(
            "Task3 communication adapter started; it validates and forwards messages "
            "but does not control navigation, pump, or gimbal."
        )

    def _publish(self, publisher, value: Mapping[str, Any]) -> None:
        message = String()
        message.data = encode_json_object(value)
        publisher.publish(message)

    def _require_factory(self, message_run_id: Any = None) -> Task3RosMessageFactory:
        if message_run_id is not None:
            if isinstance(message_run_id, bool) or not isinstance(message_run_id, int):
                raise ValueError("run_id must be a positive integer")
            if message_run_id <= 0:
                raise ValueError("run_id must be a positive integer")
        if self.factory is None:
            if message_run_id is None:
                raise ValueError("run_id is not configured or started")
            self.run_id = message_run_id
            self.factory = Task3RosMessageFactory(message_run_id)
        if message_run_id is not None and message_run_id != self.run_id:
            raise ValueError(
                f"message run_id {message_run_id} does not match active run {self.run_id}"
            )
        return self.factory

    def _on_executor_state(self, msg: String) -> None:
        try:
            value = decode_json_object(msg.data)
            factory = self._require_factory(value.get("run_id"))
            report = factory.progress_from_executor_state(value)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid Task3 executor state: {exc}")
            return
        self._publish(self.progress_pub, report)

    def _on_correct_bay(self, msg: String) -> None:
        try:
            value = decode_json_object(msg.data)
            factory = self._require_factory(value.get("run_id"))
            info = CorrectBayInfo.from_mapping(value)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid correct-bay input: {exc}")
            return
        self._publish(self.progress_pub, factory.correct_bay_progress(info))
        if not info.docking_confirmed:
            return
        report = factory.docking_report(info)
        key = ("docking", self.run_id, info.bay_id, info.confirmed_at_unix_ms)
        if key in self._published_report_keys:
            return
        self._published_report_keys.add(key)
        self._publish(self.report_pub, report)

    def _on_sprayer_window(self, msg: String) -> None:
        try:
            value = decode_json_object(msg.data)
            factory = self._require_factory(value.get("run_id"))
            info = SprayerWindowInfo.from_mapping(value)
            progress = factory.sprayer_window_progress(info)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid sprayer-window input: {exc}")
            return
        self._publish(self.progress_pub, progress)
        if not info.firefighting_complete:
            return
        key = ("firefighting", self.run_id, info.window_id, info.observed_at_unix_ms)
        if key in self._published_report_keys:
            return
        self._published_report_keys.add(key)
        self._publish(self.report_pub, factory.firefighting_report(info))

    def _on_task_command(self, msg: String) -> None:
        try:
            raw = decode_json_object(msg.data)
            command = normalize_task_command(raw)
            factory = self._require_factory(command["run_id"])
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Rejecting invalid Task3 command: {exc}")
            return

        duplicate = command["command_id"] in self._processed_command_ids
        if not duplicate:
            self._processed_command_ids.add(command["command_id"])
            self._publish(self.control_command_pub, command)
        ack = factory.ack(
            command,
            accepted=True,
            state="DUPLICATE_IGNORED" if duplicate else "DISPATCHED",
            detail=(
                "command was already dispatched"
                if duplicate
                else "validated and forwarded to Task3 executor; execution not yet confirmed"
            ),
        )
        self._publish(self.ack_pub, ack)

    def _on_heartbeat(self, vehicle_id: str, msg: String) -> None:
        try:
            value = decode_json_object(msg.data)
            if self.run_id is not None and value.get("run_id") not in (None, self.run_id):
                raise ValueError("heartbeat run_id does not match active run")
            self.heartbeat_monitor.note(vehicle_id, value, time.monotonic())
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid {vehicle_id} heartbeat: {exc}")
            return
        if self._heartbeat_stale_state[vehicle_id] is True and self.factory is not None:
            self._publish(
                self.fault_pub,
                self.factory.heartbeat_fault(vehicle_id, stale=False, age_s=0.0),
            )
        self._heartbeat_stale_state[vehicle_id] = False

    def _check_heartbeats(self) -> None:
        if self.factory is None:
            return
        now = time.monotonic()
        if now - self.started_monotonic_s < self.heartbeat_startup_grace_s:
            return
        for vehicle_id in ("T-Wave", "T-Sky"):
            stale = self.heartbeat_monitor.stale(vehicle_id, now)
            if stale == self._heartbeat_stale_state[vehicle_id]:
                continue
            self._heartbeat_stale_state[vehicle_id] = stale
            if stale:
                age = self.heartbeat_monitor.age_s(vehicle_id, now)
                self._publish(
                    self.fault_pub,
                    self.factory.heartbeat_fault(
                        vehicle_id,
                        stale=True,
                        age_s=(self.heartbeat_monitor.timeout_s if age is None else age),
                    ),
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Task3CommunicationAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ["Task3CommunicationAdapter", "main"]
