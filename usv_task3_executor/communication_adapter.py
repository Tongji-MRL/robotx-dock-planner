"""ROS 2 adapter for the fixed FSM mission-planner JSON contract.

The main planner owns the receive/send topics and expects the legacy compact
shape ``vehicle/task/progress/error/command``.  This node validates Task3
payloads, converts them to that shape, and never commands navigation, the pump,
or the gimbal.
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
    UAV_HEARTBEAT_TOPIC,
    USV_HEARTBEAT_TOPIC,
    decode_json_object,
    encode_json_object,
)
from dock_planner.fsm_contract import (
    FSM_CHECKPOINT_TOPIC,
    FSM_PROGRESS_TOPIC,
    FSM_RESULT_TOPIC,
    FSM_TASK_COMMAND_TOPIC,
    correct_bay_to_fsm_progress,
    docking_result_to_fsm_message,
    executor_state_to_fsm_progress,
    firefighting_result_to_fsm_message,
    heartbeat_fault_to_fsm_message,
    normalize_fsm_task_command,
    safe_stop_ack_progress,
    sprayer_window_to_fsm_progress,
    task4_ack_progress,
)

TASK3_CONTROL_COMMAND_TOPIC = "/tongji_mrl/twave/task3/control_command"
TASK3_EXECUTOR_STATE_TOPIC = "/tongji_mrl/twave/task3/state"
TASK3_CORRECT_BAY_TOPIC = "/tongji_mrl/twave/task3/correct_bay"
TASK3_SPRAYER_WINDOW_TOPIC = "/tongji_mrl/twave/task3/sprayer_window"


class Task3CommunicationAdapter(Node):
    """Expose the frozen Task3 communication contract over ROS 2 String topics."""

    def __init__(self) -> None:
        super().__init__("task3_communication_adapter")
        self.declare_parameter("run_id", 0)
        self.declare_parameter("heartbeat_timeout_s", 3.0)
        self.declare_parameter("heartbeat_startup_grace_s", 5.0)

        configured_run_id = int(self.get_parameter("run_id").value)
        self.run_id: int | None = configured_run_id if configured_run_id > 0 else None
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
        self._last_executor_state: dict[str, Any] | None = None

        self.progress_pub = self.create_publisher(String, FSM_PROGRESS_TOPIC, 10)
        self.result_pub = self.create_publisher(String, FSM_RESULT_TOPIC, 10)
        self.checkpoint_pub = self.create_publisher(String, FSM_CHECKPOINT_TOPIC, 10)
        # Kept as a diagnostic topic for local tooling; the FSM itself uses
        # TASK4_ACK progress on /tongji_mrl/twave/progress.
        self.ack_pub = self.create_publisher(String, "/tongji_mrl/twave/task_ack", 10)
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
        self.create_subscription(String, FSM_TASK_COMMAND_TOPIC, self._on_task_command, 10)
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

    def _on_executor_state(self, msg: String) -> None:
        try:
            value = decode_json_object(msg.data)
            self._last_executor_state = value
            report = executor_state_to_fsm_progress(value)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid Task3 executor state: {exc}")
            return
        self._publish(self.progress_pub, report)

    def _on_correct_bay(self, msg: String) -> None:
        try:
            value = decode_json_object(msg.data)
            info = CorrectBayInfo.from_mapping(value)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid correct-bay input: {exc}")
            return
        raw_info = {
            "bay_id": info.bay_id,
            "dock_id": info.dock_id,
            "confidence": info.confidence,
            "docking_confirmed": info.docking_confirmed,
        }
        self._publish(self.progress_pub, correct_bay_to_fsm_progress(raw_info))
        if not info.docking_confirmed:
            return
        report = docking_result_to_fsm_message(raw_info)
        key = ("docking", info.bay_id, info.confirmed_at_unix_ms)
        if key in self._published_report_keys:
            return
        self._published_report_keys.add(key)
        self._publish(self.result_pub, report)

    def _on_sprayer_window(self, msg: String) -> None:
        try:
            value = decode_json_object(msg.data)
            info = SprayerWindowInfo.from_mapping(value)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid sprayer-window input: {exc}")
            return
        raw_info = {
            "window_id": info.window_id,
            "color": info.color,
            "confidence": info.confidence,
            "pump_off": info.pump_off,
            "green_confirmed_duration_s": info.green_confirmed_duration_s,
            "completion_confirmed": info.completion_confirmed,
        }
        self._publish(self.progress_pub, sprayer_window_to_fsm_progress(raw_info))
        if not info.firefighting_complete:
            return
        key = ("firefighting", info.window_id, info.observed_at_unix_ms)
        if key in self._published_report_keys:
            return
        self._published_report_keys.add(key)
        self._publish(self.result_pub, firefighting_result_to_fsm_message(raw_info))

    def _on_task_command(self, msg: String) -> None:
        try:
            raw = decode_json_object(msg.data)
            command = normalize_fsm_task_command(raw)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Rejecting invalid Task3 command: {exc}")
            return

        # MAP and Task1 commands are owned by their own USV modules.
        if command is None:
            return

        duplicate = command["command_id"] in self._processed_command_ids
        if not duplicate:
            self._processed_command_ids.add(command["command_id"])
            if command["command"] == "TASK4_GOTO":
                self._publish_checkpoint_before_task4()
            self._publish(self.control_command_pub, command)
        if command["command"] == "TASK4_GOTO":
            ack = task4_ack_progress(command)
        elif command["command"] == "SAFE_STOP":
            ack = safe_stop_ack_progress(command)
        else:
            ack = {
                "vehicle": "usv",
                "task": command["task"],
                "progress": f"{command['task']}_ACK",
                "error": None,
                "duplicate": duplicate,
            }
        self._publish(self.progress_pub, ack)
        self._publish(self.ack_pub, ack)

    def _publish_checkpoint_before_task4(self) -> None:
        """Publish the checkpoint shape consumed by FSM's TASK4_OVER path."""

        state = self._last_executor_state
        if not isinstance(state, dict):
            self._publish(
                self.progress_pub,
                {
                    "vehicle": "usv",
                    "task": "TASK3",
                    "progress": "TASK4_CHECKPOINT_UNAVAILABLE",
                    "error": "CHECKPOINT_LOAD_FAILED",
                },
            )
            return
        self._publish(
            self.checkpoint_pub,
            {
                "vehicle": "usv",
                "task": "TASK3",
                "progress": state.get("state"),
                "error": state.get("error"),
                "command": "checkpoint",
                "context": state.get("context", {}),
            },
        )

    def _on_heartbeat(self, vehicle_id: str, msg: String) -> None:
        try:
            value = decode_json_object(msg.data)
            self.heartbeat_monitor.note(vehicle_id, value, time.monotonic())
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid {vehicle_id} heartbeat: {exc}")
            return
        if self._heartbeat_stale_state[vehicle_id] is True:
            self._publish(self.progress_pub, heartbeat_fault_to_fsm_message(vehicle_id, stale=False, age_s=0.0))
        self._heartbeat_stale_state[vehicle_id] = False

    def _check_heartbeats(self) -> None:
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
                    self.progress_pub,
                    heartbeat_fault_to_fsm_message(
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
