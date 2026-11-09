"""Pure Task 3 ROS JSON contracts used between child and main state machines.

The wire transport is ``std_msgs/msg/String``.  This module deliberately has
no ROS dependency, so validation and completion gates can be tested offline.
It never infers docking or firefighting completion from perception alone.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import time
import uuid
from typing import Any, Callable, Mapping

from .task3_status_codec import Task3Step
from .fsm_contract import validate_fsm_progress


PROTOCOL_VERSION = "task3-ros-v1"
VEHICLE_PROTOCOL_VERSION = "1"
# Shared local NED established from the fixed competition RTK reference.  UAV
# map objects and the USV pose may be combined only when both explicitly carry
# this same frame identifier.
COMMON_RTK_NED_FRAME = "robotx_rtk_ned"

TASK_COMMAND_TOPIC = "/tongji_mrl/twave/task_cmd"
TASK_ACK_TOPIC = "/tongji_mrl/twave/task_ack"
TASK3_CONTROL_COMMAND_TOPIC = "/tongji_mrl/twave/task3/control_command"
TASK3_EXECUTOR_STATE_TOPIC = "/tongji_mrl/twave/task3/state"
TASK3_CORRECT_BAY_TOPIC = "/tongji_mrl/twave/task3/correct_bay"
TASK3_SPRAYER_WINDOW_TOPIC = "/tongji_mrl/twave/task3/sprayer_window"
TASK_PROGRESS_TOPIC = "/tongji_mrl/twave/progress"
TASK_REPORT_TOPIC = "/tongji_mrl/twave/task_report"
TASK_FAULT_TOPIC = "/tongji_mrl/twave/fault"
USV_HEARTBEAT_TOPIC = "/tongji_mrl/twave/heartbeat"
UAV_HEARTBEAT_TOPIC = "/tongji_mrl/tsky/heartbeat"

SUPPORTED_COMMANDS = {
    "START_TASK3",
    "PAUSE_TASK3",
    "TASK4_GOTO",
    "RESUME_TASK3",
    "SAFE_STOP",
    "RESET",
}

# RoboNation's Task 3 dock drawing defines these competition IDs. ``dock_id``
# remains the identifier of the whole dock/map object and must not be confused
# with the numbered bay selected inside that dock.
OFFICIAL_BAY_IDS = (1, 2, 3)
OFFICIAL_WINDOW_IDS = (1, 2)

_FLOW_TO_STEP = {
    "IDLE": Task3Step.READY,
    "APPROACH_DOCK": Task3Step.APPROACH_DOCK,
    "SEARCH_DOCK": Task3Step.SEARCH_DOCK,
    "CHECK_BAY_COLOR": Task3Step.CHECK_BAY_COLOR,
    "NEXT_BAY": Task3Step.NEXT_BAY,
    "TERMINAL_DOCKING": Task3Step.TERMINAL_DOCKING,
    "DOCKED": Task3Step.DOCKED,
    "WATERING": Task3Step.SPRAYING,
    "FINISHED": Task3Step.COMPLETE,
    "ERROR": Task3Step.READY,
}


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


def _positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _nonempty(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value.strip()


def official_bay_id_from_left_index(left_index: int) -> int:
    """Map a bay's zero-based left-to-right order to the official bay ID."""

    if isinstance(left_index, bool) or not isinstance(left_index, int):
        raise ValueError("left_index must be an integer")
    if not 0 <= left_index < len(OFFICIAL_BAY_IDS):
        raise ValueError("bay left_index must be 0, 1, or 2")
    return OFFICIAL_BAY_IDS[left_index]


def official_window_id_from_side(side: str) -> int:
    """Map window side to its official ID while facing the dock from the water."""

    if not isinstance(side, str):
        raise ValueError("window side must be LEFT or RIGHT")
    normalized = side.strip().upper()
    try:
        return {"LEFT": 1, "RIGHT": 2}[normalized]
    except KeyError as exc:
        raise ValueError("window side must be LEFT or RIGHT") from exc


@dataclass
class BestEffortWindowId:
    """Keep the latest usable LEFT/RIGHT evidence and always yield a report ID.

    Window numbering is useful for the post-delivery FirefightingReport, but it
    must not block aiming, spraying, or the GREEN completion decision.  When no
    side evidence was available during the attempt, ``fallback_id`` supplies a
    deterministic guess so the report can still be sent.
    """

    fallback_id: int = 1
    observed_id: int | None = None

    def __post_init__(self) -> None:
        if self.fallback_id not in OFFICIAL_WINDOW_IDS:
            raise ValueError("fallback_id must be 1 (left) or 2 (right)")

    def observe_side(self, side: object) -> int | None:
        if not isinstance(side, str):
            return self.observed_id
        normalized = side.strip().upper()
        if normalized in {"LEFT", "RIGHT"}:
            self.observed_id = official_window_id_from_side(normalized)
        return self.observed_id

    @property
    def report_id(self) -> int:
        return self.observed_id or self.fallback_id

    @property
    def used_fallback(self) -> bool:
        return self.observed_id is None

    def reset(self) -> None:
        self.observed_id = None


def decode_json_object(raw: str) -> dict[str, Any]:
    try:
        value = json.loads(raw)
    except (TypeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid JSON: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError("JSON message must be an object")
    return value


def encode_json_object(value: Mapping[str, Any]) -> str:
    if not isinstance(value, Mapping):
        raise TypeError("message must be a mapping")
    return json.dumps(dict(value), ensure_ascii=True, separators=(",", ":"), sort_keys=True)


@dataclass(frozen=True)
class CorrectBayInfo:
    bay_id: int
    dock_id: str
    confidence: float
    confirmed_at_unix_ms: int
    docking_confirmed: bool
    source: str

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "CorrectBayInfo":
        if not isinstance(value, Mapping):
            raise TypeError("correct bay input must be a mapping")
        bay_id = _positive_int(value.get("bay_id"), "bay_id")
        if bay_id not in OFFICIAL_BAY_IDS:
            raise ValueError("bay_id must be between 1 and 3")
        confidence = _finite(value.get("confidence"), "confidence")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be in [0, 1]")
        confirmed = value.get("docking_confirmed")
        if not isinstance(confirmed, bool):
            raise ValueError("docking_confirmed must be boolean")
        return cls(
            bay_id=bay_id,
            dock_id=_nonempty(value.get("dock_id"), "dock_id"),
            confidence=confidence,
            confirmed_at_unix_ms=_positive_int(
                value.get("confirmed_at_unix_ms"), "confirmed_at_unix_ms"
            ),
            docking_confirmed=confirmed,
            source=_nonempty(value.get("source"), "source"),
        )


@dataclass(frozen=True)
class SprayerWindowInfo:
    window_id: int
    color: str
    confidence: float
    observed_at_unix_ms: int
    pump_off: bool
    green_confirmed_duration_s: float
    completion_confirmed: bool
    source: str

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "SprayerWindowInfo":
        if not isinstance(value, Mapping):
            raise TypeError("sprayer window input must be a mapping")
        color = _nonempty(value.get("color"), "color").upper()
        if color not in {"UNKNOWN", "RED", "GREEN", "BLUE"}:
            raise ValueError("color must be UNKNOWN, RED, GREEN, or BLUE")
        confidence = _finite(value.get("confidence"), "confidence")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be in [0, 1]")
        pump_off = value.get("pump_off")
        completion = value.get("completion_confirmed")
        if not isinstance(pump_off, bool):
            raise ValueError("pump_off must be boolean")
        if not isinstance(completion, bool):
            raise ValueError("completion_confirmed must be boolean")
        duration = _finite(
            value.get("green_confirmed_duration_s"),
            "green_confirmed_duration_s",
        )
        if duration < 0:
            raise ValueError("green_confirmed_duration_s must be non-negative")
        window_id = _positive_int(value.get("window_id"), "window_id")
        if window_id not in OFFICIAL_WINDOW_IDS:
            raise ValueError("window_id must be 1 (left) or 2 (right)")
        return cls(
            window_id=window_id,
            color=color,
            confidence=confidence,
            observed_at_unix_ms=_positive_int(
                value.get("observed_at_unix_ms"), "observed_at_unix_ms"
            ),
            pump_off=pump_off,
            green_confirmed_duration_s=duration,
            completion_confirmed=completion,
            source=_nonempty(value.get("source"), "source"),
        )

    @property
    def firefighting_complete(self) -> bool:
        return (
            self.completion_confirmed
            and self.color == "GREEN"
            and self.pump_off
            and self.green_confirmed_duration_s >= 5.0
        )


@dataclass
class Task3TargetIdGate:
    """Latch one bay/window pair and block mixed or out-of-order reports.

    The child Task 3 flow feeds already-confirmed perception decisions into
    this gate. A window cannot be accepted before a bay, and either ID cannot
    silently change during the same Task 3 run. Call :meth:`reset` when a new
    run starts or the mission planner issues RESET.
    """

    dock_id: str | None = None
    bay_id: int | None = None
    window_id: int | None = None

    def accept_bay(self, info: CorrectBayInfo) -> None:
        if not isinstance(info, CorrectBayInfo):
            raise TypeError("info must be CorrectBayInfo")
        if self.bay_id is not None and (
            self.bay_id != info.bay_id or self.dock_id != info.dock_id
        ):
            raise ValueError(
                "bay selection is already locked for this Task3 run; reset before changing it"
            )
        self.dock_id = info.dock_id
        self.bay_id = info.bay_id

    def accept_window(self, info: SprayerWindowInfo) -> None:
        if not isinstance(info, SprayerWindowInfo):
            raise TypeError("info must be SprayerWindowInfo")
        if self.bay_id is None or self.dock_id is None:
            raise ValueError("window_id cannot be accepted before bay_id is confirmed")
        if self.window_id is not None and self.window_id != info.window_id:
            raise ValueError(
                "window selection is already locked for this Task3 run; reset before changing it"
            )
        self.window_id = info.window_id

    def context(self) -> dict[str, Any]:
        return {
            "dock_id": self.dock_id,
            "bay_id": self.bay_id,
            "window_id": self.window_id,
        }

    def reset(self) -> None:
        self.dock_id = None
        self.bay_id = None
        self.window_id = None


class Task3RosMessageFactory:
    """Create compact progress messages consumed by the fixed FSM planner.

    The former 14-bit status fields remain available to internal diagnostics,
    but are deliberately absent from this ROS-facing factory output.
    """

    def __init__(
        self,
        run_id: int,
        *,
        clock_ms: Callable[[], int] | None = None,
        message_id_factory: Callable[[], str] | None = None,
    ) -> None:
        self.run_id = _positive_int(run_id, "run_id")
        self._clock_ms = clock_ms or (lambda: int(time.time() * 1000))
        self._message_id_factory = message_id_factory or (lambda: str(uuid.uuid4()))
        self._sequence = 0

    def _base(self, kind: str) -> dict[str, Any]:
        self._sequence += 1
        return {
            "vehicle": "usv",
            "task": "TASK3",
            "progress": None,
            "error": None,
            "kind": kind,
            "run_id": self.run_id,
            "sequence": self._sequence,
        }

    def progress_from_executor_state(self, value: Mapping[str, Any]) -> dict[str, Any]:
        if not isinstance(value, Mapping):
            raise TypeError("executor state must be a mapping")
        state = _nonempty(value.get("state"), "state").upper()
        try:
            step = _FLOW_TO_STEP[state]
        except KeyError as exc:
            raise ValueError(f"unsupported Task3 executor state: {state}") from exc
        progress = "TASK3_ERROR" if state == "ERROR" else f"TASK3_{step.name}"
        message = self._base("progress")
        message.update({
            "progress": progress,
            "error": (str(value.get("error") or value.get("detail") or "UNKNOWN_ERROR")
                      if state == "ERROR" else None),
            "executor_state": state,
            "ready_for_task_logic": bool(value.get("ready_for_task_logic", False)),
            "dry_run": bool(value.get("dry_run", False)),
            "detail": str(value.get("detail", "")),
        })
        if value.get("retry") is not None:
            message["retry"] = value["retry"]
        validate_fsm_progress(message)
        return message

    def docking_report(self, info: CorrectBayInfo) -> dict[str, Any]:
        if not isinstance(info, CorrectBayInfo):
            raise TypeError("info must be CorrectBayInfo")
        if not info.docking_confirmed:
            raise ValueError("docking report requires docking_confirmed=true")
        message = self._base("task_report")
        message.update({
            "progress": "TASK3_DOCKING_RESULT",
            "phase": "DOCKING_COMPLETE",
            "authority": "USV",
            "official_report": {
                "type": "docking",
                "vehicle_id": "T-Wave",
                "bay_id": info.bay_id,
                "dock_id": info.dock_id,
                "confidence": info.confidence,
            },
            "completed_at_unix_ms": info.confirmed_at_unix_ms,
        })
        validate_fsm_progress(message)
        return message

    def correct_bay_progress(self, info: CorrectBayInfo) -> dict[str, Any]:
        """Report the selected bay without claiming that docking is complete."""

        if not isinstance(info, CorrectBayInfo):
            raise TypeError("info must be CorrectBayInfo")
        message = self._base("progress")
        message.update({
            "progress": "TASK3_BAY_CONFIRMED",
            "dock_id": info.dock_id,
            "bay_id": info.bay_id,
            "confidence": info.confidence,
            "docking_confirmed": info.docking_confirmed,
            "confirmed_at_unix_ms": info.confirmed_at_unix_ms,
            "source": info.source,
        })
        validate_fsm_progress(message)
        return message

    def sprayer_window_progress(self, info: SprayerWindowInfo) -> dict[str, Any]:
        if not isinstance(info, SprayerWindowInfo):
            raise TypeError("info must be SprayerWindowInfo")
        message = self._base("progress")
        message.update({
            "progress": "TASK3_SPRAYER_WINDOW",
            "window_id": info.window_id,
            "color": info.color,
            "confidence": info.confidence,
            "pump_off": info.pump_off,
            "green_confirmed_duration_s": info.green_confirmed_duration_s,
            "completion_confirmed": info.completion_confirmed,
            "source": info.source,
        })
        validate_fsm_progress(message)
        return message

    def firefighting_report(self, info: SprayerWindowInfo) -> dict[str, Any]:
        if not isinstance(info, SprayerWindowInfo):
            raise TypeError("info must be SprayerWindowInfo")
        if not info.firefighting_complete:
            raise ValueError(
                "firefighting report requires GREEN, pump_off, explicit completion, "
                "and at least 5 seconds confirmation"
            )
        message = self._base("task_report")
        message.update({
            "progress": "TASK3_FIREFIGHTING_RESULT",
            "phase": "FIREFIGHTING_COMPLETE",
            "authority": "USV",
            "official_report": {
                "type": "firefighting",
                "vehicle_id": "T-Wave",
                "window_id": info.window_id,
                "color": info.color,
            },
            "completed_at_unix_ms": info.observed_at_unix_ms,
        })
        validate_fsm_progress(message)
        return message

    def ack(
        self,
        command: Mapping[str, Any],
        *,
        accepted: bool,
        state: str,
        detail: str,
    ) -> dict[str, Any]:
        normalized = normalize_task_command(command)
        message = self._base("ack")
        message.update({
            "progress": "TASK3_ACK",
            "error": None if accepted else (str(detail) or "COMMAND_REJECTED"),
            "in_reply_to": normalized["command_id"],
            "command_name": normalized["command"],
            "accepted": bool(accepted),
            "state": _nonempty(state, "state"),
            "detail": str(detail),
        })
        validate_fsm_progress(message)
        return message

    def heartbeat_fault(self, vehicle_id: str, *, stale: bool, age_s: float) -> dict[str, Any]:
        vehicle_id = _nonempty(vehicle_id, "vehicle_id")
        if vehicle_id not in {"T-Wave", "T-Sky"}:
            raise ValueError("vehicle_id must be T-Wave or T-Sky")
        age = _finite(age_s, "age_s")
        message = self._base("fault")
        message.update({
            "progress": "HEARTBEAT_STALE" if stale else "HEARTBEAT_RECOVERED",
            "error": "COMMUNICATION_LOST" if stale else None,
            "heartbeat_vehicle": vehicle_id,
            "age_s": age,
            "severity": "STOP" if stale else "RECOVERED",
        })
        validate_fsm_progress(message)
        return message


def normalize_task_command(value: Mapping[str, Any]) -> dict[str, Any]:
    """Normalize current structured commands and the existing legacy shape."""

    if not isinstance(value, Mapping):
        raise TypeError("task command must be a mapping")
    if "command" in value:
        command = _nonempty(value.get("command"), "command").upper()
        command_id = _nonempty(value.get("command_id"), "command_id")
        run_id = _positive_int(value.get("run_id"), "run_id")
        payload = value.get("payload", {})
        if not isinstance(payload, Mapping):
            raise ValueError("payload must be an object")
    else:
        task_id = value.get("task_id")
        action = _nonempty(value.get("action"), "action").lower()
        run_id = _positive_int(value.get("run_id"), "run_id")
        command_id = _nonempty(
            value.get("command_id") or (
                f"legacy-{value['command_seq']}" if value.get("command_seq") else None
            ),
            "command_id or command_seq",
        )
        parameters = value.get("parameters", {})
        if not isinstance(parameters, Mapping):
            raise ValueError("parameters must be an object")
        if task_id == 3 and action in {"execute_task", "start_task_execution"}:
            command = "START_TASK3"
        elif task_id == 3 and action == "pause_task":
            command = "PAUSE_TASK3"
        elif task_id == 3 and action in {"resume_task", "continue_task"}:
            command = "RESUME_TASK3"
        elif task_id == 4 and action in {"execute_task", "start_task_execution"}:
            command = "TASK4_GOTO"
        elif action in {"safe_stop", "stop_task"}:
            command = "SAFE_STOP"
        elif action == "reset":
            command = "RESET"
        else:
            raise ValueError(f"unsupported legacy task command: task_id={task_id}, action={action}")
        payload = dict(parameters)

    if command not in SUPPORTED_COMMANDS:
        raise ValueError(f"unsupported command: {command}")
    normalized = {
        "command_id": command_id,
        "run_id": run_id,
        "command": command,
        "payload": dict(payload),
    }
    if command == "TASK4_GOTO":
        target = normalized["payload"].get("target_position")
        if not isinstance(target, Mapping) or not target:
            raise ValueError("TASK4_GOTO requires target_position")
    if command == "RESUME_TASK3":
        _nonempty(normalized["payload"].get("checkpoint_id"), "checkpoint_id")
    return normalized


def validate_vehicle_heartbeat(value: Mapping[str, Any], expected_vehicle_id: str) -> None:
    """Validate the existing OCS-facing vehicle heartbeat envelope."""

    if not isinstance(value, Mapping):
        raise TypeError("heartbeat must be a mapping")
    required = {
        "protocol_version",
        "message_id",
        "sent_at_unix_ms",
        "run_id",
        "vehicle_id",
        "source",
        "frame_id",
        "kind",
        "payload",
    }
    missing = sorted(required - set(value))
    if missing:
        raise ValueError(f"heartbeat missing fields: {', '.join(missing)}")
    if value["protocol_version"] != VEHICLE_PROTOCOL_VERSION:
        raise ValueError("unsupported heartbeat protocol_version")
    if value["kind"] != "heartbeat":
        raise ValueError("heartbeat kind must be heartbeat")
    if value["vehicle_id"] != expected_vehicle_id:
        raise ValueError("heartbeat vehicle_id does not match topic")
    _nonempty(value["message_id"], "message_id")
    _positive_int(value["sent_at_unix_ms"], "sent_at_unix_ms")
    _nonempty(value["source"], "source")
    _nonempty(value["frame_id"], "frame_id")
    if value["run_id"] is not None:
        _positive_int(value["run_id"], "run_id")
    payload = value["payload"]
    if not isinstance(payload, Mapping):
        raise ValueError("heartbeat payload must be an object")
    _nonempty(payload.get("state"), "heartbeat payload.state")
    _nonempty(payload.get("current_task"), "heartbeat payload.current_task")
    position = payload.get("position")
    if position is not None:
        if not isinstance(position, Mapping):
            raise ValueError("heartbeat position must be null or an object")
        if position.get("latitude_deg") is not None:
            latitude = _finite(position["latitude_deg"], "latitude_deg")
            if not -90.0 <= latitude <= 90.0:
                raise ValueError("latitude_deg out of range")
        if position.get("longitude_deg") is not None:
            longitude = _finite(position["longitude_deg"], "longitude_deg")
            if not -180.0 <= longitude <= 180.0:
                raise ValueError("longitude_deg out of range")


class HeartbeatMonitor:
    """Receive-time watchdog for the T-Wave and T-Sky heartbeat topics."""

    def __init__(self, timeout_s: float = 3.0) -> None:
        self.timeout_s = _finite(timeout_s, "timeout_s")
        if self.timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        self._received_at: dict[str, float] = {}

    def note(self, vehicle_id: str, message: Mapping[str, Any], now_s: float) -> None:
        validate_vehicle_heartbeat(message, vehicle_id)
        self._received_at[vehicle_id] = _finite(now_s, "now_s")

    def age_s(self, vehicle_id: str, now_s: float) -> float | None:
        now = _finite(now_s, "now_s")
        received = self._received_at.get(vehicle_id)
        return None if received is None else max(0.0, now - received)

    def stale(self, vehicle_id: str, now_s: float) -> bool:
        age = self.age_s(vehicle_id, now_s)
        return age is None or age > self.timeout_s


__all__ = [
    "BestEffortWindowId",
    "COMMON_RTK_NED_FRAME",
    "CorrectBayInfo",
    "HeartbeatMonitor",
    "OFFICIAL_BAY_IDS",
    "OFFICIAL_WINDOW_IDS",
    "PROTOCOL_VERSION",
    "SprayerWindowInfo",
    "TASK3_CONTROL_COMMAND_TOPIC",
    "TASK3_CORRECT_BAY_TOPIC",
    "TASK3_EXECUTOR_STATE_TOPIC",
    "TASK3_SPRAYER_WINDOW_TOPIC",
    "TASK_ACK_TOPIC",
    "TASK_COMMAND_TOPIC",
    "TASK_FAULT_TOPIC",
    "TASK_PROGRESS_TOPIC",
    "TASK_REPORT_TOPIC",
    "Task3RosMessageFactory",
    "Task3TargetIdGate",
    "UAV_HEARTBEAT_TOPIC",
    "USV_HEARTBEAT_TOPIC",
    "decode_json_object",
    "encode_json_object",
    "normalize_task_command",
    "official_bay_id_from_left_index",
    "official_window_id_from_side",
    "validate_vehicle_heartbeat",
]
