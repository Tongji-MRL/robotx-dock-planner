"""Compatibility contract for the fixed FSM mission planner node.

The main planner in ``Tongji-MRL/FSM`` uses ``std_msgs/msg/String`` carrying
small JSON objects.  This module keeps that wire format at the ROS boundary;
the richer Task3 status objects remain internal implementation details.
"""

from __future__ import annotations

from typing import Any, Mapping


FSM_TASK_COMMAND_TOPIC = "/tongji_mrl/twave/task_cmd"
FSM_PROGRESS_TOPIC = "/tongji_mrl/twave/progress"
FSM_RESULT_TOPIC = "/tongji_mrl/twave/result"
FSM_CHECKPOINT_TOPIC = "/tongji_mrl/twave/checkpoint"
FSM_TASK3_FINISH_PROGRESS = "TASK3_FINISH"

# These are the only fields that the FSM mission_planner_node uses when it
# consumes a vehicle progress report.  Extra fields are allowed for the
# structured bay/window result data, but packed status integers are not part
# of the wire contract.
FSM_PROGRESS_REQUIRED_FIELDS = ("vehicle", "task", "progress", "error")
TASK3_WIRE_PROGRESS = {
    "TASK3_READY",
    "TASK3_WAIT_DOCK_OBSERVATION",
    "TASK3_APPROACH_DOCK",
    "TASK3_SEARCH_DOCK",
    "TASK3_CHECK_BAY_COLOR",
    "TASK3_NEXT_BAY",
    "TASK3_TERMINAL_DOCKING",
    "TASK3_DOCKED",
    "TASK3_WAIT_RED_WINDOW",
    "TASK3_AIMING",
    "TASK3_SPRAYING",
    "TASK3_VERIFY_GREEN",
    "TASK3_COLOR_SEQUENCE",
    "TASK3_WAIT_UAV_DELIVERY",
    "TASK3_EXIT_RETURN",
    FSM_TASK3_FINISH_PROGRESS,
    "TASK3_ERROR",
    "TASK3_BAY_CONFIRMED",
    "TASK3_DOCKING_RESULT",
    "TASK3_SPRAYER_WINDOW",
    "TASK3_FIREFIGHTING_RESULT",
    "TASK3_ACK",
    "TASK3_END_ACK",
    "HEARTBEAT_STALE",
    "HEARTBEAT_RECOVERED",
}


def _text(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value.strip()


def _legacy_message(
    *,
    task: str,
    progress: str | None,
    error: Any = None,
    **extra: Any,
) -> dict[str, Any]:
    return {
        "vehicle": "usv",
        "task": task,
        "progress": progress,
        "error": error,
        **extra,
    }


def validate_fsm_progress(value: Mapping[str, Any], *, allow_task4: bool = False) -> None:
    """Validate the small progress envelope consumed by ``mission_planner_node``.

    The upstream FSM does not decode the former 14-bit ``status_code`` or
    ``status_bits`` fields.  Keeping validation here prevents a child module
    from accidentally publishing the old packed representation on the ROS
    boundary while still allowing structured result fields beside the four
    fields used by the planner.
    """

    if not isinstance(value, Mapping):
        raise TypeError("FSM progress must be a JSON object")
    vehicle = _text(value.get("vehicle"), "vehicle").lower()
    if vehicle not in {"uav", "usv"}:
        raise ValueError("vehicle must be uav or usv")
    task = _text(value.get("task"), "task").upper()
    allowed_tasks = {"MAP", "TASK1", "TASK3"}
    if allow_task4:
        allowed_tasks.add("TASK4")
    if task not in allowed_tasks:
        raise ValueError(f"unsupported FSM progress task: {task}")
    progress = _text(value.get("progress"), "progress").upper()
    if task == "TASK3" and progress not in TASK3_WIRE_PROGRESS:
        raise ValueError(f"unsupported TASK3 progress: {progress}")
    if value.get("error") is not None and not isinstance(value.get("error"), str):
        raise ValueError("error must be null or a string")
    if "status_code" in value or "status_bits" in value:
        raise ValueError("packed status fields are internal-only and cannot be sent to FSM")


def normalize_fsm_task_command(value: Mapping[str, Any]) -> dict[str, Any] | None:
    """Normalize one command emitted by ``mission_planner_node``.

    ``None`` means the command belongs to MAP or Task 1 and must be handled by
    another USV module.  The Task3 adapter must not consume those commands.
    """

    if not isinstance(value, Mapping):
        raise TypeError("FSM command must be a JSON object")
    task = _text(value.get("task"), "task").upper()
    if task not in {"TASK3", "TASK4"}:
        return None
    vehicle = _text(value.get("vehicle"), "vehicle").lower()
    if vehicle != "usv":
        raise ValueError("Task3 adapter only accepts vehicle=usv commands")

    raw_command = value.get("command")
    if isinstance(raw_command, str):
        command = raw_command.lower()
        if command == "begin" and task == "TASK3":
            return {
                "command_id": "fsm-task3-begin",
                "task": "TASK3",
                "command": "START_TASK3",
                "payload": {},
                "fsm_command": dict(value),
            }
        if command == "resume" and task == "TASK3":
            return {
                "command_id": "fsm-task3-resume",
                "task": "TASK3",
                "command": "RESUME_TASK3",
                "payload": {"progress": value.get("progress")},
                "fsm_command": dict(value),
            }
        if command == "end":
            return {
                "command_id": f"fsm-{task.lower()}-end",
                "task": task,
                "command": "SAFE_STOP",
                "payload": {"error": value.get("error")},
                "fsm_command": dict(value),
            }
        raise ValueError(f"unsupported FSM command: task={task}, command={raw_command}")

    if task != "TASK4" or not isinstance(raw_command, Mapping):
        raise ValueError("TASK4 command must be an object")
    command_type = _text(raw_command.get("type"), "command.type").upper()
    if command_type != "TASK4_BEGIN":
        raise ValueError(f"unsupported TASK4 command type: {command_type}")
    command_seq = raw_command.get("command_seq")
    if isinstance(command_seq, bool) or not isinstance(command_seq, int) or command_seq <= 0:
        raise ValueError("TASK4_BEGIN command_seq must be a positive integer")
    position = raw_command.get("position")
    if not isinstance(position, Mapping) or not position:
        raise ValueError("TASK4_BEGIN position must be a non-empty object")
    return {
        "command_id": f"fsm-task4-{command_seq}",
        "task": "TASK4",
        "command": "TASK4_GOTO",
        "payload": {
            "target_position": dict(position),
            "command_seq": command_seq,
            "target_vehicle": raw_command.get("target_vehicle"),
        },
        "fsm_command": dict(value),
    }


def executor_state_to_fsm_progress(value: Mapping[str, Any]) -> dict[str, Any]:
    """Map a Task3 executor state to the legacy FSM progress shape."""

    if not isinstance(value, Mapping):
        raise TypeError("executor state must be a JSON object")
    state = _text(value.get("state"), "state").upper()
    if state in {"FINISHED", "COMPLETE", "TASK3_FINISH"}:
        progress = FSM_TASK3_FINISH_PROGRESS
        error = None
    elif state == "ERROR":
        progress = "TASK3_ERROR"
        error = value.get("error") or value.get("detail") or "UNKNOWN_ERROR"
    else:
        progress = state if state.startswith("TASK3_") else f"TASK3_{state}"
        error = value.get("error")
    report = _legacy_message(
        task="TASK3",
        progress=progress,
        error=error,
        detail=value.get("detail", ""),
    )
    if value.get("retry") is not None:
        report["retry"] = value["retry"]
    return report


def task4_ack_progress(command: Mapping[str, Any]) -> dict[str, Any]:
    """Create the progress message FSM checks after TASK4_BEGIN."""

    return _legacy_message(
        task="TASK4",
        progress="TASK4_ACK",
        command_seq=command.get("payload", {}).get("command_seq"),
    )


def safe_stop_ack_progress(command: Mapping[str, Any]) -> dict[str, Any]:
    """Acknowledge the FSM ``command=end`` without claiming task completion."""

    task = _text(command.get("task"), "task").upper()
    if task not in {"TASK3", "TASK4"}:
        raise ValueError("safe-stop ACK task must be TASK3 or TASK4")
    return _legacy_message(
        task=task,
        progress=f"{task}_END_ACK",
        error=None,
        accepted=True,
        return_to_start_required=True,
        safe_actions=[
            "CANCEL_ACTIVE_NAVIGATION",
            "PUMP_OFF",
            "STOP_GIMBAL_TRACKING",
        ],
    )


def correct_bay_to_fsm_progress(info: Mapping[str, Any]) -> dict[str, Any]:
    return _legacy_message(
        task="TASK3",
        progress="TASK3_BAY_CONFIRMED",
        error=None,
        bay_id=info["bay_id"],
        dock_id=info["dock_id"],
        confidence=info["confidence"],
        docking_confirmed=info["docking_confirmed"],
    )


def docking_result_to_fsm_message(info: Mapping[str, Any]) -> dict[str, Any]:
    return _legacy_message(
        task="TASK3",
        progress="TASK3_DOCKING_RESULT",
        error=None,
        phase="DOCKING_COMPLETE",
        authority="USV",
        official_report={
            "type": "docking",
            "vehicle_id": "T-Wave",
            "bay_id": info["bay_id"],
            "dock_id": info["dock_id"],
            "confidence": info["confidence"],
        },
    )


def sprayer_window_to_fsm_progress(info: Mapping[str, Any]) -> dict[str, Any]:
    return _legacy_message(
        task="TASK3",
        progress="TASK3_SPRAYER_WINDOW",
        error=None,
        window_id=info["window_id"],
        bay_id=info.get("bay_id"),
        dock_id=info.get("dock_id"),
        color=info["color"],
        confidence=info["confidence"],
        pump_off=info["pump_off"],
        green_confirmed_duration_s=info["green_confirmed_duration_s"],
        completion_confirmed=info["completion_confirmed"],
    )


def firefighting_result_to_fsm_message(info: Mapping[str, Any]) -> dict[str, Any]:
    return _legacy_message(
        task="TASK3",
        progress="TASK3_FIREFIGHTING_RESULT",
        error=None,
        phase="FIREFIGHTING_COMPLETE",
        authority="USV",
        bay_id=info.get("bay_id"),
        dock_id=info.get("dock_id"),
        official_report={
            "type": "firefighting",
            "vehicle_id": "T-Wave",
            "window_id": info["window_id"],
            "color": info["color"],
        },
    )


def heartbeat_fault_to_fsm_message(vehicle_id: str, *, stale: bool, age_s: float) -> dict[str, Any]:
    return _legacy_message(
        task="TASK3",
        progress="HEARTBEAT_STALE" if stale else "HEARTBEAT_RECOVERED",
        error="COMMUNICATION_LOST" if stale else None,
        heartbeat_vehicle=vehicle_id,
        age_s=float(age_s),
    )
