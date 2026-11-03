"""Transport-neutral Task 3 child-FSM/main-planner communication bridge.

This module does not publish ROS 2 topics or control hardware.  It implements
the protocol decisions that both sides need to agree on:

* normal status reports use command bits ``000``;
* structured payloads carry values that do not fit in the 14-bit status word;
* Task 4 preemption saves a Task 3 checkpoint before accepting a go-to target;
* reaching the Task 4 target only reports completion and enters WAIT_RESUME;
* the main planner chooses whether Task 1 or Task 3 continues;
* Task 3 resumes only after an explicit RESUME_TASK3 command.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
import math
import time
import uuid
from typing import Any, Mapping

from .task3_status_codec import (
    CommonError,
    PlannerCommand,
    Task3Step,
    Task4Step,
    TaskCode,
    TaskStatusCode,
    VehicleCode,
    error_definition,
    step_name,
)


class MasterCommandType(str, Enum):
    START_TASK3 = "START_TASK3"
    PAUSE_TASK3 = "PAUSE_TASK3"
    TASK4_GOTO = "TASK4_GOTO"
    RESUME_TASK3 = "RESUME_TASK3"
    SAFE_STOP = "SAFE_STOP"
    RESET = "RESET"


class ResumePolicy(str, Enum):
    CONTINUE_SAME_STEP = "CONTINUE_SAME_STEP"
    WAIT_FOR_FRESH_DOCK_DATA = "WAIT_FOR_FRESH_DOCK_DATA"
    REPLAN_APPROACH_FROM_CURRENT_POSE = "REPLAN_APPROACH_FROM_CURRENT_POSE"
    REOBSERVE_DOCK_AND_BAY = "REOBSERVE_DOCK_AND_BAY"
    REDOCK_AND_RESTART_WATERING = "REDOCK_AND_RESTART_WATERING"
    REQUIRE_FRESH_COLOR_SEQUENCE = "REQUIRE_FRESH_COLOR_SEQUENCE"
    REQUERY_UAV_DELIVERY_STATE = "REQUERY_UAV_DELIVERY_STATE"
    REPLAN_RETURN = "REPLAN_RETURN"
    NO_ACTION_TASK_ALREADY_COMPLETE = "NO_ACTION_TASK_ALREADY_COMPLETE"


@dataclass(frozen=True)
class MasterCommand:
    command_id: str
    run_id: int
    command: MasterCommandType
    payload: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "MasterCommand":
        if not isinstance(value, Mapping):
            raise TypeError("master command must be a mapping")
        command_id = value.get("command_id")
        run_id = value.get("run_id")
        payload = value.get("payload", {})
        if not isinstance(command_id, str) or not command_id:
            raise ValueError("command_id must be a non-empty string")
        if isinstance(run_id, bool) or not isinstance(run_id, int) or run_id <= 0:
            raise ValueError("run_id must be a positive integer")
        if not isinstance(payload, dict):
            raise ValueError("payload must be a JSON object")
        return cls(command_id, run_id, MasterCommandType(value.get("command")), dict(payload))


@dataclass(frozen=True)
class Task3Checkpoint:
    checkpoint_id: str
    run_id: int
    saved_step: Task3Step
    resume_step: Task3Step
    resume_policy: ResumePolicy
    saved_at_unix_ms: int
    context: dict[str, Any]
    safe_actions: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["saved_step"] = self.saved_step.name
        value["resume_step"] = self.resume_step.name
        value["resume_policy"] = self.resume_policy.value
        value["safe_actions"] = list(self.safe_actions)
        return value


@dataclass(frozen=True)
class SubFSMReport:
    sequence: int
    run_id: int | None
    message_type: str
    status: TaskStatusCode
    payload: dict[str, Any] = field(default_factory=dict)
    in_reply_to: str | None = None

    def to_dict(self) -> dict[str, Any]:
        """Return the compact FSM wire shape.

        The packed status remains available through ``to_internal_dict`` for
        diagnostics, but it must not be sent to the main mission planner.
        """
        task_name = self.status.task.name
        if self.status.task is TaskCode.TASK3:
            progress = "TASK3_FINISH" if self.status.step == int(Task3Step.COMPLETE) else f"TASK3_{step_name(self.status.task, self.status.step)}"
        elif self.status.task is TaskCode.TASK4:
            progress = f"TASK4_{step_name(self.status.task, self.status.step)}"
        else:
            progress = task_name
        error = None if self.status.error == int(CommonError.NONE) else error_definition(
            self.status.task, self.status.step, self.status.error
        ).name
        value = {
            "vehicle": "usv" if self.status.vehicle is VehicleCode.USV else "uav",
            "task": task_name,
            "progress": progress,
            "error": error,
            **dict(self.payload),
        }
        if self.in_reply_to is not None:
            value["in_reply_to"] = self.in_reply_to
        return value

    def to_internal_dict(self) -> dict[str, Any]:
        """Return the former diagnostic representation for local logs only."""
        return {
            "sequence": self.sequence,
            "run_id": self.run_id,
            "message_type": self.message_type,
            "status_code": self.status.pack(),
            "status_bits": self.status.bit_string(),
            "status": self.status.describe(),
            "payload": dict(self.payload),
            "in_reply_to": self.in_reply_to,
        }


_RESUME_RULES: Mapping[Task3Step, tuple[Task3Step, ResumePolicy]] = {
    Task3Step.READY: (Task3Step.READY, ResumePolicy.CONTINUE_SAME_STEP),
    Task3Step.WAIT_DOCK_OBSERVATION: (
        Task3Step.WAIT_DOCK_OBSERVATION,
        ResumePolicy.WAIT_FOR_FRESH_DOCK_DATA,
    ),
    Task3Step.APPROACH_DOCK: (
        Task3Step.APPROACH_DOCK,
        ResumePolicy.REPLAN_APPROACH_FROM_CURRENT_POSE,
    ),
    # Task 4 moves the boat away from the saved circular-search position, so
    # the old arc index must not be resumed blindly.
    Task3Step.SEARCH_DOCK: (
        Task3Step.APPROACH_DOCK,
        ResumePolicy.REPLAN_APPROACH_FROM_CURRENT_POSE,
    ),
    Task3Step.CHECK_BAY_COLOR: (
        Task3Step.SEARCH_DOCK,
        ResumePolicy.REOBSERVE_DOCK_AND_BAY,
    ),
    Task3Step.NEXT_BAY: (Task3Step.SEARCH_DOCK, ResumePolicy.REOBSERVE_DOCK_AND_BAY),
    Task3Step.TERMINAL_DOCKING: (
        Task3Step.SEARCH_DOCK,
        ResumePolicy.REOBSERVE_DOCK_AND_BAY,
    ),
    # Task 4 go-to necessarily breaks docking.  Watering confirmation is reset
    # and the USV must find and dock in the bay again before operating the pump.
    Task3Step.DOCKED: (
        Task3Step.APPROACH_DOCK,
        ResumePolicy.REDOCK_AND_RESTART_WATERING,
    ),
    Task3Step.WAIT_RED_WINDOW: (
        Task3Step.APPROACH_DOCK,
        ResumePolicy.REDOCK_AND_RESTART_WATERING,
    ),
    Task3Step.AIMING: (
        Task3Step.APPROACH_DOCK,
        ResumePolicy.REDOCK_AND_RESTART_WATERING,
    ),
    Task3Step.SPRAYING: (
        Task3Step.APPROACH_DOCK,
        ResumePolicy.REDOCK_AND_RESTART_WATERING,
    ),
    Task3Step.VERIFY_GREEN: (
        Task3Step.APPROACH_DOCK,
        ResumePolicy.REDOCK_AND_RESTART_WATERING,
    ),
    Task3Step.COLOR_SEQUENCE: (
        Task3Step.COLOR_SEQUENCE,
        ResumePolicy.REQUIRE_FRESH_COLOR_SEQUENCE,
    ),
    Task3Step.WAIT_UAV_DELIVERY: (
        Task3Step.WAIT_UAV_DELIVERY,
        ResumePolicy.REQUERY_UAV_DELIVERY_STATE,
    ),
    Task3Step.EXIT_RETURN: (Task3Step.EXIT_RETURN, ResumePolicy.REPLAN_RETURN),
    Task3Step.COMPLETE: (
        Task3Step.COMPLETE,
        ResumePolicy.NO_ACTION_TASK_ALREADY_COMPLETE,
    ),
}


class Task3SubFSMBridge:
    """Pure state/communication bridge owned by the Task 3 child FSM."""

    def __init__(self, *, clock_ms=None) -> None:
        self._clock_ms = clock_ms or (lambda: int(time.time() * 1000))
        self.run_id: int | None = None
        self.active_task = TaskCode.NOT_STARTED
        self.task3_step = Task3Step.READY
        self.task4_step = Task4Step.READY
        self.context: dict[str, Any] = {}
        self.checkpoint: Task3Checkpoint | None = None
        self._sequence = 0
        self._processed_commands: dict[str, SubFSMReport] = {}

    def status_report(
        self,
        *,
        error: int = CommonError.NONE,
        detail: str = "",
        fault_source: str = "",
        detail_code: str = "",
        retry_count: int = 0,
        message_type: str = "TASK3_PROGRESS",
    ) -> SubFSMReport:
        """Return a normal status report; command bits are always 000."""

        for field_name, value in (
            ("detail", detail),
            ("fault_source", fault_source),
            ("detail_code", detail_code),
        ):
            if not isinstance(value, str):
                raise TypeError(f"{field_name} must be a string")
        if isinstance(retry_count, bool) or not isinstance(retry_count, int) or retry_count < 0:
            raise ValueError("retry_count must be a non-negative integer")
        if int(error) == int(CommonError.UNKNOWN_ERROR) and not detail.strip():
            raise ValueError("UNKNOWN_ERROR requires non-empty detail")
        step = int(self.task4_step if self.active_task is TaskCode.TASK4 else self.task3_step)
        definition = error_definition(self.active_task, step, int(error))
        payload: dict[str, Any] = {}
        if int(error) != int(CommonError.NONE):
            payload["fault"] = {
                "name": definition.name,
                "source": fault_source,
                "detail_code": detail_code,
                "detail": detail,
                "retry_count": retry_count,
                "retryable": definition.retryable,
                "recovery_action": definition.recovery.value,
            }
        elif detail:
            payload["detail"] = detail
        return self._emit(
            message_type,
            error=int(error),
            command=PlannerCommand.NONE,
            payload=payload,
        )

    def update_task3_step(
        self,
        step: Task3Step,
        *,
        context: Mapping[str, Any] | None = None,
        error: int = CommonError.NONE,
        detail: str = "",
        fault_source: str = "",
        detail_code: str = "",
        retry_count: int = 0,
    ) -> SubFSMReport:
        if self.active_task is not TaskCode.TASK3:
            raise RuntimeError("Task3 step can only advance while TASK3 is active")
        self.task3_step = Task3Step(step)
        if context is not None:
            self.context = dict(context)
        return self.status_report(
            error=error,
            detail=detail,
            fault_source=fault_source,
            detail_code=detail_code,
            retry_count=retry_count,
        )

    def handle_master_command(self, raw_command: Mapping[str, Any] | MasterCommand) -> SubFSMReport:
        command = (
            raw_command
            if isinstance(raw_command, MasterCommand)
            else MasterCommand.from_mapping(raw_command)
        )
        if command.command_id in self._processed_commands:
            return self._processed_commands[command.command_id]

        if command.command is MasterCommandType.START_TASK3:
            report = self._start_task3(command)
        elif command.command is MasterCommandType.PAUSE_TASK3:
            report = self._pause_task3(command)
        elif command.command is MasterCommandType.TASK4_GOTO:
            report = self._preempt_for_task4(command)
        elif command.command is MasterCommandType.RESUME_TASK3:
            report = self._resume_task3(command)
        elif command.command is MasterCommandType.SAFE_STOP:
            report = self._safe_stop(command)
        elif command.command is MasterCommandType.RESET:
            report = self._reset(command)
        else:  # pragma: no cover - Enum construction rejects this first.
            raise ValueError(f"unsupported master command: {command.command}")

        self._processed_commands[command.command_id] = report
        return report

    def task4_target_reached(self, *, actual_position: Mapping[str, Any]) -> SubFSMReport:
        """Report Task4 completion and wait; never resume Task3 automatically."""

        if self.active_task is not TaskCode.TASK4 or self.task4_step is not Task4Step.GO_TO_TARGET:
            raise RuntimeError("Task4 target can only complete while GO_TO_TARGET is active")
        if not isinstance(actual_position, Mapping):
            raise TypeError("actual_position must be a mapping")
        self.task4_step = Task4Step.TARGET_REACHED
        report = self._emit(
            "TASK4_TARGET_REACHED",
            command=PlannerCommand.REPORT_TASK_COMPLETE,
            payload={
                "actual_position": dict(actual_position),
                "checkpoint_id": self.checkpoint.checkpoint_id if self.checkpoint else None,
            },
        )
        self.task4_step = Task4Step.WAIT_RESUME
        return report

    def report_color_sequence(
        self,
        *,
        resource_color: str,
        destination_color: str | None,
        confidence: float,
        valid: bool = True,
    ) -> SubFSMReport:
        if self.active_task is not TaskCode.TASK3:
            raise RuntimeError("color sequence can only be reported during Task3")
        normalized_resource = _color(resource_color, "resource_color")
        normalized_destination = (
            None if destination_color is None else _color(destination_color, "destination_color")
        )
        if isinstance(confidence, bool) or not isinstance(confidence, (int, float)):
            raise ValueError("confidence must be a finite number in [0, 1]")
        if not math.isfinite(confidence) or not 0 <= confidence <= 1:
            raise ValueError("confidence must be a finite number in [0, 1]")
        self.task3_step = Task3Step.COLOR_SEQUENCE
        return self._emit(
            "TASK3_COLOR_SEQUENCE",
            payload={
                "resource_color": normalized_resource,
                "destination_color": normalized_destination,
                "confidence": float(confidence),
                "valid": bool(valid),
            },
        )

    def report_task3_complete(self, *, result: Mapping[str, Any]) -> SubFSMReport:
        if self.active_task is not TaskCode.TASK3:
            raise RuntimeError("Task3 completion can only be reported during Task3")
        self.task3_step = Task3Step.COMPLETE
        return self._emit(
            "TASK3_COMPLETE",
            command=PlannerCommand.REPORT_TASK_COMPLETE,
            payload={"result": dict(result), "fsm_progress": "TASK3_FINISH"},
        )

    def _start_task3(self, command: MasterCommand) -> SubFSMReport:
        if self.run_id not in (None, command.run_id):
            raise ValueError("START_TASK3 run_id conflicts with the active run")
        self.run_id = command.run_id
        self.active_task = TaskCode.TASK3
        self.task3_step = Task3Step.READY
        self.context = dict(command.payload.get("context", {}))
        self.checkpoint = None
        return self._emit(
            "TASK3_START_ACK",
            command=PlannerCommand.ACK,
            payload={"accepted": True},
            in_reply_to=command.command_id,
        )

    def _pause_task3(self, command: MasterCommand) -> SubFSMReport:
        self._require_run_and_task3(command)
        self.checkpoint = _build_checkpoint(
            command.run_id,
            self.task3_step,
            self.context,
            self._clock_ms(),
            command.payload,
        )
        return self._emit(
            "TASK3_PAUSE_ACK",
            command=PlannerCommand.ACK,
            payload={
                "accepted": True,
                "safe_paused": True,
                "checkpoint": self.checkpoint.to_dict(),
                "safe_actions": list(self.checkpoint.safe_actions),
            },
            in_reply_to=command.command_id,
        )

    def _preempt_for_task4(self, command: MasterCommand) -> SubFSMReport:
        self._require_run_and_task3(command)
        target_position = command.payload.get("target_position")
        if not isinstance(target_position, dict) or not target_position:
            raise ValueError("TASK4_GOTO requires a target_position object")
        self.checkpoint = _build_checkpoint(
            command.run_id,
            self.task3_step,
            self.context,
            self._clock_ms(),
            command.payload,
        )
        self.active_task = TaskCode.TASK4
        self.task4_step = Task4Step.GO_TO_TARGET
        return self._emit(
            "TASK4_GOTO_ACK",
            command=PlannerCommand.ACK,
            payload={
                "accepted": True,
                "target_position": dict(target_position),
                "checkpoint": self.checkpoint.to_dict(),
                "safe_actions": list(self.checkpoint.safe_actions),
            },
            in_reply_to=command.command_id,
        )

    def _resume_task3(self, command: MasterCommand) -> SubFSMReport:
        if self.run_id != command.run_id:
            raise ValueError("RESUME_TASK3 run_id does not match the active run")
        if self.active_task is not TaskCode.TASK4 or self.task4_step not in (
            Task4Step.TARGET_REACHED,
            Task4Step.WAIT_RESUME,
        ):
            raise RuntimeError("Task3 can only resume after the Task4 target is reached")
        if self.checkpoint is None or self.checkpoint.run_id != command.run_id:
            raise RuntimeError("Task3 checkpoint is missing or invalid")
        requested_id = command.payload.get("checkpoint_id")
        if requested_id is not None and requested_id != self.checkpoint.checkpoint_id:
            raise ValueError("RESUME_TASK3 checkpoint_id does not match")
        self.task4_step = Task4Step.RESUMING
        self.active_task = TaskCode.TASK3
        self.task3_step = self.checkpoint.resume_step
        self.context = dict(self.checkpoint.context)
        report = self._emit(
            "TASK3_RESUME_ACK",
            command=PlannerCommand.ACK,
            payload={
                "accepted": True,
                "saved_step": self.checkpoint.saved_step.name,
                "resume_step": self.checkpoint.resume_step.name,
                "resume_policy": self.checkpoint.resume_policy.value,
                "restored_context": dict(self.context),
            },
            in_reply_to=command.command_id,
        )
        return report

    def _safe_stop(self, command: MasterCommand) -> SubFSMReport:
        if self.run_id not in (None, command.run_id):
            raise ValueError("SAFE_STOP run_id does not match the active run")
        self.run_id = command.run_id
        task_name = "TASK4" if self.active_task is TaskCode.TASK4 else "TASK3"
        return self._emit(
            "TASK3_SAFE_STOP_ACK",
            command=PlannerCommand.ACK,
            payload={
                "accepted": True,
                "fsm_progress": f"{task_name}_END_ACK",
                "return_to_start_required": True,
                "safe_actions": [
                    "CANCEL_ACTIVE_NAVIGATION",
                    "PUMP_OFF",
                    "STOP_GIMBAL_TRACKING",
                ],
            },
            in_reply_to=command.command_id,
        )

    def _reset(self, command: MasterCommand) -> SubFSMReport:
        self.run_id = command.run_id
        self.active_task = TaskCode.NOT_STARTED
        self.task3_step = Task3Step.READY
        self.task4_step = Task4Step.READY
        self.context = {}
        self.checkpoint = None
        return self._emit(
            "TASK3_RESET_ACK",
            command=PlannerCommand.ACK,
            payload={"accepted": True},
            in_reply_to=command.command_id,
        )

    def _require_run_and_task3(self, command: MasterCommand) -> None:
        if self.run_id != command.run_id:
            raise ValueError("command run_id does not match the active run")
        if self.active_task is not TaskCode.TASK3:
            raise RuntimeError("command requires Task3 to be active")

    def _emit(
        self,
        message_type: str,
        *,
        error: int = CommonError.NONE,
        command: PlannerCommand = PlannerCommand.NONE,
        payload: Mapping[str, Any] | None = None,
        in_reply_to: str | None = None,
    ) -> SubFSMReport:
        self._sequence += 1
        step = int(self.task4_step if self.active_task is TaskCode.TASK4 else self.task3_step)
        status = TaskStatusCode(
            VehicleCode.USV,
            self.active_task,
            step=step,
            error=int(error),
            command=command,
        )
        return SubFSMReport(
            sequence=self._sequence,
            run_id=self.run_id,
            message_type=message_type,
            status=status,
            payload=dict(payload or {}),
            in_reply_to=in_reply_to,
        )


def _build_checkpoint(
    run_id: int,
    step: Task3Step,
    context: Mapping[str, Any],
    now_ms: int,
    command_payload: Mapping[str, Any],
) -> Task3Checkpoint:
    resume_step, policy = _RESUME_RULES[step]
    safe_actions = ["CANCEL_ACTIVE_NAVIGATION"]
    if step in (Task3Step.AIMING, Task3Step.SPRAYING, Task3Step.VERIFY_GREEN):
        safe_actions.append("STOP_GIMBAL_TRACKING")
    if step in (Task3Step.SPRAYING, Task3Step.VERIFY_GREEN):
        safe_actions.append("PUMP_OFF")
    if step in (
        Task3Step.DOCKED,
        Task3Step.WAIT_RED_WINDOW,
        Task3Step.AIMING,
        Task3Step.SPRAYING,
        Task3Step.VERIFY_GREEN,
    ):
        safe_actions.append("LEAVE_DOCK_SAFELY_FOR_TASK4")

    saved_context = dict(context)
    if policy is ResumePolicy.REDOCK_AND_RESTART_WATERING:
        saved_context.update({
            "requires_redock": True,
            "reset_watering_confirmation": True,
        })
    if policy is ResumePolicy.REQUIRE_FRESH_COLOR_SEQUENCE:
        saved_context["require_fresh_color_sequence"] = True
    if policy is ResumePolicy.REQUERY_UAV_DELIVERY_STATE:
        saved_context["requery_uav_delivery_state"] = True
    saved_context["task4_target_position"] = command_payload.get("target_position")

    return Task3Checkpoint(
        checkpoint_id=str(uuid.uuid4()),
        run_id=run_id,
        saved_step=step,
        resume_step=resume_step,
        resume_policy=policy,
        saved_at_unix_ms=now_ms,
        context=saved_context,
        safe_actions=tuple(dict.fromkeys(safe_actions)),
    )


def _color(value: str, field_name: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{field_name} must be RED, GREEN, or BLUE")
    normalized = value.strip().upper()
    if normalized not in {"RED", "GREEN", "BLUE"}:
        raise ValueError(f"{field_name} must be RED, GREEN, or BLUE")
    return normalized


__all__ = [
    "MasterCommand",
    "MasterCommandType",
    "ResumePolicy",
    "SubFSMReport",
    "Task3Checkpoint",
    "Task3SubFSMBridge",
]
