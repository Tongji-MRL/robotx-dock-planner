"""Compact Task 3 child-FSM *internal* diagnostic status encoding.

The 14-bit diagnostic value follows the team layout exactly::

    vehicle(1) | task(2) | progress(4) | error(4) | command(3)

``command=000`` is the normal diagnostic status value.  The ROS boundary to
the fixed FSM mission planner uses the simplified JSON progress fields from
``fsm_contract.py``; this integer must never be sent on that wire.

Error codes are scoped by progress.  Codes 0000..0111 keep the same meaning
for every progress step; codes 1000..1111 are step-specific and may be reused
by another progress step.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Mapping


TASK3_ERROR_CATALOG_VERSION = "task3-errors-v1"


class VehicleCode(IntEnum):
    """The order follows the team's ``Vehicle (uav/usv)`` definition."""

    UAV = 0b0
    USV = 0b1


class TaskCode(IntEnum):
    NOT_STARTED = 0b00
    TASK1 = 0b01
    TASK3 = 0b10
    TASK4 = 0b11


class Task3Step(IntEnum):
    READY = 0b0000
    WAIT_DOCK_OBSERVATION = 0b0001
    APPROACH_DOCK = 0b0010
    SEARCH_DOCK = 0b0011
    CHECK_BAY_COLOR = 0b0100
    NEXT_BAY = 0b0101
    TERMINAL_DOCKING = 0b0110
    DOCKED = 0b0111
    WAIT_RED_WINDOW = 0b1000
    AIMING = 0b1001
    SPRAYING = 0b1010
    VERIFY_GREEN = 0b1011
    COLOR_SEQUENCE = 0b1100
    WAIT_UAV_DELIVERY = 0b1101
    EXIT_RETURN = 0b1110
    COMPLETE = 0b1111


class Task4Step(IntEnum):
    READY = 0b0000
    GO_TO_TARGET = 0b0001
    TARGET_REACHED = 0b0010
    WAIT_RESUME = 0b0011
    RESUMING = 0b0100
    COMPLETE = 0b0101


class CommonError(IntEnum):
    """Codes shared by every progress step."""

    NONE = 0b0000
    TIMEOUT = 0b0001
    INVALID_INPUT = 0b0010
    STALE_DATA = 0b0011
    COMMUNICATION_LOST = 0b0100
    EXECUTION_REJECTED = 0b0101
    DEPENDENCY_UNAVAILABLE = 0b0110
    UNKNOWN_ERROR = 0b0111


class PlannerCommand(IntEnum):
    """Requests/replies sent from the child FSM to the main planner."""

    NONE = 0b000
    ACK = 0b001
    REQUEST_PAUSE = 0b010
    REQUEST_RESUME = 0b011
    REPORT_TASK_COMPLETE = 0b100
    REPORT_TASK_FAILED = 0b101
    REQUEST_SAFE_HOLD = 0b110
    REQUEST_ABORT = 0b111


class RecoveryAction(str, Enum):
    """Fixed recovery hint understood by the main planner."""

    NO_ACTION = "NO_ACTION"
    RETRY_CURRENT_STEP = "RETRY_CURRENT_STEP"
    WAIT_FOR_RECOVERY = "WAIT_FOR_RECOVERY"
    REPLAN_FROM_CURRENT_POSE = "REPLAN_FROM_CURRENT_POSE"
    SELECT_NEXT_BAY = "SELECT_NEXT_BAY"
    RESTART_DOCK_SEARCH = "RESTART_DOCK_SEARCH"
    RESTART_FROM_APPROACH = "RESTART_FROM_APPROACH"
    SAFE_HOLD = "SAFE_HOLD"
    ABORT_TASK = "ABORT_TASK"


@dataclass(frozen=True)
class ErrorDefinition:
    """Machine-readable meaning attached to one error code."""

    name: str
    retryable: bool
    recovery: RecoveryAction


def _err(
    name: str,
    recovery: RecoveryAction,
    retryable: bool = True,
) -> ErrorDefinition:
    return ErrorDefinition(name=name, retryable=retryable, recovery=recovery)


COMMON_ERROR_DEFINITIONS: Mapping[int, ErrorDefinition] = {
    CommonError.NONE: _err("NONE", RecoveryAction.NO_ACTION, False),
    CommonError.TIMEOUT: _err("TIMEOUT", RecoveryAction.RETRY_CURRENT_STEP),
    CommonError.INVALID_INPUT: _err("INVALID_INPUT", RecoveryAction.WAIT_FOR_RECOVERY),
    CommonError.STALE_DATA: _err("STALE_DATA", RecoveryAction.WAIT_FOR_RECOVERY),
    CommonError.COMMUNICATION_LOST: _err("COMMUNICATION_LOST", RecoveryAction.SAFE_HOLD),
    CommonError.EXECUTION_REJECTED: _err("EXECUTION_REJECTED", RecoveryAction.WAIT_FOR_RECOVERY),
    CommonError.DEPENDENCY_UNAVAILABLE: _err("DEPENDENCY_UNAVAILABLE", RecoveryAction.WAIT_FOR_RECOVERY),
    CommonError.UNKNOWN_ERROR: _err("UNKNOWN_ERROR", RecoveryAction.SAFE_HOLD, False),
}


# Codes 8..15 are local to a progress step.  All eight slots are assigned for
# every Task 3 step, so the main planner never needs to interpret a newly added
# wire code.  An unforeseen failure uses common UNKNOWN_ERROR plus detail text.
TASK3_ERROR_DEFINITIONS: Mapping[Task3Step, Mapping[int, ErrorDefinition]] = {
    Task3Step.READY: {
        0b1000: _err("RUN_NOT_STARTED", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1001: _err("CONFIG_INVALID", RecoveryAction.ABORT_TASK, False),
        0b1010: _err("RUN_ID_CONFLICT", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1011: _err("PREVIOUS_RUN_ACTIVE", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1100: _err("CHECKPOINT_LOAD_FAILED", RecoveryAction.ABORT_TASK, False),
        0b1101: _err("REQUIRED_FRAME_UNDEFINED", RecoveryAction.ABORT_TASK, False),
        0b1110: _err("SAFETY_CONFIG_INVALID", RecoveryAction.ABORT_TASK, False),
        0b1111: _err("START_CONDITION_UNMET", RecoveryAction.WAIT_FOR_RECOVERY),
    },
    Task3Step.WAIT_DOCK_OBSERVATION: {
        0b1000: _err("DOCK_OBSERVATION_NOT_FOUND", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1001: _err("FRAME_MISMATCH", RecoveryAction.SAFE_HOLD, False),
        0b1010: _err("DOCK_ID_MISMATCH", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1011: _err("COORDINATE_CONVERSION_FAILED", RecoveryAction.SAFE_HOLD, False),
        0b1100: _err("OBSERVATION_TIMESTAMP_INVALID", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1101: _err("DOCK_CENTER_LOW_CONFIDENCE", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1110: _err("DOCK_CENTER_OUT_OF_BOUNDARY", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1111: _err("DOCK_CENTER_JUMP_DETECTED", RecoveryAction.WAIT_FOR_RECOVERY),
    },
    Task3Step.APPROACH_DOCK: {
        0b1000: _err("NAV_GOAL_REJECTED", RecoveryAction.REPLAN_FROM_CURRENT_POSE),
        0b1001: _err("BOUNDARY_VIOLATION", RecoveryAction.REPLAN_FROM_CURRENT_POSE),
        0b1010: _err("OBSTACLE_BLOCKED", RecoveryAction.REPLAN_FROM_CURRENT_POSE),
        0b1011: _err("NO_FEASIBLE_SAFETY_RADIUS", RecoveryAction.SAFE_HOLD, False),
        0b1100: _err("START_INSIDE_EXCLUSION_ZONE", RecoveryAction.REPLAN_FROM_CURRENT_POSE),
        0b1101: _err("NAV_FEEDBACK_LOST", RecoveryAction.SAFE_HOLD),
        0b1110: _err("APPROACH_RETRY_EXHAUSTED", RecoveryAction.ABORT_TASK, False),
        0b1111: _err("CURRENT_POSE_UNAVAILABLE", RecoveryAction.SAFE_HOLD),
    },
    Task3Step.SEARCH_DOCK: {
        0b1000: _err("LIDAR_NO_DATA", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1001: _err("SEARCH_ARC_EXHAUSTED", RecoveryAction.REPLAN_FROM_CURRENT_POSE),
        0b1010: _err("NO_ENTRANCE_CANDIDATE", RecoveryAction.RETRY_CURRENT_STEP),
        0b1011: _err("LIDAR_TRANSFORM_UNAVAILABLE", RecoveryAction.SAFE_HOLD),
        0b1100: _err("POINTCLOUD_INVALID", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1101: _err("ENTRANCE_ESTIMATE_UNSTABLE", RecoveryAction.RETRY_CURRENT_STEP),
        0b1110: _err("ALL_ENTRANCES_UNSAFE", RecoveryAction.SAFE_HOLD, False),
        0b1111: _err("SEARCH_ROUTE_BLOCKED", RecoveryAction.REPLAN_FROM_CURRENT_POSE),
    },
    Task3Step.CHECK_BAY_COLOR: {
        0b1000: _err("CAMERA_NO_DATA", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1001: _err("COLOR_LOW_CONFIDENCE", RecoveryAction.RETRY_CURRENT_STEP),
        0b1010: _err("BAY_NOT_SAFE", RecoveryAction.SELECT_NEXT_BAY, False),
        0b1011: _err("BAY_COLOR_UNKNOWN", RecoveryAction.RETRY_CURRENT_STEP),
        0b1100: _err("BAY_COLOR_CONFLICT", RecoveryAction.RETRY_CURRENT_STEP),
        0b1101: _err("BAY_OBSERVATION_LOST", RecoveryAction.RESTART_DOCK_SEARCH),
        0b1110: _err("BAY_ID_MISMATCH", RecoveryAction.RESTART_DOCK_SEARCH),
        0b1111: _err("BAY_CHECK_RETRY_EXHAUSTED", RecoveryAction.SELECT_NEXT_BAY, False),
    },
    Task3Step.NEXT_BAY: {
        0b1000: _err("NO_MORE_BAYS", RecoveryAction.ABORT_TASK, False),
        0b1001: _err("SAFE_EXIT_UNAVAILABLE", RecoveryAction.SAFE_HOLD, False),
        0b1010: _err("BAY_EXIT_GOAL_REJECTED", RecoveryAction.REPLAN_FROM_CURRENT_POSE),
        0b1011: _err("BAY_EXIT_NOT_CONFIRMED", RecoveryAction.RETRY_CURRENT_STEP),
        0b1100: _err("REJECTED_BAY_LIST_INVALID", RecoveryAction.RESTART_DOCK_SEARCH),
        0b1101: _err("NEXT_BAY_CANDIDATE_INVALID", RecoveryAction.RESTART_DOCK_SEARCH),
        0b1110: _err("NON_TARGET_BAY_REENTRY_RISK", RecoveryAction.REPLAN_FROM_CURRENT_POSE),
        0b1111: _err("NEXT_BAY_RETRY_EXHAUSTED", RecoveryAction.ABORT_TASK, False),
    },
    Task3Step.TERMINAL_DOCKING: {
        0b1000: _err("DOCKING_FAILED", RecoveryAction.RESTART_DOCK_SEARCH),
        0b1001: _err("DOCK_NOT_CONFIRMED", RecoveryAction.RESTART_DOCK_SEARCH),
        0b1010: _err("TERMINAL_PATH_UNSAFE", RecoveryAction.REPLAN_FROM_CURRENT_POSE),
        0b1011: _err("DOCK_ALIGNMENT_LOST", RecoveryAction.RESTART_DOCK_SEARCH),
        0b1100: _err("ENTRANCE_POSE_STALE", RecoveryAction.RESTART_DOCK_SEARCH),
        0b1101: _err("COLLISION_MARGIN_VIOLATED", RecoveryAction.SAFE_HOLD, False),
        0b1110: _err("DOCKING_CONTROLLER_FEEDBACK_LOST", RecoveryAction.SAFE_HOLD),
        0b1111: _err("DOCKING_RETRY_EXHAUSTED", RecoveryAction.SELECT_NEXT_BAY, False),
    },
    Task3Step.DOCKED: {
        0b1000: _err("DOCK_REPORT_FAILED", RecoveryAction.RETRY_CURRENT_STEP),
        0b1001: _err("DOCK_POSE_DRIFT", RecoveryAction.RESTART_FROM_APPROACH),
        0b1010: _err("DOCK_CONTACT_LOST", RecoveryAction.RESTART_FROM_APPROACH),
        0b1011: _err("DOCKED_BAY_MISMATCH", RecoveryAction.RESTART_DOCK_SEARCH),
        0b1100: _err("UNSAFE_MOTION_AFTER_DOCK", RecoveryAction.SAFE_HOLD, False),
        0b1101: _err("DOCK_RESULT_CONFLICT", RecoveryAction.SAFE_HOLD, False),
        0b1110: _err("VESSEL_STOP_NOT_CONFIRMED", RecoveryAction.SAFE_HOLD),
        0b1111: _err("WATERING_HANDOFF_NOT_READY", RecoveryAction.WAIT_FOR_RECOVERY),
    },
    Task3Step.WAIT_RED_WINDOW: {
        0b1000: _err("WINDOW_NOT_FOUND", RecoveryAction.RETRY_CURRENT_STEP),
        0b1001: _err("WINDOW_ID_INVALID", RecoveryAction.RETRY_CURRENT_STEP),
        0b1010: _err("RED_WINDOW_NOT_CONFIRMED", RecoveryAction.RETRY_CURRENT_STEP),
        0b1011: _err("WINDOW_LOW_CONFIDENCE", RecoveryAction.RETRY_CURRENT_STEP),
        0b1100: _err("WINDOW_OBSERVATION_LOST", RecoveryAction.RETRY_CURRENT_STEP),
        0b1101: _err("MULTIPLE_RED_WINDOWS", RecoveryAction.RETRY_CURRENT_STEP),
        0b1110: _err("WINDOW_UNREACHABLE", RecoveryAction.RESTART_FROM_APPROACH, False),
        0b1111: _err("WINDOW_WAIT_RETRY_EXHAUSTED", RecoveryAction.ABORT_TASK, False),
    },
    Task3Step.AIMING: {
        0b1000: _err("GIMBAL_NOT_READY", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1001: _err("AIMING_FAILED", RecoveryAction.RETRY_CURRENT_STEP),
        0b1010: _err("AIM_TARGET_LOST", RecoveryAction.RETRY_CURRENT_STEP),
        0b1011: _err("GIMBAL_LIMIT_REACHED", RecoveryAction.RESTART_FROM_APPROACH, False),
        0b1100: _err("TARGET_POSE_INVALID", RecoveryAction.RETRY_CURRENT_STEP),
        0b1101: _err("AIM_ALIGNMENT_TIMEOUT", RecoveryAction.RETRY_CURRENT_STEP),
        0b1110: _err("GIMBAL_FEEDBACK_LOST", RecoveryAction.SAFE_HOLD),
        0b1111: _err("GIMBAL_STABILIZATION_FAILED", RecoveryAction.RESTART_FROM_APPROACH, False),
    },
    Task3Step.SPRAYING: {
        0b1000: _err("PUMP_START_FAILED", RecoveryAction.RETRY_CURRENT_STEP),
        0b1001: _err("PUMP_STOP_FAILED", RecoveryAction.SAFE_HOLD, False),
        0b1010: _err("PUMP_FAULT", RecoveryAction.SAFE_HOLD, False),
        0b1011: _err("SPRAY_ALIGNMENT_LOST", RecoveryAction.SAFE_HOLD),
        0b1100: _err("SPRAY_TARGET_LOST", RecoveryAction.SAFE_HOLD),
        0b1101: _err("MAX_SPRAY_TIME_EXCEEDED", RecoveryAction.SAFE_HOLD, False),
        0b1110: _err("PUMP_STATE_UNKNOWN", RecoveryAction.SAFE_HOLD, False),
        0b1111: _err("SPRAY_EMERGENCY_CUTOFF", RecoveryAction.ABORT_TASK, False),
    },
    Task3Step.VERIFY_GREEN: {
        0b1000: _err("GREEN_CONFIRM_INTERRUPTED", RecoveryAction.RETRY_CURRENT_STEP),
        0b1001: _err("GREEN_CONFIRM_TIMEOUT", RecoveryAction.RETRY_CURRENT_STEP),
        0b1010: _err("GREEN_NOT_DETECTED", RecoveryAction.RETRY_CURRENT_STEP),
        0b1011: _err("RED_WINDOW_PERSISTS", RecoveryAction.RETRY_CURRENT_STEP),
        0b1100: _err("WINDOW_COLOR_UNSTABLE", RecoveryAction.RETRY_CURRENT_STEP),
        0b1101: _err("WINDOW_OBSERVATION_LOST", RecoveryAction.RETRY_CURRENT_STEP),
        0b1110: _err("PUMP_STILL_ON", RecoveryAction.SAFE_HOLD, False),
        0b1111: _err("WINDOW_STATE_CONFLICT", RecoveryAction.SAFE_HOLD, False),
    },
    Task3Step.COLOR_SEQUENCE: {
        0b1000: _err("COLOR_SEQUENCE_INVALID", RecoveryAction.RETRY_CURRENT_STEP),
        0b1001: _err("COLOR_SEQUENCE_TIMEOUT", RecoveryAction.RETRY_CURRENT_STEP),
        0b1010: _err("COLOR_SEQUENCE_CONFLICT", RecoveryAction.RETRY_CURRENT_STEP),
        0b1011: _err("RESOURCE_COLOR_UNKNOWN", RecoveryAction.RETRY_CURRENT_STEP),
        0b1100: _err("DESTINATION_COLOR_UNKNOWN", RecoveryAction.RETRY_CURRENT_STEP),
        0b1101: _err("COLOR_SEQUENCE_INCOMPLETE", RecoveryAction.RETRY_CURRENT_STEP),
        0b1110: _err("COLOR_SEQUENCE_LOW_CONFIDENCE", RecoveryAction.RETRY_CURRENT_STEP),
        0b1111: _err("COLOR_SEQUENCE_PUBLISH_FAILED", RecoveryAction.WAIT_FOR_RECOVERY),
    },
    Task3Step.WAIT_UAV_DELIVERY: {
        0b1000: _err("UAV_NOT_READY", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1001: _err("UAV_ACK_TIMEOUT", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1010: _err("UAV_DELIVERY_FAILED", RecoveryAction.RETRY_CURRENT_STEP),
        0b1011: _err("UAV_STATE_STALE", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1100: _err("UAV_REQUEST_REJECTED", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1101: _err("UAV_REQUEST_ID_MISMATCH", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1110: _err("UAV_DELIVERY_TIMEOUT", RecoveryAction.RETRY_CURRENT_STEP),
        0b1111: _err("UAV_DELIVERY_RESULT_CONFLICT", RecoveryAction.SAFE_HOLD, False),
    },
    Task3Step.EXIT_RETURN: {
        0b1000: _err("RETURN_GOAL_REJECTED", RecoveryAction.REPLAN_FROM_CURRENT_POSE),
        0b1001: _err("RETURN_PATH_BLOCKED", RecoveryAction.REPLAN_FROM_CURRENT_POSE),
        0b1010: _err("DOCK_EXIT_FAILED", RecoveryAction.RETRY_CURRENT_STEP),
        0b1011: _err("RETURN_BOUNDARY_VIOLATION", RecoveryAction.REPLAN_FROM_CURRENT_POSE),
        0b1100: _err("RETURN_CONTROLLER_FEEDBACK_LOST", RecoveryAction.SAFE_HOLD),
        0b1101: _err("RETURN_POSE_UNAVAILABLE", RecoveryAction.SAFE_HOLD),
        0b1110: _err("RETURN_RETRY_EXHAUSTED", RecoveryAction.ABORT_TASK, False),
        0b1111: _err("SAFE_HOLD_FAILED", RecoveryAction.ABORT_TASK, False),
    },
    Task3Step.COMPLETE: {
        0b1000: _err("TASK3_REPORT_FAILED", RecoveryAction.RETRY_CURRENT_STEP),
        0b1001: _err("PLANNER_ACK_TIMEOUT", RecoveryAction.RETRY_CURRENT_STEP),
        0b1010: _err("FINAL_RESULT_INCOMPLETE", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1011: _err("FINAL_RESULT_INCONSISTENT", RecoveryAction.SAFE_HOLD, False),
        0b1100: _err("CHECKPOINT_CLEANUP_FAILED", RecoveryAction.SAFE_HOLD, False),
        0b1101: _err("DUPLICATE_COMPLETION_CONFLICT", RecoveryAction.SAFE_HOLD, False),
        0b1110: _err("COLOR_SEQUENCE_NOT_DELIVERED", RecoveryAction.WAIT_FOR_RECOVERY),
        0b1111: _err("COMPLETION_REJECTED", RecoveryAction.WAIT_FOR_RECOVERY),
    },
}

# Backwards-compatible name-only view used by encoders and existing callers.
TASK3_STEP_ERRORS: Mapping[Task3Step, Mapping[int, str]] = {
    step: {code: definition.name for code, definition in definitions.items()}
    for step, definitions in TASK3_ERROR_DEFINITIONS.items()
}


TASK4_STEP_ERRORS: Mapping[Task4Step, Mapping[int, str]] = {
    Task4Step.READY: {0b1000: "TASK4_TARGET_INVALID"},
    Task4Step.GO_TO_TARGET: {
        0b1000: "TASK4_NAVIGATION_FAILED",
        0b1001: "TASK4_TARGET_UNREACHABLE",
    },
    Task4Step.TARGET_REACHED: {0b1000: "TASK4_REACHED_REPORT_FAILED"},
    Task4Step.WAIT_RESUME: {0b1000: "TASK4_RESUME_TIMEOUT"},
    Task4Step.RESUMING: {
        0b1000: "CHECKPOINT_INVALID",
        0b1001: "TASK3_RESUME_REJECTED",
    },
    Task4Step.COMPLETE: {},
}


_VEHICLE_SHIFT = 13
_TASK_SHIFT = 11
_STEP_SHIFT = 7
_ERROR_SHIFT = 3
_COMMAND_SHIFT = 0
_WIRE_MASK = (1 << 14) - 1


@dataclass(frozen=True)
class TaskStatusCode:
    vehicle: VehicleCode
    task: TaskCode
    step: int = 0
    error: int = 0
    command: PlannerCommand = PlannerCommand.NONE

    def __post_init__(self) -> None:
        object.__setattr__(self, "vehicle", VehicleCode(self.vehicle))
        object.__setattr__(self, "task", TaskCode(self.task))
        object.__setattr__(self, "command", PlannerCommand(self.command))
        for name, value, maximum in (
            ("step", self.step, 0b1111),
            ("error", self.error, 0b1111),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= maximum:
                raise ValueError(f"{name} must fit in 4 bits")
        self._validate_step_error_pair()

    def _validate_step_error_pair(self) -> None:
        if self.error < 0b1000:
            return
        if self.task is TaskCode.TASK3:
            try:
                step = Task3Step(self.step)
            except ValueError as exc:
                raise ValueError("unknown Task3 progress code") from exc
            if self.error not in TASK3_STEP_ERRORS.get(step, {}):
                raise ValueError(
                    f"undefined Task3 step-specific error {self.error:04b} for {step.name}"
                )
        elif self.task is TaskCode.TASK4:
            try:
                step = Task4Step(self.step)
            except ValueError as exc:
                raise ValueError("unknown Task4 progress code") from exc
            if self.error not in TASK4_STEP_ERRORS.get(step, {}):
                raise ValueError(
                    f"undefined Task4 step-specific error {self.error:04b} for {step.name}"
                )

    def pack(self) -> int:
        return (
            (int(self.vehicle) << _VEHICLE_SHIFT)
            | (int(self.task) << _TASK_SHIFT)
            | (self.step << _STEP_SHIFT)
            | (self.error << _ERROR_SHIFT)
            | (int(self.command) << _COMMAND_SHIFT)
        )

    def bit_string(self) -> str:
        return (
            f"{int(self.vehicle):01b} {int(self.task):02b} {self.step:04b} "
            f"{self.error:04b} {int(self.command):03b}"
        )

    def describe(self) -> dict[str, object]:
        definition = error_definition(self.task, self.step, self.error)
        return {
            "packed": self.pack(),
            "bits": self.bit_string(),
            "error_catalog_version": TASK3_ERROR_CATALOG_VERSION,
            "vehicle": self.vehicle.name,
            "task": self.task.name,
            "step": step_name(self.task, self.step),
            "error": definition.name,
            "error_retryable": definition.retryable,
            "recovery_action": definition.recovery.value,
            "command": self.command.name,
        }

    @classmethod
    def unpack(cls, packed: int) -> "TaskStatusCode":
        if isinstance(packed, bool) or not isinstance(packed, int):
            raise TypeError("packed status must be an integer")
        if not 0 <= packed <= _WIRE_MASK:
            raise ValueError("packed status must fit in 14 bits")
        return cls(
            vehicle=VehicleCode((packed >> _VEHICLE_SHIFT) & 0b1),
            task=TaskCode((packed >> _TASK_SHIFT) & 0b11),
            step=(packed >> _STEP_SHIFT) & 0b1111,
            error=(packed >> _ERROR_SHIFT) & 0b1111,
            command=PlannerCommand(packed & 0b111),
        )


def step_name(task: TaskCode, step: int) -> str:
    if task is TaskCode.TASK3:
        return Task3Step(step).name
    if task is TaskCode.TASK4:
        return Task4Step(step).name
    if task is TaskCode.NOT_STARTED and step == 0:
        return "NOT_STARTED"
    return f"STEP_{step:04b}"


def error_name(task: TaskCode, step: int, error: int) -> str:
    return error_definition(task, step, error).name


def error_definition(task: TaskCode, step: int, error: int) -> ErrorDefinition:
    """Return the frozen decoder entry used by the main planner."""

    if error < 0b1000:
        return COMMON_ERROR_DEFINITIONS[CommonError(error)]
    if task is TaskCode.TASK3:
        return TASK3_ERROR_DEFINITIONS[Task3Step(step)][error]
    if task is TaskCode.TASK4:
        return ErrorDefinition(
            TASK4_STEP_ERRORS[Task4Step(step)][error],
            False,
            RecoveryAction.SAFE_HOLD,
        )
    return ErrorDefinition(
        f"STEP_ERROR_{error:04b}",
        False,
        RecoveryAction.SAFE_HOLD,
    )


__all__ = [
    "COMMON_ERROR_DEFINITIONS",
    "CommonError",
    "ErrorDefinition",
    "PlannerCommand",
    "RecoveryAction",
    "TASK3_ERROR_DEFINITIONS",
    "TASK3_ERROR_CATALOG_VERSION",
    "TASK3_STEP_ERRORS",
    "TASK4_STEP_ERRORS",
    "Task3Step",
    "Task4Step",
    "TaskCode",
    "TaskStatusCode",
    "VehicleCode",
    "error_definition",
    "error_name",
    "step_name",
]
