"""ROS-independent USV-only RobotX Task 3 child state machine.

The class in this module coordinates existing planning, perception and control
adapters.  It never commands hardware directly.  Navigation advances only on
explicit controller completion, dock geometry advances only after the existing
multi-frame stabilizer has recovered three entrances, and terminal completion
is represented by the distinct ``COMPLETE`` state.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Mapping, Sequence

from dock_planner.geofence_search import (
    GeofenceSearchConfig,
    GeofenceSearchPlan,
    StableEntranceConfig,
    StableEntranceLayout,
    plan_geofence_search,
    stabilize_entrance_layout,
)
from dock_planner.integration import DockEntranceObservationNED, NEDPoint
from dock_planner.types import Pose2D


class UsvOnlyTask3State(str, Enum):
    IDLE = "IDLE"
    WAIT_INPUTS = "WAIT_INPUTS"
    PLAN_SEARCH = "PLAN_SEARCH"
    TRANSIT_TO_SEARCH = "TRANSIT_TO_SEARCH"
    SEARCH_PRIMARY = "SEARCH_PRIMARY"
    SEARCH_SECONDARY = "SEARCH_SECONDARY"
    VERIFY_DOCK = "VERIFY_DOCK"
    APPROACH_DOCK = "APPROACH_DOCK"
    INSPECT_BAYS = "INSPECT_BAYS"
    ENTER_BAY = "ENTER_BAY"
    HOLD_POSITION = "HOLD_POSITION"
    AIM_WINDOW = "AIM_WINDOW"
    SPRAY_WATER = "SPRAY_WATER"
    REPORT_RESULT = "REPORT_RESULT"
    EXIT_DOCK = "EXIT_DOCK"
    COMPLETE = "COMPLETE"
    PAUSED = "PAUSED"
    RETRY = "RETRY"
    ERROR = "ERROR"
    ABORTED = "ABORTED"


_TERMINAL_STATES = {
    UsvOnlyTask3State.COMPLETE,
    UsvOnlyTask3State.ERROR,
    UsvOnlyTask3State.ABORTED,
}


def _default_timeouts() -> dict[UsvOnlyTask3State, float]:
    return {
        UsvOnlyTask3State.WAIT_INPUTS: 30.0,
        UsvOnlyTask3State.PLAN_SEARCH: 5.0,
        UsvOnlyTask3State.TRANSIT_TO_SEARCH: 180.0,
        UsvOnlyTask3State.SEARCH_PRIMARY: 300.0,
        UsvOnlyTask3State.SEARCH_SECONDARY: 300.0,
        UsvOnlyTask3State.VERIFY_DOCK: 15.0,
        UsvOnlyTask3State.APPROACH_DOCK: 120.0,
        UsvOnlyTask3State.INSPECT_BAYS: 120.0,
        UsvOnlyTask3State.ENTER_BAY: 120.0,
        UsvOnlyTask3State.HOLD_POSITION: 20.0,
        UsvOnlyTask3State.AIM_WINDOW: 90.0,
        UsvOnlyTask3State.SPRAY_WATER: 90.0,
        UsvOnlyTask3State.REPORT_RESULT: 15.0,
        UsvOnlyTask3State.EXIT_DOCK: 120.0,
        UsvOnlyTask3State.PAUSED: 30.0,
        UsvOnlyTask3State.RETRY: 10.0,
    }


@dataclass(frozen=True)
class UsvOnlyTask3Config:
    required_frame_id: str = "robotx_rtk_ned"
    pose_timeout_s: float = 2.0
    max_retries_per_state: int = 2
    max_search_passes: int = 2
    bay_entry_depth_m: float = 1.0
    fallback_window_id: int = 1
    geofence: GeofenceSearchConfig = field(default_factory=GeofenceSearchConfig)
    stable_entrances: StableEntranceConfig = field(default_factory=StableEntranceConfig)
    state_timeouts_s: Mapping[UsvOnlyTask3State, float] = field(
        default_factory=_default_timeouts
    )

    def __post_init__(self) -> None:
        if not self.required_frame_id.strip():
            raise ValueError("required_frame_id must be nonempty")
        if not math.isfinite(self.pose_timeout_s) or self.pose_timeout_s <= 0:
            raise ValueError("pose_timeout_s must be positive and finite")
        if self.max_retries_per_state < 0 or self.max_search_passes < 1:
            raise ValueError("retry limits must be nonnegative and search passes positive")
        if not math.isfinite(self.bay_entry_depth_m) or self.bay_entry_depth_m < 0:
            raise ValueError("bay_entry_depth_m must be finite and nonnegative")
        if self.fallback_window_id not in (1, 2):
            raise ValueError("fallback_window_id must be 1 or 2")
        for state, timeout in self.state_timeouts_s.items():
            if not isinstance(state, UsvOnlyTask3State) or not math.isfinite(timeout) or timeout <= 0:
                raise ValueError("state timeouts must map states to positive finite seconds")


@dataclass(frozen=True)
class UsvHealth:
    pose_timestamp: float
    frame_id: str
    mavros_connected: bool = True
    manual_takeover: bool = False

    def ready(self, now: float, config: UsvOnlyTask3Config) -> tuple[bool, str]:
        if self.manual_takeover:
            return False, "MANUAL_TAKEOVER"
        if not self.mavros_connected:
            return False, "MAVROS_DISCONNECTED"
        if self.frame_id != config.required_frame_id:
            return False, "FRAME_MISMATCH"
        if not math.isfinite(self.pose_timestamp) or now - self.pose_timestamp > config.pose_timeout_s:
            return False, "RTK_POSE_STALE"
        if self.pose_timestamp > now + 0.1:
            return False, "RTK_POSE_FROM_FUTURE"
        return True, ""


class UsvOnlyTask3FlowManager:
    """Event-driven coordinator for the USV-only Task 3 flow."""

    def __init__(self, config: UsvOnlyTask3Config = UsvOnlyTask3Config()) -> None:
        self.config = config
        self.state = UsvOnlyTask3State.IDLE
        self.state_entered_at = 0.0
        self.error: str | None = None
        self.detail = ""
        self.search_plan: GeofenceSearchPlan | None = None
        self.layout: StableEntranceLayout | None = None
        self.current_pose: Pose2D | None = None
        self.health: UsvHealth | None = None
        self._line_index = 0
        self._station_index = 0
        self._search_pass = 1
        self._dock_frames: list[tuple[DockEntranceObservationNED, ...]] = []
        self._active_bay_index = 0
        self.bay_id: int | None = None
        self.window_id: int | None = None
        self._resume_state: UsvOnlyTask3State | None = None
        self._retry_state: UsvOnlyTask3State | None = None
        self._retry_counts: dict[UsvOnlyTask3State, int] = {}

    @property
    def releases_control(self) -> bool:
        return self.state in {
            UsvOnlyTask3State.IDLE, UsvOnlyTask3State.WAIT_INPUTS,
            UsvOnlyTask3State.PLAN_SEARCH,
            UsvOnlyTask3State.COMPLETE, UsvOnlyTask3State.PAUSED,
            UsvOnlyTask3State.RETRY, UsvOnlyTask3State.ERROR,
            UsvOnlyTask3State.ABORTED,
        }

    @property
    def current_goal(self) -> Pose2D | None:
        if self.search_plan is None:
            return None
        if self.state in {
            UsvOnlyTask3State.TRANSIT_TO_SEARCH,
            UsvOnlyTask3State.SEARCH_PRIMARY,
            UsvOnlyTask3State.SEARCH_SECONDARY,
        }:
            line = self.search_plan.lines[self._line_index]
            return line.stations[min(self._station_index, len(line.stations) - 1)]
        if self.layout is None:
            return None
        if self.state in {UsvOnlyTask3State.APPROACH_DOCK, UsvOnlyTask3State.INSPECT_BAYS}:
            return self.layout.bays[self._active_bay_index].observation_pose
        if self.state is UsvOnlyTask3State.ENTER_BAY:
            entrance = self.layout.bays[self._active_bay_index].entrance
            return Pose2D(
                entrance.entrance_center_north_m + math.cos(entrance.entrance_heading_rad) * self.config.bay_entry_depth_m,
                entrance.entrance_center_east_m + math.sin(entrance.entrance_heading_rad) * self.config.bay_entry_depth_m,
                entrance.entrance_heading_rad,
            )
        if self.state is UsvOnlyTask3State.EXIT_DOCK:
            return self.layout.bays[self._active_bay_index].observation_pose
        return None

    def _enter(self, state: UsvOnlyTask3State, now: float, detail: str = "") -> None:
        self.state = state
        self.state_entered_at = now
        self.detail = detail

    def start(self, now: float) -> None:
        self.__init__(self.config)
        self._enter(UsvOnlyTask3State.WAIT_INPUTS, now)

    def begin(
        self,
        corners_ned: Sequence[NEDPoint],
        current_pose: Pose2D,
        health: UsvHealth,
        *,
        now: float,
    ) -> None:
        if self.state not in {UsvOnlyTask3State.IDLE, UsvOnlyTask3State.WAIT_INPUTS}:
            raise RuntimeError("Task3 inputs are only accepted while idle or waiting")
        self.current_pose = current_pose
        self.health = health
        ready, reason = health.ready(now, self.config)
        if not ready:
            self._enter(UsvOnlyTask3State.WAIT_INPUTS, now, reason)
            return
        self._enter(UsvOnlyTask3State.PLAN_SEARCH, now)
        try:
            self.search_plan = plan_geofence_search(corners_ned, self.config.geofence)
        except (TypeError, ValueError) as exc:
            self.fail(f"INVALID_GEOFENCE: {exc}", now)
            return
        self._line_index = 0
        self._station_index = 0
        self._enter(UsvOnlyTask3State.TRANSIT_TO_SEARCH, now)

    def update_health(self, health: UsvHealth, current_pose: Pose2D, *, now: float) -> None:
        self.health = health
        self.current_pose = current_pose
        if self.state in _TERMINAL_STATES or self.state is UsvOnlyTask3State.IDLE:
            return
        ready, reason = health.ready(now, self.config)
        if not ready and self.state is not UsvOnlyTask3State.PAUSED:
            self._resume_state = self.state
            self._enter(UsvOnlyTask3State.PAUSED, now, reason)

    def resume(self, *, now: float) -> bool:
        if self.state is not UsvOnlyTask3State.PAUSED or self.health is None:
            return False
        ready, reason = self.health.ready(now, self.config)
        if not ready:
            self.detail = reason
            return False
        destination = self._resume_state or UsvOnlyTask3State.WAIT_INPUTS
        self._resume_state = None
        self._enter(destination, now, "RESUMED")
        return True

    def navigation_reached(self, *, now: float) -> None:
        if self.state is UsvOnlyTask3State.TRANSIT_TO_SEARCH:
            self._enter(UsvOnlyTask3State.SEARCH_PRIMARY, now)
            return
        if self.state in {UsvOnlyTask3State.SEARCH_PRIMARY, UsvOnlyTask3State.SEARCH_SECONDARY}:
            assert self.search_plan is not None
            line = self.search_plan.lines[self._line_index]
            if self._station_index + 1 < len(line.stations):
                self._station_index += 1
                self.state_entered_at = now
                return
            if self._line_index == 0:
                self._line_index = 1
                self._station_index = 0
                self._dock_frames.clear()
                self._enter(UsvOnlyTask3State.SEARCH_SECONDARY, now)
                return
            if self._search_pass < self.config.max_search_passes:
                self._search_pass += 1
                self._line_index = 0
                self._station_index = 0
                self._dock_frames.clear()
                self._enter(UsvOnlyTask3State.RETRY, now, "SEARCH_PASS_RETRY")
                self._retry_state = UsvOnlyTask3State.TRANSIT_TO_SEARCH
                return
            self.fail("DOCK_NOT_FOUND_AFTER_BOUNDED_SEARCH", now)
            return
        if self.state is UsvOnlyTask3State.APPROACH_DOCK:
            self._enter(UsvOnlyTask3State.INSPECT_BAYS, now)
            return
        if self.state is UsvOnlyTask3State.INSPECT_BAYS:
            return
        if self.state is UsvOnlyTask3State.ENTER_BAY:
            self._enter(UsvOnlyTask3State.HOLD_POSITION, now)
            return
        if self.state is UsvOnlyTask3State.EXIT_DOCK:
            self._enter(UsvOnlyTask3State.COMPLETE, now, "TASK3_FINISH")
            return
        raise RuntimeError(f"navigation completion is invalid in {self.state.value}")

    def accept_dock_frame(
        self,
        observations: Sequence[DockEntranceObservationNED],
        observer_pose: Pose2D,
        *,
        now: float,
    ) -> bool:
        if self.state not in {
            UsvOnlyTask3State.TRANSIT_TO_SEARCH,
            UsvOnlyTask3State.SEARCH_PRIMARY,
            UsvOnlyTask3State.SEARCH_SECONDARY,
        }:
            return False
        self._dock_frames.append(tuple(observations))
        keep = max(self.config.stable_entrances.required_frame_hits * 3, 6)
        del self._dock_frames[:-keep]
        layout = stabilize_entrance_layout(
            self._dock_frames, observer_pose, self.config.stable_entrances
        )
        if layout is None:
            return False
        self.layout = layout
        self._active_bay_index = 0
        self._enter(UsvOnlyTask3State.VERIFY_DOCK, now)
        return True

    def dock_verified(self, *, now: float) -> None:
        if self.state is not UsvOnlyTask3State.VERIFY_DOCK or self.layout is None:
            raise RuntimeError("no stable dock layout is ready for verification")
        self._enter(UsvOnlyTask3State.APPROACH_DOCK, now)

    def accept_bay_color(self, color: str, confidence: float, *, now: float) -> bool:
        if self.state is not UsvOnlyTask3State.INSPECT_BAYS or self.layout is None:
            return False
        if not math.isfinite(confidence) or confidence < 0.5:
            return False
        if color.strip().upper() == "GREEN":
            self.bay_id = self.layout.bays[self._active_bay_index].bay_id
            self._enter(UsvOnlyTask3State.ENTER_BAY, now)
            return True
        if self._active_bay_index + 1 < len(self.layout.bays):
            self._active_bay_index += 1
            self.state_entered_at = now
            return False
        self._retry_or_fail(UsvOnlyTask3State.INSPECT_BAYS, "SAFE_BAY_NOT_CONFIRMED", now)
        return False

    def hold_confirmed(self, *, now: float) -> None:
        if self.state is not UsvOnlyTask3State.HOLD_POSITION:
            raise RuntimeError("hold confirmation is only valid after entering the bay")
        self._enter(UsvOnlyTask3State.AIM_WINDOW, now)

    def aiming_ready(self, *, gimbal_ready: bool, window_id: int | None, now: float) -> bool:
        if self.state is not UsvOnlyTask3State.AIM_WINDOW or not gimbal_ready:
            return False
        self.window_id = window_id if window_id in (1, 2) else self.config.fallback_window_id
        self._enter(UsvOnlyTask3State.SPRAY_WATER, now)
        return True

    def spray_complete(self, *, now: float) -> None:
        if self.state is not UsvOnlyTask3State.SPRAY_WATER:
            raise RuntimeError("spray completion is only valid while spraying")
        self._enter(UsvOnlyTask3State.REPORT_RESULT, now)

    def result_reported(self, *, now: float) -> None:
        if self.state is not UsvOnlyTask3State.REPORT_RESULT:
            raise RuntimeError("result preparation is only valid while reporting")
        self._enter(UsvOnlyTask3State.EXIT_DOCK, now)

    def progress_report(self) -> dict[str, object]:
        """Return the unencoded progress JSON consumed by the main FSM."""

        progress = (
            "TASK3_FINISH"
            if self.state is UsvOnlyTask3State.COMPLETE
            else f"TASK3_{self.state.value}"
        )
        message: dict[str, object] = {
            "vehicle": "usv",
            "task": "TASK3",
            "progress": progress,
            "error": self.error,
        }
        if self.state is UsvOnlyTask3State.COMPLETE:
            if self.bay_id not in (1, 2, 3) or self.window_id not in (1, 2):
                raise RuntimeError("TASK3_FINISH requires valid bay_id and window_id")
            message.update({
                "bay_id": self.bay_id,
                "window_id": self.window_id,
                "report": {
                    "bay_id": self.bay_id,
                    "window_id": self.window_id,
                },
            })
        return message

    def retry(self, *, now: float) -> bool:
        if self.state is not UsvOnlyTask3State.RETRY or self._retry_state is None:
            return False
        destination = self._retry_state
        self._retry_state = None
        self._enter(destination, now, "RETRY_STARTED")
        return True

    def pause(self, reason: str, *, now: float) -> None:
        if self.state in _TERMINAL_STATES:
            return
        self._resume_state = self.state
        self._enter(UsvOnlyTask3State.PAUSED, now, reason or "PAUSED")

    def abort(self, reason: str, *, now: float) -> None:
        if self.state is UsvOnlyTask3State.COMPLETE:
            return
        self.error = reason or "ABORTED_BY_MASTER"
        self._enter(UsvOnlyTask3State.ABORTED, now, self.error)

    def fail(self, reason: str, now: float) -> None:
        self.error = reason
        self._enter(UsvOnlyTask3State.ERROR, now, reason)

    def _retry_or_fail(self, failed_state: UsvOnlyTask3State, reason: str, now: float) -> None:
        attempt = self._retry_counts.get(failed_state, 0) + 1
        self._retry_counts[failed_state] = attempt
        if attempt > self.config.max_retries_per_state:
            self.fail(f"{reason}: retry limit exhausted", now)
            return
        self._retry_state = failed_state
        self._enter(UsvOnlyTask3State.RETRY, now, reason)

    def tick(self, *, now: float) -> None:
        if self.state in _TERMINAL_STATES or self.state is UsvOnlyTask3State.IDLE:
            return
        if self.health is not None:
            ready, reason = self.health.ready(now, self.config)
            if not ready and self.state is not UsvOnlyTask3State.PAUSED:
                self._resume_state = self.state
                self._enter(UsvOnlyTask3State.PAUSED, now, reason)
                return
        timeout = self.config.state_timeouts_s.get(self.state)
        if timeout is None or now - self.state_entered_at < timeout:
            return
        if self.state is UsvOnlyTask3State.PAUSED:
            self.fail(f"{self.detail or 'PAUSED'}: recovery timeout", now)
            return
        if self.state is UsvOnlyTask3State.RETRY:
            self.fail(f"{self.detail or 'RETRY'}: retry dispatch timeout", now)
            return
        self._retry_or_fail(self.state, f"{self.state.value}_TIMEOUT", now)

    def status(self) -> dict[str, object]:
        return {
            "state": self.state.value,
            "detail": self.detail,
            "error": self.error,
            "bay_id": self.bay_id,
            "window_id": self.window_id,
            "search_pass": self._search_pass,
            "search_line": self._line_index + 1,
            "station_index": self._station_index,
            "release_control": self.releases_control,
            "task_complete": self.state is UsvOnlyTask3State.COMPLETE,
            "fsm_progress": "TASK3_FINISH" if self.state is UsvOnlyTask3State.COMPLETE else f"TASK3_{self.state.value}",
        }


__all__ = [
    "UsvHealth",
    "UsvOnlyTask3Config",
    "UsvOnlyTask3FlowManager",
    "UsvOnlyTask3State",
]
