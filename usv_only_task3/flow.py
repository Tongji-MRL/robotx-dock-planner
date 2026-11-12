"""ROS-independent USV-only RobotX Task 3 child state machine.

The class in this module coordinates existing planning, perception and control
adapters.  It never commands hardware directly.  Navigation advances only on
explicit controller completion, dock geometry advances only after the existing
multi-frame stabilizer has recovered three entrances, and terminal completion
is represented by the distinct ``FINISHED`` state.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from copy import deepcopy
import uuid
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
from dock_planner.task3_mission_policy import (
    Task3NavigationConstraint,
    plan_keep_out_detour,
    plan_keep_out_exit,
    resume_phase,
)


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
    FINISHED = "FINISHED"
    PAUSED = "PAUSED"
    RETRY = "RETRY"
    ERROR = "ERROR"
    ABORTED = "ABORTED"


_TERMINAL_STATES = {
    UsvOnlyTask3State.FINISHED,
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
    keep_out_safety_clearance_m: float = 1.0
    keep_out_detour_margin_m: float = 0.5
    keep_out_arc_spacing_m: float = 1.0
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
        if (not math.isfinite(self.keep_out_safety_clearance_m)
                or self.keep_out_safety_clearance_m < 0):
            raise ValueError("keep_out_safety_clearance_m must be finite and nonnegative")
        if not math.isfinite(self.keep_out_detour_margin_m) or self.keep_out_detour_margin_m < 0:
            raise ValueError("keep_out_detour_margin_m must be finite and nonnegative")
        if not math.isfinite(self.keep_out_arc_spacing_m) or self.keep_out_arc_spacing_m <= 0:
            raise ValueError("keep_out_arc_spacing_m must be positive and finite")
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
        self._bay_observation_ready = False
        self.bay_id: int | None = None
        self.window_id: int | None = None
        self._resume_state: UsvOnlyTask3State | None = None
        self._retry_state: UsvOnlyTask3State | None = None
        self._retry_counts: dict[UsvOnlyTask3State, int] = {}
        self.navigation_constraint = Task3NavigationConstraint()
        self.navigation_hold_reason = None
        self._navigation_hold_at = None
        self.core_checkpoint = None
        self._core_paused = False
        self._resume_elapsed = {}
        self._needs_dock_revalidation = False
        self._revalidation_at_observation = False
        self.watering_completed = False
        self._detour_waypoints: list[Pose2D] = []
        self._detour_side: str | None = None
        self._keep_out_exit_waypoint: Pose2D | None = None
        self._inside_official_zone = False
        self._inside_protected_zone = False


    @property
    def releases_control(self) -> bool:
        return bool(self.navigation_hold_reason) or self.state in {
            UsvOnlyTask3State.IDLE, UsvOnlyTask3State.WAIT_INPUTS,
            UsvOnlyTask3State.PLAN_SEARCH,
            UsvOnlyTask3State.FINISHED, UsvOnlyTask3State.PAUSED,
            UsvOnlyTask3State.RETRY, UsvOnlyTask3State.ERROR,
            UsvOnlyTask3State.ABORTED,
        }

    @property
    def dock_verification_ready(self) -> bool:
        """Whether the current stable layout can advance without new frames."""

        return (
            self.state is UsvOnlyTask3State.VERIFY_DOCK
            and self.layout is not None
            and not self.navigation_hold_reason
            and not self._needs_dock_revalidation
        )

    @property
    def current_goal(self) -> Pose2D | None:
        if self.releases_control or self.search_plan is None:
            return None
        if self._keep_out_exit_waypoint is not None:
            return self._keep_out_exit_waypoint
        if self._detour_waypoints:
            return self._detour_waypoints[0]
        return self._mission_goal

    @property
    def _mission_goal(self) -> Pose2D | None:
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
        self.state_entered_at = now - self._resume_elapsed.pop(state, 0.0)
        self.detail = detail

    def start(self, now: float) -> None:
        constraint = self.navigation_constraint
        hold_reason = self.navigation_hold_reason
        self.__init__(self.config)
        # START/RESET are not AllClear and must never remove an active zone.
        self.navigation_constraint = constraint
        self.navigation_hold_reason = hold_reason
        if hold_reason:
            self._navigation_hold_at = now
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
        self._refresh_keep_out_relation(now=now)

    def update_health(self, health: UsvHealth, current_pose: Pose2D, *, now: float) -> None:
        self.health = health
        self.current_pose = current_pose
        self._refresh_keep_out_relation(now=now)
        if self.state in _TERMINAL_STATES or self.state is UsvOnlyTask3State.IDLE:
            return
        ready, reason = health.ready(now, self.config)
        if not ready and self.state is not UsvOnlyTask3State.PAUSED:
            self.pause(reason, now=now)

    def resume(self, *, now: float) -> bool:
        if self._core_paused or self.navigation_hold_reason:
            return False
        if self.state is not UsvOnlyTask3State.PAUSED or self.health is None:
            return False
        ready, reason = self.health.ready(now, self.config)
        if not ready:
            self.detail = reason
            return False
        destination = self._resume_state or UsvOnlyTask3State.WAIT_INPUTS
        if destination is UsvOnlyTask3State.SPRAY_WATER:
            # PAUSE forces the outer adapter to stop both pump and gimbal.
            # Re-aim before a resumed spray instead of restarting the pump
            # against a potentially stale target.
            destination = UsvOnlyTask3State.AIM_WINDOW
        self._resume_state = None
        self._enter(destination, now, "RESUMED")
        return True

    def navigation_reached(self, *, now: float) -> None:
        if self.navigation_hold_reason:
            return
        if self._keep_out_exit_waypoint is not None:
            self._refresh_keep_out_relation(now=now)
            return
        if self._detour_waypoints:
            self._detour_waypoints.pop(0)
            if not self._detour_waypoints:
                self._detour_side = None
            return
        if self.state is UsvOnlyTask3State.TRANSIT_TO_SEARCH:
            self._enter(UsvOnlyTask3State.SEARCH_PRIMARY, now)
            return
        if self.state in {UsvOnlyTask3State.SEARCH_PRIMARY, UsvOnlyTask3State.SEARCH_SECONDARY}:
            assert self.search_plan is not None
            line = self.search_plan.lines[self._line_index]
            if self._station_index + 1 < len(line.stations):
                self._station_index += 1
                # Stability is meaningful only for consecutive frames captured
                # from the same observation station.  Mixing viewpoints can make
                # unrelated entrance candidates look stable.
                self._dock_frames.clear()
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
            if self._needs_dock_revalidation:
                self._revalidation_at_observation = True
                self._enter(UsvOnlyTask3State.VERIFY_DOCK, now)
            else:
                self._bay_observation_ready = True
                self._enter(UsvOnlyTask3State.INSPECT_BAYS, now)
            return
        if self.state is UsvOnlyTask3State.INSPECT_BAYS:
            # This is the completion of a move between Bay observation poses.
            self._bay_observation_ready = True
            self.state_entered_at = now
            return
        if self.state is UsvOnlyTask3State.ENTER_BAY:
            self._enter(UsvOnlyTask3State.HOLD_POSITION, now)
            return
        if self.state is UsvOnlyTask3State.EXIT_DOCK:
            self._enter(UsvOnlyTask3State.FINISHED, now, "FINISHED")
            return
        raise RuntimeError(f"navigation completion is invalid in {self.state.value}")

    def accept_dock_frame(
        self,
        observations: Sequence[DockEntranceObservationNED],
        observer_pose: Pose2D,
        *,
        now: float,
    ) -> bool:
        if self.navigation_hold_reason or self._core_paused:
            return False
        revalidating = self.state is UsvOnlyTask3State.VERIFY_DOCK and self._needs_dock_revalidation
        if not revalidating and self.state not in {
            UsvOnlyTask3State.TRANSIT_TO_SEARCH,
            UsvOnlyTask3State.SEARCH_PRIMARY,
            UsvOnlyTask3State.SEARCH_SECONDARY,
        }:
            return False
        if any(not item.valid or now - item.timestamp > self.config.pose_timeout_s
               or item.timestamp > now + 0.1 for item in observations):
            return False
        self._dock_frames.append(tuple(observations))
        keep = max(self.config.stable_entrances.required_frame_hits * 3, 6)
        del self._dock_frames[:-keep]
        layout = stabilize_entrance_layout(
            self._dock_frames, observer_pose, self.config.stable_entrances
        )
        if layout is None:
            return False
        if revalidating and self.layout is not None:
            # Preserve bay identity only if every entrance matches the saved layout.
            if len(layout.bays) != len(self.layout.bays):
                return False
            for old, new in zip(self.layout.bays, layout.bays):
                if math.hypot(old.entrance.entrance_center_north_m - new.entrance.entrance_center_north_m,
                              old.entrance.entrance_center_east_m - new.entrance.entrance_center_east_m) > self.config.stable_entrances.association_distance_m:
                    return False
                angle = old.entrance.entrance_heading_rad - new.entrance.entrance_heading_rad
                if abs(math.atan2(math.sin(angle), math.cos(angle))) > self.config.stable_entrances.association_heading_rad:
                    return False
        self.layout = layout
        if not revalidating:
            self._active_bay_index = 0
        self._needs_dock_revalidation = False
        self._enter(UsvOnlyTask3State.VERIFY_DOCK, now)
        return True

    def dock_verified(self, *, now: float) -> None:
        if self.navigation_hold_reason or self._needs_dock_revalidation:
            raise RuntimeError("fresh dock revalidation is required")
        if self.state is not UsvOnlyTask3State.VERIFY_DOCK or self.layout is None:
            raise RuntimeError("no stable dock layout is ready for verification")
        destination = (UsvOnlyTask3State.INSPECT_BAYS if self._revalidation_at_observation
                       else UsvOnlyTask3State.APPROACH_DOCK)
        if destination is UsvOnlyTask3State.INSPECT_BAYS:
            self._bay_observation_ready = True
        self._revalidation_at_observation = False
        self._enter(destination, now)

    def accept_bay_color(self, color: str, confidence: float, *, now: float) -> bool:
        if self.navigation_hold_reason:
            return False
        if self.state is not UsvOnlyTask3State.INSPECT_BAYS or self.layout is None:
            return False
        if not self._bay_observation_ready:
            return False
        if not math.isfinite(confidence) or confidence < 0.5:
            return False
        if color.strip().upper() == "GREEN":
            self.bay_id = self.layout.bays[self._active_bay_index].bay_id
            self._bay_observation_ready = False
            self._enter(UsvOnlyTask3State.ENTER_BAY, now)
            return True
        if self._active_bay_index + 1 < len(self.layout.bays):
            self._active_bay_index += 1
            self._bay_observation_ready = False
            self.state_entered_at = now
            return False
        self._retry_or_fail(UsvOnlyTask3State.INSPECT_BAYS, "SAFE_BAY_NOT_CONFIRMED", now)
        return False

    def hold_confirmed(self, *, now: float) -> None:
        if self.navigation_hold_reason:
            return False
        if self.state is not UsvOnlyTask3State.HOLD_POSITION:
            raise RuntimeError("hold confirmation is only valid after entering the bay")
        self._enter(UsvOnlyTask3State.AIM_WINDOW, now)

    def aiming_ready(self, *, gimbal_ready: bool, window_id: int | None, now: float) -> bool:
        if self.navigation_hold_reason:
            return False
        if self.state is not UsvOnlyTask3State.AIM_WINDOW or not gimbal_ready:
            return False
        self.window_id = window_id if window_id in (1, 2) else self.config.fallback_window_id
        self._enter(UsvOnlyTask3State.SPRAY_WATER, now)
        return True

    def spray_complete(self, *, now: float) -> None:
        if self.navigation_hold_reason:
            return False
        if self.state is not UsvOnlyTask3State.SPRAY_WATER:
            raise RuntimeError("spray completion is only valid while spraying")
        self.watering_completed = True
        self._enter(UsvOnlyTask3State.REPORT_RESULT, now)

    def result_reported(self, *, now: float) -> None:
        if self.navigation_hold_reason:
            return False
        if self.state is not UsvOnlyTask3State.REPORT_RESULT:
            raise RuntimeError("result preparation is only valid while reporting")
        self._enter(UsvOnlyTask3State.EXIT_DOCK, now)

    def progress_report(self) -> dict[str, object]:
        """Return only the progress envelope consumed by the main FSM.

        Docking and firefighting IDs are emitted separately on the result topic;
        progress remains a small state message throughout the task.
        """

        progress = (
            "FINISHED"
            if self.state is UsvOnlyTask3State.FINISHED
            else f"TASK3_{self.state.value}"
        )
        return {
            "vehicle": "usv",
            "task": "TASK3",
            "progress": progress,
            "error": self.error,
        }

    def retry(self, *, now: float) -> bool:
        if self.state is not UsvOnlyTask3State.RETRY or self._retry_state is None:
            return False
        destination = self._retry_state
        self._retry_state = None
        if destination is UsvOnlyTask3State.INSPECT_BAYS:
            self._active_bay_index = 0
            self._bay_observation_ready = False
        self._enter(destination, now, "RETRY_STARTED")
        return True

    def pause(self, reason: str, *, now: float) -> None:
        if self.state in _TERMINAL_STATES or self.state is UsvOnlyTask3State.PAUSED:
            return
        effective_now = self._navigation_hold_at if self._navigation_hold_at is not None else now
        self._resume_elapsed[self.state] = max(0.0, effective_now - self.state_entered_at)
        self._resume_state = self.state
        # Resume from the current pose and replan the detour; never reuse a
        # tangent waypoint computed before the interruption.
        self._detour_waypoints.clear()
        self._detour_side = None
        self._enter(UsvOnlyTask3State.PAUSED, now, reason or "PAUSED")

    def abort(self, reason: str, *, now: float) -> None:
        if self.state is UsvOnlyTask3State.FINISHED:
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
        self._retry_state = (
            UsvOnlyTask3State.AIM_WINDOW
            if failed_state is UsvOnlyTask3State.SPRAY_WATER
            else failed_state
        )
        self._enter(UsvOnlyTask3State.RETRY, now, reason)

    def tick(self, *, now: float) -> None:
        self._refresh_keep_out_relation(now=now)
        if self._core_paused or self.navigation_hold_reason:
            return
        if self.state in _TERMINAL_STATES or self.state is UsvOnlyTask3State.IDLE:
            return
        if self.health is not None:
            ready, reason = self.health.ready(now, self.config)
            if not ready and self.state is not UsvOnlyTask3State.PAUSED:
                self.pause(reason, now=now)
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

    def pause_for_core(self, *, now: float) -> dict:
        if self._core_paused:
            return deepcopy(self.core_checkpoint)
        saved = self._resume_state if self.state is UsvOnlyTask3State.PAUSED else self.state
        saved = saved or self.state
        context = {
            "correct_bay_confirmed": self.bay_id is not None,
            "watering_completed": self.watering_completed,
            "bay_id": self.bay_id, "window_id": self.window_id,
            "layout": None if self.layout is None else asdict(self.layout),
            "coarse_dock_center": None if self.layout is None else asdict(self.layout.coarse_center_ned),
            "search_pass": self._search_pass, "line_index": self._line_index,
            "station_index": self._station_index, "active_bay_index": self._active_bay_index,
            "retry_counts": {state.value: count for state, count in self._retry_counts.items()},
            "frame_id": self.config.required_frame_id,
        }
        self.core_checkpoint = {
            "checkpoint_id": str(uuid.uuid4()), "saved_step": saved.value,
            "resume_phase": resume_phase(saved.value, context).value,
            "elapsed_s": self._resume_elapsed.get(saved, max(0.0, now - self.state_entered_at)),
            "context": context, "motion_stopped_confirmed": False,
        }
        self.pause("TASK4_CORE", now=now)
        self._core_paused = self.state is UsvOnlyTask3State.PAUSED
        return deepcopy(self.core_checkpoint)

    def resume_core(self, checkpoint_id: str, *, motion_stopped: bool,
                    dock_exit_completed: bool = False, now: float) -> bool:
        if not self._core_paused or self.core_checkpoint is None:
            return False
        if checkpoint_id != self.core_checkpoint["checkpoint_id"]:
            raise ValueError("checkpoint_id does not match")
        if motion_stopped is not True:
            raise ValueError("main must confirm motion_stopped")
        if self.navigation_hold_reason or self.health is None or not self.health.ready(now, self.config)[0]:
            return False
        if self.watering_completed and dock_exit_completed is not True:
            self.detail = "WAIT_MAIN_DOCK_EXIT_CONFIRMATION"
            return False
        saved = self._resume_state or UsvOnlyTask3State.WAIT_INPUTS
        if self.watering_completed:
            destination = UsvOnlyTask3State.FINISHED
        elif self.layout is not None:
            # The boat may have moved during Task4: do not reuse old entrance
            # detections or jump straight back to spraying.
            destination = UsvOnlyTask3State.APPROACH_DOCK
            self._needs_dock_revalidation = True
            self._dock_frames.clear()
        else:
            destination = saved
        self._core_paused = False
        self._resume_state = None
        self._enter(destination, now, "CORE_RESUMED")
        return True

    def apply_navigation_constraint(self, value: Mapping, *, now: float) -> bool:
        try:
            changed = self.navigation_constraint.apply(value)
        except (TypeError, ValueError):
            self.hold_navigation("INVALID_NAVIGATION_CONSTRAINT", now=now)
            raise
        if not changed:
            return False
        # Preserve mission progress, but invalidate geometry and waypoints that
        # were computed before this official zone update.
        self._dock_frames.clear()
        self._detour_waypoints.clear()
        self._detour_side = None
        self._keep_out_exit_waypoint = None
        if self._navigation_hold_at is not None:
            self.state_entered_at += max(0.0, now - self._navigation_hold_at)
        self._navigation_hold_at = None
        self.navigation_hold_reason = None
        self._refresh_keep_out_relation(now=now)
        return True

    def _refresh_keep_out_relation(self, *, now: float) -> None:
        """Re-evaluate the active circle from every fresh RTK pose."""
        zone = self.navigation_constraint.zone
        managed_holds = {
            "WAIT_TASK3_GEOFENCE_FOR_KEEP_OUT_EXIT",
            "KEEP_OUT_NO_EXIT_INSIDE_GEOFENCE",
        }
        if zone is None:
            self._inside_official_zone = False
            self._inside_protected_zone = False
            self._keep_out_exit_waypoint = None
            if self.navigation_hold_reason in managed_holds:
                self.navigation_hold_reason = None
                self._navigation_hold_at = None
            return
        if self.current_pose is None:
            return
        distance = math.hypot(
            self.current_pose.x - zone.north,
            self.current_pose.y - zone.east,
        )
        self._inside_official_zone = distance <= zone.radius_m
        self._inside_protected_zone = distance <= zone.radius_m + zone.clearance_m
        if not self._inside_protected_zone:
            if self._keep_out_exit_waypoint is not None:
                self._keep_out_exit_waypoint = None
                self._detour_waypoints.clear()
                self._detour_side = None
            if self.navigation_hold_reason in managed_holds:
                self.navigation_hold_reason = None
                self._navigation_hold_at = None
            return

        # Once inside, previous mission/detour targets are invalid.  Generate a
        # short radial exit target from the latest pose instead.
        self._detour_waypoints.clear()
        self._detour_side = None
        if self._keep_out_exit_waypoint is not None:
            return
        if self.search_plan is None:
            self.hold_navigation("WAIT_TASK3_GEOFENCE_FOR_KEEP_OUT_EXIT", now=now)
            return
        boundary = tuple((point.north, point.east)
                         for point in self.search_plan.ordered_corners)
        try:
            self._keep_out_exit_waypoint = plan_keep_out_exit(
                self.current_pose,
                zone,
                boundary=boundary,
                exit_margin_m=self.config.keep_out_detour_margin_m,
            )
        except (TypeError, ValueError) as exc:
            self._keep_out_exit_waypoint = None
            self.hold_navigation(str(exc), now=now)
            return
        if self.navigation_hold_reason in managed_holds:
            self.navigation_hold_reason = None
            self._navigation_hold_at = None

    def hold_navigation(self, reason: str, *, now: float) -> None:
        if self._navigation_hold_at is None:
            self._navigation_hold_at = now
        self.navigation_hold_reason = reason

    def guard_current_goal(self, *, now: float) -> bool:
        if self.navigation_hold_reason:
            return False
        goal = self.current_goal
        if goal is None or self.current_pose is None:
            return False
        if self.health is None or not self.health.ready(now, self.config)[0]:
            self.pause("NAVIGATION_REQUIRES_FRESH_HEALTH", now=now)
            return False
        self._refresh_keep_out_relation(now=now)
        if self._keep_out_exit_waypoint is not None:
            return True
        if self._detour_waypoints:
            return True
        reason = self.navigation_constraint.segment_block_reason(
            (self.current_pose.x, self.current_pose.y), (goal.x, goal.y),
            self.config.required_frame_id)
        if reason == "KEEP_OUT_REPLAN_REQUIRED":
            zone = self.navigation_constraint.zone
            if zone is None or self.search_plan is None:
                self.hold_navigation("KEEP_OUT_REPLAN_UNAVAILABLE", now=now)
                return False
            boundary = tuple((point.north, point.east)
                             for point in self.search_plan.ordered_corners)
            try:
                detour = plan_keep_out_detour(
                    self.current_pose,
                    goal,
                    zone,
                    boundary=boundary,
                    detour_margin_m=self.config.keep_out_detour_margin_m,
                    arc_spacing_m=self.config.keep_out_arc_spacing_m,
                )
            except (TypeError, ValueError) as exc:
                self.hold_navigation(str(exc), now=now)
                return False
            self._detour_waypoints = list(detour.waypoints[:-1])
            self._detour_side = detour.side
            if not self._detour_waypoints:
                self.hold_navigation("KEEP_OUT_REPLAN_EMPTY", now=now)
                return False
            return True
        if reason:
            self.hold_navigation(reason, now=now)
            return False
        return True

    def status(self) -> dict[str, object]:
        return {
            "state": self.state.value,
            "resume_phase": resume_phase(
                (self._resume_state or self.state).value,
                {"correct_bay_confirmed": self.bay_id is not None,
                 "watering_completed": self.watering_completed,
                 "coarse_dock_center": self.layout}).value,
            "core_checkpoint_id": None if self.core_checkpoint is None else self.core_checkpoint["checkpoint_id"],
            "core_paused": self._core_paused,
            "retry_counts": {state.value: count for state, count in self._retry_counts.items()},
            "navigation_constraint": self.navigation_constraint.to_dict(),
            "navigation_hold_reason": self.navigation_hold_reason,
            "keep_out_detour_side": self._detour_side,
            "keep_out_detour_waypoints_remaining": len(self._detour_waypoints),
            "keep_out_inside_official_zone": self._inside_official_zone,
            "keep_out_inside_protected_zone": self._inside_protected_zone,
            "keep_out_mode": (
                "EXITING" if self._keep_out_exit_waypoint is not None
                else "DETOUR" if self._detour_waypoints
                else "ACTIVE" if self.navigation_constraint.zone is not None
                else "CLEAR"
            ),
            "detail": self.detail,
            "error": self.error,
            "bay_id": self.bay_id,
            "window_id": self.window_id,
            "search_pass": self._search_pass,
            "search_line": self._line_index + 1,
            "station_index": self._station_index,
            "bay_observation_ready": self._bay_observation_ready,
            "release_control": self.releases_control,
            "task_complete": self.state is UsvOnlyTask3State.FINISHED,
            "fsm_progress": "FINISHED" if self.state is UsvOnlyTask3State.FINISHED else f"TASK3_{self.state.value}",
        }


__all__ = [
    "UsvHealth",
    "UsvOnlyTask3Config",
    "UsvOnlyTask3FlowManager",
    "UsvOnlyTask3State",
]
