"""Small, ROS-independent Task 3 search-to-docking flow skeleton.

This module coordinates already-produced observations.  It deliberately does
not parse PointCloud2, run a color model, publish ROS messages, or talk to
PX4.  Those responsibilities remain at adapters around this flow.
"""

from enum import Enum
from dataclasses import dataclass
import math
import time
from typing import Optional

from .integration import (
    BayColorObservation,
    CoarseDockCenterObservationNED,
    CoarseDockCirclePosition,
    CoarseDockInfo,
    CoarseDockObservation,
    CoarseDockSearchConfig,
    DataFreshnessConfig,
    DockEntranceObservationNED,
    DockPerceptionResult,
    PreciseEntrancePoseNED,
    TerminalPlanInput,
    classify_coarse_dock_circle_position,
    entrance_observation_to_precise_pose,
    prepare_terminal_plan_input,
    sort_entrances_left_to_right,
)
from .dock_search import (
    CircularDockSearchConfig,
    CircularDockSearchPlan,
    plan_circular_dock_search,
)
from .bay_search import (
    BayCandidateAction,
    BayVisitConfig,
    BayVisitRecord,
    BayVisitTracker,
)
from .planner import DockPlanner
from .mission_safety import (
    BayExitPlan,
    BoundaryPathValidation,
    NavigableBoundaryNED,
    plan_non_target_bay_exit,
    validate_terminal_plan_to_boundary,
)
from .types import PlanResult, PlanStatus, Pose2D


class Task3FlowState(str, Enum):
    IDLE = "IDLE"
    APPROACH_DOCK = "APPROACH_DOCK"
    SEARCH_DOCK = "SEARCH_DOCK"
    CHECK_BAY_COLOR = "CHECK_BAY_COLOR"
    NEXT_BAY = "NEXT_BAY"
    TERMINAL_DOCKING = "TERMINAL_DOCKING"
    DOCKED = "DOCKED"
    WATERING = "WATERING"
    FINISHED = "FINISHED"
    ERROR = "ERROR"


@dataclass(frozen=True)
class Task3RetryConfig:
    """Bounds for transient Task3 failures before a terminal error is raised.

    ``max_attempts`` includes the first failed attempt.  For example, with
    the default value of three, the first two failures keep the flow in its
    current step and the third failure becomes ``ERROR``.  The time bound is
    measured per retry operation, so a fresh successful observation resets it.
    """

    max_attempts: int = 3
    operation_timeout_s: float = 30.0

    def __post_init__(self) -> None:
        if isinstance(self.max_attempts, bool) or not isinstance(self.max_attempts, int):
            raise ValueError("max_attempts must be a positive integer")
        if self.max_attempts <= 0:
            raise ValueError("max_attempts must be a positive integer")
        if (
            isinstance(self.operation_timeout_s, bool)
            or not isinstance(self.operation_timeout_s, (int, float))
            or not math.isfinite(self.operation_timeout_s)
            or self.operation_timeout_s <= 0
        ):
            raise ValueError("operation_timeout_s must be positive and finite")


class Task3FlowManager:
    """Coordinate the confirmed Task 3 flow using injected observations."""

    def __init__(
        self,
        planner: DockPlanner,
        *,
        target_color: str,
        bay_width: float,
        bay_depth: float,
        freshness: DataFreshnessConfig = DataFreshnessConfig(),
        planning_boundary: Optional[NavigableBoundaryNED] = None,
        min_boundary_clearance_m: float = 0.0,
        bay_visit_config: BayVisitConfig = BayVisitConfig(),
        min_entrance_confidence: float = 0.20,
        min_bay_color_confidence: float = 0.50,
        retry_config: Task3RetryConfig = Task3RetryConfig(),
    ) -> None:
        if not isinstance(planner, DockPlanner):
            raise TypeError("planner must be a DockPlanner")
        if not isinstance(target_color, str) or not target_color.strip():
            raise ValueError("target_color must be a nonempty string")
        self._planner = planner
        self._target_color = target_color.strip().upper()
        self._bay_width = bay_width
        self._bay_depth = bay_depth
        if not isinstance(freshness, DataFreshnessConfig):
            raise TypeError("freshness must be a DataFreshnessConfig")
        if planning_boundary is not None and not isinstance(
            planning_boundary, NavigableBoundaryNED
        ):
            raise TypeError("planning_boundary must be None or NavigableBoundaryNED")
        if (
            isinstance(min_boundary_clearance_m, bool)
            or not isinstance(min_boundary_clearance_m, (int, float))
            or not math.isfinite(min_boundary_clearance_m)
            or min_boundary_clearance_m < 0
        ):
            raise ValueError("min_boundary_clearance_m must be finite and nonnegative")
        if not isinstance(bay_visit_config, BayVisitConfig):
            raise TypeError("bay_visit_config must be a BayVisitConfig")
        if not isinstance(retry_config, Task3RetryConfig):
            raise TypeError("retry_config must be a Task3RetryConfig")
        for name, value in (
            ("min_entrance_confidence", min_entrance_confidence),
            ("min_bay_color_confidence", min_bay_color_confidence),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or not 0 <= value <= 1
            ):
                raise ValueError(f"{name} must be finite and in [0, 1]")
        self._freshness = freshness
        self._planning_boundary = planning_boundary
        self._min_boundary_clearance_m = float(min_boundary_clearance_m)
        self._bay_tracker = BayVisitTracker(bay_visit_config)
        self._min_entrance_confidence = float(min_entrance_confidence)
        self._min_bay_color_confidence = float(min_bay_color_confidence)
        self._state = Task3FlowState.IDLE
        self._coarse_dock: Optional[CoarseDockObservation] = None
        self._search_plan: Optional[CircularDockSearchPlan] = None
        self._coarse_circle_position: Optional[CoarseDockCirclePosition] = None
        self._search_waypoint_index = 0
        self._entrance_observation: Optional[DockEntranceObservationNED] = None
        self._precise_entrance: Optional[PreciseEntrancePoseNED] = None
        self._current_pose: Optional[Pose2D] = None
        self._terminal_input: Optional[TerminalPlanInput] = None
        self._plan: Optional[PlanResult] = None
        self._terminal_boundary_validation: Optional[BoundaryPathValidation] = None
        self._non_target_exit_plan: Optional[BayExitPlan] = None
        self._error: Optional[str] = None
        self._active_bay_index: Optional[int] = None
        self._retry_config = retry_config
        self._retry_operation: Optional[str] = None
        self._retry_attempts = 0
        self._retry_started_at: Optional[float] = None
        self._retry_last_at: Optional[float] = None
        self._retry_reason: Optional[str] = None

    @property
    def state(self) -> Task3FlowState:
        return self._state

    @property
    def plan(self) -> Optional[PlanResult]:
        return self._plan

    @property
    def precise_entrance(self) -> Optional[PreciseEntrancePoseNED]:
        return self._precise_entrance

    @property
    def terminal_input(self) -> Optional[TerminalPlanInput]:
        return self._terminal_input

    @property
    def terminal_boundary_validation(self) -> Optional[BoundaryPathValidation]:
        return self._terminal_boundary_validation

    @property
    def non_target_exit_plan(self) -> Optional[BayExitPlan]:
        """Exit plan prepared after a bay color mismatch, if any."""
        return self._non_target_exit_plan

    @property
    def error(self) -> Optional[str]:
        return self._error

    @property
    def retry_status(self) -> dict[str, object] | None:
        """Return transient retry information for communication adapters."""

        if self._retry_operation is None or self._retry_started_at is None:
            return None
        return {
            "operation": self._retry_operation,
            "attempt": self._retry_attempts,
            "max_attempts": self._retry_config.max_attempts,
            "elapsed_s": max(
                0.0,
                (self._retry_last_at or self._retry_started_at) - self._retry_started_at,
            ),
            "timeout_s": self._retry_config.operation_timeout_s,
            "reason": self._retry_reason or "",
        }

    @property
    def bay_visit_records(self) -> tuple[BayVisitRecord, ...]:
        return self._bay_tracker.records

    @property
    def search_plan(self) -> Optional[CircularDockSearchPlan]:
        return self._search_plan

    @property
    def coarse_circle_position(self) -> Optional[CoarseDockCirclePosition]:
        return self._coarse_circle_position

    @property
    def approach_goal(self) -> Optional[Pose2D]:
        return self._search_plan.staging_pose if self._search_plan is not None else None

    @property
    def current_search_goal(self) -> Optional[Pose2D]:
        if self._search_plan is None or self._state is not Task3FlowState.SEARCH_DOCK:
            return None
        if self._search_waypoint_index >= len(self._search_plan.arc_waypoints):
            return None
        return self._search_plan.arc_waypoints[self._search_waypoint_index]

    def begin(self, coarse_dock: CoarseDockObservation, *, now: Optional[float] = None) -> None:
        if not isinstance(coarse_dock, (CoarseDockInfo, CoarseDockCenterObservationNED)) or not coarse_dock.valid:
            self.fail("a valid coarse dock observation is required")
            return
        if now is not None and not coarse_dock.is_fresh_at(
            now, self._freshness.coarse_dock_timeout_s
        ):
            self.fail("coarse dock observation is stale")
            return
        self._coarse_dock = coarse_dock
        self._entrance_observation = None
        self._precise_entrance = None
        self._terminal_input = None
        self._plan = None
        self._terminal_boundary_validation = None
        self._non_target_exit_plan = None
        self._search_plan = None
        self._coarse_circle_position = None
        self._search_waypoint_index = 0
        self._error = None
        self._active_bay_index = None
        self._clear_retry()
        self._bay_tracker.reset()
        self._state = Task3FlowState.APPROACH_DOCK

    def begin_center_search(
        self,
        coarse_dock: CoarseDockCenterObservationNED,
        current_pose: Pose2D,
        *,
        approach_config: CoarseDockSearchConfig = CoarseDockSearchConfig(),
        arc_config: CircularDockSearchConfig = CircularDockSearchConfig(),
        now: Optional[float] = None,
    ) -> Optional[CircularDockSearchPlan]:
        """Start a center-only approach and precompute the bounded search arc."""

        self.begin(coarse_dock, now=now)
        if self._state is Task3FlowState.ERROR:
            return None
        try:
            self._coarse_circle_position = classify_coarse_dock_circle_position(
                current_pose,
                coarse_dock,
                approach_config,
            )
            if self._coarse_circle_position is CoarseDockCirclePosition.CENTER_UNDEFINED:
                raise ValueError(
                    "current pose is too close to the coarse center; "
                    "local perception must select an escape direction"
                )
            self._search_plan = plan_circular_dock_search(
                current_pose,
                coarse_dock,
                approach_config,
                arc_config,
            )
        except (TypeError, ValueError) as exc:
            self.fail(str(exc))
            return None
        if self._coarse_circle_position is CoarseDockCirclePosition.ON_CIRCLE:
            self._state = Task3FlowState.SEARCH_DOCK
        return self._search_plan

    def approach_reached(self) -> None:
        self._require(Task3FlowState.APPROACH_DOCK)
        self._state = Task3FlowState.SEARCH_DOCK

    def search_waypoint_reached(self) -> Optional[Pose2D]:
        """Advance one arc goal; fail safely if the configured arc is exhausted."""

        self._require(Task3FlowState.SEARCH_DOCK)
        if self._search_plan is None:
            raise RuntimeError("no circular search plan is active")
        self._search_waypoint_index += 1
        if self._search_waypoint_index >= len(self._search_plan.arc_waypoints):
            reason = "configured search arc exhausted without a dock entrance"
            if self._retry_or_fail("search_arc", reason):
                self._search_waypoint_index = 0
                return self._search_plan.arc_waypoints[0]
            return None
        return self._search_plan.arc_waypoints[self._search_waypoint_index]

    def accept_entrance_observation(
        self,
        observation: DockEntranceObservationNED,
        current_pose: Pose2D,
        *,
        now: Optional[float] = None,
    ) -> None:
        self._require(Task3FlowState.SEARCH_DOCK)
        if not isinstance(observation, DockEntranceObservationNED) or not observation.valid:
            self._retry_or_fail(
                "entrance_observation", "a valid close-range entrance observation is required"
            )
            return
        if not isinstance(current_pose, Pose2D):
            self.fail("current_pose must be Pose2D")
            return
        if observation.dock_id not in (None, self._coarse_dock.dock_id if self._coarse_dock else None):
            self._retry_or_fail("entrance_observation", "entrance observation dock_id does not match coarse dock")
            return
        if now is not None and not observation.is_fresh_at(
            now, self._freshness.perception_timeout_s
        ):
            self._retry_or_fail("entrance_observation", "entrance observation is stale", now=now)
            return
        if observation.confidence < self._min_entrance_confidence:
            self._retry_or_fail("entrance_observation", "entrance confidence is below threshold", now=now)
            return
        self._clear_retry("entrance_observation")
        decision = self._bay_tracker.observe(observation)
        if decision.action is BayCandidateAction.SEARCH_EXHAUSTED:
            self.fail("all expected dock bays have been exhausted")
            return
        if decision.action is BayCandidateAction.SKIP:
            return
        self._active_bay_index = decision.bay_index
        self._entrance_observation = observation
        self._current_pose = current_pose
        self._state = Task3FlowState.CHECK_BAY_COLOR

    def accept_perception_result(
        self,
        result: DockPerceptionResult,
        current_pose: Pose2D,
        *,
        now: Optional[float] = None,
    ) -> bool:
        """Inspect valid entrances from dock-left to dock-right.

        A valid result with no usable candidates leaves the manager in
        ``SEARCH_DOCK`` so the caller can continue local search.  A stale or
        malformed result enters ``ERROR`` because continuing on old geometry
        would be unsafe.
        """

        self._require(Task3FlowState.SEARCH_DOCK)
        if not isinstance(result, DockPerceptionResult):
            self.fail("a DockPerceptionResult is required")
            return False
        if result.dock_id != (self._coarse_dock.dock_id if self._coarse_dock else None):
            self._retry_or_fail("dock_perception", "perception result dock_id does not match coarse dock")
            return False
        if now is not None and not result.is_fresh_at(
            now, self._freshness.perception_timeout_s
        ):
            self._retry_or_fail("dock_perception", "dock perception result is stale", now=now)
            return False
        candidates = [
            observation for observation in result.entrance_observations
            if observation.valid
            and observation.confidence >= self._min_entrance_confidence
            and observation.dock_id in (None, result.dock_id)
            and (now is None or observation.is_fresh_at(now, self._freshness.perception_timeout_s))
        ]
        if not candidates:
            self._retry_or_fail("dock_perception", "no usable entrance candidate", now=now)
            return False
        self._clear_retry("dock_perception")
        for candidate in sort_entrances_left_to_right(candidates):
            self.accept_entrance_observation(candidate, current_pose, now=now)
            if self._state is Task3FlowState.CHECK_BAY_COLOR:
                return True
            if self._state is Task3FlowState.ERROR:
                return False
        return False

    def accept_color_observation(
        self,
        observation: BayColorObservation,
        *,
        now: Optional[float] = None,
    ) -> Optional[PlanResult]:
        self._require(Task3FlowState.CHECK_BAY_COLOR)
        if not isinstance(observation, BayColorObservation) or not observation.valid:
            self._retry_or_fail("bay_color", "a valid bay color observation is required", now=now)
            return None
        if observation.confidence < self._min_bay_color_confidence:
            self._retry_or_fail("bay_color", "bay color confidence is below threshold", now=now)
            return None
        if now is not None and not observation.is_fresh_at(
            now, self._freshness.perception_timeout_s
        ):
            self._retry_or_fail("bay_color", "color observation is stale", now=now)
            return None
        if now is not None and self._entrance_observation is not None and not self._entrance_observation.is_fresh_at(
            now, self._freshness.perception_timeout_s
        ):
            self._retry_or_fail("bay_color", "entrance observation expired before color decision", now=now)
            return None
        self._clear_retry("bay_color")
        if observation.color.strip().upper() != self._target_color:
            if self._active_bay_index is None:
                self.fail("selected entrance has no active bay record")
                return None
            decision = self._bay_tracker.mark_color_rejected(self._active_bay_index)
            if decision.action is BayCandidateAction.SEARCH_EXHAUSTED:
                self.fail("all expected dock bays have been rejected or failed")
                return None
            self._precise_entrance = entrance_observation_to_precise_pose(
                self._entrance_observation,
                dock_id=self._coarse_dock.dock_id if self._coarse_dock else None,
            )
            self._non_target_exit_plan = None
            self._state = Task3FlowState.NEXT_BAY
            return None
        if self._coarse_dock is None or self._entrance_observation is None or self._current_pose is None:
            self.fail("selected entrance is missing flow context")
            return None
        self._precise_entrance = entrance_observation_to_precise_pose(
            self._entrance_observation,
            dock_id=self._coarse_dock.dock_id,
        )
        self._terminal_input = prepare_terminal_plan_input(
            self._coarse_dock,
            self._precise_entrance,
            self._bay_width,
            self._bay_depth,
        )
        self._plan = self._planner.plan(
            self._current_pose,
            self._terminal_input.dock,
            obstacles=self._terminal_input.obstacles,
        )
        if self._plan.status != PlanStatus.SUCCESS:
            reason = f"terminal plan failed: {self._plan.reason}"
            if self._retry_or_fail("terminal_plan", reason, now=now):
                return self._plan
            if self._active_bay_index is not None:
                self._bay_tracker.mark_terminal_failure(self._active_bay_index)
            return self._plan
        if self._planning_boundary is not None:
            self._terminal_boundary_validation = validate_terminal_plan_to_boundary(
                self._plan,
                self._planning_boundary,
                min_boundary_clearance_m=self._min_boundary_clearance_m,
            )
            if not self._terminal_boundary_validation.valid:
                if self._active_bay_index is not None:
                    self._bay_tracker.mark_terminal_failure(self._active_bay_index)
                self.fail(
                    "terminal plan leaves the navigable boundary: "
                    f"{self._terminal_boundary_validation.reason}"
                )
                return self._plan
        self._state = Task3FlowState.TERMINAL_DOCKING
        return self._plan

    def next_bay(self) -> None:
        """Resume search after the caller has completed the exit plan."""
        self._require(Task3FlowState.NEXT_BAY)
        self._entrance_observation = None
        self._current_pose = None
        self._active_bay_index = None
        self._non_target_exit_plan = None
        self._state = Task3FlowState.SEARCH_DOCK

    def prepare_non_target_bay_exit(
        self,
        *,
        exit_distance_m: float = 3.0,
        boundary: Optional[NavigableBoundaryNED] = None,
        min_boundary_clearance_m: float = 0.0,
    ) -> BayExitPlan:
        """Create the exit path that must be executed before ``next_bay``."""

        self._require(Task3FlowState.NEXT_BAY)
        if self._precise_entrance is None or self._current_pose is None:
            raise RuntimeError("rejected bay has no entrance/current pose context")
        self._non_target_exit_plan = plan_non_target_bay_exit(
            self._precise_entrance,
            self._current_pose,
            exit_distance_m=exit_distance_m,
            boundary=boundary,
            min_boundary_clearance_m=min_boundary_clearance_m,
        )
        return self._non_target_exit_plan

    def docking_complete(self) -> None:
        self._require(Task3FlowState.TERMINAL_DOCKING)
        if self._active_bay_index is not None:
            self._bay_tracker.mark_completed(self._active_bay_index)
        self._state = Task3FlowState.DOCKED

    def terminal_docking_failed(self, reason: str) -> None:
        """Reject the active bay after an execution failure and resume search."""

        self._require(Task3FlowState.TERMINAL_DOCKING)
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("reason must be a nonempty string")
        if self._active_bay_index is None:
            self.fail("terminal failure has no active bay record")
            return
        decision = self._bay_tracker.mark_terminal_failure(self._active_bay_index)
        self._entrance_observation = None
        self._precise_entrance = None
        self._terminal_input = None
        self._plan = None
        self._terminal_boundary_validation = None
        self._active_bay_index = None
        if decision.action is BayCandidateAction.SEARCH_EXHAUSTED:
            self.fail(f"all expected dock bays have been exhausted: {reason.strip()}")
            return
        self._state = Task3FlowState.SEARCH_DOCK

    def docked_confirmed(self) -> None:
        self._require(Task3FlowState.DOCKED)
        self._state = Task3FlowState.WATERING

    def watering_complete(self) -> None:
        self._require(Task3FlowState.WATERING)
        self._state = Task3FlowState.FINISHED

    def fail(self, reason: str) -> None:
        self._error = reason
        self._clear_retry()
        self._state = Task3FlowState.ERROR

    def _retry_or_fail(self, operation: str, reason: str, *, now: Optional[float] = None) -> bool:
        """Record one transient failure; return True while another retry is allowed."""

        if not isinstance(operation, str) or not operation.strip():
            raise ValueError("operation must be a nonempty string")
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("reason must be a nonempty string")
        timestamp = time.time() if now is None else float(now)
        if not math.isfinite(timestamp):
            raise ValueError("now must be finite")
        if self._retry_operation != operation or self._retry_started_at is None:
            self._retry_operation = operation
            self._retry_attempts = 0
            self._retry_started_at = timestamp
        self._retry_attempts += 1
        self._retry_last_at = timestamp
        self._retry_reason = reason.strip()
        elapsed = timestamp - self._retry_started_at
        exhausted = (
            self._retry_attempts >= self._retry_config.max_attempts
            or elapsed >= self._retry_config.operation_timeout_s
        )
        if exhausted:
            self.fail(self._retry_reason)
            return False
        return True

    def _clear_retry(self, operation: Optional[str] = None) -> None:
        if operation is not None and self._retry_operation != operation:
            return
        self._retry_operation = None
        self._retry_attempts = 0
        self._retry_started_at = None
        self._retry_last_at = None
        self._retry_reason = None

    def _require(self, expected: Task3FlowState) -> None:
        if self._state != expected:
            raise RuntimeError(f"expected state {expected.value}, got {self._state.value}")


__all__ = ["Task3FlowManager", "Task3FlowState"]
