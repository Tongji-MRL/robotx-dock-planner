"""Pure mission supervision for boundary-safe search and retry recovery.

This module produces geometric goals and decisions only.  It does not command
the USV, avoid obstacles, or assume that a waypoint controller follows a
straight line between goals.
"""

from dataclasses import dataclass, replace
from enum import Enum
import math
from numbers import Real
from typing import Optional, Sequence, Tuple

from .dock_search import (
    CircularDockSearchConfig,
    CircularDockSearchPlan,
    plan_circular_dock_search,
)
from .integration import (
    CoarseDockCenterObservationNED,
    CoarseDockSearchConfig,
    NEDPoint,
    PreciseEntrancePoseNED,
)
from .types import Pose2D


def _finite(value: object) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def _point_segment_distance(
    x: float,
    y: float,
    start: NEDPoint,
    end: NEDPoint,
) -> float:
    dx = end.north - start.north
    dy = end.east - start.east
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.hypot(x - start.north, y - start.east)
    fraction = ((x - start.north) * dx + (y - start.east) * dy) / length_squared
    fraction = min(1.0, max(0.0, fraction))
    nearest_x = start.north + fraction * dx
    nearest_y = start.east + fraction * dy
    return math.hypot(x - nearest_x, y - nearest_y)


@dataclass(frozen=True)
class NavigableBoundaryNED:
    """A simple polygon describing water that the mission may use."""

    vertices: Tuple[NEDPoint, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.vertices, (tuple, list)) or len(self.vertices) < 3:
            raise ValueError("boundary must contain at least three vertices")
        vertices = tuple(self.vertices)
        if not all(isinstance(point, NEDPoint) for point in vertices):
            raise ValueError("boundary vertices must be NEDPoint values")
        area_twice = sum(
            start.north * end.east - end.north * start.east
            for start, end in zip(vertices, vertices[1:] + vertices[:1])
        )
        if abs(area_twice) <= 1e-9:
            raise ValueError("boundary polygon must have nonzero area")
        object.__setattr__(self, "vertices", vertices)

    def contains(self, north: float, east: float) -> bool:
        if not _finite(north) or not _finite(east):
            raise ValueError("point coordinates must be finite")
        inside = False
        vertices = self.vertices
        previous = vertices[-1]
        for current in vertices:
            crosses = ((current.east > east) != (previous.east > east))
            if crosses:
                boundary_north = (
                    (previous.north - current.north)
                    * (east - current.east)
                    / (previous.east - current.east)
                    + current.north
                )
                if north < boundary_north:
                    inside = not inside
            previous = current
        return inside

    def clearance_m(self, north: float, east: float) -> float:
        """Return distance to the closest polygon edge."""

        if not _finite(north) or not _finite(east):
            raise ValueError("point coordinates must be finite")
        return min(
            _point_segment_distance(north, east, start, end)
            for start, end in zip(
                self.vertices,
                self.vertices[1:] + self.vertices[:1],
            )
        )

    def contains_with_clearance(
        self,
        north: float,
        east: float,
        clearance_m: float,
    ) -> bool:
        if not _finite(clearance_m) or clearance_m < 0:
            raise ValueError("clearance_m must be finite and nonnegative")
        return self.contains(north, east) and self.clearance_m(north, east) >= clearance_m


@dataclass(frozen=True)
class SearchBoundaryConfig:
    min_boundary_clearance_m: float = 1.0
    min_safe_arc_length_m: float = 4.0
    approach_sample_spacing_m: float = 0.5

    def __post_init__(self) -> None:
        if not _finite(self.min_boundary_clearance_m) or self.min_boundary_clearance_m < 0:
            raise ValueError("min_boundary_clearance_m must be finite and nonnegative")
        if not _finite(self.min_safe_arc_length_m) or self.min_safe_arc_length_m <= 0:
            raise ValueError("min_safe_arc_length_m must be positive and finite")
        if not _finite(self.approach_sample_spacing_m) or self.approach_sample_spacing_m <= 0:
            raise ValueError("approach_sample_spacing_m must be positive and finite")


@dataclass(frozen=True)
class AdaptiveSearchRadiusConfig:
    """Policy for cautiously reducing only the stand-off part of the radius.

    Dock size, vessel envelope, dimension margin and coarse-position
    uncertainty are never reduced.  The caller must set
    ``min_stand_off_distance_m`` from the real remaining water-clearance
    requirement instead of letting the planner guess a safe value.
    """

    min_stand_off_distance_m: float
    radius_step_m: float = 0.5
    max_radius_reductions: int = 4

    def __post_init__(self) -> None:
        if (
            not _finite(self.min_stand_off_distance_m)
            or self.min_stand_off_distance_m < 0
        ):
            raise ValueError(
                "min_stand_off_distance_m must be finite and nonnegative"
            )
        if not _finite(self.radius_step_m) or self.radius_step_m <= 0:
            raise ValueError("radius_step_m must be positive and finite")
        if (
            isinstance(self.max_radius_reductions, bool)
            or not isinstance(self.max_radius_reductions, int)
            or self.max_radius_reductions < 0
        ):
            raise ValueError("max_radius_reductions must be a nonnegative integer")


@dataclass(frozen=True)
class BoundaryCheckedSearchPlan:
    plan: Optional[CircularDockSearchPlan]
    valid: bool
    full_requested_arc_safe: bool
    rejected_waypoint_count: int
    safe_arc_length_m: float
    minimum_clearance_m: Optional[float]
    reason: str


@dataclass(frozen=True)
class AdaptiveBoundarySearchPlan:
    """Boundary result plus the radii considered before making the decision."""

    selected: BoundaryCheckedSearchPlan
    attempted_radii_m: Tuple[float, ...]
    radius_reductions: int
    reason: str

    @property
    def valid(self) -> bool:
        return self.selected.valid

    @property
    def plan(self) -> Optional[CircularDockSearchPlan]:
        return self.selected.plan


def constrain_search_plan_to_boundary(
    plan: CircularDockSearchPlan,
    boundary: NavigableBoundaryNED,
    config: SearchBoundaryConfig = SearchBoundaryConfig(),
) -> BoundaryCheckedSearchPlan:
    """Keep the safe prefix of an arc, starting from its staging pose.

    The function never skips an unsafe section and resumes later because that
    would create an unchecked transition across shore or another excluded
    part of the boundary.
    """

    if not isinstance(plan, CircularDockSearchPlan):
        raise TypeError("plan must be a CircularDockSearchPlan")
    if not isinstance(boundary, NavigableBoundaryNED):
        raise TypeError("boundary must be a NavigableBoundaryNED")
    if not isinstance(config, SearchBoundaryConfig):
        raise TypeError("config must be a SearchBoundaryConfig")

    staging = plan.staging_pose
    if not boundary.contains_with_clearance(
        staging.x, staging.y, config.min_boundary_clearance_m
    ):
        return BoundaryCheckedSearchPlan(
            plan=None,
            valid=False,
            full_requested_arc_safe=False,
            rejected_waypoint_count=len(plan.arc_waypoints),
            safe_arc_length_m=0.0,
            minimum_clearance_m=boundary.clearance_m(staging.x, staging.y),
            reason="staging pose is outside the navigable boundary or too close to shore",
        )

    safe_waypoints = []
    clearances = [boundary.clearance_m(staging.x, staging.y)]
    previous = staging
    safe_length = 0.0
    for waypoint in plan.arc_waypoints:
        if not boundary.contains_with_clearance(
            waypoint.x, waypoint.y, config.min_boundary_clearance_m
        ):
            break
        safe_length += math.hypot(waypoint.x - previous.x, waypoint.y - previous.y)
        safe_waypoints.append(waypoint)
        clearances.append(boundary.clearance_m(waypoint.x, waypoint.y))
        previous = waypoint

    rejected = len(plan.arc_waypoints) - len(safe_waypoints)
    if safe_length < config.min_safe_arc_length_m or not safe_waypoints:
        return BoundaryCheckedSearchPlan(
            plan=None,
            valid=False,
            full_requested_arc_safe=False,
            rejected_waypoint_count=rejected,
            safe_arc_length_m=safe_length,
            minimum_clearance_m=min(clearances),
            reason="no sufficiently long continuous search arc is safely navigable",
        )

    fraction = len(safe_waypoints) / len(plan.arc_waypoints)
    safe_plan = CircularDockSearchPlan(
        dock_id=plan.dock_id,
        center_ned=plan.center_ned,
        radius_m=plan.radius_m,
        staging_pose=plan.staging_pose,
        arc_waypoints=tuple(safe_waypoints),
        clockwise=plan.clockwise,
        arc_span_rad=plan.arc_span_rad * fraction,
    )
    return BoundaryCheckedSearchPlan(
        plan=safe_plan,
        valid=True,
        full_requested_arc_safe=rejected == 0,
        rejected_waypoint_count=rejected,
        safe_arc_length_m=safe_length,
        minimum_clearance_m=min(clearances),
        reason="full requested arc is safe" if rejected == 0 else "unsafe tail removed",
    )


def plan_boundary_safe_circular_search(
    current_pose: Pose2D,
    coarse_dock: CoarseDockCenterObservationNED,
    boundary: NavigableBoundaryNED,
    *,
    approach_config: CoarseDockSearchConfig = CoarseDockSearchConfig(),
    arc_config: CircularDockSearchConfig = CircularDockSearchConfig(),
    boundary_config: SearchBoundaryConfig = SearchBoundaryConfig(),
) -> BoundaryCheckedSearchPlan:
    """Try both arc directions and return the longer safe continuous result.

    The straight transition from the current pose to staging is sampled as
    well.  This matters when the USV starts inside the search circle and must
    travel outward before beginning the arc.
    """

    if not isinstance(current_pose, Pose2D):
        raise TypeError("current_pose must be a Pose2D")
    if not isinstance(coarse_dock, CoarseDockCenterObservationNED) or not coarse_dock.valid:
        raise ValueError("a valid center-only coarse dock observation is required")
    if not isinstance(boundary, NavigableBoundaryNED):
        raise TypeError("boundary must be a NavigableBoundaryNED")
    if not isinstance(approach_config, CoarseDockSearchConfig):
        raise TypeError("approach_config must be a CoarseDockSearchConfig")
    if not isinstance(arc_config, CircularDockSearchConfig):
        raise TypeError("arc_config must be a CircularDockSearchConfig")
    if not isinstance(boundary_config, SearchBoundaryConfig):
        raise TypeError("boundary_config must be a SearchBoundaryConfig")

    center_distance = math.hypot(
        current_pose.x - coarse_dock.center_ned.north,
        current_pose.y - coarse_dock.center_ned.east,
    )
    if center_distance <= 1e-6:
        return BoundaryCheckedSearchPlan(
            plan=None,
            valid=False,
            full_requested_arc_safe=False,
            rejected_waypoint_count=0,
            safe_arc_length_m=0.0,
            minimum_clearance_m=boundary.clearance_m(current_pose.x, current_pose.y),
            reason="current pose coincides with the coarse center; escape direction is undefined",
        )

    candidates = []
    for clockwise in (arc_config.clockwise, not arc_config.clockwise):
        raw_plan = plan_circular_dock_search(
            current_pose,
            coarse_dock,
            approach_config,
            replace(arc_config, clockwise=clockwise),
        )
        constrained = constrain_search_plan_to_boundary(raw_plan, boundary, boundary_config)
        if constrained.valid:
            dx = raw_plan.staging_pose.x - current_pose.x
            dy = raw_plan.staging_pose.y - current_pose.y
            path_length = math.hypot(dx, dy)
            sample_count = max(
                1,
                int(math.ceil(path_length / boundary_config.approach_sample_spacing_m)),
            )
            samples = tuple(
                (
                    current_pose.x + dx * index / sample_count,
                    current_pose.y + dy * index / sample_count,
                )
                for index in range(sample_count + 1)
            )
            if not all(
                boundary.contains_with_clearance(
                    north,
                    east,
                    boundary_config.min_boundary_clearance_m,
                )
                for north, east in samples
            ):
                constrained = BoundaryCheckedSearchPlan(
                    plan=None,
                    valid=False,
                    full_requested_arc_safe=False,
                    rejected_waypoint_count=len(raw_plan.arc_waypoints),
                    safe_arc_length_m=0.0,
                    minimum_clearance_m=min(
                        boundary.clearance_m(north, east) for north, east in samples
                    ),
                    reason=(
                        "current-to-staging transition leaves the navigable "
                        "boundary or passes too close to shore"
                    ),
                )
        candidates.append(constrained)
    valid = [candidate for candidate in candidates if candidate.valid]
    if valid:
        return max(valid, key=lambda candidate: candidate.safe_arc_length_m)
    return max(candidates, key=lambda candidate: candidate.safe_arc_length_m)


def plan_adaptive_boundary_safe_circular_search(
    current_pose: Pose2D,
    coarse_dock: CoarseDockCenterObservationNED,
    boundary: NavigableBoundaryNED,
    *,
    radius_config: AdaptiveSearchRadiusConfig,
    approach_config: CoarseDockSearchConfig = CoarseDockSearchConfig(),
    arc_config: CircularDockSearchConfig = CircularDockSearchConfig(),
    boundary_config: SearchBoundaryConfig = SearchBoundaryConfig(),
) -> AdaptiveBoundarySearchPlan:
    """Try the largest safe radius first, then cautiously reduce stand-off.

    A smaller radius is considered only when neither direction at the current
    radius contains the configured minimum continuous safe arc.  The dock
    envelope and coarse-position uncertainty remain unchanged at every step.
    """

    if not isinstance(radius_config, AdaptiveSearchRadiusConfig):
        raise TypeError("radius_config must be an AdaptiveSearchRadiusConfig")
    if not isinstance(approach_config, CoarseDockSearchConfig):
        raise TypeError("approach_config must be a CoarseDockSearchConfig")
    if radius_config.min_stand_off_distance_m > approach_config.stand_off_distance_m:
        raise ValueError(
            "min_stand_off_distance_m cannot exceed the initial stand-off distance"
        )

    attempted_radii = []
    failed_candidates = []
    stand_off = approach_config.stand_off_distance_m
    for reduction in range(radius_config.max_radius_reductions + 1):
        candidate_approach = replace(
            approach_config,
            stand_off_distance_m=stand_off,
        )
        attempted_radii.append(candidate_approach.search_radius_m)
        candidate = plan_boundary_safe_circular_search(
            current_pose,
            coarse_dock,
            boundary,
            approach_config=candidate_approach,
            arc_config=arc_config,
            boundary_config=boundary_config,
        )
        if candidate.valid:
            return AdaptiveBoundarySearchPlan(
                selected=candidate,
                attempted_radii_m=tuple(attempted_radii),
                radius_reductions=reduction,
                reason=(
                    "initial search radius is safely navigable"
                    if reduction == 0
                    else f"safe search found after {reduction} radius reduction(s)"
                ),
            )
        failed_candidates.append(candidate)

        if stand_off <= radius_config.min_stand_off_distance_m:
            break
        stand_off = max(
            radius_config.min_stand_off_distance_m,
            stand_off - radius_config.radius_step_m,
        )

    best_failure = max(
        failed_candidates,
        key=lambda candidate: candidate.safe_arc_length_m,
    )
    minimum_radius = replace(
        approach_config,
        stand_off_distance_m=radius_config.min_stand_off_distance_m,
    ).search_radius_m
    reached_minimum = math.isclose(
        attempted_radii[-1], minimum_radius, rel_tol=0.0, abs_tol=1e-9
    )
    return AdaptiveBoundarySearchPlan(
        selected=best_failure,
        attempted_radii_m=tuple(attempted_radii),
        radius_reductions=max(0, len(attempted_radii) - 1),
        reason=(
            "no safe search arc exists at or above the minimum allowed radius"
            if reached_minimum
            else "radius reduction attempt limit reached before the minimum radius"
        ),
    )


class RecoveryAction(str, Enum):
    CONTINUE = "CONTINUE"
    RETRY = "RETRY"
    EXIT_AND_RETURN = "EXIT_AND_RETURN"
    COMPLETE = "COMPLETE"


@dataclass(frozen=True)
class DockRetryConfig:
    max_attempts: int = 3
    attempt_timeout_s: float = 120.0
    total_timeout_s: float = 300.0

    def __post_init__(self) -> None:
        if isinstance(self.max_attempts, bool) or not isinstance(self.max_attempts, int):
            raise ValueError("max_attempts must be a positive integer")
        if self.max_attempts <= 0:
            raise ValueError("max_attempts must be a positive integer")
        for name, value in (
            ("attempt_timeout_s", self.attempt_timeout_s),
            ("total_timeout_s", self.total_timeout_s),
        ):
            if not _finite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")


@dataclass(frozen=True)
class RecoveryDecision:
    action: RecoveryAction
    attempt_number: int
    reason: str


class DockAttemptSupervisor:
    """Bound docking attempts by both count and elapsed time."""

    def __init__(self, config: DockRetryConfig = DockRetryConfig()) -> None:
        if not isinstance(config, DockRetryConfig):
            raise TypeError("config must be a DockRetryConfig")
        self._config = config
        self._mission_started_at: Optional[float] = None
        self._attempt_started_at: Optional[float] = None
        self._attempt_number = 0
        self._finished = False

    @property
    def attempt_number(self) -> int:
        return self._attempt_number

    def begin(self, now: float) -> RecoveryDecision:
        if not _finite(now):
            raise ValueError("now must be finite")
        self._mission_started_at = now
        self._attempt_started_at = now
        self._attempt_number = 1
        self._finished = False
        return RecoveryDecision(RecoveryAction.CONTINUE, 1, "first docking attempt")

    def record_failure(self, now: float, reason: str) -> RecoveryDecision:
        self._require_active(now)
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("reason must be a nonempty string")
        total_expired = now - self._mission_started_at >= self._config.total_timeout_s
        attempts_exhausted = self._attempt_number >= self._config.max_attempts
        if total_expired or attempts_exhausted:
            self._finished = True
            limit = "total time limit reached" if total_expired else "attempt limit reached"
            return RecoveryDecision(
                RecoveryAction.EXIT_AND_RETURN,
                self._attempt_number,
                f"{limit}: {reason.strip()}",
            )
        self._attempt_number += 1
        self._attempt_started_at = now
        return RecoveryDecision(
            RecoveryAction.RETRY,
            self._attempt_number,
            reason.strip(),
        )

    def check_time(self, now: float) -> RecoveryDecision:
        self._require_active(now)
        if now - self._mission_started_at >= self._config.total_timeout_s:
            self._finished = True
            return RecoveryDecision(
                RecoveryAction.EXIT_AND_RETURN,
                self._attempt_number,
                "total docking time limit reached",
            )
        if now - self._attempt_started_at >= self._config.attempt_timeout_s:
            return self.record_failure(now, "current docking attempt timed out")
        return RecoveryDecision(
            RecoveryAction.CONTINUE,
            self._attempt_number,
            "attempt remains within time limits",
        )

    def record_success(self, now: float) -> RecoveryDecision:
        self._require_active(now)
        self._finished = True
        return RecoveryDecision(
            RecoveryAction.COMPLETE,
            self._attempt_number,
            "docking task completed",
        )

    def _require_active(self, now: float) -> None:
        if not _finite(now):
            raise ValueError("now must be finite")
        if self._mission_started_at is None or self._attempt_started_at is None:
            raise RuntimeError("begin() must be called first")
        if self._finished:
            raise RuntimeError("docking supervision has already finished")
        if now < self._attempt_started_at:
            raise ValueError("now cannot precede the active attempt")


@dataclass(frozen=True)
class ExitAndReturnPlan:
    exit_goal: Pose2D
    return_goal: Pose2D


def plan_exit_and_return(
    entrance: PreciseEntrancePoseNED,
    return_goal: Pose2D,
    *,
    exit_distance_m: float = 4.0,
    boundary: Optional[NavigableBoundaryNED] = None,
    min_boundary_clearance_m: float = 0.0,
) -> ExitAndReturnPlan:
    """Create an outside-dock goal followed by a configured shore wait goal."""

    if not isinstance(entrance, PreciseEntrancePoseNED) or not entrance.valid:
        raise ValueError("a valid precise entrance pose is required")
    if not isinstance(return_goal, Pose2D):
        raise TypeError("return_goal must be a Pose2D")
    if not _finite(exit_distance_m) or exit_distance_m <= 0:
        raise ValueError("exit_distance_m must be positive and finite")
    if not _finite(min_boundary_clearance_m) or min_boundary_clearance_m < 0:
        raise ValueError("min_boundary_clearance_m must be finite and nonnegative")

    outward_heading = math.atan2(
        math.sin(entrance.heading_rad + math.pi),
        math.cos(entrance.heading_rad + math.pi),
    )
    exit_goal = Pose2D(
        entrance.x_ned - exit_distance_m * math.cos(entrance.heading_rad),
        entrance.y_ned - exit_distance_m * math.sin(entrance.heading_rad),
        outward_heading,
    )
    if boundary is not None:
        if not boundary.contains_with_clearance(
            exit_goal.x, exit_goal.y, min_boundary_clearance_m
        ):
            raise ValueError("exit goal is outside the navigable boundary")
        if not boundary.contains_with_clearance(
            return_goal.x, return_goal.y, min_boundary_clearance_m
        ):
            raise ValueError("return goal is outside the navigable boundary")
    return ExitAndReturnPlan(exit_goal=exit_goal, return_goal=return_goal)


__all__ = [
    "AdaptiveBoundarySearchPlan",
    "AdaptiveSearchRadiusConfig",
    "BoundaryCheckedSearchPlan",
    "DockAttemptSupervisor",
    "DockRetryConfig",
    "ExitAndReturnPlan",
    "NavigableBoundaryNED",
    "RecoveryAction",
    "RecoveryDecision",
    "SearchBoundaryConfig",
    "constrain_search_plan_to_boundary",
    "plan_boundary_safe_circular_search",
    "plan_adaptive_boundary_safe_circular_search",
    "plan_exit_and_return",
]
