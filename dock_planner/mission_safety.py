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
from .types import PlanResult, PlanStatus, Pose2D


_GEOMETRY_EPSILON = 1e-9


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


def _orientation(a: NEDPoint, b: NEDPoint, c: NEDPoint) -> float:
    return (
        (b.north - a.north) * (c.east - a.east)
        - (b.east - a.east) * (c.north - a.north)
    )


def _on_segment(a: NEDPoint, b: NEDPoint, point: NEDPoint) -> bool:
    return (
        min(a.north, b.north) - _GEOMETRY_EPSILON
        <= point.north
        <= max(a.north, b.north) + _GEOMETRY_EPSILON
        and min(a.east, b.east) - _GEOMETRY_EPSILON
        <= point.east
        <= max(a.east, b.east) + _GEOMETRY_EPSILON
        and abs(_orientation(a, b, point)) <= _GEOMETRY_EPSILON
    )


def _segments_intersect(
    first_start: NEDPoint,
    first_end: NEDPoint,
    second_start: NEDPoint,
    second_end: NEDPoint,
) -> bool:
    o1 = _orientation(first_start, first_end, second_start)
    o2 = _orientation(first_start, first_end, second_end)
    o3 = _orientation(second_start, second_end, first_start)
    o4 = _orientation(second_start, second_end, first_end)
    if (
        ((o1 > _GEOMETRY_EPSILON and o2 < -_GEOMETRY_EPSILON)
         or (o1 < -_GEOMETRY_EPSILON and o2 > _GEOMETRY_EPSILON))
        and ((o3 > _GEOMETRY_EPSILON and o4 < -_GEOMETRY_EPSILON)
             or (o3 < -_GEOMETRY_EPSILON and o4 > _GEOMETRY_EPSILON))
    ):
        return True
    return any((
        abs(o1) <= _GEOMETRY_EPSILON and _on_segment(first_start, first_end, second_start),
        abs(o2) <= _GEOMETRY_EPSILON and _on_segment(first_start, first_end, second_end),
        abs(o3) <= _GEOMETRY_EPSILON and _on_segment(second_start, second_end, first_start),
        abs(o4) <= _GEOMETRY_EPSILON and _on_segment(second_start, second_end, first_end),
    ))


def _segment_segment_distance(
    first_start: NEDPoint,
    first_end: NEDPoint,
    second_start: NEDPoint,
    second_end: NEDPoint,
) -> float:
    if _segments_intersect(first_start, first_end, second_start, second_end):
        return 0.0
    return min(
        _point_segment_distance(first_start.north, first_start.east, second_start, second_end),
        _point_segment_distance(first_end.north, first_end.east, second_start, second_end),
        _point_segment_distance(second_start.north, second_start.east, first_start, first_end),
        _point_segment_distance(second_end.north, second_end.east, first_start, first_end),
    )


def _pose_point(pose: Pose2D) -> NEDPoint:
    return NEDPoint(pose.x, pose.y, 0.0)


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
        if len({(point.north, point.east) for point in vertices}) != len(vertices):
            raise ValueError("boundary vertices must be distinct")
        edges = tuple(zip(vertices, vertices[1:] + vertices[:1]))
        for first_index, (first_start, first_end) in enumerate(edges):
            for second_index in range(first_index + 1, len(edges)):
                if second_index in {
                    first_index,
                    first_index + 1,
                } or (first_index == 0 and second_index == len(edges) - 1):
                    continue
                second_start, second_end = edges[second_index]
                if _segments_intersect(
                    first_start,
                    first_end,
                    second_start,
                    second_end,
                ):
                    raise ValueError("boundary polygon must not self-intersect")
        area_twice = sum(
            start.north * end.east - end.north * start.east
            for start, end in edges
        )
        if abs(area_twice) <= 1e-9:
            raise ValueError("boundary polygon must have nonzero area")
        object.__setattr__(self, "vertices", vertices)

    @property
    def edges(self) -> Tuple[Tuple[NEDPoint, NEDPoint], ...]:
        return tuple(zip(self.vertices, self.vertices[1:] + self.vertices[:1]))

    def contains(self, north: float, east: float) -> bool:
        if not _finite(north) or not _finite(east):
            raise ValueError("point coordinates must be finite")
        inside = False
        vertices = self.vertices
        previous = vertices[-1]
        for current in vertices:
            if _point_segment_distance(north, east, previous, current) <= _GEOMETRY_EPSILON:
                return False
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
            for start, end in self.edges
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

    def segment_clearance_m(self, start: Pose2D, end: Pose2D) -> float:
        """Return the exact minimum distance from a straight segment to the boundary."""

        if not isinstance(start, Pose2D) or not isinstance(end, Pose2D):
            raise TypeError("segment endpoints must be Pose2D values")
        start_point = _pose_point(start)
        end_point = _pose_point(end)
        return min(
            _segment_segment_distance(start_point, end_point, edge_start, edge_end)
            for edge_start, edge_end in self.edges
        )

    def segment_with_clearance(
        self,
        start: Pose2D,
        end: Pose2D,
        clearance_m: float,
    ) -> bool:
        """Return whether an entire straight segment stays inside with clearance."""

        if not _finite(clearance_m) or clearance_m < 0:
            raise ValueError("clearance_m must be finite and nonnegative")
        if not isinstance(start, Pose2D) or not isinstance(end, Pose2D):
            raise TypeError("segment endpoints must be Pose2D values")
        if not self.contains_with_clearance(start.x, start.y, clearance_m):
            return False
        if not self.contains_with_clearance(end.x, end.y, clearance_m):
            return False
        start_point = _pose_point(start)
        end_point = _pose_point(end)
        if any(
            _segments_intersect(start_point, end_point, edge_start, edge_end)
            for edge_start, edge_end in self.edges
        ):
            return False
        return self.segment_clearance_m(start, end) + _GEOMETRY_EPSILON >= clearance_m


@dataclass(frozen=True)
class BoundaryPathValidation:
    valid: bool
    failed_segment_index: Optional[int]
    minimum_clearance_m: Optional[float]
    reason: str


def validate_pose_path_to_boundary(
    poses: Sequence[Pose2D],
    boundary: NavigableBoundaryNED,
    *,
    min_boundary_clearance_m: float = 0.0,
) -> BoundaryPathValidation:
    """Validate every straight segment of a waypoint path against a polygon."""

    if not isinstance(poses, (tuple, list)) or not poses:
        raise ValueError("poses must be a nonempty sequence")
    if not all(isinstance(pose, Pose2D) for pose in poses):
        raise TypeError("poses must contain Pose2D values")
    if not isinstance(boundary, NavigableBoundaryNED):
        raise TypeError("boundary must be a NavigableBoundaryNED")
    if not _finite(min_boundary_clearance_m) or min_boundary_clearance_m < 0:
        raise ValueError("min_boundary_clearance_m must be finite and nonnegative")

    if len(poses) == 1:
        pose = poses[0]
        clearance = boundary.clearance_m(pose.x, pose.y)
        valid = boundary.contains_with_clearance(
            pose.x,
            pose.y,
            min_boundary_clearance_m,
        )
        return BoundaryPathValidation(
            valid=valid,
            failed_segment_index=None if valid else 0,
            minimum_clearance_m=clearance,
            reason="single waypoint is safe" if valid else "waypoint is outside boundary or too close to shore",
        )

    minimum_clearance = math.inf
    for index, (start, end) in enumerate(zip(poses, poses[1:])):
        segment_clearance = boundary.segment_clearance_m(start, end)
        minimum_clearance = min(minimum_clearance, segment_clearance)
        if not boundary.segment_with_clearance(
            start,
            end,
            min_boundary_clearance_m,
        ):
            return BoundaryPathValidation(
                valid=False,
                failed_segment_index=index,
                minimum_clearance_m=segment_clearance,
                reason=(
                    f"segment {index} leaves the navigable boundary or passes "
                    "too close to shore"
                ),
            )
    return BoundaryPathValidation(
        valid=True,
        failed_segment_index=None,
        minimum_clearance_m=minimum_clearance,
        reason="all path segments are safely navigable",
    )


def validate_terminal_plan_to_boundary(
    plan: PlanResult,
    boundary: NavigableBoundaryNED,
    *,
    min_boundary_clearance_m: float = 0.0,
) -> BoundaryPathValidation:
    """Validate all straight connections in a successful terminal plan."""

    if not isinstance(plan, PlanResult):
        raise TypeError("plan must be a PlanResult")
    if plan.status is not PlanStatus.SUCCESS or len(plan.waypoints) < 2:
        raise ValueError("a successful plan with at least two waypoints is required")
    return validate_pose_path_to_boundary(
        plan.waypoints,
        boundary,
        min_boundary_clearance_m=min_boundary_clearance_m,
    )


@dataclass(frozen=True)
class SearchBoundaryConfig:
    min_boundary_clearance_m: float = 1.0
    min_safe_arc_length_m: float = 4.0

    def __post_init__(self) -> None:
        if not _finite(self.min_boundary_clearance_m) or self.min_boundary_clearance_m < 0:
            raise ValueError("min_boundary_clearance_m must be finite and nonnegative")
        if not _finite(self.min_safe_arc_length_m) or self.min_safe_arc_length_m <= 0:
            raise ValueError("min_safe_arc_length_m must be positive and finite")


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
        if not boundary.segment_with_clearance(
            previous,
            waypoint,
            config.min_boundary_clearance_m,
        ):
            break
        safe_length += math.hypot(waypoint.x - previous.x, waypoint.y - previous.y)
        safe_waypoints.append(waypoint)
        clearances.append(boundary.segment_clearance_m(previous, waypoint))
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

    The entire straight transition from the current pose to staging is checked
    as well.  This matters when the USV starts inside the search circle and
    must travel outward before beginning the arc.
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
            transition = validate_pose_path_to_boundary(
                (current_pose, raw_plan.staging_pose),
                boundary,
                min_boundary_clearance_m=boundary_config.min_boundary_clearance_m,
            )
            if not transition.valid:
                constrained = BoundaryCheckedSearchPlan(
                    plan=None,
                    valid=False,
                    full_requested_arc_safe=False,
                    rejected_waypoint_count=len(raw_plan.arc_waypoints),
                    safe_arc_length_m=0.0,
                    minimum_clearance_m=transition.minimum_clearance_m,
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
    minimum_boundary_clearance_m: Optional[float] = None


@dataclass(frozen=True)
class SafeReturnGoalConfig:
    min_dock_distance_m: float
    min_boundary_clearance_m: float = 1.0
    grid_spacing_m: float = 0.5
    max_grid_points: int = 100000

    def __post_init__(self) -> None:
        for name, value in (
            ("min_dock_distance_m", self.min_dock_distance_m),
            ("grid_spacing_m", self.grid_spacing_m),
        ):
            if not _finite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if (
            not _finite(self.min_boundary_clearance_m)
            or self.min_boundary_clearance_m < 0
        ):
            raise ValueError(
                "min_boundary_clearance_m must be finite and nonnegative"
            )
        if (
            isinstance(self.max_grid_points, bool)
            or not isinstance(self.max_grid_points, int)
            or self.max_grid_points <= 0
        ):
            raise ValueError("max_grid_points must be a positive integer")


def select_safe_return_goal(
    boundary: NavigableBoundaryNED,
    dock_center: NEDPoint,
    config: SafeReturnGoalConfig,
    *,
    preferred_goal: Optional[Pose2D] = None,
) -> Pose2D:
    """Select a deterministic safe waiting point inside a navigable polygon.

    A safe preferred point is preserved.  Otherwise a bounded grid search
    chooses the safe point nearest the preference, or the point farthest from
    the dock when no preference is available.  Path safety is still checked
    later by :func:`plan_exit_and_return`.
    """

    if not isinstance(boundary, NavigableBoundaryNED):
        raise TypeError("boundary must be a NavigableBoundaryNED")
    if not isinstance(dock_center, NEDPoint):
        raise TypeError("dock_center must be a NEDPoint")
    if not isinstance(config, SafeReturnGoalConfig):
        raise TypeError("config must be a SafeReturnGoalConfig")
    if preferred_goal is not None and not isinstance(preferred_goal, Pose2D):
        raise TypeError("preferred_goal must be None or a Pose2D")

    def safe(pose: Pose2D) -> bool:
        return (
            boundary.contains_with_clearance(
                pose.x,
                pose.y,
                config.min_boundary_clearance_m,
            )
            and math.hypot(
                pose.x - dock_center.north,
                pose.y - dock_center.east,
            ) >= config.min_dock_distance_m
        )

    if preferred_goal is not None and safe(preferred_goal):
        return preferred_goal

    min_north = min(point.north for point in boundary.vertices)
    max_north = max(point.north for point in boundary.vertices)
    min_east = min(point.east for point in boundary.vertices)
    max_east = max(point.east for point in boundary.vertices)
    north_count = int(math.ceil((max_north - min_north) / config.grid_spacing_m)) + 1
    east_count = int(math.ceil((max_east - min_east) / config.grid_spacing_m)) + 1
    if north_count * east_count > config.max_grid_points:
        raise ValueError("return-goal grid exceeds max_grid_points; increase grid spacing")

    candidates = []
    for north_index in range(north_count):
        north = min(min_north + north_index * config.grid_spacing_m, max_north)
        for east_index in range(east_count):
            east = min(min_east + east_index * config.grid_spacing_m, max_east)
            yaw = math.atan2(dock_center.east - east, dock_center.north - north)
            candidate = Pose2D(north, east, yaw)
            if safe(candidate):
                candidates.append(candidate)
    if not candidates:
        raise ValueError("no safe return goal exists for the configured clearances")

    if preferred_goal is not None:
        return min(
            candidates,
            key=lambda pose: (
                math.hypot(pose.x - preferred_goal.x, pose.y - preferred_goal.y),
                -boundary.clearance_m(pose.x, pose.y),
            ),
        )
    return max(
        candidates,
        key=lambda pose: (
            math.hypot(pose.x - dock_center.north, pose.y - dock_center.east),
            boundary.clearance_m(pose.x, pose.y),
        ),
    )


def plan_exit_and_return(
    entrance: PreciseEntrancePoseNED,
    return_goal: Pose2D,
    *,
    current_pose: Optional[Pose2D] = None,
    exit_distance_m: float = 4.0,
    boundary: Optional[NavigableBoundaryNED] = None,
    min_boundary_clearance_m: float = 0.0,
) -> ExitAndReturnPlan:
    """Create an outside-dock goal followed by a configured shore wait goal."""

    if not isinstance(entrance, PreciseEntrancePoseNED) or not entrance.valid:
        raise ValueError("a valid precise entrance pose is required")
    if not isinstance(return_goal, Pose2D):
        raise TypeError("return_goal must be a Pose2D")
    if current_pose is not None and not isinstance(current_pose, Pose2D):
        raise TypeError("current_pose must be None or a Pose2D")
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
    minimum_clearance = None
    if boundary is not None:
        if not isinstance(boundary, NavigableBoundaryNED):
            raise TypeError("boundary must be None or a NavigableBoundaryNED")
        start = current_pose or Pose2D(
            entrance.x_ned,
            entrance.y_ned,
            entrance.heading_rad,
        )
        validation = validate_pose_path_to_boundary(
            (start, exit_goal, return_goal),
            boundary,
            min_boundary_clearance_m=min_boundary_clearance_m,
        )
        if not validation.valid:
            raise ValueError(f"exit/return path is unsafe: {validation.reason}")
        minimum_clearance = validation.minimum_clearance_m
    return ExitAndReturnPlan(
        exit_goal=exit_goal,
        return_goal=return_goal,
        minimum_boundary_clearance_m=minimum_clearance,
    )


__all__ = [
    "AdaptiveBoundarySearchPlan",
    "AdaptiveSearchRadiusConfig",
    "BoundaryPathValidation",
    "BoundaryCheckedSearchPlan",
    "DockAttemptSupervisor",
    "DockRetryConfig",
    "ExitAndReturnPlan",
    "NavigableBoundaryNED",
    "RecoveryAction",
    "RecoveryDecision",
    "SafeReturnGoalConfig",
    "SearchBoundaryConfig",
    "constrain_search_plan_to_boundary",
    "plan_boundary_safe_circular_search",
    "plan_adaptive_boundary_safe_circular_search",
    "plan_exit_and_return",
    "select_safe_return_goal",
    "validate_pose_path_to_boundary",
    "validate_terminal_plan_to_boundary",
]
