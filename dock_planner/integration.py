"""Perception contracts and integration boundary for terminal docking.

NED means x=north, y=east, z=down, in metres. A planar heading is in
radians from north toward east (clockwise geographically). These numeric
x/y/heading values match the core's Cartesian +x/+y angle convention.
Source adapters must establish a common NED origin and timestamp clock.
"""

import math
from dataclasses import dataclass
from enum import Enum
from numbers import Real
from typing import Optional, Protocol, Sequence, Tuple

from .types import DockSpec, Pose2D


def _finite(value: object) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class NEDPoint:
    north: float
    east: float
    down: float

    def __post_init__(self) -> None:
        if not all(_finite(v) for v in (self.north, self.east, self.down)):
            raise ValueError("NED point coordinates must be finite real numbers")


@dataclass(frozen=True)
class CoarseDockInfo:
    """One-time UAV observation; corners are approximate, not boundaries."""

    dock_id: str
    center_ned: NEDPoint
    approximate_corners_ned: Tuple[NEDPoint, NEDPoint, NEDPoint, NEDPoint]
    timestamp: float
    valid: bool

    def __post_init__(self) -> None:
        if not isinstance(self.dock_id, str) or not self.dock_id.strip():
            raise ValueError("dock_id must be a nonempty string")
        if not isinstance(self.center_ned, NEDPoint):
            raise ValueError("center_ned must be a NEDPoint")
        corners = self.approximate_corners_ned
        if not isinstance(corners, (tuple, list)) or len(corners) != 4:
            raise ValueError("approximate_corners_ned must contain exactly four points")
        if not all(isinstance(point, NEDPoint) for point in corners):
            raise ValueError("approximate corners must be NEDPoint values")
        if len(set(corners)) != 4:
            raise ValueError("approximate corners must be four distinct points")
        object.__setattr__(self, "approximate_corners_ned", tuple(corners))
        if not _finite(self.timestamp):
            raise ValueError("timestamp must be a finite real number")
        if not isinstance(self.valid, bool):
            raise ValueError("valid must be a boolean")


@dataclass(frozen=True)
class PreciseEntrancePoseNED:
    """Future USV observation; heading points into the selected bay."""

    x_ned: float
    y_ned: float
    heading_rad: float
    timestamp: float
    valid: bool
    confidence: Optional[float] = None
    dock_id: Optional[str] = None

    def __post_init__(self) -> None:
        if not all(_finite(v) for v in (self.x_ned, self.y_ned, self.heading_rad, self.timestamp)):
            raise ValueError("entrance coordinates, heading and timestamp must be finite")
        if not isinstance(self.valid, bool):
            raise ValueError("valid must be a boolean")
        if self.confidence is not None and (not _finite(self.confidence) or not 0 <= self.confidence <= 1):
            raise ValueError("confidence must be None or in [0, 1]")
        if self.dock_id is not None and (not isinstance(self.dock_id, str) or not self.dock_id.strip()):
            raise ValueError("dock_id must be None or a nonempty string")


@dataclass(frozen=True)
class ObstaclePointNED:
    """Only an observed center; no physical radius or clearance is implied."""

    center_ned: NEDPoint

    def __post_init__(self) -> None:
        if not isinstance(self.center_ned, NEDPoint):
            raise ValueError("center_ned must be a NEDPoint")


@dataclass(frozen=True)
class GenericVelocityCommand:
    """Controller output independent of ROS, Gazebo, or a vehicle backend.

    ``timestamp`` and ``valid_until`` use the clock domain selected by the
    caller.  ``frame_id`` records the intended body frame for the internal
    contract; a bare ROS ``Twist`` does not carry it and the ROS adapter does
    not claim to publish it.
    """

    surge_mps: float
    yaw_rate_rps: float
    timestamp: float
    valid_until: float
    phase: str
    source: str
    frame_id: str = "base_link"

    def __post_init__(self) -> None:
        if not all(_finite(v) for v in (
            self.surge_mps, self.yaw_rate_rps, self.timestamp, self.valid_until,
        )):
            raise ValueError("velocity values and timestamps must be finite real numbers")
        if self.valid_until < self.timestamp:
            raise ValueError("valid_until must be greater than or equal to timestamp")
        for name, value in (("phase", self.phase), ("source", self.source), ("frame_id", self.frame_id)):
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be a nonempty string")

    def is_valid_at(self, now: float) -> bool:
        """Return whether ``now`` falls within this command's validity window."""

        if not _finite(now):
            raise ValueError("now must be a finite real number")
        return self.timestamp <= now <= self.valid_until


@dataclass(frozen=True)
class TerminalPlanInput:
    dock: DockSpec
    obstacles: Tuple[ObstaclePointNED, ...]


def prepare_terminal_plan_input(
    coarse_dock: CoarseDockInfo,
    entrance: PreciseEntrancePoseNED,
    bay_width: float,
    bay_depth: float,
    obstacle_points: Sequence[ObstaclePointNED] = (),
) -> TerminalPlanInput:
    """Bridge valid observations to the core without inferring an entrance.

    The caller supplies bay clear dimensions from a separate verified source.
    Nonempty obstacle points remain nonempty when passed to DockPlanner.plan,
    which currently returns UNSUPPORTED_INPUT. No clearance is inferred here.
    """
    if not isinstance(coarse_dock, CoarseDockInfo) or not coarse_dock.valid:
        raise ValueError("a valid coarse dock observation is required")
    if not isinstance(entrance, PreciseEntrancePoseNED) or not entrance.valid:
        raise ValueError("a valid precise entrance observation is required")
    if entrance.dock_id is not None and entrance.dock_id != coarse_dock.dock_id:
        raise ValueError("entrance dock_id does not match coarse dock_id")
    if not _finite(bay_width) or bay_width <= 0 or not _finite(bay_depth) or bay_depth <= 0:
        raise ValueError("bay_width and bay_depth must be positive finite numbers")
    if not isinstance(obstacle_points, (tuple, list)) or not all(
        isinstance(point, ObstaclePointNED) for point in obstacle_points
    ):
        raise ValueError("obstacle_points must be a list or tuple of ObstaclePointNED")
    return TerminalPlanInput(
        dock=DockSpec(
            entrance_center_x=entrance.x_ned,
            entrance_center_y=entrance.y_ned,
            entrance_heading=entrance.heading_rad,
            bay_width=bay_width,
            bay_depth=bay_depth,
            confidence=entrance.confidence,
            dock_id=coarse_dock.dock_id,
        ),
        obstacles=tuple(obstacle_points),
    )


class GlobalApproachStatus(str, Enum):
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"


@dataclass(frozen=True)
class GlobalApproachResult:
    status: GlobalApproachStatus
    waypoints: Tuple[Pose2D, ...]
    failure_reason: Optional[str] = None


class GlobalApproachPlanner(Protocol):
    """Future planner interface; no search or avoidance implementation here."""

    def plan(
        self,
        current_pose: Pose2D,
        approach_goal: Pose2D,
        obstacle_points: Sequence[ObstaclePointNED],
        *,
        safety_radius: Optional[float] = None,
        planning_boundary: Optional[Sequence[NEDPoint]] = None,
    ) -> GlobalApproachResult:
        ...
