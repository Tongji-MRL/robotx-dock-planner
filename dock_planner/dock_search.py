"""Deterministic center-only approach and circular dock-search geometry.

This module produces goals only.  It does not control a vehicle, avoid
obstacles, parse LiDAR messages, or assume that a complete circle is safe.
The configured arc must describe water that the mission layer has determined
is navigable.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from typing import Tuple

from .integration import (
    CoarseDockCenterObservationNED,
    CoarseDockSearchConfig,
    NEDPoint,
    coarse_dock_staging_pose,
)
from .types import Pose2D


def _finite(value: object) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class CircularDockSearchConfig:
    """Sampling policy for a bounded arc around a coarse dock center.

    In NED, increasing ``atan2(east, north)`` moves clockwise on a geographic
    map.  ``clockwise=True`` therefore increases the polar angle.
    """

    arc_span_rad: float = math.pi
    waypoint_spacing_m: float = 1.5
    clockwise: bool = True

    def __post_init__(self) -> None:
        if not _finite(self.arc_span_rad) or not 0 < self.arc_span_rad <= 2 * math.pi:
            raise ValueError("arc_span_rad must be finite and in (0, 2*pi]")
        if not _finite(self.waypoint_spacing_m) or self.waypoint_spacing_m <= 0:
            raise ValueError("waypoint_spacing_m must be positive and finite")
        if not isinstance(self.clockwise, bool):
            raise ValueError("clockwise must be a boolean")


@dataclass(frozen=True)
class CircularDockSearchPlan:
    """Approach goal plus bounded arc goals for one coarse dock."""

    dock_id: str
    center_ned: NEDPoint
    radius_m: float
    staging_pose: Pose2D
    arc_waypoints: Tuple[Pose2D, ...]
    clockwise: bool
    arc_span_rad: float

    def __post_init__(self) -> None:
        if not isinstance(self.dock_id, str) or not self.dock_id.strip():
            raise ValueError("dock_id must be a nonempty string")
        if not isinstance(self.center_ned, NEDPoint):
            raise ValueError("center_ned must be a NEDPoint")
        if not _finite(self.radius_m) or self.radius_m <= 0:
            raise ValueError("radius_m must be positive and finite")
        if not isinstance(self.staging_pose, Pose2D):
            raise ValueError("staging_pose must be a Pose2D")
        if not isinstance(self.arc_waypoints, (tuple, list)) or not self.arc_waypoints:
            raise ValueError("arc_waypoints must be a nonempty sequence")
        if not all(isinstance(pose, Pose2D) for pose in self.arc_waypoints):
            raise ValueError("arc_waypoints must contain Pose2D values")
        object.__setattr__(self, "arc_waypoints", tuple(self.arc_waypoints))
        if not isinstance(self.clockwise, bool):
            raise ValueError("clockwise must be a boolean")
        if not _finite(self.arc_span_rad) or not 0 < self.arc_span_rad <= 2 * math.pi:
            raise ValueError("arc_span_rad must be finite and in (0, 2*pi]")


def plan_circular_dock_search(
    current_pose: Pose2D,
    coarse_dock: CoarseDockCenterObservationNED,
    approach_config: CoarseDockSearchConfig = CoarseDockSearchConfig(),
    arc_config: CircularDockSearchConfig = CircularDockSearchConfig(),
) -> CircularDockSearchPlan:
    """Create a near-side staging goal followed by tangent arc goals.

    The first arc goal is after the staging point, so a caller can separately
    confirm that the USV reached the safe circle before entering search mode.
    """

    if not isinstance(current_pose, Pose2D):
        raise TypeError("current_pose must be a Pose2D")
    if not isinstance(coarse_dock, CoarseDockCenterObservationNED) or not coarse_dock.valid:
        raise ValueError("a valid center-only coarse dock observation is required")
    if not isinstance(approach_config, CoarseDockSearchConfig):
        raise TypeError("approach_config must be a CoarseDockSearchConfig")
    if not isinstance(arc_config, CircularDockSearchConfig):
        raise TypeError("arc_config must be a CircularDockSearchConfig")

    staging = coarse_dock_staging_pose(current_pose, coarse_dock, approach_config)
    center = coarse_dock.center_ned
    radius = approach_config.search_radius_m
    start_angle = math.atan2(staging.y - center.east, staging.x - center.north)
    direction = 1.0 if arc_config.clockwise else -1.0
    segment_count = max(
        1,
        int(math.ceil(radius * arc_config.arc_span_rad / arc_config.waypoint_spacing_m)),
    )
    step = arc_config.arc_span_rad / segment_count
    waypoints = []
    for index in range(1, segment_count + 1):
        angle = start_angle + direction * step * index
        tangent_heading = angle + direction * math.pi / 2.0
        tangent_heading = math.atan2(math.sin(tangent_heading), math.cos(tangent_heading))
        waypoints.append(Pose2D(
            center.north + radius * math.cos(angle),
            center.east + radius * math.sin(angle),
            tangent_heading,
        ))
    return CircularDockSearchPlan(
        dock_id=coarse_dock.dock_id,
        center_ned=center,
        radius_m=radius,
        staging_pose=staging,
        arc_waypoints=tuple(waypoints),
        clockwise=arc_config.clockwise,
        arc_span_rad=arc_config.arc_span_rad,
    )


__all__ = [
    "CircularDockSearchConfig",
    "CircularDockSearchPlan",
    "plan_circular_dock_search",
]
