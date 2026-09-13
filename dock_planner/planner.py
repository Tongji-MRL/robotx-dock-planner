"""Straight waypoint geometry only; no search, control or collision checking."""

import math
from numbers import Real
from typing import Optional, Sequence

from .types import DockSpec, PlannerConfig, PlanResult, PlanStatus, Pose2D


def _finite_number(value: object) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


class DockPlanner:
    def __init__(self, config: PlannerConfig):
        self.config = config

    def plan(
        self,
        usv_pose: Pose2D,
        dock: DockSpec,
        obstacles: Optional[Sequence[object]] = None,
    ) -> PlanResult:
        """Return [USV, approach, entrance, stop] in the caller's common frame.

        Yaw is counterclockwise from +X in radians. The initial USV yaw is
        preserved; all subsequent poses face into the bay. Connecting segments
        may have corners or disagree with the initial heading. SUCCESS certifies
        only this geometric construction, not a dynamically feasible trajectory.
        Nonempty obstacle input is explicitly unsupported in V0.1.
        """
        def invalid(reason: str) -> PlanResult:
            return PlanResult(None, status=PlanStatus.INVALID_INPUT, reason=reason)

        if not isinstance(usv_pose, Pose2D):
            return invalid("usv_pose must be Pose2D")
        if not isinstance(dock, DockSpec):
            return invalid("dock must be DockSpec")
        if not isinstance(self.config, PlannerConfig):
            return invalid("config must be PlannerConfig")

        fields = {
            "usv_pose.x": usv_pose.x,
            "usv_pose.y": usv_pose.y,
            "usv_pose.yaw": usv_pose.yaw,
            "entrance_center_x": dock.entrance_center_x,
            "entrance_center_y": dock.entrance_center_y,
            "entrance_heading": dock.entrance_heading,
            "bay_width": dock.bay_width,
            "bay_depth": dock.bay_depth,
            "approach_distance": self.config.approach_distance,
            "stop_distance_inside": self.config.stop_distance_inside,
        }
        for name, value in fields.items():
            if not _finite_number(value):
                return invalid(f"{name} must be a finite real number")
        if dock.bay_width <= 0 or dock.bay_depth <= 0:
            return invalid("bay_width and bay_depth must be positive")
        if self.config.approach_distance <= 0:
            return invalid("approach_distance must be positive (outside the entrance)")
        if not 0 < self.config.stop_distance_inside < dock.bay_depth:
            return invalid("stop_distance_inside must be strictly between 0 and bay_depth")
        if dock.confidence is not None and (
            not _finite_number(dock.confidence) or not 0 <= dock.confidence <= 1
        ):
            return invalid("confidence must be None or a finite number in [0, 1]")
        if dock.dock_id is not None and not isinstance(dock.dock_id, str):
            return invalid("dock_id must be None or a string")
        if obstacles is not None:
            if not isinstance(obstacles, (list, tuple)) or len(obstacles) != 0:
                return PlanResult(
                    None, status=PlanStatus.UNSUPPORTED_INPUT,
                    reason="V0.1 supports only obstacles=None or an empty list/tuple",
                )

        heading = dock.entrance_heading
        dx, dy = math.cos(heading), math.sin(heading)
        ex, ey = dock.entrance_center_x, dock.entrance_center_y
        approach = Pose2D(
            ex - self.config.approach_distance * dx,
            ey - self.config.approach_distance * dy,
            heading,
        )
        entrance = Pose2D(ex, ey, heading)
        stop = Pose2D(
            ex + self.config.stop_distance_inside * dx,
            ey + self.config.stop_distance_inside * dy,
            heading,
        )
        if not all(_finite_number(v) for p in (approach, stop) for v in (p.x, p.y)):
            return invalid("computed coordinates exceed the finite numeric range")
        return PlanResult(
            approach_pose=approach,
            waypoints=[usv_pose, approach, entrance, stop],
            status=PlanStatus.SUCCESS,
            reason="Geometric plan generated; collision and motion feasibility are not checked.",
        )
