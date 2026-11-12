"""Task3 mission memory and navigation constraints.

The main FSM forwards the official GPS centre and radius.  Task3 converts that
centre to the shared RTK NED frame, evaluates the live USV pose, and plans both
an exit (when already inside) and a detour (when outside).
"""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Mapping, Sequence

from .types import Pose2D


class Task3ResumePhase(str, Enum):
    EXPLORING = "EXPLORING"
    DOCK_KNOWN = "DOCK_KNOWN"
    CORRECT_BAY_KNOWN = "CORRECT_BAY_KNOWN"
    RETURNING = "RETURNING"


def resume_phase(step: str, context: Mapping) -> Task3ResumePhase:
    """Four summary phases supplement, rather than replace, the detailed step."""
    step = step.removeprefix("TASK3_")
    if step in {"EXIT_RETURN", "EXIT_DOCK", "REPORT_RESULT", "RETURNING", "FINISHED", "COMPLETE", "FINISH"}:
        return Task3ResumePhase.RETURNING
    if context.get("watering_completed") is True:
        return Task3ResumePhase.RETURNING
    if context.get("correct_bay_confirmed") is True or step in {
        "TERMINAL_DOCKING", "DOCKED", "WAIT_RED_WINDOW", "AIMING", "SPRAYING",
        "VERIFY_GREEN", "WATERING", "ENTER_BAY", "HOLD_POSITION", "AIM_WINDOW", "SPRAY_WATER", "COLOR_SEQUENCE", "WAIT_UAV_DELIVERY",
    }:
        return Task3ResumePhase.CORRECT_BAY_KNOWN
    if context.get("coarse_dock_center") is not None or context.get("dock_id") is not None or step in {
        "APPROACH_DOCK", "VERIFY_DOCK", "INSPECT_BAYS", "CHECK_BAY_COLOR", "NEXT_BAY",
    }:
        return Task3ResumePhase.DOCK_KNOWN
    return Task3ResumePhase.EXPLORING


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return float(value)


@dataclass(frozen=True)
class KeepOutCircle:
    north: float
    east: float
    radius_m: float
    clearance_m: float
    frame_id: str

    def intersects(self, start, end) -> bool:
        """Closest point on the entire segment; tangent contact is blocked."""
        sx, sy = start
        ex, ey = end
        for value in (*start, *end):
            _number(value, "path coordinate")
        dx, dy = ex - sx, ey - sy
        length2 = dx * dx + dy * dy
        t = 0.0 if length2 == 0 else max(0.0, min(1.0,
            ((self.north - sx) * dx + (self.east - sy) * dy) / length2))
        return math.hypot(sx + t * dx - self.north, sy + t * dy - self.east) <= self.radius_m + self.clearance_m

    def contains(self, point, *, extra_margin_m: float = 0.0) -> bool:
        return math.hypot(point[0] - self.north, point[1] - self.east) <= (
            self.radius_m + self.clearance_m + extra_margin_m
        )


@dataclass(frozen=True)
class KeepOutDetour:
    """A collision-free waypoint chain around one circular keep-out zone."""

    waypoints: tuple[Pose2D, ...]
    side: str
    length_m: float
    clearance_radius_m: float


def plan_keep_out_exit(
    start: Pose2D,
    zone: KeepOutCircle,
    *,
    boundary: Sequence[tuple[float, float]],
    exit_margin_m: float = 0.5,
) -> Pose2D:
    """Return the nearest geofence-safe point outside the protected circle.

    The normal radial direction is shortest.  Nearby directions are tried as a
    fallback when the circle touches the geofence.  This route is only used to
    leave an active zone; ordinary paths are never allowed through the circle.
    """
    if not isinstance(start, Pose2D):
        raise TypeError("start must be Pose2D")
    margin = _number(exit_margin_m, "exit_margin_m")
    if margin < 0.0:
        raise ValueError("exit_margin_m must be nonnegative")
    polygon = tuple((_number(item[0], "boundary north"),
                     _number(item[1], "boundary east")) for item in boundary)
    radius = zone.radius_m + zone.clearance_m + margin
    dx, dy = start.x - zone.north, start.y - zone.east
    preferred = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else None
    if preferred is None:
        angles = [index * math.pi / 12.0 for index in range(24)]
    else:
        offsets = [0.0]
        for index in range(1, 13):
            offsets.extend((index * math.pi / 12.0, -index * math.pi / 12.0))
        angles = [preferred + offset for offset in offsets]
    candidates = []
    for angle in angles:
        point = (zone.north + radius * math.cos(angle),
                 zone.east + radius * math.sin(angle))
        if _inside_convex_polygon(point, polygon):
            candidates.append((math.dist((start.x, start.y), point), angle, point))
    if not candidates:
        raise ValueError("KEEP_OUT_NO_EXIT_INSIDE_GEOFENCE")
    _, angle, point = min(candidates, key=lambda item: item[0])
    return Pose2D(point[0], point[1], angle)


def _inside_convex_polygon(point, polygon: Sequence[tuple[float, float]]) -> bool:
    if len(polygon) < 3:
        return False
    signs = []
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        cross = ((second[0] - first[0]) * (point[1] - first[1])
                 - (second[1] - first[1]) * (point[0] - first[0]))
        if abs(cross) > 1e-8:
            signs.append(cross > 0.0)
    return not signs or all(value == signs[0] for value in signs)


def _tangent_angles(point, center, radius):
    dx, dy = point[0] - center[0], point[1] - center[1]
    distance = math.hypot(dx, dy)
    if distance <= radius:
        return ()
    bearing = math.atan2(dy, dx)
    offset = math.acos(radius / distance)
    return (bearing - offset, bearing + offset)


def _positive_delta(start: float, end: float, direction: int) -> float:
    return (direction * (end - start)) % (2.0 * math.pi)


def plan_keep_out_detour(
    start: Pose2D,
    target: Pose2D,
    zone: KeepOutCircle,
    *,
    boundary: Sequence[tuple[float, float]],
    detour_margin_m: float = 0.5,
    arc_spacing_m: float = 1.0,
) -> KeepOutDetour:
    """Plan the shorter valid tangent/arc/tangent route around a circle.

    The polygon is expected to be the ordered, convex RobotX geofence.  Every
    sampled arc point must remain inside it.  Straight segments between those
    points therefore also remain inside the convex boundary.
    """
    if not isinstance(start, Pose2D) or not isinstance(target, Pose2D):
        raise TypeError("start and target must be Pose2D")
    margin = _number(detour_margin_m, "detour_margin_m")
    spacing = _number(arc_spacing_m, "arc_spacing_m")
    if margin < 0.0 or spacing <= 0.0:
        raise ValueError("detour margin must be nonnegative and spacing positive")
    polygon = tuple((_number(item[0], "boundary north"),
                     _number(item[1], "boundary east")) for item in boundary)
    start_xy, target_xy = (start.x, start.y), (target.x, target.y)
    radius = zone.radius_m + zone.clearance_m + margin
    if math.hypot(start.x - zone.north, start.y - zone.east) <= radius:
        raise ValueError("KEEP_OUT_START_INSIDE_CLEARANCE")
    if math.hypot(target.x - zone.north, target.y - zone.east) <= radius:
        raise ValueError("KEEP_OUT_TARGET_INSIDE_CLEARANCE")
    if not _inside_convex_polygon(start_xy, polygon) or not _inside_convex_polygon(target_xy, polygon):
        raise ValueError("KEEP_OUT_ENDPOINT_OUTSIDE_GEOFENCE")
    center = (zone.north, zone.east)
    start_angles = _tangent_angles(start_xy, center, radius)
    target_angles = _tangent_angles(target_xy, center, radius)
    candidates = []
    for start_angle in start_angles:
        for target_angle in target_angles:
            for direction, side in ((1, "COUNTERCLOCKWISE"), (-1, "CLOCKWISE")):
                sweep = _positive_delta(start_angle, target_angle, direction)
                # At least one interior point keeps a near-zero arc explicit.
                steps = max(1, int(math.ceil(radius * sweep / spacing)))
                angles = [start_angle + direction * sweep * index / steps
                          for index in range(steps + 1)]
                points = [(zone.north + radius * math.cos(angle),
                           zone.east + radius * math.sin(angle)) for angle in angles]
                if not all(_inside_convex_polygon(point, polygon) for point in points):
                    continue
                length = (math.dist(start_xy, points[0]) + radius * sweep
                          + math.dist(points[-1], target_xy))
                route = points + [target_xy]
                waypoints = []
                previous = start_xy
                for index, point in enumerate(route):
                    following = route[index + 1] if index + 1 < len(route) else point
                    if following == point:
                        yaw = target.yaw
                    else:
                        yaw = math.atan2(following[1] - point[1], following[0] - point[0])
                    if math.dist(previous, point) > 1e-6:
                        waypoints.append(Pose2D(point[0], point[1], yaw))
                    previous = point
                candidates.append(KeepOutDetour(tuple(waypoints), side, length, radius))
    if not candidates:
        raise ValueError("KEEP_OUT_NO_DETOUR_INSIDE_GEOFENCE")
    return min(candidates, key=lambda item: item.length_m)


class Task3NavigationConstraint:
    """Versioned normalized zones; no TTL and no implicit AllClear."""

    def __init__(self):
        self.run_id = None
        self.revision = -1
        self.zone = None
        self._snapshot = None

    def apply(self, value: Mapping) -> bool:
        if not isinstance(value, Mapping):
            raise ValueError("constraint must be an object")
        run_id, revision = value.get("run_id"), value.get("revision")
        for name, number in (("run_id", run_id), ("revision", revision)):
            if isinstance(number, bool) or not isinstance(number, int) or number < 1:
                raise ValueError(f"{name} must be a positive integer")
        if self.run_id is not None and run_id != self.run_id:
            raise ValueError("constraint run_id mismatch; restart for a new run")
        if revision < self.revision:
            return False
        if revision == self.revision:
            if dict(value) != self._snapshot:
                raise ValueError("conflicting constraint at the same revision")
            return False
        active = value.get("active")
        if not isinstance(active, bool):
            raise ValueError("active must be boolean; false means explicit AllClear")
        zone = None
        if active:
            frame = value.get("frame_id")
            if not isinstance(frame, str) or not frame.strip():
                raise ValueError("frame_id is required")
            center = value.get("center_ned")
            if not isinstance(center, Mapping):
                raise ValueError("center_ned must use the shared metric NED frame")
            radius = _number(value.get("radius_m"), "radius_m")
            clearance = _number(value.get("clearance_m"), "clearance_m")
            if radius <= 0 or clearance < 0:
                raise ValueError("radius must be positive and clearance nonnegative")
            zone = KeepOutCircle(_number(center.get("north"), "north"),
                                 _number(center.get("east"), "east"), radius, clearance, frame)
        self.run_id, self.revision = run_id, revision
        self.zone = zone
        self._snapshot = dict(value)
        return True

    def segment_block_reason(self, start, end, frame_id):
        if self.zone is None:
            return None
        if frame_id != self.zone.frame_id:
            return "KEEP_OUT_FRAME_MISMATCH"
        if self.zone.intersects(start, end):
            return "KEEP_OUT_REPLAN_REQUIRED"
        return None

    def to_dict(self):
        return {"run_id": self.run_id, "revision": self.revision,
                "active": self.zone is not None, "snapshot": self._snapshot}


__all__ = [
    "KeepOutCircle", "KeepOutDetour", "Task3NavigationConstraint",
    "Task3ResumePhase", "plan_keep_out_detour", "plan_keep_out_exit", "resume_phase",
]
