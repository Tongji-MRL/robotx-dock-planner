"""ROS-independent reactive obstacle detector for surface vehicles.

The core deliberately does not classify obstacle types.  Any stable LaserScan
cluster intersecting the vessel's forward swept corridor is an obstacle.  A
mission wrapper may disable this generic layer after a verified dock entrance
has been selected, because dock walls are then expected geometry handled by the
terminal docking controller.
"""

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real
from typing import Iterable, Optional, Sequence


class AvoidanceAction(str, Enum):
    PROCEED = "PROCEED"
    HOLD = "HOLD"
    REPLAN = "REPLAN"


class SteeringSide(str, Enum):
    LEFT = "LEFT"
    RIGHT = "RIGHT"


@dataclass(frozen=True)
class ReactiveAvoidanceConfig:
    stop_distance_m: float = 0.8
    replan_distance_m: float = 2.5
    release_distance_m: float = 3.0
    corridor_half_width_m: float = 0.9
    side_fov_rad: float = math.radians(100.0)
    center_exclusion_rad: float = math.radians(10.0)
    min_cluster_points: int = 3
    max_cluster_range_jump_m: float = 0.45
    trigger_frames: int = 2
    release_frames: int = 3
    decision_validity_s: float = 0.35

    def __post_init__(self) -> None:
        for name in (
            "stop_distance_m", "replan_distance_m", "release_distance_m",
            "corridor_half_width_m", "side_fov_rad", "center_exclusion_rad",
            "max_cluster_range_jump_m", "decision_validity_s",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            if value <= 0:
                raise ValueError(f"{name} must be positive")
        if not self.stop_distance_m < self.replan_distance_m < self.release_distance_m:
            raise ValueError("distance thresholds must satisfy stop < replan < release")
        if self.center_exclusion_rad >= self.side_fov_rad:
            raise ValueError("center_exclusion_rad must be smaller than side_fov_rad")
        for name in ("min_cluster_points", "trigger_frames", "release_frames"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")


@dataclass(frozen=True)
class ReactiveAvoidanceResult:
    action: AvoidanceAction
    timestamp: float
    valid_until: float
    reason: str
    nearest_forward_m: Optional[float] = None
    steering_side: Optional[SteeringSide] = None
    speed_scale: float = 1.0
    obstacle_points: int = 0

    def __post_init__(self) -> None:
        if not isinstance(self.action, AvoidanceAction):
            raise TypeError("action must be AvoidanceAction")
        if not math.isfinite(self.timestamp) or not math.isfinite(self.valid_until):
            raise ValueError("timestamps must be finite")
        if self.valid_until < self.timestamp:
            raise ValueError("valid_until cannot precede timestamp")
        if not 0.0 <= self.speed_scale <= 1.0:
            raise ValueError("speed_scale must be in [0, 1]")


class ReactiveAvoidanceCore:
    """Debounced corridor collision detector with left/right gap selection."""

    def __init__(self, config: ReactiveAvoidanceConfig = ReactiveAvoidanceConfig()) -> None:
        self.config = config
        self._blocked_frames = 0
        self._clear_frames = 0
        self._latched = False
        self._locked_side: Optional[SteeringSide] = None

    def reset(self) -> None:
        self._blocked_frames = 0
        self._clear_frames = 0
        self._latched = False
        self._locked_side = None

    def evaluate(
        self,
        ranges: Sequence[float],
        *,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        timestamp: float,
        terminal_docking: bool = False,
    ) -> ReactiveAvoidanceResult:
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        if terminal_docking:
            self.reset()
            return self._result(
                AvoidanceAction.PROCEED, timestamp,
                "terminal docking owns collision handling; generic avoidance bypassed",
            )
        if not ranges or not math.isfinite(angle_min) or not math.isfinite(angle_increment):
            return self._result(AvoidanceAction.HOLD, timestamp, "invalid or empty scan")

        points = self._valid_points(
            ranges, angle_min, angle_increment, range_min, range_max,
        )
        corridor = [
            item for item in points
            if item[1] > 0.0
            and abs(item[2]) <= self.config.corridor_half_width_m
            and item[1] <= self.config.release_distance_m
        ]
        clusters = self._clusters(corridor)
        obstacle = min(
            (item for cluster in clusters for item in cluster),
            key=lambda item: item[0],
            default=None,
        )
        blocked = obstacle is not None and obstacle[0] <= self.config.replan_distance_m
        if blocked:
            self._blocked_frames += 1
            self._clear_frames = 0
        else:
            self._blocked_frames = 0
            self._clear_frames += 1

        if not self._latched and self._blocked_frames >= self.config.trigger_frames:
            self._latched = True
        if self._latched and self._clear_frames >= self.config.release_frames:
            self.reset()

        if not self._latched:
            reason = "clear forward corridor"
            if blocked:
                reason = "candidate obstacle awaiting consecutive-frame confirmation"
            return self._result(
                AvoidanceAction.PROCEED, timestamp, reason,
                nearest=(obstacle[0] if obstacle else None),
                obstacle_points=sum(map(len, clusters)),
            )

        nearest = obstacle[0] if obstacle is not None else self.config.release_distance_m
        if nearest <= self.config.stop_distance_m:
            return self._result(
                AvoidanceAction.HOLD, timestamp,
                "confirmed obstacle inside emergency stop distance",
                nearest=nearest, speed_scale=0.0,
                obstacle_points=sum(map(len, clusters)),
            )
        if self._locked_side is None:
            self._locked_side = self._choose_side(points)
        scale = (nearest - self.config.stop_distance_m) / (
            self.config.replan_distance_m - self.config.stop_distance_m
        )
        return self._result(
            AvoidanceAction.REPLAN, timestamp,
            "confirmed obstacle intersects forward swept corridor",
            nearest=nearest, side=self._locked_side,
            speed_scale=max(0.15, min(0.65, scale)),
            obstacle_points=sum(map(len, clusters)),
        )

    def _valid_points(
        self,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
    ) -> list[tuple[float, float, float, float, int]]:
        lower = max(0.0, float(range_min)) if math.isfinite(range_min) else 0.0
        upper = float(range_max) if math.isfinite(range_max) else math.inf
        points = []
        for index, raw in enumerate(ranges):
            try:
                distance = float(raw)
            except (TypeError, ValueError, OverflowError):
                continue
            if not math.isfinite(distance) or distance < lower or distance > upper:
                continue
            angle = angle_min + index * angle_increment
            if abs(angle) > self.config.side_fov_rad:
                continue
            points.append((distance, distance * math.cos(angle), distance * math.sin(angle), angle, index))
        return points

    def _clusters(
        self, points: Iterable[tuple[float, float, float, float, int]]
    ) -> list[list[tuple[float, float, float, float, int]]]:
        ordered = sorted(points, key=lambda item: item[4])
        clusters: list[list[tuple[float, float, float, float, int]]] = []
        current: list[tuple[float, float, float, float, int]] = []
        for point in ordered:
            if current and (
                point[4] != current[-1][4] + 1
                or abs(point[0] - current[-1][0]) > self.config.max_cluster_range_jump_m
            ):
                if len(current) >= self.config.min_cluster_points:
                    clusters.append(current)
                current = []
            current.append(point)
        if len(current) >= self.config.min_cluster_points:
            clusters.append(current)
        return clusters

    def _choose_side(
        self, points: Sequence[tuple[float, float, float, float, int]]
    ) -> SteeringSide:
        left = [p[0] for p in points if self.config.center_exclusion_rad <= p[3] <= self.config.side_fov_rad]
        right = [p[0] for p in points if -self.config.side_fov_rad <= p[3] <= -self.config.center_exclusion_rad]
        left_clearance = self._robust_clearance(left)
        right_clearance = self._robust_clearance(right)
        return SteeringSide.LEFT if left_clearance >= right_clearance else SteeringSide.RIGHT

    def _robust_clearance(self, values: Sequence[float]) -> float:
        if not values:
            return self.config.release_distance_m
        ordered = sorted(min(value, self.config.release_distance_m) for value in values)
        return ordered[len(ordered) // 2]

    def _result(
        self,
        action: AvoidanceAction,
        timestamp: float,
        reason: str,
        *,
        nearest: Optional[float] = None,
        side: Optional[SteeringSide] = None,
        speed_scale: float = 1.0,
        obstacle_points: int = 0,
    ) -> ReactiveAvoidanceResult:
        return ReactiveAvoidanceResult(
            action=action,
            timestamp=timestamp,
            valid_until=timestamp + self.config.decision_validity_s,
            reason=reason,
            nearest_forward_m=nearest,
            steering_side=side,
            speed_scale=speed_scale,
            obstacle_points=obstacle_points,
        )
