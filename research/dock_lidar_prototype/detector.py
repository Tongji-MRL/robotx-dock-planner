"""Pure NumPy dock entrance geometry detection.

The prototype works in one already aligned local 2D frame.  It intentionally
does not know about ROS, TF, NED, vehicle motion, or a complete dock template.
The detector looks for two approximately parallel wall segments whose
connecting segment is wide enough and has little point support in its interior.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from typing import Optional, Sequence, Tuple

import numpy as np


def _finite(value: object) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _line_angle_difference(a: float, b: float) -> float:
    """Small angle between unoriented lines, in [0, pi/2]."""

    difference = abs(_wrap_pi(a - b)) % math.pi
    return min(difference, math.pi - difference)


@dataclass(frozen=True)
class DockLidarConfig:
    """Thresholds for the first offline point-cloud prototype."""

    z_min: float = -0.25
    z_max: float = 0.25
    roi_radius_m: float = 8.0
    min_entrance_width_m: float = 0.8
    max_entrance_width_m: float = 3.0
    cluster_distance_m: float = 0.28
    min_cluster_points: int = 8
    line_distance_threshold_m: float = 0.08
    min_wall_length_m: float = 1.0
    min_line_inliers: int = 12
    ransac_iterations: int = 300
    max_wall_segments: int = 8
    max_parallel_angle_rad: float = 0.28
    gap_endpoint_margin_m: float = 0.12
    max_gap_interior_points: int = 2
    min_entrance_before_center_m: float = 0.35
    approach_corridor_half_width_m: float = 0.18
    approach_corridor_start_margin_m: float = 0.5
    max_approach_corridor_points: int = 3
    random_seed: int = 7

    def __post_init__(self) -> None:
        for name in (
            "z_min", "z_max", "roi_radius_m", "min_entrance_width_m",
            "max_entrance_width_m", "cluster_distance_m",
            "line_distance_threshold_m", "min_wall_length_m",
            "max_parallel_angle_rad", "gap_endpoint_margin_m",
            "min_entrance_before_center_m", "approach_corridor_half_width_m",
            "approach_corridor_start_margin_m",
        ):
            value = getattr(self, name)
            if not _finite(value):
                raise ValueError(f"{name} must be finite")
        if self.z_min >= self.z_max:
            raise ValueError("z_min must be smaller than z_max")
        if self.roi_radius_m <= 0:
            raise ValueError("roi_radius_m must be positive")
        if not 0 < self.min_entrance_width_m <= self.max_entrance_width_m:
            raise ValueError("entrance width limits are invalid")
        if self.cluster_distance_m <= 0 or self.line_distance_threshold_m <= 0:
            raise ValueError("distance thresholds must be positive")
        if self.min_wall_length_m <= 0 or self.min_line_inliers < 2:
            raise ValueError("wall thresholds are invalid")
        if self.ransac_iterations < 1 or self.max_wall_segments < 2:
            raise ValueError("RANSAC/segment limits are invalid")
        if not 0 <= self.max_parallel_angle_rad < math.pi / 2:
            raise ValueError("max_parallel_angle_rad must be in [0, pi/2)")
        if self.gap_endpoint_margin_m < 0 or self.max_gap_interior_points < 0:
            raise ValueError("gap thresholds must be nonnegative")
        if self.min_entrance_before_center_m < 0:
            raise ValueError("min_entrance_before_center_m must be nonnegative")
        if self.approach_corridor_half_width_m <= 0 or self.approach_corridor_start_margin_m < 0:
            raise ValueError("approach corridor thresholds are invalid")
        if self.max_approach_corridor_points < 0:
            raise ValueError("max_approach_corridor_points must be nonnegative")


@dataclass(frozen=True)
class EntranceCandidate:
    center_x: float
    center_y: float
    heading_rad: float
    width_m: float
    confidence: float
    valid: bool

    def __post_init__(self) -> None:
        values = (self.center_x, self.center_y, self.heading_rad, self.width_m, self.confidence)
        if not all(_finite(value) for value in values):
            raise ValueError("entrance candidate values must be finite")
        if self.width_m <= 0 or not 0 <= self.confidence <= 1:
            raise ValueError("entrance width/confidence is invalid")
        if not isinstance(self.valid, bool):
            raise ValueError("valid must be boolean")


@dataclass(frozen=True)
class WallSegment:
    start: Tuple[float, float]
    end: Tuple[float, float]
    angle_rad: float
    support_points: int

    @property
    def length_m(self) -> float:
        return math.hypot(self.end[0] - self.start[0], self.end[1] - self.start[1])


@dataclass(frozen=True)
class DetectionDebug:
    """Intermediate values used by tests and the offline visualization."""

    filtered_xy: np.ndarray
    clusters: Tuple[np.ndarray, ...]
    wall_segments: Tuple[WallSegment, ...]
    candidates: Tuple[EntranceCandidate, ...]
    selected: Optional[EntranceCandidate]


def _filter_points(
    points_xyz: np.ndarray,
    coarse_center_xy: Tuple[float, float],
    config: DockLidarConfig,
) -> np.ndarray:
    points = np.asarray(points_xyz, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_xyz must have shape (N, 3)")
    center = np.asarray(coarse_center_xy, dtype=float)
    if center.shape != (2,) or not np.isfinite(center).all():
        raise ValueError("coarse_dock_center_xy must contain two finite values")
    finite = np.isfinite(points).all(axis=1)
    in_height = (points[:, 2] >= config.z_min) & (points[:, 2] <= config.z_max)
    delta = points[:, :2] - center
    in_roi = np.sum(delta * delta, axis=1) <= config.roi_radius_m ** 2
    return points[finite & in_height & in_roi, :2]


def _cluster_points(points: np.ndarray, config: DockLidarConfig) -> Tuple[np.ndarray, ...]:
    if len(points) == 0:
        return ()
    cell_size = config.cluster_distance_m
    cells = np.floor(points / cell_size).astype(np.int64)
    buckets = {}
    for index, cell in enumerate(cells):
        buckets.setdefault((int(cell[0]), int(cell[1])), []).append(index)

    remaining = set(range(len(points)))
    clusters = []
    while remaining:
        seed = remaining.pop()
        queue = [seed]
        members = [seed]
        while queue:
            current = queue.pop()
            cx, cy = cells[current]
            for nx in range(int(cx) - 1, int(cx) + 2):
                for ny in range(int(cy) - 1, int(cy) + 2):
                    for candidate in buckets.get((nx, ny), ()):
                        if candidate not in remaining:
                            continue
                        if np.linalg.norm(points[candidate] - points[current]) <= cell_size:
                            remaining.remove(candidate)
                            queue.append(candidate)
                            members.append(candidate)
        if len(members) >= config.min_cluster_points:
            clusters.append(points[np.asarray(members, dtype=int)])
    clusters.sort(key=len, reverse=True)
    return tuple(clusters)


def _segment_from_inliers(points: np.ndarray, inliers: np.ndarray) -> Optional[WallSegment]:
    if len(inliers) < 2:
        return None
    selected = points[inliers]
    mean = selected.mean(axis=0)
    centered = selected - mean
    covariance = centered.T @ centered
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    direction = eigenvectors[:, int(np.argmax(eigenvalues))]
    angle = math.atan2(float(direction[1]), float(direction[0]))
    projection = centered @ direction
    start = mean + direction * float(projection.min())
    end = mean + direction * float(projection.max())
    if np.linalg.norm(end - start) <= 0:
        return None
    if angle < 0:
        start, end = end, start
        angle = _wrap_pi(angle + math.pi)
    return WallSegment(
        start=(float(start[0]), float(start[1])),
        end=(float(end[0]), float(end[1])),
        angle_rad=_wrap_pi(angle),
        support_points=len(inliers),
    )


def _extract_wall_segments(cluster: np.ndarray, config: DockLidarConfig) -> Tuple[WallSegment, ...]:
    remaining = cluster.copy()
    segments = []
    rng = np.random.default_rng(config.random_seed)
    for _ in range(config.max_wall_segments):
        if len(remaining) < config.min_line_inliers:
            break
        best_inliers = None
        best_score = (-1, -1.0)
        for _ in range(config.ransac_iterations):
            pair = rng.choice(len(remaining), size=2, replace=False)
            p0, p1 = remaining[pair[0]], remaining[pair[1]]
            direction = p1 - p0
            length = float(np.linalg.norm(direction))
            if length < config.min_wall_length_m * 0.25:
                continue
            direction /= length
            distances = np.abs((remaining - p0)[:, 0] * direction[1] - (remaining - p0)[:, 1] * direction[0])
            inliers = np.flatnonzero(distances <= config.line_distance_threshold_m)
            if len(inliers) < config.min_line_inliers:
                continue
            projection = (remaining[inliers] - p0) @ direction
            line_length = float(projection.max() - projection.min())
            score = (len(inliers), line_length)
            if score > best_score:
                best_score = score
                best_inliers = inliers
        if best_inliers is None:
            break
        segment = _segment_from_inliers(remaining, best_inliers)
        if segment is None or segment.length_m < config.min_wall_length_m:
            break
        segments.append(segment)
        keep = np.ones(len(remaining), dtype=bool)
        keep[best_inliers] = False
        remaining = remaining[keep]
    return tuple(segments)


def _point_segment_interior_count(
    points: np.ndarray,
    a: np.ndarray,
    b: np.ndarray,
    config: DockLidarConfig,
) -> int:
    vector = b - a
    length = float(np.linalg.norm(vector))
    if length <= 0:
        return 0
    unit = vector / length
    relative = points - a
    projection = relative @ unit
    lateral = np.abs(relative[:, 0] * unit[1] - relative[:, 1] * unit[0])
    interior = (
        (projection > config.gap_endpoint_margin_m)
        & (projection < length - config.gap_endpoint_margin_m)
        & (lateral <= config.line_distance_threshold_m)
    )
    return int(np.count_nonzero(interior))


def _approach_corridor_count(
    points: np.ndarray,
    observer: np.ndarray,
    entrance: np.ndarray,
    config: DockLidarConfig,
) -> int:
    """Count structure points in the open corridor before an entrance."""

    vector = entrance - observer
    length = float(np.linalg.norm(vector))
    if length <= config.approach_corridor_start_margin_m + config.gap_endpoint_margin_m:
        return 0
    unit = vector / length
    relative = points - observer
    projection = relative @ unit
    lateral = np.abs(relative[:, 0] * unit[1] - relative[:, 1] * unit[0])
    inside = (
        (projection > config.approach_corridor_start_margin_m)
        & (projection < length - config.gap_endpoint_margin_m)
        & (lateral <= config.approach_corridor_half_width_m)
    )
    return int(np.count_nonzero(inside))


def _gap_candidates(
    points: np.ndarray,
    segments: Sequence[WallSegment],
    coarse_center_xy: Tuple[float, float],
    config: DockLidarConfig,
    observer_origin_xy: Optional[Tuple[float, float]] = None,
) -> Tuple[EntranceCandidate, ...]:
    center = np.asarray(coarse_center_xy, dtype=float)
    observer = None
    center_distance = None
    view_direction = None
    if observer_origin_xy is not None:
        observer = np.asarray(observer_origin_xy, dtype=float)
        if observer.shape != (2,) or not np.isfinite(observer).all():
            raise ValueError("observer_origin_xy must contain two finite values")
        center_vector = center - observer
        center_distance = float(np.linalg.norm(center_vector))
        if center_distance <= 1e-9:
            raise ValueError("observer and coarse dock center must be different")
        view_direction = center_vector / center_distance
    candidates = []
    for i, first in enumerate(segments):
        for second in segments[i + 1:]:
            if _line_angle_difference(first.angle_rad, second.angle_rad) > config.max_parallel_angle_rad:
                continue
            endpoints_first = (np.asarray(first.start), np.asarray(first.end))
            endpoints_second = (np.asarray(second.start), np.asarray(second.end))
            for a in endpoints_first:
                for b in endpoints_second:
                    width = float(np.linalg.norm(b - a))
                    if not config.min_entrance_width_m <= width <= config.max_entrance_width_m:
                        continue
                    interior_count = _point_segment_interior_count(points, a, b, config)
                    if interior_count > config.max_gap_interior_points:
                        continue
                    midpoint = (a + b) / 2.0
                    external_score = 1.0
                    corridor_score = 1.0
                    if observer is not None:
                        candidate_depth = float((midpoint - observer) @ view_direction)
                        before_center = center_distance - candidate_depth
                        if before_center < config.min_entrance_before_center_m:
                            continue
                        corridor_count = _approach_corridor_count(points, observer, midpoint, config)
                        if corridor_count > config.max_approach_corridor_points:
                            continue
                        external_score = min(1.0, before_center / max(config.min_entrance_before_center_m * 3.0, 1e-9))
                        corridor_score = 1.0 - min(
                            1.0,
                            corridor_count / max(1.0, config.max_approach_corridor_points + 1.0),
                        )
                    normal = np.asarray((- (b - a)[1], (b - a)[0]), dtype=float)
                    normal /= np.linalg.norm(normal)
                    to_center = center - midpoint
                    if float(normal @ to_center) < 0:
                        normal = -normal
                    if np.linalg.norm(to_center) <= 1e-9:
                        continue
                    alignment = max(0.0, min(1.0, float(normal @ (to_center / np.linalg.norm(to_center)))))
                    support = min(first.support_points, second.support_points)
                    support_score = min(1.0, support / max(1.0, config.min_line_inliers * 2.0))
                    empty_score = 1.0 - min(1.0, interior_count / max(1.0, config.max_gap_interior_points + 1.0))
                    if observer is None:
                        confidence = 0.45 * alignment + 0.35 * support_score + 0.20 * empty_score
                    else:
                        confidence = (
                            0.30 * alignment
                            + 0.25 * support_score
                            + 0.15 * empty_score
                            + 0.20 * external_score
                            + 0.10 * corridor_score
                        )
                    confidence = max(0.0, min(1.0, confidence))
                    candidates.append(EntranceCandidate(
                        center_x=float(midpoint[0]),
                        center_y=float(midpoint[1]),
                        heading_rad=float(math.atan2(normal[1], normal[0])),
                        width_m=width,
                        confidence=confidence,
                        valid=True,
                    ))
    candidates.sort(key=lambda candidate: candidate.confidence, reverse=True)
    return tuple(candidates)


def detect_entrance_debug(
    points_xyz: np.ndarray,
    coarse_dock_center_xy: Tuple[float, float],
    config: DockLidarConfig = DockLidarConfig(),
    observer_origin_xy: Optional[Tuple[float, float]] = None,
) -> DetectionDebug:
    """Run detection and return intermediate geometry for offline inspection."""

    points = _filter_points(points_xyz, coarse_dock_center_xy, config)
    clusters = _cluster_points(points, config)
    wall_segments = []
    for cluster in clusters:
        wall_segments.extend(_extract_wall_segments(cluster, config))
    wall_segments.sort(key=lambda segment: segment.support_points, reverse=True)
    candidates = _gap_candidates(
        points,
        wall_segments,
        coarse_dock_center_xy,
        config,
        observer_origin_xy,
    )
    return DetectionDebug(
        filtered_xy=points,
        clusters=clusters,
        wall_segments=tuple(wall_segments),
        candidates=candidates,
        selected=candidates[0] if candidates else None,
    )


def detect_entrance(
    points_xyz: np.ndarray,
    coarse_dock_center_xy: Tuple[float, float],
    config: DockLidarConfig = DockLidarConfig(),
    observer_origin_xy: Optional[Tuple[float, float]] = None,
) -> Optional[EntranceCandidate]:
    """Return the highest-scoring valid gap candidate, or ``None``."""

    return detect_entrance_debug(
        points_xyz,
        coarse_dock_center_xy,
        config,
        observer_origin_xy,
    ).selected


def detect_entrances(
    points_xyz: np.ndarray,
    coarse_dock_center_xy: Tuple[float, float],
    config: DockLidarConfig = DockLidarConfig(),
    observer_origin_xy: Optional[Tuple[float, float]] = None,
) -> Tuple[EntranceCandidate, ...]:
    """Return all valid candidates, sorted from highest to lowest confidence."""

    return detect_entrance_debug(
        points_xyz,
        coarse_dock_center_xy,
        config,
        observer_origin_xy,
    ).candidates


__all__ = [
    "DetectionDebug",
    "DockLidarConfig",
    "EntranceCandidate",
    "WallSegment",
    "detect_entrance",
    "detect_entrance_debug",
    "detect_entrances",
]
