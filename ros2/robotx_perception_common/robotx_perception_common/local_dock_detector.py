"""Seeded local-frame detector for low-profile floating dock entrances.

The first field implementation clustered the whole horizontal point cloud and
then fitted one PCA box per cluster. Around water that is brittle: sparse
water returns bridge a dock to nearby clutter, while a low floating dock often
does not form one dense 3-D object. This detector uses a bounded height/forward
ROI, small XY clusters, RANSAC line segments, paired parallel edges, and empty
entrance/approach-corridor checks.

All coordinates are ROS ``base_link`` (x forward, y left, z up). A coarse dock
centre is used as a spatial prior and to orient the entrance heading; it is
never returned as a made-up detection.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np


def _line_angle_error(first: float, second: float) -> float:
    value = abs(math.atan2(math.sin(first - second), math.cos(first - second)))
    return min(value, math.pi - value)


@dataclass(frozen=True)
class LocalDockDetectorConfig:
    """Thresholds for the measured Mid-360 and low floating dock."""

    min_range_m: float = 0.75
    max_range_m: float = 10.0
    z_min_m: float = -0.35
    z_max_m: float = 0.85
    forward_margin_m: float = 2.5
    lateral_half_width_m: float = 3.0
    coarse_roi_radius_m: float = 6.0
    self_x_min_m: float = -0.70
    self_x_max_m: float = 0.85
    self_y_min_m: float = -0.55
    self_y_max_m: float = 0.55
    cluster_distance_m: float = 0.28
    min_cluster_points: int = 8
    line_distance_m: float = 0.10
    min_wall_length_m: float = 0.65
    min_line_inliers: int = 8
    ransac_iterations: int = 140
    max_segments_per_cluster: int = 8
    max_parallel_error_rad: float = 0.38
    max_wall_view_error_rad: float = 0.80
    max_front_view_error_rad: float = 0.45
    max_endpoint_depth_offset_m: float = 2.0
    min_entrance_before_center_m: float = 0.25
    min_gap_m: float = 0.75
    max_gap_m: float = 4.0
    gap_margin_m: float = 0.10
    max_gap_interior_points: int = 4
    approach_half_width_m: float = 0.24
    approach_start_m: float = 0.75
    max_approach_points: int = 5
    min_confidence: float = 0.30
    max_candidates: int = 6
    random_seed: int = 7

    def __post_init__(self) -> None:
        finite_values = (
            self.min_range_m, self.max_range_m, self.z_min_m, self.z_max_m,
            self.forward_margin_m, self.lateral_half_width_m,
            self.coarse_roi_radius_m, self.cluster_distance_m,
            self.line_distance_m, self.min_wall_length_m,
            self.max_parallel_error_rad, self.max_wall_view_error_rad,
            self.max_front_view_error_rad,
            self.max_endpoint_depth_offset_m, self.min_entrance_before_center_m,
            self.min_gap_m, self.max_gap_m,
            self.gap_margin_m, self.approach_half_width_m,
            self.approach_start_m, self.min_confidence,
        )
        if not all(math.isfinite(float(value)) for value in finite_values):
            raise ValueError("detector thresholds must be finite")
        if not 0 <= self.min_range_m < self.max_range_m:
            raise ValueError("range limits are invalid")
        if self.z_min_m >= self.z_max_m:
            raise ValueError("height limits are invalid")
        if not 0 < self.min_gap_m < self.max_gap_m:
            raise ValueError("gap limits are invalid")
        if self.cluster_distance_m <= 0 or self.line_distance_m <= 0:
            raise ValueError("clustering and line distances must be positive")
        if min(self.min_cluster_points, self.min_line_inliers) < 3:
            raise ValueError("point-count limits must be at least three")
        if self.ransac_iterations < 1 or self.max_segments_per_cluster < 1:
            raise ValueError("RANSAC limits must be positive")
        if not 0 <= self.min_confidence <= 1 or self.max_candidates < 1:
            raise ValueError("confidence/candidate limits are invalid")


@dataclass(frozen=True)
class LocalEntranceCandidate:
    center_x_m: float
    center_y_m: float
    heading_rad: float
    gap_m: float
    confidence: float


@dataclass(frozen=True)
class _Segment:
    start: np.ndarray
    end: np.ndarray
    angle_rad: float
    support: int

    @property
    def length_m(self) -> float:
        return float(np.linalg.norm(self.end - self.start))

    @property
    def direction(self) -> np.ndarray:
        delta = self.end - self.start
        return delta / max(float(np.linalg.norm(delta)), 1e-9)


@dataclass(frozen=True)
class LocalDetectionDebug:
    filtered_xyz: np.ndarray
    clusters: Tuple[np.ndarray, ...]
    segments: Tuple[_Segment, ...]
    candidates: Tuple[LocalEntranceCandidate, ...]


def _filter_points(
    points_xyz: Sequence[Sequence[float]],
    config: LocalDockDetectorConfig,
    coarse_center_xy: Optional[Tuple[float, float]],
) -> Tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
    points = np.asarray(points_xyz, dtype=float)
    if points.size == 0:
        return np.empty((0, 3), dtype=float), None, None
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_xyz must have shape (N, 3)")
    finite = np.isfinite(points).all(axis=1)
    nonzero = np.linalg.norm(points, axis=1) > 1e-6
    ranges = np.linalg.norm(points[:, :2], axis=1)
    self_return = (
        (points[:, 0] >= config.self_x_min_m)
        & (points[:, 0] <= config.self_x_max_m)
        & (points[:, 1] >= config.self_y_min_m)
        & (points[:, 1] <= config.self_y_max_m)
    )
    keep = (
        finite
        & nonzero
        & ~self_return
        & (ranges >= config.min_range_m)
        & (ranges <= config.max_range_m)
        & (points[:, 2] >= config.z_min_m)
        & (points[:, 2] <= config.z_max_m)
    )
    coarse = None
    view = None
    if coarse_center_xy is not None:
        coarse = np.asarray(coarse_center_xy, dtype=float)
        if coarse.shape != (2,) or not np.isfinite(coarse).all():
            raise ValueError("coarse_center_xy must contain two finite values")
        distance = float(np.linalg.norm(coarse))
        if distance <= config.min_range_m:
            return np.empty((0, 3), dtype=float), coarse, None
        view = coarse / distance
        cross = np.asarray((-view[1], view[0]), dtype=float)
        along = points[:, :2] @ view
        lateral = points[:, :2] @ cross
        roi_distance = np.linalg.norm(points[:, :2] - coarse, axis=1)
        keep &= (
            (along >= config.min_range_m)
            & (along <= distance + config.forward_margin_m)
            & (np.abs(lateral) <= config.lateral_half_width_m)
            & (roi_distance <= config.coarse_roi_radius_m)
        )
    return np.ascontiguousarray(points[keep]), coarse, view


def _clusters(points_xy: np.ndarray, config: LocalDockDetectorConfig) -> Tuple[np.ndarray, ...]:
    if len(points_xy) == 0:
        return ()
    cell = config.cluster_distance_m
    cells = np.floor(points_xy / cell).astype(np.int64)
    buckets = {}
    for index, item in enumerate(cells):
        buckets.setdefault((int(item[0]), int(item[1])), []).append(index)
    remaining = set(range(len(points_xy)))
    result: List[np.ndarray] = []
    while remaining:
        seed = remaining.pop()
        queue = [seed]
        members = [seed]
        while queue:
            current = queue.pop()
            cx, cy = cells[current]
            for gx in range(int(cx) - 1, int(cx) + 2):
                for gy in range(int(cy) - 1, int(cy) + 2):
                    for candidate in buckets.get((gx, gy), ()):
                        if candidate not in remaining:
                            continue
                        if np.linalg.norm(points_xy[candidate] - points_xy[current]) <= cell:
                            remaining.remove(candidate)
                            queue.append(candidate)
                            members.append(candidate)
        if len(members) >= config.min_cluster_points:
            result.append(points_xy[np.asarray(members, dtype=int)])
    result.sort(key=len, reverse=True)
    return tuple(result)


def _segment_from_points(points: np.ndarray) -> _Segment:
    mean = points.mean(axis=0)
    centered = points - mean
    _, vectors = np.linalg.eigh(centered.T @ centered)
    direction = vectors[:, -1]
    direction /= max(float(np.linalg.norm(direction)), 1e-9)
    projection = centered @ direction
    start = mean + direction * float(projection.min())
    end = mean + direction * float(projection.max())
    return _Segment(start, end, math.atan2(float(direction[1]), float(direction[0])), len(points))


def _extract_segments(
    clusters: Sequence[np.ndarray],
    config: LocalDockDetectorConfig,
) -> Tuple[_Segment, ...]:
    rng = np.random.default_rng(config.random_seed)
    segments: List[_Segment] = []
    for cluster in clusters:
        remaining = cluster.copy()
        for _ in range(config.max_segments_per_cluster):
            if len(remaining) < config.min_line_inliers:
                break
            best: Optional[np.ndarray] = None
            best_score = (-1.0, -1.0)
            for _ in range(config.ransac_iterations):
                pair = rng.choice(len(remaining), size=2, replace=False)
                p0, p1 = remaining[pair]
                direction = p1 - p0
                norm = float(np.linalg.norm(direction))
                if norm < config.min_wall_length_m * 0.20:
                    continue
                direction /= norm
                relative = remaining - p0
                distances = np.abs(relative[:, 0] * direction[1] - relative[:, 1] * direction[0])
                inliers = np.flatnonzero(distances <= config.line_distance_m)
                if len(inliers) < config.min_line_inliers:
                    continue
                projected = relative[inliers] @ direction
                length = float(projected.max() - projected.min())
                score = (length, float(len(inliers)))
                if score > best_score:
                    best_score = score
                    best = inliers
            if best is None:
                break
            segment = _segment_from_points(remaining[best])
            if segment.length_m >= config.min_wall_length_m:
                segments.append(segment)
            keep = np.ones(len(remaining), dtype=bool)
            keep[best] = False
            remaining = remaining[keep]
    segments.sort(key=lambda item: (item.length_m, item.support), reverse=True)
    return tuple(segments)


def _corridor_count(
    points: np.ndarray,
    start: np.ndarray,
    end: np.ndarray,
    half_width: float,
    start_margin: float,
    end_margin: float,
) -> int:
    vector = end - start
    length = float(np.linalg.norm(vector))
    if length <= start_margin + end_margin:
        return 0
    unit = vector / length
    relative = points - start
    projection = relative @ unit
    lateral = np.abs(relative[:, 0] * unit[1] - relative[:, 1] * unit[0])
    inside = (
        (projection > start_margin)
        & (projection < length - end_margin)
        & (lateral <= half_width)
    )
    return int(np.count_nonzero(inside))


def _candidate_side_pairs(
    points_xy: np.ndarray,
    segments: Sequence[_Segment],
    config: LocalDockDetectorConfig,
    coarse: Optional[np.ndarray],
    view: Optional[np.ndarray],
) -> Tuple[LocalEntranceCandidate, ...]:
    observer = np.zeros(2, dtype=float)
    coarse_distance = float(np.linalg.norm(coarse)) if coarse is not None else config.max_range_m
    result: List[LocalEntranceCandidate] = []
    for first_index, first in enumerate(segments):
        for second in segments[first_index + 1:]:
            parallel_error = _line_angle_error(first.angle_rad, second.angle_rad)
            if parallel_error > config.max_parallel_error_rad:
                continue
            first_direction = first.direction
            second_direction = second.direction
            if float(first_direction @ second_direction) < 0:
                second_direction = -second_direction
            wall_axis = first_direction + second_direction
            wall_axis /= max(float(np.linalg.norm(wall_axis)), 1e-9)
            if coarse is None:
                # Without a supplied centre the observer is on the water side.
                # Only the nearer end of each parallel finger can be an
                # entrance; pairing far ends creates a convincing but unsafe
                # false opening at the dock back wall.
                endpoint_pairs = ((
                    min((first.start, first.end), key=lambda item: float(np.linalg.norm(item))),
                    min((second.start, second.end), key=lambda item: float(np.linalg.norm(item))),
                ),)
            else:
                endpoint_pairs = tuple(
                    (endpoint_first, endpoint_second)
                    for endpoint_first in (first.start, first.end)
                    for endpoint_second in (second.start, second.end)
                )
            for endpoint_first, endpoint_second in endpoint_pairs:
                    gap_vector = endpoint_second - endpoint_first
                    gap = float(np.linalg.norm(gap_vector))
                    if not config.min_gap_m <= gap <= config.max_gap_m:
                        continue
                    gap_unit = gap_vector / gap
                    if abs(float(gap_unit @ wall_axis)) > math.sin(config.max_parallel_error_rad):
                        continue
                    center = (endpoint_first + endpoint_second) / 2.0
                    candidate_view = view
                    if candidate_view is None:
                        center_range = float(np.linalg.norm(center - observer))
                        if center_range <= config.min_range_m:
                            continue
                        candidate_view = (center - observer) / center_range
                    wall_view_error = math.acos(min(
                        1.0,
                        max(-1.0, abs(float(wall_axis @ candidate_view))),
                    ))
                    if wall_view_error > config.max_wall_view_error_rad:
                        continue
                    if abs(float(gap_vector @ candidate_view)) > config.max_endpoint_depth_offset_m:
                        continue
                    forward = float(center @ candidate_view)
                    cross = np.asarray((-candidate_view[1], candidate_view[0]), dtype=float)
                    lateral = abs(float(center @ cross))
                    before_center = coarse_distance - forward
                    if forward < config.min_range_m:
                        continue
                    if coarse is not None and before_center < config.min_entrance_before_center_m:
                        continue
                    if lateral > config.lateral_half_width_m:
                        continue
                    gap_points = _corridor_count(
                        points_xy, endpoint_first, endpoint_second,
                        config.line_distance_m, config.gap_margin_m, config.gap_margin_m,
                    )
                    if gap_points > config.max_gap_interior_points:
                        continue
                    approach_points = _corridor_count(
                        points_xy, observer, center,
                        config.approach_half_width_m, config.approach_start_m, config.gap_margin_m,
                    )
                    if approach_points > config.max_approach_points:
                        continue
                    heading_axis = wall_axis
                    target = coarse - center if coarse is not None else candidate_view
                    if float(heading_axis @ target) < 0:
                        heading_axis = -heading_axis
                    support_score = min(1.0, min(first.support, second.support) / 24.0)
                    parallel_score = 1.0 - parallel_error / config.max_parallel_error_rad
                    view_score = 1.0 - wall_view_error / config.max_wall_view_error_rad
                    empty_score = 1.0 - gap_points / max(1.0, config.max_gap_interior_points + 1.0)
                    approach_score = 1.0 - approach_points / max(1.0, config.max_approach_points + 1.0)
                    lateral_score = max(0.0, 1.0 - lateral / config.lateral_half_width_m)
                    external_score = (
                        min(1.0, max(0.0, before_center / max(coarse_distance, 1e-9)))
                        if coarse is not None else 0.5
                    )
                    confidence = (
                        0.18 * parallel_score + 0.15 * view_score + 0.12 * support_score
                        + 0.14 * empty_score + 0.13 * approach_score + 0.10 * lateral_score
                        + 0.18 * external_score
                    )
                    if confidence < config.min_confidence:
                        continue
                    result.append(LocalEntranceCandidate(
                        float(center[0]), float(center[1]),
                        math.atan2(float(heading_axis[1]), float(heading_axis[0])),
                        gap, float(min(1.0, max(0.0, confidence))),
                    ))
    result.sort(key=lambda item: item.confidence, reverse=True)
    kept: List[LocalEntranceCandidate] = []
    for candidate in result:
        if any(math.hypot(candidate.center_x_m - other.center_x_m,
                          candidate.center_y_m - other.center_y_m) < 0.5
               for other in kept):
            continue
        kept.append(candidate)
        if len(kept) >= config.max_candidates:
            break
    return tuple(kept)


def _candidate_front_gaps(
    points_xy: np.ndarray,
    segments: Sequence[_Segment],
    config: LocalDockDetectorConfig,
    coarse: Optional[np.ndarray],
    view: Optional[np.ndarray],
) -> Tuple[LocalEntranceCandidate, ...]:
    """Find an opening between two visible front-face line fragments.

    A low floating dock often gives weak grazing-angle returns along its side
    walls but strong returns from the transverse front faces.  The target bay
    then appears as two nearly collinear lateral segments with an empty gap.
    """

    observer = np.zeros(2, dtype=float)
    coarse_distance = float(np.linalg.norm(coarse)) if coarse is not None else config.max_range_m
    result: List[LocalEntranceCandidate] = []
    for first_index, first in enumerate(segments):
        for second in segments[first_index + 1:]:
            parallel_error = _line_angle_error(first.angle_rad, second.angle_rad)
            if parallel_error > config.max_parallel_error_rad:
                continue
            first_direction = first.direction
            second_direction = second.direction
            if float(first_direction @ second_direction) < 0.0:
                second_direction = -second_direction
            front_axis = first_direction + second_direction
            front_axis /= max(float(np.linalg.norm(front_axis)), 1e-9)
            first_mid = (first.start + first.end) / 2.0
            second_mid = (second.start + second.end) / 2.0
            candidate_view = view
            if candidate_view is None:
                pair_center = (first_mid + second_mid) / 2.0
                pair_range = float(np.linalg.norm(pair_center - observer))
                if pair_range <= config.min_range_m:
                    continue
                candidate_view = (pair_center - observer) / pair_range
            cross = np.asarray((-candidate_view[1], candidate_view[0]), dtype=float)
            coarse_lateral = float(coarse @ cross) if coarse is not None else 0.0
            front_error = abs(
                math.pi / 2.0
                - math.acos(min(
                    1.0,
                    max(-1.0, abs(float(front_axis @ candidate_view))),
                ))
            )
            if front_error > config.max_front_view_error_rad:
                continue
            first_side = float(first_mid @ cross) - coarse_lateral
            second_side = float(second_mid @ cross) - coarse_lateral
            if first_side * second_side >= 0.0:
                continue
            # Each segment contributes the endpoint facing the coarse-centre
            # lateral coordinate.  This rejects gaps between two outer ends.
            endpoint_first = min(
                (first.start, first.end),
                key=lambda item: abs(float(item @ cross) - coarse_lateral),
            )
            endpoint_second = min(
                (second.start, second.end),
                key=lambda item: abs(float(item @ cross) - coarse_lateral),
            )
            gap_vector = endpoint_second - endpoint_first
            gap = float(np.linalg.norm(gap_vector))
            if not config.min_gap_m <= gap <= config.max_gap_m:
                continue
            if abs(float(gap_vector @ candidate_view)) > config.max_endpoint_depth_offset_m:
                continue
            gap_unit = gap_vector / gap
            if abs(float(gap_unit @ candidate_view)) > math.sin(config.max_parallel_error_rad):
                continue
            center = (endpoint_first + endpoint_second) / 2.0
            forward = float(center @ candidate_view)
            lateral = abs(float(center @ cross) - coarse_lateral)
            before_center = coarse_distance - forward
            if forward < config.min_range_m:
                continue
            if coarse is not None and before_center < config.min_entrance_before_center_m:
                continue
            if lateral > config.lateral_half_width_m:
                continue
            gap_points = _corridor_count(
                points_xy, endpoint_first, endpoint_second,
                config.line_distance_m, config.gap_margin_m, config.gap_margin_m,
            )
            if gap_points > config.max_gap_interior_points:
                continue
            approach_points = _corridor_count(
                points_xy, observer, center,
                config.approach_half_width_m, config.approach_start_m,
                config.gap_margin_m,
            )
            if approach_points > config.max_approach_points:
                continue
            support_score = min(1.0, min(first.support, second.support) / 24.0)
            parallel_score = 1.0 - parallel_error / config.max_parallel_error_rad
            front_score = 1.0 - front_error / config.max_front_view_error_rad
            empty_score = 1.0 - gap_points / max(1.0, config.max_gap_interior_points + 1.0)
            approach_score = 1.0 - approach_points / max(1.0, config.max_approach_points + 1.0)
            lateral_score = max(0.0, 1.0 - lateral / config.lateral_half_width_m)
            external_score = (
                min(1.0, max(0.0, before_center / max(coarse_distance, 1e-9)))
                if coarse is not None else 0.5
            )
            confidence = (
                0.17 * parallel_score + 0.18 * front_score + 0.12 * support_score
                + 0.14 * empty_score + 0.13 * approach_score + 0.10 * lateral_score
                + 0.16 * external_score
            )
            if confidence < config.min_confidence:
                continue
            result.append(LocalEntranceCandidate(
                float(center[0]), float(center[1]),
                math.atan2(float(candidate_view[1]), float(candidate_view[0])),
                gap, float(min(1.0, max(0.0, confidence))),
            ))
    return tuple(result)


def _rank_and_deduplicate(
    candidates: Sequence[LocalEntranceCandidate],
    config: LocalDockDetectorConfig,
) -> Tuple[LocalEntranceCandidate, ...]:
    kept: List[LocalEntranceCandidate] = []
    for candidate in sorted(candidates, key=lambda item: item.confidence, reverse=True):
        if any(
            math.hypot(
                candidate.center_x_m - other.center_x_m,
                candidate.center_y_m - other.center_y_m,
            ) < 0.5
            for other in kept
        ):
            continue
        kept.append(candidate)
        if len(kept) >= config.max_candidates:
            break
    return tuple(kept)


def detect_local_entrances_debug(
    points_xyz: Sequence[Sequence[float]],
    *,
    coarse_center_xy: Optional[Tuple[float, float]] = None,
    config: LocalDockDetectorConfig = LocalDockDetectorConfig(),
) -> LocalDetectionDebug:
    """Return candidates and bounded intermediate geometry for diagnostics."""

    filtered, coarse, view = _filter_points(points_xyz, config, coarse_center_xy)
    clusters = _clusters(filtered[:, :2], config)
    segments = _extract_segments(clusters, config)
    side_candidates = _candidate_side_pairs(
        filtered[:, :2], segments, config, coarse, view,
    )
    front_candidates = _candidate_front_gaps(
        filtered[:, :2], segments, config, coarse, view,
    )
    candidates = _rank_and_deduplicate(
        side_candidates + front_candidates, config,
    )
    return LocalDetectionDebug(filtered, clusters, segments, candidates)


def detect_local_entrances(
    points_xyz: Sequence[Sequence[float]],
    *,
    coarse_center_xy: Optional[Tuple[float, float]] = None,
    min_confidence: Optional[float] = None,
    config: Optional[LocalDockDetectorConfig] = None,
) -> Tuple[LocalEntranceCandidate, ...]:
    """Return ranked entrance candidates in ``base_link`` coordinates."""

    resolved = config or LocalDockDetectorConfig()
    if min_confidence is not None and min_confidence != resolved.min_confidence:
        values = dict(resolved.__dict__)
        values["min_confidence"] = float(min_confidence)
        resolved = LocalDockDetectorConfig(**values)
    return detect_local_entrances_debug(
        points_xyz, coarse_center_xy=coarse_center_xy, config=resolved,
    ).candidates


__all__ = [
    "LocalDetectionDebug", "LocalDockDetectorConfig", "LocalEntranceCandidate",
    "detect_local_entrances", "detect_local_entrances_debug",
]
