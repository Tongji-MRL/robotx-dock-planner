"""NumPy-only prototype for close-range dock entrance candidates.

The detector is deliberately independent of ROS, TF, Mid-360 drivers and
colour perception.  It consumes points already expressed in the common NED
frame and returns the existing :class:`DockEntranceObservationNED` contract.

This is a geometry prototype, not a production dock detector.  Its structure
pipeline is:

``finite/ROI/range filter -> optional ground-plane RANSAC -> 2-D clusters
-> PCA OBBs -> paired parallel structures -> opening gap``

The pair of long structures is treated as the sides of one entrance.  The
coarse dock centre is used only to choose the inward sign of the heading; it
is not used to infer an entrance position directly.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .integration import (
    CoarseDockCenterObservationNED,
    CoarseDockInfo,
    CoarseDockObservation,
    DockEntranceObservationNED,
    DockPerceptionResult,
)


def _finite(value: float) -> bool:
    return math.isfinite(float(value))


def _clip01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


@dataclass(frozen=True)
class DockPerceptionConfig:
    """All prototype thresholds and policies in one place."""

    range_min_m: float = 0.25
    range_max_m: float = 40.0
    roi_radius_m: float = 25.0
    down_min_m: float = -5.0
    down_max_m: float = 5.0
    cluster_tolerance_m: float = 0.75
    min_cluster_points: int = 12
    max_cluster_points: int = 100000
    min_structure_length_m: float = 1.5
    max_structure_length_m: float = 30.0
    max_structure_width_m: float = 3.0
    max_pair_depth_offset_m: float = 2.0
    max_parallel_error_rad: float = 0.35
    min_gap_m: float = 1.0
    max_gap_m: float = 10.0
    min_inward_sign_distance_m: float = 0.25
    allow_ambiguous_heading: bool = True
    ambiguous_heading_confidence_scale: float = 0.55
    min_confidence: float = 0.20
    confidence_parallel_weight: float = 0.30
    confidence_gap_weight: float = 0.35
    confidence_points_weight: float = 0.35
    point_count_saturation: int = 80
    ransac_enabled: bool = True
    ransac_iterations: int = 128
    ransac_distance_threshold_m: float = 0.08
    ransac_min_inliers: int = 40
    ransac_min_abs_down_normal: float = 0.85
    random_seed: int = 7
    range_origin_ned: Optional[Tuple[float, float, float]] = None
    source: str = "numpy_dock_perception_v1"
    default_timestamp: float = 0.0

    def __post_init__(self) -> None:
        positive = (
            ("range_max_m", self.range_max_m),
            ("roi_radius_m", self.roi_radius_m),
            ("cluster_tolerance_m", self.cluster_tolerance_m),
            ("min_structure_length_m", self.min_structure_length_m),
            ("max_structure_length_m", self.max_structure_length_m),
            ("max_structure_width_m", self.max_structure_width_m),
            ("min_gap_m", self.min_gap_m),
            ("max_gap_m", self.max_gap_m),
            ("point_count_saturation", self.point_count_saturation),
        )
        for name, value in positive:
            if not _finite(value) or float(value) <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if not _finite(self.range_min_m) or self.range_min_m < 0:
            raise ValueError("range_min_m must be finite and nonnegative")
        if self.range_min_m >= self.range_max_m:
            raise ValueError("range_min_m must be smaller than range_max_m")
        if not _finite(self.down_min_m) or not _finite(self.down_max_m):
            raise ValueError("down limits must be finite")
        if self.down_min_m >= self.down_max_m:
            raise ValueError("down_min_m must be smaller than down_max_m")
        if self.min_cluster_points < 3 or self.max_cluster_points < self.min_cluster_points:
            raise ValueError("cluster point limits are invalid")
        if self.max_structure_length_m < self.min_structure_length_m:
            raise ValueError("structure length limits are invalid")
        if self.max_gap_m < self.min_gap_m:
            raise ValueError("gap limits are invalid")
        for name, value in (
            ("max_parallel_error_rad", self.max_parallel_error_rad),
            ("min_inward_sign_distance_m", self.min_inward_sign_distance_m),
            ("ransac_distance_threshold_m", self.ransac_distance_threshold_m),
            ("ransac_min_abs_down_normal", self.ransac_min_abs_down_normal),
            ("default_timestamp", self.default_timestamp),
        ):
            if not _finite(value) or float(value) < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if not 0 <= self.min_confidence <= 1:
            raise ValueError("min_confidence must be in [0, 1]")
        if not 0 < self.ambiguous_heading_confidence_scale <= 1:
            raise ValueError("ambiguous_heading_confidence_scale must be in (0, 1]")
        if self.ransac_iterations < 0 or self.ransac_min_inliers < 3:
            raise ValueError("RANSAC limits are invalid")
        if self.ransac_min_abs_down_normal > 1:
            raise ValueError("ransac_min_abs_down_normal must be <= 1")
        weights = (
            self.confidence_parallel_weight,
            self.confidence_gap_weight,
            self.confidence_points_weight,
        )
        if any(not _finite(v) or v < 0 for v in weights) or sum(weights) <= 0:
            raise ValueError("confidence weights must be nonnegative and nonzero")
        if self.range_origin_ned is not None and (
            len(self.range_origin_ned) != 3
            or not all(_finite(v) for v in self.range_origin_ned)
        ):
            raise ValueError("range_origin_ned must contain three finite values")
        if not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("source must be a nonempty string")
        if not _finite(self.default_timestamp):
            raise ValueError("default_timestamp must be finite")


@dataclass(frozen=True)
class OrientedBoundingBox2D:
    """PCA OBB for one horizontal structure cluster."""

    center_north_m: float
    center_east_m: float
    major_axis_north: float
    major_axis_east: float
    major_extent_m: float
    minor_extent_m: float
    down_min_m: float
    down_max_m: float
    point_count: int

    @property
    def center(self) -> np.ndarray:
        return np.array([self.center_north_m, self.center_east_m], dtype=float)

    @property
    def major_axis(self) -> np.ndarray:
        return np.array([self.major_axis_north, self.major_axis_east], dtype=float)


@dataclass(frozen=True)
class DockOpeningCandidate:
    """Geometry of one paired-structure opening before contract conversion."""

    entrance_center_north_m: float
    entrance_center_east_m: float
    heading_rad: float
    gap_width_m: float
    confidence: float
    structure_indices: Tuple[int, int]


@dataclass(frozen=True)
class DockPerceptionDebugResult:
    """Debuggable result while the public protocol remains a sequence."""

    filtered_points: np.ndarray
    structures: Tuple[OrientedBoundingBox2D, ...]
    openings: Tuple[DockOpeningCandidate, ...]
    observations: Tuple[DockEntranceObservationNED, ...]


@dataclass(frozen=True)
class SyntheticDockConfig:
    """Small deterministic dock-shaped cloud generator for offline tests."""

    center_north_m: float = 0.0
    center_east_m: float = 0.0
    heading_rad: float = 0.0
    opening_width_m: float = 3.0
    depth_m: float = 6.0
    structure_width_m: float = 0.45
    height_m: float = 1.5
    points_along: int = 24
    points_across: int = 3
    points_vertical: int = 5
    noise_std_m: float = 0.0
    dropout_probability: float = 0.0
    occlusion_depth_fraction: Optional[float] = None
    occlusion_side: int = 0
    seed: int = 11
    include_intensity: bool = False

    def __post_init__(self) -> None:
        for name, value in (
            ("opening_width_m", self.opening_width_m),
            ("depth_m", self.depth_m),
            ("structure_width_m", self.structure_width_m),
            ("height_m", self.height_m),
        ):
            if not _finite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if min(self.points_along, self.points_across, self.points_vertical) < 2:
            raise ValueError("synthetic point counts must be at least two")
        if not _finite(self.heading_rad) or self.noise_std_m < 0:
            raise ValueError("heading and noise settings are invalid")
        if not 0 <= self.dropout_probability < 1:
            raise ValueError("dropout_probability must be in [0, 1)")
        if self.occlusion_depth_fraction is not None and not 0 < self.occlusion_depth_fraction <= 1:
            raise ValueError("occlusion_depth_fraction must be in (0, 1]")
        if self.occlusion_side not in (0, 1):
            raise ValueError("occlusion_side must be 0 or 1")


def _as_xyz(point_cloud: Sequence[Sequence[float]]) -> np.ndarray:
    points = np.asarray(point_cloud, dtype=float)
    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError("point_cloud must have shape Nx3 or Nx4")
    return points[:, :3]


def filter_point_cloud(
    point_cloud: Sequence[Sequence[float]],
    coarse_dock: CoarseDockObservation,
    config: DockPerceptionConfig,
) -> np.ndarray:
    """Return finite points inside the configured range, down and ROI gates."""

    if not isinstance(coarse_dock, (CoarseDockInfo, CoarseDockCenterObservationNED)):
        raise TypeError("coarse_dock must be a coarse dock observation")
    if not coarse_dock.valid:
        return np.empty((0, 3), dtype=float)
    points = _as_xyz(point_cloud)
    finite = np.isfinite(points).all(axis=1)
    if not finite.any():
        return np.empty((0, 3), dtype=float)
    points = points[finite]
    center = np.array(
        [coarse_dock.center_ned.north, coarse_dock.center_ned.east,
         coarse_dock.center_ned.down],
        dtype=float,
    )
    origin = np.array(config.range_origin_ned if config.range_origin_ned is not None else center)
    relative_to_origin = points - origin
    ranges = np.hypot(relative_to_origin[:, 0], relative_to_origin[:, 1])
    relative_to_dock = points - center
    roi = np.hypot(relative_to_dock[:, 0], relative_to_dock[:, 1])
    keep = (
        (ranges >= config.range_min_m)
        & (ranges <= config.range_max_m)
        & (roi <= config.roi_radius_m)
        & (points[:, 2] >= config.down_min_m)
        & (points[:, 2] <= config.down_max_m)
    )
    return points[keep]


def _remove_ground_plane(points: np.ndarray, config: DockPerceptionConfig) -> np.ndarray:
    if not config.ransac_enabled or config.ransac_iterations == 0:
        return points
    if len(points) < max(3, config.ransac_min_inliers):
        return points
    rng = np.random.default_rng(config.random_seed)
    best_inliers: Optional[np.ndarray] = None
    best_count = 0
    for _ in range(config.ransac_iterations):
        sample = points[rng.choice(len(points), size=3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = float(np.linalg.norm(normal))
        if norm <= 1e-9:
            continue
        normal /= norm
        if abs(float(normal[2])) < config.ransac_min_abs_down_normal:
            continue
        distances = np.abs((points - sample[0]) @ normal)
        inliers = distances <= config.ransac_distance_threshold_m
        count = int(inliers.sum())
        if count > best_count:
            best_count = count
            best_inliers = inliers
    if best_inliers is None or best_count < config.ransac_min_inliers:
        return points
    return points[~best_inliers]


def _cluster_indices(points: np.ndarray, config: DockPerceptionConfig) -> List[np.ndarray]:
    if len(points) == 0:
        return []
    cell = config.cluster_tolerance_m
    xy = points[:, :2]
    grid = {}
    for index, point in enumerate(xy):
        key = (int(math.floor(point[0] / cell)), int(math.floor(point[1] / cell)))
        grid.setdefault(key, []).append(index)
    seen = np.zeros(len(points), dtype=bool)
    clusters: List[np.ndarray] = []
    for start in range(len(points)):
        if seen[start]:
            continue
        stack = [start]
        seen[start] = True
        component: List[int] = []
        while stack:
            current = stack.pop()
            component.append(current)
            cx, cy = grid_key = (
                int(math.floor(xy[current, 0] / cell)),
                int(math.floor(xy[current, 1] / cell)),
            )
            for gx in (cx - 1, cx, cx + 1):
                for gy in (cy - 1, cy, cy + 1):
                    for candidate in grid.get((gx, gy), ()):
                        if seen[candidate]:
                            continue
                        if np.linalg.norm(xy[candidate] - xy[current]) <= cell:
                            seen[candidate] = True
                            stack.append(candidate)
        if config.min_cluster_points <= len(component) <= config.max_cluster_points:
            clusters.append(np.asarray(component, dtype=int))
    return clusters


def _make_obb(points: np.ndarray) -> OrientedBoundingBox2D:
    xy = points[:, :2]
    mean = xy.mean(axis=0)
    centered = xy - mean
    if len(xy) == 1:
        major = np.array([1.0, 0.0])
    else:
        covariance = np.cov(centered, rowvar=False)
        values, vectors = np.linalg.eigh(np.atleast_2d(covariance))
        major = vectors[:, int(np.argmax(values))]
        norm = np.linalg.norm(major)
        major = major / norm if norm > 1e-9 else np.array([1.0, 0.0])
    minor = np.array([-major[1], major[0]])
    major_values = centered @ major
    minor_values = centered @ minor
    major_min, major_max = float(major_values.min()), float(major_values.max())
    minor_min, minor_max = float(minor_values.min()), float(minor_values.max())
    obb_center = mean + major * ((major_min + major_max) / 2.0) + minor * ((minor_min + minor_max) / 2.0)
    return OrientedBoundingBox2D(
        center_north_m=float(obb_center[0]),
        center_east_m=float(obb_center[1]),
        major_axis_north=float(major[0]),
        major_axis_east=float(major[1]),
        major_extent_m=major_max - major_min,
        minor_extent_m=minor_max - minor_min,
        down_min_m=float(points[:, 2].min()),
        down_max_m=float(points[:, 2].max()),
        point_count=len(points),
    )


def _angle_between_axes(a: np.ndarray, b: np.ndarray) -> float:
    cosine = abs(float(np.dot(a, b))) / max(float(np.linalg.norm(a) * np.linalg.norm(b)), 1e-9)
    return math.acos(max(-1.0, min(1.0, cosine)))


class NumpyDockPerception:
    """Concrete implementation of the existing ROS-independent protocol."""

    def __init__(self, config: DockPerceptionConfig = DockPerceptionConfig()) -> None:
        self.config = config

    def detect_with_debug(
        self,
        point_cloud: Sequence[Sequence[float]],
        coarse_dock: CoarseDockObservation,
        *,
        timestamp: Optional[float] = None,
    ) -> DockPerceptionDebugResult:
        if not isinstance(coarse_dock, (CoarseDockInfo, CoarseDockCenterObservationNED)):
            raise TypeError("coarse_dock must be a coarse dock observation")
        if not coarse_dock.valid:
            return DockPerceptionDebugResult(
                np.empty((0, 3)), (), (), (),
            )
        if timestamp is None:
            timestamp = self.config.default_timestamp
        if not _finite(timestamp):
            raise ValueError("timestamp must be finite")
        filtered = filter_point_cloud(point_cloud, coarse_dock, self.config)
        filtered = _remove_ground_plane(filtered, self.config)
        clusters = _cluster_indices(filtered, self.config)
        structures = []
        for indices in clusters:
            obb = _make_obb(filtered[indices])
            if not (
                self.config.min_structure_length_m <= obb.major_extent_m <= self.config.max_structure_length_m
                and obb.minor_extent_m <= self.config.max_structure_width_m
            ):
                continue
            structures.append(obb)
        structures_tuple = tuple(structures)
        openings = self._detect_openings(structures_tuple, coarse_dock)
        observations = tuple(
            DockEntranceObservationNED(
                entrance_center_north_m=item.entrance_center_north_m,
                entrance_center_east_m=item.entrance_center_east_m,
                entrance_heading_rad=item.heading_rad,
                confidence=item.confidence,
                timestamp=float(timestamp),
                source=self.config.source,
                valid=True,
                dock_id=coarse_dock.dock_id,
            )
            for item in openings
        )
        return DockPerceptionDebugResult(filtered, structures_tuple, openings, observations)

    def detect(
        self,
        point_cloud: Sequence[Sequence[float]],
        coarse_dock: CoarseDockObservation,
        *,
        timestamp: Optional[float] = None,
    ) -> DockPerceptionResult:
        """Return a timestamped integration result; timestamp is sensor time."""

        debug = self.detect_with_debug(point_cloud, coarse_dock, timestamp=timestamp)
        resolved_timestamp = self.config.default_timestamp if timestamp is None else float(timestamp)
        return DockPerceptionResult(
            dock_id=coarse_dock.dock_id,
            entrance_observations=debug.observations,
            timestamp=resolved_timestamp,
            source=self.config.source,
            valid=coarse_dock.valid,
        )

    def _detect_openings(
        self,
        structures: Tuple[OrientedBoundingBox2D, ...],
        coarse_dock: CoarseDockObservation,
    ) -> Tuple[DockOpeningCandidate, ...]:
        coarse = np.array([coarse_dock.center_ned.north, coarse_dock.center_ned.east], dtype=float)
        candidates: List[DockOpeningCandidate] = []
        weight_sum = (
            self.config.confidence_parallel_weight
            + self.config.confidence_gap_weight
            + self.config.confidence_points_weight
        )
        for first in range(len(structures)):
            for second in range(first + 1, len(structures)):
                left, right = structures[first], structures[second]
                axis_left = left.major_axis
                axis_right = right.major_axis
                if float(np.dot(axis_left, axis_right)) < 0:
                    axis_right = -axis_right
                axis = axis_left + axis_right
                axis_norm = float(np.linalg.norm(axis))
                if axis_norm <= 1e-9:
                    continue
                axis /= axis_norm
                parallel_error = _angle_between_axes(axis_left, axis_right)
                if parallel_error > self.config.max_parallel_error_rad:
                    continue
                delta = right.center - left.center
                depth_offset = abs(float(np.dot(delta, axis)))
                if depth_offset > self.config.max_pair_depth_offset_m:
                    continue
                cross_axis = np.array([-axis[1], axis[0]])
                separation = abs(float(np.dot(delta, cross_axis)))
                gap = separation - (left.minor_extent_m + right.minor_extent_m) / 2.0
                if not self.config.min_gap_m <= gap <= self.config.max_gap_m:
                    continue
                midpoint = (left.center + right.center) / 2.0
                inward_projection = float(np.dot(coarse - midpoint, axis))
                if (
                    abs(inward_projection) < self.config.min_inward_sign_distance_m
                    and not self.config.allow_ambiguous_heading
                ):
                    continue
                half_depth = (left.major_extent_m + right.major_extent_m) / 4.0
                gap_score = _clip01(
                    1.0 - abs(gap - (self.config.min_gap_m + self.config.max_gap_m) / 2.0)
                    / max((self.config.max_gap_m - self.config.min_gap_m) / 2.0, 1e-9)
                )
                parallel_score = _clip01(
                    1.0 - parallel_error / max(self.config.max_parallel_error_rad, 1e-9)
                )
                point_score = _clip01(
                    min(left.point_count, right.point_count) / self.config.point_count_saturation
                )
                confidence = _clip01(
                    (
                        self.config.confidence_parallel_weight * parallel_score
                        + self.config.confidence_gap_weight * gap_score
                        + self.config.confidence_points_weight * point_score
                    )
                    / weight_sum
                )
                heading_axes = (
                    (axis if inward_projection > 0 else -axis, 1.0),
                )
                if abs(inward_projection) < self.config.min_inward_sign_distance_m:
                    heading_axes = (
                        (axis, self.config.ambiguous_heading_confidence_scale),
                        (-axis, self.config.ambiguous_heading_confidence_scale),
                    )
                for heading_axis, confidence_scale in heading_axes:
                    candidate_confidence = confidence * confidence_scale
                    if candidate_confidence < self.config.min_confidence:
                        continue
                    entrance = midpoint - heading_axis * half_depth
                    candidates.append(
                        DockOpeningCandidate(
                            entrance_center_north_m=float(entrance[0]),
                            entrance_center_east_m=float(entrance[1]),
                            heading_rad=math.atan2(float(heading_axis[1]), float(heading_axis[0])),
                            gap_width_m=float(gap),
                            confidence=candidate_confidence,
                            structure_indices=(first, second),
                        )
                    )
        candidates.sort(key=lambda item: item.confidence, reverse=True)
        return tuple(candidates)


def generate_synthetic_dock_point_cloud(config: SyntheticDockConfig = SyntheticDockConfig()) -> np.ndarray:
    """Generate two elongated dock-side structures as an Nx3 or Nx4 array."""

    rng = np.random.default_rng(config.seed)
    heading = np.array([math.cos(config.heading_rad), math.sin(config.heading_rad)])
    cross = np.array([-heading[1], heading[0]])
    entrance = np.array([
        config.center_north_m,
        config.center_east_m,
    ]) - heading * (config.depth_m / 2.0)
    depth_values = np.linspace(0.0, config.depth_m, config.points_along)
    cross_values = np.linspace(
        -config.structure_width_m / 2.0,
        config.structure_width_m / 2.0,
        config.points_across,
    )
    down_values = np.linspace(0.0, config.height_m, config.points_vertical)
    points: List[np.ndarray] = []
    offsets = (
        -(config.opening_width_m + config.structure_width_m) / 2.0,
        +(config.opening_width_m + config.structure_width_m) / 2.0,
    )
    for side, side_offset in enumerate(offsets):
        for depth in depth_values:
            if (
                config.occlusion_depth_fraction is not None
                and side == config.occlusion_side
                and depth > config.depth_m * config.occlusion_depth_fraction
            ):
                continue
            for across in cross_values:
                for down in down_values:
                    local = heading * depth + cross * (side_offset + across)
                    points.append(
                        np.array([
                            entrance[0] + local[0],
                            entrance[1] + local[1],
                            down,
                        ])
                    )
    cloud = np.asarray(points, dtype=float)
    if config.dropout_probability > 0:
        keep = rng.random(len(cloud)) >= config.dropout_probability
        cloud = cloud[keep]
    if config.noise_std_m > 0 and len(cloud):
        cloud = cloud + rng.normal(0.0, config.noise_std_m, size=cloud.shape)
    if config.include_intensity:
        return np.column_stack([cloud, np.ones(len(cloud), dtype=float)])
    return cloud


__all__ = [
    "DockPerceptionConfig",
    "DockOpeningCandidate",
    "DockPerceptionDebugResult",
    "NumpyDockPerception",
    "OrientedBoundingBox2D",
    "SyntheticDockConfig",
    "filter_point_cloud",
    "generate_synthetic_dock_point_cloud",
]
