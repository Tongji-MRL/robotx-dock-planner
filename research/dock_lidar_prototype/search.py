"""Unseeded offline search for dock entrance candidates.

The normal detector uses the UAV supplied dock center as a spatial prior.  This
module is deliberately separate: it searches a bounded local point cloud when
that prior is not available yet.  It never changes coordinates or invents a
TF/NED transform.  ``observer_origin_xy`` is simply the origin of the input
local frame (normally the LiDAR origin).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Optional, Sequence, Tuple

import numpy as np

from .detector import (
    DetectionDebug,
    DockLidarConfig,
    EntranceCandidate,
    detect_entrance_debug,
)


@dataclass(frozen=True)
class UnseededSearchConfig:
    """All thresholds for searching without a coarse dock center."""

    detector_config: DockLidarConfig = field(default_factory=DockLidarConfig)
    observer_origin_xy: Tuple[float, float] = (0.0, 0.0)
    min_range_m: float = 1.5
    max_range_m: float = 12.0
    grid_spacing_m: float = 2.0
    max_grid_centers: int = 96
    min_candidate_confidence: float = 0.35
    dedup_distance_m: float = 0.6
    max_candidates: int = 8

    def __post_init__(self) -> None:
        observer = np.asarray(self.observer_origin_xy, dtype=float)
        if observer.shape != (2,) or not np.isfinite(observer).all():
            raise ValueError("observer_origin_xy must contain two finite values")
        for name in (
            "min_range_m", "max_range_m", "grid_spacing_m",
            "min_candidate_confidence", "dedup_distance_m",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if self.min_range_m < 0 or self.max_range_m <= self.min_range_m:
            raise ValueError("range limits are invalid")
        if self.grid_spacing_m <= 0:
            raise ValueError("grid_spacing_m must be positive")
        if not 0 <= self.min_candidate_confidence <= 1:
            raise ValueError("min_candidate_confidence must be in [0, 1]")
        if self.dedup_distance_m <= 0:
            raise ValueError("dedup_distance_m must be positive")
        if self.max_grid_centers < 1 or self.max_candidates < 1:
            raise ValueError("candidate limits must be positive")


@dataclass(frozen=True)
class SearchDebug:
    """Machine-readable diagnostics for one unseeded search."""

    filtered_xyz: np.ndarray
    removed_near_points: int
    grid_centers: Tuple[Tuple[float, float], ...]
    raw_candidates: Tuple[EntranceCandidate, ...]
    candidates: Tuple[EntranceCandidate, ...]
    selected: Optional[EntranceCandidate]


def _filter_ranges(points_xyz: np.ndarray, config: UnseededSearchConfig) -> Tuple[np.ndarray, int]:
    points = np.asarray(points_xyz, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_xyz must have shape (N, 3)")
    finite = np.isfinite(points).all(axis=1)
    observer = np.asarray(config.observer_origin_xy, dtype=float)
    delta = points[:, :2] - observer
    ranges = np.linalg.norm(delta, axis=1)
    in_range = (ranges >= config.min_range_m) & (ranges <= config.max_range_m)
    kept = points[finite & in_range]
    removed_near = int(np.count_nonzero(finite & (ranges < config.min_range_m)))
    return kept, removed_near


def _grid_centers(points_xyz: np.ndarray, config: UnseededSearchConfig) -> Tuple[Tuple[float, float], ...]:
    """Create a deterministic bounded grid over the observed local XY area."""

    if len(points_xyz) == 0:
        return ()
    observer = np.asarray(config.observer_origin_xy, dtype=float)
    xy = points_xyz[:, :2]
    relative = xy - observer
    radius = np.linalg.norm(relative, axis=1)
    visible = relative[radius <= config.max_range_m]
    if len(visible) == 0:
        return ()
    low = np.floor(visible.min(axis=0) / config.grid_spacing_m) * config.grid_spacing_m
    high = np.ceil(visible.max(axis=0) / config.grid_spacing_m) * config.grid_spacing_m
    xs = np.arange(low[0], high[0] + config.grid_spacing_m * 0.5, config.grid_spacing_m)
    ys = np.arange(low[1], high[1] + config.grid_spacing_m * 0.5, config.grid_spacing_m)
    centers = []
    for x in xs:
        for y in ys:
            candidate = np.asarray((x, y), dtype=float)
            if np.linalg.norm(candidate) > config.max_range_m:
                continue
            if np.linalg.norm(candidate - observer) <= 1e-9:
                continue
            centers.append((float(candidate[0] + observer[0]), float(candidate[1] + observer[1])))
    if len(centers) <= config.max_grid_centers:
        return tuple(centers)

    # Preserve coverage over the whole area while keeping runtime bounded.
    indices = np.linspace(0, len(centers) - 1, config.max_grid_centers, dtype=int)
    return tuple(centers[int(index)] for index in np.unique(indices))


def _deduplicate(
    candidates: Sequence[EntranceCandidate],
    config: UnseededSearchConfig,
) -> Tuple[EntranceCandidate, ...]:
    ranked = sorted(candidates, key=lambda item: item.confidence, reverse=True)
    kept = []
    for candidate in ranked:
        point = np.asarray((candidate.center_x, candidate.center_y))
        if any(np.linalg.norm(point - np.asarray((other.center_x, other.center_y))) < config.dedup_distance_m
               for other in kept):
            continue
        kept.append(candidate)
        if len(kept) >= config.max_candidates:
            break
    return tuple(kept)


def search_entrances_debug(
    points_xyz: np.ndarray,
    config: UnseededSearchConfig = UnseededSearchConfig(),
) -> SearchDebug:
    """Search the local cloud for gap candidates without a supplied center."""

    filtered_xyz, removed_near = _filter_ranges(points_xyz, config)
    centers = _grid_centers(filtered_xyz, config)
    raw = []
    for center in centers:
        try:
            debug: DetectionDebug = detect_entrance_debug(
                filtered_xyz,
                center,
                config.detector_config,
                observer_origin_xy=config.observer_origin_xy,
            )
        except ValueError:
            continue
        raw.extend(candidate for candidate in debug.candidates
                   if candidate.confidence >= config.min_candidate_confidence)
    candidates = _deduplicate(raw, config)
    return SearchDebug(
        filtered_xyz=filtered_xyz,
        removed_near_points=removed_near,
        grid_centers=centers,
        raw_candidates=tuple(raw),
        candidates=candidates,
        selected=candidates[0] if candidates else None,
    )


def search_entrances(
    points_xyz: np.ndarray,
    config: UnseededSearchConfig = UnseededSearchConfig(),
) -> Tuple[EntranceCandidate, ...]:
    """Return deduplicated candidates, ranked by detector confidence."""

    return search_entrances_debug(points_xyz, config).candidates


def search_entrance(
    points_xyz: np.ndarray,
    config: UnseededSearchConfig = UnseededSearchConfig(),
) -> Optional[EntranceCandidate]:
    """Return the highest-scoring candidate, or ``None``."""

    return search_entrances_debug(points_xyz, config).selected


__all__ = [
    "SearchDebug",
    "UnseededSearchConfig",
    "search_entrance",
    "search_entrances",
    "search_entrances_debug",
]
