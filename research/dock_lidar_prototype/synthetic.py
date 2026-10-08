"""Synthetic U-shaped dock point-cloud generator for offline tests."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class SyntheticDockConfig:
    length_m: float = 4.0
    entrance_width_m: float = 2.0
    point_spacing_m: float = 0.08
    center_x: float = 0.0
    center_y: float = 0.0
    heading_rad: float = 0.0
    z_m: float = 0.0
    noise_std_m: float = 0.0
    dropout_probability: float = 0.0
    outlier_count: int = 0
    outlier_radius_m: float = 6.0
    seed: int = 11

    def __post_init__(self) -> None:
        if self.length_m <= 0 or self.entrance_width_m <= 0 or self.point_spacing_m <= 0:
            raise ValueError("dock dimensions and point spacing must be positive")
        if not 0 <= self.dropout_probability < 1:
            raise ValueError("dropout_probability must be in [0, 1)")
        if self.noise_std_m < 0 or self.outlier_count < 0 or self.outlier_radius_m <= 0:
            raise ValueError("noise/outlier parameters are invalid")


def _line(start: Tuple[float, float], end: Tuple[float, float], spacing: float) -> np.ndarray:
    a = np.asarray(start, dtype=float)
    b = np.asarray(end, dtype=float)
    count = max(2, int(math.ceil(np.linalg.norm(b - a) / spacing)) + 1)
    return np.linspace(a, b, count)


def generate_synthetic_dock_point_cloud(config: SyntheticDockConfig = SyntheticDockConfig()) -> np.ndarray:
    """Generate two side walls and a rear wall with one open entrance."""

    half_length = config.length_m / 2.0
    half_width = config.entrance_width_m / 2.0
    points = np.vstack((
        _line((-half_length, -half_width), (half_length, -half_width), config.point_spacing_m),
        _line((-half_length, half_width), (half_length, half_width), config.point_spacing_m),
        _line((half_length, -half_width), (half_length, half_width), config.point_spacing_m),
    ))
    rng = np.random.default_rng(config.seed)
    keep = rng.random(len(points)) >= config.dropout_probability
    points = points[keep]
    if config.noise_std_m:
        points = points + rng.normal(0.0, config.noise_std_m, points.shape)
    if config.outlier_count:
        angles = rng.uniform(-math.pi, math.pi, config.outlier_count)
        radii = rng.uniform(config.entrance_width_m, config.outlier_radius_m, config.outlier_count)
        outliers = np.column_stack((np.cos(angles) * radii, np.sin(angles) * radii))
        points = np.vstack((points, outliers))

    c = math.cos(config.heading_rad)
    s = math.sin(config.heading_rad)
    rotation = np.asarray(((c, -s), (s, c)))
    points = (points @ rotation.T) + np.asarray((config.center_x, config.center_y))
    z = np.full((len(points), 1), config.z_m, dtype=float)
    return np.hstack((points, z))


__all__ = ["SyntheticDockConfig", "generate_synthetic_dock_point_cloud"]
