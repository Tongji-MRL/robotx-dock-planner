"""Pillow visualization for the offline detector."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Optional, Tuple

import numpy as np
from PIL import Image, ImageDraw

from .detector import DetectionDebug, DockLidarConfig, detect_entrance_debug


def _world_to_pixel(point: np.ndarray, center: np.ndarray, scale: float, size: int) -> Tuple[int, int]:
    x = int(round(size / 2 + (float(point[0] - center[0]) * scale)))
    y = int(round(size / 2 - (float(point[1] - center[1]) * scale)))
    return x, y


def render_detection_png(
    points_xyz: np.ndarray,
    coarse_dock_center_xy: Tuple[float, float],
    output_path: str | Path,
    config: DockLidarConfig = DockLidarConfig(),
    debug: Optional[DetectionDebug] = None,
    size_px: int = 800,
) -> DetectionDebug:
    """Render points, wall segments, gap candidates, selected heading and center."""

    result = debug or detect_entrance_debug(points_xyz, coarse_dock_center_xy, config)
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    center = np.asarray(coarse_dock_center_xy, dtype=float)
    extent = max(config.roi_radius_m, 1.0)
    scale = size_px / (2.0 * extent)
    image = Image.new("RGB", (size_px, size_px), "white")
    draw = ImageDraw.Draw(image)

    def pixel(point: np.ndarray) -> Tuple[int, int]:
        return _world_to_pixel(point, center, scale, size_px)

    for point in result.filtered_xy:
        x, y = pixel(point)
        draw.ellipse((x - 2, y - 2, x + 2, y + 2), fill=(100, 100, 100))
    for segment in result.wall_segments:
        draw.line((pixel(np.asarray(segment.start)), pixel(np.asarray(segment.end))), fill=(30, 100, 220), width=3)
    for candidate in result.candidates:
        gap_a = np.asarray((candidate.center_x, candidate.center_y))
        normal = np.asarray((np.cos(candidate.heading_rad), np.sin(candidate.heading_rad)))
        half = candidate.width_m / 2.0
        a = gap_a - np.asarray((-normal[1], normal[0])) * half
        b = gap_a + np.asarray((-normal[1], normal[0])) * half
        color = (30, 180, 70) if candidate is result.selected else (240, 150, 30)
        draw.line((pixel(a), pixel(b)), fill=color, width=5)
    coarse_pixel = pixel(center)
    draw.ellipse((coarse_pixel[0] - 6, coarse_pixel[1] - 6, coarse_pixel[0] + 6, coarse_pixel[1] + 6), fill=(150, 40, 180))
    if result.selected is not None:
        candidate = result.selected
        p = np.asarray((candidate.center_x, candidate.center_y))
        q = p + np.asarray((np.cos(candidate.heading_rad), np.sin(candidate.heading_rad))) * max(0.6, candidate.width_m * 0.55)
        draw.ellipse((*tuple(np.asarray(pixel(p)) - 6), *tuple(np.asarray(pixel(p)) + 6)), fill=(220, 30, 40))
        draw.line((pixel(p), pixel(q)), fill=(220, 30, 40), width=5)
    image.save(output)
    return result


__all__ = ["render_detection_png"]
