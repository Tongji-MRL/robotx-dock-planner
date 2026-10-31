"""Run the center-free entrance search on one offline NPY/NPZ frame."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
from PIL import Image, ImageDraw

from .detector import DockLidarConfig
from .run_recorded_frame import load_points_xyz
from .search import SearchDebug, UnseededSearchConfig, search_entrances_debug


def _render_search(points_xyz: np.ndarray, debug: SearchDebug, output_path: Path, size_px: int = 900) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    xy = points_xyz[:, :2] if len(points_xyz) else np.zeros((1, 2))
    all_xy = np.vstack((xy, np.asarray(debug.grid_centers, dtype=float))) if debug.grid_centers else xy
    low = all_xy.min(axis=0) - 1.0
    high = all_xy.max(axis=0) + 1.0
    span = max(float(np.max(high - low)), 1.0)
    scale = size_px * 0.88 / span

    def pixel(point: np.ndarray) -> tuple[int, int]:
        return (
            int(round((float(point[0]) - low[0]) * scale + size_px * 0.06)),
            int(round(size_px * 0.94 - (float(point[1]) - low[1]) * scale)),
        )

    image = Image.new("RGB", (size_px, size_px), "white")
    draw = ImageDraw.Draw(image)
    for point in xy[::max(1, len(xy) // 12000)]:
        x, y = pixel(point)
        draw.ellipse((x - 1, y - 1, x + 1, y + 1), fill=(150, 150, 150))
    observer = np.asarray((0.0, 0.0))
    ox, oy = pixel(observer)
    draw.ellipse((ox - 5, oy - 5, ox + 5, oy + 5), fill=(150, 40, 180))
    for candidate in debug.candidates:
        center = np.asarray((candidate.center_x, candidate.center_y))
        normal = np.asarray((np.cos(candidate.heading_rad), np.sin(candidate.heading_rad)))
        tangent = np.asarray((-normal[1], normal[0]))
        a = center - tangent * candidate.width_m / 2.0
        b = center + tangent * candidate.width_m / 2.0
        color = (25, 160, 70) if candidate is debug.selected else (230, 140, 30)
        draw.line((pixel(a), pixel(b)), fill=color, width=5)
        draw.line((pixel(center), pixel(center + normal * max(0.6, candidate.width_m * 0.55))), fill=color, width=4)
        cx, cy = pixel(center)
        draw.ellipse((cx - 5, cy - 5, cx + 5, cy + 5), fill=color)
    image.save(output_path)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output-png", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--min-range", type=float, default=1.5)
    parser.add_argument("--max-range", type=float, default=12.0)
    parser.add_argument("--grid-spacing", type=float, default=2.0)
    parser.add_argument("--z-min", type=float, default=-0.25)
    parser.add_argument("--z-max", type=float, default=0.25)
    parser.add_argument("--roi-radius", type=float, default=8.0)
    parser.add_argument("--min-width", type=float, default=0.8)
    parser.add_argument("--max-width", type=float, default=3.0)
    parser.add_argument("--min-confidence", type=float, default=0.35)
    parser.add_argument("--disable-self-mask", action="store_true")
    parser.add_argument("--self-x-min", type=float, default=-0.70)
    parser.add_argument("--self-x-max", type=float, default=0.85)
    parser.add_argument("--self-y-min", type=float, default=-0.55)
    parser.add_argument("--self-y-max", type=float, default=0.55)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = _parser().parse_args(argv)
    points = load_points_xyz(args.input)
    config = UnseededSearchConfig(
        min_range_m=args.min_range,
        max_range_m=args.max_range,
        grid_spacing_m=args.grid_spacing,
        min_candidate_confidence=args.min_confidence,
        self_mask_enabled=not args.disable_self_mask,
        self_mask_x_min_m=args.self_x_min,
        self_mask_x_max_m=args.self_x_max,
        self_mask_y_min_m=args.self_y_min,
        self_mask_y_max_m=args.self_y_max,
        detector_config=DockLidarConfig(
            z_min=args.z_min,
            z_max=args.z_max,
            roi_radius_m=args.roi_radius,
            min_entrance_width_m=args.min_width,
            max_entrance_width_m=args.max_width,
        ),
    )
    debug = search_entrances_debug(points, config)
    _render_search(points, debug, args.output_png)
    result = {
        "input": str(args.input.resolve()),
        "input_points": int(len(points)),
        "filtered_points": int(len(debug.filtered_xyz)),
        "removed_near_points": debug.removed_near_points,
        "removed_self_points": debug.removed_self_points,
        "grid_centers_evaluated": len(debug.grid_centers),
        "raw_candidate_count": len(debug.raw_candidates),
        "candidate_count": len(debug.candidates),
        "selected": asdict(debug.selected) if debug.selected is not None else None,
        "candidates": [asdict(candidate) for candidate in debug.candidates],
        "coordinate_frame": "unchanged input local frame; no TF/NED conversion",
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
