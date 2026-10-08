"""Run the offline dock detector on one NPY/NPZ point-cloud frame."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np

from .detector import DockLidarConfig, detect_entrance_debug
from .visualize import render_detection_png


def load_points_xyz(path: str | Path) -> np.ndarray:
    source = Path(path)
    loaded = np.load(source, allow_pickle=False)
    if isinstance(loaded, np.lib.npyio.NpzFile):
        try:
            if "points_xyz" not in loaded.files:
                raise ValueError("NPZ must contain a points_xyz array")
            points = loaded["points_xyz"]
        finally:
            loaded.close()
    else:
        points = loaded
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("recorded point cloud must have shape (N, 3)")
    return points


def inspect_recorded_frame(
    input_path: str | Path,
    coarse_center_xy: Tuple[float, float],
    output_png: str | Path,
    output_json: str | Path,
    *,
    observer_origin_xy: Optional[Tuple[float, float]] = None,
    config: DockLidarConfig = DockLidarConfig(),
) -> dict:
    points = load_points_xyz(input_path)
    debug = detect_entrance_debug(
        points,
        coarse_center_xy,
        config,
        observer_origin_xy=observer_origin_xy,
    )
    render_detection_png(
        points,
        coarse_center_xy,
        output_png,
        config,
        debug=debug,
    )
    result = {
        "input": str(Path(input_path).resolve()),
        "input_points": int(len(points)),
        "filtered_points": int(len(debug.filtered_xy)),
        "wall_segments": len(debug.wall_segments),
        "candidate_count": len(debug.candidates),
        "selected": asdict(debug.selected) if debug.selected is not None else None,
        "coordinate_frame": "unchanged input local frame",
    }
    output = Path(output_json)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--center-x", type=float, required=True)
    parser.add_argument("--center-y", type=float, required=True)
    parser.add_argument("--observer-x", type=float)
    parser.add_argument("--observer-y", type=float)
    parser.add_argument("--output-png", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--z-min", type=float, default=-0.25)
    parser.add_argument("--z-max", type=float, default=0.25)
    parser.add_argument("--roi-radius", type=float, default=8.0)
    parser.add_argument("--min-width", type=float, default=0.8)
    parser.add_argument("--max-width", type=float, default=3.0)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = _parser().parse_args(argv)
    if (args.observer_x is None) != (args.observer_y is None):
        raise SystemExit("observer-x and observer-y must be provided together")
    observer = None
    if args.observer_x is not None:
        observer = (args.observer_x, args.observer_y)
    result = inspect_recorded_frame(
        args.input,
        (args.center_x, args.center_y),
        args.output_png,
        args.output_json,
        observer_origin_xy=observer,
        config=DockLidarConfig(
            z_min=args.z_min,
            z_max=args.z_max,
            roi_radius_m=args.roi_radius,
            min_entrance_width_m=args.min_width,
            max_entrance_width_m=args.max_width,
        ),
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
