"""Run center-free search and temporal tracking over sampled NPY frames."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
from PIL import Image, ImageDraw, ImageOps

from .detector import DockLidarConfig
from .run_recorded_frame import load_points_xyz
from .run_unseeded_search import _render_search
from .search import UnseededSearchConfig, search_entrances_debug
from .tracking import EntranceTrackConfig, track_entrance_candidates


def inspect_sequence(
    input_root: str | Path,
    output_root: str | Path,
    *,
    min_range_m: float = 0.5,
    max_range_m: float = 10.0,
    grid_spacing_m: float = 2.0,
    max_grid_centers: int = 48,
    min_confidence: float = 0.5,
    max_step_m: float = 4.0,
    min_track_hits: int = 4,
) -> dict:
    source = Path(input_root)
    output = Path(output_root)
    output.mkdir(parents=True, exist_ok=True)
    detector_config = DockLidarConfig(
        z_min=-0.5,
        z_max=0.5,
        roi_radius_m=6.0,
        min_entrance_width_m=0.8,
        max_entrance_width_m=3.0,
        ransac_iterations=60,
        max_wall_segments=6,
        min_line_inliers=8,
        random_seed=7,
    )
    search_config = UnseededSearchConfig(
        detector_config=detector_config,
        min_range_m=min_range_m,
        max_range_m=max_range_m,
        grid_spacing_m=grid_spacing_m,
        max_grid_centers=max_grid_centers,
        min_candidate_confidence=min_confidence,
        dedup_distance_m=0.7,
        max_candidates=5,
        self_mask_enabled=True,
    )
    track_config = EntranceTrackConfig(
        max_step_m=max_step_m,
        max_gap_frames=1,
        position_penalty=0.65,
        heading_penalty=0.15,
        min_frame_hits=min_track_hits,
    )

    scenes = {}
    representative_paths = []
    for scene_dir in sorted(path for path in source.iterdir() if path.is_dir()):
        files = sorted(scene_dir.glob("*.npy"))
        if not files:
            continue
        scene_output = output / scene_dir.name
        scene_output.mkdir(parents=True, exist_ok=True)
        frame_candidates = []
        frame_rows = []
        representative_indices = {0, len(files) // 2, len(files) - 1}
        for index, frame_path in enumerate(files):
            points = load_points_xyz(frame_path)
            debug = search_entrances_debug(points, search_config)
            frame_candidates.append(debug.candidates)
            selected = debug.selected
            row = {
                "frame": frame_path.name,
                "input_points": int(len(points)),
                "filtered_points": int(len(debug.filtered_xyz)),
                "removed_near_points": debug.removed_near_points,
                "removed_self_points": debug.removed_self_points,
                "candidate_count": len(debug.candidates),
                "selected": asdict(selected) if selected is not None else None,
            }
            frame_rows.append(row)
            if index in representative_indices:
                image_path = scene_output / f"{frame_path.stem}_search.png"
                _render_search(points, debug, image_path)
                representative_paths.append((scene_dir.name, frame_path.name, image_path))

        track = track_entrance_candidates(frame_candidates, track_config)
        track_result = None
        if track is not None:
            track_result = {
                "frame_indices": list(track.frame_indices),
                "frame_hits": track.frame_hits,
                "score": track.score,
                "center_xy": list(track.center_xy),
                "center_std_m": track.center_std_m,
                "mean_confidence": track.mean_confidence,
                "median_width_m": track.median_width_m,
                "valid": track.valid,
            }
        summary = {
            "scene": scene_dir.name,
            "frame_count": len(files),
            "frames_with_candidate": sum(bool(row["selected"]) for row in frame_rows),
            "mean_removed_self_points": float(np.mean([row["removed_self_points"] for row in frame_rows])),
            "track": track_result,
            "frames": frame_rows,
        }
        scenes[scene_dir.name] = summary
        (scene_output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    thumbnails = []
    for scene, frame, image_path in representative_paths:
        image = Image.open(image_path).convert("RGB")
        canvas = Image.new("RGB", (520, 410), "white")
        canvas.paste(ImageOps.contain(image, (500, 360)), (10, 35))
        ImageDraw.Draw(canvas).text((10, 10), f"{scene} {frame}", fill="black")
        thumbnails.append(canvas)
    if thumbnails:
        columns = 3
        rows = (len(thumbnails) + columns - 1) // columns
        sheet = Image.new("RGB", (columns * 520, rows * 410), "white")
        for index, thumbnail in enumerate(thumbnails):
            sheet.paste(thumbnail, ((index % columns) * 520, (index // columns) * 410))
        sheet.save(output / "representative_searches.jpg", quality=90)

    report = {
        "input_root": str(source.resolve()),
        "output_root": str(output.resolve()),
        "mode": "center-free search + self-footprint mask + temporal tracking",
        "coarse_center_used": False,
        "search_config": {
            "min_range_m": search_config.min_range_m,
            "max_range_m": search_config.max_range_m,
            "grid_spacing_m": search_config.grid_spacing_m,
            "max_grid_centers": search_config.max_grid_centers,
            "min_confidence": search_config.min_candidate_confidence,
            "self_mask": [
                search_config.self_mask_x_min_m,
                search_config.self_mask_x_max_m,
                search_config.self_mask_y_min_m,
                search_config.self_mask_y_max_m,
            ],
        },
        "track_config": asdict(track_config),
        "scenes": scenes,
    }
    (output / "batch_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_root", type=Path)
    parser.add_argument("output_root", type=Path)
    parser.add_argument("--min-range", type=float, default=0.5)
    parser.add_argument("--max-range", type=float, default=10.0)
    parser.add_argument("--grid-spacing", type=float, default=2.0)
    parser.add_argument("--max-grid-centers", type=int, default=48)
    parser.add_argument("--min-confidence", type=float, default=0.5)
    parser.add_argument("--max-step", type=float, default=4.0)
    parser.add_argument("--min-track-hits", type=int, default=4)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = _parser().parse_args(argv)
    report = inspect_sequence(
        args.input_root,
        args.output_root,
        min_range_m=args.min_range,
        max_range_m=args.max_range,
        grid_spacing_m=args.grid_spacing,
        max_grid_centers=args.max_grid_centers,
        min_confidence=args.min_confidence,
        max_step_m=args.max_step,
        min_track_hits=args.min_track_hits,
    )
    compact = {
        scene: {
            "frames": summary["frame_count"],
            "candidate_frames": summary["frames_with_candidate"],
            "track": summary["track"],
        }
        for scene, summary in report["scenes"].items()
    }
    print(json.dumps(compact, indent=2))


if __name__ == "__main__":
    main()
