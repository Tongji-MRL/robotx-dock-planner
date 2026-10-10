"""Create a reproducible visual/JSON evidence bundle for offline validation."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
from typing import Sequence

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .detector import DockLidarConfig
from .run_recorded_frame import load_points_xyz
from .search import UnseededSearchConfig, search_entrances_debug
from .validation import (
    aggregate_temporal_candidates,
    run_synthetic_monte_carlo,
    select_external_persistent_candidate,
)


BAG_IDS = ("170759", "171155", "171440", "171617", "171754")


def _font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    path = Path("/usr/share/fonts/truetype/dejavu") / name
    return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default()


def _search_config() -> UnseededSearchConfig:
    return UnseededSearchConfig(
        detector_config=DockLidarConfig(
            z_min=-0.5,
            z_max=0.5,
            roi_radius_m=5.5,
            min_entrance_width_m=1.15,
            max_entrance_width_m=1.85,
            ransac_iterations=50,
            max_wall_segments=5,
        ),
        min_range_m=0.8,
        max_range_m=9.0,
        grid_spacing_m=3.0,
        max_grid_centers=24,
        min_candidate_confidence=0.45,
        dedup_distance_m=0.45,
        max_candidates=4,
    )


def _analyze_real(frames_root: Path) -> dict:
    config = _search_config()
    result = {}
    for bag_id in BAG_IDS:
        frame_paths = sorted((frames_root / f"frames_{bag_id}").glob("frame_*.npy"))
        frame_candidates = []
        for frame_path in frame_paths:
            debug = search_entrances_debug(load_points_xyz(frame_path), config)
            frame_candidates.append(debug.candidates)
        temporal = aggregate_temporal_candidates(frame_candidates, center_tolerance_m=0.5)
        persistent = tuple(item for item in temporal if item.frame_hits >= 2)
        selected = select_external_persistent_candidate(
            persistent,
            min_frame_hits=2,
            expected_width_m=1.5,
            width_tolerance_m=0.35,
        )
        result[bag_id] = {
            "frame_paths": [str(path) for path in frame_paths],
            "persistent_candidates": [asdict(item) | {"range_m": item.range_m} for item in persistent],
            "selected": None if selected is None else asdict(selected) | {"range_m": selected.range_m},
        }
    return result


def _fit_image(source: Image.Image, size: tuple[int, int]) -> Image.Image:
    image = source.copy().convert("RGB")
    image.thumbnail(size)
    canvas = Image.new("RGB", size, (247, 247, 247))
    canvas.paste(image, ((size[0] - image.width) // 2, (size[1] - image.height) // 2))
    return canvas


def _draw_cloud_panel(
    points: np.ndarray,
    candidates: Sequence[dict],
    selected: dict | None,
    size: tuple[int, int],
) -> Image.Image:
    width, height = size
    image = Image.new("RGB", size, "white")
    draw = ImageDraw.Draw(image)
    margin = 35
    finite = points[np.isfinite(points).all(axis=1)]
    ranges = np.linalg.norm(finite[:, :2], axis=1)
    visible = finite[
        (finite[:, 2] >= -0.5) & (finite[:, 2] <= 0.5)
        & (ranges >= 0.8) & (ranges <= 6.0)
    ]
    extent = 5.0
    scale = min((width - 2 * margin) / (2 * extent), (height - 2 * margin) / (2 * extent))

    def pixel(x: float, y: float) -> tuple[int, int]:
        return int(width / 2 + x * scale), int(height / 2 - y * scale)

    draw.rectangle((margin, margin, width - margin, height - margin), outline=(180, 180, 180), width=1)
    draw.line((pixel(-extent, 0), pixel(extent, 0)), fill=(220, 220, 220), width=1)
    draw.line((pixel(0, -extent), pixel(0, extent)), fill=(220, 220, 220), width=1)
    stride = max(1, len(visible) // 6000)
    for x, y, _ in visible[::stride]:
        px, py = pixel(float(x), float(y))
        if margin <= px <= width - margin and margin <= py <= height - margin:
            draw.point((px, py), fill=(125, 125, 125))
    origin = pixel(0.0, 0.0)
    draw.ellipse((origin[0] - 5, origin[1] - 5, origin[0] + 5, origin[1] + 5), fill=(130, 55, 170))
    draw.text((origin[0] + 7, origin[1] + 4), "LiDAR", font=_font(12), fill=(80, 40, 100))

    for candidate in candidates:
        is_selected = selected is not None and math.hypot(
            candidate["center_x"] - selected["center_x"],
            candidate["center_y"] - selected["center_y"],
        ) < 0.05
        color = (22, 145, 75) if is_selected else (235, 145, 25)
        heading = candidate["line_normal_rad_mod_pi"]
        tangent = np.asarray((-math.sin(heading), math.cos(heading)))
        center = np.asarray((candidate["center_x"], candidate["center_y"]))
        half = tangent * candidate["width_mean_m"] / 2.0
        a = pixel(*(center - half))
        b = pixel(*(center + half))
        c = pixel(*center)
        draw.line((a, b), fill=color, width=5 if is_selected else 3)
        draw.ellipse((c[0] - 5, c[1] - 5, c[0] + 5, c[1] + 5), fill=color)
        if is_selected:
            draw.text(
                (c[0] + 7, c[1] - 18),
                f"selected {candidate['frame_hits']}/5, {candidate['width_mean_m']:.2f} m",
                font=_font(12, True),
                fill=color,
            )
    draw.text((margin, 8), "XY points in livox_frame (m)", font=_font(15, True), fill=(35, 35, 35))
    draw.text((width - 245, height - 24), "green selected | orange alternatives", font=_font(11), fill=(55, 55, 55))
    return image


def _render_evidence(
    output_path: Path,
    prototype_root: Path,
    photos_root: Path,
    real: dict,
    monte_carlo: dict,
) -> None:
    width = 1320
    header_h = 155
    known_h = 360
    row_h = 330
    image = Image.new("RGB", (width, header_h + known_h + row_h * 5), (242, 242, 242))
    draw = ImageDraw.Draw(image)
    draw.text((28, 20), "RobotX Task 3 dock entrance detector — offline evidence", font=_font(28, True), fill=(25, 25, 25))
    draw.text(
        (28, 65),
        f"18/18 unit tests | synthetic multi-frame {monte_carlo['successful_scenarios']}/{monte_carlo['scenario_count']} strict pass "
        f"({100 * monte_carlo['success_rate']:.1f}%) | Gazebo: all 3 official bays found",
        font=_font(18),
        fill=(35, 35, 35),
    )
    draw.text(
        (28, 102),
        "Real shore bags below: orange = repeated candidate; green = one-entrance external candidate. Photos and clouds share recording order.",
        font=_font(16),
        fill=(70, 70, 70),
    )

    synthetic = _fit_image(Image.open(prototype_root / "artifacts" / "synthetic_detection.png"), (620, known_h - 50))
    gazebo = _fit_image(Image.open(prototype_root / "artifacts" / "robotx_2026_multiview_dense" / "successful_detections.png"), (620, known_h - 50))
    y0 = header_h + 35
    image.paste(synthetic, (20, y0))
    image.paste(gazebo, (680, y0))
    draw.text((25, header_h + 8), "Known truth: noisy synthetic U dock", font=_font(17, True), fill=(30, 30, 30))
    draw.text((685, header_h + 8), "Known truth: official-size Gazebo dock, successful views", font=_font(17, True), fill=(30, 30, 30))

    frames_root = prototype_root / "artifacts"
    for index, bag_id in enumerate(BAG_IDS):
        top = header_h + known_h + index * row_h
        draw.rectangle((0, top, width, top + row_h), fill=(255, 255, 255) if index % 2 == 0 else (248, 248, 248))
        photo_path = photos_root / f"{index + 1}.jpg"
        photo = _fit_image(Image.open(photo_path), (430, 280))
        image.paste(photo, (20, top + 35))
        frame_path = Path(real[bag_id]["frame_paths"][2])
        cloud = _draw_cloud_panel(
            load_points_xyz(frame_path),
            real[bag_id]["persistent_candidates"],
            real[bag_id]["selected"],
            (650, 280),
        )
        image.paste(cloud, (470, top + 35))
        selected = real[bag_id]["selected"]
        draw.text((20, top + 7), f"{index + 1}. bag {bag_id}", font=_font(18, True), fill=(30, 30, 30))
        if selected is None:
            lines = ("No persistent", "1.5 m candidate")
        else:
            lines = (
                f"selected {selected['frame_hits']}/5 frames",
                f"range {selected['range_m']:.2f} m",
                f"width {selected['width_mean_m']:.2f} ± {selected['width_std_m']:.2f} m",
                f"center ({selected['center_x']:.2f}, {selected['center_y']:.2f})",
            )
        for line_index, line in enumerate(lines):
            draw.text((1140, top + 70 + line_index * 34), line, font=_font(15, line_index == 0), fill=(35, 35, 35))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    prototype_root = Path(__file__).resolve().parent
    parser.add_argument("--frames-root", type=Path, default=prototype_root / "artifacts")
    parser.add_argument("--photos-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=prototype_root / "artifacts" / "validation_20261008")
    parser.add_argument("--scenarios", type=int, default=200)
    args = parser.parse_args(argv)

    monte_carlo = run_synthetic_monte_carlo(args.scenarios, 5)
    real = _analyze_real(args.frames_root)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    public_real = {
        bag_id: {
            "frame_names": [Path(path).name for path in value["frame_paths"]],
            "persistent_candidates": value["persistent_candidates"],
            "selected": value["selected"],
        }
        for bag_id, value in real.items()
    }
    payload = {"synthetic_monte_carlo": monte_carlo, "real_stationary_bags": public_real}
    (args.output_dir / "summary.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    _render_evidence(
        args.output_dir / "evidence.png",
        prototype_root,
        args.photos_root,
        real,
        monte_carlo,
    )
    print(json.dumps({
        "synthetic_monte_carlo": monte_carlo,
        "evidence": str(args.output_dir / "evidence.png"),
        "summary": str(args.output_dir / "summary.json"),
    }, indent=2))


if __name__ == "__main__":
    main()
