"""Capture fixed Gazebo LiDAR samples around the RobotX 2026 dock model.

The script is an experiment runner, not a ROS node.  Start the isolated world
first, then run this module with the same GZ_IP and GZ_PARTITION values.  It
moves only the static test LiDAR model through discrete poses; no vehicle or
control interface is involved.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
from pathlib import Path
import subprocess
import time
from typing import Iterable, Tuple

import numpy as np
from PIL import Image, ImageDraw

from .detector import DockLidarConfig, EntranceCandidate, detect_entrance_debug
from .visualize import render_detection_png


WORLD = "robotx_2026_dock_lidar_test"
MODEL = "test_lidar"
POINT_TOPIC = "/robotx_2026_dock_lidar/scan/points"
DOCK_CENTER_WORLD = np.asarray((1.5, 0.0), dtype=float)
TRUE_ENTRANCES_WORLD = (
    np.asarray((0.0, -2.0295384615), dtype=float),
    np.asarray((0.0, 0.0), dtype=float),
    np.asarray((0.0, 2.0295384615), dtype=float),
)


def _run(command: list[str], timeout_s: float = 25.0) -> str:
    result = subprocess.run(
        command,
        check=True,
        capture_output=True,
        text=True,
        timeout=timeout_s,
        env=os.environ.copy(),
    )
    return result.stdout


def _move_sensor(position: np.ndarray, yaw: float) -> None:
    request = (
        f'name: "{MODEL}" '
        f'position: {{x: {position[0]:.9f}, y: {position[1]:.9f}, z: 1.2}} '
        f'orientation: {{z: {math.sin(yaw / 2.0):.12f}, w: {math.cos(yaw / 2.0):.12f}}}'
    )
    response = _run([
        "gz", "service",
        "-s", f"/world/{WORLD}/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "3000",
        "--req", request,
    ])
    if "true" not in response.lower():
        raise RuntimeError(f"Gazebo rejected pose update: {response.strip()}")


def _capture_xyz() -> np.ndarray:
    text = _run([
        "gz", "topic", "-e", "-n", "1", "--json-output", "-t", POINT_TOPIC,
    ])
    payload = json.loads(text)
    raw = base64.b64decode(payload["data"])
    count = int(payload["height"]) * int(payload["width"])
    step = int(payload["pointStep"])
    return np.ndarray(
        (count, 3), dtype=np.float32, buffer=raw, strides=(step, 4)
    ).copy()


def _to_world(point: Tuple[float, float], observer: np.ndarray, yaw: float) -> np.ndarray:
    c = math.cos(yaw)
    s = math.sin(yaw)
    rotation = np.asarray(((c, -s), (s, c)))
    return observer + rotation @ np.asarray(point, dtype=float)


def _match_true_bay(candidate: EntranceCandidate, observer: np.ndarray, yaw: float) -> int | None:
    world_point = _to_world((candidate.center_x, candidate.center_y), observer, yaw)
    errors = [float(np.linalg.norm(world_point - truth)) for truth in TRUE_ENTRANCES_WORLD]
    index = int(np.argmin(errors))
    world_heading = math.atan2(
        math.sin(candidate.heading_rad + yaw),
        math.cos(candidate.heading_rad + yaw),
    )
    heading_error = abs(math.atan2(math.sin(world_heading), math.cos(world_heading)))
    if errors[index] <= 0.45 and heading_error <= math.radians(25.0):
        return index
    return None


def _label_frame(path: Path, angle_deg: float, matched: Iterable[int], selected: EntranceCandidate | None) -> None:
    image = Image.open(path).convert("RGB")
    draw = ImageDraw.Draw(image)
    bay_text = ", ".join(f"BAY_{index + 1}" for index in sorted(set(matched))) or "none"
    selected_text = "none" if selected is None else f"({selected.center_x:.2f}, {selected.center_y:.2f})"
    draw.rectangle((0, 0, image.width, 48), fill=(255, 255, 255))
    draw.text((10, 7), f"circle angle: {angle_deg:+.0f} deg | true bays found: {bay_text}", fill=(0, 0, 0))
    draw.text((10, 26), f"selected local entrance: {selected_text}", fill=(0, 0, 0))
    image.save(path)


def _make_contact_sheet(frames: list[Path], output: Path) -> None:
    thumbnails = []
    for frame in frames:
        image = Image.open(frame).convert("RGB")
        image.thumbnail((360, 360))
        thumbnails.append(image.copy())
    columns = 4
    rows = math.ceil(len(thumbnails) / columns)
    sheet = Image.new("RGB", (columns * 360, rows * 360), "white")
    for index, image in enumerate(thumbnails):
        sheet.paste(image, ((index % columns) * 360, (index // columns) * 360))
    sheet.save(output)


def run(output_dir: Path, view_count: int = 16, radius_m: float = 7.0) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    config = DockLidarConfig(
        z_min=-1.3,
        z_max=-0.2,
        roi_radius_m=5.0,
        min_entrance_width_m=1.3,
        max_entrance_width_m=1.8,
        cluster_distance_m=0.18,
        min_cluster_points=8,
        line_distance_threshold_m=0.06,
        min_wall_length_m=0.7,
        min_line_inliers=10,
        max_gap_interior_points=5,
        max_wall_segments=12,
    )
    frames = []
    records = []
    found_bays = set()
    # Start at the front (-X) and move counter-clockwise around a full circle.
    angles = np.linspace(math.pi, math.pi + 2.0 * math.pi, view_count, endpoint=False)
    for index, angle in enumerate(angles):
        observer = DOCK_CENTER_WORLD + radius_m * np.asarray((math.cos(angle), math.sin(angle)))
        yaw = math.atan2(DOCK_CENTER_WORLD[1] - observer[1], DOCK_CENTER_WORLD[0] - observer[0])
        _move_sensor(observer, yaw)
        time.sleep(0.65)
        points = _capture_xyz()
        coarse_center_local = (radius_m, 0.0)
        debug = detect_entrance_debug(
            points,
            coarse_center_local,
            config,
            observer_origin_xy=(0.0, 0.0),
        )
        matched = []
        for candidate in debug.candidates:
            bay = _match_true_bay(candidate, observer, yaw)
            if bay is not None:
                matched.append(bay)
                found_bays.add(bay)
        frame = output_dir / f"view_{index:02d}_{math.degrees(angle):.0f}deg.png"
        render_detection_png(points, coarse_center_local, frame, config, debug)
        _label_frame(frame, math.degrees(angle), matched, debug.selected)
        frames.append(frame)
        records.append({
            "index": index,
            "circle_angle_deg": float(math.degrees(angle)),
            "observer_world_xy": observer.tolist(),
            "yaw_rad": yaw,
            "candidate_count": len(debug.candidates),
            "matched_bays": [f"BAY_{bay + 1}" for bay in sorted(set(matched))],
            "selected": None if debug.selected is None else {
                "center_local_xy": [debug.selected.center_x, debug.selected.center_y],
                "heading_local_rad": debug.selected.heading_rad,
                "width_m": debug.selected.width_m,
                "confidence": debug.selected.confidence,
            },
        })

    _make_contact_sheet(frames, output_dir / "contact_sheet.png")
    gif_frames = [Image.open(frame).convert("RGB") for frame in frames]
    gif_frames[0].save(
        output_dir / "circle_detection.gif",
        save_all=True,
        append_images=gif_frames[1:],
        duration=650,
        loop=0,
    )
    summary = {
        "view_count": view_count,
        "radius_m": radius_m,
        "found_bays": [f"BAY_{bay + 1}" for bay in sorted(found_bays)],
        "all_three_bays_found": found_bays == {0, 1, 2},
        "records": records,
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).with_name("artifacts") / "robotx_2026_multiview",
    )
    parser.add_argument("--views", type=int, default=16)
    parser.add_argument("--radius", type=float, default=7.0)
    args = parser.parse_args()
    summary = run(args.output_dir, args.views, args.radius)
    print(json.dumps({key: value for key, value in summary.items() if key != "records"}, indent=2))


if __name__ == "__main__":
    main()
