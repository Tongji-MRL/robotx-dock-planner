"""Reproducible offline validation helpers for the dock entrance prototype.

This module is intentionally independent from ROS and vehicle control.  It
contains two kinds of evidence:

* Monte Carlo trials with known synthetic ground truth and a supplied coarse
  dock center.
* Temporal consistency checks over several frames from one stationary real
  recording when a coarse center is not yet available.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence, Tuple

import numpy as np

from .detector import DockLidarConfig, EntranceCandidate, detect_entrance
from .synthetic import SyntheticDockConfig, generate_synthetic_dock_point_cloud


@dataclass(frozen=True)
class TemporalCandidate:
    center_x: float
    center_y: float
    line_normal_rad_mod_pi: float
    width_mean_m: float
    width_std_m: float
    center_std_m: float
    mean_confidence: float
    frame_hits: int
    frame_count: int

    @property
    def persistence(self) -> float:
        return self.frame_hits / self.frame_count

    @property
    def range_m(self) -> float:
        return math.hypot(self.center_x, self.center_y)


def aggregate_temporal_candidates(
    frame_candidates: Sequence[Sequence[EntranceCandidate]],
    center_tolerance_m: float = 0.5,
) -> Tuple[TemporalCandidate, ...]:
    """Associate repeated entrance candidates in one stationary recording."""

    if center_tolerance_m <= 0:
        raise ValueError("center_tolerance_m must be positive")
    groups: list[dict] = []
    observations = [
        (frame_index, candidate)
        for frame_index, candidates in enumerate(frame_candidates)
        for candidate in candidates
    ]
    observations.sort(key=lambda item: item[1].confidence, reverse=True)
    for frame_index, candidate in observations:
        point = np.asarray((candidate.center_x, candidate.center_y), dtype=float)
        match = None
        for group in groups:
            if np.linalg.norm(point - group["mean"]) <= center_tolerance_m:
                match = group
                break
        if match is None:
            groups.append({"items": [(frame_index, candidate)], "mean": point})
        else:
            match["items"].append((frame_index, candidate))
            match["mean"] = np.mean(
                [(item.center_x, item.center_y) for _, item in match["items"]],
                axis=0,
            )

    aggregated = []
    frame_count = len(frame_candidates)
    for group in groups:
        items = group["items"]
        unique_frames = len({frame_index for frame_index, _ in items})
        centers = np.asarray([(item.center_x, item.center_y) for _, item in items])
        widths = np.asarray([item.width_m for _, item in items])
        confidences = np.asarray([item.confidence for _, item in items])
        doubled_headings = np.asarray([2.0 * item.heading_rad for _, item in items])
        heading_mod_pi = 0.5 * math.atan2(
            float(np.sin(doubled_headings).mean()),
            float(np.cos(doubled_headings).mean()),
        )
        mean_center = centers.mean(axis=0)
        center_spread = np.linalg.norm(centers - mean_center, axis=1)
        aggregated.append(TemporalCandidate(
            center_x=float(mean_center[0]),
            center_y=float(mean_center[1]),
            line_normal_rad_mod_pi=heading_mod_pi,
            width_mean_m=float(widths.mean()),
            width_std_m=float(widths.std()),
            center_std_m=float(center_spread.std()),
            mean_confidence=float(confidences.mean()),
            frame_hits=unique_frames,
            frame_count=frame_count,
        ))
    aggregated.sort(
        key=lambda item: (item.frame_hits, item.mean_confidence, -item.center_std_m),
        reverse=True,
    )
    return tuple(aggregated)


def select_external_persistent_candidate(
    candidates: Iterable[TemporalCandidate],
    *,
    min_frame_hits: int = 2,
    expected_width_m: float = 1.5,
    width_tolerance_m: float = 0.35,
) -> TemporalCandidate | None:
    """Select the outermost persistent, dimensionally plausible single entrance.

    This rule is suitable for the one-entrance shore mock.  The competition
    runtime should use the UAV center to establish the exterior side instead.
    """

    plausible = [
        candidate for candidate in candidates
        if candidate.frame_hits >= min_frame_hits
        and abs(candidate.width_mean_m - expected_width_m) <= width_tolerance_m
    ]
    if not plausible:
        return None
    highest_hits = max(candidate.frame_hits for candidate in plausible)
    stable = [candidate for candidate in plausible if candidate.frame_hits == highest_hits]
    return min(
        stable,
        key=lambda item: (
            item.range_m,
            abs(item.width_mean_m - expected_width_m),
            item.center_std_m,
        ),
    )


def run_synthetic_monte_carlo(
    scenario_count: int = 200,
    frames_per_scenario: int = 5,
    seed: int = 20261008,
) -> dict:
    """Measure strict multi-frame accuracy under randomized noise and dropout."""

    if scenario_count < 1 or frames_per_scenario < 1:
        raise ValueError("scenario/frame counts must be positive")
    rng = np.random.default_rng(seed)
    detector_config = DockLidarConfig(
        z_min=-0.3,
        z_max=0.3,
        roi_radius_m=6.0,
        min_entrance_width_m=1.15,
        max_entrance_width_m=1.85,
        ransac_iterations=150,
        max_wall_segments=8,
    )
    accepted = 0
    successes = 0
    errors = []
    for scenario in range(scenario_count):
        heading = float(rng.uniform(-math.pi, math.pi))
        forward = np.asarray((math.cos(heading), math.sin(heading)))
        lateral_axis = np.asarray((-forward[1], forward[0]))
        true_center = (
            forward * float(rng.uniform(4.0, 8.0))
            + lateral_axis * float(rng.uniform(-0.8, 0.8))
        )
        coarse_center = true_center + rng.normal(0.0, 0.25, 2)
        detections = []
        for frame_index in range(frames_per_scenario):
            cloud = generate_synthetic_dock_point_cloud(SyntheticDockConfig(
                length_m=4.0,
                entrance_width_m=1.5,
                center_x=float(true_center[0]),
                center_y=float(true_center[1]),
                heading_rad=heading,
                noise_std_m=float(rng.uniform(0.0, 0.05)),
                dropout_probability=float(rng.uniform(0.0, 0.30)),
                outlier_count=int(rng.integers(0, 21)),
                outlier_radius_m=9.0,
                seed=1000 + scenario * frames_per_scenario + frame_index,
            ))
            detection = detect_entrance(
                cloud,
                tuple(coarse_center),
                detector_config,
                observer_origin_xy=(0.0, 0.0),
            )
            if detection is not None:
                detections.append(detection)
        required = math.ceil(frames_per_scenario / 2.0)
        if len(detections) < required:
            continue
        accepted += 1
        detected_center = np.median(
            [(item.center_x, item.center_y) for item in detections], axis=0
        )
        detected_width = float(np.median([item.width_m for item in detections]))
        headings = np.asarray([item.heading_rad for item in detections])
        detected_heading = math.atan2(float(np.sin(headings).mean()), float(np.cos(headings).mean()))
        true_entrance = true_center - forward * 2.0
        center_error = float(np.linalg.norm(detected_center - true_entrance))
        width_error = abs(detected_width - 1.5)
        heading_error = abs(math.atan2(
            math.sin(detected_heading - heading),
            math.cos(detected_heading - heading),
        ))
        errors.append((center_error, width_error, heading_error))
        if center_error <= 0.35 and width_error <= 0.25 and heading_error <= math.radians(15.0):
            successes += 1

    error_array = np.asarray(errors, dtype=float)
    percentile_95 = np.percentile(error_array, 95, axis=0) if len(error_array) else np.full(3, np.nan)
    median = np.median(error_array, axis=0) if len(error_array) else np.full(3, np.nan)
    return {
        "scenario_count": scenario_count,
        "frames_per_scenario": frames_per_scenario,
        "accepted_scenarios": accepted,
        "successful_scenarios": successes,
        "success_rate": successes / scenario_count,
        "strict_limits": {
            "center_error_m": 0.35,
            "width_error_m": 0.25,
            "heading_error_deg": 15.0,
        },
        "median_error": {
            "center_m": float(median[0]),
            "width_m": float(median[1]),
            "heading_deg": math.degrees(float(median[2])),
        },
        "p95_error": {
            "center_m": float(percentile_95[0]),
            "width_m": float(percentile_95[1]),
            "heading_deg": math.degrees(float(percentile_95[2])),
        },
    }


__all__ = [
    "TemporalCandidate",
    "aggregate_temporal_candidates",
    "run_synthetic_monte_carlo",
    "select_external_persistent_candidate",
]
