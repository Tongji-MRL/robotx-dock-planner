"""Cross-frame association for center-free entrance candidates.

The detector scores each frame independently.  This module adds a small
dynamic-programming tracker so a high-scoring one-frame false positive does
not immediately replace a candidate that moves smoothly through the scan.
It operates entirely in the input local frame and does not estimate TF or
vehicle motion.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

import numpy as np

from .detector import EntranceCandidate


def _angle_difference(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


@dataclass(frozen=True)
class EntranceTrackConfig:
    """Motion and scoring limits for a sequence of candidate lists."""

    max_step_m: float = 3.5
    max_gap_frames: int = 1
    position_penalty: float = 0.45
    heading_penalty: float = 0.12
    start_penalty: float = 0.15
    min_frame_hits: int = 3

    def __post_init__(self) -> None:
        for name in (
            "max_step_m", "position_penalty", "heading_penalty", "start_penalty",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if self.max_step_m <= 0:
            raise ValueError("max_step_m must be positive")
        if self.max_gap_frames < 0 or self.min_frame_hits < 1:
            raise ValueError("frame limits are invalid")


@dataclass(frozen=True)
class EntranceTrack:
    """One smooth candidate path through a sequence of frames."""

    frame_indices: Tuple[int, ...]
    candidates: Tuple[EntranceCandidate, ...]
    score: float

    @property
    def frame_hits(self) -> int:
        return len(self.candidates)

    @property
    def center_xy(self) -> Tuple[float, float]:
        centers = np.asarray([(c.center_x, c.center_y) for c in self.candidates], dtype=float)
        mean = centers.mean(axis=0)
        return float(mean[0]), float(mean[1])

    @property
    def center_std_m(self) -> float:
        centers = np.asarray([(c.center_x, c.center_y) for c in self.candidates], dtype=float)
        mean = centers.mean(axis=0)
        return float(np.linalg.norm(centers - mean, axis=1).std())

    @property
    def mean_confidence(self) -> float:
        return float(np.mean([c.confidence for c in self.candidates]))

    @property
    def median_width_m(self) -> float:
        return float(np.median([c.width_m for c in self.candidates]))

    @property
    def valid(self) -> bool:
        return self.frame_hits >= 1


def _edge_score(
    previous: EntranceCandidate,
    current: EntranceCandidate,
    frame_delta: int,
    config: EntranceTrackConfig,
) -> Optional[float]:
    distance = math.hypot(
        current.center_x - previous.center_x,
        current.center_y - previous.center_y,
    )
    allowed = config.max_step_m * frame_delta
    if distance > allowed:
        return None
    position_cost = config.position_penalty * distance / max(allowed, 1e-9)
    heading_cost = config.heading_penalty * _angle_difference(
        current.heading_rad, previous.heading_rad,
    ) / math.pi
    return current.confidence - position_cost - heading_cost


def track_entrance_candidates(
    frame_candidates: Sequence[Sequence[EntranceCandidate]],
    config: EntranceTrackConfig = EntranceTrackConfig(),
) -> Optional[EntranceTrack]:
    """Return the best smooth path, or ``None`` if it is too short.

    Empty candidate frames are allowed.  A path can bridge at most
    ``max_gap_frames`` empty/intervening frames, and displacement is scaled by
    the number of frame indices crossed.
    """

    nodes = [
        (frame_index, candidate)
        for frame_index, candidates in enumerate(frame_candidates)
        for candidate in candidates
    ]
    if not nodes:
        return None

    scores = [candidate.confidence - config.start_penalty for _, candidate in nodes]
    predecessors: list[Optional[int]] = [None] * len(nodes)
    path_lengths = [1] * len(nodes)
    for current_index, (frame_index, candidate) in enumerate(nodes):
        first_previous = max(0, current_index - len(nodes))
        for previous_index in range(first_previous, current_index):
            previous_frame, previous = nodes[previous_index]
            frame_delta = frame_index - previous_frame
            if frame_delta < 1 or frame_delta > config.max_gap_frames + 1:
                continue
            edge = _edge_score(previous, candidate, frame_delta, config)
            if edge is None:
                continue
            score = scores[previous_index] + edge
            if score > scores[current_index]:
                scores[current_index] = score
                predecessors[current_index] = previous_index
                path_lengths[current_index] = path_lengths[previous_index] + 1

    eligible = [
        index for index, length in enumerate(path_lengths)
        if length >= config.min_frame_hits
    ]
    if not eligible:
        return None
    best_index = max(eligible, key=lambda index: scores[index])
    path_indices = []
    while best_index is not None:
        path_indices.append(best_index)
        best_index = predecessors[best_index]
    path_indices.reverse()
    frame_indices = tuple(nodes[index][0] for index in path_indices)
    candidates = tuple(nodes[index][1] for index in path_indices)
    return EntranceTrack(
        frame_indices=frame_indices,
        candidates=candidates,
        score=float(scores[path_indices[-1]]),
    )


__all__ = ["EntranceTrack", "EntranceTrackConfig", "track_entrance_candidates"]
