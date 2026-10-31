"""Offline RobotX Task 3 dock entrance geometry prototype."""

from .detector import (
    DetectionDebug,
    DockLidarConfig,
    EntranceCandidate,
    WallSegment,
    detect_entrance,
    detect_entrance_debug,
    detect_entrances,
)
from .synthetic import SyntheticDockConfig, generate_synthetic_dock_point_cloud
from .visualize import render_detection_png
from .search import SearchDebug, UnseededSearchConfig, search_entrance, search_entrances, search_entrances_debug
from .tracking import EntranceTrack, EntranceTrackConfig, track_entrance_candidates
from .validation import (
    TemporalCandidate,
    aggregate_temporal_candidates,
    run_synthetic_monte_carlo,
    select_external_persistent_candidate,
)

__all__ = [
    "DetectionDebug",
    "DockLidarConfig",
    "EntranceCandidate",
    "WallSegment",
    "detect_entrance",
    "detect_entrance_debug",
    "detect_entrances",
    "SyntheticDockConfig",
    "generate_synthetic_dock_point_cloud",
    "render_detection_png",
    "SearchDebug",
    "UnseededSearchConfig",
    "search_entrance",
    "search_entrances",
    "search_entrances_debug",
    "EntranceTrack",
    "EntranceTrackConfig",
    "track_entrance_candidates",
    "TemporalCandidate",
    "aggregate_temporal_candidates",
    "run_synthetic_monte_carlo",
    "select_external_persistent_candidate",
]
