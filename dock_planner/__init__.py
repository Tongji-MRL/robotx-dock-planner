"""ROS-independent DockPlanner V0.1 and Task3 integration contracts."""

from .planner import DockPlanner
from .integration import (
    ActiveBayContext, BayColorObservation, CoarseDockCenterObservationNED, CoarseDockInfo,
    CoarseDockObservation,
    CoarseDockSearchConfig, DockEntranceObservationNED,
    CameraEntranceCenter, DataFreshnessConfig, DockPerceptionResult,
    DockVisionCoordinateConverter, DockVisionFace, DockVisionObservation,
    GlobalApproachPlanner,
    GlobalApproachResult, GenericVelocityCommand,
    GlobalApproachStatus, NEDPoint, ObstaclePointNED, PreciseEntrancePoseNED,
    TerminalPlanInput, coarse_dock_search_radius_m, coarse_dock_staging_pose,
    dock_vision_to_bay_color_observation,
    entrance_observation_to_precise_pose, is_timestamp_fresh,
    parse_dock_vision_json, prepare_terminal_plan_input,
)
from .ros_twist_adapter import DEFAULT_RATE_HZ, DEFAULT_TOPIC, RosTwistAdapter, to_twist_message
from .task3_flow import Task3FlowManager, Task3FlowState
from .dock_search import (
    CircularDockSearchConfig, CircularDockSearchPlan, plan_circular_dock_search,
)
from .dock_perception import (
    DockOpeningCandidate, DockPerceptionConfig, DockPerceptionDebugResult,
    NumpyDockPerception, OrientedBoundingBox2D, SyntheticDockConfig,
    filter_point_cloud, generate_synthetic_dock_point_cloud,
)
from .types import DockSpec, PlannerConfig, PlanResult, PlanStatus, Pose2D
from .color_sequence import (
    COLOR_SEQUENCE_MESSAGE_TYPE, COLOR_SEQUENCE_TOPIC, ColorCode,
    ColorSequenceVector, decode_color_sequence,
)
from .sprayer_interfaces import SprayerTargetObservation
from .watering import (
    WateringCompletionTracker, WateringSequenceController, WateringSequenceResult,
    WateringSequenceState, WateringVerificationConfig, WateringVerificationResult,
    WateringVerificationState,
)

__all__ = [
    "DockPlanner", "DockSpec", "PlannerConfig", "PlanResult", "PlanStatus", "Pose2D",
    "NEDPoint", "CoarseDockInfo", "CoarseDockCenterObservationNED", "CoarseDockObservation",
    "CoarseDockSearchConfig",
    "coarse_dock_search_radius_m", "coarse_dock_staging_pose", "PreciseEntrancePoseNED",
    "ActiveBayContext",
    "BayColorObservation", "ObstaclePointNED", "GenericVelocityCommand",
    "CameraEntranceCenter", "DockVisionFace", "DockVisionObservation",
    "DockVisionCoordinateConverter", "parse_dock_vision_json",
    "dock_vision_to_bay_color_observation",
    "DataFreshnessConfig", "DockPerceptionResult", "is_timestamp_fresh",
    "TerminalPlanInput", "prepare_terminal_plan_input", "entrance_observation_to_precise_pose",
    "GlobalApproachPlanner",
    "GlobalApproachResult", "GlobalApproachStatus", "DEFAULT_TOPIC", "DEFAULT_RATE_HZ",
    "RosTwistAdapter", "to_twist_message",
    "Task3FlowManager", "Task3FlowState",
    "CircularDockSearchConfig", "CircularDockSearchPlan", "plan_circular_dock_search",
    "DockPerceptionConfig", "DockOpeningCandidate",
    "DockPerceptionDebugResult", "NumpyDockPerception", "OrientedBoundingBox2D", "SyntheticDockConfig",
    "filter_point_cloud", "generate_synthetic_dock_point_cloud",
    "COLOR_SEQUENCE_TOPIC", "COLOR_SEQUENCE_MESSAGE_TYPE", "ColorCode",
    "ColorSequenceVector", "decode_color_sequence", "SprayerTargetObservation",
    "WateringCompletionTracker", "WateringSequenceController", "WateringSequenceResult",
    "WateringSequenceState", "WateringVerificationConfig", "WateringVerificationResult",
    "WateringVerificationState",
]
