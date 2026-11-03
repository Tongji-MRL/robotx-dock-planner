"""ROS-independent DockPlanner V0.1 and Task3 integration contracts."""

from .planner import DockPlanner
from .integration import (
    ActiveBayContext, BayColorObservation, CoarseDockCenterObservationNED,
    CoarseDockCirclePosition, CoarseDockInfo,
    CoarseDockObservation,
    CoarseDockSearchConfig, DockEntranceObservationNED,
    CameraEntranceCenter, DataFreshnessConfig, DockPerceptionResult,
    DockVisionCoordinateConverter, DockVisionFace, DockVisionObservation,
    GlobalApproachPlanner,
    GlobalApproachResult, GenericVelocityCommand,
    GlobalApproachStatus, NEDPoint, ObstaclePointNED, PreciseEntrancePoseNED,
    ResourceColorSequenceObservation, TerminalPlanInput,
    classify_coarse_dock_circle_position, coarse_dock_search_radius_m,
    coarse_dock_staging_pose,
    dock_vision_to_bay_color_observation,
    entrance_observation_to_precise_pose, is_timestamp_fresh,
    parse_dock_vision_json, parse_logistics_request_json, prepare_terminal_plan_input,
)
from .ros_twist_adapter import DEFAULT_RATE_HZ, DEFAULT_TOPIC, RosTwistAdapter, to_twist_message
from .task3_flow import Task3FlowManager, Task3FlowState, Task3RetryConfig
from .dock_search import (
    CircularDockSearchConfig, CircularDockSearchPlan, plan_circular_dock_search,
)
from .dock_perception import (
    DockOpeningCandidate, DockPerceptionConfig, DockPerceptionDebugResult,
    NumpyDockPerception, OrientedBoundingBox2D, SyntheticDockConfig,
    filter_point_cloud, generate_synthetic_dock_point_cloud,
)
from .types import DockSpec, PlannerConfig, PlanResult, PlanStatus, Pose2D
from .watering import (
    WateringCompletionTracker, WateringVerificationConfig,
    WateringSequenceController, WateringSequenceResult, WateringSequenceState,
    WateringVerificationResult, WateringVerificationState,
)
from .mission_safety import (
    AdaptiveBoundarySearchPlan, AdaptiveSearchRadiusConfig,
    BayExitPlan, BoundaryCheckedSearchPlan, BoundaryPathValidation,
    DockAttemptSupervisor, DockRetryConfig,
    ExitAndReturnPlan, NavigableBoundaryNED, RecoveryAction, RecoveryDecision,
    SafeReturnGoalConfig,
    SearchBoundaryConfig, constrain_search_plan_to_boundary,
    plan_adaptive_boundary_safe_circular_search,
    plan_boundary_safe_circular_search, plan_exit_and_return,
    plan_non_target_bay_exit, select_safe_return_goal,
    validate_pose_path_to_boundary, validate_terminal_plan_to_boundary,
)
from .goto_adapter import (
    GotoReferenceSample, GotoStatus, GotoStatusKind, GotoTargetENU,
    ned_goal_to_goto_enu, parse_goto_status,
)
from .control_arbiter import (
    MotionCommandArbiter, MotionCommandArbiterConfig, MotionCommandDecision,
    MotionCommandOwner,
)
from .avoidance_contract import AvoidanceAction, ObstacleAvoidanceDecision
from .pointcloud_adapter import (
    PointCloud2Buffer, PointCloudXYZQuality, PointFieldSpec,
    pointcloud2_buffer_to_xyz, pointcloud2_message_to_xyz,
    summarize_xyz_quality,
)
from .lidar_transform import (
    LidarExtrinsics, default_mid360_extrinsics,
    pointcloud_message_to_base_link_xyz,
    transform_base_link_points_to_lidar,
    transform_lidar_point_to_base_link,
    transform_lidar_points_to_base_link,
)
from .ned_map_adapter import (
    GpsOrigin, NedOffset, add_ned_offset, geodetic_to_ecef,
    semantic_map_to_coarse_docks, transform_semantic_map,
    uav_origin_in_usv_ned,
)
from .color_sequence import (
    COLOR_SEQUENCE_MESSAGE_TYPE, COLOR_SEQUENCE_TOPIC, ColorCode,
    ColorSequenceVector, decode_color_sequence,
)
from .sprayer_interfaces import SprayerTargetObservation
from .bay_search import (
    BayCandidateAction, BayCandidateDecision, BayDisposition,
    BayVisitConfig, BayVisitRecord, BayVisitTracker,
)
from .task3_status_codec import (
    CommonError, PlannerCommand, Task3Step, Task4Step, TaskCode,
    TaskStatusCode, VehicleCode,
)
from .task3_subfsm_bridge import (
    MasterCommand, MasterCommandType, ResumePolicy, SubFSMReport,
    Task3Checkpoint, Task3SubFSMBridge,
)
from .task3_ros_contract import (
    CorrectBayInfo, HeartbeatMonitor, SprayerWindowInfo,
    Task3RosMessageFactory, normalize_task_command,
    validate_vehicle_heartbeat,
)
from .fsm_contract import (
    FSM_TASK3_FINISH_PROGRESS,
    FSM_PROGRESS_REQUIRED_FIELDS,
    TASK3_WIRE_PROGRESS,
    correct_bay_to_fsm_progress,
    docking_result_to_fsm_message,
    executor_state_to_fsm_progress,
    firefighting_result_to_fsm_message,
    normalize_fsm_task_command,
    sprayer_window_to_fsm_progress,
    safe_stop_ack_progress,
    validate_fsm_progress,
)

__all__ = [
    "DockPlanner", "DockSpec", "PlannerConfig", "PlanResult", "PlanStatus", "Pose2D",
    "NEDPoint", "CoarseDockInfo", "CoarseDockCenterObservationNED",
    "CoarseDockCirclePosition", "CoarseDockObservation", "CoarseDockSearchConfig",
    "classify_coarse_dock_circle_position", "coarse_dock_search_radius_m",
    "coarse_dock_staging_pose", "PreciseEntrancePoseNED",
    "ActiveBayContext",
    "BayColorObservation", "ObstaclePointNED", "GenericVelocityCommand",
    "ResourceColorSequenceObservation", "parse_logistics_request_json",
    "SprayerTargetObservation",
    "CameraEntranceCenter", "DockVisionFace", "DockVisionObservation",
    "DockVisionCoordinateConverter", "parse_dock_vision_json",
    "dock_vision_to_bay_color_observation",
    "DataFreshnessConfig", "DockPerceptionResult", "is_timestamp_fresh",
    "TerminalPlanInput", "prepare_terminal_plan_input", "entrance_observation_to_precise_pose",
    "GlobalApproachPlanner",
    "GlobalApproachResult", "GlobalApproachStatus", "DEFAULT_TOPIC", "DEFAULT_RATE_HZ",
    "RosTwistAdapter", "to_twist_message",
    "Task3FlowManager", "Task3FlowState", "Task3RetryConfig",
    "CircularDockSearchConfig", "CircularDockSearchPlan", "plan_circular_dock_search",
    "DockPerceptionConfig", "DockOpeningCandidate",
    "DockPerceptionDebugResult", "NumpyDockPerception", "OrientedBoundingBox2D", "SyntheticDockConfig",
    "filter_point_cloud", "generate_synthetic_dock_point_cloud",
    "WateringCompletionTracker", "WateringVerificationConfig",
    "WateringVerificationResult", "WateringVerificationState",
    "WateringSequenceController", "WateringSequenceResult", "WateringSequenceState",
    "AdaptiveBoundarySearchPlan", "AdaptiveSearchRadiusConfig",
    "BayExitPlan",
    "BoundaryCheckedSearchPlan", "BoundaryPathValidation",
    "DockAttemptSupervisor", "DockRetryConfig",
    "ExitAndReturnPlan", "NavigableBoundaryNED", "RecoveryAction",
    "RecoveryDecision", "SafeReturnGoalConfig", "SearchBoundaryConfig",
    "constrain_search_plan_to_boundary", "plan_adaptive_boundary_safe_circular_search",
    "plan_boundary_safe_circular_search",
    "plan_exit_and_return",
    "plan_non_target_bay_exit",
    "select_safe_return_goal",
    "validate_pose_path_to_boundary", "validate_terminal_plan_to_boundary",
    "GotoReferenceSample", "GotoStatus", "GotoStatusKind", "GotoTargetENU",
    "ned_goal_to_goto_enu", "parse_goto_status",
    "MotionCommandArbiter", "MotionCommandArbiterConfig",
    "MotionCommandDecision", "MotionCommandOwner",
    "AvoidanceAction", "ObstacleAvoidanceDecision",
    "PointCloud2Buffer", "PointCloudXYZQuality", "PointFieldSpec",
    "pointcloud2_buffer_to_xyz", "pointcloud2_message_to_xyz",
    "summarize_xyz_quality",
    "LidarExtrinsics", "default_mid360_extrinsics",
    "pointcloud_message_to_base_link_xyz",
    "transform_lidar_points_to_base_link", "transform_lidar_point_to_base_link",
    "transform_base_link_points_to_lidar",
    "GpsOrigin", "NedOffset", "add_ned_offset", "geodetic_to_ecef",
    "semantic_map_to_coarse_docks", "transform_semantic_map",
    "uav_origin_in_usv_ned",
    "COLOR_SEQUENCE_TOPIC", "COLOR_SEQUENCE_MESSAGE_TYPE", "ColorCode",
    "ColorSequenceVector", "decode_color_sequence",
    "BayCandidateAction", "BayCandidateDecision", "BayDisposition",
    "BayVisitConfig", "BayVisitRecord", "BayVisitTracker",
    "CommonError", "PlannerCommand", "Task3Step", "Task4Step", "TaskCode",
    "TaskStatusCode", "VehicleCode",
    "MasterCommand", "MasterCommandType", "ResumePolicy", "SubFSMReport",
    "Task3Checkpoint", "Task3SubFSMBridge",
    "CorrectBayInfo", "HeartbeatMonitor", "SprayerWindowInfo",
    "Task3RosMessageFactory", "normalize_task_command",
    "validate_vehicle_heartbeat",
    "FSM_TASK3_FINISH_PROGRESS", "FSM_PROGRESS_REQUIRED_FIELDS", "TASK3_WIRE_PROGRESS",
    "normalize_fsm_task_command", "executor_state_to_fsm_progress",
    "correct_bay_to_fsm_progress", "docking_result_to_fsm_message",
    "sprayer_window_to_fsm_progress", "firefighting_result_to_fsm_message",
    "safe_stop_ack_progress", "validate_fsm_progress",
]
