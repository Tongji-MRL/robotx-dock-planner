"""Run the synthetic geofence search scene through production perception."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robotx_perception_common",
            executable="task3_geofence_search_simulator",
            name="task3_geofence_search_simulator",
            output="screen",
        ),
        Node(
            package="robotx_perception_common",
            executable="task3_lidar_perception",
            name="task3_lidar_perception",
            output="screen",
            parameters=[{
                "input_topic": "/task3/sim/livox/lidar",
                "input_format": "pointcloud2",
                "usv_pose_topic": "/tongji_mrl/twave/ned_position",
                "coarse_dock_topic": "/tongji_mrl/twave/task3/coarse_dock",
                "output_topic": "/tongji_mrl/twave/task3/dock_perception",
                "detector_mode": "unseeded_lines",
                "process_period_s": 0.2,
                "pose_timeout_s": 2.0,
                "local_min_confidence": 0.30,
                "local_z_min_m": -0.35,
                "local_z_max_m": 0.85,
                "local_cluster_distance_m": 0.20,
                "local_min_wall_length_m": 0.65,
                "local_lateral_half_width_m": 12.0,
                "local_coarse_roi_radius_m": 12.0,
                "local_max_candidates": 6,
                "unseeded_history_frames": 6,
                "unseeded_required_detections": 3,
                "unseeded_min_entrances": 3,
                "unseeded_association_distance_m": 0.75,
                "unseeded_heading_tolerance_rad": 0.35,
                "unseeded_dock_depth_m": 6.0,
                "unseeded_observation_standoff_m": 4.0,
                "required_ned_frame_id": "robotx_rtk_ned",
                "diagnostic_log_period_s": 1.0,
            }],
        ),
    ])
