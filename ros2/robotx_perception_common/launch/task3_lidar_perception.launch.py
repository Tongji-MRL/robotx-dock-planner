"""Start the measured TF and the Task3 LiDAR perception bridge.

The Livox driver is intentionally not started here.  Start it separately with
the official launch file, then use this launch to avoid duplicate publishers.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("input_format", default_value="pointcloud2"),
        DeclareLaunchArgument(
            "coarse_dock_topic",
            default_value="/tongji_mrl/twave/task3/coarse_dock",
        ),
        DeclareLaunchArgument(
            "usv_pose_topic",
            default_value="/tongji_mrl/twave/ned_position",
        ),
        DeclareLaunchArgument(
            "output_topic",
            default_value="/tongji_mrl/twave/task3/dock_perception",
        ),
        DeclareLaunchArgument("process_period_s", default_value="0.5"),
        DeclareLaunchArgument("coarse_timeout_s", default_value="30.0"),
        DeclareLaunchArgument("pose_timeout_s", default_value="2.0"),
        DeclareLaunchArgument("roi_radius_m", default_value="25.0"),
        DeclareLaunchArgument("min_confidence", default_value="0.20"),
        DeclareLaunchArgument("detector_mode", default_value="seeded_lines"),
        DeclareLaunchArgument("local_min_confidence", default_value="0.30"),
        DeclareLaunchArgument("local_z_min_m", default_value="-0.35"),
        DeclareLaunchArgument("local_z_max_m", default_value="0.85"),
        DeclareLaunchArgument("local_cluster_distance_m", default_value="0.28"),
        DeclareLaunchArgument("local_min_wall_length_m", default_value="0.65"),
        DeclareLaunchArgument("local_lateral_half_width_m", default_value="3.0"),
        DeclareLaunchArgument("local_coarse_roi_radius_m", default_value="6.0"),
        DeclareLaunchArgument("diagnostic_log_period_s", default_value="2.0"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="mid360_base_link_static_tf",
            arguments=[
                "--x", "0.15",
                "--y", "0.10",
                "--z", "0.22",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "base_link",
                "--child-frame-id", "livox_frame",
            ],
            output="screen",
        ),
        Node(
            package="robotx_perception_common",
            executable="task3_lidar_perception",
            name="task3_lidar_perception",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "input_format": LaunchConfiguration("input_format"),
                "coarse_dock_topic": LaunchConfiguration("coarse_dock_topic"),
                "usv_pose_topic": LaunchConfiguration("usv_pose_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "process_period_s": LaunchConfiguration("process_period_s"),
                "coarse_timeout_s": LaunchConfiguration("coarse_timeout_s"),
                "pose_timeout_s": LaunchConfiguration("pose_timeout_s"),
                "roi_radius_m": LaunchConfiguration("roi_radius_m"),
                "min_confidence": LaunchConfiguration("min_confidence"),
                "detector_mode": LaunchConfiguration("detector_mode"),
                "local_min_confidence": LaunchConfiguration("local_min_confidence"),
                "local_z_min_m": LaunchConfiguration("local_z_min_m"),
                "local_z_max_m": LaunchConfiguration("local_z_max_m"),
                "local_cluster_distance_m": LaunchConfiguration(
                    "local_cluster_distance_m"
                ),
                "local_min_wall_length_m": LaunchConfiguration(
                    "local_min_wall_length_m"
                ),
                "local_lateral_half_width_m": LaunchConfiguration(
                    "local_lateral_half_width_m"
                ),
                "local_coarse_roi_radius_m": LaunchConfiguration(
                    "local_coarse_roi_radius_m"
                ),
                "diagnostic_log_period_s": LaunchConfiguration(
                    "diagnostic_log_period_s"
                ),
            }],
            output="screen",
        ),
    ])
