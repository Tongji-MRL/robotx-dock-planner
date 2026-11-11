"""Run FAST-LIO deskewing and feed its current body-frame scan to Task3."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("robotx_perception_common")
    fast_lio_share = FindPackageShare("fast_lio")

    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fast_lio_share, "launch", "mapping.launch.py"])
        ),
        launch_arguments={
            "config_path": PathJoinSubstitution([package_share, "config"]),
            "config_file": "task3_fast_lio_mid360_pointcloud2.yaml",
            "rviz": "false",
            "use_sim_time": "false",
        }.items(),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "task3_lidar_perception.launch.py"]
            )
        ),
        launch_arguments={
            "input_topic": "/cloud_registered_body",
            "input_format": "pointcloud2",
            "input_frame_mode": "sensor",
            "expected_input_frame": "body",
            # /cloud_registered_body is in the Mid-360 internal IMU frame.
            # Convert it to the measured USV base_link installation point.
            "input_to_base_x_m": "0.161",
            "input_to_base_y_m": "0.12329",
            "input_to_base_z_m": "0.17588",
            "detector_mode": "unseeded_lines",
            "local_max_candidates": LaunchConfiguration("local_max_candidates"),
            "local_max_range_m": LaunchConfiguration("local_max_range_m"),
            "local_lateral_half_width_m": LaunchConfiguration(
                "local_lateral_half_width_m"
            ),
            "unseeded_required_detections": LaunchConfiguration(
                "unseeded_required_detections"
            ),
            "unseeded_min_entrances": LaunchConfiguration(
                "unseeded_min_entrances"
            ),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("local_max_candidates", default_value="6"),
        DeclareLaunchArgument("local_max_range_m", default_value="15.0"),
        DeclareLaunchArgument("local_lateral_half_width_m", default_value="15.0"),
        DeclareLaunchArgument("unseeded_required_detections", default_value="3"),
        DeclareLaunchArgument("unseeded_min_entrances", default_value="3"),
        fast_lio,
        perception,
    ])
