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
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="mid360_base_link_static_tf",
            arguments=[
                "--x", "0.15",
                "--y", "0.0",
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
                "coarse_dock_topic": LaunchConfiguration("coarse_dock_topic"),
                "usv_pose_topic": LaunchConfiguration("usv_pose_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "process_period_s": LaunchConfiguration("process_period_s"),
                "coarse_timeout_s": LaunchConfiguration("coarse_timeout_s"),
                "pose_timeout_s": LaunchConfiguration("pose_timeout_s"),
                "roi_radius_m": LaunchConfiguration("roi_radius_m"),
                "min_confidence": LaunchConfiguration("min_confidence"),
            }],
            output="screen",
        ),
    ])
