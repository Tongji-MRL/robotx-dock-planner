"""Start the shared reactive avoidance monitor without taking control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scan_topic", default_value="/mavros/obstacle/send"),
        DeclareLaunchArgument(
            "output_topic", default_value="/tongji_mrl/twave/navigation/avoidance"
        ),
        Node(
            package="usv_task3_executor",
            executable="usv_reactive_avoidance",
            output="screen",
            parameters=[{
                "scan_topic": LaunchConfiguration("scan_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
            }],
        ),
    ])
