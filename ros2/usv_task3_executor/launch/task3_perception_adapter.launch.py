from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("dry_run", default_value="true"),
            Node(
                package="usv_task3_executor",
                executable="task3_perception_adapter",
                name="task3_perception_adapter",
                output="screen",
                parameters=[{"dry_run": LaunchConfiguration("dry_run")}],
            ),
        ]
    )