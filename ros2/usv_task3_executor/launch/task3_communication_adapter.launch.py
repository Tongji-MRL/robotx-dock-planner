from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("run_id", default_value="0"),
        DeclareLaunchArgument("heartbeat_timeout_s", default_value="3.0"),
        DeclareLaunchArgument("heartbeat_startup_grace_s", default_value="5.0"),
        Node(
            package="usv_task3_executor",
            executable="task3_communication_adapter",
            name="task3_communication_adapter",
            output="screen",
            parameters=[{
                "run_id": LaunchConfiguration("run_id"),
                "heartbeat_timeout_s": LaunchConfiguration("heartbeat_timeout_s"),
                "heartbeat_startup_grace_s": LaunchConfiguration(
                    "heartbeat_startup_grace_s"
                ),
            }],
        ),
    ])
