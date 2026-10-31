from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("platform_id", default_value="0"),
        Node(
            package="usv_task3_executor",
            executable="task3_semantic_map_adapter",
            output="screen",
            parameters=[{
                "platform_id": ParameterValue(
                    LaunchConfiguration("platform_id"), value_type=int
                ),
            }],
        ),
    ])
