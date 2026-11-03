from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("dock_id", default_value="0"),
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("frame_id", default_value="mock_ned"),
            Node(
                package="usv_task3_executor",
                executable="task3_semantic_map_mock_adapter",
                output="screen",
                parameters=[
                    {
                        "dock_id": ParameterValue(
                            LaunchConfiguration("dock_id"), value_type=int
                        ),
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        ),
                        "frame_id": LaunchConfiguration("frame_id"),
                    }
                ],
            ),
        ]
    )
