from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("dry_run", default_value="true"),
            Node(
                package="usv_task3_executor",
                executable="task3_executor",
                output="screen",
                parameters=[
                    {
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        )
                    }
                ],
            ),
        ]
    )
