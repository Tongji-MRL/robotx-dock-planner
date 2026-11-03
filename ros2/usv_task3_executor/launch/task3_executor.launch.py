from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("enable_goto_output", default_value="false"),
            DeclareLaunchArgument("goto_target_topic", default_value="/safe_passage/goto/target"),
            DeclareLaunchArgument("goto_status_topic", default_value="/safe_passage/goto/status"),
            DeclareLaunchArgument("goto_odom_topic", default_value="/mavros/local_position/odom"),
            Node(
                package="usv_task3_executor",
                executable="task3_executor",
                output="screen",
                parameters=[
                    {
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        ),
                        "enable_goto_output": ParameterValue(
                            LaunchConfiguration("enable_goto_output"), value_type=bool
                        ),
                        "goto_target_topic": LaunchConfiguration("goto_target_topic"),
                        "goto_status_topic": LaunchConfiguration("goto_status_topic"),
                        "goto_odom_topic": LaunchConfiguration("goto_odom_topic"),
                    }
                ],
            ),
        ]
    )
