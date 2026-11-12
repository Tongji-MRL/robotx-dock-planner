from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic", default_value="/mavros/local_position/odom"
            ),
            DeclareLaunchArgument(
                "output_topic", default_value="/tongji_mrl/twave/ned_position"
            ),
            DeclareLaunchArgument(
                "output_frame_id", default_value="usv_fcu_local_ned"
            ),
            Node(
                package="usv_task3_executor",
                executable="task3_ned_pose_adapter",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "output_frame_id": LaunchConfiguration("output_frame_id"),
                    }
                ],
            ),
        ]
    )
