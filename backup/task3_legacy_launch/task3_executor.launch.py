from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("target_color", default_value="GREEN"),
            DeclareLaunchArgument("enable_goto_output", default_value="false"),
            DeclareLaunchArgument("goto_target_topic", default_value="/safe_passage/goto/target"),
            DeclareLaunchArgument("goto_status_topic", default_value="/safe_passage/goto/status"),
            DeclareLaunchArgument("goto_odom_topic", default_value="/mavros/local_position/odom"),
            DeclareLaunchArgument("required_ned_frame_id", default_value="robotx_rtk_ned"),
            DeclareLaunchArgument("field_direct_dock", default_value="false"),
            DeclareLaunchArgument("field_coarse_center_ahead_m", default_value="4.0"),
            DeclareLaunchArgument("field_required_detections", default_value="3"),
            DeclareLaunchArgument("dock_led_timeout_s", default_value="0.75"),
            DeclareLaunchArgument("dock_led_required_detections", default_value="3"),
            Node(
                package="usv_task3_executor",
                executable="task3_executor",
                output="screen",
                parameters=[
                    {
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        ),
                        "target_color": LaunchConfiguration("target_color"),
                        "enable_goto_output": ParameterValue(
                            LaunchConfiguration("enable_goto_output"), value_type=bool
                        ),
                        "goto_target_topic": LaunchConfiguration("goto_target_topic"),
                        "goto_status_topic": LaunchConfiguration("goto_status_topic"),
                        "goto_odom_topic": LaunchConfiguration("goto_odom_topic"),
                        "required_ned_frame_id": LaunchConfiguration(
                            "required_ned_frame_id"
                        ),
                        "field_direct_dock": ParameterValue(
                            LaunchConfiguration("field_direct_dock"), value_type=bool
                        ),
                        "field_coarse_center_ahead_m": ParameterValue(
                            LaunchConfiguration("field_coarse_center_ahead_m"), value_type=float
                        ),
                        "field_required_detections": ParameterValue(
                            LaunchConfiguration("field_required_detections"), value_type=int
                        ),
                        "dock_led_timeout_s": ParameterValue(
                            LaunchConfiguration("dock_led_timeout_s"), value_type=float
                        ),
                        "dock_led_required_detections": ParameterValue(
                            LaunchConfiguration("dock_led_required_detections"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
