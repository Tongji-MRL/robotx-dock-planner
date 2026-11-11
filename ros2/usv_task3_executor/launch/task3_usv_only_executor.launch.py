from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("enable_goto_output", default_value="false"),
        DeclareLaunchArgument("enable_aux_output", default_value="false"),
        DeclareLaunchArgument("auto_verify_stable_dock", default_value="true"),
        DeclareLaunchArgument("team_id", default_value="TONG"),
        DeclareLaunchArgument("vehicle_id", default_value="usv"),
        DeclareLaunchArgument("report_seq_start", default_value="0"),
        DeclareLaunchArgument("required_ned_frame_id", default_value="robotx_rtk_ned"),
        DeclareLaunchArgument("goto_target_topic", default_value="/safe_passage/goto/target"),
        DeclareLaunchArgument("goto_status_topic", default_value="/safe_passage/goto/status"),
        DeclareLaunchArgument("keep_out_detour_margin_m", default_value="0.5"),
        DeclareLaunchArgument("keep_out_arc_spacing_m", default_value="1.0"),
        DeclareLaunchArgument("goto_cancel_topic", default_value="/safe_passage/goto/cancel"),
        DeclareLaunchArgument(
            "gimbal_request_topic",
            default_value="/tongji_mrl/twave/task3/gimbal_request",
        ),
        DeclareLaunchArgument(
            "spray_request_topic",
            default_value="/tongji_mrl/twave/task3/spray_request",
        ),
        Node(
            package="usv_task3_executor",
            executable="task3_usv_only_executor",
            name="usv_task3_executor",
            output="screen",
            parameters=[{
                "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
                "enable_goto_output": ParameterValue(
                    LaunchConfiguration("enable_goto_output"), value_type=bool
                ),
                "enable_aux_output": ParameterValue(
                    LaunchConfiguration("enable_aux_output"), value_type=bool
                ),
                "auto_verify_stable_dock": ParameterValue(
                    LaunchConfiguration("auto_verify_stable_dock"), value_type=bool
                ),
                "team_id": LaunchConfiguration("team_id"),
                "vehicle_id": LaunchConfiguration("vehicle_id"),
                "report_seq_start": ParameterValue(
                    LaunchConfiguration("report_seq_start"), value_type=int
                ),
                "required_ned_frame_id": LaunchConfiguration("required_ned_frame_id"),
                "goto_target_topic": LaunchConfiguration("goto_target_topic"),
                "goto_status_topic": LaunchConfiguration("goto_status_topic"),
                "keep_out_detour_margin_m": ParameterValue(
                    LaunchConfiguration("keep_out_detour_margin_m"), value_type=float
                ),
                "keep_out_arc_spacing_m": ParameterValue(
                    LaunchConfiguration("keep_out_arc_spacing_m"), value_type=float
                ),
                "goto_cancel_topic": LaunchConfiguration("goto_cancel_topic"),
                "gimbal_request_topic": LaunchConfiguration("gimbal_request_topic"),
                "spray_request_topic": LaunchConfiguration("spray_request_topic"),
            }],
        ),
    ])
