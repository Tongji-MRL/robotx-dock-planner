"""Start Task 3 planning plus the new dock/window vision stack.

Motion, gimbal motion and the pump are disabled by default.  They require
explicit launch arguments during a supervised field run.
"""

import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _include(package: str, filename: str, *, condition=None, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", filename])
        ),
        condition=condition,
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    executor = _include(
        "usv_task3_executor",
        "task3_executor.launch.py",
        arguments={
            "dry_run": LaunchConfiguration("dry_run"),
            "enable_goto_output": LaunchConfiguration("enable_goto_output"),
            "target_color": LaunchConfiguration("target_color"),
            "required_ned_frame_id": LaunchConfiguration("required_ned_frame_id"),
        },
    )
    dock_indicator = _include(
        "robotx_usv_perception",
        "task3_dock_vm.launch.py",
        condition=IfCondition(LaunchConfiguration("start_dock_indicator")),
        arguments={
            "image_topic": "/oak/rgb/image_raw",
            "output_topic": "/robotx/perception/leds",
            "show_gui": LaunchConfiguration("show_gui"),
            "detector_python": LaunchConfiguration("detector_python"),
        },
    )
    sprayer = _include(
        "robotx_gimbal_interfaces",
        "task3_full.launch.py",
        condition=IfCondition(LaunchConfiguration("start_window_stack")),
        arguments={
            "show_gui": LaunchConfiguration("show_gui"),
            "start_camera": LaunchConfiguration("start_camera"),
            "start_perception": "true",
            "start_aiming": LaunchConfiguration("start_aiming"),
            "start_gimbal": LaunchConfiguration("start_gimbal"),
            "imu_feedforward": LaunchConfiguration("imu_feedforward"),
            "decode_task3_signal": "true",
            "start_pump": LaunchConfiguration("start_pump"),
            "auto_pump": LaunchConfiguration("auto_pump"),
            "bench_docked": LaunchConfiguration("bench_docked"),
            "docked_topic": "/robotx/task3/docked_confirmed",
        },
    )
    sequence_adapter = Node(
        package="usv_task3_executor",
        executable="task3_vision_contract_adapter",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_window_stack")),
        parameters=[{
            "fallback_window_id": ParameterValue(
                LaunchConfiguration("fallback_window_id"), value_type=int
            ),
            "green_confirmation_s": ParameterValue(
                LaunchConfiguration("green_confirmation_s"), value_type=float
            ),
        }],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("enable_goto_output", default_value="false"),
            DeclareLaunchArgument("target_color", default_value="GREEN"),
            DeclareLaunchArgument("required_ned_frame_id", default_value="robotx_rtk_ned"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_dock_indicator", default_value="true"),
            DeclareLaunchArgument("start_window_stack", default_value="true"),
            DeclareLaunchArgument("start_aiming", default_value="false"),
            DeclareLaunchArgument("start_gimbal", default_value="false"),
            DeclareLaunchArgument("imu_feedforward", default_value="false"),
            DeclareLaunchArgument("start_pump", default_value="false"),
            DeclareLaunchArgument("auto_pump", default_value="false"),
            DeclareLaunchArgument("bench_docked", default_value="false"),
            DeclareLaunchArgument("fallback_window_id", default_value="1"),
            DeclareLaunchArgument("green_confirmation_s", default_value="5.0"),
            DeclareLaunchArgument("show_gui", default_value="false"),
            DeclareLaunchArgument("detector_python", default_value=sys.executable),
            executor,
            sprayer,
            dock_indicator,
            sequence_adapter,
        ]
    )
