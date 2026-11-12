"""Start the real USV-only Task 3 navigation and dock-search core.

The team state machine remains the owner of RTK NED, geofence and task
commands.  This launch starts the Orin-side hardware/data path, MAVROS,
FAST-LIO, dock entrance perception, waypoint controller and Task 3 executor.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
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
    start_obstacle_stack = LaunchConfiguration("start_obstacle_stack")
    start_mavros = LaunchConfiguration("start_mavros")
    start_fast_lio = LaunchConfiguration("start_fast_lio")
    control_enabled = LaunchConfiguration("control_enabled")

    obstacle_stack = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [EnvironmentVariable("HOME"), "start_obstacle_stack.sh"]
            ),
            "start",
        ],
        output="screen",
        condition=IfCondition(start_obstacle_stack),
    )

    mavros = Node(
        package="mavros",
        executable="mavros_node",
        name="mavros_node",
        output="screen",
        condition=IfCondition(start_mavros),
        parameters=[{"fcu_url": LaunchConfiguration("fcu_url")}],
    )

    fast_lio = _include(
        "fast_lio",
        "task3_mid360.launch.py",
        condition=IfCondition(start_fast_lio),
    )

    perception = _include(
        "robotx_perception_common",
        "task3_lio_perception.launch.py",
        arguments={
            "start_fast_lio": "false",
            "fast_lio_body_cloud_topic": "/fast_lio/cloud_registered_body",
            "fast_lio_body_frame": "fast_lio_body",
        },
    )

    goto = Node(
        package="safe_passage",
        executable="position_waypoint_goto",
        name="position_waypoint_goto",
        output="screen",
        parameters=[{
            "auto_home_south": False,
            "actuation_enabled": ParameterValue(control_enabled, value_type=bool),
            "frame_mode": "map",
            "odom_topic": "/mavros/local_position/odom",
            "mavros_state_topic": "/mavros/state",
            "target_topic": "/safe_passage/goto/target",
            "abort_topic": "/safe_passage/goto/abort",
            "status_topic": "/safe_passage/goto/status",
            "scan_topic": "/mavros/obstacle/send",
            "require_scan": True,
            "expected_odom_frame": "map",
            "max_target_distance": ParameterValue(
                LaunchConfiguration("max_target_distance_m"), value_type=float
            ),
            "max_duration": ParameterValue(
                LaunchConfiguration("goto_max_duration_s"), value_type=float
            ),
        }],
    )

    avoidance_monitor = _include(
        "usv_task3_executor",
        "task3_reactive_avoidance.launch.py",
    )

    executor = _include(
        "usv_task3_executor",
        "task3_usv_only_executor.launch.py",
        arguments={
            "dry_run": LaunchConfiguration("dry_run"),
            "enable_goto_output": control_enabled,
            "enable_aux_output": LaunchConfiguration("enable_aux_output"),
            "required_ned_frame_id": LaunchConfiguration("required_ned_frame_id"),
            "goto_cancel_topic": "/safe_passage/goto/abort",
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_obstacle_stack", default_value="true"),
        DeclareLaunchArgument("start_mavros", default_value="true"),
        DeclareLaunchArgument("start_fast_lio", default_value="true"),
        DeclareLaunchArgument("control_enabled", default_value="false"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("enable_aux_output", default_value="false"),
        DeclareLaunchArgument("required_ned_frame_id", default_value="robotx_rtk_ned"),
        DeclareLaunchArgument("max_target_distance_m", default_value="30.0"),
        DeclareLaunchArgument("goto_max_duration_s", default_value="120.0"),
        DeclareLaunchArgument(
            "fcu_url",
            default_value=(
                "serial:///dev/serial/by-id/"
                "usb-Holybro_Pixhawk6C_38002E001851333531333639-if02:57600"
            ),
        ),
        obstacle_stack,
        mavros,
        fast_lio,
        perception,
        goto,
        avoidance_monitor,
        executor,
    ])
