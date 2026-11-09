"""Start the Mid-360 driver and the measured base_link static transform."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_driver = LaunchConfiguration("start_driver")
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_roll = LaunchConfiguration("lidar_roll")
    lidar_pitch = LaunchConfiguration("lidar_pitch")
    lidar_yaw = LaunchConfiguration("lidar_yaw")
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("livox_ros_driver2"),
                "launch_ROS2",
                "msg_MID360_launch.py",
            ])
        ),
        condition=IfCondition(start_driver),
    )
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mid360_base_link_static_tf",
        arguments=[
            "--x", lidar_x,
            "--y", lidar_y,
            "--z", lidar_z,
            "--roll", lidar_roll,
            "--pitch", lidar_pitch,
            "--yaw", lidar_yaw,
            "--frame-id", "base_link",
            "--child-frame-id", "livox_frame",
        ],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "start_driver",
            default_value="true",
            description="Start the Livox Mid-360 driver in this launch.",
        ),
        DeclareLaunchArgument("lidar_x", default_value="0.15"),
        DeclareLaunchArgument("lidar_y", default_value="0.10"),
        DeclareLaunchArgument("lidar_z", default_value="0.22"),
        DeclareLaunchArgument("lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
        driver_launch,
        static_tf,
    ])
