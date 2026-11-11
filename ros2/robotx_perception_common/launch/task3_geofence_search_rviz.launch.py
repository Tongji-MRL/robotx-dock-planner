"""Open the synthetic Task3 search and its point cloud in RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("robotx_perception_common")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "task3_geofence_search_sim.launch.py")
        )),
        Node(
            package="rviz2",
            executable="rviz2",
            name="task3_geofence_search_rviz",
            arguments=[
                "-d",
                os.path.join(package_share, "rviz", "task3_geofence_search.rviz"),
            ],
            output="screen",
        ),
    ])
