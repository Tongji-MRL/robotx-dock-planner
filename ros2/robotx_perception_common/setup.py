from setuptools import find_packages, setup


package_name = "robotx_perception_common"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", [
            "launch/task3_geofence_search_sim.launch.py",
            "launch/task3_geofence_search_rviz.launch.py",
            "launch/task3_lidar_bringup.launch.py",
            "launch/task3_lidar_perception.launch.py",
            "launch/task3_lio_perception.launch.py",
        ]),
        (f"share/{package_name}/config", [
            "config/task3_fast_lio_mid360_pointcloud2.yaml",
        ]),
        (f"share/{package_name}/rviz", [
            "rviz/task3_geofence_search.rviz",
        ]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="RobotX team",
    maintainer_email="robotx@todo.invalid",
    description="Shared Mid-360 PointCloud2/CustomMsg parsing and base_link conversion",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "task3_lidar_perception = robotx_perception_common.task3_lidar_perception_node:main",
            "task3_geofence_search_simulator = robotx_perception_common.task3_geofence_search_simulator_node:main",
        ],
    },
)
