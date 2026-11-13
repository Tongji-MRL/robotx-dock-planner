from glob import glob
from setuptools import setup

package_name = "usv_task3_executor"
setup(
    name=package_name,
    version="0.1.0",
    # Keep the ROS wrapper here while installing the planner directly from the
    # repository root.  This avoids maintaining a second copy of dock_planner
    # inside the ROS package.
    packages=[
        "usv_task3_executor", "dock_planner", "usv_reactive_avoidance",
        "usv_only_task3",
    ],
    package_dir={
        "usv_task3_executor": "../../usv_task3_executor",
        "dock_planner": "../../dock_planner",
        "usv_reactive_avoidance": "../../usv_reactive_avoidance",
        "usv_only_task3": "../../usv_only_task3",
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "task3_executor = usv_task3_executor.node:main",
            "task3_usv_only_executor = usv_only_task3.node:main",
            "task3_perception_adapter = usv_task3_executor.perception_adapter:main",
            "task3_semantic_map_mock_adapter = usv_task3_executor.semantic_map_adapter:main",
            "task3_communication_adapter = usv_task3_executor.communication_adapter:main",
            "task3_ned_pose_adapter = usv_task3_executor.ned_pose_adapter:main",
            "task3_vision_contract_adapter = usv_task3_executor.vision_contract_adapter:main",
            "task3_usv_only_contract_adapter = usv_task3_executor.usv_only_contract_adapter:main",
            "usv_reactive_avoidance = usv_task3_executor.reactive_avoidance_node:main",
        ]
    },
)
