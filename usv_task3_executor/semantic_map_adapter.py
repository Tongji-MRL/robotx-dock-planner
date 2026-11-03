"""Convert semantic-map mock dock centers into Task3 coarse-dock messages."""

import json
import math
from typing import Any


def _finite_number(value: Any, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field_name} must be a finite number")
    try:
        number = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{field_name} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{field_name} must be a finite number")
    return number


def parse_semantic_map_dock(
    data: str,
    dock_id: int,
    timestamp: float,
    frame_id: str = "mock_ned",
) -> dict[str, Any]:
    """Return one selected dock as an explicitly marked center-only mock."""

    try:
        payload = json.loads(data)
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid semantic map JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("semantic map JSON must be an object")

    objects = payload.get("objects")
    if not isinstance(objects, list):
        raise ValueError("objects must be an array")
    if isinstance(dock_id, bool) or not isinstance(dock_id, int):
        raise ValueError("dock_id parameter must be an integer")
    timestamp = _finite_number(timestamp, "timestamp")
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError("frame_id must be a nonempty string")

    matches = []
    for item in objects:
        if not isinstance(item, dict) or item.get("type") != "dock":
            continue
        object_id = item.get("id", item.get("dock_id"))
        if (
            isinstance(object_id, bool)
            or not isinstance(object_id, int)
            or object_id != dock_id
        ):
            continue
        matches.append(item)

    if not matches:
        raise ValueError(f"dock id {dock_id} was not found")
    if len(matches) != 1:
        raise ValueError(f"dock id {dock_id} matched multiple objects")

    coordinates = matches[0].get("NED_position")
    if not isinstance(coordinates, list) or len(coordinates) != 3:
        raise ValueError("NED_position must contain exactly three numbers")
    coordinates = [
        _finite_number(value, f"NED_position[{index}]")
        for index, value in enumerate(coordinates)
    ]

    return {
        "dock_id": str(dock_id),
        "center_ned": {
            "north": coordinates[0],
            "east": coordinates[1],
            "down": coordinates[2],
        },
        "timestamp": timestamp,
        "valid": True,
        "source": "semantic_map_mock",
        "mock": True,
        "frame_id": frame_id.strip(),
    }


def main(args=None) -> None:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SemanticMapMockAdapter(Node):
        def __init__(self) -> None:
            super().__init__("task3_semantic_map_mock_adapter")
            self.declare_parameter("dock_id", 0)
            self.declare_parameter("dry_run", True)
            self.declare_parameter("frame_id", "mock_ned")
            self.dock_id = self.get_parameter("dock_id").value
            self.dry_run = self.get_parameter("dry_run").value
            self.frame_id = str(self.get_parameter("frame_id").value)
            if isinstance(self.dock_id, bool) or not isinstance(self.dock_id, int):
                raise ValueError("dock_id ROS parameter must be an integer")

            self.coarse_dock_pub = self.create_publisher(
                String, "/tongji_mrl/twave/task3/coarse_dock", 10
            )
            self.semantic_map_sub = self.create_subscription(
                String, "/task/semantic_map", self._on_semantic_map, 10
            )
            if not self.dry_run:
                self.get_logger().warning(
                    "dry_run=false: semantic-map mock publishing is disabled."
                )

        def _on_semantic_map(self, msg: String) -> None:
            if not self.dry_run:
                return
            timestamp = self.get_clock().now().nanoseconds / 1_000_000_000.0
            try:
                observation = parse_semantic_map_dock(
                    msg.data, self.dock_id, timestamp, self.frame_id
                )
            except ValueError as exc:
                self.get_logger().warning(
                    f"Ignoring semantic_map message: {exc}"
                )
                return

            output = String()
            output.data = json.dumps(
                observation, ensure_ascii=True, separators=(",", ":")
            )
            self.coarse_dock_pub.publish(output)
            center = observation["center_ned"]
            self.get_logger().info(
                "Published semantic-map mock dock center "
                f"id={observation['dock_id']} "
                f"NED=({center['north']:.3f}, {center['east']:.3f}, "
                f"{center['down']:.3f}) frame={observation['frame_id']}"
            )

    rclpy.init(args=args)
    node = SemanticMapMockAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
