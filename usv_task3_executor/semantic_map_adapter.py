"""Convert shared semantic-map snapshots into Task3 internal inputs."""

from __future__ import annotations

import json
import math
from typing import Any

from robotx_semantic_map import parse_snapshot, select_objects_by_type


def _finite_number(value: Any, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field_name} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{field_name} must be a finite number")
    return number


def build_platform_report(data: str) -> dict[str, Any]:
    selected = select_objects_by_type(data, "platform")
    return {
        "task_id": 3,
        "data_kind": "platforms",
        "map_epoch": selected["map_epoch"],
        "version": selected["version"],
        "objects": selected["objects"],
    }


def parse_semantic_map_dock(
    data: str, dock_id: int, timestamp: float
) -> dict[str, Any]:
    """Select one platform/dock center already expressed in common NED."""

    payload = parse_snapshot(data)
    if isinstance(dock_id, bool) or not isinstance(dock_id, int):
        raise ValueError("dock_id parameter must be an integer")
    timestamp = _finite_number(timestamp, "timestamp")
    candidates = []
    for object_type in ("platform", "dock"):
        candidates.extend(select_objects_by_type(payload, object_type)["objects"])
    matches = []
    for item in candidates:
        object_id = item.get("id", item.get("dock_id"))
        if isinstance(object_id, bool):
            continue
        try:
            matches_id = int(object_id) == dock_id
        except (TypeError, ValueError):
            matches_id = False
        if matches_id:
            matches.append(item)

    if not matches:
        raise ValueError(f"dock/platform id {dock_id} was not found")
    if len(matches) != 1:
        raise ValueError(f"dock/platform id {dock_id} matched multiple objects")

    coordinates = matches[0].get("NED_position")
    if not isinstance(coordinates, list) or len(coordinates) != 3:
        raise ValueError("NED_position must contain exactly three numbers")
    coordinates = [
        _finite_number(value, f"NED_position[{index}]")
        for index, value in enumerate(coordinates)
    ]
    confidence = matches[0].get("confidence")
    return {
        "dock_id": str(dock_id),
        "center_ned": {
            "north": coordinates[0],
            "east": coordinates[1],
            "down": coordinates[2],
        },
        "timestamp": timestamp,
        "valid": True,
        "confidence": (
            None if confidence is None else _finite_number(confidence, "confidence")
        ),
        "source": "semantic_map",
        "mock": False,
        "frame_id": "ned",
        "map_epoch": payload.get("map_epoch", "default"),
        "map_version": payload.get("version", 0),
    }


def main(args=None) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    class SemanticMapAdapter(Node):
        def __init__(self) -> None:
            super().__init__("task3_semantic_map_adapter")
            self.declare_parameter("platform_id", 0)
            self.declare_parameter("input_topic", "/task/semantic_map")
            self.declare_parameter(
                "platform_topic", "/tongji_mrl/twave/task3/platforms"
            )
            self.declare_parameter(
                "coarse_dock_topic", "/tongji_mrl/twave/task3/coarse_dock"
            )
            self.platform_id = self.get_parameter("platform_id").value
            if isinstance(self.platform_id, bool) or not isinstance(self.platform_id, int):
                raise ValueError("platform_id ROS parameter must be an integer")
            self.platform_pub = self.create_publisher(
                String, str(self.get_parameter("platform_topic").value), 10
            )
            self.coarse_dock_pub = self.create_publisher(
                String, str(self.get_parameter("coarse_dock_topic").value), 10
            )
            map_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.semantic_map_sub = self.create_subscription(
                String,
                str(self.get_parameter("input_topic").value),
                self._on_semantic_map,
                map_qos,
            )
            self.last_snapshot: tuple[str, int] | None = None

        def _on_semantic_map(self, message: String) -> None:
            try:
                platforms = build_platform_report(message.data)
                snapshot = (
                    str(platforms["map_epoch"]),
                    int(platforms["version"]),
                )
                if snapshot == self.last_snapshot:
                    return
                self.last_snapshot = snapshot
                timestamp = self.get_clock().now().nanoseconds / 1_000_000_000.0
                coarse = parse_semantic_map_dock(
                    message.data, self.platform_id, timestamp
                )
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                self.get_logger().warning(f"Ignoring semantic-map snapshot: {exc}")
                return

            platform_message = String()
            platform_message.data = json.dumps(
                platforms, ensure_ascii=True, separators=(",", ":")
            )
            self.platform_pub.publish(platform_message)
            coarse_message = String()
            coarse_message.data = json.dumps(
                coarse, ensure_ascii=True, separators=(",", ":")
            )
            self.coarse_dock_pub.publish(coarse_message)

    rclpy.init(args=args)
    node = SemanticMapAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


__all__ = ["build_platform_report", "parse_semantic_map_dock"]
