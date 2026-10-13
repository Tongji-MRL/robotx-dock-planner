"""Protected ROS adapter for the currently available Task 3 perception inputs."""

import math
from typing import Any

import rclpy
from rclpy.node import Node
from robotx_msgs.msg import DockingStatus, ResourceObservation
from std_msgs.msg import String


def parse_safe_bay(data: str) -> dict[str, Any]:
    """Parse the perception node's semicolon-delimited safe-bay string."""

    expected_fields = {"BAY", "COLOR", "X", "Y", "Z", "CONF"}
    fields: dict[str, str] = {}
    for part in data.split(";"):
        if "=" not in part:
            raise ValueError("each safe_bay field must use KEY=VALUE format")
        key, value = part.split("=", 1)
        if key in fields:
            raise ValueError(f"duplicate safe_bay field: {key}")
        fields[key] = value

    if set(fields) != expected_fields:
        missing = sorted(expected_fields - set(fields))
        extra = sorted(set(fields) - expected_fields)
        raise ValueError(f"safe_bay fields mismatch; missing={missing}, extra={extra}")

    bay = fields["BAY"]
    color = fields["COLOR"]
    if bay not in {"LEFT", "CENTER", "RIGHT", "UNKNOWN"}:
        raise ValueError(f"unsupported BAY value: {bay}")
    if color != "GREEN":
        raise ValueError(f"unsupported COLOR value: {color}")

    numbers = {}
    for source_name, result_name in (("X", "x"), ("Y", "y"), ("Z", "z"), ("CONF", "confidence")):
        try:
            number = float(fields[source_name])
        except ValueError as exc:
            raise ValueError(f"{source_name} must be numeric") from exc
        if not math.isfinite(number):
            raise ValueError(f"{source_name} must be finite")
        numbers[result_name] = number

    if not 0.0 <= numbers["confidence"] <= 1.0:
        raise ValueError("CONF must be between 0 and 1")

    return {
        "bay": bay,
        "color": color,
        "x_camera_m": numbers["x"],
        "y_camera_m": numbers["y"],
        "z_camera_m": numbers["z"],
        "confidence": numbers["confidence"],
        "frame": "camera optical frame; not NED",
    }


class Task3PerceptionAdapter(Node):
    """Record perception inputs without fabricating missing navigation data."""

    def __init__(self) -> None:
        super().__init__("task3_perception_adapter")
        self.declare_parameter("dry_run", True)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.latest_safe_bay: dict[str, Any] | None = None
        self.latest_alignment: dict[str, Any] | None = None
        self.latest_resource: dict[str, Any] | None = None

        self.safe_bay_sub = self.create_subscription(
            String,
            "/robotx/perception/docking/safe_bay",
            self._on_safe_bay,
            10,
        )
        self.alignment_sub = self.create_subscription(
            DockingStatus,
            "/robotx/perception/docking/alignment",
            self._on_alignment,
            10,
        )
        self.resources_sub = self.create_subscription(
            ResourceObservation,
            "/robotx/perception/resources",
            self._on_resource,
            10,
        )

        self.get_logger().warning(
            "Task3 adapter has no camera-to-NED transform, absolute entrance heading, "
            "or verified arrival input. It will not publish coarse_dock, "
            "dock_perception, or approach_reached."
        )
        if self.dry_run:
            self.get_logger().info("dry_run=true: parsing and recording inputs only.")
        else:
            self.get_logger().warning(
                "dry_run=false does not enable incomplete NED output; "
                "publication remains disabled until required interfaces exist."
            )

    def _on_safe_bay(self, msg: String) -> None:
        try:
            observation = parse_safe_bay(msg.data)
        except ValueError as exc:
            self.get_logger().warning(f"Ignoring malformed safe_bay message: {exc}")
            return

        self.latest_safe_bay = observation
        self.get_logger().info(
            "safe_bay parsed in camera optical frame (not NED): "
            f"BAY={observation['bay']} COLOR={observation['color']} "
            f"XYZ=({observation['x_camera_m']:.3f}, "
            f"{observation['y_camera_m']:.3f}, {observation['z_camera_m']:.3f}) m "
            f"CONF={observation['confidence']:.3f}; no Task3 geometry published."
        )

    def _on_alignment(self, msg: DockingStatus) -> None:
        status_names = {
            DockingStatus.STATUS_UNKNOWN: "UNKNOWN",
            DockingStatus.STATUS_SEARCHING: "SEARCHING",
            DockingStatus.STATUS_APPROACHING: "APPROACHING",
            DockingStatus.STATUS_ALIGNED: "ALIGNED",
            DockingStatus.STATUS_DOCKED: "DOCKED",
            DockingStatus.STATUS_ABORTED: "ABORTED",
        }
        stamp = msg.header.stamp
        timestamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        self.latest_alignment = {
            "status": status_names.get(int(msg.status), f"UNKNOWN_{int(msg.status)}"),
            "alignment_error_m": float(msg.alignment_error_m),
            "heading_error_rad": float(msg.heading_error_rad),
            "range_m": float(msg.range_m),
            "frame_id": msg.header.frame_id,
            "timestamp": timestamp,
        }
        self.get_logger().info(
            "alignment diagnostic only: "
            f"status={self.latest_alignment['status']} "
            f"alignment_error_m={self.latest_alignment['alignment_error_m']} "
            f"heading_error_rad={self.latest_alignment['heading_error_rad']} "
            f"range_m={self.latest_alignment['range_m']} "
            f"frame_id={self.latest_alignment['frame_id']!r} "
            f"timestamp={timestamp:.9f} (ROS Header stamp); "
            "ALIGNED is not approach_reached."
        )

    def _on_resource(self, msg: ResourceObservation) -> None:
        stamp = msg.header.stamp
        self.latest_resource = {
            "observation_id": msg.observation_id,
            "resource_id": msg.resource_id,
            "resource_type": int(msg.resource_type),
            "semantic_label": msg.semantic_label,
            "confidence": float(msg.confidence),
            "frame_id": msg.header.frame_id,
            "timestamp": float(stamp.sec) + float(stamp.nanosec) * 1e-9,
        }
        self.get_logger().info(
            "resource observation recorded without frame conversion: "
            f"semantic_label={msg.semantic_label} frame_id={msg.header.frame_id!r}; "
            "not used to construct dock geometry."
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Task3PerceptionAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()