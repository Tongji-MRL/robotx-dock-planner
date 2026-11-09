"""Convert MAVROS ENU odometry into the Task 3 NED pose contract."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Mapping


@dataclass(frozen=True)
class NEDPoseContract:
    north_m: float
    east_m: float
    down_m: float
    heading_rad: float
    timestamp: float
    sequence: int
    frame_id: str


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def normalize_angle(angle_rad: float) -> float:
    """Normalize an angle to [-pi, pi)."""

    angle = _finite_number(angle_rad, "angle_rad")
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return ROS ENU yaw from a finite quaternion."""

    qx = _finite_number(x, "quaternion.x")
    qy = _finite_number(y, "quaternion.y")
    qz = _finite_number(z, "quaternion.z")
    qw = _finite_number(w, "quaternion.w")
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise ValueError("quaternion must have nonzero norm")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(sin_yaw, cos_yaw)


def enu_odometry_to_ned_payload(
    *,
    east_m: float,
    north_m: float,
    up_m: float,
    quaternion_x: float,
    quaternion_y: float,
    quaternion_z: float,
    quaternion_w: float,
    timestamp: float,
    sequence: int,
    frame_id: str = "usv_fcu_local_ned",
) -> dict[str, Any]:
    """Build the JSON-ready Task 3 pose payload from MAVROS ENU odometry."""

    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ValueError("sequence must be a nonnegative integer")
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError("frame_id must be a nonempty string")
    yaw_enu = quaternion_to_yaw(
        quaternion_x,
        quaternion_y,
        quaternion_z,
        quaternion_w,
    )
    # ENU yaw is counterclockwise from East. NED heading is clockwise from North.
    heading_ned = normalize_angle(math.pi / 2.0 - yaw_enu)
    return {
        "position_ned_m": {
            "north": _finite_number(north_m, "north_m"),
            "east": _finite_number(east_m, "east_m"),
            "down": -_finite_number(up_m, "up_m"),
        },
        "heading_rad": heading_ned,
        "stamp_unix_sec": _finite_number(timestamp, "timestamp"),
        "sequence": sequence,
        "frame_id": frame_id.strip(),
        "body_reference": "base_link",
        "valid": True,
    }


def parse_ned_pose_payload(payload: Mapping[str, Any]) -> NEDPoseContract:
    """Validate the shared Task 3 position and heading contract."""

    if not isinstance(payload, Mapping):
        raise ValueError("message JSON must be an object")
    if payload.get("valid") is not True:
        raise ValueError("position message is not valid")
    position = payload.get("position_ned_m")
    if not isinstance(position, Mapping):
        raise ValueError("position_ned_m must be an object")
    frame_id = payload.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError("frame_id must be a nonempty string")
    sequence = payload.get("sequence")
    if isinstance(sequence, bool) or not isinstance(sequence, int):
        raise ValueError("sequence must be an integer")
    return NEDPoseContract(
        north_m=_finite_number(position.get("north"), "position_ned_m.north"),
        east_m=_finite_number(position.get("east"), "position_ned_m.east"),
        down_m=_finite_number(position.get("down"), "position_ned_m.down"),
        heading_rad=_finite_number(
            payload.get("heading_rad", payload.get("yaw_rad")),
            "heading_rad",
        ),
        timestamp=_finite_number(
            payload.get("stamp_unix_sec", payload.get("timestamp")),
            "stamp_unix_sec",
        ),
        sequence=sequence,
        frame_id=frame_id.strip(),
    )


def main(args=None) -> None:
    """Run the ROS 2 odometry-to-NED adapter."""

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    class Task3NEDPoseAdapter(Node):
        def __init__(self) -> None:
            super().__init__("task3_ned_pose_adapter")
            self.declare_parameter("input_topic", "/mavros/local_position/odom")
            self.declare_parameter("output_topic", "/tongji_mrl/twave/ned_position")
            self.declare_parameter("output_frame_id", "usv_fcu_local_ned")
            self._sequence = 0
            self._publisher = self.create_publisher(
                String,
                str(self.get_parameter("output_topic").value),
                10,
            )
            self._subscription = self.create_subscription(
                Odometry,
                str(self.get_parameter("input_topic").value),
                self._on_odometry,
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                "Task3 NED pose adapter ready; MAVROS ENU odometry will be "
                "published with an explicit NED heading."
            )

        def _on_odometry(self, message: Odometry) -> None:
            stamp = message.header.stamp
            timestamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            if timestamp <= 0.0:
                timestamp = self.get_clock().now().nanoseconds * 1e-9
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            try:
                payload = enu_odometry_to_ned_payload(
                    east_m=position.x,
                    north_m=position.y,
                    up_m=position.z,
                    quaternion_x=orientation.x,
                    quaternion_y=orientation.y,
                    quaternion_z=orientation.z,
                    quaternion_w=orientation.w,
                    timestamp=timestamp,
                    sequence=self._sequence,
                    frame_id=str(self.get_parameter("output_frame_id").value),
                )
            except ValueError as exc:
                self.get_logger().warning(f"Ignoring invalid MAVROS odometry: {exc}")
                return
            self._sequence += 1
            output = String()
            output.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
            self._publisher.publish(output)

    rclpy.init(args=args)
    node = Task3NEDPoseAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = [
    "NEDPoseContract",
    "enu_odometry_to_ned_payload",
    "normalize_angle",
    "parse_ned_pose_payload",
    "quaternion_to_yaw",
]
