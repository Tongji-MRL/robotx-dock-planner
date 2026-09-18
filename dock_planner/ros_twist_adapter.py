"""ROS ``Twist`` adapter for the generic velocity command contract.

This module only translates a command to the candidate ``/usv/control/cmd_vel``
topic. It does not connect to, or make assumptions about, ArduPilot. ROS
imports are deliberately lazy so the planner package remains usable without
ROS installed.
"""

from typing import Any, Optional, Tuple, Type

from .integration import GenericVelocityCommand


DEFAULT_TOPIC = "/usv/control/cmd_vel"
DEFAULT_RATE_HZ = 10.0


def _twist_type():
    try:
        from geometry_msgs.msg import Twist
    except ImportError as exc:  # pragma: no cover - exercised only without ROS
        raise RuntimeError("geometry_msgs is required to publish ROS Twist messages") from exc
    return Twist


def to_twist_message(
    command: GenericVelocityCommand,
    message_type: Optional[Type[Any]] = None,
) -> Any:
    """Convert surge and yaw rate to a ``geometry_msgs/msg/Twist`` instance."""

    if not isinstance(command, GenericVelocityCommand):
        raise TypeError("command must be a GenericVelocityCommand")
    twist = (message_type or _twist_type())()
    twist.linear.x = float(command.surge_mps)
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = float(command.yaw_rate_rps)
    return twist


def twist_values(command: GenericVelocityCommand) -> Tuple[float, float]:
    """Return the ROS-facing ``(linear.x, angular.z)`` values."""

    if not isinstance(command, GenericVelocityCommand):
        raise TypeError("command must be a GenericVelocityCommand")
    return float(command.surge_mps), float(command.yaw_rate_rps)


class RosTwistAdapter:
    """Publish generic commands as ROS ``Twist`` messages."""

    def __init__(
        self,
        node: Any,
        topic: str = DEFAULT_TOPIC,
        queue_depth: int = 10,
        message_type: Optional[Type[Any]] = None,
    ) -> None:
        if not isinstance(topic, str) or not topic.strip():
            raise ValueError("topic must be a nonempty string")
        if not isinstance(queue_depth, int) or isinstance(queue_depth, bool) or queue_depth <= 0:
            raise ValueError("queue_depth must be a positive integer")
        self.topic = topic
        self._message_type = message_type or _twist_type()
        self._publisher = node.create_publisher(self._message_type, topic, queue_depth)

    def publish(self, command: GenericVelocityCommand, *, now: Optional[float] = None) -> Any:
        """Publish a command, rejecting it when an explicit clock says it is stale."""

        if not isinstance(command, GenericVelocityCommand):
            raise TypeError("command must be a GenericVelocityCommand")
        if now is not None and not command.is_valid_at(now):
            raise ValueError("velocity command is expired or not yet valid")
        message = to_twist_message(command, self._message_type)
        self._publisher.publish(message)
        return message


__all__ = ["DEFAULT_RATE_HZ", "DEFAULT_TOPIC", "RosTwistAdapter", "to_twist_message", "twist_values"]
