"""ROS-independent contract for a shared obstacle-avoidance supervisor."""

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real


class AvoidanceAction(str, Enum):
    """Action requested by the shared navigation safety module."""

    PROCEED = "PROCEED"
    HOLD = "HOLD"
    REPLAN = "REPLAN"
    ABORT = "ABORT"


@dataclass(frozen=True)
class ObstacleAvoidanceDecision:
    """Time-bounded decision consumed at the final command boundary.

    The future Task 1/common Orin module may use any sensors and ROS topics it
    needs. Its Task 3 adapter only has to translate the result into this
    contract. ``REPLAN`` is a stop request plus a signal for the mission
    supervisor; the command arbiter never invents a replacement path itself.
    """

    action: AvoidanceAction
    timestamp: float
    valid_until: float
    reason: str
    source: str = "shared_obstacle_avoidance"

    def __post_init__(self) -> None:
        if not isinstance(self.action, AvoidanceAction):
            raise TypeError("action must be an AvoidanceAction")
        for name, value in (
            ("timestamp", self.timestamp),
            ("valid_until", self.valid_until),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, Real)
                or not math.isfinite(value)
            ):
                raise ValueError(f"{name} must be a finite real number")
        if self.valid_until < self.timestamp:
            raise ValueError("valid_until cannot precede timestamp")
        if not isinstance(self.reason, str) or not self.reason.strip():
            raise ValueError("reason must be a nonempty string")
        if not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("source must be a nonempty string")

    def is_valid_at(self, now: float) -> bool:
        if (
            isinstance(now, bool)
            or not isinstance(now, Real)
            or not math.isfinite(now)
        ):
            raise ValueError("now must be a finite real number")
        return self.timestamp <= now <= self.valid_until


__all__ = ["AvoidanceAction", "ObstacleAvoidanceDecision"]
