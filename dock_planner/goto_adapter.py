"""Pure conversions for the team's single-waypoint goto controller.

The team goto controller consumes local ROS ``map`` coordinates (ENU), while
Task 3 plans in NED.  The conversion below uses a synchronized pair of current
position samples, so the two local frames do not need the same origin.

This module does not publish ROS messages or arbitrate ``cmd_vel`` ownership.
"""

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real
from typing import Optional

from .integration import NEDPoint
from .types import Pose2D


def _finite(value: object) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class GotoReferenceSample:
    """Simultaneous current position in Task3 NED and MAVROS odom ENU."""

    current_ned: NEDPoint
    odom_east_m: float
    odom_north_m: float
    ned_timestamp: float
    odom_timestamp: float

    def __post_init__(self) -> None:
        if not isinstance(self.current_ned, NEDPoint):
            raise ValueError("current_ned must be a NEDPoint")
        if not all(_finite(value) for value in (
            self.odom_east_m,
            self.odom_north_m,
            self.ned_timestamp,
            self.odom_timestamp,
        )):
            raise ValueError("odom coordinates and timestamps must be finite")

    @property
    def sample_skew_s(self) -> float:
        return abs(self.ned_timestamp - self.odom_timestamp)


@dataclass(frozen=True)
class GotoTargetENU:
    """ROS-independent representation of one PointStamped map target."""

    east_m: float
    north_m: float
    up_m: float = 0.0
    frame_id: str = "map"

    def __post_init__(self) -> None:
        if not all(_finite(value) for value in (self.east_m, self.north_m, self.up_m)):
            raise ValueError("target coordinates must be finite")
        if not isinstance(self.frame_id, str) or not self.frame_id.strip():
            raise ValueError("frame_id must be a nonempty string")


def ned_goal_to_goto_enu(
    target_ned: Pose2D,
    reference: GotoReferenceSample,
    *,
    max_reference_skew_s: float = 0.2,
) -> GotoTargetENU:
    """Convert a Task3 NED goal into the team's local odom ENU frame.

    Only position is converted.  ``target_ned.yaw`` is intentionally ignored
    because ``/safe_passage/goto/target`` does not accept a target heading.
    """

    if not isinstance(target_ned, Pose2D):
        raise TypeError("target_ned must be a Pose2D")
    if not isinstance(reference, GotoReferenceSample):
        raise TypeError("reference must be a GotoReferenceSample")
    if not _finite(max_reference_skew_s) or max_reference_skew_s < 0:
        raise ValueError("max_reference_skew_s must be finite and nonnegative")
    if reference.sample_skew_s > max_reference_skew_s:
        raise ValueError("NED and odom reference samples are not synchronized")

    delta_north = target_ned.x - reference.current_ned.north
    delta_east = target_ned.y - reference.current_ned.east
    return GotoTargetENU(
        east_m=reference.odom_east_m + delta_east,
        north_m=reference.odom_north_m + delta_north,
    )


class GotoStatusKind(str, Enum):
    WAIT_TARGET = "WAIT_TARGET"
    PRECHECK = "PRECHECK"
    RUNNING = "RUNNING"
    DONE = "DONE"
    ABORTED = "ABORTED"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class GotoStatus:
    kind: GotoStatusKind
    raw: str
    reason: Optional[str] = None

    @property
    def succeeded(self) -> bool:
        return self.kind is GotoStatusKind.DONE

    @property
    def terminal(self) -> bool:
        return self.kind in {GotoStatusKind.DONE, GotoStatusKind.ABORTED}


def parse_goto_status(message: str) -> GotoStatus:
    """Parse the stable prefix of ``/safe_passage/goto/status`` strings."""

    if not isinstance(message, str) or not message.strip():
        raise ValueError("message must be a nonempty string")
    raw = message.strip()
    prefix = raw.split(maxsplit=1)[0].upper()
    try:
        kind = GotoStatusKind(prefix)
    except ValueError:
        kind = GotoStatusKind.UNKNOWN
    reason = None
    if kind is GotoStatusKind.ABORTED:
        marker = "reason="
        index = raw.find(marker)
        reason = raw[index + len(marker):].strip() if index >= 0 else "UNKNOWN"
    return GotoStatus(kind=kind, raw=raw, reason=reason)


__all__ = [
    "GotoReferenceSample",
    "GotoStatus",
    "GotoStatusKind",
    "GotoTargetENU",
    "ned_goal_to_goto_enu",
    "parse_goto_status",
]
