"""Task3 color-sequence communication contract.

The ROS adapter can map :class:`ColorSequenceVector.as_ros_data` to
``std_msgs/msg/Int32MultiArray.data``. This module intentionally has no ROS
dependency so the planning package remains usable offline.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Iterable


COLOR_SEQUENCE_TOPIC = "/tongji_mrl/twave/task3/color_sequence"
COLOR_SEQUENCE_MESSAGE_TYPE = "std_msgs/msg/Int32MultiArray"


class ColorCode(IntEnum):
    UNKNOWN = 0
    RED = 1
    GREEN = 2
    BLUE = 3
    WHITE = 4
    AMBER = 5


@dataclass(frozen=True)
class ColorSequenceVector:
    """Two-element vector: ``[tin_color, circle_color]``."""

    tin_color: ColorCode
    circle_color: ColorCode

    def as_ros_data(self) -> list[int]:
        return [int(self.tin_color), int(self.circle_color)]

    @property
    def valid(self) -> bool:
        return self.tin_color is not ColorCode.UNKNOWN and self.circle_color is not ColorCode.UNKNOWN


def decode_color_sequence(data: Iterable[int]) -> ColorSequenceVector:
    """Decode and validate a ROS ``Int32MultiArray.data`` value."""

    values = tuple(data)
    if len(values) != 2:
        raise ValueError(f"color sequence must contain 2 values, got {len(values)}")
    try:
        tin = ColorCode(int(values[0]))
        circle = ColorCode(int(values[1]))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"unknown color code in {values!r}") from exc
    return ColorSequenceVector(tin_color=tin, circle_color=circle)
