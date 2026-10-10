"""ROS-independent sprayer perception observation contract."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class SprayerTargetObservation:
    """Minimal observation consumed from the gimbal perception bridge.

    A RED target normally has ``target_valid=True``. A detected GREEN window
    is represented by ``delivery_confirmed=True`` and may intentionally have
    ``target_valid=False`` because it is no longer an aiming target.
    """

    target_valid: bool
    delivery_confirmed: bool
    timestamp: float
    source: str = "/vision/target_observation"

    def __post_init__(self) -> None:
        if not isinstance(self.target_valid, bool):
            raise ValueError("target_valid must be a boolean")
        if not isinstance(self.delivery_confirmed, bool):
            raise ValueError("delivery_confirmed must be a boolean")
        if (
            isinstance(self.timestamp, bool)
            or not isinstance(self.timestamp, (int, float))
            or not math.isfinite(self.timestamp)
        ):
            raise ValueError("timestamp must be a finite real number")
        if not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("source must be a nonempty string")

    def is_fresh_at(self, now: float, timeout_s: float) -> bool:
        if (
            isinstance(now, bool)
            or not isinstance(now, (int, float))
            or not math.isfinite(now)
            or isinstance(timeout_s, bool)
            or not isinstance(timeout_s, (int, float))
            or not math.isfinite(timeout_s)
            or timeout_s < 0
        ):
            raise ValueError("now and timeout_s must be finite real numbers")
        return now - self.timestamp <= timeout_s
