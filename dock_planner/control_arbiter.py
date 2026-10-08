"""ROS-independent ownership arbitration for Task 3 velocity commands."""

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real
from typing import Optional

from .integration import GenericVelocityCommand


def _positive_optional(value: object, name: str) -> None:
    if value is None:
        return
    if (
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(value)
        or value <= 0
    ):
        raise ValueError(f"{name} must be None or a positive finite number")


class MotionCommandOwner(str, Enum):
    STOP = "STOP"
    GOTO = "GOTO"
    TERMINAL = "TERMINAL"


@dataclass(frozen=True)
class MotionCommandArbiterConfig:
    expected_frame_id: str = "base_link"
    stop_validity_s: float = 0.2
    max_abs_surge_mps: Optional[float] = None
    max_abs_yaw_rate_rps: Optional[float] = None

    def __post_init__(self) -> None:
        if not isinstance(self.expected_frame_id, str) or not self.expected_frame_id.strip():
            raise ValueError("expected_frame_id must be a nonempty string")
        if (
            isinstance(self.stop_validity_s, bool)
            or not isinstance(self.stop_validity_s, Real)
            or not math.isfinite(self.stop_validity_s)
            or self.stop_validity_s <= 0
        ):
            raise ValueError("stop_validity_s must be a positive finite number")
        _positive_optional(self.max_abs_surge_mps, "max_abs_surge_mps")
        _positive_optional(self.max_abs_yaw_rate_rps, "max_abs_yaw_rate_rps")


@dataclass(frozen=True)
class MotionCommandDecision:
    owner: MotionCommandOwner
    command: GenericVelocityCommand
    failsafe_stop: bool
    reason: str


class MotionCommandArbiter:
    """Forward only the explicitly selected controller's fresh command.

    ROS wrappers should remap goto and terminal controllers to different input
    topics and publish only this decision to ``/usv/control/cmd_vel``.
    """

    def __init__(
        self,
        config: MotionCommandArbiterConfig = MotionCommandArbiterConfig(),
    ) -> None:
        if not isinstance(config, MotionCommandArbiterConfig):
            raise TypeError("config must be a MotionCommandArbiterConfig")
        self._config = config
        self._owner = MotionCommandOwner.STOP

    @property
    def owner(self) -> MotionCommandOwner:
        return self._owner

    def set_owner(self, owner: MotionCommandOwner) -> None:
        if not isinstance(owner, MotionCommandOwner):
            raise TypeError("owner must be a MotionCommandOwner")
        self._owner = owner

    def select(
        self,
        *,
        now: float,
        goto_command: Optional[GenericVelocityCommand] = None,
        terminal_command: Optional[GenericVelocityCommand] = None,
    ) -> MotionCommandDecision:
        if isinstance(now, bool) or not isinstance(now, Real) or not math.isfinite(now):
            raise ValueError("now must be a finite real number")
        if self._owner is MotionCommandOwner.STOP:
            return self._stop(now, "no motion controller owns the output")

        selected = (
            goto_command
            if self._owner is MotionCommandOwner.GOTO
            else terminal_command
        )
        if selected is None:
            return self._stop(now, f"{self._owner.value} command is missing")
        if not isinstance(selected, GenericVelocityCommand):
            raise TypeError("controller commands must be GenericVelocityCommand values")
        if selected.frame_id != self._config.expected_frame_id:
            return self._stop(now, "selected command frame_id does not match arbiter frame")
        if not selected.is_valid_at(now):
            return self._stop(now, "selected command is stale or not active yet")
        if (
            self._config.max_abs_surge_mps is not None
            and abs(selected.surge_mps) > self._config.max_abs_surge_mps
        ):
            return self._stop(now, "selected surge command exceeds configured limit")
        if (
            self._config.max_abs_yaw_rate_rps is not None
            and abs(selected.yaw_rate_rps) > self._config.max_abs_yaw_rate_rps
        ):
            return self._stop(now, "selected yaw-rate command exceeds configured limit")
        return MotionCommandDecision(
            owner=self._owner,
            command=selected,
            failsafe_stop=False,
            reason=f"forwarding {self._owner.value} command",
        )

    def _stop(self, now: float, reason: str) -> MotionCommandDecision:
        return MotionCommandDecision(
            owner=MotionCommandOwner.STOP,
            command=GenericVelocityCommand(
                surge_mps=0.0,
                yaw_rate_rps=0.0,
                timestamp=float(now),
                valid_until=float(now) + self._config.stop_validity_s,
                phase="SAFE_STOP",
                source="motion_command_arbiter",
                frame_id=self._config.expected_frame_id,
            ),
            failsafe_stop=True,
            reason=reason,
        )


__all__ = [
    "MotionCommandArbiter",
    "MotionCommandArbiterConfig",
    "MotionCommandDecision",
    "MotionCommandOwner",
]
