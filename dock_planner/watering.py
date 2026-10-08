"""ROS-independent completion logic for the Task 3 water-delivery step."""

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .integration import SprayerTargetObservation


class WateringVerificationState(str, Enum):
    WAITING_FOR_GREEN = "WAITING_FOR_GREEN"
    VERIFYING_GREEN = "VERIFYING_GREEN"
    COMPLETE = "COMPLETE"


@dataclass(frozen=True)
class WateringVerificationConfig:
    """Timing required before reporting water delivery as complete."""

    green_confirmation_s: float = 5.0
    observation_timeout_s: float = 1.0

    def __post_init__(self) -> None:
        for name, value in (
            ("green_confirmation_s", self.green_confirmation_s),
            ("observation_timeout_s", self.observation_timeout_s),
        ):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a finite positive number")
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be a finite positive number")


@dataclass(frozen=True)
class WateringVerificationResult:
    state: WateringVerificationState
    stop_pump_requested: bool
    watering_complete: bool
    confirmed_green_s: float


class WateringCompletionTracker:
    """Stop on first GREEN, then require a continuous GREEN confirmation.

    This class only returns decisions.  It does not publish a pump command and
    does not restart the pump after GREEN has been observed.
    """

    def __init__(
        self,
        config: WateringVerificationConfig = WateringVerificationConfig(),
    ) -> None:
        if not isinstance(config, WateringVerificationConfig):
            raise TypeError("config must be a WateringVerificationConfig")
        self._config = config
        self.reset()

    @property
    def state(self) -> WateringVerificationState:
        return self._state

    def reset(self) -> None:
        self._state = WateringVerificationState.WAITING_FOR_GREEN
        self._first_green_timestamp: Optional[float] = None
        self._last_green_timestamp: Optional[float] = None
        self._last_observation_timestamp: Optional[float] = None
        self._green_seen = False

    def update(
        self,
        observation: SprayerTargetObservation,
        *,
        now: Optional[float] = None,
    ) -> WateringVerificationResult:
        if not isinstance(observation, SprayerTargetObservation):
            raise TypeError("observation must be a SprayerTargetObservation")
        if now is not None and not observation.is_fresh_at(
            now, self._config.observation_timeout_s
        ):
            raise ValueError("sprayer target observation is stale")
        if (
            self._last_observation_timestamp is not None
            and observation.timestamp < self._last_observation_timestamp
        ):
            raise ValueError("sprayer target observations must be time ordered")
        self._last_observation_timestamp = observation.timestamp

        if self._state is WateringVerificationState.COMPLETE:
            return self._result(self._config.green_confirmation_s)

        if observation.delivery_confirmed:
            self._green_seen = True
            if (
                self._last_green_timestamp is None
                or observation.timestamp - self._last_green_timestamp
                > self._config.observation_timeout_s
            ):
                self._first_green_timestamp = observation.timestamp
            self._last_green_timestamp = observation.timestamp
            self._state = WateringVerificationState.VERIFYING_GREEN
            elapsed = observation.timestamp - self._first_green_timestamp
            if elapsed >= self._config.green_confirmation_s:
                self._state = WateringVerificationState.COMPLETE
                elapsed = self._config.green_confirmation_s
            return self._result(elapsed)

        # A non-GREEN update interrupts continuous confirmation.  Once GREEN
        # has been seen, the pump remains stopped while perception recovers.
        self._first_green_timestamp = None
        self._last_green_timestamp = None
        self._state = (
            WateringVerificationState.VERIFYING_GREEN
            if self._green_seen
            else WateringVerificationState.WAITING_FOR_GREEN
        )
        return self._result(0.0)

    def _result(self, elapsed: float) -> WateringVerificationResult:
        return WateringVerificationResult(
            state=self._state,
            stop_pump_requested=self._green_seen,
            watering_complete=self._state is WateringVerificationState.COMPLETE,
            confirmed_green_s=max(0.0, elapsed),
        )
