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


class WateringSequenceState(str, Enum):
    """High-level states around docking, aiming and water verification."""

    WAITING_FOR_DOCKED = "WAITING_FOR_DOCKED"
    WAITING_FOR_RED = "WAITING_FOR_RED"
    AIMING = "AIMING"
    SPRAYING = "SPRAYING"
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


@dataclass(frozen=True)
class WateringSequenceResult:
    state: WateringSequenceState
    pump_on_requested: bool = False
    pump_off_requested: bool = False
    watering_complete: bool = False
    reason: str = ""


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


class WateringSequenceController:
    """Pure orchestration for the docked-to-watering sequence.

    The controller consumes already-decoded observations.  It does not call a
    ROS service or command a gimbal.  ``pump_on_requested`` and
    ``pump_off_requested`` are one-shot decisions for the outer adapter.

    A red window is represented by ``target_valid=True`` and
    ``delivery_confirmed=False``.  A green completion observation is
    represented by ``delivery_confirmed=True`` and may have
    ``target_valid=False``.
    """

    def __init__(
        self,
        verification_config: WateringVerificationConfig = WateringVerificationConfig(),
    ) -> None:
        self._verification_config = verification_config
        self._tracker = WateringCompletionTracker(verification_config)
        self.reset()

    @property
    def state(self) -> WateringSequenceState:
        return self._state

    def reset(self) -> None:
        self._state = WateringSequenceState.WAITING_FOR_DOCKED
        self._tracker.reset()
        self._red_seen = False

    def on_docked(self) -> WateringSequenceResult:
        """Start looking for the red spraying window after DOCKED."""

        if self._state is not WateringSequenceState.WAITING_FOR_DOCKED:
            raise RuntimeError(f"expected WAITING_FOR_DOCKED, got {self._state.value}")
        self._state = WateringSequenceState.WAITING_FOR_RED
        return self._result()

    def on_window_observation(
        self,
        observation: SprayerTargetObservation,
        *,
        now: Optional[float] = None,
    ) -> WateringSequenceResult:
        """Consume red/green window status from the gimbal perception bridge."""

        if not isinstance(observation, SprayerTargetObservation):
            raise TypeError("observation must be a SprayerTargetObservation")
        if now is not None and not observation.is_fresh_at(
            now, self._verification_config.observation_timeout_s
        ):
            return self._result("stale sprayer window observation")

        if self._state is WateringSequenceState.WAITING_FOR_RED:
            if observation.target_valid and not observation.delivery_confirmed:
                self._red_seen = True
                self._state = WateringSequenceState.AIMING
            return self._result()

        if self._state is WateringSequenceState.AIMING:
            return self._result("waiting for gimbal ready")

        if self._state not in (
            WateringSequenceState.SPRAYING,
            WateringSequenceState.VERIFYING_GREEN,
        ):
            return self._result()

        verification = self._tracker.update(observation, now=now)
        pump_off = verification.stop_pump_requested and self._state is WateringSequenceState.SPRAYING
        if verification.watering_complete:
            self._state = WateringSequenceState.COMPLETE
        elif verification.stop_pump_requested:
            self._state = WateringSequenceState.VERIFYING_GREEN
        return WateringSequenceResult(
            state=self._state,
            pump_off_requested=pump_off,
            watering_complete=verification.watering_complete,
            reason="",
        )

    def on_gimbal_ready(self, ready: bool) -> WateringSequenceResult:
        """Start spraying once the gimbal reports a valid aiming state."""

        if not isinstance(ready, bool):
            raise TypeError("ready must be a boolean")
        if self._state is WateringSequenceState.AIMING and ready:
            self._state = WateringSequenceState.SPRAYING
            return WateringSequenceResult(
                state=self._state,
                pump_on_requested=True,
            )
        return self._result("waiting for red window" if not self._red_seen else "")

    def _result(self, reason: str = "") -> WateringSequenceResult:
        return WateringSequenceResult(state=self._state, reason=reason)
