"""Pure mission memory for avoiding repeated Task 3 bay attempts."""

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real
from typing import List, Optional, Tuple

from .integration import DockEntranceObservationNED


def _finite(value: object) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


class BayDisposition(str, Enum):
    AVAILABLE = "AVAILABLE"
    COLOR_REJECTED = "COLOR_REJECTED"
    TERMINAL_FAILED = "TERMINAL_FAILED"
    COMPLETED = "COMPLETED"


class BayCandidateAction(str, Enum):
    INSPECT = "INSPECT"
    SKIP = "SKIP"
    SEARCH_EXHAUSTED = "SEARCH_EXHAUSTED"


@dataclass(frozen=True)
class BayVisitConfig:
    expected_bay_count: int = 3
    association_distance_m: float = 0.75
    max_terminal_failures_per_bay: int = 1

    def __post_init__(self) -> None:
        if (
            isinstance(self.expected_bay_count, bool)
            or not isinstance(self.expected_bay_count, int)
            or self.expected_bay_count <= 0
        ):
            raise ValueError("expected_bay_count must be a positive integer")
        if not _finite(self.association_distance_m) or self.association_distance_m <= 0:
            raise ValueError("association_distance_m must be positive and finite")
        if (
            isinstance(self.max_terminal_failures_per_bay, bool)
            or not isinstance(self.max_terminal_failures_per_bay, int)
            or self.max_terminal_failures_per_bay <= 0
        ):
            raise ValueError("max_terminal_failures_per_bay must be a positive integer")


@dataclass(frozen=True)
class BayVisitRecord:
    bay_index: int
    center_north_m: float
    center_east_m: float
    observation_count: int
    terminal_failure_count: int
    disposition: BayDisposition


@dataclass(frozen=True)
class BayCandidateDecision:
    action: BayCandidateAction
    bay_index: Optional[int]
    reason: str


@dataclass
class _MutableBayRecord:
    bay_index: int
    center_north_m: float
    center_east_m: float
    observation_count: int = 1
    terminal_failure_count: int = 0
    disposition: BayDisposition = BayDisposition.AVAILABLE

    def update_center(self, observation: DockEntranceObservationNED) -> None:
        count = self.observation_count + 1
        self.center_north_m += (
            observation.entrance_center_north_m - self.center_north_m
        ) / count
        self.center_east_m += (
            observation.entrance_center_east_m - self.center_east_m
        ) / count
        self.observation_count = count

    def snapshot(self) -> BayVisitRecord:
        return BayVisitRecord(
            bay_index=self.bay_index,
            center_north_m=self.center_north_m,
            center_east_m=self.center_east_m,
            observation_count=self.observation_count,
            terminal_failure_count=self.terminal_failure_count,
            disposition=self.disposition,
        )


class BayVisitTracker:
    """Associate entrance centers and stop revisiting rejected/failed bays.

    Association uses entrance-center distance because the current LiDAR
    observation does not yet contain a stable physical bay ID.  The tracker
    is deliberately mission-local and must be reset for every new dock task.
    """

    def __init__(self, config: BayVisitConfig = BayVisitConfig()) -> None:
        if not isinstance(config, BayVisitConfig):
            raise TypeError("config must be a BayVisitConfig")
        self._config = config
        self._records: List[_MutableBayRecord] = []

    @property
    def records(self) -> Tuple[BayVisitRecord, ...]:
        return tuple(record.snapshot() for record in self._records)

    @property
    def search_exhausted(self) -> bool:
        return (
            len(self._records) >= self._config.expected_bay_count
            and all(
                record.disposition is not BayDisposition.AVAILABLE
                for record in self._records
            )
        )

    def reset(self) -> None:
        self._records.clear()

    def observe(self, observation: DockEntranceObservationNED) -> BayCandidateDecision:
        if not isinstance(observation, DockEntranceObservationNED) or not observation.valid:
            raise ValueError("a valid DockEntranceObservationNED is required")
        record = self._associate(observation)
        if record is None:
            if len(self._records) >= self._config.expected_bay_count:
                action = (
                    BayCandidateAction.SEARCH_EXHAUSTED
                    if self.search_exhausted
                    else BayCandidateAction.SKIP
                )
                return BayCandidateDecision(
                    action,
                    None,
                    "candidate does not match any expected bay",
                )
            record = _MutableBayRecord(
                bay_index=len(self._records),
                center_north_m=observation.entrance_center_north_m,
                center_east_m=observation.entrance_center_east_m,
            )
            self._records.append(record)
        elif record.disposition is BayDisposition.AVAILABLE:
            record.update_center(observation)

        if record.disposition is BayDisposition.AVAILABLE:
            return BayCandidateDecision(
                BayCandidateAction.INSPECT,
                record.bay_index,
                "bay is available for inspection",
            )
        action = (
            BayCandidateAction.SEARCH_EXHAUSTED
            if self.search_exhausted
            else BayCandidateAction.SKIP
        )
        return BayCandidateDecision(
            action,
            record.bay_index,
            f"bay is unavailable: {record.disposition.value}",
        )

    def mark_color_rejected(self, bay_index: int) -> BayCandidateDecision:
        record = self._record(bay_index)
        record.disposition = BayDisposition.COLOR_REJECTED
        return self._post_update(record, "bay color does not match the task target")

    def mark_terminal_failure(self, bay_index: int) -> BayCandidateDecision:
        record = self._record(bay_index)
        record.terminal_failure_count += 1
        if record.terminal_failure_count >= self._config.max_terminal_failures_per_bay:
            record.disposition = BayDisposition.TERMINAL_FAILED
        return self._post_update(record, "terminal docking failed for this bay")

    def mark_completed(self, bay_index: int) -> BayCandidateDecision:
        record = self._record(bay_index)
        record.disposition = BayDisposition.COMPLETED
        return BayCandidateDecision(
            BayCandidateAction.SKIP,
            record.bay_index,
            "bay completed",
        )

    def _post_update(
        self,
        record: _MutableBayRecord,
        reason: str,
    ) -> BayCandidateDecision:
        return BayCandidateDecision(
            (
                BayCandidateAction.SEARCH_EXHAUSTED
                if self.search_exhausted
                else BayCandidateAction.SKIP
            ),
            record.bay_index,
            reason,
        )

    def _associate(
        self,
        observation: DockEntranceObservationNED,
    ) -> Optional[_MutableBayRecord]:
        if not self._records:
            return None
        record = min(
            self._records,
            key=lambda item: math.hypot(
                observation.entrance_center_north_m - item.center_north_m,
                observation.entrance_center_east_m - item.center_east_m,
            ),
        )
        distance = math.hypot(
            observation.entrance_center_north_m - record.center_north_m,
            observation.entrance_center_east_m - record.center_east_m,
        )
        return record if distance <= self._config.association_distance_m else None

    def _record(self, bay_index: int) -> _MutableBayRecord:
        if isinstance(bay_index, bool) or not isinstance(bay_index, int):
            raise TypeError("bay_index must be an integer")
        if not 0 <= bay_index < len(self._records):
            raise ValueError("bay_index does not identify an observed bay")
        return self._records[bay_index]


__all__ = [
    "BayCandidateAction",
    "BayCandidateDecision",
    "BayDisposition",
    "BayVisitConfig",
    "BayVisitRecord",
    "BayVisitTracker",
]
