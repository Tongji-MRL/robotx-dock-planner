"""Task 3 result envelopes forwarded by the main FSM to RoboCommand."""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any


def _timestamp(unix_seconds: float) -> str:
    return datetime.fromtimestamp(unix_seconds, tz=timezone.utc).isoformat().replace(
        "+00:00", "Z"
    )


class Task3ResultFactory:
    """Build monotonically sequenced Docking and Firefighting reports."""

    def __init__(
        self,
        team_id: str = "TONG",
        vehicle_id: str = "usv",
        initial_seq: int = 0,
    ) -> None:
        if not team_id.strip() or not vehicle_id.strip():
            raise ValueError("team_id and vehicle_id must be non-empty")
        if isinstance(initial_seq, bool) or not isinstance(initial_seq, int) or initial_seq < 0:
            raise ValueError("initial_seq must be a nonnegative integer")
        self.team_id = team_id.strip()
        self.vehicle_id = vehicle_id.strip()
        self._seq = initial_seq

    def _envelope(self, body_name: str, body: dict[str, Any], now: float) -> dict[str, Any]:
        self._seq += 1
        return {
            "team_id": self.team_id,
            "vehicle_id": self.vehicle_id,
            "seq": self._seq,
            "sent_at": _timestamp(now),
            body_name: body,
        }

    def docking(self, bay_id: int, *, now: float) -> dict[str, Any]:
        if bay_id not in (1, 2, 3):
            raise ValueError("bay_id must be 1, 2, or 3")
        return self._envelope("docking", {"bay_id": bay_id}, now)

    def firefighting(self, window_id: int, *, now: float) -> dict[str, Any]:
        if window_id not in (1, 2):
            raise ValueError("window_id must be 1 or 2")
        return self._envelope("firefighting", {"window_id": window_id}, now)


__all__ = ["Task3ResultFactory"]
