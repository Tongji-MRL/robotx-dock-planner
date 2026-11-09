"""Pure helpers for the Task3 LiDAR perception bridge.

The bridge receives LiDAR points in ``base_link`` and an explicit USV pose in
the task NED frame.  No implicit ENU/NED or yaw convention is guessed here.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Mapping, Sequence

import numpy as np

from dock_planner.integration import (
    CoarseDockCenterObservationNED,
    DockPerceptionResult,
    NEDPoint,
)


@dataclass(frozen=True)
class USVPoseNED:
    """Current USV reference point in the common NED frame."""

    north_m: float
    east_m: float
    down_m: float
    heading_rad: float
    timestamp: float

    def __post_init__(self) -> None:
        values = (
            self.north_m,
            self.east_m,
            self.down_m,
            self.heading_rad,
            self.timestamp,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("USV NED pose values must be finite")


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _point_from_payload(payload: Mapping[str, Any], name: str) -> NEDPoint:
    point = payload.get(name)
    if not isinstance(point, Mapping):
        raise ValueError(f"{name} must be an object")
    return NEDPoint(
        north=_finite_number(point.get("north"), f"{name}.north"),
        east=_finite_number(point.get("east"), f"{name}.east"),
        down=_finite_number(point.get("down"), f"{name}.down"),
    )


def parse_coarse_center_json(payload: Mapping[str, Any]) -> CoarseDockCenterObservationNED:
    """Parse the center-only UAV coarse-dock JSON contract."""

    if not isinstance(payload, Mapping):
        raise ValueError("coarse dock payload must be an object")
    if payload.get("valid") is not True:
        raise ValueError("coarse dock payload is not valid")
    dock_id = payload.get("dock_id")
    if not isinstance(dock_id, str) or not dock_id.strip():
        raise ValueError("dock_id must be a nonempty string")
    source = payload.get("source", "uav_coarse_center")
    if not isinstance(source, str) or not source.strip():
        raise ValueError("source must be a nonempty string")
    return CoarseDockCenterObservationNED(
        dock_id=dock_id,
        center_ned=_point_from_payload(payload, "center_ned"),
        timestamp=_finite_number(payload.get("timestamp"), "timestamp"),
        valid=True,
        confidence=(
            None
            if payload.get("confidence") is None
            else _finite_number(payload.get("confidence"), "confidence")
        ),
        source=source,
    )


def parse_usv_pose_json(payload: Mapping[str, Any]) -> USVPoseNED:
    """Parse a position message with an explicit NED heading.

    The heading is intentionally required.  ``ned_position`` messages that
    contain position only cannot rotate ``base_link`` points into NED safely.
    """

    if not isinstance(payload, Mapping):
        raise ValueError("USV pose payload must be an object")
    if payload.get("valid") is not True:
        raise ValueError("USV pose payload is not valid")
    position = payload.get("position_ned_m")
    if not isinstance(position, Mapping):
        raise ValueError("position_ned_m must be an object")
    heading = payload.get("heading_rad", payload.get("yaw_rad"))
    if heading is None:
        raise ValueError("USV pose must contain heading_rad or yaw_rad")
    stamp = payload.get("stamp_unix_sec", payload.get("timestamp"))
    if stamp is None:
        raise ValueError("USV pose must contain stamp_unix_sec or timestamp")
    return USVPoseNED(
        north_m=_finite_number(position.get("north"), "position_ned_m.north"),
        east_m=_finite_number(position.get("east"), "position_ned_m.east"),
        down_m=_finite_number(position.get("down"), "position_ned_m.down"),
        heading_rad=_finite_number(heading, "heading_rad"),
        timestamp=_finite_number(stamp, "stamp_unix_sec"),
    )


def transform_base_link_points_to_ned(
    points_xyz: Sequence[Sequence[float]],
    pose: USVPoseNED,
) -> np.ndarray:
    """Rotate/translate ``base_link`` points into the common NED frame.

    ``base_link`` uses ROS FLU axes: x forward, y left, z up.  NED uses x
    north, y east, z down.  The horizontal mapping therefore includes the
    left/right reflection, and the vertical component changes sign.
    """

    points = np.asarray(points_xyz, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_xyz must have shape (N, 3)")
    if not np.isfinite(points).all():
        raise ValueError("points_xyz must contain only finite values")
    c = math.cos(pose.heading_rad)
    s = math.sin(pose.heading_rad)
    # A forward point follows heading (cos, sin); a left point follows
    # (sin, -cos) in the north/east plane.
    rotation = np.array(((c, s), (s, -c)), dtype=float)
    horizontal = points[:, :2] @ rotation.T
    horizontal += np.array((pose.north_m, pose.east_m), dtype=float)
    result = np.column_stack((horizontal, pose.down_m - points[:, 2]))
    return np.ascontiguousarray(result)


def transform_ned_point_to_base_link(
    north_m: float,
    east_m: float,
    pose: USVPoseNED,
) -> tuple[float, float]:
    """Express one horizontal NED point in ``base_link`` FLU coordinates."""

    delta_n = _finite_number(north_m, "north_m") - pose.north_m
    delta_e = _finite_number(east_m, "east_m") - pose.east_m
    c = math.cos(pose.heading_rad)
    s = math.sin(pose.heading_rad)
    # The horizontal FLU->NED matrix is symmetric and orthonormal, so its
    # inverse has the same coefficients.
    return c * delta_n + s * delta_e, s * delta_n - c * delta_e


def transform_base_link_yaw_to_ned_heading(
    yaw_rad: float,
    pose: USVPoseNED,
) -> float:
    """Convert a local FLU yaw into a clockwise NED heading."""

    local_yaw = _finite_number(yaw_rad, "yaw_rad")
    heading = pose.heading_rad - local_yaw
    return math.atan2(math.sin(heading), math.cos(heading))


def dock_perception_result_to_json(result: DockPerceptionResult) -> str:
    """Serialize the perception contract consumed by ``usv_task3_executor``."""

    payload = {
        "dock_id": result.dock_id,
        "entrance_observations": [
            {
                "entrance_center_north_m": observation.entrance_center_north_m,
                "entrance_center_east_m": observation.entrance_center_east_m,
                "entrance_heading_rad": observation.entrance_heading_rad,
                "confidence": observation.confidence,
                "timestamp": observation.timestamp,
                "source": observation.source,
                "valid": observation.valid,
                "dock_id": observation.dock_id,
            }
            for observation in result.entrance_observations
        ],
        "timestamp": result.timestamp,
        "source": result.source,
        "valid": result.valid,
        "valid_until": result.valid_until,
    }
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


__all__ = [
    "USVPoseNED",
    "dock_perception_result_to_json",
    "parse_coarse_center_json",
    "parse_usv_pose_json",
    "transform_base_link_points_to_ned",
    "transform_base_link_yaw_to_ned_heading",
    "transform_ned_point_to_base_link",
]
