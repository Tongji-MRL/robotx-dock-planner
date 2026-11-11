"""Translate UAV semantic-map positions into the USV local NED frame.

The UAV and USV each establish a local NED origin.  Their origin GPS/RTK
positions define a fixed translation for one mapping session::

    usv_position_ned = uav_position_ned + uav_origin_in_usv_ned

This module deliberately contains no ROS, PX4, MQTT or OCS dependency.  A
runtime adapter must latch the UAV mapping-start origin and USV task-start
origin, keep their WGS84 ellipsoid-height convention consistent, and provide
the receive timestamp for each semantic-map update.

The project interface confirms that both local frames use the same
north/east/down axes, so this adapter intentionally applies translation only.
It does not infer or apply a yaw rotation.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
import json
import math
from numbers import Real
from typing import Any, Mapping, Sequence, Tuple, Union

from .integration import CoarseDockCenterObservationNED, NEDPoint


WGS84_A_M = 6378137.0
WGS84_E2 = 6.6943799901413165e-3
NedOffset = Tuple[float, float, float]
SemanticMapPayload = Union[Mapping[str, Any], str, bytes]


def _finite(value: object) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class GpsOrigin:
    """One lat/lon/ellipsoid-height origin used to establish a local NED frame."""

    latitude_deg: float
    longitude_deg: float
    altitude_ellipsoid_m: float

    def __post_init__(self) -> None:
        if not all(_finite(value) for value in (
            self.latitude_deg, self.longitude_deg, self.altitude_ellipsoid_m,
        )):
            raise ValueError("GPS origin values must be finite real numbers")
        if not -90.0 <= self.latitude_deg <= 90.0:
            raise ValueError("latitude_deg must be in [-90, 90]")
        if not -180.0 <= self.longitude_deg <= 180.0:
            raise ValueError("longitude_deg must be in [-180, 180]")

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "GpsOrigin":
        if not isinstance(value, Mapping):
            raise TypeError("GPS origin must be a mapping")
        required = ("latitude_deg", "longitude_deg", "altitude_ellipsoid_m")
        missing = [name for name in required if name not in value]
        if missing:
            raise ValueError(f"missing GPS origin fields: {', '.join(missing)}")
        try:
            return cls(*(float(value[name]) for name in required))
        except (TypeError, ValueError) as exc:
            raise ValueError("GPS origin fields must be numeric") from exc


def geodetic_to_ecef(origin: GpsOrigin) -> Tuple[float, float, float]:
    """Convert a WGS84 origin to Earth-centred Earth-fixed metres."""

    if not isinstance(origin, GpsOrigin):
        raise TypeError("origin must be a GpsOrigin")
    latitude = math.radians(origin.latitude_deg)
    longitude = math.radians(origin.longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    radius = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_latitude**2)
    x = (radius + origin.altitude_ellipsoid_m) * cos_latitude * math.cos(longitude)
    y = (radius + origin.altitude_ellipsoid_m) * cos_latitude * math.sin(longitude)
    z = (radius * (1.0 - WGS84_E2) + origin.altitude_ellipsoid_m) * sin_latitude
    return x, y, z


def uav_origin_in_usv_ned(uav_origin: GpsOrigin, usv_origin: GpsOrigin) -> NedOffset:
    """Return the fixed NED offset added to UAV-map points for one session."""

    if not isinstance(uav_origin, GpsOrigin) or not isinstance(usv_origin, GpsOrigin):
        raise TypeError("uav_origin and usv_origin must be GpsOrigin values")
    uav_ecef = geodetic_to_ecef(uav_origin)
    usv_ecef = geodetic_to_ecef(usv_origin)
    dx, dy, dz = (uav_ecef[index] - usv_ecef[index] for index in range(3))

    latitude = math.radians(usv_origin.latitude_deg)
    longitude = math.radians(usv_origin.longitude_deg)
    sin_latitude, cos_latitude = math.sin(latitude), math.cos(latitude)
    sin_longitude, cos_longitude = math.sin(longitude), math.cos(longitude)
    north = (
        -sin_latitude * cos_longitude * dx
        - sin_latitude * sin_longitude * dy
        + cos_latitude * dz
    )
    east = -sin_longitude * dx + cos_longitude * dy
    down = (
        -cos_latitude * cos_longitude * dx
        - cos_latitude * sin_longitude * dy
        - sin_latitude * dz
    )
    return north, east, down


def geodetic_point_to_ned(point: GpsOrigin, origin: GpsOrigin) -> NEDPoint:
    """Express one WGS84 point in the local NED frame established by origin."""

    north, east, down = uav_origin_in_usv_ned(point, origin)
    return NEDPoint(north, east, down)


def add_ned_offset(position: Sequence[float], offset: NedOffset) -> NEDPoint:
    """Translate one three-element UAV NED position into USV NED."""

    if not isinstance(position, Sequence) or isinstance(position, (str, bytes)):
        raise TypeError("NED_position must be a sequence")
    if len(position) != 3:
        raise ValueError("NED_position must contain exactly three values")
    if len(offset) != 3 or not all(_finite(value) for value in offset):
        raise ValueError("offset must contain three finite values")
    try:
        values = tuple(float(value) for value in position)
    except (TypeError, ValueError) as exc:
        raise ValueError("NED_position values must be numeric") from exc
    if not all(math.isfinite(value) for value in values):
        raise ValueError("NED_position values must be finite")
    return NEDPoint(*(values[index] + offset[index] for index in range(3)))


def _decode_semantic_map(payload: SemanticMapPayload) -> Mapping[str, Any]:
    if isinstance(payload, bytes):
        try:
            payload = payload.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ValueError("semantic map must be UTF-8 JSON") from exc
    if isinstance(payload, str):
        try:
            payload = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise ValueError(f"semantic map is not valid JSON: {exc}") from exc
    if not isinstance(payload, Mapping):
        raise ValueError("semantic map must be a JSON object")
    objects = payload.get("objects")
    if not isinstance(objects, list):
        raise ValueError("semantic map must contain an objects list")
    return payload


def transform_semantic_map(
    payload: SemanticMapPayload,
    uav_origin: GpsOrigin,
    usv_origin: GpsOrigin,
) -> dict[str, Any]:
    """Return a copied semantic map with every object expressed in USV NED."""

    source = _decode_semantic_map(payload)
    offset = uav_origin_in_usv_ned(uav_origin, usv_origin)
    transformed = copy.deepcopy(dict(source))
    objects = []
    for index, item in enumerate(source["objects"]):
        if not isinstance(item, Mapping):
            raise ValueError(f"objects[{index}] must be a JSON object")
        if "NED_position" not in item:
            raise ValueError(f"objects[{index}] is missing NED_position")
        result = copy.deepcopy(dict(item))
        point = add_ned_offset(item["NED_position"], offset)
        result["NED_position"] = [point.north, point.east, point.down]
        objects.append(result)
    transformed["objects"] = objects
    transformed["source_coordinate_frame"] = "uav_ned"
    transformed["coordinate_frame"] = "usv_ned"
    transformed["offset_to_add_m"] = {
        "north": offset[0],
        "east": offset[1],
        "down": offset[2],
    }
    return transformed


def semantic_map_to_coarse_docks(
    payload: SemanticMapPayload,
    uav_origin: GpsOrigin,
    usv_origin: GpsOrigin,
    *,
    timestamp: float,
    source: str = "/task/semantic_map",
) -> Tuple[CoarseDockCenterObservationNED, ...]:
    """Extract transformed dock centres for the existing Task3 integration API."""

    if not _finite(timestamp):
        raise ValueError("timestamp must be a finite real number")
    if not isinstance(source, str) or not source.strip():
        raise ValueError("source must be a nonempty string")
    transformed = transform_semantic_map(payload, uav_origin, usv_origin)
    observations = []
    for index, item in enumerate(transformed["objects"]):
        if str(item.get("type", "")).strip().lower() != "dock":
            continue
        if "id" not in item or not str(item["id"]).strip():
            raise ValueError(f"dock object at index {index} is missing id")
        valid_value = item.get("valid", True)
        if not isinstance(valid_value, bool):
            raise ValueError(f"dock object at index {index} has non-boolean valid")
        confidence = item.get("confidence")
        if confidence is not None:
            try:
                confidence = float(confidence)
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"dock object at index {index} has non-numeric confidence"
                ) from exc
        position = item["NED_position"]
        observations.append(CoarseDockCenterObservationNED(
            dock_id=str(item["id"]).strip(),
            center_ned=NEDPoint(float(position[0]), float(position[1]), float(position[2])),
            timestamp=float(timestamp),
            valid=valid_value,
            confidence=confidence,
            source=source,
        ))
    return tuple(observations)


__all__ = [
    "GpsOrigin",
    "NedOffset",
    "add_ned_offset",
    "geodetic_to_ecef",
    "geodetic_point_to_ned",
    "semantic_map_to_coarse_docks",
    "transform_semantic_map",
    "uav_origin_in_usv_ned",
]
