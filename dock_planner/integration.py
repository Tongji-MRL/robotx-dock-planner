"""Perception contracts and integration boundary for terminal docking.

NED means x=north, y=east, z=down, in metres. A planar heading is in
radians from north toward east (clockwise geographically). These numeric
x/y/heading values match the core's Cartesian +x/+y angle convention.
Source adapters must establish a common NED origin and timestamp clock.
"""

import json
import math
from dataclasses import dataclass, replace
from enum import Enum
from numbers import Real
from typing import Any, Mapping, Optional, Protocol, Sequence, Tuple, Union

from .types import DockSpec, Pose2D


def _finite(value: object) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class NEDPoint:
    north: float
    east: float
    down: float

    def __post_init__(self) -> None:
        if not all(_finite(v) for v in (self.north, self.east, self.down)):
            raise ValueError("NED point coordinates must be finite real numbers")


@dataclass(frozen=True)
class CoarseDockInfo:
    """One-time UAV observation; corners are approximate, not boundaries."""

    dock_id: str
    center_ned: NEDPoint
    approximate_corners_ned: Tuple[NEDPoint, NEDPoint, NEDPoint, NEDPoint]
    timestamp: float
    valid: bool

    def __post_init__(self) -> None:
        if not isinstance(self.dock_id, str) or not self.dock_id.strip():
            raise ValueError("dock_id must be a nonempty string")
        if not isinstance(self.center_ned, NEDPoint):
            raise ValueError("center_ned must be a NEDPoint")
        corners = self.approximate_corners_ned
        if not isinstance(corners, (tuple, list)) or len(corners) != 4:
            raise ValueError("approximate_corners_ned must contain exactly four points")
        if not all(isinstance(point, NEDPoint) for point in corners):
            raise ValueError("approximate corners must be NEDPoint values")
        if len(set(corners)) != 4:
            raise ValueError("approximate corners must be four distinct points")
        object.__setattr__(self, "approximate_corners_ned", tuple(corners))
        if not _finite(self.timestamp):
            raise ValueError("timestamp must be a finite real number")
        if not isinstance(self.valid, bool):
            raise ValueError("valid must be a boolean")

    def is_fresh_at(self, now: float, timeout_s: float) -> bool:
        """Return whether this one-shot UAV observation is still usable."""

        return self.valid and is_timestamp_fresh(self.timestamp, now, timeout_s)


@dataclass(frozen=True)
class CoarseDockCenterObservationNED:
    """Center-only coarse UAV observation used by the approach/search adapter.

    This is a rough seed for getting the USV near the dock.  It is deliberately
    separate from :class:`CoarseDockInfo`: a center point is not promoted to
    four approximate corners or to a precise entrance pose.
    """

    dock_id: str
    center_ned: NEDPoint
    timestamp: float
    valid: bool
    confidence: Optional[float] = None
    source: str = "uav_coarse_center"

    def __post_init__(self) -> None:
        if not isinstance(self.dock_id, str) or not self.dock_id.strip():
            raise ValueError("dock_id must be a nonempty string")
        if not isinstance(self.center_ned, NEDPoint):
            raise ValueError("center_ned must be a NEDPoint")
        if not _finite(self.timestamp):
            raise ValueError("timestamp must be a finite real number")
        if not isinstance(self.valid, bool):
            raise ValueError("valid must be a boolean")
        if self.confidence is not None and (
            not _finite(self.confidence) or not 0 <= self.confidence <= 1
        ):
            raise ValueError("confidence must be None or in [0, 1]")
        if not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("source must be a nonempty string")

    def is_fresh_at(self, now: float, timeout_s: float) -> bool:
        """Return whether the center seed is valid and still usable."""

        return self.valid and is_timestamp_fresh(self.timestamp, now, timeout_s)


CoarseDockObservation = Union[CoarseDockInfo, CoarseDockCenterObservationNED]


@dataclass(frozen=True)
class CoarseDockSearchConfig:
    """Configurable staging/search radius for a center-only coarse seed.

    The default footprint matches the currently supplied Task 3 dock drawing.
    These values are approach guidance only; USV close-range perception still
    has to find the real dock structure and entrance.
    """

    dock_length_m: float = 6.596
    dock_width_m: float = 3.0
    stand_off_distance_m: float = 2.0
    position_uncertainty_m: float = 1.0

    def __post_init__(self) -> None:
        for name, value in (
            ("dock_length_m", self.dock_length_m),
            ("dock_width_m", self.dock_width_m),
            ("stand_off_distance_m", self.stand_off_distance_m),
            ("position_uncertainty_m", self.position_uncertainty_m),
        ):
            if not _finite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if self.dock_length_m <= 0 or self.dock_width_m <= 0:
            raise ValueError("dock dimensions must be positive")

    @property
    def search_radius_m(self) -> float:
        """Return a conservative circle radius around the coarse center."""

        dock_envelope_radius = math.hypot(
            self.dock_length_m / 2.0,
            self.dock_width_m / 2.0,
        )
        return (
            dock_envelope_radius
            + self.stand_off_distance_m
            + self.position_uncertainty_m
        )


def coarse_dock_search_radius_m(config: CoarseDockSearchConfig) -> float:
    """Compute the staging/search radius without touching planner core."""

    if not isinstance(config, CoarseDockSearchConfig):
        raise TypeError("config must be a CoarseDockSearchConfig")
    return config.search_radius_m


def coarse_dock_staging_pose(
    current_pose: Pose2D,
    coarse_center: Union[NEDPoint, CoarseDockCenterObservationNED],
    config: CoarseDockSearchConfig,
) -> Pose2D:
    """Choose a staging pose on the near side of the coarse search circle.

    The returned position is always on the near side of the configured search
    circle and its yaw points toward the coarse center.  This is only an
    approach hint; local USV perception must take over before entrance
    alignment.
    """

    if not isinstance(current_pose, Pose2D):
        raise TypeError("current_pose must be a Pose2D")
    if isinstance(coarse_center, CoarseDockCenterObservationNED):
        if not coarse_center.valid:
            raise ValueError("a valid coarse center observation is required")
        center = coarse_center.center_ned
    elif isinstance(coarse_center, NEDPoint):
        center = coarse_center
    else:
        raise TypeError("coarse_center must be a NEDPoint or center observation")
    if not isinstance(config, CoarseDockSearchConfig):
        raise TypeError("config must be a CoarseDockSearchConfig")

    north_delta = center.north - current_pose.x
    east_delta = center.east - current_pose.y
    distance = math.hypot(north_delta, east_delta)
    if distance == 0.0:
        raise ValueError("current pose and coarse center must not coincide")

    yaw = math.atan2(east_delta, north_delta)
    scale = config.search_radius_m / distance
    return Pose2D(
        center.north - north_delta * scale,
        center.east - east_delta * scale,
        yaw,
    )


@dataclass(frozen=True)
class ActiveBayContext:
    """The bay currently being inspected within the active coarse dock."""

    dock_id: str
    bay_id: str

    @classmethod
    def from_coarse_dock(
        cls,
        coarse_dock: CoarseDockObservation,
        bay_id: str,
    ) -> "ActiveBayContext":
        if not isinstance(coarse_dock, (CoarseDockInfo, CoarseDockCenterObservationNED)) or not coarse_dock.valid:
            raise ValueError("a valid coarse dock is required for active bay context")
        return cls(dock_id=coarse_dock.dock_id, bay_id=bay_id)

    def __post_init__(self) -> None:
        if not isinstance(self.dock_id, str) or not self.dock_id.strip():
            raise ValueError("dock_id must be a nonempty string")
        if not isinstance(self.bay_id, str) or not self.bay_id.strip():
            raise ValueError("bay_id must be a nonempty string")
        if self.bay_id.strip().upper() == "NONE":
            raise ValueError("bay_id cannot be NONE")

    def matches(self, observation: "DockVisionObservation") -> bool:
        if not isinstance(observation, DockVisionObservation):
            raise TypeError("observation must be a DockVisionObservation")
        return bool(
            observation.bay
            and observation.bay.strip().upper() == self.bay_id.strip().upper()
        )


@dataclass(frozen=True)
class PreciseEntrancePoseNED:
    """USV close-range result for the selected bay; heading points inward."""

    x_ned: float
    y_ned: float
    heading_rad: float
    timestamp: float
    valid: bool
    confidence: Optional[float] = None
    dock_id: Optional[str] = None

    def __post_init__(self) -> None:
        if not all(_finite(v) for v in (self.x_ned, self.y_ned, self.heading_rad, self.timestamp)):
            raise ValueError("entrance coordinates, heading and timestamp must be finite")
        if not isinstance(self.valid, bool):
            raise ValueError("valid must be a boolean")
        if self.confidence is not None and (not _finite(self.confidence) or not 0 <= self.confidence <= 1):
            raise ValueError("confidence must be None or in [0, 1]")
        if self.dock_id is not None and (not isinstance(self.dock_id, str) or not self.dock_id.strip()):
            raise ValueError("dock_id must be None or a nonempty string")


@dataclass(frozen=True)
class DockEntranceObservationNED:
    """USV close-range entrance observation produced by perception."""

    entrance_center_north_m: float
    entrance_center_east_m: float
    entrance_heading_rad: float
    confidence: float
    timestamp: float
    source: str
    valid: bool = True
    dock_id: Optional[str] = None

    def __post_init__(self) -> None:
        if not all(_finite(v) for v in (
            self.entrance_center_north_m,
            self.entrance_center_east_m,
            self.entrance_heading_rad,
            self.confidence,
            self.timestamp,
        )):
            raise ValueError("entrance observation values must be finite real numbers")
        if not 0 <= self.confidence <= 1:
            raise ValueError("confidence must be in [0, 1]")
        if not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("source must be a nonempty string")
        if not isinstance(self.valid, bool):
            raise ValueError("valid must be a boolean")
        if self.dock_id is not None and (not isinstance(self.dock_id, str) or not self.dock_id.strip()):
            raise ValueError("dock_id must be None or a nonempty string")

    def is_fresh_at(self, now: float, timeout_s: float) -> bool:
        """Return whether this close-range observation is still usable."""

        return self.valid and is_timestamp_fresh(self.timestamp, now, timeout_s)


@dataclass(frozen=True)
class BayColorObservation:
    """Color model output consumed from the perception group."""

    color: str
    confidence: float
    timestamp: float
    source: str
    valid: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.color, str) or not self.color.strip():
            raise ValueError("color must be a nonempty string")
        if not _finite(self.confidence) or not 0 <= self.confidence <= 1:
            raise ValueError("confidence must be a finite number in [0, 1]")
        if not _finite(self.timestamp):
            raise ValueError("timestamp must be a finite real number")
        if not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("source must be a nonempty string")
        if not isinstance(self.valid, bool):
            raise ValueError("valid must be a boolean")

    def is_fresh_at(self, now: float, timeout_s: float) -> bool:
        """Return whether this model result is still usable."""

        return self.valid and is_timestamp_fresh(self.timestamp, now, timeout_s)


@dataclass(frozen=True)
class DockPerceptionResult:
    """Integration-layer result from one dock perception update.

    The result contains entrance candidates only.  Raw point clouds, cluster
    internals and detector-specific OBBs stay outside this contract.
    ``valid_until`` is optional because an adapter may instead apply the
    configured timeout at the consumer boundary.
    """

    dock_id: str
    entrance_observations: Tuple[DockEntranceObservationNED, ...]
    timestamp: float
    source: str
    valid: bool = True
    valid_until: Optional[float] = None

    def __post_init__(self) -> None:
        if not isinstance(self.dock_id, str) or not self.dock_id.strip():
            raise ValueError("dock_id must be a nonempty string")
        if not isinstance(self.entrance_observations, (tuple, list)):
            raise ValueError("entrance_observations must be a tuple or list")
        observations = tuple(self.entrance_observations)
        if not all(isinstance(item, DockEntranceObservationNED) for item in observations):
            raise ValueError("entrance_observations must contain DockEntranceObservationNED values")
        if not all(item.dock_id in (None, self.dock_id) for item in observations):
            raise ValueError("entrance observation dock_id does not match result dock_id")
        object.__setattr__(self, "entrance_observations", observations)
        if not _finite(self.timestamp):
            raise ValueError("timestamp must be a finite real number")
        if self.valid_until is not None and (
            not _finite(self.valid_until) or self.valid_until < self.timestamp
        ):
            raise ValueError("valid_until must be None or >= timestamp")
        if not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("source must be a nonempty string")
        if not isinstance(self.valid, bool):
            raise ValueError("valid must be a boolean")

    @property
    def observations(self) -> Tuple[DockEntranceObservationNED, ...]:
        """Short alias used by flow adapters and mock integrations."""

        return self.entrance_observations

    def is_fresh_at(self, now: float, timeout_s: float) -> bool:
        """Return whether the result is valid and within its freshness window."""

        if not self.valid or not is_timestamp_fresh(self.timestamp, now, timeout_s):
            return False
        return self.valid_until is None or now <= self.valid_until


@dataclass(frozen=True)
class CameraEntranceCenter:
    """Entrance center as reported in the perception camera optical frame."""

    x: float
    y: float
    z: float
    pixel_u: Optional[float] = None
    pixel_v: Optional[float] = None

    def __post_init__(self) -> None:
        if not all(_finite(value) for value in (self.x, self.y, self.z)):
            raise ValueError("camera entrance coordinates must be finite real numbers")
        if self.pixel_u is not None and not _finite(self.pixel_u):
            raise ValueError("pixel_u must be finite when provided")
        if self.pixel_v is not None and not _finite(self.pixel_v):
            raise ValueError("pixel_v must be finite when provided")


@dataclass(frozen=True)
class DockVisionFace:
    """One entry from the vision node's ``visible_faces`` array."""

    face_class: str
    confidence: Optional[float] = None

    def __post_init__(self) -> None:
        if not isinstance(self.face_class, str) or not self.face_class.strip():
            raise ValueError("face_class must be a nonempty string")
        if self.confidence is not None and (
            not _finite(self.confidence) or not 0 <= self.confidence <= 1
        ):
            raise ValueError("confidence must be None or in [0, 1]")


@dataclass(frozen=True)
class DockVisionObservation:
    """Raw JSON contract from the Task3 vision node.

    This is deliberately still in the camera optical frame.  It is not a
    ``DockEntranceObservationNED`` and must pass through a future coordinate
    conversion layer before it can be consumed by the planner flow.
    """

    entrance_center_camera: Optional[CameraEntranceCenter]
    frame_id: Optional[str]
    timestamp: Optional[float]
    valid: bool
    heading_valid: bool
    heading_quality: Optional[str] = None
    heading_error_deg: Optional[float] = None
    range_m: Optional[float] = None
    forward_z_m: Optional[float] = None
    forward_distance_m: Optional[float] = None
    lateral_error_m: Optional[float] = None
    bearing_x_deg: Optional[float] = None
    face_class: Optional[str] = None
    confidence: Optional[float] = None
    visible_faces: Tuple[DockVisionFace, ...] = ()
    side_face_class: Optional[str] = None
    side_consistency_deg: Optional[float] = None
    bay: Optional[str] = None
    observed_light_bay: Optional[str] = None
    bay_color: Optional[str] = None
    bay_yolo_confidence: Optional[float] = None
    bay_color_confidence: Optional[float] = None
    alignment_status: Optional[str] = None
    plane_points: Optional[int] = None
    plane_rmse_m: Optional[float] = None
    geometry_source: Optional[str] = None
    coordinate_convention: Optional[str] = None
    invalid_reason: Optional[str] = None
    source: str = "robotx_perception"
    valid_until: Optional[float] = None

    def __post_init__(self) -> None:
        if self.frame_id is not None and (
            not isinstance(self.frame_id, str) or not self.frame_id.strip()
        ):
            raise ValueError("frame_id must be None or a nonempty string")
        if self.timestamp is not None and not _finite(self.timestamp):
            raise ValueError("timestamp must be None or finite")
        if self.valid_until is not None and (
            not _finite(self.valid_until)
            or self.timestamp is None
            or self.valid_until < self.timestamp
        ):
            raise ValueError("valid_until requires a timestamp and must be >= timestamp")
        if not isinstance(self.valid, bool) or not isinstance(self.heading_valid, bool):
            raise ValueError("valid and heading_valid must be booleans")
        for name, value in (
            ("heading_error_deg", self.heading_error_deg),
            ("range_m", self.range_m),
            ("forward_z_m", self.forward_z_m),
            ("forward_distance_m", self.forward_distance_m),
            ("lateral_error_m", self.lateral_error_m),
            ("bearing_x_deg", self.bearing_x_deg),
            ("side_consistency_deg", self.side_consistency_deg),
            ("plane_rmse_m", self.plane_rmse_m),
        ):
            if value is not None and not _finite(value):
                raise ValueError(f"{name} must be None or finite")
        for name, value in (
            ("confidence", self.confidence),
            ("bay_yolo_confidence", self.bay_yolo_confidence),
            ("bay_color_confidence", self.bay_color_confidence),
        ):
            if value is not None and (not _finite(value) or not 0 <= value <= 1):
                raise ValueError(f"{name} must be None or in [0, 1]")
        if self.plane_points is not None and (
            not isinstance(self.plane_points, int)
            or isinstance(self.plane_points, bool)
            or self.plane_points < 0
        ):
            raise ValueError("plane_points must be None or a nonnegative integer")
        if not all(isinstance(face, DockVisionFace) for face in self.visible_faces):
            raise ValueError("visible_faces must contain DockVisionFace values")
        if not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("source must be a nonempty string")

    def valid_until_for(self, freshness: "DataFreshnessConfig") -> Optional[float]:
        """Compute the perception validity deadline from the raw stamp."""

        if not isinstance(freshness, DataFreshnessConfig):
            raise TypeError("freshness must be a DataFreshnessConfig")
        if self.timestamp is None:
            return None
        return self.timestamp + freshness.perception_timeout_s

    def with_valid_until(self, freshness: "DataFreshnessConfig") -> "DockVisionObservation":
        """Return this raw observation with a deadline derived from its stamp."""

        return replace(self, valid_until=self.valid_until_for(freshness))

    def is_fresh_at(self, now: float, freshness: "DataFreshnessConfig") -> bool:
        """Check source-time freshness without changing the raw observation."""

        if not isinstance(freshness, DataFreshnessConfig):
            raise TypeError("freshness must be a DataFreshnessConfig")
        if not self.valid or self.timestamp is None or not _finite(now):
            return False
        deadline = self.valid_until
        if deadline is None:
            deadline = self.valid_until_for(freshness)
        return self.timestamp <= now <= deadline

    def matches_active_bay(self, context: ActiveBayContext) -> bool:
        """Return whether this result belongs to the bay being inspected."""

        if not isinstance(context, ActiveBayContext):
            raise TypeError("context must be an ActiveBayContext")
        return context.matches(self)

    @property
    def can_convert_to_ned(self) -> bool:
        """Whether required raw fields exist for a future real converter."""

        return bool(
            self.valid
            and self.heading_valid
            and self.entrance_center_camera is not None
            and self.frame_id
            and self.timestamp is not None
            and self.heading_error_deg is not None
            and self.confidence is not None
        )


def _optional_float(value: object) -> Optional[float]:
    if not isinstance(value, Real) or isinstance(value, bool) or not math.isfinite(value):
        return None
    return float(value)


def _optional_confidence(value: object) -> Optional[float]:
    value = _optional_float(value)
    return value if value is not None and 0 <= value <= 1 else None


def _optional_nonnegative_int(value: object) -> Optional[int]:
    value = _optional_float(value)
    if value is None or not value.is_integer() or value < 0:
        return None
    return int(value)


def _optional_text(value: object, *, uppercase: bool = False) -> Optional[str]:
    if not isinstance(value, str) or not value.strip():
        return None
    value = value.strip()
    return value.upper() if uppercase else value


def _parse_vision_timestamp(value: object) -> Optional[float]:
    if not isinstance(value, Mapping):
        return None
    sec = _optional_float(value.get("sec"))
    nanosec = _optional_float(value.get("nanosec"))
    if sec is None or nanosec is None:
        return None
    if not sec.is_integer() or not nanosec.is_integer() or not 0 <= nanosec < 1_000_000_000:
        return None
    return float(int(sec) + int(nanosec) * 1e-9)


def _parse_camera_center(value: object) -> Optional[CameraEntranceCenter]:
    if not isinstance(value, Mapping):
        return None
    x = _optional_float(value.get("x"))
    y = _optional_float(value.get("y"))
    z = _optional_float(value.get("z"))
    if x is None or y is None or z is None:
        return None
    return CameraEntranceCenter(
        x=x,
        y=y,
        z=z,
        pixel_u=_optional_float(value.get("pixel_u")),
        pixel_v=_optional_float(value.get("pixel_v")),
    )


def _parse_visible_faces(value: object) -> Tuple[DockVisionFace, ...]:
    if not isinstance(value, (list, tuple)):
        return ()
    faces = []
    for item in value:
        if not isinstance(item, Mapping):
            continue
        face_class = _optional_text(item.get("class"))
        if face_class is None:
            continue
        faces.append(DockVisionFace(face_class, _optional_confidence(item.get("confidence"))))
    return tuple(faces)


def parse_dock_vision_json(
    payload: Union[str, bytes, bytearray, Mapping[str, Any]],
    *,
    source: str = "robotx_perception",
    freshness: Optional["DataFreshnessConfig"] = None,
) -> DockVisionObservation:
    """Parse one vision ``std_msgs/msg/String`` JSON payload.

    Missing or ``null`` optional values are represented as ``None``.  A
    missing/invalid ``valid`` field becomes ``False``; no coordinate or heading
    inference is performed here.
    """

    if isinstance(payload, Mapping):
        data = payload
    elif isinstance(payload, (str, bytes, bytearray)):
        try:
            decoded = json.loads(payload)
        except (TypeError, json.JSONDecodeError) as exc:
            raise ValueError("payload must contain valid JSON") from exc
        if not isinstance(decoded, Mapping):
            raise ValueError("perception JSON root must be an object")
        data = decoded
    else:
        raise TypeError("payload must be JSON text or a mapping")

    source_name = _optional_text(source) or "robotx_perception"
    valid = data.get("valid") if isinstance(data.get("valid"), bool) else False
    heading_valid = data.get("heading_valid") if isinstance(data.get("heading_valid"), bool) else False
    observation = DockVisionObservation(
        entrance_center_camera=_parse_camera_center(data.get("entrance_center")),
        frame_id=_optional_text(data.get("frame_id")),
        timestamp=_parse_vision_timestamp(data.get("stamp")),
        valid=valid,
        heading_valid=heading_valid,
        heading_quality=_optional_text(data.get("heading_quality")),
        heading_error_deg=_optional_float(data.get("heading_error_deg")),
        range_m=_optional_float(data.get("range_m")),
        forward_z_m=_optional_float(data.get("forward_z_m")),
        forward_distance_m=_optional_float(data.get("forward_distance_m")),
        lateral_error_m=_optional_float(data.get("lateral_error_m")),
        bearing_x_deg=_optional_float(data.get("bearing_x_deg")),
        face_class=_optional_text(data.get("face_class")),
        confidence=_optional_confidence(data.get("confidence")),
        visible_faces=_parse_visible_faces(data.get("visible_faces")),
        side_face_class=_optional_text(data.get("side_face_class")),
        side_consistency_deg=_optional_float(data.get("side_consistency_deg")),
        bay=_optional_text(data.get("bay")),
        observed_light_bay=_optional_text(data.get("observed_light_bay")),
        bay_color=_optional_text(data.get("bay_color"), uppercase=True),
        bay_yolo_confidence=_optional_confidence(data.get("bay_yolo_confidence")),
        bay_color_confidence=_optional_confidence(data.get("bay_color_confidence")),
        alignment_status=_optional_text(data.get("alignment_status")),
        plane_points=_optional_nonnegative_int(data.get("plane_points")),
        plane_rmse_m=_optional_float(data.get("plane_rmse_m")),
        geometry_source=_optional_text(data.get("geometry_source")),
        coordinate_convention=_optional_text(data.get("coordinate_convention")),
        invalid_reason=_optional_text(data.get("invalid_reason")),
        source=source_name,
    )
    return observation.with_valid_until(freshness) if freshness is not None else observation


def dock_vision_to_bay_color_observation(
    observation: DockVisionObservation,
    *,
    source: Optional[str] = None,
    active_bay: Optional[ActiveBayContext] = None,
) -> Optional[BayColorObservation]:
    """Convert a known bay color, while blocking UNKNOWN/NONE results."""

    if not isinstance(observation, DockVisionObservation):
        raise TypeError("observation must be a DockVisionObservation")
    if active_bay is not None and not observation.matches_active_bay(active_bay):
        return None
    color = (observation.bay_color or "").upper()
    bay = (observation.bay or "").upper()
    if (
        not observation.valid
        or observation.timestamp is None
        or not bay
        or bay == "NONE"
        or color in {"", "UNKNOWN", "NONE"}
        or observation.bay_color_confidence is None
    ):
        return None
    return BayColorObservation(
        color=color,
        confidence=observation.bay_color_confidence,
        timestamp=observation.timestamp,
        source=_optional_text(source) or observation.source,
        valid=True,
    )


class DockVisionCoordinateConverter(Protocol):
    """Future boundary for a real camera-frame to NED conversion layer."""

    def convert(
        self,
        observation: DockVisionObservation,
        *,
        dock_id: str,
    ) -> DockEntranceObservationNED:
        """Return a fully transformed NED observation; no fake transform is provided."""
        ...


@dataclass(frozen=True)
class DataFreshnessConfig:
    """Timeouts for one-shot coarse and close-range integration data."""

    coarse_dock_timeout_s: float = 30.0
    perception_timeout_s: float = 2.0

    def __post_init__(self) -> None:
        for name, value in (
            ("coarse_dock_timeout_s", self.coarse_dock_timeout_s),
            ("perception_timeout_s", self.perception_timeout_s),
        ):
            if not _finite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")


def is_timestamp_fresh(timestamp: float, now: float, timeout_s: float) -> bool:
    """Check a timestamp in the caller's shared clock domain."""

    if not all(_finite(value) for value in (timestamp, now, timeout_s)):
        raise ValueError("timestamp, now and timeout_s must be finite")
    if timeout_s < 0:
        raise ValueError("timeout_s must be nonnegative")
    return timestamp <= now <= timestamp + timeout_s


class DockPerception(Protocol):
    """Boundary for a future PointCloud2-to-entrance perception adapter."""

    def detect(
        self,
        point_cloud: object,
        coarse_dock: CoarseDockObservation,
    ) -> DockPerceptionResult:
        """Return one timestamped candidate result; do not process points here."""
        ...


class ColorDetector(Protocol):
    """Boundary for the perception group's camera color model."""

    def detect(
        self,
        image: object,
        entrance: DockEntranceObservationNED,
    ) -> BayColorObservation:
        """Return the color observation for the currently inspected entrance."""
        ...


def entrance_observation_to_precise_pose(
    observation: DockEntranceObservationNED,
    dock_id: Optional[str] = None,
) -> PreciseEntrancePoseNED:
    """Convert a valid close-range observation to the existing terminal pose type."""

    if not isinstance(observation, DockEntranceObservationNED):
        raise TypeError("observation must be a DockEntranceObservationNED")
    if not observation.valid:
        raise ValueError("a valid entrance observation is required")
    if dock_id is not None and observation.dock_id is not None and dock_id != observation.dock_id:
        raise ValueError("dock_id does not match the observation dock_id")
    resolved_dock_id = dock_id if dock_id is not None else observation.dock_id
    return PreciseEntrancePoseNED(
        x_ned=observation.entrance_center_north_m,
        y_ned=observation.entrance_center_east_m,
        heading_rad=observation.entrance_heading_rad,
        timestamp=observation.timestamp,
        valid=True,
        confidence=observation.confidence,
        dock_id=resolved_dock_id,
    )


@dataclass(frozen=True)
class ObstaclePointNED:
    """Only an observed center; no physical radius or clearance is implied."""

    center_ned: NEDPoint

    def __post_init__(self) -> None:
        if not isinstance(self.center_ned, NEDPoint):
            raise ValueError("center_ned must be a NEDPoint")


@dataclass(frozen=True)
class GenericVelocityCommand:
    """Controller output independent of ROS, Gazebo, or a vehicle backend.

    ``timestamp`` and ``valid_until`` use the clock domain selected by the
    caller.  ``frame_id`` records the intended body frame for the internal
    contract; a bare ROS ``Twist`` does not carry it and the ROS adapter does
    not claim to publish it.
    """

    surge_mps: float
    yaw_rate_rps: float
    timestamp: float
    valid_until: float
    phase: str
    source: str
    frame_id: str = "base_link"

    def __post_init__(self) -> None:
        if not all(_finite(v) for v in (
            self.surge_mps, self.yaw_rate_rps, self.timestamp, self.valid_until,
        )):
            raise ValueError("velocity values and timestamps must be finite real numbers")
        if self.valid_until < self.timestamp:
            raise ValueError("valid_until must be greater than or equal to timestamp")
        for name, value in (("phase", self.phase), ("source", self.source), ("frame_id", self.frame_id)):
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be a nonempty string")

    def is_valid_at(self, now: float) -> bool:
        """Return whether ``now`` falls within this command's validity window."""

        if not _finite(now):
            raise ValueError("now must be a finite real number")
        return self.timestamp <= now <= self.valid_until


@dataclass(frozen=True)
class TerminalPlanInput:
    dock: DockSpec
    obstacles: Tuple[ObstaclePointNED, ...]


def prepare_terminal_plan_input(
    coarse_dock: CoarseDockObservation,
    entrance: PreciseEntrancePoseNED,
    bay_width: float,
    bay_depth: float,
    obstacle_points: Sequence[ObstaclePointNED] = (),
) -> TerminalPlanInput:
    """Bridge valid observations to the core without inferring an entrance.

    The caller supplies bay clear dimensions from a separate verified source.
    Nonempty obstacle points remain nonempty when passed to DockPlanner.plan,
    which currently returns UNSUPPORTED_INPUT. No clearance is inferred here.
    """
    if not isinstance(coarse_dock, (CoarseDockInfo, CoarseDockCenterObservationNED)) or not coarse_dock.valid:
        raise ValueError("a valid coarse dock observation is required")
    if not isinstance(entrance, PreciseEntrancePoseNED) or not entrance.valid:
        raise ValueError("a valid precise entrance observation is required")
    if entrance.dock_id is not None and entrance.dock_id != coarse_dock.dock_id:
        raise ValueError("entrance dock_id does not match coarse dock_id")
    if not _finite(bay_width) or bay_width <= 0 or not _finite(bay_depth) or bay_depth <= 0:
        raise ValueError("bay_width and bay_depth must be positive finite numbers")
    if not isinstance(obstacle_points, (tuple, list)) or not all(
        isinstance(point, ObstaclePointNED) for point in obstacle_points
    ):
        raise ValueError("obstacle_points must be a list or tuple of ObstaclePointNED")
    return TerminalPlanInput(
        dock=DockSpec(
            entrance_center_x=entrance.x_ned,
            entrance_center_y=entrance.y_ned,
            entrance_heading=entrance.heading_rad,
            bay_width=bay_width,
            bay_depth=bay_depth,
            confidence=entrance.confidence,
            dock_id=coarse_dock.dock_id,
        ),
        obstacles=tuple(obstacle_points),
    )


class GlobalApproachStatus(str, Enum):
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"


@dataclass(frozen=True)
class GlobalApproachResult:
    status: GlobalApproachStatus
    waypoints: Tuple[Pose2D, ...]
    failure_reason: Optional[str] = None


class GlobalApproachPlanner(Protocol):
    """Future planner interface; no search or avoidance implementation here."""

    def plan(
        self,
        current_pose: Pose2D,
        approach_goal: Pose2D,
        obstacle_points: Sequence[ObstaclePointNED],
        *,
        safety_radius: Optional[float] = None,
        planning_boundary: Optional[Sequence[NEDPoint]] = None,
    ) -> GlobalApproachResult:
        ...
