"""Geofence search-line planning and stable three-bay layout recovery.

The functions in this module are ROS independent.  GPS corners must already
be converted into the shared RobotX NED frame.  The planner creates two
inset midpoint-to-midpoint transects through a convex four-corner geofence.
The line with the smaller worst-case corner distance is searched first.

Entrance observations are oriented from the observing USV toward the dock,
associated across several stationary frames, ordered left-to-right while
facing inward, and converted into bay observation poses plus a coarse dock
centre.  No unobserved corner geometry is fabricated.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence, Tuple

from .integration import (
    DockEntranceObservationNED,
    NEDPoint,
    sort_entrances_left_to_right,
)
from .types import Pose2D


def _finite(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _angle_error(first: float, second: float) -> float:
    return abs(_wrap(first - second))


@dataclass(frozen=True)
class GeofenceSearchConfig:
    """Safety margin and sampling policy for the two internal transects."""

    safety_inset_m: float = 3.0
    station_spacing_m: float = 4.0
    max_stations_per_line: int = 64

    def __post_init__(self) -> None:
        if not _finite(self.safety_inset_m) or self.safety_inset_m < 0:
            raise ValueError("safety_inset_m must be finite and nonnegative")
        if not _finite(self.station_spacing_m) or self.station_spacing_m <= 0:
            raise ValueError("station_spacing_m must be positive and finite")
        if self.max_stations_per_line < 2:
            raise ValueError("max_stations_per_line must be at least two")


@dataclass(frozen=True)
class SearchLine:
    """One safe line and its stop-and-observe stations in NED."""

    start: NEDPoint
    end: NEDPoint
    stations: Tuple[Pose2D, ...]
    length_m: float
    worst_corner_distance_m: float


@dataclass(frozen=True)
class GeofenceSearchPlan:
    """Primary search line plus an orthogonal fallback line."""

    centroid: NEDPoint
    ordered_corners: Tuple[NEDPoint, NEDPoint, NEDPoint, NEDPoint]
    lines: Tuple[SearchLine, SearchLine]

    @property
    def primary(self) -> SearchLine:
        return self.lines[0]

    @property
    def fallback(self) -> SearchLine:
        return self.lines[1]


@dataclass(frozen=True)
class StableEntranceConfig:
    """Association and geometry limits for stationary multi-frame scans."""

    required_frame_hits: int = 3
    min_entrances: int = 3
    association_distance_m: float = 0.75
    association_heading_rad: float = 0.35
    max_layout_heading_spread_rad: float = 0.30
    dock_depth_m: float = 6.0
    bay_observation_standoff_m: float = 4.0

    def __post_init__(self) -> None:
        if self.required_frame_hits < 1:
            raise ValueError("required_frame_hits must be positive")
        if self.min_entrances < 1:
            raise ValueError("min_entrances must be positive")
        for name in (
            "association_distance_m",
            "association_heading_rad",
            "max_layout_heading_spread_rad",
            "dock_depth_m",
            "bay_observation_standoff_m",
        ):
            value = getattr(self, name)
            if not _finite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")


@dataclass(frozen=True)
class BayObservationTarget:
    """Official left-to-right bay index and safe colour observation pose."""

    bay_id: int
    entrance: DockEntranceObservationNED
    observation_pose: Pose2D


@dataclass(frozen=True)
class StableEntranceLayout:
    """Stable entrance line, inferred dock centre, and bay observation poses."""

    entrances_left_to_right: Tuple[DockEntranceObservationNED, ...]
    inward_heading_rad: float
    coarse_center_ned: NEDPoint
    bays: Tuple[BayObservationTarget, ...]
    frame_hits: Tuple[int, ...]


def _polygon_area(points: Sequence[tuple[float, float]]) -> float:
    return 0.5 * sum(
        first[0] * second[1] - second[0] * first[1]
        for first, second in zip(points, points[1:] + points[:1])
    )


def _ordered_convex_corners(
    corners: Sequence[NEDPoint],
) -> Tuple[NEDPoint, NEDPoint, NEDPoint, NEDPoint]:
    values = tuple(corners)
    if len(values) != 4 or not all(isinstance(item, NEDPoint) for item in values):
        raise ValueError("corners must contain exactly four NEDPoint values")
    if len({(item.north, item.east) for item in values}) != 4:
        raise ValueError("geofence corners must be distinct")
    mean_n = sum(item.north for item in values) / 4.0
    mean_e = sum(item.east for item in values) / 4.0
    ordered = tuple(sorted(
        values,
        key=lambda item: math.atan2(item.east - mean_e, item.north - mean_n),
    ))
    xy = [(item.north, item.east) for item in ordered]
    area = _polygon_area(xy)
    if abs(area) <= 1e-6:
        raise ValueError("geofence corners form a degenerate polygon")
    signs = []
    for index in range(4):
        a = xy[index]
        b = xy[(index + 1) % 4]
        c = xy[(index + 2) % 4]
        cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
        if abs(cross) <= 1e-8:
            raise ValueError("geofence corners must form a strictly convex polygon")
        signs.append(math.copysign(1.0, cross))
    if len(set(signs)) != 1:
        raise ValueError("geofence corners must form a convex quadrilateral")
    return ordered  # type: ignore[return-value]


def _polygon_centroid(ordered: Sequence[NEDPoint]) -> NEDPoint:
    xy = [(item.north, item.east) for item in ordered]
    twice_area = sum(
        first[0] * second[1] - second[0] * first[1]
        for first, second in zip(xy, xy[1:] + xy[:1])
    )
    if abs(twice_area) <= 1e-9:
        raise ValueError("geofence polygon has zero area")
    north = 0.0
    east = 0.0
    for first, second in zip(xy, xy[1:] + xy[:1]):
        cross = first[0] * second[1] - second[0] * first[1]
        north += (first[0] + second[0]) * cross
        east += (first[1] + second[1]) * cross
    down = sum(item.down for item in ordered) / 4.0
    return NEDPoint(north / (3.0 * twice_area), east / (3.0 * twice_area), down)


def _move_toward(point: NEDPoint, target: NEDPoint, distance_m: float) -> NEDPoint:
    dn = target.north - point.north
    de = target.east - point.east
    length = math.hypot(dn, de)
    if distance_m >= length:
        raise ValueError("safety inset consumes the complete search half-line")
    scale = distance_m / max(length, 1e-9)
    return NEDPoint(
        point.north + dn * scale,
        point.east + de * scale,
        point.down + (target.down - point.down) * scale,
    )


def _point_segment_distance(point: NEDPoint, start: NEDPoint, end: NEDPoint) -> float:
    vx = end.north - start.north
    vy = end.east - start.east
    wx = point.north - start.north
    wy = point.east - start.east
    denom = vx * vx + vy * vy
    ratio = 0.0 if denom <= 1e-12 else max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
    return math.hypot(point.north - (start.north + ratio * vx), point.east - (start.east + ratio * vy))


def _make_search_line(
    first_midpoint: NEDPoint,
    second_midpoint: NEDPoint,
    centroid: NEDPoint,
    corners: Sequence[NEDPoint],
    config: GeofenceSearchConfig,
) -> SearchLine:
    start = _move_toward(first_midpoint, centroid, config.safety_inset_m)
    end = _move_toward(second_midpoint, centroid, config.safety_inset_m)
    dn = end.north - start.north
    de = end.east - start.east
    length = math.hypot(dn, de)
    if length <= 1e-6:
        raise ValueError("search line has zero length after safety inset")
    intervals = max(1, int(math.ceil(length / config.station_spacing_m)))
    if intervals + 1 > config.max_stations_per_line:
        intervals = config.max_stations_per_line - 1
    yaw = math.atan2(de, dn)
    stations = tuple(
        Pose2D(
            start.north + dn * index / intervals,
            start.east + de * index / intervals,
            yaw,
        )
        for index in range(intervals + 1)
    )
    worst = max(_point_segment_distance(item, start, end) for item in corners)
    return SearchLine(start, end, stations, length, worst)


def plan_geofence_search(
    corners: Sequence[NEDPoint],
    config: GeofenceSearchConfig = GeofenceSearchConfig(),
) -> GeofenceSearchPlan:
    """Plan two inset stop-and-observe transects inside a convex geofence."""

    if not isinstance(config, GeofenceSearchConfig):
        raise TypeError("config must be GeofenceSearchConfig")
    ordered = _ordered_convex_corners(corners)
    centroid = _polygon_centroid(ordered)
    edge_midpoints = tuple(
        NEDPoint(
            (ordered[index].north + ordered[(index + 1) % 4].north) / 2.0,
            (ordered[index].east + ordered[(index + 1) % 4].east) / 2.0,
            (ordered[index].down + ordered[(index + 1) % 4].down) / 2.0,
        )
        for index in range(4)
    )
    lines = (
        _make_search_line(edge_midpoints[0], edge_midpoints[2], centroid, ordered, config),
        _make_search_line(edge_midpoints[1], edge_midpoints[3], centroid, ordered, config),
    )
    ranked = tuple(sorted(
        lines,
        key=lambda item: (item.worst_corner_distance_m, -item.length_m),
    ))
    return GeofenceSearchPlan(centroid, ordered, ranked)  # type: ignore[arg-type]


def orient_entrance_from_observer(
    observation: DockEntranceObservationNED,
    observer_pose: Pose2D,
) -> DockEntranceObservationNED:
    """Resolve the 180-degree line ambiguity so heading points into the dock."""

    if not isinstance(observation, DockEntranceObservationNED):
        raise TypeError("observation must be DockEntranceObservationNED")
    if not isinstance(observer_pose, Pose2D):
        raise TypeError("observer_pose must be Pose2D")
    to_entrance_n = observation.entrance_center_north_m - observer_pose.x
    to_entrance_e = observation.entrance_center_east_m - observer_pose.y
    if math.hypot(to_entrance_n, to_entrance_e) <= 1e-6:
        raise ValueError("observer cannot coincide with an entrance")
    heading = observation.entrance_heading_rad
    if math.cos(heading) * to_entrance_n + math.sin(heading) * to_entrance_e < 0.0:
        heading = _wrap(heading + math.pi)
    return DockEntranceObservationNED(
        entrance_center_north_m=observation.entrance_center_north_m,
        entrance_center_east_m=observation.entrance_center_east_m,
        entrance_heading_rad=heading,
        confidence=observation.confidence,
        timestamp=observation.timestamp,
        source=observation.source,
        valid=observation.valid,
        dock_id=observation.dock_id,
    )


def _circular_mean(values: Iterable[float]) -> float:
    values = tuple(values)
    return math.atan2(sum(math.sin(item) for item in values), sum(math.cos(item) for item in values))


def stabilize_entrance_layout(
    frames: Sequence[Sequence[DockEntranceObservationNED]],
    observer_pose: Pose2D,
    config: StableEntranceConfig = StableEntranceConfig(),
) -> StableEntranceLayout | None:
    """Associate entrances over stationary frames and recover a bay layout."""

    if not isinstance(config, StableEntranceConfig):
        raise TypeError("config must be StableEntranceConfig")
    clusters: list[dict[str, object]] = []
    for frame_index, frame in enumerate(frames):
        used_clusters: set[int] = set()
        for raw in sorted(frame, key=lambda item: item.confidence, reverse=True):
            if not raw.valid:
                continue
            item = orient_entrance_from_observer(raw, observer_pose)
            best_index = None
            best_distance = math.inf
            for cluster_index, cluster in enumerate(clusters):
                if cluster_index in used_clusters:
                    continue
                samples = cluster["samples"]
                assert isinstance(samples, list)
                center_n = sum(sample.entrance_center_north_m for sample in samples) / len(samples)
                center_e = sum(sample.entrance_center_east_m for sample in samples) / len(samples)
                heading = _circular_mean(sample.entrance_heading_rad for sample in samples)
                distance = math.hypot(item.entrance_center_north_m - center_n, item.entrance_center_east_m - center_e)
                if (
                    distance <= config.association_distance_m
                    and _angle_error(item.entrance_heading_rad, heading) <= config.association_heading_rad
                    and distance < best_distance
                ):
                    best_index = cluster_index
                    best_distance = distance
            if best_index is None:
                clusters.append({"samples": [item], "frames": {frame_index}})
                used_clusters.add(len(clusters) - 1)
            else:
                samples = clusters[best_index]["samples"]
                frame_hits = clusters[best_index]["frames"]
                assert isinstance(samples, list) and isinstance(frame_hits, set)
                samples.append(item)
                frame_hits.add(frame_index)
                used_clusters.add(best_index)

    stable: list[tuple[DockEntranceObservationNED, int]] = []
    for cluster in clusters:
        samples = cluster["samples"]
        frame_hits = cluster["frames"]
        assert isinstance(samples, list) and isinstance(frame_hits, set)
        if len(frame_hits) < config.required_frame_hits:
            continue
        heading = _circular_mean(item.entrance_heading_rad for item in samples)
        stable.append((
            DockEntranceObservationNED(
                entrance_center_north_m=sum(item.entrance_center_north_m for item in samples) / len(samples),
                entrance_center_east_m=sum(item.entrance_center_east_m for item in samples) / len(samples),
                entrance_heading_rad=heading,
                confidence=sum(item.confidence for item in samples) / len(samples),
                timestamp=max(item.timestamp for item in samples),
                source="mid360_stationary_multiframe",
                valid=True,
                dock_id=next((item.dock_id for item in samples if item.dock_id), "field_search"),
            ),
            len(frame_hits),
        ))
    if len(stable) < config.min_entrances:
        return None

    # Prefer the most persistent and confident clusters, then establish the
    # official bay order in the recovered dock frame.
    stable.sort(key=lambda item: (item[1], item[0].confidence), reverse=True)
    selected = stable[:config.min_entrances]
    reference_heading = _circular_mean(item[0].entrance_heading_rad for item in selected)
    if any(
        _angle_error(item[0].entrance_heading_rad, reference_heading)
        > config.max_layout_heading_spread_rad
        for item in selected
    ):
        return None
    ordered = sort_entrances_left_to_right([item[0] for item in selected])
    hits_by_center = {
        (item.entrance_center_north_m, item.entrance_center_east_m): hits
        for item, hits in selected
    }
    frame_hits = tuple(
        hits_by_center[(item.entrance_center_north_m, item.entrance_center_east_m)]
        for item in ordered
    )
    heading = _circular_mean(item.entrance_heading_rad for item in ordered)
    mean_n = sum(item.entrance_center_north_m for item in ordered) / len(ordered)
    mean_e = sum(item.entrance_center_east_m for item in ordered) / len(ordered)
    forward_n = math.cos(heading)
    forward_e = math.sin(heading)
    coarse = NEDPoint(
        mean_n + forward_n * config.dock_depth_m / 2.0,
        mean_e + forward_e * config.dock_depth_m / 2.0,
        0.0,
    )
    bays = tuple(
        BayObservationTarget(
            bay_id=index,
            entrance=item,
            observation_pose=Pose2D(
                item.entrance_center_north_m - forward_n * config.bay_observation_standoff_m,
                item.entrance_center_east_m - forward_e * config.bay_observation_standoff_m,
                heading,
            ),
        )
        for index, item in enumerate(ordered, 1)
    )
    return StableEntranceLayout(ordered, heading, coarse, bays, frame_hits)


__all__ = [
    "BayObservationTarget",
    "GeofenceSearchConfig",
    "GeofenceSearchPlan",
    "SearchLine",
    "StableEntranceConfig",
    "StableEntranceLayout",
    "orient_entrance_from_observer",
    "plan_geofence_search",
    "stabilize_entrance_layout",
]
