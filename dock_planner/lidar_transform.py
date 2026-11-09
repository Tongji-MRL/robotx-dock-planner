"""Transform Mid-360 points from ``livox_frame`` into ``base_link``.

The module is deliberately ROS-independent.  ROS 2 adapters can use the same
math after extracting an ``(N, 3)`` array from ``sensor_msgs/PointCloud2``.

The translation stored in :class:`LidarExtrinsics` is the position of the
LiDAR frame origin expressed in the parent (``base_link``) frame.  Therefore
the forward transform is::

    p_base = R_base_lidar @ p_lidar + t_base_lidar

The current field measurement for the USV is 0.15 m forward and 0.22 m above
the ``base_link`` reference point, with the sensor level and facing forward.
It remains an initial measurement and should be checked with a live TF echo.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from .pointcloud_adapter import pointcloud2_message_to_xyz


def _finite_float(name: str, value: float) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True)
class LidarExtrinsics:
    """Rigid installation transform from a LiDAR frame to ``base_link``.

    Angles use ROS FLU axes and are in radians.  ``x_m``, ``y_m`` and ``z_m``
    are the LiDAR origin coordinates in ``base_link``.
    """

    x_m: float = 0.15
    y_m: float = 0.10
    z_m: float = 0.22
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    parent_frame: str = "base_link"
    child_frame: str = "livox_frame"

    def __post_init__(self) -> None:
        for name in (
            "x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad",
        ):
            object.__setattr__(self, name, _finite_float(name, getattr(self, name)))
        if not isinstance(self.parent_frame, str) or not self.parent_frame:
            raise ValueError("parent_frame must be a nonempty string")
        if not isinstance(self.child_frame, str) or not self.child_frame:
            raise ValueError("child_frame must be a nonempty string")
        if self.parent_frame == self.child_frame:
            raise ValueError("parent_frame and child_frame must differ")

    @property
    def translation_m(self) -> np.ndarray:
        """Return LiDAR origin coordinates in the parent frame."""

        return np.array([self.x_m, self.y_m, self.z_m], dtype=float)

    @property
    def rotation_parent_from_lidar(self) -> np.ndarray:
        """Return the 3x3 rotation mapping LiDAR vectors to parent vectors."""

        cr, sr = math.cos(self.roll_rad), math.sin(self.roll_rad)
        cp, sp = math.cos(self.pitch_rad), math.sin(self.pitch_rad)
        cy, sy = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        rx = np.array(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
        ry = np.array(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)))
        rz = np.array(((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)))
        return rz @ ry @ rx


def default_mid360_extrinsics() -> LidarExtrinsics:
    """Return the currently measured USV Mid-360 installation transform."""

    return LidarExtrinsics()


def _validate_points(points_xyz: Sequence[Sequence[float]]) -> np.ndarray:
    points = np.asarray(points_xyz, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_xyz must have shape (N, 3)")
    if not np.isfinite(points).all():
        raise ValueError("points_xyz must contain only finite values")
    return points


def transform_lidar_points_to_base_link(
    points_xyz: Sequence[Sequence[float]],
    extrinsics: LidarExtrinsics | None = None,
) -> np.ndarray:
    """Transform an ``(N, 3)`` LiDAR array into ``base_link`` coordinates.

    The returned array is a new floating-point array.  It is suitable for
    obstacle geometry, dock entrance detection, or any other downstream code
    that must reason about positions relative to the USV rather than the
    moving sensor origin.
    """

    points = _validate_points(points_xyz)
    transform = extrinsics or default_mid360_extrinsics()
    rotation = transform.rotation_parent_from_lidar
    return np.ascontiguousarray(points @ rotation.T + transform.translation_m)


def transform_lidar_point_to_base_link(
    point_xyz: Sequence[float],
    extrinsics: LidarExtrinsics | None = None,
) -> np.ndarray:
    """Transform one LiDAR point into ``base_link`` coordinates."""

    point = np.asarray(point_xyz, dtype=float)
    if point.shape != (3,):
        raise ValueError("point_xyz must have shape (3,)")
    return transform_lidar_points_to_base_link(point.reshape(1, 3), extrinsics)[0]


def pointcloud_message_to_base_link_xyz(
    message: object,
    extrinsics: LidarExtrinsics | None = None,
) -> np.ndarray:
    """Extract a PointCloud2-like message and transform it to ``base_link``.

    This is the intended boundary for a ROS 2 subscription callback: the ROS
    adapter supplies the message, while this package keeps PointCloud2 parsing
    and installation geometry in testable, ROS-independent functions.
    """

    lidar_points = pointcloud2_message_to_xyz(message)
    return transform_lidar_points_to_base_link(lidar_points, extrinsics)


def transform_base_link_points_to_lidar(
    points_xyz: Sequence[Sequence[float]],
    extrinsics: LidarExtrinsics | None = None,
) -> np.ndarray:
    """Apply the inverse transform, useful for synthetic/test geometry."""

    points = _validate_points(points_xyz)
    transform = extrinsics or default_mid360_extrinsics()
    rotation = transform.rotation_parent_from_lidar
    return np.ascontiguousarray((points - transform.translation_m) @ rotation)


__all__ = [
    "LidarExtrinsics",
    "default_mid360_extrinsics",
    "transform_lidar_points_to_base_link",
    "transform_lidar_point_to_base_link",
    "pointcloud_message_to_base_link_xyz",
    "transform_base_link_points_to_lidar",
]
