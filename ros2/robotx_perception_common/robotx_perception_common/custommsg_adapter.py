"""ROS-independent extraction of Livox ``CustomMsg`` XYZ points."""

from __future__ import annotations

from typing import Any

import numpy as np


def custom_msg_to_xyz(
    message: Any,
    *,
    remove_nonfinite: bool = True,
    remove_zero_points: bool = True,
) -> np.ndarray:
    """Convert a ``livox_ros_driver2/msg/CustomMsg``-like object to ``(N, 3)``.

    The function intentionally duck-types the ROS message, which keeps the
    parser unit-testable on machines where the Livox message package is not
    installed.  The driver supplies ``message.points`` containing points with
    ``x``, ``y`` and ``z`` fields in metres.
    """

    if not hasattr(message, "points"):
        raise TypeError("CustomMsg is missing points")
    try:
        points = getattr(message, "points")
        xyz = np.asarray([(point.x, point.y, point.z) for point in points], dtype=np.float64)
    except (TypeError, AttributeError, ValueError) as exc:
        raise TypeError("CustomMsg points must contain x, y and z fields") from exc
    if xyz.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("CustomMsg points must produce shape (N, 3)")
    if remove_nonfinite:
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if remove_zero_points:
        xyz = xyz[~np.all(xyz == 0.0, axis=1)]
    return np.ascontiguousarray(xyz)


__all__ = ["custom_msg_to_xyz"]
