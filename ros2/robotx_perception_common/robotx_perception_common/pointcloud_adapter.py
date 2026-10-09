"""ROS-independent PointCloud2 binary layout handling for XYZ extraction."""

from dataclasses import dataclass
import math
from numbers import Integral
from typing import Optional, Sequence, Tuple

import numpy as np


_POINT_FIELD_DTYPES = {
    1: "i1",  # INT8
    2: "u1",  # UINT8
    3: "i2",  # INT16
    4: "u2",  # UINT16
    5: "i4",  # INT32
    6: "u4",  # UINT32
    7: "f4",  # FLOAT32
    8: "f8",  # FLOAT64
}


@dataclass(frozen=True)
class PointFieldSpec:
    name: str
    offset: int
    datatype: int
    count: int = 1

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("field name must be a nonempty string")
        if isinstance(self.offset, bool) or not isinstance(self.offset, Integral) or self.offset < 0:
            raise ValueError("field offset must be a nonnegative integer")
        if self.datatype not in _POINT_FIELD_DTYPES:
            raise ValueError("unsupported PointField datatype")
        if isinstance(self.count, bool) or not isinstance(self.count, Integral) or self.count <= 0:
            raise ValueError("field count must be a positive integer")


@dataclass(frozen=True)
class PointCloud2Buffer:
    height: int
    width: int
    fields: Tuple[PointFieldSpec, ...]
    is_bigendian: bool
    point_step: int
    row_step: int
    data: bytes

    def __post_init__(self) -> None:
        for name, value in (
            ("height", self.height),
            ("width", self.width),
            ("point_step", self.point_step),
            ("row_step", self.row_step),
        ):
            if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
                raise ValueError(f"{name} must be a nonnegative integer")
        if self.point_step <= 0:
            raise ValueError("point_step must be positive")
        if self.row_step < self.width * self.point_step:
            raise ValueError("row_step is too small for width and point_step")
        if not isinstance(self.fields, (tuple, list)):
            raise ValueError("fields must be a tuple or list")
        fields = tuple(self.fields)
        if not all(isinstance(field, PointFieldSpec) for field in fields):
            raise ValueError("fields must contain PointFieldSpec values")
        object.__setattr__(self, "fields", fields)
        if not isinstance(self.is_bigendian, bool):
            raise ValueError("is_bigendian must be a boolean")
        try:
            view = memoryview(self.data)
        except TypeError as exc:
            raise ValueError("data must support the buffer protocol") from exc
        if not view.contiguous or view.itemsize != 1:
            raise ValueError("data must be a contiguous byte buffer")
        data = view.tobytes()
        if len(data) < self.height * self.row_step:
            raise ValueError("data is shorter than height * row_step")
        object.__setattr__(self, "data", data)


@dataclass(frozen=True)
class PointCloudXYZQuality:
    total_points: int
    finite_points: int
    nonfinite_points: int
    zero_points: int
    x_range: Optional[Tuple[float, float]]
    y_range: Optional[Tuple[float, float]]
    z_range: Optional[Tuple[float, float]]
    distance_range_m: Optional[Tuple[float, float]]
    median_distance_m: Optional[float]


def _field_dtype(field: PointFieldSpec, is_bigendian: bool) -> np.dtype:
    endian = ">" if is_bigendian else "<"
    return np.dtype(endian + _POINT_FIELD_DTYPES[field.datatype])


def pointcloud2_buffer_to_xyz(
    cloud: PointCloud2Buffer,
    *,
    remove_nonfinite: bool = True,
    remove_zero_points: bool = True,
) -> np.ndarray:
    """Extract valid XYZ rows even when the remaining fields mix types.

    Mid-360 clouds can contain placeholder rows whose x, y and z values are
    all exactly zero.  They are not physical returns, so they are removed by
    default independently from NaN/Inf filtering.  Either filter can be
    disabled when raw-data diagnostics are required.
    """

    if not isinstance(cloud, PointCloud2Buffer):
        raise TypeError("cloud must be a PointCloud2Buffer")
    field_by_name = {field.name: field for field in cloud.fields}
    missing = [name for name in ("x", "y", "z") if name not in field_by_name]
    if missing:
        raise ValueError(f"PointCloud2 is missing XYZ fields: {', '.join(missing)}")

    arrays = []
    for name in ("x", "y", "z"):
        field = field_by_name[name]
        if field.count != 1:
            raise ValueError(f"{name} field count must be 1")
        dtype = _field_dtype(field, cloud.is_bigendian)
        if field.offset + dtype.itemsize > cloud.point_step:
            raise ValueError(f"{name} field exceeds point_step")
        array = np.ndarray(
            shape=(cloud.height, cloud.width),
            dtype=dtype,
            buffer=cloud.data,
            offset=field.offset,
            strides=(cloud.row_step, cloud.point_step),
        )
        arrays.append(array.reshape(-1))

    xyz = np.column_stack(arrays).astype(np.float64, copy=False)
    if remove_nonfinite:
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if remove_zero_points:
        xyz = xyz[~np.all(xyz == 0.0, axis=1)]
    return np.ascontiguousarray(xyz)


def pointcloud2_message_to_xyz(
    message: object,
    *,
    remove_nonfinite: bool = True,
    remove_zero_points: bool = True,
) -> np.ndarray:
    """Duck-type a sensor_msgs/PointCloud2 without importing ROS."""

    required = ("height", "width", "fields", "is_bigendian", "point_step", "row_step", "data")
    missing = [name for name in required if not hasattr(message, name)]
    if missing:
        raise TypeError(f"message is missing PointCloud2 attributes: {', '.join(missing)}")
    fields = tuple(
        PointFieldSpec(field.name, field.offset, field.datatype, field.count)
        for field in message.fields
    )
    return pointcloud2_buffer_to_xyz(
        PointCloud2Buffer(
            height=message.height,
            width=message.width,
            fields=fields,
            is_bigendian=message.is_bigendian,
            point_step=message.point_step,
            row_step=message.row_step,
            data=message.data,
        ),
        remove_nonfinite=remove_nonfinite,
        remove_zero_points=remove_zero_points,
    )


def summarize_xyz_quality(xyz: Sequence[Sequence[float]]) -> PointCloudXYZQuality:
    points = np.asarray(xyz, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("xyz must have shape (N, 3)")
    finite_mask = np.isfinite(points).all(axis=1)
    finite = points[finite_mask]
    zero_points = int(np.count_nonzero(finite_mask & np.all(points == 0.0, axis=1)))
    if not len(finite):
        return PointCloudXYZQuality(
            total_points=len(points),
            finite_points=0,
            nonfinite_points=len(points),
            zero_points=0,
            x_range=None,
            y_range=None,
            z_range=None,
            distance_range_m=None,
            median_distance_m=None,
        )
    distances = np.linalg.norm(finite, axis=1)
    ranges = [
        (float(np.min(finite[:, index])), float(np.max(finite[:, index])))
        for index in range(3)
    ]
    return PointCloudXYZQuality(
        total_points=len(points),
        finite_points=len(finite),
        nonfinite_points=len(points) - len(finite),
        zero_points=zero_points,
        x_range=ranges[0],
        y_range=ranges[1],
        z_range=ranges[2],
        distance_range_m=(float(np.min(distances)), float(np.max(distances))),
        median_distance_m=float(np.median(distances)),
    )


__all__ = [
    "PointCloud2Buffer",
    "PointCloudXYZQuality",
    "PointFieldSpec",
    "pointcloud2_buffer_to_xyz",
    "pointcloud2_message_to_xyz",
    "summarize_xyz_quality",
]
