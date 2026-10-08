from array import array
import math
import struct
import unittest
from types import SimpleNamespace

from dock_planner import (
    PointCloud2Buffer,
    PointFieldSpec,
    pointcloud2_buffer_to_xyz,
    pointcloud2_message_to_xyz,
    summarize_xyz_quality,
)


LIVOX_FIELDS = (
    PointFieldSpec("x", 0, 7),
    PointFieldSpec("y", 4, 7),
    PointFieldSpec("z", 8, 7),
    PointFieldSpec("intensity", 12, 7),
    PointFieldSpec("tag", 16, 2),
    PointFieldSpec("line", 17, 2),
    PointFieldSpec("timestamp", 18, 8),
)


def livox_cloud():
    data = bytearray(3 * 26)
    struct.pack_into("<ffffBBd", data, 0, 1.0, 2.0, 3.0, 20.0, 1, 2, 100.0)
    struct.pack_into("<ffffBBd", data, 26, math.nan, 5.0, 6.0, 21.0, 3, 4, 101.0)
    struct.pack_into("<ffffBBd", data, 52, 0.0, 0.0, 0.0, 22.0, 5, 6, 102.0)
    return PointCloud2Buffer(
        height=1,
        width=3,
        fields=LIVOX_FIELDS,
        is_bigendian=False,
        point_step=26,
        row_step=78,
        data=data,
    )


class PointCloudAdapterTests(unittest.TestCase):
    def test_extracts_xyz_from_real_mid360_mixed_field_layout(self):
        xyz = pointcloud2_buffer_to_xyz(
            livox_cloud(), remove_nonfinite=False, remove_zero_points=False,
        )
        self.assertEqual(xyz.shape, (3, 3))
        self.assertEqual(tuple(xyz[0]), (1.0, 2.0, 3.0))
        self.assertTrue(math.isnan(xyz[1, 0]))
        self.assertEqual(tuple(xyz[2]), (0.0, 0.0, 0.0))

    def test_default_filters_nonfinite_and_all_zero_points(self):
        xyz = pointcloud2_buffer_to_xyz(livox_cloud())
        self.assertEqual(xyz.shape, (1, 3))
        self.assertEqual(tuple(xyz[0]), (1.0, 2.0, 3.0))

    def test_zero_filter_can_be_disabled_for_raw_diagnostics(self):
        xyz = pointcloud2_buffer_to_xyz(
            livox_cloud(), remove_zero_points=False,
        )
        self.assertEqual(xyz.shape, (2, 3))
        self.assertEqual(tuple(xyz[1]), (0.0, 0.0, 0.0))

    def test_quality_summary_counts_finite_nonfinite_and_zero_points(self):
        xyz = pointcloud2_buffer_to_xyz(
            livox_cloud(), remove_nonfinite=False, remove_zero_points=False,
        )
        quality = summarize_xyz_quality(xyz)
        self.assertEqual(quality.total_points, 3)
        self.assertEqual(quality.finite_points, 2)
        self.assertEqual(quality.nonfinite_points, 1)
        self.assertEqual(quality.zero_points, 1)
        self.assertEqual(quality.x_range, (0.0, 1.0))
        self.assertAlmostEqual(quality.median_distance_m, math.sqrt(14.0) / 2.0)

    def test_duck_typed_ros_message_is_supported(self):
        cloud = livox_cloud()
        message = SimpleNamespace(
            height=cloud.height,
            width=cloud.width,
            fields=[SimpleNamespace(
                name=field.name,
                offset=field.offset,
                datatype=field.datatype,
                count=field.count,
            ) for field in cloud.fields],
            is_bigendian=cloud.is_bigendian,
            point_step=cloud.point_step,
            row_step=cloud.row_step,
            # rclpy deserializes sequence<uint8> as array.array('B').
            data=array("B", cloud.data),
        )
        self.assertEqual(pointcloud2_message_to_xyz(message).shape, (1, 3))

    def test_big_endian_xyz_layout(self):
        data = struct.pack(">fff", 1.5, -2.5, 3.5)
        cloud = PointCloud2Buffer(
            1,
            1,
            (
                PointFieldSpec("x", 0, 7),
                PointFieldSpec("y", 4, 7),
                PointFieldSpec("z", 8, 7),
            ),
            True,
            12,
            12,
            data,
        )
        self.assertEqual(tuple(pointcloud2_buffer_to_xyz(cloud)[0]), (1.5, -2.5, 3.5))

    def test_missing_xyz_is_rejected(self):
        cloud = PointCloud2Buffer(
            1, 1, (PointFieldSpec("x", 0, 7),), False, 4, 4, b"\0" * 4,
        )
        with self.assertRaisesRegex(ValueError, "missing XYZ"):
            pointcloud2_buffer_to_xyz(cloud)


if __name__ == "__main__":
    unittest.main()
