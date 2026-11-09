import math
import struct
import unittest
from types import SimpleNamespace

import numpy as np

from dock_planner import (
    LidarExtrinsics,
    default_mid360_extrinsics,
    pointcloud_message_to_base_link_xyz,
    transform_base_link_points_to_lidar,
    transform_lidar_point_to_base_link,
    transform_lidar_points_to_base_link,
)


class LidarTransformTests(unittest.TestCase):
    def test_default_measurement_translates_lidar_points_to_base_link(self):
        points = np.array(((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)))
        result = transform_lidar_points_to_base_link(points)
        np.testing.assert_allclose(result, ((0.15, 0.10, 0.22), (1.15, 0.10, 0.22)))

    def test_yaw_rotation_is_applied_before_translation(self):
        transform = LidarExtrinsics(yaw_rad=math.pi / 2.0)
        result = transform_lidar_point_to_base_link((1.0, 0.0, 0.0), transform)
        np.testing.assert_allclose(result, (0.15, 1.10, 0.22), atol=1e-12)

    def test_inverse_round_trip(self):
        transform = LidarExtrinsics(0.15, 0.0, 0.22, 0.02, -0.03, 0.04)
        points = np.array(((1.0, 2.0, 0.5), (-2.0, 0.4, 1.2)))
        base = transform_lidar_points_to_base_link(points, transform)
        recovered = transform_base_link_points_to_lidar(base, transform)
        np.testing.assert_allclose(recovered, points, atol=1e-12)

    def test_default_factory_matches_dataclass_defaults(self):
        self.assertEqual(default_mid360_extrinsics(), LidarExtrinsics())

    def test_invalid_points_are_rejected(self):
        with self.assertRaises(ValueError):
            transform_lidar_points_to_base_link(((1.0, 2.0),))
        with self.assertRaises(ValueError):
            transform_lidar_points_to_base_link(((1.0, 2.0, math.nan),))

    def test_pointcloud_message_is_extracted_then_transformed(self):
        data = struct.pack("<fff", 1.0, 2.0, 3.0)
        message = SimpleNamespace(
            height=1,
            width=1,
            fields=(
                SimpleNamespace(name="x", offset=0, datatype=7, count=1),
                SimpleNamespace(name="y", offset=4, datatype=7, count=1),
                SimpleNamespace(name="z", offset=8, datatype=7, count=1),
            ),
            is_bigendian=False,
            point_step=12,
            row_step=12,
            data=data,
        )
        result = pointcloud_message_to_base_link_xyz(message)
        np.testing.assert_allclose(result, ((1.15, 2.10, 3.22),))


if __name__ == "__main__":
    unittest.main()
