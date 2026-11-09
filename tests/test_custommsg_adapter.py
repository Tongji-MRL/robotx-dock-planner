import math
import unittest
from types import SimpleNamespace

import numpy as np

from robotx_perception_common.custommsg_adapter import custom_msg_to_xyz
from robotx_perception_common.lidar_transform import custom_msg_to_base_link_xyz


class CustomMsgAdapterTests(unittest.TestCase):
    def test_extracts_xyz_and_filters_invalid_returns(self):
        message = SimpleNamespace(points=(
            SimpleNamespace(x=1.0, y=2.0, z=3.0),
            SimpleNamespace(x=0.0, y=0.0, z=0.0),
            SimpleNamespace(x=math.nan, y=1.0, z=2.0),
        ))
        result = custom_msg_to_xyz(message)
        np.testing.assert_allclose(result, [[1.0, 2.0, 3.0]])

    def test_transform_applies_measured_mid360_offset(self):
        message = SimpleNamespace(points=(
            SimpleNamespace(x=1.0, y=2.0, z=3.0),
        ))
        result = custom_msg_to_base_link_xyz(message)
        np.testing.assert_allclose(result, [[1.15, 2.10, 3.22]])

    def test_missing_points_is_rejected(self):
        with self.assertRaises(TypeError):
            custom_msg_to_xyz(SimpleNamespace())


if __name__ == "__main__":
    unittest.main()
