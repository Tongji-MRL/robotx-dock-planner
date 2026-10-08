import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from dock_lidar_prototype.run_recorded_frame import (
    inspect_recorded_frame,
    load_points_xyz,
)
from dock_lidar_prototype.synthetic import (
    SyntheticDockConfig,
    generate_synthetic_dock_point_cloud,
)


class RecordedFrameTests(unittest.TestCase):
    def test_npy_frame_produces_machine_readable_result_and_png(self):
        cloud = generate_synthetic_dock_point_cloud(SyntheticDockConfig(seed=31))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "frame.npy"
            png_path = root / "result.png"
            json_path = root / "result.json"
            np.save(input_path, cloud, allow_pickle=False)

            result = inspect_recorded_frame(
                input_path,
                (0.0, 0.0),
                png_path,
                json_path,
            )

            self.assertEqual(load_points_xyz(input_path).shape, cloud.shape)
            self.assertIsNotNone(result["selected"])
            self.assertTrue(png_path.is_file())
            self.assertEqual(json.loads(json_path.read_text())["input_points"], len(cloud))

    def test_npz_requires_points_xyz_key(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.npz"
            np.savez(path, other=np.zeros((2, 3)))
            with self.assertRaisesRegex(ValueError, "points_xyz"):
                load_points_xyz(path)


if __name__ == "__main__":
    unittest.main()
