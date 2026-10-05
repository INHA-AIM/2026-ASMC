from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from inventory_capture import inspect_frame


class CaptureInventoryTest(unittest.TestCase):
    def test_sentinel_and_box_full_dimensions_use_correct_columns(self):
        with tempfile.TemporaryDirectory() as directory:
            prefix = Path(directory) / "test_000000"
            xyz = np.array([[0., 0., .04], [4., 1., 1.]], dtype="<f4")
            for suffix, values in [("Intensity.bin", [0, 255]),
                                   ("Semantic.bin", [0, 125]), ("Instance.bin", [0, 17])]:
                np.column_stack((xyz, np.array(values, dtype="<f4"))).astype("<f4").tofile(str(prefix) + "_" + suffix)
            Path(str(prefix) + "_Instance.txt").write_text("Sedan 4 1 1 0 0 .2 4.5 1.8 1.5 4.24 0 0 0 17\n")
            Path(str(prefix) + "_Instance_8Points.txt").write_text("")
            result = inspect_frame(Path(str(prefix) + "_Intensity.bin"))
            self.assertTrue(result["xyz_exactly_equal"])
            self.assertEqual(result["raw_points"], 2)
            self.assertEqual(result["valid_points"], 1)
            self.assertEqual(result["boxes"][0]["dimensions_m"], [4.5, 1.8, 1.5])
            self.assertEqual(result["boxes"][0]["id"], 17)
            self.assertEqual(result["boxes"][0]["points"], 1)

    def test_missing_companions_do_not_become_empty_scene(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test_000000_Intensity.bin"
            path.write_bytes(b"")
            result = inspect_frame(path)
            self.assertEqual(len(result["missing"]), 4)
            self.assertNotIn("boxes", result)


if __name__ == "__main__":
    unittest.main()
