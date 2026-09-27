from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from capture_stationary_scenes import lane_pose


class LanePoseTest(unittest.TestCase):
    def test_interpolates_by_distance_not_vertex_number(self):
        pose = lane_pose([[0, 0, 0], [1, 0, 0], [1, 9, 0]], .5, 28.4)
        self.assertEqual(pose["x"], 1)
        self.assertEqual(pose["y"], 4)
        self.assertEqual(pose["yaw"], 90)
        self.assertEqual(pose["z"], 28.4)

    def test_endpoints_and_duplicate_vertices(self):
        points = [[4, 2], [4, 2], [1, 2]]
        self.assertEqual(lane_pose(points, 0, 28)["x"], 4)
        end = lane_pose(points, 1, 28)
        self.assertEqual(end["x"], 1)
        self.assertEqual(end["yaw"], 180)

    def test_invalid_and_degenerate_inputs_fail(self):
        for points, fraction in [([[0, 0]], .5), ([[0, 0], [0, 0]], .5),
                                 ([[0, 0], [1, 1]], -.1), ([[0, 0], [1, 1]], 1.1)]:
            with self.assertRaises(ValueError):
                lane_pose(points, fraction, 28)


if __name__ == "__main__":
    unittest.main()
