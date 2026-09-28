import unittest
import numpy as np
if __package__:
    from .fixtures import first_return_scene
else:
    from fixtures import first_return_scene


class FirstReturnTest(unittest.TestCase):
    def test_no_wall_returns_behind_the_visible_object_on_same_ray(self):
        points, labels, box = first_return_scene("car")
        directions = points[:, :3] / np.linalg.norm(points[:, :3], axis=1)[:, None]
        # Each angular direction contributes exactly one intersection.
        self.assertEqual(len(np.unique(np.round(directions, 6), axis=0)), len(points))
        self.assertGreater(np.count_nonzero(labels == 3), 10)
        obj = points[labels == 3, :3]
        self.assertTrue(np.all(obj >= np.array(box["lo"]) - 1e-5))
        self.assertTrue(np.all(obj <= np.array(box["hi"]) + 1e-5))

    def test_left_right_symmetry_and_determinism(self):
        left = first_return_scene("person", 1)
        right = first_return_scene("person", -1)
        self.assertEqual(np.count_nonzero(left[1] == 3), np.count_nonzero(right[1] == 3))
        repeated = first_return_scene("person", 1)
        np.testing.assert_array_equal(left[0], repeated[0])
        np.testing.assert_array_equal(left[1], repeated[1])

    def test_wall_only_has_no_target_label(self):
        for shape in ("straight", "step", "curve"):
            points, labels, box = first_return_scene(None, shape=shape)
            self.assertIsNone(box)
            self.assertFalse(np.any(labels == 3))
            self.assertTrue(np.isfinite(points).all())


if __name__ == "__main__":
    unittest.main()
