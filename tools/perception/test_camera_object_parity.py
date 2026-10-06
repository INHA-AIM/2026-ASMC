import unittest
from tools.perception.camera_object_parity import boundary_only


class BoundaryExceptionTest(unittest.TestCase):
    def test_only_threshold_insertions_and_deletions_qualify(self):
        a, b = ("person", 1, 2, 3, 4), ("car", 5, 6, 7, 8)
        near = [a + (0.701,), b + (0.9,)]
        self.assertEqual([("old_only", a)], boundary_only([a, b], [b], near))
        self.assertEqual([("new_only", a)], boundary_only([b], [a, b], near))
        self.assertIsNone(boundary_only([a, b], [a], near))

    def test_coordinates_order_and_mixed_edits_never_qualify(self):
        a, b = ("person", 1, 2, 3, 4), ("person", 2, 2, 3, 4)
        near = [a + (0.701,), b + (0.701,)]
        self.assertIsNone(boundary_only([a], [b], near))
        self.assertIsNone(boundary_only([a, b], [b, a], near))
        self.assertIsNone(boundary_only([a, b], [b, ("person", 3, 2, 3, 4)], near))


if __name__ == "__main__":
    unittest.main()
