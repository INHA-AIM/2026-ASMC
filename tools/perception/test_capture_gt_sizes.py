import unittest
from audit_capture_gt_sizes import tensor_size, category


class CaptureSizesTest(unittest.TestCase):
    def test_view_scale_and_thin_boxes(self):
        self.assertEqual(tensor_size([0,0,8,16], (1280,720), 960), (6,12))
        self.assertEqual(tensor_size([0,0,8,16], (640,480), 960), (12,24))
        self.assertEqual(tensor_size([0,0,8,16], (640,480), 960, False), (8,16))
        self.assertEqual(category(1,100), 'near_point')
        self.assertEqual(category(4,4), 'very_small')
        self.assertEqual(category(8,16), 'larger')

    def test_malformed_geometry_is_not_a_small_positive(self):
        for box in ([0,0,0,1], [0,0,float('nan'),1], [2,0,1,1]):
            with self.assertRaises(ValueError):
                tensor_size(box, (1280,720), 960)


if __name__ == '__main__':unittest.main()
