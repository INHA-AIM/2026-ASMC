import unittest
from analyze_camera_small_supervision import footprint, rejected

class SmallSupervisionTest(unittest.TestCase):
    def test_cell_boundary_has_no_extra_cell(self):
        self.assertEqual(footprint([[0,0,8,8]],8).sum(),1)
        self.assertEqual(footprint([[7,7,9,9]],8).sum(),4)
        self.assertEqual(footprint([[7,7,9,9]],8,'center').sum(),0)

    def test_downsample_and_native_both_apply(self):
        self.assertTrue(rejected([0,0,4,8],1280,720,'short4_area32'))
        self.assertTrue(rejected([0,0,3,20],640,480,'short4_area32'))
        self.assertFalse(rejected([0,0,4,8],960,960,'short4_area32'))

    def test_overlapping_ignore_union(self):
        self.assertEqual(footprint([[1,1,2,2],[2,2,3,3]],8).sum(),1)

if __name__=='__main__':unittest.main()
