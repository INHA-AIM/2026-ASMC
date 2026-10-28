import unittest
from scen_runner.records.local_route_review import project


class SegmentReviewTest(unittest.TestCase):
    def test_mid_segment_has_no_nearest_vertex_floor(self):
        s,error,signed=project([[0,0,0],[.25,0,0]], [10.,10.25], [.125,0])
        self.assertAlmostEqual(s,10.125)
        self.assertAlmostEqual(error,0.)

    def test_clamp_and_sign_and_duplicate_point(self):
        s,e,signed=project([[0,0,0],[0,0,0],[1,0,0]], [0.,0.,1.], [.5,-.2])
        self.assertAlmostEqual(s,.5)
        self.assertAlmostEqual(e,.2)
        self.assertAlmostEqual(signed,-.2)
        s,e,_=project([[0,0,0],[1,0,0]], [0.,1.], [2,0])
        self.assertEqual(s,1.)
        self.assertEqual(e,1.)
