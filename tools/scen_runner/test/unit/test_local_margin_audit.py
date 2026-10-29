import unittest
from scen_runner.records.local_margin_audit import footprint


class NativeBoxAxesTest(unittest.TestCase):
    def state(self,kind,extents,x=0):
        return dict(kind=kind,box_extents=extents,box_center=[x,0,0],
                    pose=dict(roll=0,pitch=0,yaw=0))

    def test_vehicle_forward_extent_and_gap(self):
        a=footprint(self.state(1,[1,2,.8]))
        b=footprint(self.state(3,[.5,1,.6],3.2))
        self.assertAlmostEqual(a.area,8.)
        self.assertAlmostEqual(a.distance(b),.2)

    def test_pedestrian_vertical_axis_is_not_footprint_length(self):
        p=footprint(self.state(2,[.35,.9,.2]))
        self.assertAlmostEqual(p.area,.28)
        self.assertAlmostEqual(p.bounds[2]-.0,.2)
