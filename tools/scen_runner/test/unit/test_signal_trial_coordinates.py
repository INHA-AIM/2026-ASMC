import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.morai.signal_trial_coordinates import Coordinates
from scen_runner.runtime.signal_trial_oracle import Oracle
# Support both the README's whole-suite discovery and unit-only discovery.
if __package__:
    from .test_signal_trial_oracle import actor
else:
    from test_signal_trial_oracle import actor


class CoordinateTest(unittest.TestCase):
    def test_kcity_heading_correction_and_round_trip(self):
        frame=Coordinates([302595.,4124145.]);offset=frame.yaw_offset(-105.9,-277.7)
        self.assertAlmostEqual(offset,-.023525,places=5)
        for grid in (0.,1.2,3.,-2.):
            enu=frame.enu_yaw_degrees(-105.9,-277.7,grid)
            self.assertAlmostEqual(frame.grid_yaw(-105.9,-277.7,enu),grid)

    def test_oracle_uses_grid_yaw_for_actor_and_box(self):
        frame=Coordinates([302595.,4124145.]);vehicle=actor(0)
        enu=frame.enu_yaw_degrees(0,0,0)
        vehicle.transform.rotation.z=vehicle.bounding_box.rotation.z=enu
        road={'drivable_areas':[{'polygon':[[-10,-1.00001],[20,-1.00001],[20,1.00001],[-10,1.00001]]}]}
        row=Oracle(road,[],frame).observe(vehicle,1,{})
        self.assertAlmostEqual(row['yaw'],0)
        self.assertTrue(row['road_ok'])


if __name__=='__main__':unittest.main()
