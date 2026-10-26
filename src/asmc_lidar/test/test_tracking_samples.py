#!/usr/bin/env python3
import importlib.util
from pathlib import Path
from types import SimpleNamespace as N
import unittest
spec=importlib.util.spec_from_file_location('tracking_samples',Path(__file__).resolve().parents[1]/'tools/tracking_samples.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class Samples(unittest.TestCase):
    def frame(self):
        p=N(track_id=7,age=3,miss_count=0,observation_state=1,classification='vehicle',
            center=N(x=2.,y=3.,z=1.),absolute_velocity_valid=True,absolute_velocity_mps=N(x=3.,y=4.,z=0.),dynamic_valid=False)
        a=N(header=N(frame_id='map'),status=1,objects=[p])
        return N(observation=N(objects=a,producer_session_id='session',generation=1,scan_sequence=2),tracking=[N(track_id=7,motion_state=0)])
    def test_invalid_velocity_and_pose_are_not_zero_measurements(self):
        f=self.frame();rows,reason=m.observation_samples(f);self.assertEqual(rows[0]['speed'],5.)
        f.observation.objects.objects[0].absolute_velocity_valid=False
        self.assertEqual(m.observation_samples(f),([], 'invalid_velocity_samples:1'))
        f.observation.objects.status=4
        self.assertEqual(m.observation_samples(f),([], 'map_observation_unavailable'))
    def test_identity_alignment_and_empty(self):
        f=self.frame();f.tracking[0].track_id=99
        self.assertEqual(m.observation_samples(f),([], 'tracking_alignment'))
        f.observation.objects.objects=[];f.tracking=[];f.observation.objects.status=2
        self.assertEqual(m.observation_samples(f),([], 'observed_empty'))
if __name__=='__main__':unittest.main()
