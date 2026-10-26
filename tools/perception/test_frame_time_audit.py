#!/usr/bin/env python3
"""Small C0 regression. No bag dataset, GPU, external research clone or ROS master."""
import copy
import json
import os
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).parent))
from audit_frame_time import timing_eligibility, stats, pose_comparison, invariant, json_probe, projection_cases

class AuditTests(unittest.TestCase):
    def test_clock_session_and_fault_gates(self):
        a=dict(recording='one',clock='morai_sensor',session='a',valid=True)
        self.assertEqual(timing_eligibility(a,a),'comparable')
        for field,value,reason in [('recording','two','different_recording'),('clock','velodyne_device_hour','different_clock'),('session','b','different_session'),('valid',False,'invalid'),('duplicate',True,'duplicate'),('regression',True,'regression'),('conflict',True,'conflict')]:
            self.assertEqual(timing_eligibility(a,dict(a,**{field:value})),reason)
        self.assertEqual(timing_eligibility(None,a),'missing')
    def test_async_state_is_not_same_time_error(self):
        scan=dict(stamp_ns=200000000,arrival_ns=300000000,map=dict(valid=True,map_from_base=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]))
        state=dict(scan_ns=scan['stamp_ns'],arrival_ns=scan['arrival_ns'],state_ns=190000000,valid=True,x=1,y=0,yaw=0)
        r=pose_comparison(scan,state)
        self.assertEqual(r['reason'],'asynchronous_policy_difference');self.assertEqual(r['dt_ms'],-10)
        self.assertEqual(pose_comparison(scan,dict(state,state_ns=200000000))['reason'],'same_time')
        self.assertEqual(pose_comparison(scan,dict(state,valid=False))['reason'],'pose_unavailable')
        with self.assertRaises(ValueError):pose_comparison(scan,dict(state,scan_ns=100))
    def test_object_data_not_mutated_or_dropped(self):
        row={'producer_session_id':'a','typed':{'objects':[{'track_id':1,'velocity':[3,0,0],'corners':[[1,2,3]]}]},'comparison_ms':1}
        original=copy.deepcopy(row);normalized=invariant(row)
        self.assertEqual(row,original);self.assertEqual(row['typed'],normalized['typed'])
        changed=copy.deepcopy(row);changed['typed']['objects'][0]['track_id']=2
        self.assertNotEqual(invariant(row),invariant(changed))
    def test_empty_statistics_are_unavailable(self):
        self.assertEqual(stats([None,float('nan')]),{'count':0})
        self.assertEqual(stats([1,2,3])['p50'],2)

class CoreTests(unittest.TestCase):
    def test_54_original_pixel_cases(self):
        result=projection_cases(Path(os.environ['C0_PROBES'])/'frame_projection_probe')
        self.assertEqual(len(result['cases']),54);self.assertTrue(result['passed'])
    def test_context_roundtrip_and_invalid_rotation(self):
        from scipy.spatial.transform import Rotation
        import numpy as np
        p=np.eye(4);p[:3,:3]=Rotation.from_euler('xyz',[.1,-.08,.9]).as_matrix();p[:3,3]=[15,20,.36]
        data=dict(map_from_base=p.ravel().tolist(),alternative=dict(x=16,y=21,yaw=1.),points=[[1,2,3],[-4,5,6],[10,-3,-.36]])
        result=json_probe(Path(os.environ['C0_PROBES'])/'context_probe',[data])[0]
        self.assertLess(result['roundtrip_m'],1e-8)
        self.assertGreater(np.max(np.abs(np.array(result['original'])-result['alternative'])),.1)
        data['map_from_base'][0]=2
        import subprocess
        with self.assertRaises(subprocess.CalledProcessError):json_probe(Path(os.environ['C0_PROBES'])/'context_probe',[data])

if __name__=='__main__':
    # CI explicitly requests both sets with C0_PROBES; host-only contract tests need no ROS/numpy.
    suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromTestCase(AuditTests))
    if os.environ.get('C0_PROBES'):suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(CoreTests))
    sys.exit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
