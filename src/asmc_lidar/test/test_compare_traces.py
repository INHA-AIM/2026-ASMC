#!/usr/bin/env python3
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
spec=importlib.util.spec_from_file_location('compare_traces',Path(__file__).resolve().parents[1]/'tools/compare_traces.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class TraceComparison(unittest.TestCase):
    def frame(self):
        return dict(generation=1,config_id='config',calibration_id='calib',producer_session_id='a',
            stamp_ns=42,timings_ms=dict(total=5),typed=dict(local=dict(objects=[dict(id=7,x=3.)])),
            hd_map_audit=dict(calculation_ms=1,policy=dict(camera_age_ms=500),
                              objects=[dict(comparison_ms=.1,road_ratio=.25,keep=True)]))
    def test_timing_is_ignored_without_mutating_original(self):
        a=self.frame();saved=copy.deepcopy(a);b=copy.deepcopy(a)
        b['producer_session_id']='b';b['timings_ms']['total']=900
        b['hd_map_audit']['calculation_ms']=99;b['hd_map_audit']['objects'][0]['comparison_ms']=99
        self.assertEqual(m.numerical(a,True),m.numerical(b,True));self.assertEqual(a,saved)
    def test_policy_geometry_and_reason_differences_remain_failures(self):
        for mutation in (lambda x:x['hd_map_audit']['policy'].update(camera_age_ms=501),
                         lambda x:x['hd_map_audit']['objects'][0].update(road_ratio=.26),
                         lambda x:x['typed']['local']['objects'][0].update(x=3.01)):
            a=self.frame();b=copy.deepcopy(a);mutation(b)
            self.assertNotEqual(m.numerical(a,True),m.numerical(b,True))
    def test_strict_ids_and_old_comparison_policy_are_distinct(self):
        for field in ('generation','config_id','calibration_id'):
            a=self.frame();b=copy.deepcopy(a);b[field]='different'
            self.assertNotEqual(m.numerical(a,True),m.numerical(b,True))
            self.assertEqual(m.numerical(a),m.numerical(b))
    def test_empty_or_different_lengths_cannot_pass(self):
        with tempfile.TemporaryDirectory() as d:
            a=Path(d)/'a';b=Path(d)/'b';a.write_text('');b.write_text('')
            self.assertFalse(m.compare(a,b,True)['equal'])
            a.write_text(json.dumps(self.frame())+'\n')
            self.assertFalse(m.compare(a,b,True)['equal'])
if __name__=='__main__':unittest.main()
