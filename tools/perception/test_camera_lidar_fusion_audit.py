#!/usr/bin/env python3
import sqlite3
import unittest
from audit_camera_lidar_fusion import timing_relation,exact_timing,decision_errors

class F1Tests(unittest.TestCase):
    def timing(self,**kw):
        return dict(bridge_session_id='s',source_valid=True,source_clock_domain='morai_sensor',source_sec=3,source_nsec=4,**kw)
    def test_clocks_are_not_directly_subtracted(self):
        a=self.timing();b={**a,'source_clock_domain':'velodyne_device_hour'}
        self.assertEqual(timing_relation(a,b)['reason'],'different_source_clock')
        self.assertIsNone(timing_relation(a,b)['source_delta_ns'])
        self.assertEqual(timing_relation(a,{**a,'bridge_session_id':'old'})['reason'],'different_bridge_session')
        self.assertEqual(timing_relation(a,{**a,'source_regression':True})['reason'],'source_regression')
        self.assertEqual(timing_relation(a,None)['reason'],'timing_missing_or_ambiguous')
    def test_identical_stamp_other_view_and_restart_are_not_joined(self):
        db=sqlite3.connect(':memory:');db.execute('CREATE TABLE timing(topic,frame,stamp,wire,payload)')
        db.execute("INSERT INTO timing VALUES('/front','front',123,8,'{}')")
        self.assertIsNone(exact_timing(db,'/front','left',123,8))
        self.assertIsNone(exact_timing(db,'/front','front',123,9))
        self.assertEqual(exact_timing(db,'/front','front',123,8),{})
        db.execute("INSERT INTO timing VALUES('/front','front',123,8,'{}')")
        self.assertIsNone(exact_timing(db,'/front','front',123,8))
    def test_historical_scan_and_signed_offset(self):
        row=dict(stamp_ns=200,fusion=dict(emitting_scan_stamp_ns=200))
        d=dict(action='match',scan_stamp_ns=100,image_stamp_ns=110,image_minus_scan_ns=10,projection_valid=True)
        self.assertEqual(decision_errors(row,d),[])
        self.assertEqual(decision_errors(row,{**d,'image_minus_scan_ns':-10}),['signed_offset_mismatch'])
        self.assertEqual(decision_errors(row,{**d,'projection_valid':False}),['matched_invalid_projection'])

if __name__=='__main__':unittest.main()
