#!/usr/bin/env python3
"""Offline contract tests: no ROS master, no writes to operational data."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import unittest
import tempfile

import numpy as np

SCRIPT = Path(__file__).resolve().parents[1] / 'tools/compare_hd_map_filters.py'
SPEC = importlib.util.spec_from_file_location('hd_map_audit', SCRIPT)
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


def box(x, y, length=2, width=1, yaw=0, z=0, height=1):
    xy = np.array([[-length/2,-width/2],[length/2,-width/2],[length/2,width/2],[-length/2,width/2]])
    c,s=math.cos(yaw),math.sin(yaw)
    xy=xy @ np.array([[c,s],[-s,c]]) + [x,y]
    corners=[[*p,z+h] for h in (0,height) for p in xy]
    return dict(track_id=1, center=[x,y,z+height/2], corners=corners, length=length, width=width,
                height=height,yaw=yaw,classification='unknown',camera_class='unknown',observation_state='observed')


def frame():
    return dict(schema=audit.SCHEMA,frame_id='map',status=1,map_id='test',map_origin=[1,2,0],stamp_ns=1_000_000_000,
                pose=dict(valid=True,at_stamp_ns=1_000_000_000,position_error_verified=True,position_error_bound_m=0.1,source_age_ms=100),
                mission_scope_verified=True, objects=[])


class AuditTest(unittest.TestCase):
    def setUp(self):
        self.map=audit.MapAudit.__new__(audit.MapAudit)
        self.map.occupancy=np.full((120,120),100,np.int8)
        self.map.occupancy[20:100,48:54]=0
        self.map.exceptions=np.zeros((120,120),np.uint8)
        self.map.exceptions[60:80,25:35]=2
        self.map.origin=np.array([0.,0.]);self.map.resolution=1.
        self.map.product={'map_id':'test'}
        self.config={'candidate_guard_m':1.,'exception_protection_m':3.,'map_origin':[1,2,0],'pose_age_limit_ms':300}

    def evaluate(self,obj=None,source=None):
        return self.map.evaluate(source or frame(),obj or box(80,40),self.config)

    def test_clear_outside_only_candidate(self):
        self.assertEqual(self.evaluate()['decision'],'exclude_candidate')

    def test_road_and_boundary_keep_without_clipping(self):
        for x in (50,48,55,56):
            o=box(x,40);before=copy.deepcopy(o)
            self.assertEqual(self.evaluate(o)['decision'],'keep')
            self.assertEqual(o,before)

    def test_exception_space_even_when_ego_not_in_mission(self):
        f=frame();f['ego_link']='ordinary'
        self.assertEqual(self.evaluate(box(30,68),f)['reason'],'exception_space_protected')

    def test_each_exception_type(self):
        for i in (1,2,3):
            self.map.exceptions[60:80,25:35]=i
            self.assertEqual(self.evaluate(box(30,68))['decision'],'keep')

    def test_missing_or_invalid_pose_no_exclusion(self):
        for pose in ({},dict(valid=False),dict(valid=True,at_stamp_ns=0)):
            f=frame();f['pose']=pose
            self.assertEqual(self.evaluate(source=f)['decision'],'undetermined')

    def test_missing_accuracy_is_not_zero_error(self):
        f=frame();f['pose'].pop('position_error_verified')
        self.assertEqual(self.evaluate(source=f)['reason'],'pose_error_bound_unverified')

    def test_pose_age_and_scope_do_not_imply_precision(self):
        for age in (None,-1,301,float('inf')):
            f=frame();f['pose']['source_age_ms']=age
            self.assertEqual(self.evaluate(source=f)['decision'],'undetermined')
        f=frame();f['mission_scope_verified']=False
        self.assertEqual(self.evaluate(source=f)['reason'],'mission_scope_unverified')

    def test_larger_pose_uncertainty_preserves_boundary(self):
        f=frame();f['pose']['position_error_bound_m']=8
        self.assertEqual(self.evaluate(box(62,40),f)['decision'],'keep')

    def test_outside_map_and_unknown_cells(self):
        self.assertEqual(self.evaluate(box(-1,40))['decision'],'undetermined')
        self.map.occupancy[35:45,75:85]=-1
        self.assertEqual(self.evaluate()['reason'],'unknown_map_cells')

    def test_invalid_or_huge_geometry(self):
        for o in (dict(corners=[]),dict(corners=[[1,1,0]]*8),box(80,40,length=1e6)):
            self.assertEqual(self.evaluate(o)['decision'],'undetermined')

    def test_all_eight_corners_affect_footprint(self):
        o=box(58,40);o['corners'][4][0]=50
        self.assertEqual(self.evaluate(o)['decision'],'keep')

    def test_rotated_box_and_predicted_state(self):
        o=box(50,40,yaw=math.pi/4);o['observation_state']='predicted'
        d=self.evaluate(o)
        self.assertEqual(d['decision'],'keep');self.assertEqual(d['input_state'],'predicted')

    def test_invalid_velocity_not_reinterpreted_or_mutated(self):
        o=box(50,40);o.update(absolute_velocity_valid=False,absolute_velocity_mps=[100,0,0])
        before=copy.deepcopy(o);self.evaluate(o);self.assertEqual(o,before)

    def test_wrong_frame_map_origin_and_status(self):
        for key,value in [('frame_id','base_link'),('map_id','other'),('map_origin',[0,0,0]),('status',4)]:
            f=frame();f[key]=value
            self.assertEqual(self.evaluate(source=f)['decision'],'undetermined')

    def test_reproducible_decision_ignoring_measured_time(self):
        a=self.evaluate();b=self.evaluate();a.pop('comparison_ms');b.pop('comparison_ms')
        self.assertEqual(a,b)

    def test_actual_map_links_and_hash_checked(self):
        c=audit.yaml.safe_load((SCRIPT.parent/'filter_audit/audit.yaml').read_text())
        m=audit.MapAudit(*(audit.REPO/c[k] for k in ('product','grid','metadata','source')))
        self.assertIn('A2256W001125',m.exception_links['dynamic_obstacle'])
        self.assertNotIn('A2256W000236',m.exception_links['dynamic_obstacle'])
        self.assertFalse(any(z['links_without_area'] for z in m.mission_audit))


    def test_committed_synthetic_cases_keep_required_objects(self):
        c=audit.yaml.safe_load((SCRIPT.parent/'filter_audit/audit.yaml').read_text())
        m=audit.MapAudit(*(audit.REPO/c[k] for k in ('product','grid','metadata','source')))
        fixture=SCRIPT.parents[1]/'test/fixtures/hd_map_filter_audit/cases.jsonl'
        for line in fixture.read_text().splitlines():
            f=json.loads(line)
            with self.subTest(case=f['case']):
                before=copy.deepcopy(f)
                d=m.evaluate(f,f['objects'][0],c)
                self.assertEqual(d['decision'],f['expected_decision'])
                self.assertEqual(f,before)
                if f['objects'][0]['required_object']:
                    self.assertNotEqual(d['decision'],'exclude_candidate')


    def test_gallery_draws_clipped_legacy_corners_without_mutation(self):
        obj=box(50,40);before=copy.deepcopy(obj)
        self.map.product['drivable_areas']=[]
        row=dict(object=obj,candidate=self.evaluate(obj),legacy={'main':dict(geometry_clipped=True,output_corners=box(50,40,width=.5)['corners'])},
                 frame_id='map',source_id='unit',key='test',stamp_ns=1,source_kind='synthetic_fixture',pose={})
        with tempfile.TemporaryDirectory() as directory:
            audit.write_gallery(Path(directory),[row],self.map)
            svg=(Path(directory)/'case-000.svg').read_text()
            self.assertIn('stroke-dasharray',svg)
            self.assertTrue((Path(directory)/'index.html').is_file())
        self.assertEqual(obj,before)


if __name__=='__main__':
    unittest.main()
