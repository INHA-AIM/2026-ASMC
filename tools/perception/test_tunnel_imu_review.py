import unittest
from pathlib import Path
import tempfile
import json
from types import SimpleNamespace as NS
import numpy as np
from review_tunnel_imu_motion import mask_distances,temporal,MOTION
from run_tunnel_motion_trial import phase_command,project_route
from review_source_tunnel_motion import source_signals


class TunnelReviewTest(unittest.TestCase):
    def test_pixel_distances_keep_classes_and_exclude_nonfinite_offscreen(self):
        camera=NS(project_ground=lambda xy:(np.asarray(xy),np.ones(len(xy))))
        geometry=NS(scale_xy=(1,1),pad_ltrb=(0,0,0,0))
        classes=np.zeros((12,12),np.uint8);classes[:,4]=1;classes[:,8]=2
        result=mask_distances([[4,5],[6,5],[8,5],[float('nan'),0],[100,0]],
                              [1,1,2,1,1],camera,geometry,classes)
        self.assertEqual(result,{'1':[0.,2.],'2':[0.]})

    def test_temporal_metric_aligns_stationary_world_and_uses_identical_samples(self):
        def boundary(x,y):return dict(role='yellow_centre_boundary',class_id=2,
            xy=[[x+i,y] for i in range(16)],support=[1]*16)
        old=dict(run_id='r',phase='interior',stamp_ns=10**9,
            variants={name:dict(boundaries=[boundary(0,0)]) for name in MOTION})
        new=dict(run_id='r',phase='interior',stamp_ns=1100000000,
            variants={name:dict(boundaries=[boundary(-1,0 if name=='M0' else .2)]) for name in MOTION})
        signals={'r':{'/Ego_topic':[
            dict(source_ns=10**9,position_xyz=[0,0,0],heading_deg=0),
            dict(source_ns=1100000000,position_xyz=[1,0,0],heading_deg=0)]}}
        pairs=temporal([old,new],signals)
        self.assertEqual(len(pairs),1)
        self.assertAlmostEqual(pairs[0]['variants']['M0']['p95'],0.)
        self.assertAlmostEqual(pairs[0]['variants']['M1_receive']['p95'],.2)
        self.assertEqual(pairs[0]['variants']['M0']['count'],pairs[0]['variants']['M1_receive']['count'])

    def test_motion_envelope_and_route_projection_units(self):
        for t in np.linspace(0,25,200):
            _,speed,offset=phase_command(t)
            self.assertLessEqual(speed,5.5);self.assertGreaterEqual(speed,0)
            self.assertLessEqual(abs(offset),.22)
        self.assertEqual(phase_command(24)[1],0.)
        station,lateral,_=project_route(np.array([[0,0,0],[10,0,0],[20,0,0]]),np.array([5.,.4]))
        self.assertAlmostEqual(station,5.);self.assertAlmostEqual(lateral,.4)

    def test_source_review_requires_exact_wire_and_rejects_clock_conflict(self):
        imu=dict(header=dict(stamp=1234,seq=7,frame_id='imu'),
                 orientation=dict(x=0,y=0,z=0,w=1),orientation_covariance=[0]*9)
        timing=dict(published_stamp=1234,sensor_wire_sequence=7,sensor_frame_id='imu',
            bridge_session_id='b',sensor_wire_sequence_valid=True,source_valid=True,
            source_clock_domain='morai_sensor',source_duplicate=False,source_conflict=False,
            source_regression=False,source_sec=10,source_nsec=0)
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'events.jsonl'
            def save(t):
                path.write_text(json.dumps(dict(kind='imu',message=imu))+'\n'+
                                json.dumps(dict(kind='imu_timing',message=t))+'\n')
            save(timing)
            result,_,report=source_signals(Path(d),[])
            self.assertEqual(result[0]['source_ns'],10**10)
            self.assertEqual(report['unique_imu'],1)
            save(dict(timing,sensor_wire_sequence=8))
            result,_,report=source_signals(Path(d),[])
            self.assertEqual(result,[]);self.assertEqual(report['counts']['missing_timing'],1)
            save(dict(timing,source_conflict=True))
            with self.assertRaisesRegex(ValueError,'source conflict'):source_signals(Path(d),[])


if __name__=='__main__':unittest.main()
