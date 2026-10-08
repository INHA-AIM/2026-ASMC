import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
sys.path.insert(0,str(Path(__file__).parent))
from evaluate_coco7_models import evaluate, summary
from run_coco7_live_trials import measurement_gate, deadline_report
from b0_measurements import output_report
from fusion_log_accounting import parse_fusion_logs
from analyze_coco7_trials import temporal_report
from analyze_coco7_fusion import reuse_report
from verify_coco7_runtime import check_view


def ann(label, verified=True):
    return dict(source_class_name=label,subtype_verified=verified,xyxy=[0,0,10,10],actor_id='a')


class Coco7EvaluationTest(unittest.TestCase):
    def test_drive_gate_honors_stop_and_caps_trajectory_speed(self):
        from drive_coco7_preview import speed_command
        preview=SimpleNamespace(brake=0.,longlCmdType=1,velocity=0.)
        trajectory=SimpleNamespace(valid=True,points=[SimpleNamespace(velocity=4.)])
        self.assertEqual(5.,speed_command(preview,trajectory))
        preview.brake=1.
        self.assertEqual(0.,speed_command(preview,trajectory))
        preview.brake=0.;preview.longlCmdType=2;preview.velocity=2.
        self.assertEqual(2.,speed_command(preview,trajectory))
        preview.velocity=0.
        self.assertEqual(0.,speed_command(preview,trajectory))
        trajectory.valid=False
        self.assertEqual(0.,speed_command(preview,trajectory))

    def test_three_view_verifier_checks_each_source_stream(self):
        rows=[dict(arrival=i*.1+.025,stamp_ns=i+1,valid=True,
                   wall_latency_ms=25.,source_age_ms=25.,input_anchor='matched') for i in range(50)]
        # Three cameras may share stamps; each camera is checked separately.
        for _ in range(3): self.assertTrue(check_view(rows,0,5,9.5)['passed'])
        self.assertFalse(check_view([],0,5,9.5)['passed'])
        bad=[dict(r) for r in rows];bad[5]['wall_latency_ms']=None;bad[5]['input_anchor']='missing'
        self.assertFalse(check_view(bad,0,5,9.5)['passed'])

    def test_reuse_counts_geometry_pairs_without_duplicating_temporal_class_evidence(self):
        first=dict(event='camera_observation_use',stamp_ns=100,source_stamp_ns=100,
                   camera_id='front',calibration_id='c',epoch=1,reused=False)
        again=dict(first,stamp_ns=120,reused=True)
        observe=dict(event='class_observation',stamp_ns=100,id=7)
        final=dict(event='fusion_summary',matched=2,fresh=1,reused=1)
        self.assertTrue(reuse_report([first,observe,again,final],'bounded_reuse')['passed'])
        self.assertFalse(reuse_report([first,observe,again,observe,final],'bounded_reuse')['passed'])
        empty=dict(event='yolo_input',stamp_ns=110,boxes=[])
        admitted=dict(event='yolo_admitted',stamp_ns=110)
        self.assertFalse(reuse_report([first,observe,empty,admitted,again,final],'bounded_reuse')['passed'])
        waiting_old_lidar=dict(again,stamp_ns=105)
        self.assertFalse(reuse_report([first,observe,empty,admitted,waiting_old_lidar,final],'bounded_reuse')['passed'])
        self.assertFalse(reuse_report([first,observe,again],'bounded_reuse')['passed'])

    def test_unknown_subtype_is_not_a_negative_or_a_positive(self):
        c,ignored,pairs,_,_=evaluate([ann('Vehicle',False)],
            [dict(source_class_name='car',confidence=.9,xyxy=[0,0,10,10])],.7)
        self.assertEqual(1,ignored);self.assertEqual([],pairs)
        self.assertIsNone(summary(c)['macro_f1'])
        self.assertEqual(0,c['car']['fp'])

    def test_raw_labels_are_not_combined_and_duplicates_are_false_positive(self):
        ps=[dict(source_class_name=l,confidence=.9,xyxy=[0,0,10,10]) for l in ('car','car','truck')]
        c,_,pairs,_,_=evaluate([ann('car')],ps,.7)
        self.assertEqual(1,len(pairs));self.assertEqual(1,c['car']['tp'])
        self.assertEqual(1,c['car']['fp']);self.assertEqual(1,c['truck']['fp'])

    def test_wall_latency_missing_partial_window_and_duplicates_fail(self):
        rows=[dict(arrival=i*.1+.025,stamp_ns=i+1,valid=True,wall_latency_ms=25.,source_age_ms=25.,input_anchor='matched') for i in range(50)]
        self.assertTrue(measurement_gate(output_report(rows,0,5,expected_seconds=5)))
        self.assertFalse(measurement_gate(output_report(rows,0,4.9,expected_seconds=5)))
        rows[25]['stamp_ns']=1
        self.assertFalse(measurement_gate(output_report(rows,0,5,expected_seconds=5)))

    def test_idle_planner_and_missing_tail_do_not_pass(self):
        self.assertFalse(deadline_report([],0,5,.25,19)['passed'])
        self.assertFalse(deadline_report([{'arrival':i*.05} for i in range(95)],0,5,.25,19)['passed'])
        self.assertTrue(deadline_report([{'arrival':i*.05} for i in range(100)],0,5,.25,19)['passed'])

    def test_fusion_reuse_accounting_is_backward_compatible_and_conserved(self):
        tail='Fusion sync tail: matched=3 dropped=1 offset median/p95/max=10/20/30 ms\n'
        legacy='Fusion sync total: matched=3 dropped=1 pending lidar/yolo=0/0'
        self.assertTrue(parse_fusion_logs(tail+legacy)[2])
        total='Fusion sync total: matched=3 dropped=1 fresh=2 reused=1 pending lidar/yolo=0/0'
        self.assertTrue(parse_fusion_logs(tail+total)[2])
        self.assertEqual(1,parse_fusion_logs(tail+total)[1]['reused'])
        self.assertFalse(parse_fusion_logs(tail+total.replace('reused=1','reused=2'))[2])

    def test_stable_detection_requires_five_consecutive_inputs(self):
        gt={str(i):dict(run='r',view='front',frame=str(i),annotations=[ann('car')]) for i in range(7)}
        rows=[dict(image=str(i),predictions=[dict(source_class_name='car',confidence=.9,xyxy=[0,0,10,10])] if i in (1,3,4) else []) for i in range(7)]
        r=temporal_report(gt,rows,.7)['objects'][0]
        self.assertEqual(4,r['first_stable_frame'])
        missing=temporal_report(gt,[rows[i] for i in (0,1,4,5,6)],.7)
        self.assertEqual(1,missing['insufficient_contiguous_gt'])
        self.assertEqual(0,missing['not_stably_detected'])


if __name__=='__main__':unittest.main()
