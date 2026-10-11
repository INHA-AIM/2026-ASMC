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
    def test_grpc_diagnostic_speed_unit_and_ceiling_are_explicit(self):
        from drive_coco7_preview import grpc_speed_mps
        self.assertEqual(0.,grpc_speed_mps(0.))
        self.assertAlmostEqual(1.,grpc_speed_mps(3.6))
        self.assertAlmostEqual(5/3.6,grpc_speed_mps(5.))
        for invalid in (-1.,5.01,float('nan'),float('inf')):
            with self.assertRaises(ValueError):grpc_speed_mps(invalid)

    def test_reused_image_origin_cannot_be_replaced_by_a_future_lidar_update(self):
        from analyze_fusion_trace import object_state
        obj=dict(id=7)
        rows=[dict(event='class_after',stamp_ns=100,yolo_stamp_ns=90,objects=[obj]),
              dict(event='class_current',stamp_ns=110,yolo_stamp_ns=90,objects=[obj]),
              dict(event='class_after',stamp_ns=120,yolo_stamp_ns=90,objects=[obj]),
              dict(event='class_current',stamp_ns=125,yolo_stamp_ns=90,objects=[obj])]
        self.assertEqual(100,object_state(rows,'class_current',110,7)[0]['fused_lidar_stamp_ns'])
        self.assertEqual(120,object_state(rows,'class_current',125,7)[0]['fused_lidar_stamp_ns'])
        rows.insert(1,dict(event='camera_epoch_invalidated',stamp_ns=105))
        self.assertIsNone(object_state(rows,'class_current',110,7)[0]['fused_lidar_stamp_ns'])

    def test_fusion_rate_override_replaces_view_map_without_mutating_source(self):
        from run_coco7_fusion_trials import pin_uniform_rate
        original=dict(target_hz=20.,target_hz_by_view=dict(front=20.,left=15.,right=15.),conf_thres=.65)
        pinned=pin_uniform_rate(original,10.)
        self.assertEqual(dict(front=10.,left=10.,right=10.),pinned['target_hz_by_view'])
        self.assertEqual(10.,pinned['target_hz'])
        self.assertEqual(20.,original['target_hz_by_view']['front'])
        self.assertEqual(.65,pinned['conf_thres'])
        for hz in (0,-1,21,float('nan')):
            with self.assertRaises(ValueError): pin_uniform_rate(original,hz)

    def test_debug_measurement_consumes_exactly_the_enabled_outputs(self):
        from run_coco7_live_trials import debug_topics, debug_masks_enabled
        self.assertIs(False,debug_masks_enabled(dict(model='yolo26m-seg')))
        self.assertEqual('false',str(debug_masks_enabled({})).lower())
        self.assertIs(True,debug_masks_enabled(dict(debug=True)))
        self.assertEqual([],debug_topics(dict(model='yolo26m-seg',debug=False)))
        masks=debug_topics(dict(model='yolo26m-seg',debug=True))
        standard=debug_topics(dict(model='yolo26m-seg',debug=True,debug_masks=False))
        self.assertEqual(10,len(set(masks)))
        self.assertEqual(7,len(set(standard)))
        self.assertIn('/perception/camera/objects/debug/mosaic/compressed',standard)
        self.assertFalse(any('/instances/' in t for t in standard))
        self.assertEqual(standard,debug_topics(dict(model='yolo26m',debug=True)))

    def test_matrix_retains_failed_repeat_and_rejects_partial_or_invalid_provenance(self):
        import json
        import tempfile
        from summarize_coco7_live_matrix import summarize as live_summary
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            for experiment,repeats in [('live-failed',[1,2,3]),('live-partial',[1,2]),('live-complete',[1,2,3])]:
                for repeat in repeats:
                    directory=root/experiment/str(repeat);directory.mkdir(parents=True)
                    job=dict(model='m',width=960,rates='all10',batch='batch3',profile='C6',repeat=repeat)
                    passed=not (experiment=='live-failed' and repeat==2)
                    (directory/'report.json').write_text(json.dumps(dict(job=job,passed=passed,failures=[] if passed else ['gap'])))
            rejected=root/'live-aborted';rejected.mkdir()
            (rejected/'REJECTED_PROVENANCE.json').write_text('{}')
            result=live_summary(root)
            rows={r['experiment']:r for r in result['groups']}
            self.assertFalse(rows['live-failed']['all_passed'])
            self.assertEqual(1,len(rows['live-failed']['failures']))
            self.assertFalse(rows['live-partial']['all_passed'])
            self.assertTrue(rows['live-complete']['all_passed'])
            self.assertEqual(['live-aborted'],result['rejected_provenance'])

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

    def test_restoration_requires_stationary_pose_mode_and_gear(self):
        import copy
        from drive_coco7_preview import restoration_check
        xyz=lambda **kw:SimpleNamespace(**dict(x=kw.get('x',0.),y=0.,z=kw.get('z',0.)))
        target=SimpleNamespace(transform=SimpleNamespace(location=xyz(),rotation=xyz(z=359.9)),
            global_velocity=xyz(),vehicle_state=SimpleNamespace(gear_mode=1))
        state=copy.deepcopy(target);state.transform.rotation.z=-.1
        self.assertTrue(restoration_check(state,target,1,1)['passed'])
        for mutate in (lambda s:setattr(s.global_velocity,'x',.06),
                       lambda s:setattr(s.transform.location,'x',.11),
                       lambda s:setattr(s.transform.rotation,'z',2.),
                       lambda s:setattr(s.vehicle_state,'gear_mode',4)):
            state=copy.deepcopy(target);mutate(state)
            self.assertFalse(restoration_check(state,target,1,1)['passed'])
        self.assertFalse(restoration_check(target,target,3,1)['passed'])

    def test_reentry_distinguishes_absent_light_verdict_from_processing_supply(self):
        from run_coco7_drive_rehearsal import reentry_gate
        row=dict(complete=True,five_second_hz=[9.8,10.],messages=99,valid_messages=0,
            first_second=10,duplicate_stamps=0,reversed_stamps=0,max_gap_sec=.15,
            input_anchor_missing=0,input_to_output_wall_ms=dict(p95=90.,p99=100.))
        self.assertTrue(reentry_gate('resnet18',row))
        self.assertEqual(0,row['valid_messages'])
        self.assertFalse(reentry_gate('yolo',row))
        for field,value in [('first_second',8),('max_gap_sec',.3),('duplicate_stamps',1),('complete',False),('stale_messages',1)]:
            failing=dict(row);failing[field]=value
            self.assertFalse(reentry_gate('resnet18',failing))

    def test_restore_transform_mode_side_effect_is_repaired_last(self):
        import copy
        from unittest.mock import patch
        from drive_coco7_preview import restore_ego
        xyz=lambda:SimpleNamespace(x=0.,y=0.,z=0.)
        target=SimpleNamespace(transform=SimpleNamespace(location=xyz(),rotation=xyz()),
            global_velocity=xyz(),vehicle_state=SimpleNamespace(gear_mode=1))
        request=lambda **kw:SimpleNamespace(**copy.deepcopy(kw))
        modules={'morai.actor.actor_set_pb2':SimpleNamespace(**{name:request for name in
            ('EnableActor','SetVelocityParam','SetTransformParam','VehicleControlModeParam','VehicleGear')}),
            'morai.actor.actor_control_pb2':SimpleNamespace(VehicleCtrlCmd=request),
            'capture_coco7_actors':SimpleNamespace(MessageToDict=lambda _: {})}
        class Actors:
            def __init__(self):
                self.state=copy.deepcopy(target);self.mode=1;self.calls=[]
            def SetPause(self,r):self.calls.append('pause')
            def SetVelocity(self,r):self.calls.append('velocity')
            def ControlVehicle(self,r):self.calls.append('brake')
            def SetTransform(self,r):
                self.state.transform=copy.deepcopy(r.transform);self.mode=3
                self.state.transform.rotation.z+=1.344
                self.state.vehicle_state.gear_mode=4;self.calls.append('transform')
            def SetVehicleControlMode(self,r):self.mode=r.mode;self.calls.append('mode')
            def SetVehicleGear(self,r):self.state.vehicle_state.gear_mode=r.gear;self.calls.append('gear')
            def GetActorState(self,*args,**kw):return self.state
            def GetVehicleControlMode(self,*args,**kw):return SimpleNamespace(mode=self.mode)
        actors=Actors();report={}
        with patch.dict(sys.modules,modules):
            self.assertTrue(restore_ego(actors,SimpleNamespace(),target,1,lambda f,r:f(r),report,sleep=lambda _:None))
        self.assertEqual(2,actors.calls.count('transform'))
        self.assertAlmostEqual(-1.344,report['restore_yaw_correction_deg'])
        self.assertGreater(max(i for i,v in enumerate(actors.calls) if v=='mode'),max(i for i,v in enumerate(actors.calls) if v=='transform'))
        self.assertEqual(3,len(report['restoration_checks']))

    def test_policy_comparison_keeps_failed_repeat_and_classification_difference(self):
        import json
        import tempfile
        from compare_coco7_fusion_trials import compare_trials
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); index=[]
            for bag in ('primary','shadow'):
                for policy in ('unique','bounded_reuse'):
                    for repeat in (1,2,3):
                        bad=bag=='primary' and policy=='bounded_reuse' and repeat==2
                        label=f'{bag}-{policy}-r{repeat}'
                        directory=root/label;directory.mkdir()
                        frame=dict(stamp_ns=100,objects=[[7,'unknown' if bad else 'car','car']],
                                   centers={'7':[1.,2.,3.]})
                        (directory/'observations.json').write_text(json.dumps(dict(tracks=[frame])))
                        index.append(dict(path=label,complete=True,passed=not bad,checks={},
                            job=dict(bag_id=bag,policy=policy,repeat=repeat,diagnostic=False),
                            fusion=dict(totals=dict(matched=1),offset_p95_max_ms=1.)))
            (root/'index.json').write_text(json.dumps(index))
            result=compare_trials(root)
            self.assertEqual(['primary-bounded_reuse-r2'],result['failed_trials'])
            self.assertEqual(1,len(result['bags']['primary']['stable']['regressions']))
            (root/'index.json').write_text(json.dumps(index[:-1]))
            with self.assertRaisesRegex(ValueError,'three complete repeats'):compare_trials(root)
            (root/'REJECTED_PROTOCOL.json').write_text('{}')
            with self.assertRaisesRegex(ValueError,'rejected experiment'):compare_trials(root)

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
        older_empty=dict(event='yolo_input',stamp_ns=100,boxes=[])
        self.assertTrue(reuse_report([older_empty,first,observe,empty,admitted,waiting_old_lidar,final],'bounded_reuse')['passed'])
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
        self.assertEqual(1,r['reentries'])  # Initial miss -> first hit is acquisition.
        missing=temporal_report(gt,[rows[i] for i in (0,1,4,5,6)],.7)
        self.assertEqual(1,missing['insufficient_contiguous_gt'])
        self.assertEqual(0,missing['not_stably_detected'])
        self.assertEqual(0,missing['objects'][0]['reentries'])


if __name__=='__main__':unittest.main()
