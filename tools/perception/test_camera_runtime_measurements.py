import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools/perception'))
sys.path.insert(0, str(ROOT/'tools/audit'))
from compare_camera_runtime import measurements, resource_summary
from summarize_camera_visualization import readiness_events
from b0_resources import ResourceSampler, _TimeoutTransport, qt_viewers
import tempfile
import subprocess
from unittest.mock import patch
from validate_p1b_contract import load_message_specs, all_message_md5
from evaluate_camera_bbox_pilot import frame_counts, summarize


class RuntimeMeasurementsTest(unittest.TestCase):
    def test_qt_inventory_distinguishes_owned_user_and_zombie_processes(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            for pid,state,args in [(1,'S',['python3','/opt/rqt_image_view','__name:=camera_runtime_viewer']),
                                   (2,'S',['python3','/opt/rqt_image_view']),
                                   (3,'Z',['python3','/opt/rqt_image_view','__name:=camera_runtime_viewer']),
                                   (4,'S',['bash','-c','rqt_image_view __name:=camera_runtime_viewer'])]:
                p=root/str(pid);p.mkdir();(p/'stat').write_text(str(pid)+' (test) '+state+' 0 0')
                (p/'cmdline').write_bytes(('\0'.join(args)+'\0').encode())
            result=qt_viewers(root)
            self.assertEqual([r['pid'] for r in result],[1,2])
            self.assertEqual([r['owned_trial'] for r in result],[True,False])

    def test_child_stop_uses_selected_grace_then_kills_ignored_interrupt(self):
        from run_b0_matrix import stop
        from unittest.mock import Mock
        process=Mock(pid=123);process.poll.return_value=None
        process.wait.side_effect=[subprocess.TimeoutExpired('qt',5),0]
        with patch('run_b0_matrix.os.killpg') as kill:
            stop(process,timeout=5)
        self.assertEqual(process.wait.call_args_list[0].kwargs,dict(timeout=5))
        self.assertEqual([c.args[0] for c in kill.call_args_list],[123,123])
        self.assertEqual(kill.call_count,2)

    def test_readiness_links_only_prior_independent_family_observations(self):
        values=dict(pipeline_ready='False',family_source_ns='10')
        diagnostics=[dict(at=5,statuses=[dict(name='camera_semantic_perception/readiness',values=values)])]
        families=[dict(at=3,stamp=10,valid=True),dict(at=6,stamp=30,valid=True)]
        result=readiness_events(diagnostics,families,0,10)[0]
        self.assertFalse(result['newer_family_already_seen'])
        families.append(dict(at=4.9,stamp=20,valid=True))
        result=readiness_events(diagnostics,families,0,10)[0]
        self.assertTrue(result['newer_family_already_seen'])
        self.assertEqual(result['latest_probe_family_stamp_ns'],20)
        self.assertAlmostEqual(result['latest_probe_family_receipt_age_ms'],100)
        self.assertFalse(readiness_events(diagnostics,families,6,10))

    def test_resource_coverage_missing_cpu_is_unknown_not_zero(self):
        row = dict(arrival=.1, gpu=dict(memory_used_mib=100, memory_total_mib=1000),
                   nodes={'node':dict(cpu_percent=None,rss_mib=1)})
        result = resource_summary([row],0,60)
        self.assertIsNone(result['camera_cpu_mean'])
        self.assertFalse(result['resource_coverage_pass'])
        complete = [dict(row,arrival=i*.5,nodes={'node':dict(cpu_percent=25,rss_mib=1)})
                    for i in range(120)]
        result = resource_summary(complete,0,60)
        self.assertTrue(result['resource_coverage_pass'])
        self.assertEqual(result['camera_cpu_mean'],25)
        self.assertFalse(resource_summary(complete[:50]+complete[60:],0,60)['resource_coverage_pass'])

    def test_xmlrpc_discovery_uses_bounded_transport_and_failure_backoff(self):
        connection = _TimeoutTransport().make_connection('127.0.0.1:11311')
        self.assertEqual(connection.timeout,.25)
        sampler=ResourceSampler('http://127.0.0.1:11311',['/missing'])
        with patch('b0_resources.ServerProxy',side_effect=TimeoutError) as proxy:
            sampler._discover(); sampler._discover()
            self.assertEqual(proxy.call_count,1)
            self.assertIsInstance(proxy.call_args.kwargs['transport'],_TimeoutTransport)
        sampler.stop.set()
        with patch('b0_resources.ServerProxy') as proxy:
            sampler._discover()
            proxy.assert_not_called()

    def test_raw_percentiles_full_windows_gaps_and_duplicate_count(self):
        rows = [dict(event='output', at=.1+i*.1, stamp=i+1, received=.05+i*.1,
                     wall_ms=i, valid=True, stale=False,
                     topic='/perception/camera/objects', frame='camera_front') for i in range(59)]
        result = next(iter(measurements(rows,0,5.9,15).values()))
        self.assertEqual(len(result['five_second_hz']), 1)
        self.assertEqual(result['five_second_hz'][0],9.8)
        self.assertAlmostEqual(result['wall_ms']['p50'],28.5)
        self.assertAlmostEqual(result['wall_ms']['p95'],54.15)
        self.assertFalse(result['target_95pct_pass'])
        self.assertTrue(result['minimum_9_5_pass'])
        self.assertFalse(result['minimum_10_pass'])
        self.assertEqual(result['duplicate_stamps'],0)
        rows.append(dict(rows[10],at=5.85))
        result = next(iter(measurements(rows,0,5.9,15).values()))
        self.assertEqual(result['duplicate_stamps'],1)
        self.assertEqual(result['reversed_stamps'],1)

    def test_invalid_and_stale_outputs_do_not_raise_fresh_rate(self):
        row = dict(event='output',at=1,stamp=1,received=.9,wall_ms=100,
                   topic='/perception/camera/objects',frame='camera_front',valid=False,stale=False)
        self.assertFalse(measurements([row,dict(row,valid=True,stale=True)],0,5,15))

    def test_frozen_old_md5_and_additive_observation_fields(self):
        specs = load_message_specs(ROOT/'src/asmc_msgs/msg')
        md5 = all_message_md5(specs)
        self.assertEqual(md5['CameraObjectBox'],'34412c90553e06734f1acade77c8e1e3')
        self.assertEqual(md5['CameraObjectBoxArray'],'310c671afb3693e7909db6453d115b79')
        self.assertEqual(md5['CameraObjectObservation'],'fbddb8fe0071685b60811037c0302106')
        self.assertEqual([f.name for f in specs['CameraObjectObservation'].fields],
                         ['detections','producer_session_id','generation','calibration_id','model_id','config_id'])

    def test_frozen_small_positive_duplicate_fp_partial_and_sha(self):
        row=dict(id='a',image_sha256='sha',supervision='complete',split_group='run',annotations=[
            dict(training_class='car',xyxy=[0,0,8,8],supervision='positive'),
            dict(training_class='person',xyxy=[20,20,22,22],supervision='ignore')])
        boxes=[dict(class_name='car',xyxy=[0,0,8,8],confidence=.8),
               dict(class_name='car',xyxy=[0,0,8,8],confidence=.7),
               dict(class_name='person',xyxy=[20,20,22,22],confidence=.9)]
        counts,pairs=frame_counts(row,boxes,.4,pixel_scope=True)
        self.assertEqual(len(pairs),1)
        self.assertEqual(counts['car']['complete_gt'],1)  # no legacy 24/12px display filter
        self.assertEqual(counts['car']['complete_predictions'],2)
        self.assertEqual(counts['person']['complete_predictions'],0)
        partial=dict(row,supervision='partial')
        values,_=frame_counts(partial,boxes,.4,pixel_scope=True)
        self.assertEqual(values['car']['tp'],1)
        self.assertEqual(values['car']['complete_predictions'],0)
        with self.assertRaises(ValueError):
            summarize([row],[dict(id='a',image_sha256='wrong',predictions=boxes)],.4,pixel_scope=True)


if __name__ == '__main__':
    unittest.main()
