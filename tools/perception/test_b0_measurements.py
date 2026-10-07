import unittest
from pathlib import Path
from collections import Counter
import xml.etree.ElementTree as ET
from unittest.mock import MagicMock, patch
import yaml

from tools.perception.b0_measurements import (output_report, output_gate, resource_report, input_window_end, make_protocol,
                                             fusion_report, gps_phase_report)


def output(t, stamp, valid=True, anchor='matched'):
    return dict(arrival=t, stamp_ns=stamp, valid=valid, source_age_ms=25.,
                wall_latency_ms=20. if anchor == 'matched' else None, input_anchor=anchor)


class B0MeasurementsTest(unittest.TestCase):
    def test_resource_discovery_uses_private_xmlrpc_clients_from_uri(self):
        from tools.perception.b0_resources import ResourceSampler
        master, node = MagicMock(), MagicMock()
        master.__enter__.return_value = master
        node.__enter__.return_value = node
        master.lookupNode.return_value = (1, '', 'http://127.0.0.1:8888')
        node.getPid.return_value = (1, '', 123)
        sampler = ResourceSampler('http://127.0.0.1:11312', ['/Main'])
        with patch('tools.perception.b0_resources.ServerProxy', side_effect=[master, node]) as factory:
            sampler._discover()
        self.assertEqual({'/Main': 123}, sampler.pids)
        self.assertEqual(['http://127.0.0.1:11312', 'http://127.0.0.1:8888'],
                         [call.args[0] for call in factory.call_args_list])

    def test_fixed_protocol_keeps_three_repeats_and_rotates_order(self):
        root = Path(__file__).resolve().parents[2]
        config = yaml.safe_load((root/'config/experiments/camera_lidar_b0_v1.yaml').read_text())
        jobs = make_protocol(config, ['camera', 'fusion', 'gps', 'debug'])
        self.assertEqual(51, len(jobs))
        cameras = [j for j in jobs if j['phase'] == 'camera' and not j.get('debug')]
        self.assertEqual({name: 3 for name in config['profiles']}, dict(Counter(j['profile'] for j in cameras)))
        self.assertEqual(['C1', 'C2', 'C3'], [next(j['profile'] for j in cameras if j['repeat'] == r) for r in (1, 2, 3)])
        self.assertEqual(6, sum(j['phase'] == 'gps' for j in jobs))
        self.assertEqual(3, sum(bool(j.get('gps_gap')) for j in jobs))

    def test_test_launch_quarantines_existing_main_control_outputs(self):
        launch = ET.parse(Path(__file__).with_name('b0_main.launch')).getroot()
        group = launch.find('group')
        remaps = {r.get('from'): r.get('to') for r in group.findall('remap')}
        self.assertEqual('/b0/discarded_ctrl_cmd_0', remaps['/ctrl_cmd_0'])
        self.assertEqual('/b0/discarded_ctrl_cmd', remaps['/ctrl_cmd'])
        self.assertEqual('false', next(a.get('value') for a in group.find('include').findall('arg') if a.get('name') == 'start_yolo'))

    def test_old_epoch_and_invalidity_are_distinct(self):
        row = output_report([output(10.05, 5), output(10.1, 11, False), output(10.2, 12)], 10, 11, 10)
        self.assertEqual(2, row['messages'])
        self.assertEqual(1, row['excluded_old_epoch_messages'])
        self.assertEqual(.5, row['valid_ratio'])

    def test_rpc_anchor_cannot_shift_to_first_output(self):
        rows = [output(10.25+i*.1, 100+i) for i in range(48)]
        row = output_report(rows, 10, 15)
        self.assertEqual(8, row['first_second'])
        self.assertFalse(output_gate('front', row))

    def test_duplicate_and_reverse_fail_even_at_good_hz(self):
        rows = [output(i*.05+.01, i+1) for i in range(100)]
        rows[50]['stamp_ns'] = 1
        row = output_report(rows, 0, 5)
        self.assertEqual(1, row['duplicate_stamps'])
        self.assertEqual(1, row['reversed_stamps'])
        self.assertFalse(output_gate('yolo', row))

    def test_partial_window_never_passes(self):
        rows = [output(i*.05+.01, i+1) for i in range(99)]
        row = output_report(rows, 0, 4.99, expected_seconds=5)
        self.assertFalse(row['complete'])
        self.assertFalse(output_gate('yolo', row))

    def test_bag_shutdown_drain_is_not_an_active_output_window(self):
        rows = [output(i*.1+.02, i+1) for i in range(200)]
        rows.append(output(20.4, 200, False))
        measured = output_report(rows, 0, 20.)
        self.assertEqual(200, measured['messages'])
        self.assertEqual(0, measured['duplicate_stamps'])
        self.assertTrue(output_gate('front', measured, lifecycle=False))
        drained = output_report(rows, 0, 20.6)
        self.assertFalse(output_gate('front', drained, lifecycle=False))

    def test_full_bag_family_requires_all_three_camera_inputs(self):
        inputs = {'front': [dict(arrival=19.)], 'left': [dict(arrival=19.1)], 'right': [dict(arrival=18.9)]}
        self.assertEqual(18.9, input_window_end('family', inputs, 20.))
        self.assertEqual(19., input_window_end('yolo', inputs, 20.))
        self.assertIsNone(input_window_end('resnet18', inputs, 20.))

    def test_ambiguous_family_anchor_is_not_a_latency_sample(self):
        row = output_report([output(.1, 1, anchor='ambiguous')], 0, 1)
        self.assertEqual(1, row['input_anchor_ambiguous'])
        self.assertEqual(0, row['input_to_output_wall_ms']['samples'])
        self.assertIsNone(row['input_to_output_wall_ms']['p95'])

    def test_missing_gpu_metric_is_not_zero(self):
        report = resource_report([dict(gpu=None, nodes={})])
        self.assertFalse(report['gpu_available'])
        self.assertIsNone(report['gpu_memory_mib']['maximum'])
        self.assertEqual(1, report['gpu_missing_samples'])

    def test_shutdown_tail_must_reconcile(self):
        log = 'Fusion sync tail: matched=2 dropped=0 offset median/p95/max=1/2/3 ms\nFusion sync total: matched=3 dropped=0 pending lidar/yolo=0/0'
        report = fusion_report(log, [], [], [])
        self.assertFalse(report['accounting_complete'])
        self.assertFalse(fusion_report('', [], [], [])['accounting_complete'])

    def test_lidar_stamp_lookup_respects_existing_millisecond_truncation(self):
        inputs = [dict(stamp_ns=123456789), dict(stamp_ns=223456789)]
        report = fusion_report('', [], inputs, [dict(stamp_ns=123000000)])
        self.assertEqual([223000000], report['unpublished_raw_lidar_stamps_ns'])

    def test_gps_gap_has_distinct_input_output_and_recovery(self):
        start = 100000000000
        inputs = {'lidar': [dict(ros_ns=start+int(t*1e9)) for t in (7, 9, 15)]}
        tracks = [dict(ros_ns=start+int(t*1e9)) for t in (7, 14.4)]
        relays = [dict(elapsed_sec=t, delivered=not(8 <= t < 14)) for t in (7, 8, 13, 14)]
        report = gps_phase_report(inputs, tracks, relays, start)
        self.assertEqual(1, report['gap']['sensor_inputs']['lidar'])
        self.assertEqual(0, report['gap']['gps_delivered'])
        self.assertEqual(0, report['gap']['tracked_frames'])
        self.assertAlmostEqual(.4, report['first_tracked_after_restore_sec'])


if __name__ == '__main__':
    unittest.main()
