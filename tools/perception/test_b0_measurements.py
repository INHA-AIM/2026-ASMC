import unittest

from tools.perception.b0_measurements import (output_report, output_gate, resource_report,
                                             fusion_report, gps_phase_report)


def output(t, stamp, valid=True, anchor='matched'):
    return dict(arrival=t, stamp_ns=stamp, valid=valid, source_age_ms=25.,
                wall_latency_ms=20. if anchor == 'matched' else None, input_anchor=anchor)


class B0MeasurementsTest(unittest.TestCase):
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
