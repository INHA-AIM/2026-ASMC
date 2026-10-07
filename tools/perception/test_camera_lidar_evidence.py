import unittest

from tools.perception.summarize_camera_lidar_evidence import input_coverage, native_semantic_log


class SupplementalEvidenceTest(unittest.TestCase):
    def test_native_periodic_log_is_not_a_claim_of_final_accounting(self):
        row = native_semantic_log("semantic fps=29.9 output=100 batches=20 replaced={'front': 4} "
                                  "decode_errors=0 inference_errors=0 avg_ms={'inference_ms': 25.1}")
        self.assertEqual({'front': 4}, row['replaced_by_view_at_last_periodic_log'])
        self.assertFalse(row['final_shutdown_counter'])
        self.assertIsNone(native_semantic_log('no semantic sample'))

    def test_input_coverage_excludes_standby_and_drain(self):
        observation = dict(inputs={'front': [dict(arrival=t, stamp_ns=s) for t, s in ((1, 1), (10.1, 2), (10.2, 3), (11.1, 4))]},
                           outputs={'yolo': [dict(arrival=10.3, stamp_ns=3)]},
                           requests=[dict(model='yolo', mode=2, call_started=10)])
        report = dict(phase='camera', measurements={'yolo': dict(duration_sec=1)})
        row = input_coverage(observation, report)['yolo']
        self.assertEqual(2, row['input_frames'])
        self.assertEqual([2], row['unprocessed_source_stamps'])

    def test_full_bag_coverage_respects_supplied_input_end(self):
        observation = dict(inputs={'front': [dict(arrival=t, stamp_ns=s) for t, s in ((10, 1), (10.1, 2), (11.5, 3))],
                                   'lidar': [dict(arrival=10)]},
                           outputs={'yolo': [dict(arrival=10.2, stamp_ns=2), dict(arrival=11.8, stamp_ns=3)]})
        report = dict(phase='fusion', measurements={'yolo': dict(duration_sec=2, window_end_monotonic=11.5)})
        row = input_coverage(observation, report)['yolo']
        self.assertEqual(2, row['input_frames'])
        self.assertEqual(1, row['output_frames'])


if __name__ == '__main__':
    unittest.main()
