import unittest
from copy import deepcopy

from analyze_coco7_broad_selection import selected_frames, confirmation_summary


class ReviewedExclusionsTest(unittest.TestCase):
    def setUp(self):
        self.gt = [dict(image=f'{n}.png', source_sha256=dict(intensity=str(n)*64)) for n in (1, 2)]
        self.predictions = deepcopy(self.gt)
        self.exclusions = [dict(image='1.png', sha256='1'*64)]

    def test_only_exact_reviewed_frame_is_excluded_without_mutating_originals(self):
        gt, predictions = selected_frames(self.gt, self.predictions, self.exclusions)
        self.assertEqual(['2.png'], list(gt))
        self.assertEqual(['2.png'], [p['image'] for p in predictions])
        self.assertEqual(2, len(self.gt))
        self.assertEqual(2, len(self.predictions))

    def test_exclusion_requires_matching_rgb_not_just_filename(self):
        self.exclusions[0]['sha256'] = 'a'*64
        with self.assertRaisesRegex(ValueError, 'SHA mismatch'):
            selected_frames(self.gt, self.predictions, self.exclusions)

    def test_incomplete_or_duplicate_predictions_cannot_improve_metrics(self):
        for rows in (self.predictions[:1], self.predictions+[self.predictions[0]]):
            with self.assertRaises(ValueError):
                selected_frames(self.gt, rows, self.exclusions)

    def test_changed_prediction_rgb_is_rejected_even_if_frame_is_excluded(self):
        self.predictions[0]['source_sha256']['intensity'] = 'b'*64
        with self.assertRaisesRegex(ValueError, 'prediction RGB SHA mismatch'):
            selected_frames(self.gt, self.predictions, self.exclusions)

    def test_foreign_or_duplicate_exclusion_is_rejected(self):
        for rows in ([dict(image='other.png', sha256='1'*64)], self.exclusions*2):
            with self.assertRaises(ValueError):
                selected_frames(self.gt, self.predictions, rows)


class ConfirmationTest(unittest.TestCase):
    candidate = dict(model='yolo26l', width=1280, confidence=.5)

    def fixture(self):
        metric = dict(hz=14.9, input_to_output_wall_ms=dict(p95=90., p99=110.),
                      max_gap_sec=.2, duplicate_stamps=0, reversed_stamps=0)
        return [dict(job=dict(self.candidate, weather_requested=w, repeat=n),
                     observation_window_completed=True, passed=False,
                     failures=['planning trajectory deadline gate'],
                     measurements={k: deepcopy(metric) for k in ('front', 'left', 'right',
                         'resnet18', 'yolo_front', 'yolo_left', 'yolo_right')},
                     target_attainment={v: dict(met=True) for v in ('front', 'left', 'right')},
                     resources_startup_and_active=dict(gpu_memory_mib=dict(maximum=5800.)))
                for w in ('sunny', 'foggy') for n in (1, 2, 3)]

    def test_camera_pass_does_not_hide_planner_failure(self):
        result = confirmation_summary(self.fixture(), [self.candidate])[0]
        self.assertTrue(result['camera_supply_passed'])
        self.assertFalse(result['whole_stack_passed'])
        self.assertEqual(3, len(result['failures']))

    def test_missing_repeat_or_model_output_is_not_pass(self):
        rows = self.fixture()
        self.assertFalse(confirmation_summary(rows[1:], [self.candidate])[0]['camera_supply_passed'])
        del rows[0]['measurements']['resnet18']
        self.assertFalse(confirmation_summary(rows, [self.candidate])[0]['camera_supply_passed'])

    def test_duplicate_repeat_and_unknown_failure_cannot_pass(self):
        rows = self.fixture()
        with self.assertRaises(ValueError):
            confirmation_summary(rows+[rows[0]], [self.candidate])
        rows[0]['failures'].append('unexpected worker failure')
        self.assertFalse(confirmation_summary(rows, [self.candidate])[0]['camera_supply_passed'])


if __name__ == '__main__':
    unittest.main()
