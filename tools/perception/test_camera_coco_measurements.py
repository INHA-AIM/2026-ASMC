import unittest
from camera_coco_measurements import common_label_comparison, make_protocol, select_gallery, validate_profiles


class CocoMeasurementTest(unittest.TestCase):
    def test_duplicate_or_incomplete_allowlist_and_nonclass_change_rejected(self):
        baseline = dict(allowed_names=['bus', 'truck', 'person', 'car'], conf_thres=.7)
        expanded = dict(allowed_names=['label%d' % i for i in range(80)], conf_thres=.7)
        self.assertEqual([], validate_profiles(baseline, expanded, {'excluded': []}))
        expanded['allowed_names'][1] = expanded['allowed_names'][0]
        self.assertTrue(validate_profiles(baseline, expanded, {'excluded': []}))
        expanded['conf_thres'] = .6
        self.assertIn('class expansion changed conf_thres', validate_profiles(baseline, expanded, {'excluded': []}))

    def test_model_name_mismatch_and_excluded_infrastructure_rejected(self):
        baseline = dict(allowed_names=['bus', 'truck', 'person', 'car'])
        expanded = dict(allowed_names=['label%d' % i for i in range(80)])
        errors = validate_profiles(baseline, expanded, {'excluded': ['stop sign']}, {0: 'person'})
        self.assertEqual(2, len(errors))

    def test_all_repeats_and_debug_fusion_are_preserved(self):
        jobs = make_protocol(dict(repeats=3, fusion=dict(repeats=2, bags={'primary': '', 'shadow': ''})), ['camera', 'debug', 'fusion'])
        self.assertEqual(20, len(jobs))
        self.assertEqual(['baseline', 'all', 'all', 'baseline', 'baseline', 'all'], [j['classes'] for j in jobs if j['phase']=='camera' and not j['debug']])
        self.assertEqual(8, sum(j['phase']=='fusion' for j in jobs))

    def test_top_max_det_loss_and_order_change_are_not_exact(self):
        def box(label, x): return dict(source_class_name=label, xyxy=[x, 0, x+1, 1], confidence=.9)
        old = [box('car', 0), box('person', 2)]
        self.assertTrue(common_label_comparison(old, [old[0], box('stop sign', 4), old[1]])['exact'])
        self.assertFalse(common_label_comparison(old, [old[1], old[0]])['exact'])
        self.assertFalse(common_label_comparison(old, [old[0], box('stop sign', 4)])['exact'])

    def test_gallery_has_first_best_uniform_and_no_baseline_labels(self):
        rows = [dict(all=[dict(source_class_name='stop sign', confidence=.99 if i==3 else .7),
                          dict(source_class_name='car', confidence=.9)]) for i in range(20)]
        selected = select_gallery(rows)
        self.assertEqual(['stop sign'], list(selected))
        self.assertIn(0, selected['stop sign'])
        self.assertIn(3, selected['stop sign'])
        self.assertIn(19, selected['stop sign'])
        self.assertLessEqual(len(selected['stop sign']), 5)
        self.assertEqual({}, select_gallery([]))


if __name__ == '__main__':
    unittest.main()
