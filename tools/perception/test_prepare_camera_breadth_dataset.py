import copy
from pathlib import Path
import unittest

from camera_data_audit import label_metadata, load_label_policy
from prepare_camera_breadth_dataset import expand

ROOT = Path(__file__).resolve().parents[2]


class BreadthDatasetTests(unittest.TestCase):
    def setUp(self):
        self.policy = load_label_policy(ROOT/'config/datasets/camera_object_labels_v2.json')

    def row(self, ident='a', target='car', split='train', eligible=False):
        ann = dict(id='1', original_label=target or 'mixed', canonical=target,
                   disposition='candidate' if target else 'ambiguous', xyxy=[10, 10, 30, 30])
        label_metadata(ann, self.policy)
        return dict(id=ident, source='roboflow:test/source', split=split, eligible=eligible,
                    width=100, height=80, image_sha256=ident*64, pixel_sha256=ident*64,
                    annotations=[ann], reasons=[] if eligible else [
                        'source_class_sample_failed', 'annotation_completeness_unverified'])

    def test_source_sample_and_vehicle_subclass_failures_become_visible_quality_tier(self):
        row = self.row(); before = copy.deepcopy(row)
        result, stats = expand([row], self.policy, {'a': dict(decision='reject')})
        self.assertTrue(result[0]['eligible'])
        self.assertFalse(result[0]['breadth_quality']['complete_target_annotation_asserted'])
        self.assertEqual(stats['new_images_with_previous_individual_review_failure'], 1)
        self.assertEqual(result[0]['annotations'], before['annotations'])
        self.assertEqual(row, before)

    def test_excluded_signals_do_not_block_target_or_become_negative_images(self):
        row = self.row(); signal = dict(id='2', original_label='traffic light',
            canonical=None, disposition='excluded', xyxy=[40, 10, 50, 20])
        label_metadata(signal, self.policy); row['annotations'].append(signal)
        other = self.row('b'); other['annotations'] = [signal]
        empty = self.row('c'); empty['annotations'] = []
        result, stats = expand([row, other, empty], self.policy, {})
        self.assertEqual([r['eligible'] for r in result], [True, False, False])
        self.assertEqual(stats['new_objects'], 1)

    def test_fixed_evaluation_and_existing_reviewed_subset_are_immutable(self):
        rows = [self.row('a', eligible=True), self.row('b', split='heldout'),
                self.row('c', split='selection')]
        before = copy.deepcopy(rows); result, stats = expand(rows, self.policy, {})
        self.assertEqual(stats['new_images'], 0)
        for old, new in zip(before, result):
            self.assertEqual(old['annotations'], new['annotations'])
            self.assertEqual(old['eligible'], new['eligible'])
            self.assertEqual(old['split'], new['split'])

    def test_verified_class_only_change_resolves_mixed_box_without_new_geometry(self):
        row = self.row(target=None)
        decision = dict(image_sha256=row['image_sha256'], reviewer='Codex independent RGB inspection',
            decision='pending', notes='Wooden crate visible; another target not annotated.',
            overrides=[dict(id='1', canonical='wooden_box')])
        result, stats = expand([row], self.policy, {'a': decision})
        self.assertEqual(stats['new_images'], 1)
        self.assertEqual(result[0]['annotations'][0]['training_class'], 'box')
        self.assertEqual(result[0]['annotations'][0]['xyxy'], row['annotations'][0]['xyxy'])
        decision['image_sha256'] = 'changed'
        with self.assertRaisesRegex(ValueError, 'SHA'): expand([row], self.policy, {'a': decision})

    def test_teacher_uncertain_and_geometry_proposals_cannot_resolve_mixed_truth(self):
        row = self.row(target=None)
        base = dict(image_sha256=row['image_sha256'], reviewer='Codex independent RGB inspection',
            notes='', overrides=[dict(id='1', canonical='dog')])
        for changes in (dict(requires_human_confirmation=True), dict(notes='uncertain dog/deer'),
                        dict(reviewer='teacher'), dict(additions=[dict(id='2')]),
                        dict(overrides=[dict(id='1', canonical='dog', xyxy=[1, 1, 5, 5])])):
            result, _ = expand([row], self.policy, {'a': dict(base, **changes)})
            self.assertFalse(result[0]['eligible'])

    def test_known_pe_mixture_needs_class_review_even_when_source_maps_to_drum(self):
        row = self.row(target='pe_drum')
        row['source'] = 'roboflow:tooth-decay-rnacu/morai_ai_mission-fhqkh'
        row['annotations'][0]['original_label'] = 'PE'
        config = dict(individual_label_review={row['source']: ['pe']})
        result, _ = expand([row], self.policy, {}, config)
        self.assertFalse(result[0]['eligible'])
        review = dict(image_sha256=row['image_sha256'], reviewer='Codex independent RGB inspection',
            overrides=[dict(id='1', canonical='pe_firewall')])
        result, _ = expand([row], self.policy, {'a': review}, config)
        self.assertTrue(result[0]['eligible'])
        self.assertEqual(result[0]['annotations'][0]['training_class'], 'plastic_barrier')

    def test_invalid_boxes_and_split_warning_are_not_admitted(self):
        bad = self.row(); bad['annotations'][0]['xyxy'] = [10, 10, 101, 30]
        near = self.row('b'); near['reasons'] += ['cross_split_similar_image_review']
        result, _ = expand([bad, near], self.policy, {})
        self.assertEqual([r['eligible'] for r in result], [False, False])

    def test_exact_duplicate_prefers_existing_review_and_cross_split_is_blocked(self):
        old = self.row(eligible=True); new = self.row('b'); new['pixel_sha256'] = old['pixel_sha256']
        result, stats = expand([old, new], self.policy, {})
        self.assertEqual([r['eligible'] for r in result], [True, False])
        self.assertEqual(stats['new_images'], 0)
        old['eligible'] = False; old['split'] = 'heldout'
        result, _ = expand([old, new], self.policy, {})
        self.assertFalse(result[1]['eligible'])


if __name__ == '__main__':
    unittest.main()
