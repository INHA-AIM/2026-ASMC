import copy
import json
from pathlib import Path
import tempfile
import unittest

from PIL import Image

from camera_data_audit import (export_dataset, image_facts, label_metadata,
                               load_label_policy, sha, training_names)
from camera_data_expansion_v7 import (coverage, direct_cohort, direct_samples,
                                      expand, leakage_gate, similarity_pairs)


class AdoptionTests(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[2]
        self.policy = load_label_policy(root/'config/datasets/camera_object_labels_v2.json')
        self.admission = json.loads((root/'config/datasets/camera_object_adoption_v7.json').read_text())

    def row(self, ident='new', split='heldout', label='pe barrigate'):
        from camera_data_audit import classify
        source = 'roboflow:enzo-mdikr/my-first-project-eeree'
        target, disposition = classify(label, self.policy, source)
        ann = label_metadata(dict(id='a', original_label=label, original_category_id=0,
            canonical=target, disposition=disposition, xyxy=[1, 1, 3, 3], issues=[]), self.policy)
        return dict(id=ident, source=source, image_sha256=ident+'sha',
            pixel_sha256=ident+'pixels', phash='0000000000000000',
            split=split, split_group=ident+'scene', width=100, height=100,
            annotations=[ann], eligible=False, issues=['review_pending'],
            reasons=['visual_review_required'])

    def review(self, row):
        return dict(image_sha256=row['image_sha256'], correct=True,
                    all_targets_annotated=True, reviewer='RGB reviewer', evidence='sheet/panel')

    def cohort(self, rows):
        samples = direct_samples(rows)
        sampled = {i for members in samples.values() for i in members}
        return dict(source=rows[0]['source'], member_ids=[r['id'] for r in rows],
            samples=samples, sample_reviews={r['id']:self.review(r) for r in rows if r['id'] in sampled})

    def test_fixed_samples_are_order_independent_and_capped_per_original_label(self):
        rows = [self.row(str(i)) for i in range(80)]
        self.assertEqual(direct_samples(rows), direct_samples(list(reversed(rows))))
        self.assertEqual(len(next(iter(direct_samples(rows).values()))), 50)

    def test_source_admission_needs_no_teacher_and_preserves_heldout_tiny_boxes(self):
        row = self.row(); old = copy.deepcopy(row)
        result, changed, reports = expand([row], self.policy, [self.cohort([row])], {}, self.admission)
        self.assertEqual(changed, {'new'})
        self.assertEqual(result[0]['split'], 'heldout')
        self.assertEqual(result[0]['annotations'], old['annotations'])
        self.assertEqual(row, old)
        self.assertFalse(reports[0]['teacher_approval_required'])
        self.assertNotIn('annotation_complete', result[0])  # sampled != independently complete

    def test_direct_missing_sample_or_stale_image_or_incomplete_gt_rejected(self):
        row = self.row()
        for field, value in [('image_sha256','stale'), ('all_targets_annotated',False), ('evidence',None)]:
            cohort = self.cohort([row]); cohort['sample_reviews']['new'][field] = value
            with self.assertRaises(ValueError): direct_cohort([row],cohort,self.policy)
        cohort = self.cohort([row]); cohort['sample_reviews'] = {}
        with self.assertRaisesRegex(ValueError,'incomplete'): direct_cohort([row],cohort,self.policy)

    def test_pe_mixture_cannot_use_historical_direct_mapping(self):
        row = self.row()
        row['source'] = 'roboflow:tooth-decay-rnacu/morai_ai_mission-fhqkh'
        row['annotations'][0]['original_label'] = 'PE'
        with self.assertRaisesRegex(ValueError,'individual RGB'):
            direct_cohort([row],self.cohort([row]),self.policy,self.admission)

    def test_parent_admission_immutable_and_individual_cannot_overlap(self):
        row = self.row(); row['eligible'] = True; row['issues'] = []; row['reasons'] = []
        result, changed, _ = expand([row],self.policy,[self.cohort([row])],{},self.admission)
        self.assertEqual(result,[row]); self.assertFalse(changed)
        with self.assertRaisesRegex(ValueError,'overlaps'):
            expand([row],self.policy,[],{'new':self.review(row)},self.admission)

    def test_direct_negative_or_ambiguous_source_needs_individual_review(self):
        for label in ('speed','dog'):
            row=self.row(label=label)
            with self.assertRaises(ValueError): direct_cohort([row],self.cohort([row]),self.policy)

    def test_all_opposing_pairs_including_new_vs_new_are_required(self):
        rows = [self.row('a','train'), self.row('b','heldout'), self.row('c','selection')]
        for row in rows: row['eligible'] = True
        pairs = similarity_pairs(rows, {'a','b'})
        self.assertEqual({(p['new_id'],p['opposing_id']) for p in pairs},
                         {('a','b'),('a','c'),('b','a'),('b','c')})
        review = dict(new_image_sha256='asha', opposing_image_sha256='bsha',
                      reviewer='RGB', evidence='pair', decision='different_scene')
        leakage_gate(rows, {'a','b'}, pairs, {'a:b':review})
        self.assertFalse(rows[0]['eligible'])  # checking one convenient pair is insufficient

    def test_exact_pixels_and_scene_split_conflicts_cannot_be_waived(self):
        for field in ('pixel_sha256','split_group'):
            rows=[self.row('a','train'),self.row('b','heldout')]
            for row in rows: row['eligible']=True
            rows[0][field]=rows[1][field]
            leakage_gate(rows,{'a'},[],{})
            self.assertFalse(rows[0]['eligible']);self.assertTrue(rows[1]['eligible'])

    def test_similarity_review_binds_both_image_hashes(self):
        rows=[self.row('a','train'),self.row('b','heldout')]
        for row in rows: row['eligible']=True
        pairs=similarity_pairs(rows,{'a'})
        with self.assertRaisesRegex(ValueError,'SHA'):
            leakage_gate(rows,{'a'},pairs,{'a:b':dict(new_image_sha256='asha',opposing_image_sha256='stale')})

    def test_admitted_individual_preserves_box_and_corrects_teacher_name(self):
        row=self.row(label='pe barrigate')
        decision=self.review(row)
        decision['overrides']=[dict(annotation_id='a',canonical='box',teacher_raw_phrase='concrete barrier')]
        result,_,_=expand([row],self.policy,[],{'new':decision},self.admission)
        self.assertEqual(result[0]['annotations'][0]['xyxy'],row['annotations'][0]['xyxy'])
        self.assertEqual(result[0]['annotations'][0]['training_class'],'box')

    def test_collection_weights_use_image_presence_not_box_counts(self):
        row=self.row(split='train');row['eligible']=True
        # Above the minimum-50 floor, repeating boxes must not change weights.
        rows=[dict(copy.deepcopy(row), id=str(i), split_group=str(i)) for i in range(70)]
        before=coverage(rows,self.policy)
        for r in rows:r['annotations']*=30
        after=coverage(rows,self.policy)
        self.assertEqual(before['collection_proposal'],after['collection_proposal'])
        self.assertEqual(after['classes']['plastic_barrier']['train']['images'],70)
        self.assertEqual(after['collection_proposal']['pools']['obstacles']['slots'],10)
        self.assertEqual(after['collection_proposal']['pools']['rare_compact']['slots'],2)
        self.assertFalse(after['collection_proposal']['executable'])

    def test_real_yolo_export_keeps_tiny_gt_and_fixed_16_ids(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);image=root/'source.png';Image.new('RGB',(100,100),'navy').save(image)
            row=self.row(split='train');row.update(image=str(image),eligible=True,issues=[],reasons=[])
            row.update(image_facts(row));row['image_sha256']=sha(image)
            output=root/'export';export_dataset([row],self.policy,output,True)
            summary=json.loads((output/'summary.json').read_text())
            verification=json.loads((output/'verification.json').read_text())
            self.assertEqual(summary['classes'],training_names(self.policy))
            self.assertEqual(verification['verified_objects'],1)
            self.assertIn('0.0200000000',(output/'labels/train/new.txt').read_text())


if __name__ == '__main__': unittest.main()
