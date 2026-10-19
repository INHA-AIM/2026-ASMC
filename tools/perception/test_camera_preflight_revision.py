import unittest
from copy import deepcopy
from prepare_camera_preflight_revision import apply_reviews


class ReviewTests(unittest.TestCase):
    def fixture(self):
        ann=dict(training_class='car',canonical='car',original_label='0',
                 original_category_id=1,xyxy=[10,20,50,60],supervision='positive')
        row=dict(id='a',split='train',image_sha256='pin',annotations=[ann],supervision='partial')
        review=dict(id='a',image_sha256='pin',annotation_index=0,
                    original_annotation=deepcopy(ann),decision='relabel',target_class='bus')
        return [row],dict(vehicles=[review],backgrounds=[],complete_images=[])

    def test_relabel_preserves_source_box_and_parent(self):
        rows,reviews=self.fixture();before=deepcopy(rows)
        output,changes=apply_reviews(rows,reviews)
        self.assertEqual(rows,before)
        ann=output[0]['annotations'][0]
        self.assertEqual(ann['training_class'],'bus')
        self.assertEqual(ann['original_label'],'0')
        self.assertEqual(ann['xyxy'],before[0]['annotations'][0]['xyxy'])
        self.assertEqual(ann['preflight_previous_annotation'],before[0]['annotations'][0])
        self.assertEqual(len(changes),1)

    def test_reject_evaluation_change(self):
        rows,reviews=self.fixture();rows[0]['split']='heldout'
        with self.assertRaisesRegex(ValueError,'evaluation'):apply_reviews(rows,reviews)

    def test_reject_stale_review(self):
        rows,reviews=self.fixture();reviews['vehicles'][0]['original_annotation']['xyxy'][0]=11
        with self.assertRaisesRegex(ValueError,'annotation identity'):apply_reviews(rows,reviews)
        rows,reviews=self.fixture();reviews['vehicles'][0]['image_sha256']='wrong'
        with self.assertRaisesRegex(ValueError,'SHA'):apply_reviews(rows,reviews)

    def test_negative_requires_empty_annotations(self):
        rows,reviews=self.fixture();reviews['vehicles']=[]
        reviews['backgrounds']=[dict(id='a',image_sha256='pin',decision='reviewed_negative',all_targets_annotated=True,review_scope='camera_objects_v2_16')]
        with self.assertRaisesRegex(ValueError,'no annotations'):apply_reviews(rows,reviews)
        rows[0].update(annotations=[],spatial_background_only=True)
        output,_=apply_reviews(rows,reviews)
        self.assertEqual(output[0]['supervision'],'reviewed_negative')
        self.assertNotIn('spatial_background_only',output[0])
        self.assertFalse(output[0]['known_class_background'])

    def test_complete_requires_full16_and_keeps_ignore(self):
        rows,reviews=self.fixture();reviews['vehicles']=[]
        rows[0]['annotations'].append(dict(training_class='car',supervision='ignore',xyxy=[1,1,2,2]))
        review=dict(id='a',image_sha256='pin',decision='complete',all_targets_annotated=False,review_scope='camera_objects_v2_16')
        reviews['complete_images']=[review]
        with self.assertRaisesRegex(ValueError,'full-frame'):apply_reviews(rows,reviews)
        review['all_targets_annotated']=True
        output,_=apply_reviews(rows,reviews)
        self.assertEqual(output[0]['annotations'],rows[0]['annotations'])
        self.assertEqual(output[0]['supervision'],'complete')

    def test_defer_does_not_promote(self):
        rows,reviews=self.fixture();reviews['vehicles'][0]['decision']='defer'
        result,changes=apply_reviews(rows,reviews)
        self.assertEqual(result,rows);self.assertFalse(changes)

    def test_duplicate_retains_correct_positive_and_raw_annotation(self):
        rows,reviews=self.fixture();reviews['vehicles']=[]
        keeper=deepcopy(rows[0]['annotations'][0]);keeper['training_class']='bus'
        rows[0]['annotations'].append(keeper)
        review=dict(id='a',image_sha256='pin',annotation_index=0,
                    original_annotation=deepcopy(rows[0]['annotations'][0]),
                    decision='exclude_duplicate',duplicate_of=1,target_class='bus')
        reviews['duplicates']=[review]
        result,_=apply_reviews(rows,reviews)
        self.assertEqual(result[0]['annotations'][0]['supervision'],'excluded')
        self.assertEqual(result[0]['annotations'][0]['original_label'],'0')
        self.assertEqual(result[0]['annotations'][1],keeper)
        review['target_class']='car'
        with self.assertRaisesRegex(ValueError,'class mismatch'):apply_reviews(rows,reviews)
        review['target_class']='bus';keeper['xyxy']=[100,100,150,150]
        with self.assertRaisesRegex(ValueError,'overlap'):apply_reviews(rows,reviews)


if __name__=='__main__':unittest.main()
