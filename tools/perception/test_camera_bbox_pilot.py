import copy
import unittest
from prepare_camera_bbox_pilot import sample_train, verify_membership, evaluation_candidates


def row(i,source='roboflow:a',cls='car',split='train'):
    return dict(id=str(i),source=source,eligible=True,split=split,split_group=source,
        pixel_sha256=str(i),image_sha256=str(i),source_key=f'front/{i:06d}',
        annotations=[dict(canonical=cls,training_class=cls,disposition='candidate',xyxy=[0,0,1,1])])


class PilotTest(unittest.TestCase):
    def setUp(self):
        self.config=dict(max_train_images=20,rare_image_threshold=3,priority_classes=['box'],
            seed=20261011,source_fraction_cap=.25,capture_fraction_cap=.4)
        self.rows=[row(i,'roboflow:'+str(i%6),'box' if i<2 else 'car') for i in range(60)]

    def test_reproducible_caps_rare_and_annotation_preservation(self):
        before=copy.deepcopy(self.rows)
        selected,stats=sample_train(self.rows,self.config)
        again,_=sample_train(list(reversed(self.rows)),self.config)
        self.assertEqual([r['id'] for r in selected],[r['id'] for r in again])
        self.assertEqual(len(selected),20)
        self.assertEqual(stats['rare_images_not_selected']['box'],0)
        self.assertLessEqual(max(stats['source_images'].values()),5)
        self.assertEqual(self.rows,before)
        verify_membership(self.rows,selected)

    def test_capture_cap(self):
        rows=[row(i,'capture:'+str(i%4)) for i in range(40)]+self.rows
        selected,stats=sample_train(rows,self.config)
        self.assertLessEqual(stats['capture_images'],8)

    def test_frozen_split_and_geometry(self):
        selected=[copy.deepcopy(self.rows[0])]
        selected[0]['split']='heldout'
        with self.assertRaisesRegex(ValueError,'split'):verify_membership(self.rows,selected)
        selected[0]['split']='train';selected[0]['annotations'][0]['xyxy']=[0,0,2,2]
        with self.assertRaisesRegex(ValueError,'annotations'):verify_membership(self.rows,selected)

    def test_eval_partial_gt_is_not_precision(self):
        r=row(100,split='selection');r['annotation_complete']=False
        candidates,_=evaluation_candidates([r],self.rows)
        self.assertEqual(candidates[0]['evaluation_scope'],'recall_only')
        r['annotation_complete']=True
        r['review_scope']='camera_objects_v2_16'
        r['review_record']=dict(decision='accept',all_targets_annotated=True)
        candidates,_=evaluation_candidates([r],self.rows)
        self.assertEqual(candidates[0]['evaluation_scope'],'complete')
        r['reasons']=['cross_split_similar_image_review']
        self.assertFalse(evaluation_candidates([r],self.rows)[0])

    def test_eval_same_pixel_never_admitted(self):
        r=row(0,split='selection')
        self.assertFalse(evaluation_candidates([r],self.rows)[0])

    def test_existing_eval_is_not_its_own_leak(self):
        r=row(100,source='roboflow:eval',split='heldout')
        self.assertEqual(len(evaluation_candidates([r],[r])[0]),1)


if __name__=='__main__':unittest.main()
