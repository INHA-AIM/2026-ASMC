import copy
from pathlib import Path
import tempfile
import unittest

from camera_data_audit import label_metadata, load_label_policy, sha
from prepare_capture_training_data import bind_reviews, expand_capture, intervals


class CaptureTrainingTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name)/'rgb.png'
        self.path.write_bytes(b'immutable source RGB evidence')
        self.policy=load_label_policy(Path(__file__).resolve().parents[2]/'config/datasets/camera_object_labels_v2.json')

    def row(self,ident='a',frame=10,category='Obstacle',canonical=None,split='train'):
        ann=dict(id='0',original_label=category,actor_id='2',canonical=canonical,
            disposition='candidate' if canonical else 'ambiguous',xyxy=[1,2,3,4],
            issues=[] if canonical else ['capture_subtype_unknown'],native_mask='immutable.npz',mask_index=0)
        label_metadata(ann,self.policy)
        return dict(id=ident,source='capture:run',source_key=f'front/{frame:06d}',
            view='front',split=split,frozen_split=split,width=1280,height=720,
            image=str(self.path),image_sha256=sha(self.path),pixel_sha256=ident,
            original_annotation=dict(bbox_file_present=True),annotations=[ann],
            eligible=False,annotation_complete=True,reasons=['capture_target_scope_unverified'])

    def review(self,rows,index=0,**changes):
        decision=dict(index=index,canonical='wooden_box',notes='Wooden crate visible in source RGB.',**changes)
        return bind_reviews(rows,intervals(rows),[decision])

    def test_actor_link_preserves_tiny_gt_mask_and_original_geometry(self):
        rows=[self.row(),self.row('b',11)];before=copy.deepcopy(rows)
        new,stats=expand_capture(rows,self.policy,self.review(rows))
        self.assertEqual(stats['new_images'],2)
        self.assertEqual(stats['new_classes'],{'box':2})
        for old,row in zip(before,new):
            self.assertEqual(row['annotations'][0]['xyxy'],old['annotations'][0]['xyxy'])
            self.assertEqual(row['annotations'][0]['native_mask'],'immutable.npz')
            self.assertFalse(row['breadth_quality']['complete_target_annotation_asserted'])
        self.assertEqual(rows,before)

    def test_run_category_and_trimmed_interval_do_not_leak_actor_type(self):
        rows=[self.row(),self.row('b',11),self.row('c',12,category='Vehicle')]
        foreign=self.row('d',10);foreign['source']='capture:another_run';rows.append(foreign)
        new,_=expand_capture(rows,self.policy,self.review(rows,index=1,first_frame=11))
        # Sorted namespaces place another_run first, then run/Obstacle.
        self.assertEqual([r['eligible'] for r in new],[False,True,False,False])

    def test_large_observation_gap_needs_an_independent_review(self):
        rows=[self.row(frame=0),self.row('b',41)]
        self.assertEqual(len(intervals(rows)),2)
        new,_=expand_capture(rows,self.policy,self.review(rows))
        self.assertEqual([r['eligible'] for r in new],[True,False])
        with self.assertRaisesRegex(ValueError,'extend'):
            self.review(rows,last_frame=41)

    def test_prior_accepted_and_evaluation_rows_remain_byte_equivalent(self):
        rows=[self.row(canonical='car',category='car'),self.row('b',canonical='car',split='heldout')]
        rows[0].update(eligible=True,admission_tier='previous_reviewed_subset')
        before=copy.deepcopy(rows);new,stats=expand_capture(rows,self.policy,[])
        self.assertEqual(new,before);self.assertEqual(stats['new_images'],0)

    def test_unresolved_annotation_missing_bbox_and_empty_gt_are_not_negatives(self):
        good=self.row(canonical='car',category='car')
        mixed=self.row('b',canonical='car',category='car');mixed['annotations'].append(self.row()['annotations'][0])
        missing=self.row('c',canonical='car',category='car');missing['original_annotation']['bbox_file_present']=False
        empty=self.row('d');empty['annotations']=[]
        new,_=expand_capture([good,mixed,missing,empty],self.policy,[])
        self.assertEqual([r['eligible'] for r in new],[True,False,False,False])

    def test_source_hash_and_identity_changes_fail_review(self):
        row=self.row();proof=self.review([row])
        proof[0]['evidence'][0]['xyxy']=[0,0,10,10]
        with self.assertRaisesRegex(ValueError,'identity'):
            expand_capture([row],self.policy,proof)
        proof=self.review([row]);self.path.write_bytes(b'changed image')
        with self.assertRaisesRegex(ValueError,'SHA'):
            expand_capture([row],self.policy,proof)

    def test_teacher_or_overlapping_proof_cannot_define_capture_ground_truth(self):
        row=self.row();proof=self.review([row]);proof[0]['teacher_derived']=True
        with self.assertRaisesRegex(ValueError,'independent'):
            expand_capture([row],self.policy,proof)
        proof=self.review([row])
        with self.assertRaisesRegex(ValueError,'overlapping'):
            expand_capture([row],self.policy,proof+proof)

    def test_duplicate_and_cross_split_pixels_do_not_expand_training(self):
        a=self.row(canonical='car',category='car');b=self.row('b',canonical='car',category='car')
        b['pixel_sha256']=a['pixel_sha256']
        new,stats=expand_capture([a,b],self.policy,[])
        self.assertEqual(stats['new_images'],1)
        b['split']=b['frozen_split']='heldout'
        new,stats=expand_capture([a,b],self.policy,[])
        self.assertEqual(stats['new_images'],0)
        self.assertEqual(new[1],b)

    def test_previously_changed_source_image_is_not_admitted_as_original_gt(self):
        row=self.row(canonical='car',category='car')
        row['reasons'].append('source_image_changed_since_gt')
        new,stats=expand_capture([row],self.policy,[])
        self.assertFalse(new[0]['eligible'])
        self.assertEqual(stats['new_images'],0)

    def test_bicycle_is_a_distinct_vehicle_annotation_beside_person(self):
        row=self.row(category='Vehicle')
        person=copy.deepcopy(row['annotations'][0]);person.update(id='1',original_label='person',
            canonical='person',disposition='candidate',actor_id='5',issues=[])
        row['annotations'].append(person)
        proof=bind_reviews([row],intervals([row]),[dict(index=0,canonical='bicycle',
            notes='Separate Vehicle bicycle and person GT visible, not a composite box.')])
        new,stats=expand_capture([row],self.policy,proof)
        self.assertEqual(stats['new_classes'],{'bicycle':1,'person':1})
        self.assertEqual(len(new[0]['annotations']),2)


if __name__=='__main__':unittest.main()
