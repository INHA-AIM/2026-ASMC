import copy
import json
from pathlib import Path
import tempfile
import unittest
from PIL import Image

from camera_data_audit import (load_label_policy, curate, export_dataset, verify_export,
    bucket_review_states, review_samples, training_names)
from camera_data_readiness import bbox_rows, preserve_splits, readiness_report
from test_camera_data_audit import row, accept

ROOT=Path(__file__).resolve().parents[2]


class ReadinessTests(unittest.TestCase):
    def setUp(self):
        self.policy=load_label_policy(ROOT/'config/datasets/camera_object_labels_v2.json')

    def test_v4_class_ids_are_the_agreed_immutable_order(self):
        names=['person','dog','deer','bicycle','car','motorcycle','bus','truck','bollard','box',
            'concrete_barrier','steel_barricade','traffic_drum','plastic_barrier','traffic_cone','stroller']
        experiment=json.loads((ROOT/'config/experiments/camera_training_data_v4.json').read_text())
        self.assertEqual(training_names(self.policy),names)
        self.assertEqual(experiment['class_order'],names)

    def test_segmentation_diagnostics_do_not_waive_bbox_or_visibility(self):
        r=row();r['issues']=['unverified_semantic_palette_run','missing_capture_bbox_source','no_visible_instance_pixels']
        r['annotations'][0]['issues']=['invalid_bbox','invalid_polygon']
        before=copy.deepcopy(r);out=bbox_rows([r])[0]
        self.assertEqual(r,before)
        self.assertEqual(out['issues'],['missing_capture_bbox_source','no_visible_instance_pixels'])
        self.assertEqual(out['annotations'][0]['issues'],['invalid_bbox'])
        self.assertIn('invalid_polygon',out['annotations'][0]['auxiliary_segmentation_issues'])

    def test_split_registry_freezes_images_and_new_members_of_old_scene(self):
        a=row(ident='a');old=copy.deepcopy(a);old['split']='heldout'
        b=row(ident='b');new=preserve_splits([a,b],[old])
        self.assertEqual([r['frozen_split'] for r in new],['heldout','heldout'])
        self.assertIsNone(a['frozen_split'])
        old['image_sha256']='changed'
        with self.assertRaisesRegex(ValueError,'SHA'):preserve_splits([a],[old])

    def test_capture_independent_geometry_review_does_not_require_instance_link(self):
        from camera_data_audit import apply_review
        a=row(source='capture:r',split='train');a['issues']=['no_visible_instance_pixels']
        d=accept(a,review_scope=self.policy['review_scope']);d['bbox_geometry_verified']=True
        out,_=apply_review(a,d,self.policy)
        self.assertIn('no_visible_instance_pixels',out['issues'])
        d['human_confirmed']=True
        out,_=apply_review(a,d,self.policy)
        self.assertNotIn('no_visible_instance_pixels',out['issues'])
        self.assertIn('no_visible_instance_pixels',out['auxiliary_segmentation_issues'])
        a['issues'].append('missing_capture_bbox_source')
        out,_=apply_review(a,d,self.policy)
        self.assertIn('missing_capture_bbox_source',out['issues'])
        self.assertIn('no_visible_instance_pixels',out['issues'])

    def test_capture_split_and_scene_identity_cannot_change(self):
        a=row(source='capture:r',split='train');old=copy.deepcopy(a);old['split']='heldout'
        with self.assertRaisesRegex(ValueError,'frozen split'):preserve_splits([a],[old])
        old['split']='train';old['split_group']='different'
        with self.assertRaisesRegex(ValueError,'scene group'):preserve_splits([a],[old])

    def test_bucket_approval_is_hash_bound_and_individual_repair_survives_failed_bucket(self):
        a=row(ident='a');b=row(ident='b');p=self.policy
        decisions={a['id']:accept(a,review_scope=p['review_scope']),b['id']:dict(image_sha256=b['image_sha256'],decision='reject')}
        rows,samples=curate([a,b],p,decisions)
        self.assertTrue(rows[0]['eligible']);self.assertFalse(rows[1]['eligible'])
        state=next(iter(bucket_review_states([a,b],p,decisions,samples).values()))
        self.assertFalse(state['approved']);self.assertEqual(state['state'],'failed_sample')
        decisions['a']['image_sha256']='stale'
        state=next(iter(bucket_review_states([a,b],p,decisions,samples).values()))
        self.assertEqual(state['verified_ids'],[])

    def test_pending_and_mixed_labels_have_distinct_progress(self):
        a=row(ident='a');b=row(name='animal',ident='b')
        rows,samples=curate([a,b],self.policy)
        report=readiness_report(rows,self.policy,{},samples)
        self.assertEqual({r['meaning'] for r in report['buckets']},{'candidate','ambiguous'})
        self.assertFalse(report['target_split_coverage'])
        self.assertFalse(report['teacher_approval_required_for_direct_mapping'])

    def test_pending_review_does_not_count_as_completed_sample(self):
        from report_camera_data_audit import review_coverage
        rows=[dict(id='a',review_record={'decision':'pending'}),dict(id='b',review_record={'decision':'reject'})]
        result=review_coverage(rows,{'one':['a','b']})
        self.assertEqual(result['reviewed_images'],1)
        self.assertEqual(result['pending_images'],1)
        self.assertEqual(result['fully_reviewed_sample_buckets'],0)
        self.assertFalse(result['full_visual_review_complete'])

    def test_noop_legacy_override_can_verify_bucket_but_effective_repair_cannot(self):
        a=row(ident='a');decision=accept(a,review_scope=self.policy['review_scope'])
        decision['overrides']=[dict(id=a['annotations'][0]['id'],canonical='car')]
        samples=review_samples([a],self.policy)
        state=lambda d:next(iter(bucket_review_states([a],self.policy,{'a':d},samples).values()))
        self.assertTrue(state(decision)['approved'])
        decision['overrides'][0]['canonical']='truck'
        self.assertFalse(state(decision)['approved'])
        repaired,_=curate([a],self.policy,{'a':decision})
        report=readiness_report(repaired,self.policy,{'a':decision},samples)
        self.assertFalse(report['buckets'][0]['sample']['approved'])
        self.assertEqual(report['classes']['truck'][repaired[0]['split']]['objects'],1)
        decision['decision']='pending'
        rows,_=curate([a],self.policy,{'a':decision})
        self.assertIn('review_pending',rows[0]['issues'])
        self.assertNotIn('review_rejected',rows[0]['issues'])

    def test_fixed_ids_keep_absent_classes_and_tiny_positives(self):
        import yaml
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);im=root/'a.png';Image.new('RGB',(100,80)).save(im)
            r=row(name='box');r.update(image=str(im),eligible=True,split='train')
            from camera_data_audit import sha
            r['image_sha256']=sha(im);r['annotations'][0]['xyxy']=[1,1,2,2]
            r['annotations'][0].update(canonical='box',disposition='candidate')
            export_dataset([r],self.policy,root/'fixed',fixed_class_order=True)
            summary=json.loads((root/'fixed/summary.json').read_text())
            self.assertEqual(len(summary['classes']),16)
            self.assertEqual(summary['present_classes'],['box'])
            self.assertFalse(summary['full_target_coverage'])
            self.assertEqual((root/'fixed/labels/train/a.txt').read_text().split()[0],'9')
            self.assertEqual(verify_export(root/'fixed')['verified_objects'],1)
            cfg=yaml.safe_load((root/'fixed/data.yaml').read_text());cfg['names'][0],cfg['names'][1]=cfg['names'][1],cfg['names'][0]
            (root/'fixed/data.yaml').write_text(yaml.safe_dump(cfg))
            with self.assertRaisesRegex(ValueError,'fixed_class_order'):verify_export(root/'fixed')
            export_dataset([r],self.policy,root/'legacy')
            self.assertEqual(verify_export(root/'legacy')['verified_objects'],1)
            self.assertEqual(json.loads((root/'legacy/summary.json').read_text())['classes'],['box'])


if __name__=='__main__':unittest.main()
