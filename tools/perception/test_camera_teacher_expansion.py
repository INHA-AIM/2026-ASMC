import copy
import json
from pathlib import Path
import tempfile
import unittest

from camera_data_audit import (load_label_policy, label_metadata, training_names, sha,
                              image_facts, write_jsonl, verify_export)
from camera_teacher_expansion import (phrase_label, verify_prediction, evaluate,
    apply_decision, admit_batch, model_semantic, completed_predictions, reclassification_metrics,
    export, coverage, compare)


class ExpansionTests(unittest.TestCase):
    def setUp(self):
        self.policy=load_label_policy(Path(__file__).resolve().parents[2]/'config/datasets/camera_object_labels_v2.json')
        self.ann=label_metadata(dict(id='1',original_label='9',canonical=None,disposition='ambiguous',
                                    xyxy=[10,10,40,40],issues=[]),self.policy)
        self.row=dict(id='r',image_sha256='sha',width=100,height=100,annotations=[self.ann],
                      source='roboflow:test/project',split='train',split_group='scene1',eligible=False,reasons=['ambiguous_label:9'])
        self.review=dict(image_sha256='sha',reviewer='Independent RGB inspection',
            evidence='WoodBox visible, all target objects checked',all_targets_annotated=True,
            overrides=[dict(annotation_id='1',canonical='box',teacher_raw_phrase='concrete barrier')])

    def test_phrase_alias_and_unknown_raw_text(self):
        self.assertEqual(phrase_label('wooden crate',self.policy)['training_class'],'box')
        self.assertEqual(phrase_label('traffic barrel',self.policy)['training_class'],'traffic_drum')
        self.assertIsNone(phrase_label('a rectangular object',self.policy)['training_class'])
        self.assertEqual(phrase_label('a rectangular object',self.policy)['raw_phrase'],'a rectangular object')

    def test_wrong_name_can_be_individually_corrected_without_global_alias(self):
        before=copy.deepcopy(self.row)
        result=apply_decision(self.row,self.review,self.policy)
        self.assertEqual(result['annotations'][0]['training_class'],'box')
        self.assertEqual(result['annotations'][0]['xyxy'],self.ann['xyxy'])
        self.assertEqual(result['annotations'][0]['original_label'],'9')
        self.assertEqual(phrase_label('concrete barrier',self.policy)['training_class'],'concrete_barrier')
        self.assertEqual(self.row,before)

    def test_empty_prediction_never_creates_negative_or_resolves_unknown(self):
        bad=copy.deepcopy(self.review);bad['overrides']=[]
        with self.assertRaisesRegex(ValueError,'unresolved'):apply_decision(self.row,bad,self.policy)
        bad['overrides']=[dict(annotation_id='1',canonical=None)]
        with self.assertRaisesRegex(ValueError,'negative'):apply_decision(self.row,bad,self.policy)

    def test_composite_cannot_duplicate_original_or_silently_change_box(self):
        bad=copy.deepcopy(self.review);bad['overrides'][0]['xyxy']=[0,0,5,5]
        with self.assertRaisesRegex(ValueError,'original box'):apply_decision(self.row,bad,self.policy)
        bad=copy.deepcopy(self.review);bad['additions']=[dict(id='1',canonical='bicycle',xyxy=[0,0,5,5])]
        with self.assertRaisesRegex(ValueError,'ID'):apply_decision(self.row,bad,self.policy)

    def test_image_and_model_binding(self):
        pred=dict(id='r',image_sha256='sha',predictions=[],binding={'model':'a'})
        verify_prediction(self.row,pred,{'model':'a'})
        with self.assertRaisesRegex(ValueError,'binding'):verify_prediction(self.row,pred,{'model':'b'})
        pred['image_sha256']='changed'
        with self.assertRaisesRegex(ValueError,'SHA'):verify_prediction(self.row,pred)

    def fixture(self,complete=True):
        row=apply_decision(self.row,self.review,self.policy)
        row.update(annotation_complete=complete,independent_gt_review=complete)
        pred=dict(id='r',image_sha256='sha',predictions=[dict(phrase_label('concrete barrier',self.policy),
            xyxy=self.ann['xyxy'],confidence=.8)])
        return row,{'r':pred}

    def test_geometry_separate_from_name_and_partial_gt_precision(self):
        row,pred=self.fixture()
        semantic=evaluate([row],pred,self.policy,.4)
        geometry=evaluate([row],pred,self.policy,.4,'geometry')
        self.assertEqual(semantic['classes']['box']['tp'],0)
        self.assertEqual(geometry['classes']['box']['tp'],1)
        self.assertEqual(geometry['pooled_geometry']['precision'],1)
        self.assertIsNone(geometry['classes']['box']['precision'])
        row['annotation_complete']=False
        partial=evaluate([row],pred,self.policy,.4,'geometry')
        self.assertEqual(partial['classes']['box']['gt'],0)
        self.assertEqual(partial['partial_gt_localization']['box']['tp'],1)
        self.assertIsNone(partial['classes']['box']['precision'])

    def test_duplicate_prediction_is_not_multiple_tp(self):
        row,pred=self.fixture();pred['r']['predictions']*=2
        result=evaluate([row],pred,self.policy,.4,'geometry')
        self.assertEqual(result['classes']['box']['tp'],1)
        self.assertEqual(result['pooled_geometry']['precision'],.5)

    def test_tiny_gt_training_kept_evaluation_only_scope(self):
        row,pred=self.fixture();row['annotations'][0]['xyxy']=[1,1,3,3]
        pred['r']['predictions'][0]['xyxy']=[1,1,3,3]
        before=copy.deepcopy(row)
        row['evaluation_scope']=dict(min_long_edge=16,min_short_edge=8,ignore_prediction_ioa=.5)
        result=evaluate([row],pred,self.policy,.4,'geometry')
        self.assertEqual(result['classes']['box']['gt'],0)
        self.assertEqual(row['annotations'],before['annotations'])

    def test_batch_requires_actual_validation_and_bound_samples(self):
        batch=dict(training_class='box',sample_reviews={'r':dict(image_sha256='sha',correct=True,all_targets_annotated=True,evidence='RGB')},reviewer='review',cohort_binding={'source':'project'})
        with self.assertRaisesRegex(ValueError,'approved'):admit_batch([self.row],batch,self.policy,{})
        result=admit_batch([self.row],batch,self.policy,{'box':dict(approved=True,threshold=.6)})
        self.assertTrue(result['approved'])
        batch['sample_reviews']['r']['all_targets_annotated']=False
        with self.assertRaisesRegex(ValueError,'95'):admit_batch([self.row],batch,self.policy,{'box':dict(approved=True,threshold=.6)})

    def test_asset_priors_are_explicit_and_standing_scooter_unresolved(self):
        self.assertEqual(model_semantic('PE_Firewall_Orange'),'pe_firewall')
        self.assertEqual(model_semantic('Sportbike_01'),'motorcycle')
        self.assertEqual(model_semantic('Scooter_02'),'motorcycle')
        self.assertIsNone(model_semantic('ElectricScooter1'))
        self.assertIsNone(model_semantic('Ped_Pet1'))

    def test_resume_preserves_interrupted_append_and_rejects_internal_corruption(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'predictions.jsonl';p.write_bytes(b'{"id":"a"}\n{"id":')
            self.assertEqual(set(completed_predictions(p)),{'a'})
            self.assertEqual(p.read_bytes(),b'{"id":"a"}\n')
            self.assertEqual(len(list(Path(d).glob('predictions-interrupted-*'))),1)
            p.write_text('{"id":"a"}\nbroken\n{"id":"b"}\n')
            with self.assertRaisesRegex(ValueError,'before final'):completed_predictions(p)

    def test_resume_rejects_duplicate_completed_records(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'p.jsonl';p.write_text('{"id":"a"}\n{"id":"a"}\n')
            with self.assertRaisesRegex(ValueError,'duplicate'):completed_predictions(p)

    def test_semantic_review_cannot_clear_geometry_failure(self):
        self.row['annotations'][0]['issues']=['invalid_bbox','capture_subtype_unknown']
        with self.assertRaisesRegex(ValueError,'unresolved'):apply_decision(self.row,self.review,self.policy)

    def test_unapproved_teacher_name_not_admitted_by_geometric_match(self):
        row,pred=self.fixture();row['verified_annotation_ids']=['1']
        result=reclassification_metrics([row],pred,self.policy,.4)
        self.assertEqual(result['box']['tp'],0)
        self.assertEqual(result['concrete_barrier']['predictions'],1)
        self.assertEqual(result['concrete_barrier']['precision'],0)

    def test_proposal_bounds_and_nonfinite_confidence(self):
        row,pred=self.fixture();p=pred['r']
        p['predictions'][0]['confidence']=float('nan')
        with self.assertRaisesRegex(ValueError,'geometry/confidence'):verify_prediction(row,p)
        p['predictions'][0]['confidence']=.8;p['predictions'][0]['xyxy']=[-1,0,2,3]
        with self.assertRaisesRegex(ValueError,'geometry/confidence'):verify_prediction(row,p)

    def test_scope_also_applies_to_existing_box_name_approval(self):
        row,pred=self.fixture();row['verified_annotation_ids']=['1']
        row['annotations'][0]['xyxy']=[1,1,3,3]
        row['evaluation_scope']=dict(min_long_edge=16,min_short_edge=8,ignore_prediction_ioa=.5)
        self.assertEqual(reclassification_metrics([row],pred,self.policy,.4)['box']['gt'],0)

    def test_source_decode_failure_cannot_be_waived_by_semantic_review(self):
        self.row['issues']=['image_dimensions_disagree']
        with self.assertRaisesRegex(ValueError,'structural'):apply_decision(self.row,self.review,self.policy)

    def test_review_state_history_can_be_superseded_without_waiving_geometry(self):
        self.row['issues']=['review_pending','review_rejected']
        result=apply_decision(self.row,self.review,self.policy)
        self.assertEqual(result['issues'],[])
        self.assertEqual(result['prior_row_issues_before_individual_review'],self.row['issues'])
        self.assertEqual(self.row['issues'],['review_pending','review_rejected'])

    def test_fixed_structure_needs_explicit_negative_review(self):
        row=dict(self.row,annotations=[])
        decision=dict(self.review,overrides=[])
        with self.assertRaisesRegex(ValueError,'negative'):apply_decision(row,decision,self.policy)
        decision.update(negative_verified=True,evidence='User confirmed fixed structure; full RGB checked for all target objects')
        self.assertEqual(apply_decision(row,decision,self.policy)['annotations'],[])
        decision['additions']=[dict(id='manual',canonical='plastic_barrier',xyxy=[10,10,40,40],annotation_origin='independently_reviewed_manual_bbox')]
        self.assertEqual(apply_decision(row,decision,self.policy)['annotations'][0]['annotation_origin'],'independently_reviewed_manual_bbox')
        decision['additions'][0]['annotation_origin']='automatic_unapproved_teacher'
        with self.assertRaisesRegex(ValueError,'origin'):apply_decision(row,decision,self.policy)

    def test_actual_export_keeps_class_ids_small_gt_parent_and_frozen_split(self):
        from PIL import Image
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);old=[]
            for index,split in enumerate(('train','selection','heldout')):
                path=root/('%s.png'%split)
                Image.new('RGB',(100,100),(index*70,10,20)).save(path)
                row=copy.deepcopy(self.row)
                row.update(id=split,split=split,split_group=split,image=str(path),issues=[])
                row=image_facts(row)
                review=dict(self.review,image_sha256=row['image_sha256'])
                row=apply_decision(row,review,self.policy)
                row['annotations'][0]['xyxy']=[1,1,3,3]
                old.append(row)
            import numpy as np
            path=root/'new.png';Image.fromarray(np.random.RandomState(23).randint(0,256,(100,100,3),dtype=np.uint8)).save(path)
            new=copy.deepcopy(self.row);new.update(id='new',image=str(path),issues=[])
            new=image_facts(new);review=dict(self.review,image_sha256=new['image_sha256'])
            curated=root/'curated.jsonl';write_jsonl(curated,old+[new])
            reviews=root/'reviews.json';reviews.write_text(json.dumps({'new':review}))
            args=SimpleNamespace(labels=Path(__file__).resolve().parents[2]/'config/datasets/camera_object_labels_v2.json',
                curated=curated,reviews=reviews,additional=None,batches=None,approval=None,predictions=None,
                output=root/'v6',dataset=root/'export')
            export(args)
            result=[json.loads(x) for x in (args.output/'curated.jsonl').read_text().splitlines()]
            self.assertEqual(result[:3],old)
            self.assertEqual(verify_export(args.dataset)['verified_objects'],4)
            parent=root/'parent.jsonl';write_jsonl(parent,old)
            report=root/'coverage.json'
            coverage(SimpleNamespace(curated=args.output/'curated.jsonl',labels=args.labels,parent=parent,output=report))
            covered=json.loads(report.read_text())
            self.assertTrue(covered['parent_accepted_rows_unchanged'])
            self.assertEqual(covered['new_images'],1)
            self.assertEqual(covered['classes']['stroller']['support_status'],'no_data')
            self.assertEqual(covered['classes']['box']['objects'],4)
            import yaml
            self.assertEqual(yaml.safe_load((args.dataset/'data.yaml').read_text())['names'],training_names(self.policy))
            label=args.dataset/'labels/train/train.txt'
            self.assertIn('0.0200000000',label.read_text())
            label.write_text('0 0.02 0.02 0.02 0.02\n')
            with self.assertRaisesRegex(ValueError,'class_order'):verify_export(args.dataset)

    def test_export_refuses_frozen_evaluation_relabel(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);row=dict(self.row,split='selection')
            source=root/'source.jsonl';write_jsonl(source,[row])
            reviews=root/'reviews.json';reviews.write_text(json.dumps({'r':self.review}))
            args=SimpleNamespace(labels=Path(__file__).resolve().parents[2]/'config/datasets/camera_object_labels_v2.json',
                curated=source,reviews=reviews,additional=None,batches=None,approval=None,predictions=None,
                output=root/'v6',dataset=root/'export')
            with self.assertRaisesRegex(ValueError,'frozen'):export(args)

    def test_recording_run_count_alone_cannot_approve_automatic_teacher(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);run=root/'run';run.mkdir();binding={'model':'test'}
            (run/'binding.json').write_text(json.dumps(binding))
            rows=[];predictions=[]
            for role in ('selection','heldout'):
                for index in range(3):
                    row,_=self.fixture();row.update(id=role+str(index),split=role,
                        split_group=role+str(index),independent_scene=role+str(index),verified_annotation_ids=['1'])
                    rows.append(row)
                    predictions.append(dict(id=row['id'],image_sha256='sha',binding=binding,
                        predictions=[dict(phrase_label('wooden box',self.policy),confidence=.8,xyxy=self.ann['xyxy'])]))
            manifest=root/'frames.jsonl';write_jsonl(manifest,rows)
            write_jsonl(run/'predictions.jsonl',predictions)
            config=root/'config.json';config.write_text(json.dumps(dict(confidence_sweep=[.4],fixed_confidence=.4,
                minimum_precision=.95,minimum_gt=2,minimum_predictions=2,minimum_scenes=3)))
            args=SimpleNamespace(manifest=manifest,labels=Path(__file__).resolve().parents[2]/'config/datasets/camera_object_labels_v2.json',
                config=config,runs=[run],output=root/'report.json')
            compare(args)
            state=json.loads(args.output.read_text())['models']['test']['teacher_classes']['box']
            self.assertTrue(state['statistical_thresholds_passed'])
            self.assertFalse(state['approved'])
            self.assertEqual(state['reason'],'scene_independence_unverified')
            for row in rows:row['independent_scene_verified']=True
            write_jsonl(manifest,rows);compare(args)
            self.assertTrue(json.loads(args.output.read_text())['models']['test']['teacher_classes']['box']['approved'])
            write_jsonl(run/'predictions.jsonl',predictions+[predictions[0]])
            with self.assertRaisesRegex(ValueError,'duplicate comparison'):compare(args)


if __name__=='__main__':unittest.main()
