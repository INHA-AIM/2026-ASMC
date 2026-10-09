import copy
import json
from pathlib import Path
import unittest
import tempfile
from camera_data_audit import load_label_policy, label_metadata, training_names
from camera_teacher_audit import analyze, prediction_label, suggest, complete_gt, analysis_provenance

ROOT=Path(__file__).resolve().parents[2]

class TeacherTests(unittest.TestCase):
    def setUp(self):
        self.policy=load_label_policy(ROOT/'config/datasets/camera_object_labels_v2.json')
        self.config=json.loads((ROOT/'config/experiments/camera_yoloe_x_teacher_v1.json').read_text())

    def fixtures(self,count=20):
        gt=[];pred=[]
        for split in ('selection','heldout'):
            for i in range(count):
                ident=split+str(i);ann=label_metadata(dict(canonical='deer',xyxy=[0,0,20,20],issues=[]),self.policy)
                gt.append(dict(id=ident,split=split,annotation_complete=True,review_scope=self.policy['review_scope'],
                    image_sha256=ident,annotations=[ann],issues=[]))
                detected=prediction_label('doe',self.policy);detected.update(confidence=.6,xyxy=ann['xyxy'])
                pred.append(dict(id=ident,image_sha256=ident,predictions=[detected]))
        return gt,pred

    def test_rich21_and_prompt_training_mapping(self):
        self.assertEqual(len(self.config['prompt_profiles']['rich21']),21)
        self.assertEqual(len(self.config['prompt_profiles']['base18']),18)
        self.assertEqual(prediction_label('wooden crate',self.policy)['training_class'],'box')
        self.assertEqual(prediction_label('traffic drum',self.policy)['training_class'],'traffic_drum')
        for names in self.config['prompt_profiles'].values():
            for name in names:prediction_label(name,self.policy)

    def test_heldout_frozen_threshold_and_low_sample(self):
        gt,pred=self.fixtures();result=analyze(gt,pred,self.policy,self.config)
        self.assertEqual(result['approved_classes'],['deer'])
        self.assertEqual(result['teacher_classes']['deer']['selection_threshold'],.6)
        gt,pred=self.fixtures(19);self.assertEqual(analyze(gt,pred,self.policy,self.config)['approved_classes'],[])

    def test_incomplete_annotation_cannot_supply_false_positive_or_approval(self):
        gt,pred=self.fixtures()
        for r in gt:r.pop('review_scope')
        result=analyze(gt,pred,self.policy,self.config)
        self.assertEqual(result['approved_classes'],[])
        self.assertEqual(result['fixed_selection']['training_class']['evaluated_frames'],0)

    def test_duplicate_detections_not_multiple_true_positives(self):
        gt,pred=self.fixtures()
        for p in pred:p['predictions']*=2
        result=analyze(gt,pred,self.policy,self.config)
        c=result['fixed_selection']['training_class']['classes']['deer']
        self.assertEqual((c['tp'],c['fp']),(20,20));self.assertEqual(result['approved_classes'],[])

    def test_dog_prediction_is_animal_localization_not_deer_truth(self):
        gt,pred=self.fixtures()
        for p in pred:p['predictions'][0].update(prediction_label('dog',self.policy))
        result=analyze(gt,pred,self.policy,self.config)
        self.assertEqual(result['fixed_selection']['consumer_group']['classes']['animal']['tp'],20)
        self.assertEqual(result['fixed_selection']['training_class']['classes']['deer']['tp'],0)
        self.assertEqual(gt[0]['annotations'][0]['canonical'],'deer')

    def test_missing_frame_or_image_hash_is_not_silently_skipped(self):
        gt,pred=self.fixtures()
        with self.assertRaisesRegex(ValueError,'membership'):analyze(gt,pred[:-1],self.policy,self.config)
        pred[0]['image_sha256']='wrong'
        with self.assertRaisesRegex(ValueError,'SHA'):analyze(gt,pred,self.policy,self.config)

    def test_reanalysis_binding_rejects_checkpoint_or_prompt_mixture(self):
        from camera_data_audit import sha
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);config=root/'config.json';config.write_text(json.dumps(self.config))
            gt,pred=self.fixtures();gtfile=root/'gt.json';gtfile.write_text(json.dumps(gt))
            predfile=root/'pred.json';predfile.write_text(json.dumps(pred))
            for r in pred:r.update(checkpoint_sha256='weights',embeddings_sha256='embedding',
                config_sha256=sha(config),profile='rich21')
            binding=analysis_provenance(pred,self.config,self.policy,config,gtfile,predfile)
            self.assertEqual(binding['checkpoint_sha256'],'weights')
            pred[0]['checkpoint_sha256']='other'
            with self.assertRaisesRegex(ValueError,'mixed prediction'):analysis_provenance(pred,self.config,self.policy,config,gtfile,predfile)
            pred[0]['checkpoint_sha256']='weights';pred[0]['predictions'][0]['source_prompt']='unbound'
            with self.assertRaisesRegex(ValueError,'outside bound'):analysis_provenance(pred,self.config,self.policy,config,gtfile,predfile)

    def test_actor_candidates_wait_for_image_confirmation(self):
        gt,pred=self.fixtures()
        gt[0]['annotations'][0]['actor_review']={'human_confirmation_pending':True}
        self.assertFalse(complete_gt(gt[0],self.policy))
        self.assertEqual(analyze(gt,pred,self.policy,self.config)['approved_classes'],[])

    def test_suggestions_do_not_edit_labels_or_split_composites(self):
        gt,pred=self.fixtures();approval=analyze(gt,pred,self.policy,self.config)
        ann=dict(id='7',original_label='6',disposition='ambiguous',canonical=None,xyxy=[0,0,20,20])
        row=dict(id='a',image_sha256='sha',source='roboflow:cone-z2vga/katri-q4vkc',annotations=[ann])
        before=copy.deepcopy(row);p=copy.deepcopy(pred[0]);p['image_sha256']='sha'
        proposed=suggest([row],{'a':p},approval,self.policy)
        self.assertEqual(len(proposed),1);self.assertTrue(proposed[0]['requires_human_confirmation'])
        self.assertEqual(row,before)
        row['annotations'][0]['original_label']='4'
        self.assertEqual(suggest([row],{'a':p},approval,self.policy),[])

    def test_actor_id_namespace_and_interval_and_independent_evidence(self):
        from prepare_capture_teacher_gt import prepare
        ann=dict(id='0',canonical=None,disposition='ambiguous',original_label='Obstacle',actor_id='15',issues=['capture_subtype_unknown'])
        row=dict(id='frame',frozen_split='selection',source='capture:run',source_key='front/000010',annotations=[ann],annotation_complete=True)
        proof=dict(source='capture:run',original_label='Obstacle',actor_id='15',first_frame=9,last_frame=11,canonical='box',evidence_image_sha256='rgb',teacher_derived=False)
        self.assertEqual(prepare([row],self.policy,[proof],{})[0]['annotations'][0]['training_class'],'box')
        self.assertFalse(prepare([row],self.policy,[proof],{})[0]['annotation_complete'])
        row['annotations'][0]['original_label']='Vehicle'
        self.assertIsNone(prepare([row],self.policy,[proof],{})[0]['annotations'][0]['canonical'])
        row['annotations'][0]['original_label']='Obstacle';proof['teacher_derived']=True
        with self.assertRaisesRegex(ValueError,'independent'):prepare([row],self.policy,[proof],{})
        proof['teacher_derived']=False
        with self.assertRaisesRegex(ValueError,'overlapping'):prepare([row],self.policy,[proof,proof],{})
