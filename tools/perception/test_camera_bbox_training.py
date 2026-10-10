import copy
import json
from pathlib import Path
import tempfile
import unittest
from train_camera_bbox_pilot import common_epochs,validate_resume,sha,accumulation_threshold
from evaluate_camera_bbox_pilot import summarize,scoped_complete,evaluation_weights
from report_camera_bbox_pilot import compare
from camera_bbox_temporal import prepare,analyze


class TrainingTest(unittest.TestCase):
    def test_common_budget(self):
        self.assertEqual(common_epochs([500,600]),15)
        self.assertEqual(common_epochs([100,150]),20)
        self.assertLess(common_epochs([2500,2000]),5)
        with self.assertRaises(ValueError):common_epochs([0,100])

    def test_resume_first_optimizer_update_keeps_effective_batch(self):
        last=-1;updates=[]
        for index in range(8):
            ni=2000+index
            threshold=accumulation_threshold(16,4,1,2000,index)
            if ni-last>=threshold:updates.append(index);last=ni
        self.assertEqual(updates,[3,7])
        self.assertEqual(accumulation_threshold(16,4,0,2000),4)

    def test_coco_baseline_does_not_evaluate_random_resized_head(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);dataset=root/'data/dataset_versions/pilot';dataset.mkdir(parents=True)
            paired=root/'paired';run=paired/'yolo26l';run.mkdir(parents=True)
            initial=run/'initial.pt';initial.write_bytes(b'random sixteen class head')
            pretrained=root/'data/models/coco.pt';pretrained.parent.mkdir();pretrained.write_bytes(b'COCO head')
            (paired/'comparison.json').write_text(json.dumps(dict(dataset=str(dataset))))
            record=dict(model_key='yolo26l',initial_weights_sha256=sha(pretrained),
                before_training_checkpoint=dict(sha256=sha(initial)))
            (run/'run.json').write_text(json.dumps(record))
            cfg=dict(models=dict(yolo26l='models/coco.pt'))
            actual,kind=evaluation_weights(initial,cfg)
            self.assertEqual(actual,pretrained);self.assertIn('COCO80',kind)
            self.assertEqual(evaluation_weights(initial,cfg,'initialization')[0],initial)
            record['model_key']='yoloe26l';(run/'run.json').write_text(json.dumps(record))
            self.assertEqual(evaluation_weights(initial,cfg)[0],initial)
            record['model_key']='yolo26l';(run/'run.json').write_text(json.dumps(record))
            pretrained.write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'changed'):evaluation_weights(initial,cfg)

    def test_resume_bindings_and_smoke_rejected(self):
        r=dict(model_key='yolo26l',manifest_sha256='a',config_sha256='b',initial_weights_sha256='c',batch=4,epochs=10)
        validate_resume(r,dict(r))
        changed=dict(r,batch=2)
        with self.assertRaises(ValueError):validate_resume(r,changed)
        with self.assertRaisesRegex(ValueError,'smoke'):validate_resume(dict(r,stage='smoke'),r)

    def test_partial_gt_no_false_fp_and_duplicates_one_tp(self):
        row=dict(id='a',image_sha256='sha',split_group='run',evaluation_scope='recall_only',
            annotations=[dict(canonical='box',xyxy=[0,0,30,30])])
        box=dict(class_name='box',confidence=.6,xyxy=[0,0,30,30])
        pred=dict(id='a',image_sha256='sha',predictions=[box,box])
        c=summarize([row],[pred],.4)['classes']['box']
        self.assertEqual(c['tp'],1);self.assertIsNone(c['precision']);self.assertIsNone(c['f1'])
        row['evaluation_scope']='complete';c=summarize([row],[pred],.4)['classes']['box']
        self.assertEqual(c['complete_fp'],1)

    def test_coarse_semantics_do_not_excuse_geometry(self):
        row=dict(id='a',image_sha256='sha',split_group='run',evaluation_scope='complete',
            annotations=[dict(canonical='deer',xyxy=[0,0,30,30])])
        pred=dict(id='a',image_sha256='sha',predictions=[dict(class_name='dog',confidence=.6,xyxy=[0,0,30,30])])
        self.assertEqual(summarize([row],[pred],.4,'accepted')['classes']['animal']['tp'],1)
        pred['predictions'][0]['xyxy']=[100,100,130,130]
        self.assertEqual(summarize([row],[pred],.4,'accepted')['classes']['animal'].get('tp',0),0)

    def test_tiny_gt_ignored_only_for_eval_and_sha_enforced(self):
        row=dict(id='a',image_sha256='sha',split_group='run',evaluation_scope='complete',
            annotations=[dict(canonical='person',xyxy=[0,0,2,2])])
        pred=dict(id='a',image_sha256='sha',predictions=[])
        original=copy.deepcopy(row)
        self.assertEqual(summarize([row],[pred],.4)['classes']['person']['gt'],1)
        self.assertEqual(summarize([row],[pred],.4,pixel_scope=True)['classes']['person'].get('gt',0),0)
        self.assertEqual(row,original)
        pred['image_sha256']='wrong'
        with self.assertRaises(ValueError):summarize([row],[pred],.4)

    def test_candidate_recall_gate_and_missing_animal_gt(self):
        def evaluation(tp):
            return dict(selection_confidence=.4,control=dict(heldout=dict(
                **{'class':dict(classes={'box':dict(complete_gt=25,f1=.8,complete_fp=2)})},
                accepted=dict(classes={'person':dict(gt=100,tp=tp)}))))
        evaluations={'a-before-960':evaluation(90),'a-after-960':evaluation(87)}
        r=compare(evaluations)
        self.assertEqual(r['status'],'selection_deferred')
        self.assertIn('person',r['comparisons'][0]['old_regression_failures'])
        evaluations['a-after-960']=evaluation(91)
        r=compare(evaluations)
        self.assertIsNotNone(r['provisional_candidate'])
        self.assertIn('animal',r['provisional_candidate']['old_regression_unresolved'])
        evaluations['a-after-960']['selected_heldout_accepted']=dict(classes={'person':dict(gt=100,tp=80)})
        r=compare(evaluations)
        self.assertEqual(r['status'],'selection_deferred')
        self.assertIn('person',r['comparisons'][0]['selected_old_regression_failures'])

    def test_native_capture_only_tiny_unknown_is_still_partial(self):
        anns=[dict(canonical='car',xyxy=[0,0,60,30],disposition='candidate',issues=[]),
              dict(canonical=None,xyxy=[100,100,102,102],disposition='ambiguous',issues=['capture_subtype_unknown'])]
        row=dict(source='capture:run',annotations=anns,provenance_verified=True,
            annotation_complete=True,issues=[],original_annotation=dict(bbox_file_present=True,annotations=anns))
        self.assertFalse(scoped_complete(row,True));self.assertFalse(scoped_complete(row,False))
        anns[1]['xyxy']=[100,100,120,120]
        self.assertFalse(scoped_complete(row,True))
        anns[1]['xyxy']=[100,100,102,102];row['source']='roboflow:source'
        self.assertFalse(scoped_complete(row,True))

    def test_excluded_infrastructure_is_not_an_ignore_region(self):
        row=dict(id='a',image_sha256='sha',split_group='run',evaluation_scope='complete',annotations=[
            dict(canonical='person',xyxy=[0,0,20,20]),
            dict(canonical=None,disposition='excluded',xyxy=[30,30,60,60])])
        pred=dict(id='a',image_sha256='sha',predictions=[dict(class_name='box',confidence=.6,xyxy=[30,30,60,60])])
        self.assertEqual(summarize([row],[pred],.4)['classes']['box']['complete_fp'],1)

    def test_temporal_requires_consecutive_inputs_and_one_to_one(self):
        rows=[];pred=[]
        for i in range(6):
            rows.append(dict(id=str(i),image_sha256=str(i),source='capture:run',view='front',
                source_key=f'front/{i:06d}',split='heldout',reasons=[],issues=[],
                annotations=[dict(canonical='car',actor_id='7',original_label='car',xyxy=[0,0,60,30])]))
            boxes=[dict(class_name='car',confidence=.8,xyxy=[0,0,60,30])]*2 if i in [0,2,4] else []
            pred.append(dict(id=str(i),image_sha256=str(i),predictions=boxes))
        frozen=prepare(rows);r=analyze(frozen,pred,.4)['segments'][0]
        self.assertEqual(r['first_stable_window_end_frame'],4)
        self.assertEqual(r['matched_inputs'],3)
        self.assertEqual(r['largest_miss_streak_inputs'],1)
        self.assertFalse(prepare(rows[:2]+rows[3:]))


if __name__=='__main__':unittest.main()
