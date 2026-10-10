import copy
import unittest
from train_camera_bbox_pilot import common_epochs,validate_resume
from evaluate_camera_bbox_pilot import summarize


class TrainingTest(unittest.TestCase):
    def test_common_budget(self):
        self.assertEqual(common_epochs([500,600]),15)
        self.assertEqual(common_epochs([100,150]),20)
        self.assertLess(common_epochs([2500,2000]),5)
        with self.assertRaises(ValueError):common_epochs([0,100])

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


if __name__=='__main__':unittest.main()
