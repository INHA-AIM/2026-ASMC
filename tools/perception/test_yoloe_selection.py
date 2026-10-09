import hashlib
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).parent))
from prepare_yoloe_l import export_profile
from write_yoloe_selection_configs import configuration
from evaluate_coco7_models import evaluate
from check_yoloe_selection_engine import compare
from analyze_yoloe_selection import best_threshold,EVALUATION_GROUPS
from asmc_camera.detection.batching import shapes_for_front,pack_image,ImageTransform
from asmc_camera.detection.batched_backend import BatchedSegmentationDetector
from asmc_camera.detection.backend import Detection


class YoloeSelectionTest(unittest.TestCase):
    def test_selection_tie_and_evaluation_only_groups(self):
        thresholds=[dict(confidence=.2,macro_f1=.5),dict(confidence=.4,macro_f1=.5),dict(confidence=.7,macro_f1=None)]
        self.assertEqual(.4,best_threshold(thresholds)['confidence'])
        self.assertIsNone(best_threshold([]))
        self.assertEqual('animal',EVALUATION_GROUPS['deer'])
        self.assertEqual('vehicle',EVALUATION_GROUPS['scooter'])
        self.assertEqual(17,len(EVALUATION_GROUPS))

    def test_engine_correspondence_preserves_labels_and_one_to_one_matching(self):
        ref=[dict(group='car',xyxy=[0,0,10,10])]
        current=[dict(group='truck',xyxy=[0,0,10,10])]
        self.assertEqual(0,compare(ref,current)[1]['matched'])
        current=[dict(group='car',xyxy=[0,0,10,10])]*2
        self.assertEqual(1,compare(ref,current)[1]['engine_unmatched'])

    def test_export_profile_covers_front_sides_and_common_batch_without_upscaling(self):
        for width in (640,960,1280):
            front,minimum,optimum,maximum=export_profile(width)
            shapes=shapes_for_front(width)
            for shape in [(1,3,*front),(2,3,480,640),(3,3,max(front[0],480),width)]:
                self.assertTrue(all(lo<=x<=hi for lo,x,hi in zip(minimum,shape,maximum)))
            image=np.zeros((720,1280,3),np.uint8)
            _,t=pack_image(image,'front',shapes=shapes)
            d=Detection(t.left,t.top,t.left+t.resized_hw[1],t.top+t.resized_hw[0],'car',.8)
            restored=t.box(d)
            self.assertEqual([0.,0.,1280.,720.],[restored.xmin,restored.ymin,restored.xmax,restored.ymax])
            bitmap=np.zeros(t.tensor_hw,bool);bitmap[t.top:t.top+t.resized_hw[0],t.left:t.left+t.resized_hw[1]]=True
            self.assertTrue(t.mask(bitmap).all())
        self.assertEqual(384,export_profile(640)[0][0])
        with self.assertRaises(ValueError):export_profile(800)

    def test_actual_yoloe_batch_path_honors_width_and_isolates_absent_views(self):
        class Model:
            def __init__(self):self.calls=[]
            def predict(self,images,**kwargs):
                self.calls.append([im.shape for im in images])
                return [SimpleNamespace(names={0:'person'},boxes=[],masks=None) for im in images]
        node=object.__new__(BatchedSegmentationDetector)
        node.device='cuda:0';node.conf=.4;node.names_list=['person']
        node.torch=SimpleNamespace(cuda=SimpleNamespace(synchronize=lambda _:None))
        images=dict(front=np.zeros((720,1280,3),np.uint8),left=np.zeros((480,640,3),np.uint8),right=np.zeros((480,640,3),np.uint8))
        for width in (640,960,1280):
            node.shapes=shapes_for_front(width)
            for mode in ('2plus1','batch3'):
                node.mode=mode;node.model=Model()
                with patch.dict(sys.modules,{'ultralytics.utils.ops':SimpleNamespace(scale_masks=lambda *a:None)}):
                    output=node.infer_images(images,['front','left','right'])
                self.assertEqual(set(images),set(output))
                self.assertEqual(3,sum(len(c) for c in node.model.calls))
                if mode=='2plus1':self.assertEqual((node.shapes['front'][0],width,3),node.model.calls[0][0])
                else:self.assertEqual([(max(node.shapes['front'][0],480),width,3)]*3,node.model.calls[0])
                node.model.calls=[]
                with patch.dict(sys.modules,{'ultralytics.utils.ops':SimpleNamespace(scale_masks=lambda *a:None)}):
                    self.assertEqual({'front'},set(node.infer_images(images,['front'])))
                self.assertEqual(1,sum(len(c) for c in node.model.calls))

    def test_checkpoint_prompt_and_exported_vocabulary_must_agree(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);sub=root/'m';sub.mkdir();checkpoint=sub/'yoloe-26m-seg.pt';checkpoint.write_bytes(b'm-checkpoint')
            digest=hashlib.sha256(checkpoint.read_bytes()).hexdigest()
            prompt=sub/'obstacle17.npz'
            np.savez(prompt,names=['car'],embeddings=np.zeros((1,1,8)),weights_sha256=np.array(digest))
            engine=sub/'obstacle17-640.engine'
            def header(names):
                data=json.dumps(dict(names={0:names},end2end=False,task='segment')).encode()
                engine.write_bytes(struct.pack('<i',len(data))+data+b'fake-plan')
            header('car')
            cfg=configuration(root,'m',640,['car'],'/models')
            self.assertEqual('source',cfg['label_mode'])
            self.assertFalse(cfg['batched_runtime'])  # Override the preceding COCO launch's private parameter.
            header('truck')
            with self.assertRaises(ValueError):configuration(root,'m',640,['car'],'/models')
            header('car');checkpoint.write_bytes(b'l-checkpoint')
            with self.assertRaises(ValueError):configuration(root,'m',640,['car'],'/models')

    def test_unannotated_new_prompt_is_not_a_false_positive_in_coco7_metric(self):
        gt=[dict(source_class_name='car',subtype_verified=True,actor_id=1,xyxy=[0,0,10,10])]
        predicted=[dict(source_class_name='car',confidence=.8,xyxy=[0,0,10,10]),dict(source_class_name='wooden box',confidence=.9,xyxy=[20,20,30,30])]
        counts,_,_,_,_=evaluate(gt,predicted,.4)
        self.assertEqual(1,counts['car']['tp'])
        self.assertEqual(0,sum(c['fp'] for c in counts.values()))


if __name__=='__main__':unittest.main()
