#!/usr/bin/env python3
"""CPU forward/backward only; never instantiate an optimizer or use CUDA."""
import copy
import json
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest

import torch
import cv2
import numpy as np
from torch import nn
from ultralytics.utils.loss import v8DetectionLoss, E2ELoss, E2EDetectLoss
from camera_pretrain_torch import classification_keep, wrap_criterion, PretrainDataset
from camera_pretrain_policy import CLASSES, load_policy, sha, guarded_data_yaml


class FakeModel(nn.Module):
    def __init__(self):
        super().__init__()
        self.weight=nn.Parameter(torch.zeros(1))
        self.args=SimpleNamespace(box=7.5,cls=.5,dfl=1.5,epochs=20)
        self.model=[SimpleNamespace(stride=torch.tensor([8.]),nc=16,reg_max=1)]


def prediction():
    return dict(feats=[torch.zeros(1,1,4,4)], boxes=torch.full((1,4,16),1.,requires_grad=True),
                scores=torch.zeros(1,16,16,requires_grad=True))


def batch(complete=True,positive=False,ignore=()):
    return dict(batch_idx=torch.tensor([0.]) if positive else torch.zeros(0),
        cls=torch.tensor([[0.]]) if positive else torch.zeros(0,1),
        bboxes=torch.tensor([[.375,.375,.5,.5]]) if positive else torch.zeros(0,4),
        supervision_complete=torch.tensor([complete]),
        ignore_bboxes=torch.tensor(ignore,dtype=torch.float32).reshape(-1,4),
        ignore_batch_idx=torch.zeros(len(ignore),dtype=torch.long))


class LossTests(unittest.TestCase):
    def test_complete_no_ignore_matches_upstream(self):
        model=FakeModel();base=v8DetectionLoss(model);aware=wrap_criterion(v8DetectionLoss(model))
        for positive in (False,True):
            a,b=prediction(),prediction();ba=batch(positive=positive)
            la,_=base(a,ba);lb,_=aware(b,ba)
            torch.testing.assert_close(la,lb,rtol=0,atol=0)
            la.sum().backward();lb.sum().backward()
            torch.testing.assert_close(a['scores'].grad,b['scores'].grad,rtol=0,atol=0)

    def test_subpixel_ignore_masks_stride_cell(self):
        p=prediction();criterion=wrap_criterion(v8DetectionLoss(FakeModel()))
        loss,_=criterion(p,batch(ignore=[[.1,.1,.2,.2]]));loss.sum().backward()
        self.assertTrue((p['scores'].grad[:,:,0]==0).all())
        self.assertTrue((p['scores'].grad[:,:,1]!=0).all())

    def test_partial_has_no_background_gradient(self):
        p=prediction();criterion=wrap_criterion(v8DetectionLoss(FakeModel()))
        loss,_=criterion(p,batch(complete=False));loss.sum().backward()
        self.assertTrue((p['scores'].grad==0).all())

    def test_positive_wins_and_box_gradient_survives(self):
        for complete in (False,True):
            p=prediction();criterion=wrap_criterion(v8DetectionLoss(FakeModel()))
            p['boxes']=torch.full((1,4,16),.85,requires_grad=True)
            loss,_=criterion(p,batch(complete=complete,positive=True,ignore=[[0,0,32,32]]))
            loss.sum().backward()
            self.assertGreater(float(p['scores'].grad.abs().sum()),0)
            self.assertGreater(float(p['boxes'].grad.abs().sum()),0)

    def test_both_e2e_heads(self):
        for factory in (E2ELoss,E2EDetectLoss):
            criterion=wrap_criterion(factory(FakeModel()))
            preds=dict(one2many=prediction(),one2one=prediction())
            loss,_=criterion(preds,batch(ignore=[[.1,.1,.2,.2]]));loss.sum().backward()
            for key in ('one2many','one2one'):
                self.assertTrue((preds[key]['scores'].grad[:,:,0]==0).all())
                self.assertTrue((preds[key]['scores'].grad[:,:,1]!=0).all())

    def test_missing_metadata_fails_closed(self):
        b=batch();del b['ignore_bboxes']
        with self.assertRaisesRegex(ValueError,'ignore-aware'):
            wrap_criterion(v8DetectionLoss(FakeModel()))(prediction(),b)

    def test_actual_yolo26_and_yoloe_detection_criteria(self):
        from ultralytics.nn.tasks import DetectionModel, YOLOEModel
        from ultralytics.cfg import get_cfg
        for factory,yaml in ((DetectionModel,'yolo26n.yaml'),(YOLOEModel,'yoloe-26n.yaml')):
            # Same head/loss classes as L; no pretrained download or optimizer.
            model=factory(yaml,nc=16,verbose=False);model.args=get_cfg()
            criterion=wrap_criterion(model.init_criterion())
            self.assertEqual(type(criterion.one2many).__name__,'IgnoreDetectionLoss')
            self.assertEqual(type(criterion.one2one).__name__,'IgnoreDetectionLoss')
            self.assertNotIn('Segment',type(model.model[-1]).__name__)


class LoaderTests(unittest.TestCase):
    def test_stock_ultralytics_loader_is_blocked(self):
        import yaml
        from ultralytics.data.utils import check_det_dataset
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);file=root/'data.yaml'
            (root/'images/train').mkdir(parents=True);(root/'images/val').mkdir()
            file.write_text(yaml.safe_dump(guarded_data_yaml(root)))
            with self.assertRaisesRegex(FileNotFoundError,'IGNORE_AWARE_TRAINER_REQUIRED'):
                check_det_dataset(str(file),autodownload=False)

    def test_real_resize_flip_collation_and_sha_guard(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);image=root/'rgb.png'
            cv2.imwrite(str(image),np.full((480,640,3),80,np.uint8))
            policy=load_policy();policy['augmentation']['fliplr']=1.
            policy['augmentation'].update(hsv_h=0,hsv_s=0,hsv_v=0)
            (root/'policy.json').write_text(json.dumps(policy))
            row=dict(id='one',split='train',image=str(image),width=640,height=480,
                image_sha256=sha(image),view='unknown',ego_mask_applied=False,supervision='partial',
                annotations=[dict(training_class='dog',supervision='positive',xyxy=[100,100,200,200]),
                             dict(training_class='car',supervision='ignore',xyxy=[300,300,301,301])])
            (root/'manifest.jsonl').write_text(json.dumps(row)+'\n')
            (root/'data.yaml').write_text('names: '+json.dumps(list(CLASSES)))
            (root/'verification.json').write_text('{"passed":true}')
            meta=dict(classes=list(CLASSES),requires_ignore_aware_trainer=True,final_version=True,
                original_data_root=tmp,artifact_shas={n:sha(root/n) for n in ('policy.json','manifest.jsonl')})
            (root/'pretrain.json').write_text(json.dumps(meta))
            with self.assertRaisesRegex(ValueError,'finalized'):
                PretrainDataset(root)
            (root/'_SUCCESS').write_text(sha(root/'pretrain.json'))
            ds=PretrainDataset(root,augment=True);sample=ds[0]
            self.assertEqual(tuple(sample['img'].shape),(3,960,960))
            torch.testing.assert_close(sample['ignore_bboxes'],torch.tensor([[508.5,570,510,571.5]]))
            torch.testing.assert_close(sample['bboxes'],torch.tensor([[.765625,.359375,.15625,.15625]]))
            self.assertFalse(sample['supervision_complete'])
            joined=ds.collate_fn([sample,sample])
            self.assertEqual(joined['ignore_batch_idx'].tolist(),[0,1])
            self.assertEqual(joined['batch_idx'].tolist(),[0,1])
            self.assertEqual(tuple(joined['img'].shape),(2,3,960,960))
            image.write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'SHA changed'):ds[0]

    def test_batch_views_do_not_share_ignore_or_partial_state(self):
        anchors=torch.tensor([[.5,.5],[1.5,.5]])
        strides=torch.full((2,1),8.)
        b=dict(supervision_complete=torch.tensor([True,False,True]),
               ignore_bboxes=torch.tensor([[0.,0.,1.,1.]]),ignore_batch_idx=torch.tensor([0]))
        foreground=torch.tensor([[False,False],[False,True],[False,False]])
        actual=classification_keep(b,anchors,strides,foreground)
        self.assertEqual(actual.tolist(),[[False,True],[False,True],[True,True]])


if __name__=='__main__':unittest.main()
