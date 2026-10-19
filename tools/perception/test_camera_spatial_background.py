#!/usr/bin/env python3
"""Spatial negative supervision: real CPU gradients, no optimizer updates."""
import unittest
import json
import tempfile
from pathlib import Path
import cv2
import numpy as np
import torch
from ultralytics.utils.loss import E2ELoss
from camera_pretrain_torch import classification_keep, known_class_keep, wrap_criterion, PretrainDataset
from camera_pretrain_policy import CLASSES, load_policy, sha
from prepare_camera_spatial_background import semantic_grid
from test_camera_pretrain_torch import FakeModel, prediction, batch


class SpatialTests(unittest.TestCase):
    def test_unmapped_tiny_pixel_blocks_full_cell_on_shrink(self):
        sem=np.full((64,64,3),[127,127,127],np.uint8);sem[1,1]=[236,255,2]
        g=semantic_grid(sem,[(127,127,127)],target=32)
        self.assertFalse(g[0,0]);self.assertTrue(g[-1,-1])

    def test_unknown_semantics_and_padding_never_become_background(self):
        sem=np.full((16,32,3),[113,178,37],np.uint8)
        self.assertFalse(semantic_grid(sem,[(127,127,127)],target=32).any())
        sem[:]=[127,127,127]
        g=semantic_grid(sem,[(127,127,127)],target=32)
        self.assertFalse(g[0].any());self.assertTrue(g[1:3].all());self.assertFalse(g[3].any())

    def test_stride_reduction_ignore_and_positive_priority(self):
        anchors=torch.tensor([[.5,.5],[1.5,.5],[.5,.5]])
        strides=torch.tensor([[8.],[8.],[16.]])
        b=dict(supervision_complete=torch.tensor([False]),background_safe_grid=torch.tensor([[[True,False],[True,True]]]),
            ignore_bboxes=torch.tensor([[.1,.1,.2,.2]]),ignore_batch_idx=torch.tensor([0]))
        fg=torch.tensor([[True,False,False]])
        self.assertEqual(classification_keep(b,anchors,strides,fg).tolist(),[[True,False,False]])

    def test_partial_known_box_negatives_only_its_class(self):
        for head in ('one2many','one2one'):
            c=wrap_criterion(E2ELoss(FakeModel()));p={'one2many':prediction(),'one2one':prediction()}
            b=batch(complete=False,positive=True);b['img']=torch.zeros(1,3,32,32)
            b['bboxes']=torch.tensor([[.5,.5,1.,1.]])
            b['known_class_background']=torch.tensor([True]);captured={}
            getattr(c,head).assigner.register_forward_hook(lambda m,a,o:captured.update(fg=o[3]))
            loss,_=c(p,b);loss.sum().backward();g=p[head]['scores'].grad.transpose(1,2)
            fg=captured['fg'];bg=~fg
            self.assertGreater(float(g[bg][:,0].abs().sum()),0)
            self.assertEqual(float(g[bg][:,1:].abs().sum()),0)
            self.assertGreater(float(g[fg].abs().sum()),0)

    def test_spatial_background_gradient_in_both_heads(self):
        c=wrap_criterion(E2ELoss(FakeModel()));p={'one2many':prediction(),'one2one':prediction()}
        b=batch(complete=False);b['background_safe_grid']=torch.zeros(1,4,4,dtype=torch.bool);b['background_safe_grid'][0,0,0]=True
        loss,_=c(p,b);loss.sum().backward()
        for head in p:
            self.assertGreater(float(p[head]['scores'].grad[:,:,0].abs().sum()),0)
            self.assertEqual(float(p[head]['scores'].grad[:,:,1:].abs().sum()),0)

    def test_ignored_area_blocks_new_class_negatives(self):
        anchors=torch.tensor([[.5,.5],[1.5,.5]]);strides=torch.full((2,1),8.)
        b=batch(complete=False,positive=True,ignore=[[0,0,32,32]])
        b['img']=torch.zeros(1,3,32,32);b['known_class_background']=torch.tensor([True])
        self.assertFalse(known_class_keep(b,anchors,strides,torch.zeros(1,2,dtype=torch.bool),16).any())

    def test_spatial_loader_flip_hash_and_batch_isolation(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);image=root/'rgb.png'
            cv2.imwrite(str(image),np.full((480,640,3),80,np.uint8))
            policy=load_policy();policy['spatial_background_version']=2
            policy['augmentation'].update(fliplr=1.,hsv_h=0,hsv_s=0,hsv_v=0)
            (root/'policy.json').write_text(json.dumps(policy))
            grid=np.zeros((120,120),bool);grid[:, :3]=True
            np.savez_compressed(root/'grid.npz',packed=np.packbits(grid))
            row=dict(id='one',split='train',image=str(image),width=640,height=480,
                image_sha256=sha(image),view='unknown',ego_mask_applied=False,supervision='partial',
                annotations=[],background_policy='spatial_v2',known_class_background=False,
                background_safe_grid='grid.npz',background_safe_grid_sha256=sha(root/'grid.npz'))
            (root/'manifest.jsonl').write_text(json.dumps(row)+'\n')
            (root/'data.yaml').write_text('names: '+json.dumps(list(CLASSES)))
            (root/'verification.json').write_text('{"passed":true}')
            meta=dict(classes=list(CLASSES),requires_ignore_aware_trainer=True,final_version=True,
                original_data_root=tmp,requires_spatial_background_trainer=2,
                artifact_shas={n:sha(root/n) for n in ('policy.json','manifest.jsonl')})
            (root/'pretrain.json').write_text(json.dumps(meta));(root/'_SUCCESS').write_text(sha(root/'pretrain.json'))
            ds=PretrainDataset(root,augment=True);sample=ds[0]
            self.assertTrue(sample['background_safe_grid'][:,-3:].all())
            self.assertFalse(sample['background_safe_grid'][:,:-3].any())
            self.assertFalse(sample['known_class_background'])
            joined=ds.collate_fn([sample,sample])
            self.assertEqual(tuple(joined['background_safe_grid'].shape),(2,120,120))
            (root/'grid.npz').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'SHA changed'):ds[0]


if __name__=='__main__':unittest.main()
