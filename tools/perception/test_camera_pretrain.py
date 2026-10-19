#!/usr/bin/env python3
"""Policy/export regressions without Torch or GPU."""
from collections import Counter
import copy
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from camera_pretrain_policy import (CLASSES, epoch_order, letterbox_geometry,
    load_policy, sha, supervision_state, tiny_reasons, transform_boxes, validate_dataset_contract,
    validate_source_paths, pretrain_data_dict)
from prepare_camera_pretrain import normalize_row, yolo_lines


class PolicyTests(unittest.TestCase):
    def setUp(self):
        self.policy = load_policy()

    def test_threshold_and_native_upscale(self):
        self.assertEqual(tiny_reasons([0,0,4,4],640,480,self.policy,4), [])
        self.assertIn('native_near_point', tiny_reasons([0,0,3,4],640,480,self.policy))
        self.assertIn('tensor_near_point', tiny_reasons([0,0,4,4],1280,720,self.policy))
        self.assertIn('insufficient_visible_instance_pixels', tiny_reasons([0,0,20,20],640,480,self.policy,3))
        self.assertEqual(tiny_reasons([0,0,2,8],640,480,self.policy,4), [])

    def test_letterbox_flip(self):
        self.assertEqual(letterbox_geometry(1280,720),(.75,.75,0,210,960,540))
        self.assertEqual(letterbox_geometry(640,480),(1.5,1.5,0,120,960,720))
        box = [100,200,160,300]
        a=transform_boxes([box],640,480)[0]
        b=transform_boxes([box],640,480,flip=True)[0]
        np.testing.assert_array_equal(b,[960-a[2],a[1],960-a[0],a[3]])

    def test_complete_is_not_source_name_assumption(self):
        self.assertEqual(supervision_state({'annotation_complete':True}),'partial')
        row=dict(annotation_complete=True,review_scope='camera_objects_v2_16',
                 review_record=dict(decision='accept',all_targets_annotated=True,review_scope='camera_objects_v2_16'))
        self.assertEqual(supervision_state(row),'complete')
        self.assertEqual(supervision_state({},native_route=True),'partial')

    def test_full_coverage_and_repeat_limit(self):
        rows=[dict(id=str(i),correlation_group='g'+str(i//3),annotations=[dict(
            training_class='person' if i<90 else 'truck',supervision='positive')]) for i in range(100)]
        original=copy.deepcopy(rows)
        a, report=epoch_order(rows,alignment=16)
        b, _=epoch_order(rows,alignment=16)
        c, _=epoch_order(rows,epoch=1,alignment=16)
        self.assertEqual(a,b)
        self.assertNotEqual(a,c)
        self.assertEqual(set(a),set(range(100)))
        self.assertLessEqual(max(Counter(a).values()),2)
        self.assertEqual(len(a)%16,0)
        self.assertEqual(len(a),len(c))
        self.assertEqual(rows,original)
        self.assertTrue(report['all_images_included'])

    def test_ignores_do_not_change_balance(self):
        rows=[dict(id=str(i),correlation_group=str(i),annotations=[dict(training_class='car',supervision='positive'),
             dict(training_class='stroller',supervision='ignore')]) for i in range(20)]
        _,r=epoch_order(rows)
        self.assertEqual(r['class_image_frequency'],{'car':20})

    def test_positive_only_yolo_and_wrong_trainer(self):
        row=dict(width=640,height=480,annotations=[dict(training_class='dog',supervision='positive',xyxy=[0,0,20,20]),
            dict(training_class='car',supervision='ignore',xyxy=[0,0,1,1])])
        self.assertEqual(len(yolo_lines(row)),1)
        self.assertTrue(yolo_lines(row)[0].startswith(str(CLASSES.index('dog'))+' '))
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'pretrain.json').write_text(json.dumps(dict(requires_ignore_aware_trainer=True)))
            with self.assertRaisesRegex(ValueError,'ignore-aware'):
                validate_dataset_contract(p)

    def test_source_mount_preflight_and_trainer_class_mapping(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);image=root/'rgb.png';image.write_bytes(b'source')
            (root/'manifest.jsonl').write_text(json.dumps(dict(image=str(image)))+'\n')
            self.assertEqual(validate_source_paths(root,dict(original_data_root=tmp)),1)
            actual=pretrain_data_dict(root)
            self.assertEqual(list(actual['names'].values()),list(CLASSES))
            self.assertEqual(actual['nc'],16)
            (root/'manifest.jsonl').write_text(json.dumps(dict(image='/missing_external_mount/rgb.png'))+'\n')
            with self.assertRaisesRegex(ValueError,'mount its original directory'):
                validate_source_paths(root,dict(original_data_root=tmp))

    def test_mask_recomputes_visible_instance_not_bbox_intersection(self):
        class Mask:
            config_sha256='test'
            def geometry(self,h,w):
                b=np.zeros((h,w),bool);b[:,5:]=True;return b,None
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);image=p/'image.png';cv2.imwrite(str(image),np.zeros((20,20,3),np.uint8))
            native=np.zeros((1,20,20),np.uint8);native[0,2:12,2:12]=1
            np.savez(p/'m.npz',masks=np.packbits(native,axis=-1),width=20,height=20)
            old=dict(id='one',source='capture:run',source_key='front/0',image=str(image),
                image_sha256=sha(image),pixel_sha256='original',split='train',split_group='run',view='front',
                annotations=[dict(id='0',training_class='person',xyxy=[2,2,12,12],
                    native_mask_root=str(p),native_mask='m.npz',mask_index=0,visible_pixels=100)])
            row=normalize_row(p,old,np.zeros((20,20,3),np.uint8),self.policy,{'front':Mask()},
                {'parent_shas':{str(p):'p'}})
            self.assertEqual(row['annotations'][0]['xyxy'],[2,2,5,12])
            self.assertEqual(row['annotations'][0]['visible_pixels'],30)
            self.assertEqual(old['annotations'][0]['xyxy'],[2,2,12,12])
            self.assertEqual(row['annotations'][0]['supervision'],'positive')

    def test_fully_occluded_is_never_positive(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);image=p/'rgb.png';cv2.imwrite(str(image),np.zeros((20,20,3),np.uint8))
            row=dict(id='hidden',source='capture:run',source_key='front/0',image=str(image),
                image_sha256=sha(image),pixel_sha256='source',split='train',split_group='run',view='unknown',
                annotations=[dict(training_class='car',xyxy=[0,0,10,10],visible_pixels=0)])
            result=normalize_row(p,row,np.zeros((20,20,3),np.uint8),self.policy,{},
                                 {'parent_shas':{str(p):'parent'}})
            self.assertEqual(result['annotations'][0]['supervision'],'excluded')
            self.assertEqual(yolo_lines(result),[])
            self.assertEqual(row['annotations'][0]['xyxy'],[0,0,10,10])

    def test_v8_evaluation_positive_before_ignore_and_partial(self):
        from evaluate_camera_bbox_pilot import frame_counts
        row=dict(supervision='complete',annotations=[
            dict(training_class='person',canonical='person',supervision='positive',xyxy=[0,0,10,10]),
            dict(training_class='person',canonical='person',supervision='ignore',xyxy=[0,0,10,10]),
            dict(training_class='dog',canonical='dog',supervision='ignore',xyxy=[20,20,25,25])])
        preds=[dict(class_name='person',confidence=.8,xyxy=[0,0,10,10]),
               dict(class_name='dog',confidence=.8,xyxy=[20,20,25,25])]
        counts,pairs=frame_counts(row,preds,.4,pixel_scope=True)
        self.assertEqual(len(pairs),1)
        self.assertEqual(counts['person']['complete_tp'],1)
        self.assertEqual(counts['dog']['complete_predictions'],0)
        row['supervision']='partial'
        row['evaluation_scope']='complete'  # Explicit v8 completeness takes priority over legacy fields.
        counts,_=frame_counts(row,preds,.4,pixel_scope=True)
        self.assertEqual(counts['person']['tp'],1)
        self.assertEqual(counts['person']['complete_tp'],0)


if __name__=='__main__':unittest.main()
