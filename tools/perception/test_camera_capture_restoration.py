import unittest
from copy import deepcopy
from prepare_camera_capture_restoration import merge_patch
from prepare_camera_spatial_background import semantic_grid
from prepare_camera_pretrain_v10 import apply_policy
from camera_pretrain_policy import load_policy, ROOT
import numpy as np

class RestorationTests(unittest.TestCase):
 def test_patch_preserves_original_boxes_and_identity(self):
  parent=dict(id='a',image_sha256='sha',annotations=[dict(training_class='car',xyxy=[1,2,10,20],supervision='positive')],candidate_train_eligible=True)
  patch=deepcopy(parent);patch['annotations'].append(dict(training_class='traffic_drum',xyxy=[20,20,40,60],supervision='positive'));patch['map_drum_review']=dict(original_annotation_count=1)
  result=merge_patch(parent,patch)
  self.assertEqual(result['annotations'][0],parent['annotations'][0]);self.assertEqual(len(parent['annotations']),1)
  patch['annotations'][0]['xyxy'][0]=9
  with self.assertRaises(ValueError):merge_patch(parent,patch)
 def test_patch_sha_cannot_change(self):
  parent=dict(id='a',image_sha256='sha',annotations=[]);patch=dict(parent,image_sha256='wrong',map_drum_review=dict(original_annotation_count=0))
  with self.assertRaises(ValueError):merge_patch(parent,patch)
 def test_partial_background_excludes_map_object_and_padding(self):
  s=np.full((16,32,3),[127,127,127],np.uint8);s[:,0:8]=[113,178,37]
  grid=semantic_grid(s,[(127,127,127)],target=32)
  self.assertFalse(grid[:,0].any());self.assertFalse(grid[0].any());self.assertTrue(grid[1:3,1:].all())
 def test_same_tiny_policy_keeps_boxes_not_class_dependent(self):
  policy=load_policy(ROOT/'config/datasets/camera_pretrain_policy_v3.json')
  r=dict(id='r',width=1280,height=720,annotations=[dict(training_class='car',xyxy=[0,0,20,20],supervision='positive'),dict(training_class='traffic_drum',xyxy=[0,0,4,16],supervision='positive')])
  v=apply_policy(r,policy);self.assertEqual([a['supervision'] for a in v['annotations']],['positive','ignore'])
if __name__=='__main__':unittest.main()
