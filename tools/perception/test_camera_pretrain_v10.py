import unittest
from copy import deepcopy
from pathlib import Path
import json
import tempfile
from camera_pretrain_policy import CLASSES,load_policy,ROOT,epoch_order
from prepare_camera_pretrain_v10 import apply_policy,reviewed_panel
from review_camera_supervision_policy import assert_isolation,indexes
from finish_camera_epoch_boundary import completed

class V10Tests(unittest.TestCase):
 def setUp(self):self.policy=load_policy(ROOT/'config/datasets/camera_pretrain_policy_v3.json')
 def row(self,box,w=960,h=960):return dict(id='x',width=w,height=h,annotations=[dict(xyxy=box,training_class='person',supervision='positive',supervision_reasons=[])])
 def test_boundaries_and_native_tensor_agreement(self):
  self.assertEqual(apply_policy(self.row([0,0,4,16]),self.policy)['annotations'][0]['supervision'],'positive')
  for r in [self.row([0,0,3.99,100]),self.row([0,0,7,9]),self.row([0,0,4,16],1280,720),self.row([0,0,3,30],640,480)]:
   self.assertEqual(apply_policy(r,self.policy)['annotations'][0]['supervision'],'ignore')
 def test_original_geometry_and_policy_independent_of_class(self):
  r=self.row([2,2,6,10]);old=deepcopy(r)
  for c in CLASSES:
   r['annotations'][0]['training_class']=c
   out=apply_policy(r,self.policy);self.assertEqual(out['annotations'][0]['supervision'],'ignore');self.assertEqual(out['annotations'][0]['xyxy'],old['annotations'][0]['xyxy'])
  self.assertEqual(r['annotations'][0]['supervision'],'positive')
 def test_visible_and_prior_ignore_never_become_background(self):
  r=self.row([0,0,40,40]);r['annotations'][0]['visible_pixels']=3
  self.assertEqual(apply_policy(r,self.policy)['annotations'][0]['supervision'],'ignore')
  r['annotations'][0].update(visible_pixels=100,supervision='ignore')
  self.assertEqual(apply_policy(r,self.policy)['annotations'][0]['supervision'],'ignore')
 def test_expansion_excludes_ambiguous_semantic_categories(self):
  self.assertTrue({'building','road_edge','road_sign','traffic_sign','traffic_light'}<=set(self.policy['safe_semantic_classes']))
  self.assertFalse({'etc','standing_object','Obstacle'}&set(self.policy['safe_semantic_classes']))
 def test_cross_field_pixel_leak_rejected(self):
  row=dict(id='x',pixel_sha256='same');denied=indexes([dict(id='y',source_pixel_sha256='same')])
  with self.assertRaises(ValueError):assert_isolation([row],denied)
 def test_full_epoch_only_and_failures_block_closeout(self):
  self.assertFalse(completed(dict(last_epoch=7,epoch_boundary_pause=True),8))
  self.assertTrue(completed(dict(completed_epochs=8,epoch_boundary_pause=True,errors=[]),8))
  for r in [dict(completed_epochs=9),dict(errors=['OOM']),dict(status='failed')]:
   with self.assertRaises(RuntimeError):completed(r,8)

class TransitionGuards(unittest.TestCase):
 def test_unverified_data_cannot_start_and_identity_changes_fail(self):
  from transition_camera_pretrained import valid_data,sha
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);ready=root/'ready.json'
   self.assertFalse(valid_data(root,ready))
   (root/'pretrain.json').write_text('{}');(root/'_SUCCESS').write_text(sha(root/'pretrain.json'))
   proof=root/'proof.json';proof.write_text('passed')
   ready.write_text(json.dumps(dict(passed=True,dataset_metadata_sha256=sha(root/'pretrain.json'),required_files={str(proof):sha(proof)})))
   self.assertTrue(valid_data(root,ready));proof.write_text('modified')
   with self.assertRaises(ValueError):valid_data(root,ready)
 def test_closeout_requires_matching_eighth_epoch_checkpoint(self):
  from transition_camera_pretrained import valid_closeout,sha
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);self.assertFalse(valid_closeout(root));c=root/'operator-closeout';c.mkdir()
   weight=root/'epoch.pt';weight.write_bytes(b'checkpoint')
   result=c/'result.json';result.write_text(json.dumps(dict(epoch=7,checkpoint=str(weight),sha256=sha(weight))))
   (c/'_EVALUATED').write_text(sha(result))
   with self.assertRaises(ValueError):valid_closeout(root)

if __name__=='__main__':unittest.main()
