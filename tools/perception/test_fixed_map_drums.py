"""Tests for conservative semantic proposals and reviewed patch decisions."""
import unittest
import numpy as np
from prepare_fixed_map_drums import components, silhouette_ratio, bbox_iou
from export_fixed_map_drums import decide
from audit_camera_source_coverage import audit

class DrumTests(unittest.TestCase):
    def scene(self):
        rgb=np.zeros((80,80,3),dtype=np.uint8);sem=rgb.copy()
        rgb[10:50,20:40]=[240,130,25];sem[10:50,20:40]=[113,178,37]
        return rgb,sem
    def test_palette_is_not_class_identity(self):
        rgb,sem=self.scene();self.assertEqual(len(components(rgb,sem)),1)
        rgb[:]=80;self.assertEqual(components(rgb,sem),[])
    def test_ego_and_full_merge_exclusion(self):
        rgb,sem=self.scene();ego=np.zeros((80,80),bool);ego[10:50,20:40]=True
        self.assertEqual(components(rgb,sem,ego),[])
        rgb[10:50,40:75]=[240,130,25];sem[10:50,40:75]=[113,178,37]
        self.assertEqual(components(rgb,sem),[])
    def test_join_not_split_and_small_identity_gate(self):
        m=np.ones((40,20),bool);m[18:22,:]=False;m[18:22,9:11]=True
        self.assertLess(silhouette_ratio(m),.7)
        p=dict(xyxy=[0,0,20,40],proposal_id='x',conflicting_existing_labels=[])
        self.assertTrue(decide(p,m,set())[0])
        self.assertFalse(decide(p,np.ones((40,20),bool),set())[0])
        p['xyxy']=[0,0,11,22]
        self.assertTrue(decide(p,np.ones((22,11),bool),set())[0])
    def test_review_conflict_and_duplicates(self):
        p=dict(xyxy=[0,0,20,40],proposal_id='x',conflicting_existing_labels=['box'])
        self.assertIn('existing_label_conflict',decide(p,np.ones((40,20),bool),{'x'})[0])
        self.assertEqual(bbox_iou([0,0,20,40],[0,0,20,40]),1)
    def test_source_boxes_not_images_or_models(self):
        r=dict(split='train',id='1',source='roboflow:test',split_group='family',annotations=[dict(training_class='car',supervision='positive')]*2+[dict(training_class='car',supervision='ignore')])
        a=audit([r])[0]
        self.assertEqual((a['boxes'],a['images'],a['groups']),(2,1,1));self.assertEqual(a['subtypes'],{'unknown':2})

if __name__=='__main__':unittest.main()
