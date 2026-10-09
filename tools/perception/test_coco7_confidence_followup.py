import unittest
from copy import deepcopy
from analyze_coco7_confidence_followup import observations, same_fp


class ConfidenceObservationTest(unittest.TestCase):
    def test_gt_identity_survives_unknown_annotation_and_added_group(self):
        gt={'a':dict(annotations=[dict(source_class_name='unknown',subtype_verified=False,xyxy=[20,20,30,30]),
            dict(source_class_name='car',subtype_verified=True,xyxy=[0,0,10,10])])}
        before=deepcopy(gt)
        predictions=[dict(image='a',predictions=[dict(source_class_name='car',confidence=.45,xyxy=[0,0,10,10]),
            dict(source_class_name='truck',confidence=.42,xyxy=[0,0,10,10]),
            dict(source_class_name='person',confidence=.31,xyxy=[40,40,50,50]),
            dict(source_class_name='car',confidence=.4,xyxy=[20,20,30,30])])]
        low, fps=observations(gt,predictions,.3)
        self.assertEqual({('a',1)},set(low))
        self.assertEqual(['wrong_label_overlap','unmatched_box'],[f['kind'] for f in fps])
        high, high_fp=observations(gt,predictions,.5)
        self.assertFalse(high); self.assertFalse(high_fp)
        self.assertEqual(before,gt)

    def test_fp_equivalence_requires_same_source_label_and_geometry(self):
        a=dict(image='a',box=dict(source_class_name='person',xyxy=[0,0,10,10]))
        self.assertTrue(same_fp(a,deepcopy(a)))
        for field,value in [('image','b')]:
            b=deepcopy(a);b[field]=value;self.assertFalse(same_fp(a,b))
        b=deepcopy(a);b['box']['source_class_name']='car';self.assertFalse(same_fp(a,b))
        b=deepcopy(a);b['box']['xyxy']=[5,5,15,15];self.assertFalse(same_fp(a,b))


if __name__=='__main__': unittest.main()
