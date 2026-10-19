import unittest
from review_camera_supervision_policy import (RangeLinker, assert_isolation, balanced,
    band, indexes, proposed_row, palette_counts, color_key, resplit_group)
import numpy as np
from pathlib import Path
import json
import tempfile
from review_camera_supervision_policy import export_eval


def row(key, labels=('person',), split='selection', group=None):
    return dict(id=key,image_sha256='sha:'+key,pixel_sha256='pixels:'+key,
        split_group=group or key,correlation_group=key,source='source:'+key,
        split=split,width=960,height=960,view='front',weather='SUNNY',
        supervision='complete',annotations=[dict(id=str(i),training_class=c,
        xyxy=[0,0,8,8],supervision='positive') for i,c in enumerate(labels)])


class PolicyReviewTests(unittest.TestCase):
    def test_threshold_boundaries_and_previous_ignore_preserved(self):
        r=row('a');r['annotations'] += [dict(training_class='car',xyxy=[0,0,100,100],supervision='ignore')]
        self.assertEqual(proposed_row(r)['annotations'][0]['supervision'],'positive')
        r['annotations'][0]['xyxy']=[0,0,4,15]
        new=proposed_row(r)
        self.assertEqual(new['annotations'][0]['supervision'],'ignore')
        self.assertEqual(new['annotations'][1]['supervision'],'ignore')
        self.assertEqual(r['annotations'][0]['supervision'],'positive')

    def test_native_downsample_even_if_original_passes(self):
        r=row('a');r.update(width=1280,height=720)
        self.assertEqual(proposed_row(r)['annotations'][0]['supervision'],'ignore')

    def test_image_balancing_never_drops_coexisting_annotations(self):
        pool=[row('a',('person','box'))]+[row(str(i),('person',)) for i in range(12)]
        chosen=balanced(pool,target=2)
        selected=next(r for r in chosen if r['id']=='a')
        self.assertEqual(len(selected['annotations']),2)
        self.assertEqual(len({r['id'] for r in chosen}),len(chosen))
        self.assertEqual([r['id'] for r in chosen],[r['id'] for r in balanced(list(reversed(pool)),target=2)])

    def test_cross_field_train_pixel_leak_and_group_leak(self):
        train=row('train',split='train');test=row('test')
        test['source_pixel_sha256']=train['pixel_sha256']
        with self.assertRaises(ValueError):assert_isolation([test],indexes([train]))
        test.pop('source_pixel_sha256');test['split_group']=train['split_group']
        with self.assertRaises(ValueError):assert_isolation([test],indexes([train]))

    def test_distance_unknown_and_boundary(self):
        self.assertEqual(band(None),'unknown')
        self.assertEqual(band(20),'20-40m')
        self.assertEqual(band(60),'60m+')
        obj=object.__new__(RangeLinker)
        self.assertIsNone(obj.distance(({'1':{'x':0,'y':0,'z':0}},dict(x=0,y=0,z=0)),dict(actor_id='10001')))
        self.assertEqual(obj.distance(({'1':{'x':3,'y':4,'z':0}},dict(x=0,y=0,z=0)),dict(actor_id='1')),5)

    def test_palette_histogram_preserves_tiny_pixels_and_excludes_ego(self):
        sem=np.array([[[1,2,3],[1,2,3]],[[255,0,0],[0,0,0]]],dtype=np.uint8)
        ego=np.array([[False,True],[False,False]])
        self.assertEqual(palette_counts(sem,ego),{color_key([1,2,3]):1,color_key([255,0,0]):1,0:1})

    def test_weather_scenario_family_and_correlation_cap(self):
        a=row('a');b=row('b')
        a['source']='capture:20260903_comp_sample_sunny_11'
        b['source']='capture:20260904_comp_sample_foggy_15'
        self.assertEqual(resplit_group(a),resplit_group(b))
        pool=[row(str(i)) for i in range(8)]
        for r in pool:r['correlation_group']='same_sequence'
        self.assertEqual(len(balanced(pool,target=8,correlation_cap=2)),2)

    def test_empty_positive_export_keeps_partial_ignore_sidecar(self):
        r=row('tiny');r['supervision']='partial';r['annotations'][0]['xyxy']=[0,0,4,8]
        r=proposed_row(r)
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'candidate';export_eval(p,[r])
            self.assertEqual((p/'labels/tiny.txt').read_text(),'')
            side=json.loads((p/'supervision/tiny.json').read_text())
            self.assertEqual(side['supervision'],'partial')
            self.assertEqual(side['annotations'][0]['supervision'],'ignore')


if __name__=='__main__':unittest.main()
