import unittest
from asmc_camera.detection.mission import GROUPS, CLASSES, group_of
from asmc_camera.detection.classes import load_class_groups
from asmc_camera.detection.output_nms import suppress

class MissionTests(unittest.TestCase):
    def test_fixed_mapping_and_unknown(self):
        c=load_class_groups(dict(profile='mission16_v1',groups={k:list(v) for k,v in GROUPS.items()},default_group='unknown'))
        self.assertEqual(len(CLASSES),16)
        for n in CLASSES:self.assertEqual(c.group_of(n),group_of(n))
        self.assertEqual(c.group_of('motorcycle'),'static_obstacle')
        self.assertEqual(c.group_of('bicycle'),'dynamic_obstacle')
        self.assertEqual(c.group_of('stroller'),'dynamic_obstacle')
        self.assertIsNone(c.group_of('building'))
    def test_wrong_mapping_fails(self):
        bad={k:list(v) for k,v in GROUPS.items()};bad['vehicle'].append('motorcycle')
        with self.assertRaises(ValueError):load_class_groups(dict(profile='mission16_v1',groups=bad,default_group='unknown'))
    def test_legacy_mapping_preserved(self):
        c=load_class_groups(dict(groups=dict(person=['person'],vehicle=['car'])))
        self.assertEqual(c.group_of('person'),'person');self.assertEqual(c.group_of('box'),'obstacle')
    def test_no_group_nms_and_stable_tie(self):
        b=lambda n,c:dict(class_name=n,confidence=c,xyxy=[0,0,20,20])
        out,removed=suppress([b('car',.8),b('car',.8),b('bus',.8)],.4,.7)
        self.assertEqual([v['class_name'] for v in out],['car','bus'])
        self.assertEqual(removed[0]['index'],1)
    def test_exclusive_names_preserve_person_bicycle_stroller_and_neighbors(self):
        boxes=[dict(class_name=n,confidence=.9-i*.01,xyxy=[0,0,20,20])
               for i,n in enumerate(['car','bus','person','bicycle','stroller','dog','deer'])]
        boxes.append(dict(class_name='car',confidence=.6,xyxy=[20,0,40,20]))
        out,removed=suppress(boxes,.4,.7,.9)
        self.assertEqual([b['class_name'] for b in out],['car','person','bicycle','stroller','dog','car'])
        self.assertEqual(len(removed),2)
    def test_nms_strict_boundary(self):
        boxes=[dict(class_name='car',confidence=.8,xyxy=[0,0,30,10]),
               dict(class_name='car',confidence=.7,xyxy=[10,0,40,10])]
        self.assertEqual(len(suppress(boxes,.4,.5)[0]),2)
        self.assertEqual(len(suppress(boxes,.4,.49)[0]),1)
    def test_padding_cache_uses_packed_geometry_but_preserves_published_box(self):
        a=dict(class_name='car',confidence=.8,xyxy=[0,0,10,20],nms_xyxy=[0,0,20,20])
        b=dict(class_name='car',confidence=.7,xyxy=[0,0,10,20],nms_xyxy=[10,0,20,20])
        kept,_=suppress([a,b],.4,.7)
        self.assertEqual(len(kept),2)
        self.assertEqual(kept[0]['xyxy'],[0,0,10,20])
if __name__=='__main__':unittest.main()
