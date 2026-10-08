import unittest
import numpy as np
from prepare_camera_seg_gt import convert_arrays,parse_bbox,split_runs


class GTTest(unittest.TestCase):
    def test_actor_link_visible_bbox_and_source_preserved(self):
        inst=np.zeros((4,5,4),np.uint8); sem=np.zeros((4,5,3),np.uint8)
        inst[1:3,2:4]=[1,2,3,255]; sem[1:3,2:4]=[255,90,90]
        rows=parse_bbox('Vehicle 0 0 5 4 17 (1, 2, 3, 255)')
        a,m,e=convert_arrays(inst,sem,rows)
        self.assertEqual(e,[])
        self.assertEqual(a[0]['source_class_name'],'truck')
        self.assertEqual(a[0]['gt_source_label'],'Vehicle')
        self.assertEqual(a[0]['xyxy'],[2,1,4,3])
        self.assertEqual(m[0].sum(),4)

    def test_missing_bbox_with_visible_actor_is_not_negative(self):
        inst=np.zeros((2,2,4),np.uint8); sem=np.zeros((2,2,3),np.uint8)
        sem[0,0]=[98,2,255]
        _,_,e=convert_arrays(inst,sem,[])
        self.assertEqual(e[0]['reason'],'unlinked_actor_semantic_pixels')

    def test_invisible_or_shared_instance_colors_are_quarantined(self):
        inst=np.zeros((2,2,4),np.uint8); sem=np.zeros((2,2,3),np.uint8)
        rows=parse_bbox('Pedestrian 0 0 2 2 1 (1, 2, 3, 255)')
        self.assertEqual(convert_arrays(inst,sem,rows)[2][0]['reason'],'no_visible_instance_pixels')
        self.assertEqual(convert_arrays(inst,sem,rows*2)[2][0]['reason'],'ambiguous_instance_color')

    def test_split_is_stable_and_whole_run(self):
        names=[str(i) for i in range(18)]
        result=split_runs(names)
        self.assertEqual(result,split_runs(list(reversed(names))))
        self.assertEqual(list(result.values()).count('train'),12)
        self.assertEqual(list(result.values()).count('selection'),3)
        self.assertEqual(list(result.values()).count('heldout'),3)

    def test_invalid_bbox_fails(self):
        with self.assertRaises(ValueError):parse_bbox('broken')
        with self.assertRaises(ValueError):parse_bbox('Truck 3 0 1 4 1 (1, 2, 3, 255)')


if __name__=='__main__':unittest.main()
