#!/usr/bin/env python3
import unittest
import numpy as np
from std_msgs.msg import Header
from asmc_camera.detection.backend import Detection
from asmc_camera.detection.batching import pack_image, shapes_for_front
from asmc_camera.detection.classes import load_class_groups
from asmc_camera.detection.messages import build_box_array
from asmc_camera.detection.yolo_batch_backend import COCO7
from asmc_camera.runtime.state import InferenceState, ACTIVE


class Coco7RuntimeTest(unittest.TestCase):
    def test_source_names_order_and_coordinates(self):
        groups = load_class_groups({'groups': {'person': ['person'], 'vehicle': ['car']},
                                    'default_group': 'obstacle', 'excluded': []})
        boxes = [Detection(1.2,2.3,12.4,23.5,n,.8) for n in COCO7]
        header = Header(frame_id='camera_right')
        header.stamp.secs = 12
        result = build_box_array(header,640,480,True,boxes,groups,'source')
        self.assertEqual(list(COCO7), [b.class_name for b in result.boxes])
        self.assertEqual(list(COCO7), [b.source_class_name for b in result.boxes])
        self.assertEqual(header, result.header)
        self.assertAlmostEqual(1.2, result.boxes[0].xmin, places=5)

    def test_packing_widths_and_inverse(self):
        for width,height in ((640,384),(960,544),(1280,736)):
            shapes=shapes_for_front(width)
            image=np.zeros((720,1280,3),np.uint8)
            packed,t=pack_image(image,'front',shapes=shapes)
            self.assertEqual((height,width,3),packed.shape)
            d=Detection(t.left,t.top,t.left+t.resized_hw[1],t.top+t.resized_hw[0],'car',.9)
            restored=t.box(d)
            self.assertEqual((0,0,1280,720),(restored.xmin,restored.ymin,restored.xmax,restored.ymax))
            for side in ('left','right'):
                common=(max(height,480),max(width,640))
                _,st=pack_image(np.zeros((480,640,3),np.uint8),side,common,shapes)
                self.assertEqual((480,640),st.resized_hw)

    def test_view_rates_and_service_override(self):
        clock=[1.]
        state=InferenceState(['front','left','right'],managed=True,initial_mode=ACTIVE,
            target_hz=20,target_hz_by_view={'left':10,'right':10},clock=lambda:clock[0])
        for view in state.views:
            self.assertIsNotNone(state.begin({view:1},per_view=True))
        clock[0]+=.06
        self.assertIsNotNone(state.begin({'front':2},per_view=True))
        self.assertIsNone(state.begin({'left':2},per_view=True))
        accepted,_,_=state.request(ACTIVE,20)
        self.assertTrue(accepted)
        self.assertEqual({},state.target_hz_by_view)

    def test_invalid_configuration(self):
        with self.assertRaises(ValueError): shapes_for_front(1000)
        with self.assertRaises(ValueError):
            InferenceState(['front'],target_hz_by_view={'left':10})


if __name__=='__main__': unittest.main()
