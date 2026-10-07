import unittest
from types import SimpleNamespace
import numpy as np
from asmc_camera.detection.visualization import render_object_overlay

class ObjectDrawingTest(unittest.TestCase):
    def setUp(self):
        self.image=np.zeros((160,240,3),np.uint8)
        self.box=SimpleNamespace(xmin=60.,ymin=70.,xmax=140.,ymax=130.,confidence=.9,
            confidence_valid=True,source_class_name='car',class_name='vehicle')

    def test_box_is_drawn_without_changing_input_or_detector_coordinates(self):
        before=vars(self.box).copy()
        output=render_object_overlay(self.image,SimpleNamespace(valid=True,boxes=[self.box]))
        self.assertTrue(output[70,60].any())
        self.assertFalse(self.image.any())
        self.assertEqual(before,vars(self.box))

    def test_empty_valid_result_still_has_an_image(self):
        output=render_object_overlay(self.image,SimpleNamespace(valid=True,boxes=[]))
        self.assertEqual(self.image.shape,output.shape)
        self.assertTrue(output[:35].any())

    def test_invalid_result_never_draws_its_boxes(self):
        output=render_object_overlay(self.image,SimpleNamespace(valid=False,boxes=[self.box]))
        self.assertFalse(output[60:].any())
        self.assertTrue(output[:35].any())

    def test_nonfinite_and_reversed_boxes_are_skipped(self):
        for key,value in [('xmin',float('nan')),('confidence',float('inf')),('xmax',20.)]:
            box=SimpleNamespace(**vars(self.box));setattr(box,key,value)
            output=render_object_overlay(self.image,SimpleNamespace(valid=True,boxes=[box]))
            self.assertFalse(output[60:].any())
