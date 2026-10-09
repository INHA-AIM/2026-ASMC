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


class ObjectMosaicDrawingTest(unittest.TestCase):
    def test_left_front_right_order_preserves_aspect_ratio_and_input_pixels(self):
        from asmc_camera.detection.visualization import render_object_mosaic
        images = {view:np.full(shape,color,np.uint8) for view,shape,color in (
            ('left',(60,80,3),(80,0,0)),('front',(90,160,3),(0,80,0)),
            ('right',(60,80,3),(0,0,80)))}
        before = {v:i.copy() for v,i in images.items()}
        output = render_object_mosaic({v:(i,9.9,19.9) for v,i in images.items()},10.,20.,80,60)
        self.assertEqual((112,240,3),output.shape)
        for n,view in enumerate(('left','front','right')):
            np.testing.assert_array_equal(output[82,40+n*80],images[view][0,0])
            np.testing.assert_array_equal(images[view],before[view])
        self.assertFalse(output[52:59,80:160].any())  # Front letterbox, not stretched.

    def test_source_age_and_wall_receipt_each_hide_old_boxes_independently(self):
        from asmc_camera.detection.visualization import render_object_mosaic
        image=np.full((60,80,3),200,np.uint8)
        for stamp,received in ((9.,19.9),(9.9,19.),(10.1,19.9)):
            output=render_object_mosaic({'front':(image,stamp,received)},10.,20.,80,60)
            self.assertFalse(output[52:].any())
            self.assertTrue(output[:52].any())

    def test_missing_view_stays_blank_while_other_views_update(self):
        from asmc_camera.detection.visualization import render_object_mosaic
        image=np.full((60,80,3),100,np.uint8)
        output=render_object_mosaic({'front':(image,10.,20.)},10.,20.,80,60)
        self.assertFalse(output[52:,:80].any())
        self.assertTrue(output[52:,80:160].all())
        self.assertFalse(output[52:,160:].any())
