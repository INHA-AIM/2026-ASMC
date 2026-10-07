import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

import numpy as np
from rospy import Time
from std_msgs.msg import Header

from asmc_camera.detection.backend import Detection
from asmc_camera.detection.classes import load_class_groups
from asmc_camera.detection.hood_mask import HoodMask
from asmc_camera.detection.processor import ObjectFrameProcessor
from asmc_camera.detection.visualization import render_object_overlay


POLYGON = [(0,.79),(1,.79),(1,1),(0,1)]


class HoodMaskTest(unittest.TestCase):
    def test_disabled_is_identity_even_without_config(self):
        image = np.full((720,1280,3),127,np.uint8)
        self.assertIs(image,HoodMask.load(False,'missing.yaml').apply(image))

    def test_source_unchanged_dimensions_preserved_and_cache_readonly(self):
        mask=HoodMask(POLYGON)
        for h,w in [(720,1280),(480,640)]:
            image=np.full((h,w,3),127,np.uint8)
            out=mask.apply(image)
            cutoff=int(h*.79)
            self.assertEqual(image.shape,out.shape)
            self.assertTrue((image==127).all())
            self.assertTrue((out[:cutoff]==127).all())
            self.assertFalse(out[cutoff:].any())
            bitmap,_=mask.geometry(h,w)
            self.assertIs(bitmap,mask.geometry(h,w)[0])
            self.assertFalse(bitmap.flags.writeable)
            self.assertEqual((h-cutoff)*w,mask.info['excluded_pixels'])

    def test_nonrectangular_mask_keeps_lower_visible_pixels(self):
        image=np.ones((100,100,3),np.uint8)
        out=HoodMask([(0,.9),(.5,.7),(1,.9),(1,1),(0,1)]).apply(image)
        self.assertTrue(out[75,5].all())
        self.assertFalse(out[75,50].any())

    def test_invalid_polygons_are_rejected(self):
        for polygon in [[],[(0,0),(1,1)],[(0,0),(1,1),(0,1),(1,0)],
                        [(0,0),(1,0),(1,1),(0,1)],[(0,0),(0,0),(1,1)],
                        [(0,0),(1,1),(.5,.5)],[(0,0),(1,0),(1,float('nan'))],
                        [(0,0),(1,0),(1,1.1)],[(0,0,1),(1,0),(1,1)]]:
            with self.subTest(polygon=polygon),self.assertRaises(ValueError):HoodMask(polygon)

    def test_config_sha_and_invalid_configuration(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'mask.yaml';p.write_text('version: 1\npolygon: [[0, 0.79], [1, 0.79], [1, 1], [0, 1]]\n')
            mask=HoodMask.load(True,p)
            self.assertEqual(64,len(mask.config_sha256))
            p.write_text('version: 2\npolygon: []\n')
            with self.assertRaises(ValueError):HoodMask.load(True,p)

    def test_processing_keeps_boundary_boxes_labels_order_and_header(self):
        source=np.full((100,120,3),127,np.uint8)
        backend=Mock()
        backend.detect.return_value=([Detection(5,60,30,95,'person',.9),Detection(40,30,80,90,'car',.8)],{})
        groups=load_class_groups({'groups':{'person':['person'],'vehicle':['car']}})
        processor=ObjectFrameProcessor(backend,groups,decode=lambda *a:source,hood_mask=HoodMask(POLYGON))
        header=Header(stamp=Time(42),frame_id='camera_front')
        message,timings,error=processor.process(header,b'fixture')
        self.assertIsNone(error);self.assertTrue(message.valid)
        self.assertEqual(header,message.header)
        self.assertEqual((120,100),(message.image_width,message.image_height))
        self.assertEqual(['person','car'],[b.source_class_name for b in message.boxes])
        self.assertEqual([95,90],[b.ymax for b in message.boxes])
        self.assertFalse(backend.detect.call_args.args[0][79:].any())
        self.assertTrue((source==127).all());self.assertIn('hood_mask_ms',timings)

    def test_mask_failure_is_invalid_and_never_calls_backend(self):
        backend=Mock();groups=load_class_groups({'groups':{'person':['person'],'vehicle':['car']}})
        processor=ObjectFrameProcessor(backend,groups,decode=lambda *a:np.zeros((1,1,3),np.uint8),hood_mask=HoodMask(POLYGON))
        header=Header(stamp=Time(42),frame_id='camera_front')
        message,_,error=processor.process(header,b'fixture')
        self.assertEqual('mask',error[0]);self.assertFalse(message.valid)
        self.assertEqual((1,1),(message.image_width,message.image_height));self.assertEqual(header,message.header)
        backend.detect.assert_not_called()

    def test_overlay_marks_mask_without_modifying_source(self):
        image=np.full((100,120,3),100,np.uint8)
        result=Mock(valid=True,boxes=[])
        out=render_object_overlay(image,result,HoodMask(POLYGON,'fixture-sha'))
        self.assertEqual(image.shape,out.shape);self.assertTrue((image==100).all())
        self.assertGreater(int(out[95,60,2]),int(out[95,60,1]))
