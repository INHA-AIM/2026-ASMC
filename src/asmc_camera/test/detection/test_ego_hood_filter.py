import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

import numpy as np
from rospy import Time
from std_msgs.msg import Header

from asmc_camera.detection.backend import Detection
from asmc_camera.detection.classes import load_class_groups
from asmc_camera.detection.ego_hood_filter import EgoHoodFilter
from asmc_camera.detection.hood_mask import HoodMask
from asmc_camera.detection.processor import ObjectFrameProcessor


def box(x1=20, y1=80, x2=80, y2=95, name='car'):
    return Detection(x1,y1,x2,y2,name,.91)


class EgoHoodFilterTest(unittest.TestCase):
    def test_all_three_inclusive_boundaries_required(self):
        f=EgoHoodFilter(True)
        self.assertTrue(f.excludes(box(),100,100))
        for d in [box(x2=79),box(y1=74),box(y2=94),box(x1=70,x2=70)]:
            self.assertFalse(f.excludes(d,100,100))

    def test_noncar_and_nearby_tall_vehicles_remain(self):
        f=EgoHoodFilter(True)
        for name in ['person','truck','bus','bicycle']:
            self.assertFalse(f.excludes(box(name=name),100,100))
        self.assertFalse(f.excludes(box(y1=20,y2=100),100,100))
        self.assertTrue(f.excludes(box(name='Car'),100,100))

    def test_predicate_uses_legacy_int_without_mutating_detection(self):
        d=box(x1=20.99,y1=80.9,x2=80.1,y2=95.7)
        self.assertTrue(EgoHoodFilter(True).excludes(d,100,100))
        self.assertEqual((20.99,80.9,80.1,95.7),(d.xmin,d.ymin,d.xmax,d.ymax))

    def test_scale_and_disabled_filter(self):
        f=EgoHoodFilter(True)
        self.assertTrue(f.excludes(box(40,160,160,190),200,200))
        self.assertFalse(EgoHoodFilter().excludes(box(),100,100))
        self.assertFalse(EgoHoodFilter.load(False,'missing').enabled)

    def test_nonfinite_box_is_not_labeled_as_ego(self):
        self.assertFalse(EgoHoodFilter(True).excludes(box(x1=float('nan')),100,100))

    def test_partition_retains_order_labels_and_identity(self):
        detections=[box(name='person'),box(),box(y1=20),box(name='truck')]
        kept,excluded=EgoHoodFilter(True).partition(detections,100,100)
        self.assertEqual([detections[i] for i in [0,2,3]],kept)
        self.assertIs(detections[1],excluded[0])

    def test_invalid_config_or_expanding_classes_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'filter.yaml'
            valid='version: 1\nsource_class_names: [car]\nmin_width_ratio: 0.60\nmin_center_y_ratio: 0.85\nmin_bottom_ratio: 0.95\n'
            p.write_text(valid);self.assertEqual(64,len(EgoHoodFilter.load(True,p).config_sha256))
            for content in [valid.replace('version: 1','version: true'),valid.replace('[car]','[car, truck]'),
                            valid.replace('0.60','0'),valid.replace('0.85','.nan'),valid+'unknown: 1\n']:
                p.write_text(content)
                with self.assertRaises(ValueError):EgoHoodFilter.load(True,p)

    def processor(self,detections):
        backend=Mock();backend.detect.return_value=(detections,{})
        groups=load_class_groups({'groups':{'person':['person'],'vehicle':['car','truck']}})
        image=np.full((100,100,3),127,np.uint8)
        return ObjectFrameProcessor(backend,groups,decode=lambda *a:image,ego_hood_filter=EgoHoodFilter(True)),backend,image

    def test_one_inference_raw_and_filtered_share_source_contract(self):
        p,backend,image=self.processor([box(name='person'),box(),box(y1=20)])
        h=Header(stamp=Time(42),frame_id='camera_front')
        filtered,raw,timings,error=p.process_with_raw(h,b'fixture')
        self.assertIsNone(error);backend.detect.assert_called_once()
        self.assertIs(image,backend.detect.call_args.args[0]);self.assertTrue((image==127).all())
        self.assertEqual(3,len(raw.boxes));self.assertEqual(2,len(filtered.boxes))
        self.assertEqual(h,raw.header);self.assertEqual(h,filtered.header)
        self.assertEqual((100,100),(filtered.image_width,filtered.image_height))
        self.assertEqual([raw.boxes[0],raw.boxes[2]],filtered.boxes)
        self.assertIn('ego_hood_filter_ms',timings)

    def test_only_hood_and_empty_results_are_valid_empty(self):
        for detections in [[],[box()]]:
            p,_,_=self.processor(detections);message,_,error=p.process(Header(),b'fixture')
            self.assertIsNone(error);self.assertTrue(message.valid);self.assertFalse(message.boxes)

    def test_filter_error_invalidates_filtered_but_retains_successful_raw(self):
        p,_,_=self.processor([box()])
        p.ego_hood_filter.partition=Mock(side_effect=RuntimeError('injected'))
        h=Header(stamp=Time(42),frame_id='camera_front')
        message,raw,_,error=p.process_with_raw(h,b'fixture')
        self.assertEqual('filter',error[0]);self.assertFalse(message.valid);self.assertTrue(raw.valid)
        self.assertEqual(1,len(raw.boxes));self.assertEqual(h,message.header)
        self.assertEqual((100,100),(message.image_width,message.image_height))

    def test_mask_and_filter_cannot_run_together(self):
        with self.assertRaises(ValueError):
            ObjectFrameProcessor(Mock(),Mock(),hood_mask=HoodMask([(0,.8),(1,.8),(1,1),(0,1)]),ego_hood_filter=EgoHoodFilter(True))
