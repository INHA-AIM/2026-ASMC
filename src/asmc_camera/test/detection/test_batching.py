import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import cv2
import numpy as np
import yaml
from std_msgs.msg import Header
from rospy import Time
from asmc_camera.detection.backend import Detection
from asmc_camera.detection.batching import pack_image, batch_groups
from asmc_camera.detection.batch_processor import process_batch
from asmc_camera.detection.classes import load_class_groups
from asmc_camera.detection.processor import ObjectFrameProcessor
from asmc_camera.detection.segmentation import SegmentationFrame


class BatchTest(unittest.TestCase):
    def test_native_front_padding_and_inverse(self):
        image=np.ones((720,1280,3),np.uint8)*17
        packed,t=pack_image(image,'front')
        self.assertEqual((736,1280,3),packed.shape)
        self.assertTrue((packed[8:728]==image).all())
        d=Detection(20.5,28.5,100.5,108.5,'car',.9)
        self.assertEqual(Detection(20.5,20.5,100.5,100.5,'car',.9),t.box(d))
        bitmap=np.zeros(packed.shape[:2],bool);bitmap[28:108,20:100]=True
        self.assertTrue(t.mask(bitmap)[20:100,20:100].all())
        self.assertEqual(6400,t.mask(bitmap).sum())

    def test_batch3_sides_are_padded_without_upscaling(self):
        image=np.ones((480,640,3),np.uint8)*37
        packed,t=pack_image(image,'left',(736,1280))
        self.assertEqual((480,640),t.resized_hw)
        self.assertTrue((packed[128:608,320:960]==image).all())
        self.assertEqual(Detection(0,0,640,480,'dog',.8),t.box(Detection(320,128,960,608,'dog',.8)))
        self.assertTrue((image==37).all())

    def test_partial_views_and_order(self):
        self.assertEqual([['right','left'],['front']],batch_groups(['right','front','left'],'2plus1'))
        self.assertEqual([['front'],['right']],batch_groups(['front','right'],'2plus1'))
        self.assertEqual([],batch_groups([],'batch3'))
        for views,mode in [(['front','front'],'batch3'),(['extra'],'sequential'),(['front'],'unknown')]:
            with self.assertRaises(ValueError):batch_groups(views,mode)

    def test_prompt_groups_are_explicit_and_baseline_retained(self):
        root=Path(__file__).resolve().parents[2]
        g=load_class_groups(yaml.safe_load((root/'config/camera/object_classes_yoloe_l_v1.yaml').read_text())['class_groups'])
        self.assertEqual('animal',g.group_of('animal'));self.assertEqual('vehicle',g.group_of('scooter'))
        c=yaml.safe_load((root.parents[1]/'config/experiments/camera_yoloe_l_first_v1.yaml').read_text())
        self.assertEqual(27,len(c['prompt_profiles']['base27']));self.assertEqual(42,len(c['prompt_profiles']['rich42']))
        self.assertTrue(set(c['prompt_profiles']['base27'])<=set(c['prompt_profiles']['rich42']))

    def fixture(self):
        backend=Mock();g=load_class_groups({'groups':{'person':['person'],'vehicle':['car']}})
        processors={v:ObjectFrameProcessor(backend,g) for v in ['front','left']}
        image=np.zeros((48,64,3),np.uint8);_,jpeg=cv2.imencode('.jpg',image)
        frames=[SimpleNamespace(view=v,header=Header(stamp=Time(42),frame_id='camera_'+v),data=jpeg.tobytes()) for v in processors]
        backend.infer_images.side_effect=lambda images,order:{v:SegmentationFrame([],[],{}) for v in order}
        return backend,processors,frames

    def test_invalid_jpeg_does_not_drop_other_view_or_change_header(self):
        backend,processors,frames=self.fixture();frames[0].data=b'broken'
        rows=process_batch(frames,processors,backend,True)
        self.assertFalse(rows[0][1].valid);self.assertEqual('decode',rows[0][-1][0])
        self.assertTrue(rows[1][1].valid);self.assertEqual(frames[1].header,rows[1][1].header)
        self.assertEqual((64,48),(rows[1][1].image_width,rows[1][1].image_height))
        self.assertEqual(['left'],backend.infer_images.call_args.args[1])

    def test_batch_failure_is_invalid_for_each_original_observation(self):
        backend,processors,frames=self.fixture();backend.infer_images.side_effect=RuntimeError('backend failure')
        for frame,msg,raw,timings,error in process_batch(frames,processors,backend):
            self.assertFalse(msg.valid);self.assertEqual(frame.header,msg.header)
            self.assertEqual((64,48),(msg.image_width,msg.image_height));self.assertEqual('inference',error[0])

    def test_box_mask_count_mismatch_is_not_silently_truncated(self):
        backend,processors,frames=self.fixture()
        backend.infer_images.side_effect=lambda images,order:{v:SegmentationFrame([Detection(1,2,3,4,'car',.9)],[],{}) for v in order}
        rows=process_batch(frames,processors,backend)
        self.assertTrue(all(not r[1].valid for r in rows))


if __name__=='__main__':unittest.main()
