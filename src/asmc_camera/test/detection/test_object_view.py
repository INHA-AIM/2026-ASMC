from collections import Counter
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import cv2
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage
from asmc_msgs.msg import CameraObjectBoxArray, CameraObjectObservation
from asmc_camera.detection.hood_mask import HoodMask
from asmc_camera.detection.view_cache import ObjectViewCache,VIEWS
from asmc_camera.nodes.object_view_node import ObjectViewNode


def inputs(view='front',stamp=10000000000,session='one',generation=2,valid=True):
    header=Header(stamp=rospy.Time(stamp//1000000000,stamp%1000000000),frame_id='camera_'+view)
    image=np.full((96,128,3),(30,80,120),np.uint8)
    jpeg=CompressedImage(header=header,format='jpeg',data=cv2.imencode('.jpg',image)[1].tobytes())
    d=CameraObjectBoxArray(header=header,valid=valid,image_width=128,image_height=96)
    o=CameraObjectObservation(detections=d,producer_session_id=session,generation=generation,
        calibration_id='cal-'+view,model_id='model',config_id='config')
    return jpeg,o


class ViewCacheTest(unittest.TestCase):
    def test_exact_camera_stamp_and_bounded_expiring_images(self):
        c=ObjectViewCache();image,o=inputs();c.image('front',image,20);c.observe(o,20)
        self.assertIsNotNone(c.pair('front',20.1));self.assertIsNone(c.pair('left',20.1))
        for i in range(1,20):c.image('front',inputs(stamp=10000000000+i)[0],20.1)
        self.assertEqual(len(c.images['front']),8);self.assertIsNone(c.pair('front',20.1))
        c.expire(20.61);self.assertFalse(c.images['front'])
        image.header.frame_id='camera_left';self.assertFalse(c.image('front',image,21))

    def test_restart_generation_invalid_empty_and_calibration(self):
        c=ObjectViewCache();image,o=inputs();c.image('front',image,20);c.observe(o,20)
        epoch=c.epoch;_,new=inputs(session='two');c.observe(new,20)
        self.assertGreater(c.epoch,epoch);self.assertIsNone(c.pair('front',20))
        self.assertFalse(c.observe(o,20.1))
        c.image('front',image,20.1);self.assertIsNotNone(c.pair('front',20.1))
        _,old=inputs(session='two',generation=1);self.assertFalse(c.observe(old,20.1))
        _,invalid=inputs(session='two',valid=False);c.observe(invalid,20.1)
        self.assertIsNone(c.pair('front',20.1))
        image,empty=inputs(stamp=10000000001,session='two');c.image('front',image,20.2);c.observe(empty,20.2)
        self.assertEqual(c.pair('front',20.2)[1].detections.boxes,[])
        empty.calibration_id='other';c.observe(empty,20.2);self.assertIsNone(c.pair('front',20.2))
        c.control(NS(generation=3,mode=0,managed=True));self.assertFalse(c.observations)
        c.control(NS(generation=4,mode=2,managed=True))
        self.assertFalse(c.observe(empty,20.3))

    def test_same_generation_config_conflict_blocks_until_new_generation(self):
        c=ObjectViewCache();image,o=inputs();c.image('front',image,20);c.observe(o,20)
        _,changed=inputs();changed.config_id='unexpected';self.assertFalse(c.observe(changed,20))
        self.assertIsNone(c.pair('front',20));self.assertFalse(c.observe(o,20.1))
        image,new=inputs(generation=3);c.observe(new,20.2);c.image('front',image,20.2)
        self.assertIsNotNone(c.pair('front',20.2))


class ObjectViewTest(unittest.TestCase):
    def node(self):
        n=ObjectViewNode.__new__(ObjectViewNode)
        n.cache,n.rendered,n.failed_keys,n.last_headers=ObjectViewCache(),{},{},{}
        n.lock,n.stop=threading.RLock(),threading.Event()
        n.ttl,n.width,n.height,n.quality=.5,128,96,80
        n.masks={v:HoodMask.load(False,'') for v in VIEWS}
        n.profiles={v:dict(enabled=False,config_sha256='') for v in VIEWS}
        n.counts=Counter();n.last_mosaic_key=None;n.last_individual_keys={}
        n.render_by_view,n.last_fresh_views=Counter(),[]
        n.publishers={v:Mock() for v in VIEWS};n.mosaic=Mock()
        for p in n.publishers.values():p.get_num_connections.return_value=0
        n.mosaic.get_num_connections.return_value=1
        n._subscriptions=Mock()
        for v in VIEWS:
            image,o=inputs(v);n.cache.image(v,image,20);n.cache.observe(o,20)
        return n

    def test_mosaic_decodes_each_selected_source_once_and_has_no_intermediate_jpeg(self):
        n=self.node()
        with patch('time.monotonic',return_value=20.1),patch('rospy.Time.now',return_value=rospy.Time(10,100000000)):
            n._tick();n._tick()
            self.assertEqual(n.counts['decode'],3);self.assertEqual(n.counts['mosaic_encode'],1)
            self.assertEqual(n.counts['individual_encode'],0)
            n.publishers['front'].get_num_connections.return_value=1;n._tick()
            self.assertEqual(n.counts['decode'],3);self.assertEqual(n.counts['individual_encode'],1)
            self.assertEqual(n.publishers['front'].publish.call_args[0][0].header.frame_id,'camera_front')

    def test_no_consumer_or_mask_mismatch_does_no_decode(self):
        n=self.node();n.mosaic.get_num_connections.return_value=0
        n._tick();self.assertEqual(n.counts['decode'],0)
        n.mosaic.get_num_connections.return_value=1;n.profiles={}
        with patch('time.monotonic',return_value=20.1),patch('rospy.Time.now',return_value=rospy.Time(10,100000000)):
            n._tick();self.assertEqual(n.counts['decode'],0)

    def test_mode_change_while_encoding_withholds_old_boxes(self):
        n=self.node();encode=n._encode
        def changed(*args):
            message=encode(*args);n.cache.control(NS(generation=3,mode=0,managed=True));return message
        n._encode=changed
        with patch('time.monotonic',return_value=20.1),patch('rospy.Time.now',return_value=rospy.Time(10,100000000)):
            n._tick();n.mosaic.publish.assert_not_called()
            self.assertEqual(n.counts['invalidated_render'],1)

    def test_corrupt_source_is_not_redecoded_and_stale_panels_are_blank(self):
        n=self.node();n.cache.images['front'][10000000000][0].data=b'broken'
        with patch('time.monotonic',return_value=20.1),patch('rospy.Time.now',return_value=rospy.Time(10,100000000)):
            n._tick();n._tick();self.assertEqual(n.counts['decode'],3)
        with patch('time.monotonic',return_value=20.7),patch('rospy.Time.now',return_value=rospy.Time(10,700000000)):
            n._tick()
        m=n.mosaic.publish.call_args[0][0];image=cv2.imdecode(np.frombuffer(m.data,np.uint8),cv2.IMREAD_COLOR)
        self.assertLess(image[70:].max(),10)


if __name__=='__main__':unittest.main()
