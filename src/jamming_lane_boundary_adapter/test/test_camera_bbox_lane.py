#!/usr/bin/env python3
"""Private ROS master: real YOLO message -> exact image bridge -> C++ classifier."""
import importlib.util
import time
import unittest
from pathlib import Path

import cv2
import numpy as np
import rospy
import rospkg
import rostest
from asmc_msgs.msg import LaneBoundaryFamily
from asmc_msgs.msg import CameraObjectBoxArray
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header
from lane_boundary_adapter.msg import CameraBBoxLaneResult as Result
from asmc_camera.bev.calibration import load_calibration_snapshot

spec = importlib.util.spec_from_file_location('fixture', str(Path(__file__).with_name('lane_family_fixture.py')))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
camera_box = fixture.camera_box


class BboxLaneTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('bbox_lane_test')
        cls.camera = load_calibration_snapshot(Path(rospkg.RosPack().get_path('asmc_camera'))/'config/bev/cameras_v2.yaml')[1]['front']
        cls.result = cls.overlay = None
        cls.lane_pub = rospy.Publisher('/perception/lane/boundary_family', LaneBoundaryFamily, queue_size=1)
        cls.image_pub = rospy.Publisher('/bbox_test/image', CompressedImage, queue_size=1)
        cls.box_pub = rospy.Publisher('/bbox_test/yolo', CameraObjectBoxArray, queue_size=1)
        cls.sub = rospy.Subscriber('/planning/camera_bbox_lane/result', Result, lambda m: setattr(cls, 'result', m))
        cls.img_sub = rospy.Subscriber('/planning/camera_bbox_lane/image', Image, lambda m: setattr(cls, 'overlay', m))
        cls.jpeg = cv2.imencode('.jpg', np.zeros((720,1280,3),dtype=np.uint8))[1].tobytes()
        deadline=time.monotonic()+8
        while min(cls.lane_pub.get_num_connections(),cls.image_pub.get_num_connections(),cls.box_pub.get_num_connections())==0 and time.monotonic()<deadline:
            time.sleep(.03)

    @classmethod
    def box(cls,x,y,width=30):
        uv,_=cls.camera.project_ground(np.array([[x,y]]))
        u,v=uv[0]
        return fixture.camera_box(u-width/2,v-60,u+width/2,v,'car')

    def feed(self, boxes=(), duration=.5, shift=0., gap=False, camera=True, image=True, stamp=None, invalid_lane=False, stop_x=15.0, far_gap=False, uneven_end=False):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            source=rospy.Time.now() if stamp is None else stamp
            header=Header(stamp=source,frame_id='camera_front')
            if camera:
                lane=fixture.family(shift=shift,gap=gap,stop_x=stop_x)
                if far_gap:
                    for member in lane.boundaries:
                        for i,point in enumerate(member.polyline):
                            if 17.75 <= point.x <= 18.25: member.support_state[i]=0
                if uneven_end:
                    for member in lane.boundaries:
                        if member.role==3:
                            for i,point in enumerate(member.polyline):
                                if point.x>stop_x-.5:member.support_state[i]=0
                lane.header.stamp=source
                if invalid_lane:
                    lane.valid=False
                self.lane_pub.publish(lane)
            if image:
                self.image_pub.publish(CompressedImage(header=header,format='jpeg',data=self.jpeg))
            self.box_pub.publish(fixture.camera_boxes(header,boxes))
            time.sleep(.05)
        time.sleep(.025)
        self.assertIsNotNone(self.result)

    def test_classifies_lane_overlap_and_outside(self):
        boxes=[self.box(10,0),self.box(10,3.5),self.box(10,1.75),self.box(10,-4)]
        self.feed(boxes)
        self.assertTrue(self.result.valid,self.result.reason)
        self.assertEqual(self.result.own_lane,1)
        self.assertEqual(list(self.result.assignments),[Result.OWN,Result.ADJACENT,Result.BOTH,Result.OUTSIDE])
        self.assertEqual(self.result.own_state,Result.DETECTED)
        self.assertEqual(self.result.adjacent_state,Result.DETECTED)
        self.assertEqual(self.overlay.width,1280)
        # Ego in other strip: image left/right is not used as a fixed lane label.
        self.feed([self.box(10,0),self.box(10,-3.5)],shift=-3.5)
        self.assertEqual(self.result.own_lane,0)
        self.assertEqual(list(self.result.assignments),[Result.OWN,Result.ADJACENT])
        publishers,_,_=rospy.get_master().getSystemState()[2]
        self.assertNotIn('/ctrl_cmd',dict(publishers))

    def test_early_observed_horizon_preserves_unknown_gaps(self):
        self.feed([self.box(18,0)],stop_x=20.0)
        self.assertTrue(self.result.valid,self.result.reason)
        self.assertEqual(list(self.result.assignments),[Result.OWN])
        # Extending the consumer window does not invent missing camera support.
        self.feed([self.box(18,0)],stop_x=15.0)
        self.assertTrue(self.result.valid,self.result.reason)
        self.assertEqual(list(self.result.assignments),[Result.DEFERRED_FAR])
        self.assertEqual(self.result.own_state,Result.UNOBSERVED)
        self.feed([self.box(18,0)],stop_x=20.0,far_gap=True)
        self.assertFalse(self.result.valid)
        self.assertEqual(list(self.result.assignments),[Result.UNKNOWN])

    def test_far_approach_reaches_observed_lane_without_dead_band(self):
        for x in [18.0, 16.0, 15.5, 15.1, 15.01]:
            self.feed([self.box(x,0)],stop_x=15.0,duration=.25)
            self.assertTrue(self.result.valid,(x,self.result.reason))
            self.assertIn(self.result.assignments[0],[Result.DEFERRED_FAR,Result.OWN])
        self.feed([self.box(14.99,0)],stop_x=15.0)
        self.assertTrue(self.result.valid,self.result.reason)
        self.assertEqual(list(self.result.assignments),[Result.OWN])

    def test_contact_overlap_tolerates_far_end_pixel_jitter(self):
        original=self.box(15,0)
        for dy in [-5.,-3.,0.,3.]:
            box=fixture.camera_box(original.xmin,original.ymin+dy,original.xmax,original.ymax+dy,'car')
            self.feed([box],stop_x=15.,duration=.3)
            self.assertTrue(self.result.valid,(dy,self.result.reason))
            self.assertEqual(list(self.result.assignments),[Result.OWN])
            self.assertIn('contact_overlap:own=',self.result.box_reasons[0])

    def test_contact_overlap_handles_uneven_observed_lane_ends(self):
        self.feed([self.box(15,0)],stop_x=15.,uneven_end=True)
        self.assertTrue(self.result.valid,self.result)
        self.assertEqual(list(self.result.assignments),[Result.OWN])
        self.assertEqual(self.result.adjacent_state,Result.NO_DETECTION)
        # The same contact band must not bridge an interior unsupported gap.
        self.feed([self.box(18,0)],stop_x=20.,far_gap=True)
        self.assertFalse(self.result.valid)
        self.assertEqual(list(self.result.assignments),[Result.UNKNOWN])

    def test_small_contact_is_unknown_and_shared_contact_is_both(self):
        edge=self.box(10,-1.75)
        u=(edge.xmin+edge.xmax)/2
        self.feed([fixture.camera_box(u-1,edge.ymin,u+199,edge.ymax,'car')])
        self.assertFalse(self.result.valid,self.result)
        self.assertEqual(list(self.result.assignments),[Result.UNKNOWN])
        self.feed([self.box(10,1.75,width=100)])
        self.assertTrue(self.result.valid,self.result)
        self.assertEqual(list(self.result.assignments),[Result.BOTH])
        self.assertEqual(self.result.own_state,Result.DETECTED)
        self.assertEqual(self.result.adjacent_state,Result.DETECTED)

    def test_far_box_does_not_claim_either_lane_is_clear(self):
        self.feed([self.box(30,0)])
        self.assertTrue(self.result.valid,self.result.reason)
        self.assertEqual(list(self.result.assignments),[Result.DEFERRED_FAR])
        self.assertEqual(self.result.own_state,Result.UNOBSERVED)
        self.assertEqual(self.result.adjacent_state,Result.UNOBSERVED)
        self.assertEqual(list(self.result.box_reasons),['beyond_observed_lane_horizon'])
        self.feed([self.box(10,0),self.box(30,0)])
        self.assertTrue(self.result.valid,self.result.reason)
        self.assertEqual(self.result.own_state,Result.DETECTED)
        self.assertEqual(self.result.adjacent_state,Result.UNOBSERVED)

    def test_gap_far_and_clipped_are_unknown(self):
        for box,kwargs in [(self.box(6,0),{'gap':True}),(self.box(24,0),{'stop_x':10.0}),
                           (fixture.camera_box(610,270,690,320,'car'),{}),
                           (fixture.camera_box(600,600,700,720,'car'),{}),
                           (fixture.camera_box(float('nan'),500,700,600,'car'),{})]:
            self.feed([box],**kwargs)
            self.assertFalse(self.result.valid,self.result)
            self.assertEqual(list(self.result.assignments),[Result.UNKNOWN])
            self.assertEqual(self.result.own_state,Result.UNOBSERVED)

    def test_input_loss_and_duplicate_dont_mean_empty_lane(self):
        self.feed([])
        self.assertTrue(self.result.valid,self.result.reason)
        self.assertEqual(self.result.own_state,Result.NO_DETECTION)
        old=rospy.Time.now()
        self.feed([],duration=.7,stamp=old)
        self.assertFalse(self.result.valid)
        self.assertEqual(self.result.own_state,Result.UNOBSERVED)
        self.feed([self.box(10,0)])
        self.assertTrue(self.result.valid,self.result.reason)
        self.feed([self.box(10,0)],camera=False,duration=.7)
        self.assertFalse(self.result.valid)
        self.feed([self.box(10,0)])
        self.assertTrue(self.result.valid,self.result.reason)
        self.feed([self.box(10,0)],image=False,duration=.5)
        self.assertFalse(self.result.valid)
        self.assertIn(self.result.reason,['matching_image_missing','bbox_timeout'])
        self.feed([],invalid_lane=True)
        self.assertFalse(self.result.valid)

    def test_public_front_view_and_consumer_cutoff(self):
        box=self.box(10,0);box.confidence=.6
        self.feed([box]);self.assertTrue(self.result.valid)
        self.assertEqual(self.result.own_state,Result.NO_DETECTION)
        box.confidence=.9
        self.feed([box]);self.assertEqual(self.result.own_state,Result.DETECTED)
        header=Header(stamp=rospy.Time.now(),frame_id='camera_left')
        self.box_pub.publish(fixture.camera_boxes(header,[self.box(10,0)]))
        time.sleep(.18)
        self.assertTrue(self.result.valid,self.result.reason)
        self.assertEqual(self.result.own_state,Result.DETECTED)

    def test_resolution_and_validity_contract(self):
        self.feed()
        header=Header(stamp=rospy.Time.now(),frame_id='camera_front')
        self.image_pub.publish(CompressedImage(header=header,format='jpeg',data=cv2.imencode('.jpg',np.zeros((480,640,3),dtype=np.uint8))[1].tobytes()))
        self.box_pub.publish(fixture.camera_boxes(header))
        time.sleep(.15)
        self.assertFalse(self.result.valid)
        self.assertEqual(self.result.reason,'image_size_calibration_mismatch')
        self.feed()
        header.stamp=rospy.Time.now()
        self.image_pub.publish(CompressedImage(header=header,format='jpeg',data=self.jpeg))
        self.box_pub.publish(fixture.camera_boxes(header,valid=False))
        time.sleep(.15)
        self.assertFalse(self.result.valid)
        self.assertEqual(self.result.reason,'invalid_camera_detections')


if __name__=='__main__':
    rostest.rosrun('lane_boundary_adapter','camera_bbox_lane_integration',BboxLaneTest)
