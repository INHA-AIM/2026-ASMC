#!/usr/bin/env python3
"""Real classifier input boundary: local OBB support, never empty-as-free."""
import time
import copy
import unittest
import numpy as np
import rospy
import rostest
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from asmc_msgs.msg import LidarTrackingObservation,LidarObject
from test_camera_bbox_lane import BboxLaneTest,camera_box

class LocalContactTest(unittest.TestCase):
    setUpClass=classmethod(BboxLaneTest.setUpClass.__func__)
    feed=BboxLaneTest.feed
    def test_mask_requires_fresh_unique_published_observation(self):
        self.pub=rospy.Publisher('/perception/lidar/tracks_local',LidarTrackingObservation,queue_size=1)
        o=LidarObject(track_id=8,observation_state=1,source_point_count=20,length=.5,width=.4,height=.85,
                      camera_class='car',classification='vehicle')
        xyz=np.array([[x,y,z] for z in (-.35,.5) for x,y in ((5.2,-2.),(5.7,-2.),(5.7,-1.6),(5.2,-1.6))])
        o.obb_corners=[Point(*p) for p in xyz]
        optical=self.camera.ground_to_optical(xyz);uv=(self.camera.intrinsic@optical.T).T;uv=uv[:,:2]/uv[:,2:]
        box=camera_box(*uv.min(axis=0),*uv.max(axis=0),'car')
        self.objects=[o];self.sequence=0
        def publish(_):
            m=LidarTrackingObservation();a=m.observation;a.producer_session_id='local-fixture';a.generation=1
            self.sequence+=1;a.scan_sequence=self.sequence;a.config_id='fixture-config';a.calibration_id='fixture-calibration'
            a.objects.header=Header(stamp=rospy.Time.now(),frame_id='base_link');a.objects.status=1 if self.objects else 2
            a.objects.objects=self.objects;self.pub.publish(m)
        timer=rospy.Timer(rospy.Duration(.025),publish)
        try:
            self.feed([box],duration=.8)
            self.assertTrue(self.result.local_obstacles_valid)
            self.assertEqual(list(self.result.ground_contact_sources),['lidar_observed'])
            self.assertTrue(self.result.ground_contact_valid[0])
            self.assertEqual(len(self.result.local_obstacle_footprints),1)
            # Two projected candidates: geometry stays available, image identity
            # is ambiguous and cannot authorize masked-contact continuation.
            self.objects=[o,copy.deepcopy(o)];self.objects[1].track_id=9
            self.feed([box],duration=.5)
            self.assertFalse(self.result.ground_contact_valid[0])
            self.objects=[];self.feed([box],duration=.5)
            self.assertTrue(self.result.local_obstacles_valid)
            self.assertFalse(self.result.ground_contact_valid[0])
            self.objects=[o];self.feed([box],duration=.4)
            timer.shutdown();self.feed([box],duration=.5)
            self.assertFalse(self.result.local_obstacles_valid)
            self.assertFalse(self.result.ground_contact_valid[0])
        finally:timer.shutdown()

if __name__=='__main__':rostest.rosrun('lane_boundary_adapter','local_contact_integration',LocalContactTest)
