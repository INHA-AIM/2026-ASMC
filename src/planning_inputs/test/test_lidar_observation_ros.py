#!/usr/bin/env python3
"""Exercise actual adapter transport, rejection and fresh recovery on isolated ROS."""
import time
import unittest
import rospy
import rostest
from asmc_msgs.msg import LidarPlanningObservation, LidarObject
from planning_inputs.msg import DetectedObjectArray


class ObservationTest(unittest.TestCase):
    def test_invalid_stale_and_duplicate_never_publish_clear_road(self):
        received = []
        sub = rospy.Subscriber('/planning/objects', DetectedObjectArray, received.append)
        pub = rospy.Publisher('/perception/lidar/planning_observations_map', LidarPlanningObservation, queue_size=2)
        deadline = time.monotonic() + 5
        while not pub.get_num_connections() and time.monotonic() < deadline: time.sleep(.02)
        self.assertEqual(pub.get_num_connections(), 1)
        time.sleep(.1)
        def packet(seq, status, age=0.):
            p = LidarPlanningObservation()
            o = p.observation
            o.producer_session_id = 'integration'; o.config_id = 'config'; o.calibration_id = 'calibration'
            o.scan_sequence = seq; o.generation = 1
            o.objects.header.frame_id = 'map'
            o.objects.header.stamp = rospy.Time.now() - rospy.Duration(age)
            o.objects.status = status
            return p
        p = packet(1, 2)  # enum looked up explicitly below
        p.observation.objects.status = p.observation.objects.OBSERVED_EMPTY
        pub.publish(p); time.sleep(.15)
        self.assertEqual(len(received), 1)
        self.assertEqual(received[0].header.stamp, p.observation.objects.header.stamp)
        self.assertEqual(received[0].header.frame_id, 'map')
        pub.publish(p); time.sleep(.1)
        invalid = packet(2, p.observation.objects.POSE_UNAVAILABLE)
        pub.publish(invalid); time.sleep(.1)
        stale = packet(3, p.observation.objects.OBSERVED_EMPTY, 1.)
        pub.publish(stale); time.sleep(.1)
        self.assertEqual(len(received), 1)
        recovered = packet(4, p.observation.objects.OBSERVED_EMPTY)
        pub.publish(recovered); time.sleep(.15)
        self.assertEqual(len(received), 2)
        self.assertEqual(received[-1].header.stamp, recovered.observation.objects.header.stamp)
        self.assertEqual(received[-1].header.frame_id, 'map')
        newborn = packet(5, p.observation.objects.OBSERVED)
        b = LidarObject(track_id=7, observation_state=LidarObject.OBSERVED,
                        length=4., width=2., height=1., classification='vehicle', camera_class='truck')
        b.center.x=100.; b.center.y=100.; b.center.z=.5
        for i in range(8):
            b.obb_corners[i].x=102. if i & 1 else 98.
            b.obb_corners[i].y=101. if i & 2 else 99.
            b.obb_corners[i].z=1. if i & 4 else 0.
        newborn.observation.objects.objects=[b]; newborn.motion_states=[newborn.UNKNOWN]
        pub.publish(newborn); time.sleep(.15)
        self.assertEqual(len(received), 3)
        self.assertEqual(len(received[-1].objects), 1)
        self.assertTrue(received[-1].objects[0].velocity_unavailable)
        self.assertEqual(received[-1].objects[0].classification, 'vehicle')
        self.assertEqual(received[-1].objects[0].camera_class, 'truck')
        self.assertEqual(received[-1].producer_session_id, 'integration')
        self.assertEqual(received[-1].scan_sequence, 5)
        self.assertEqual(received[-1].generation, 1)
        self.assertEqual(received[-1].config_id, 'config')
        self.assertEqual(received[-1].calibration_id, 'calibration')
        self.assertEqual(received[-1].objects[0].motion_state, newborn.UNKNOWN)
        sub.unregister()


if __name__ == '__main__':
    rospy.init_node('lidar_observation_integration')
    rostest.rosrun('planning_inputs', 'lidar_observation_ros', ObservationTest)
