#!/usr/bin/env python3
import time
import unittest

import rospy
import rostest
from morai_msgs.msg import GPSMessage
from std_msgs.msg import UInt8


class GpsModeSourceTest(unittest.TestCase):
    def setUp(self):
        self.modes = []
        self.sub = rospy.Subscriber('/planning/mode', UInt8,
                                    lambda msg: self.modes.append(msg.data),
                                    queue_size=10)
        self.gps = rospy.Publisher('/gps', GPSMessage, queue_size=1)
        deadline = time.monotonic() + 5
        while (not self.modes or not self.gps.get_num_connections()) and time.monotonic() < deadline:
            time.sleep(.02)
        self.assertEqual(0, self.modes[-1])
        self.assertTrue(self.gps.get_num_connections())

    def publish_until(self, gps, expected):
        prior = len(self.modes)
        deadline = time.monotonic() + 3
        while len(self.modes) == prior and time.monotonic() < deadline:
            self.gps.publish(gps)
            time.sleep(.05)
        self.assertGreater(len(self.modes), prior)
        self.assertEqual(expected, self.modes[-1])

    def test_normal_jamming_recovery(self):
        nonzero = GPSMessage(latitude=37.0, longitude=126.0, altitude=12.0)
        self.gps.publish(nonzero)
        time.sleep(.1)
        self.assertEqual(0, self.modes[-1])
        self.publish_until(GPSMessage(latitude=0.0, longitude=0.0, altitude=0.0), 1)
        self.publish_until(nonzero, 0)


if __name__ == '__main__':
    rospy.init_node('gps_mode_source_test')
    rostest.rosrun('asmc_planner', 'gps_mode_source_test', GpsModeSourceTest)
