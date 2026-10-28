#!/usr/bin/env python3
import time
import unittest

import rospy
import rostest
from morai_msgs.msg import GPSMessage
from std_msgs.msg import Bool, UInt8
from asmc_msgs.msg import FrenetFeedback


class GpsModeSourceTest(unittest.TestCase):
    def setUp(self):
        self.modes = []
        self.sub = rospy.Subscriber('/planning/mode', UInt8,
                                    lambda msg: self.modes.append(msg.data),
                                    queue_size=10)
        self.gps_return = []
        self.gps_return_sub = rospy.Subscriber('/planning/mode/gps_return', Bool,
                                               lambda msg: self.gps_return.append(msg.data),
                                               queue_size=10)
        self.gps = rospy.Publisher('/gps', GPSMessage, queue_size=1)
        self.feedback = rospy.Publisher('/planning/frenet_feedback_v2', FrenetFeedback, queue_size=1)
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
        nonzero = GPSMessage(latitude=37.0, longitude=126.0, altitude=12.0, status=1)
        self.gps.publish(nonzero)
        time.sleep(.1)
        self.assertEqual(0, self.modes[-1])
        self.publish_until(GPSMessage(latitude=0.0, longitude=0.0, altitude=0.0), 1)
        self.gps.publish(nonzero)
        time.sleep(.1)
        self.assertEqual(1, self.modes[-1])
        self.assertTrue(self.gps_return[-1])  # still JAMMING, cause cleared: steer back now
        self.ready()
        self.assertEqual(1, self.modes[-1])
        self.ready()
        self.assertEqual(0, self.modes[-1])
        time.sleep(.1)
        self.assertFalse(self.gps_return[-1])

    def ready(self, stamp=None, valid=True, source=FrenetFeedback.TRAJECTORY_NEW):
        feedback = FrenetFeedback(command_accepted=True, plan_valid=valid,
                                  trajectory_source=source)
        feedback.header.frame_id = 'map'
        feedback.header.stamp = stamp if stamp is not None else rospy.Time.now()
        self.feedback.publish(feedback)
        time.sleep(.07)
        return feedback.header.stamp

    def recover(self):
        self.publish_until(GPSMessage(), 1)
        self.gps.publish(GPSMessage(latitude=37., longitude=126., altitude=12., status=1))
        time.sleep(.1)

    def test_cached_stale_duplicate_and_failed_plans_cannot_handoff(self):
        self.recover()
        self.ready(stamp=rospy.Time.now()-rospy.Duration(1.))
        self.ready(source=FrenetFeedback.TRAJECTORY_REUSED)
        self.assertEqual(1, self.modes[-1])
        stamp=self.ready()
        self.ready(stamp=stamp)
        self.assertEqual(1, self.modes[-1])
        self.ready(valid=False)
        self.ready()
        self.assertEqual(1, self.modes[-1])
        self.ready()
        self.assertEqual(0, self.modes[-1])

    def test_rejamming_cancels_pending_handoff(self):
        self.recover()
        self.ready()
        self.gps.publish(GPSMessage())
        time.sleep(.1)
        self.ready(); self.ready()
        self.assertEqual(1, self.modes[-1])
        self.gps.publish(GPSMessage(latitude=37., longitude=126., altitude=12., status=1))
        time.sleep(.1)
        self.ready(); self.ready()
        self.assertEqual(0, self.modes[-1])

    def test_unready_recovery_has_bounded_deadline(self):
        self.recover()
        time.sleep(.3)
        self.assertEqual(1, self.modes[-1])
        time.sleep(3.9)  # recovery_timeout_s default 4 s
        self.assertEqual(0, self.modes[-1])


if __name__ == '__main__':
    rospy.init_node('gps_mode_source_test')
    rostest.rosrun('asmc_planner', 'gps_mode_source_test', GpsModeSourceTest)
