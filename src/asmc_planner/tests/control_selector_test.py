#!/usr/bin/env python3
import io
import time
import unittest

import rospy
import rostest
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import UInt8
from rosgraph.masterapi import Master


class ControlSelectorTest(unittest.TestCase):
    def setUp(self):
        self.received = []
        self.output = rospy.Subscriber('/ctrl_cmd', CtrlCmd, self.received.append,
                                       queue_size=100)
        self.mode = rospy.Publisher('/planning/mode', UInt8, queue_size=1,
                                    latch=True)
        self.normal = rospy.Publisher('/ctrl_cmd/normal', CtrlCmd, queue_size=10)
        self.jamming = rospy.Publisher('/ctrl_cmd/jamming', CtrlCmd, queue_size=10)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if (self.mode.get_num_connections() and self.normal.get_num_connections()
                    and self.jamming.get_num_connections()):
                break
            rospy.sleep(0.02)
        self.assertTrue(self.mode.get_num_connections())
        self.assertTrue(self.normal.get_num_connections())
        self.assertTrue(self.jamming.get_num_connections())

    def tearDown(self):
        for endpoint in [self.output, self.mode, self.normal, self.jamming]:
            endpoint.unregister()

    def assert_forwarded(self, publisher, command):
        before = len(self.received)
        expected = io.BytesIO()
        command.serialize(expected)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            publisher.publish(command)
            rospy.sleep(0.05)
            for received in self.received[before:]:
                actual = io.BytesIO()
                received.serialize(actual)
                if expected.getvalue() == actual.getvalue():
                    return
        self.fail('selected command was not forwarded')

    def assert_blocked(self, publisher, command):
        before = len(self.received)
        for _ in range(4):
            publisher.publish(command)
            rospy.sleep(0.05)
        for actual in self.received[before:]:
            # The watchdog may publish STOP, but never the unselected command.
            self.assertEqual(actual.longlCmdType, 1)
            self.assertEqual(actual.accel, 0.0)
            self.assertEqual(actual.brake, 1.0)

    def test_switch_and_ownership(self):
        normal = CtrlCmd(longlCmdType=1, accel=0.17, brake=0.0,
                         steering=-0.23, velocity=0.0, acceleration=0.0)
        jamming = CtrlCmd(longlCmdType=2, accel=0.0, brake=0.0,
                          steering=0.34, velocity=15.0, acceleration=0.0)
        publishers = dict(Master(rospy.get_name()).getSystemState()[0])
        self.assertEqual(['/asmc_control_selector'], publishers.get('/ctrl_cmd'))
        self.mode.publish(UInt8(0))
        rospy.sleep(0.1)
        self.assert_forwarded(self.normal, normal)
        self.assert_blocked(self.jamming, jamming)
        self.mode.publish(UInt8(1))
        rospy.sleep(0.1)
        self.assert_blocked(self.normal, normal)
        self.assert_forwarded(self.jamming, jamming)
        self.mode.publish(UInt8(0))
        rospy.sleep(0.1)
        self.assert_blocked(self.jamming, jamming)
        self.assert_forwarded(self.normal, normal)

    def test_selected_source_timeout_and_recovery(self):
        self.mode.publish(UInt8(0))
        rospy.sleep(0.1)
        drive = CtrlCmd(longlCmdType=1, accel=0.2)
        self.assert_forwarded(self.normal, drive)
        before = len(self.received)
        # Unselected commands cannot keep an old NORMAL throttle alive.
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            self.jamming.publish(drive)
            rospy.sleep(0.03)
        self.assertGreater(len(self.received), before)
        self.assertEqual(self.received[-1].brake, 1.0)
        self.assertEqual(self.received[-1].accel, 0.0)
        self.assert_forwarded(self.normal, drive)

    def test_switch_requires_fresh_selected_command(self):
        drive = CtrlCmd(longlCmdType=1, accel=0.2)
        self.mode.publish(UInt8(1))
        rospy.sleep(0.1)
        self.assert_forwarded(self.jamming, drive)
        self.mode.publish(UInt8(0))
        rospy.sleep(0.1)
        self.assertEqual(self.received[-1].brake, 1.0)
        self.assert_forwarded(self.normal, drive)
        self.mode.publish(UInt8(9))
        rospy.sleep(0.1)
        self.assertEqual(self.received[-1].brake, 1.0)


if __name__ == '__main__':
    rospy.init_node('control_selector_test')
    rostest.rosrun('asmc_planner', 'control_selector_test', ControlSelectorTest)
