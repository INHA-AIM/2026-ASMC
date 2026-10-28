#!/usr/bin/env python3
"""An inactive jamming controller cannot consume an unexecuted lane change."""
import importlib.util
from pathlib import Path
import time
import unittest

import rospy
import rostest
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import UInt8

spec = importlib.util.spec_from_file_location(
    'legacy_lane_test', str(Path(__file__).with_name('test_camera_bbox_lane_change.py')))
legacy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(legacy)


class InactiveLifecycleTest(unittest.TestCase):
    def wait_mode(self, helper, active):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if helper.diag is not None and helper.status()[1].get('mode_active') == active:
                return
            time.sleep(.03)
        self.fail('jamming node did not enter mode_active=' + active)

    def test_inactive_longer_than_pause_timeout(self):
        legacy.LaneChangeTest.setUpClass()
        helper = legacy.LaneChangeTest('runTest')
        helper.setUp()
        commands = []
        subscriber = rospy.Subscriber('/bbox_change_test/command', CtrlCmd,
                                      commands.append, queue_size=100)
        try:
            helper.feed()
            helper.feed([0])
            self.assertEqual(helper.status()[1]['change_state'], 'changing')
            helper.mode_pub.publish(UInt8(0))
            self.wait_mode(helper, 'false')
            helper.feed([0], duration=.55)
            inactive = helper.status()[1]
            self.assertLess(float(inactive['source_age_sec']), .5)
            self.assertEqual(inactive['change_state'], 'changing')
            count = len(commands)
            # Existing pause timeout is 30 s; this interval exceeds it.
            time.sleep(30.5)
            after = helper.status()[1]
            self.assertEqual(after['change_state'], inactive['change_state'])
            self.assertEqual(after['change_completed'], inactive['change_completed'])
            self.assertEqual(after['change_drive_time_sec'],
                             inactive['change_drive_time_sec'])
            self.assertEqual(after['change_pause_time_sec'],
                             inactive['change_pause_time_sec'])
            self.assertEqual(len(commands), count)
            self.assertNotIn('restart_required', helper.status()[0])
            helper.mode_pub.publish(UInt8(1))
            self.wait_mode(helper, 'true')
            helper.feed([], duration=.8)
            active = helper.status()[1]
            self.assertEqual(active['change_state'], 'following_lane')
            self.assertEqual(active['change_completed'], 'false')
            self.assertEqual(active['change_drive_time_sec'], '0.000000')
            self.assertEqual(active['change_pause_time_sec'], '0.000000')
            helper.following()
        finally:
            subscriber.unregister()
            helper.tearDown()

    def test_quick_switch_discards_unexecuted_change(self):
        legacy.LaneChangeTest.setUpClass()
        helper = legacy.LaneChangeTest('runTest')
        helper.setUp()
        try:
            helper.feed()
            helper.feed([0])
            self.assertEqual(helper.status()[1]['change_state'], 'changing')
            helper.mode_pub.publish(UInt8(0))
            self.wait_mode(helper, 'false')
            helper.mode_pub.publish(UInt8(1))
            self.wait_mode(helper, 'true')
            reset = helper.status()[1]
            self.assertEqual(reset['change_state'], 'following_lane')
            self.assertEqual(reset['target_lane'], '-1')
            self.assertEqual(reset['locked_lane'], '-1')
            self.assertEqual(reset['bbox_valid'], 'false')
            helper.feed([], duration=.8)
            helper.following()
        finally:
            helper.tearDown()


if __name__ == '__main__':
    rospy.init_node('camera_bbox_inactive_lifecycle_test')
    rostest.rosrun('lane_boundary_adapter', 'camera_bbox_inactive_lifecycle',
                   InactiveLifecycleTest)
