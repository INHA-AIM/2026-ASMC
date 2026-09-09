#!/usr/bin/env python3

import importlib.util
import os
import unittest
from unittest import mock


SCRIPT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts", "morai_udp_bridge.py")
)
SPEC = importlib.util.spec_from_file_location("morai_udp_bridge_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SourceTimestampTest(unittest.TestCase):
    def bridge(self):
        value = MODULE.MoraiUdpBridge.__new__(MODULE.MoraiUdpBridge)
        value.timestamp_mode = "source"
        value.timestamp_fallbacks = {"ego": 0}
        value.timestamp_regressions = {"ego": 0}
        value._last_source_ns = {}
        return value

    def test_valid_packet_stamp_is_preserved(self):
        bridge = self.bridge()
        stamp = bridge.packet_stamp(123, 456, "ego")
        self.assertEqual(stamp.secs, 123)
        self.assertEqual(stamp.nsecs, 456)
        self.assertEqual(bridge.timestamp_fallbacks["ego"], 0)

    def test_regression_falls_back_and_is_counted(self):
        bridge = self.bridge()
        bridge.packet_stamp(123, 456, "ego")
        with mock.patch.object(
            MODULE.rospy.Time, "now", return_value=MODULE.rospy.Time(999, 1)
        ):
            stamp = bridge.packet_stamp(122, 456, "ego")
        self.assertEqual(stamp, MODULE.rospy.Time(999, 1))
        self.assertEqual(bridge.timestamp_fallbacks["ego"], 1)
        self.assertEqual(bridge.timestamp_regressions["ego"], 1)


if __name__ == "__main__":
    unittest.main()
