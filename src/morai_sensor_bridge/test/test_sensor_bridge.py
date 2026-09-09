#!/usr/bin/env python3

import importlib.util
import os
import struct
import unittest

import numpy as np
import rospy
from sensor_msgs.msg import PointField


SCRIPT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts", "morai_sensor_bridge.py")
)
SPEC = importlib.util.spec_from_file_location("morai_sensor_bridge_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SensorBridgeProtocolTest(unittest.TestCase):
    def test_camera_packet_uses_reported_payload_size(self):
        payload = b"jpeg"
        padded = payload + b"\x00" * 10
        packet = b"MOR" + struct.pack("<IIii", 10, 20, 2, len(payload)) \
            + padded + b"EI"
        sec, nsec, index, parsed, final = MODULE.parse_camera_packet(packet)
        self.assertEqual((sec, nsec, index), (10, 20, 2))
        self.assertEqual(parsed, payload)
        self.assertTrue(final)

    def test_instance_cloud_has_uint32_fourth_field(self):
        bridge = MODULE.SensorBridge.__new__(MODULE.SensorBridge)
        bridge.frame_lidar = "velodyne"
        values = np.array([0, 7, 255], dtype=np.uint8)
        cloud = bridge._make_cloud(
            np.array([1, 2, 3], dtype=np.float32),
            np.array([4, 5, 6], dtype=np.float32),
            np.array([7, 8, 9], dtype=np.float32),
            values,
            rospy.Time(10, 20),
            "instance_id",
        )
        self.assertEqual(cloud.point_step, 16)
        self.assertEqual(cloud.fields[-1].name, "instance_id")
        self.assertEqual(cloud.fields[-1].datatype, PointField.UINT32)
        decoded = np.frombuffer(
            cloud.data,
            dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("id", "<u4")],
        )
        np.testing.assert_array_equal(decoded["id"], [0, 7, 255])


if __name__ == "__main__":
    unittest.main()
