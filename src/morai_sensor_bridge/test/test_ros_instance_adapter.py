#!/usr/bin/env python3

import importlib.util
import os
import unittest

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField


SCRIPT = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__), "..", "scripts", "ros_instance_lidar_adapter.py"
    )
)
SPEC = importlib.util.spec_from_file_location("ros_instance_lidar_adapter", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class PublisherStub(object):
    def __init__(self):
        self.message = None

    def publish(self, message):
        self.message = message


class RosInstanceAdapterTest(unittest.TestCase):
    def test_float_intensity_is_normalized_to_uint32_instance_id(self):
        values = np.zeros(
            3,
            dtype=[
                ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                ("intensity", "<f4"),
            ],
        )
        values["x"] = [1.0, 2.0, 3.0]
        values["intensity"] = [0.0, 7.0, 255.0]
        message = PointCloud2()
        message.header.stamp = rospy.Time(10, 20)
        message.height = 1
        message.width = 3
        message.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
        ]
        message.point_step = 16
        message.row_step = 48
        message.data = values.tobytes()

        adapter = MODULE.RosInstanceLidarAdapter.__new__(
            MODULE.RosInstanceLidarAdapter
        )
        adapter.publisher = PublisherStub()
        adapter.converted = 0
        adapter.rejected = 0
        adapter.callback(message)

        result = adapter.publisher.message
        self.assertIsNotNone(result)
        self.assertEqual(result.header.stamp, message.header.stamp)
        self.assertEqual(result.fields[-1].name, "instance_id")
        self.assertEqual(result.fields[-1].datatype, PointField.UINT32)
        decoded = np.frombuffer(
            result.data,
            dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("id", "<u4")],
        )
        np.testing.assert_array_equal(decoded["id"], [0, 7, 255])
        self.assertEqual(adapter.converted, 1)
        self.assertEqual(adapter.rejected, 0)


if __name__ == "__main__":
    unittest.main()
