#!/usr/bin/env python3
"""Synthetic bag contract for three-way LiDAR GT auditing."""

import importlib.util
import os
import struct
import tempfile
import unittest

import rosbag
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from data_collection.bag_run import missing_recorded_topics, recorded_topic_counts


SCRIPT = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "scripts", "audit_rosbag_run.py"
))
SPEC = importlib.util.spec_from_file_location("audit_rosbag_run", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def cloud(stamp, field, label):
    message = PointCloud2()
    message.header.stamp = stamp
    message.header.frame_id = "velodyne"
    message.height = 1
    message.width = 1
    message.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField(field, 12, (
            PointField.FLOAT32 if field == "intensity" else PointField.UINT32
        ), 1),
    ]
    message.point_step = 16
    message.row_step = 16
    message.is_dense = True
    message.data = (
        struct.pack("<ffff", 1.0, 2.0, 3.0, float(label))
        if field == "intensity"
        else struct.pack("<fffI", 1.0, 2.0, 3.0, int(label))
    )
    return message


class LidarGtAuditTest(unittest.TestCase):
    def test_closed_bag_requires_real_messages_on_each_gt_topic(self):
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, "incomplete.bag")
            stamp = rospy.Time.from_sec(10.0)
            with rosbag.Bag(path, "w") as bag:
                bag.write("/velodyne_points", cloud(stamp, "intensity", 0.5), t=stamp)
                bag.write("/velodyne_points_semantic", cloud(stamp, "semantic_id", 17), t=stamp)
            counts = recorded_topic_counts([path])
        self.assertEqual(1, counts["/velodyne_points"])
        self.assertEqual(1, counts["/velodyne_points_semantic"])
        self.assertEqual(["/velodyne_points_instance"], missing_recorded_topics(
            ["/velodyne_points", "/velodyne_points_semantic", "/velodyne_points_instance"],
            counts,
        ))

    def test_three_way_schema_rates_and_pairing(self):
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, "gt.bag")
            with rosbag.Bag(path, "w") as bag:
                for index in range(4):
                    stamp = rospy.Time.from_sec(10.0 + index / 15.0)
                    for topic, field, label in (
                        ("/velodyne_points", "intensity", 0.5),
                        ("/velodyne_points_semantic", "semantic_id", 17),
                        ("/velodyne_points_instance", "instance_id", 3),
                    ):
                        bag.write(topic, cloud(stamp, field, label), t=stamp)
            report = MODULE.audit([path])
        self.assertTrue(report["ok"], report["checks"])
        self.assertTrue(report["semantic_lidar"]["schema_ok"])
        self.assertTrue(report["instance_lidar"]["schema_ok"])
        self.assertTrue(report["pairing"]["intensity_semantic_lidar"]["ok"])
        self.assertTrue(report["pairing"]["semantic_instance_lidar"]["ok"])
        self.assertEqual(3, len([
            topic for topic in report["topics"] if topic.startswith("/velodyne")
        ]))


if __name__ == "__main__":
    unittest.main()
