#!/usr/bin/env python3

import importlib.util
import os
import struct
import sys
import unittest

import numpy as np
import rospy
from sensor_msgs.msg import PointField


SCRIPT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts", "morai_sensor_bridge.py")
)
sys.path.insert(0, os.path.join(os.path.dirname(SCRIPT), '..', 'src'))
SPEC = importlib.util.spec_from_file_location("morai_sensor_bridge_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SensorBridgeProtocolTest(unittest.TestCase):
    def test_launch_default_and_gt_share_15hz_reference_without_forcing_output_rate(self):
        import roslaunch
        launch = os.path.join(os.path.dirname(SCRIPT), '..', 'launch', 'bridge.launch')
        for profile, enabled in [('competition', False), ('development', True)]:
            config = roslaunch.config.load_config_default([(launch, ['stream_profile:='+profile])], None, verbose=False)
            self.assertEqual(config.params['/morai_sensor_bridge/lidar_rotation_rate'].value, 15.)
            self.assertEqual(config.params['/morai_sensor_bridge/lidar_frame_mode'].value, 'azimuth')
            self.assertEqual(config.params['/morai_sensor_bridge/enable_semantic_camera'].value, enabled)
            self.assertEqual(config.params['/morai_sensor_bridge/enable_instance_lidar'].value, enabled)
            self.assertEqual(config.params['/morai_udp_bridge/enable_object'].value, enabled)
        recovery = roslaunch.config.load_config_default([(launch, ['lidar_rotation_rate:=10.0'])], None, verbose=False)
        self.assertEqual(recovery.params['/morai_sensor_bridge/lidar_rotation_rate'].value, 10.)

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

    @staticmethod
    def lidar_packet(azimuth):
        packet = bytearray(1200)
        struct.pack_into("<H", packet, 2, azimuth)
        return bytes(packet)

    def test_azimuth_assembler_resynchronizes_and_emits_one_revolution(self):
        assembler = MODULE.VelodyneFrameAssembler("azimuth", 75)
        # Initial partial revolution is deliberately dropped at the first wrap.
        for index, azimuth in enumerate((30000, 33000, 35900, 100)):
            self.assertIsNone(
                assembler.push(self.lidar_packet(azimuth), index, index)
            )
        completed = None
        full_revolution = tuple(range(100, 36100, 2000)) + (50,)
        for index, azimuth in enumerate(full_revolution, 4):
            completed = assembler.push(
                self.lidar_packet(azimuth), index, index
            )
        self.assertIsNotNone(completed)
        chunks, device_times, started_ns = completed
        self.assertEqual(len(full_revolution), len(chunks))
        self.assertEqual(
            list(range(3, 3 + len(full_revolution))), device_times
        )
        self.assertEqual(3, started_ns)

    def test_packet_count_mode_preserves_legacy_grouping(self):
        assembler = MODULE.VelodyneFrameAssembler("packet_count", 3)
        self.assertIsNone(assembler.push(self.lidar_packet(100), 1, 10))
        self.assertIsNone(assembler.push(self.lidar_packet(200), 2, 20))
        completed = assembler.push(self.lidar_packet(300), 3, 30)
        self.assertEqual(3, len(completed[0]))


if __name__ == "__main__":
    unittest.main()
