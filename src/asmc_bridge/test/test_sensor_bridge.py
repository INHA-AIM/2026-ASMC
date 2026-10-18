#!/usr/bin/env python3

import importlib.util
import os
import struct
import sys
import unittest
from unittest import mock
from types import SimpleNamespace

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
    @staticmethod
    def bridge_for_timing():
        bridge = MODULE.SensorBridge.__new__(MODULE.SensorBridge)
        bridge.telemetry = MODULE.BridgeTelemetry()
        for name in ('cam1', 'imu', 'lidar'):
            bridge.telemetry.register(name)
        bridge._timing_sequences = {}; bridge._timing_trackers = {}
        bridge._header_fallback_reason = {}; bridge._last_source_ns = {}
        bridge.pub_timing = mock.Mock(); bridge.bridge_session_id = 'test-session'
        bridge.use_sim_time = False; bridge.timestamp_mode = 'receive'
        return bridge

    @mock.patch.object(MODULE.rospy.Time, 'now', return_value=rospy.Time(50))
    def test_raw_validation_runs_even_when_sensor_header_uses_receive(self, _now):
        from asmc_bridge.frame_timing import PacketReceipt
        bridge = self.bridge_for_timing()
        publisher = SimpleNamespace(resolved_name='/imu')
        sensor = SimpleNamespace(header=SimpleNamespace(frame_id='imu', stamp=rospy.Time(50)))
        for sec, signature in [(10, (1,)), (10, (1,)), (10, (2,)), (9, (3,)), (0, (0,))]:
            bridge._publish_timing('imu', publisher, sensor, [PacketReceipt(100, 20, True)],
                                   (23, True), sec=sec, signature=signature, reference='imu_sample')
        records = [call.args[0] for call in bridge.pub_timing.publish.call_args_list]
        self.assertEqual(len(records), 5)
        self.assertTrue(records[1].source_duplicate)
        self.assertTrue(records[2].source_conflict)
        self.assertTrue(records[3].source_regression)
        self.assertFalse(records[4].source_valid)
        self.assertEqual(records[4].source_sec, 0)
        self.assertEqual(records[4].published_stamp, rospy.Time(50))
        self.assertEqual(records[4].frame_sequence, 5)
        self.assertEqual(records[4].sensor_wire_sequence, 23)

    def test_camera_missing_and_reordered_fragments_do_not_create_metadata(self):
        from asmc_bridge.frame_timing import PacketReceipt
        bridge = self.bridge_for_timing()
        bridge._publish_timing = mock.Mock()
        bridge._stamp_from_sec_nsec = lambda sec, nsec, name: rospy.Time(sec, nsec)
        def packet(sec, index, final):
            return b'MOR' + struct.pack('<IIii', sec, 0, index, 1) + b'x' + (b'EI' if final else b'AI')
        # Interrupted frame, out-of-order tail, then one complete two-packet frame.
        packets = [packet(1, 0, False), packet(1, 2, True), packet(2, 0, False), packet(2, 1, True)]
        bridge.telemetry.receive_timed = mock.Mock(side_effect=[
            (value, None, PacketReceipt(100 + i, 20 + i, True)) for i, value in enumerate(packets)])
        publisher = mock.Mock()
        with mock.patch.object(MODULE.rospy, 'is_shutdown', side_effect=[False] * len(packets) + [True]), \
                mock.patch.object(MODULE, 'publish_sensor', return_value=(1, True)) as publish:
            bridge._camera_loop(None, 'cam1', publisher, None, 'camera_front')
        self.assertEqual(publish.call_count, 1)
        self.assertEqual(publish.call_args.args[1].data, b'xx')
        receipts = bridge._publish_timing.call_args.args[3]
        self.assertEqual([value.unix_ns for value in receipts], [102, 103])

    def test_assembler_receipts_belong_to_completed_sweep_not_next_wrap(self):
        assembler = MODULE.VelodyneFrameAssembler('packet_count', 2)
        assembler.push(self.lidar_packet(100), 10, 1, 'first')
        assembler.push(self.lidar_packet(200), 20, 2, 'last')
        self.assertEqual(assembler.completed_receipts, ('first', 'last'))
        assembler.push(self.lidar_packet(300), 30, 3, 'next')
        self.assertEqual(assembler.receipts, ['next'])

    def test_camera_timeout_does_not_publish_a_late_tail_as_a_fresh_image(self):
        from asmc_bridge.frame_timing import PacketReceipt
        bridge = self.bridge_for_timing(); bridge._publish_timing = mock.Mock()
        def packet(index, final):
            return b'MOR' + struct.pack('<IIii', 100, 0, index, 1) + b'x' + (b'EI' if final else b'AI')
        bridge.telemetry.receive_timed = mock.Mock(side_effect=[
            (packet(0, False), None, PacketReceipt(100, 1)), MODULE.socket.timeout(),
            (packet(1, True), None, PacketReceipt(200, 2))])
        with mock.patch.object(MODULE.rospy, 'is_shutdown', side_effect=[False] * 3 + [True]):
            bridge._camera_loop(None, 'cam1', mock.Mock(), None, 'camera_front')
        bridge._publish_timing.assert_not_called()
        row = bridge.telemetry.snapshot()[2]['cam1']
        self.assertEqual(row['assembly_timeouts'], 1)
        self.assertEqual(row['incomplete_frames'], 1)

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
