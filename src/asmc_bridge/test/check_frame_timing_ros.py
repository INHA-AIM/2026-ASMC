#!/usr/bin/env python3
"""Real UDP->ROS serialization, raw-clock validation and wire join contract."""
import socket
import struct
import threading
import time
import unittest

import rospy
import rostest
from asmc_msgs.msg import SensorFrameTiming
from asmc_bridge.frame_timing import publish_sensor
from sensor_msgs.msg import CompressedImage, Imu


class TimingWireTest(unittest.TestCase):
    def test_source_and_wire_sequences_are_independent(self):
        lock = threading.Lock(); sensors = {}; timings = []
        def sensor(message, topic):
            with lock:
                sensors[(topic, message.header.seq)] = message
        def timing(message):
            with lock:
                timings.append(message)
        subs = [rospy.Subscriber('/timing_fixture/image', CompressedImage, sensor, callback_args='/timing_fixture/image'),
                rospy.Subscriber('/imu', Imu, sensor, callback_args='/imu'),
                rospy.Subscriber('/morai_sensor_bridge/frame_timing', SensorFrameTiming, timing)]
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            deadline = time.monotonic() + 8
            while not all(sub.get_num_connections() for sub in subs) and time.monotonic() < deadline:
                time.sleep(.02)
            self.assertTrue(all(sub.get_num_connections() for sub in subs))
            def camera(index, payload, final):
                return b'MOR' + struct.pack('<IIii', 100, 200, index, len(payload)) + payload + (b'EI' if final else b'AI')
            for packet in (camera(0, b'one', False), camera(1, b'two', True)):
                udp.sendto(packet, ('127.0.0.1', 19291))
                time.sleep(.01)
            for sec, gyro in ((100, 0.), (100, 0.), (100, 1.), (99, 0.)):
                packet = b'#IMUData$'.ljust(25, b'\0') + struct.pack('<II10d', sec, 200,
                    1., 0., 0., 0., gyro, 0., 0., 0., 0., 9.8)
                udp.sendto(packet, ('127.0.0.1', 19303)); time.sleep(.01)
            deadline = time.monotonic() + 5
            while (len(timings) < 5 or len(sensors) < 5) and time.monotonic() < deadline:
                time.sleep(.02)
            with lock:
                self.assertEqual(len(timings), 5)
                self.assertEqual(len(sensors), 5)
                for row in timings:
                    self.assertTrue(row.sensor_wire_sequence_valid)
                    original = sensors[row.sensor_topic, row.sensor_wire_sequence]
                    self.assertEqual(original.header.stamp, row.published_stamp)
                    self.assertEqual(original.header.frame_id, row.sensor_frame_id)
                    self.assertEqual(row.header_timestamp_policy, 'receive')
                    self.assertGreater(row.published_stamp.secs, 100)
                    self.assertGreaterEqual(row.publish_monotonic_ns, row.rx_last_monotonic_ns)
                camera_row = next(row for row in timings if row.stream_id == 'cam1')
                self.assertEqual(camera_row.packet_count, 2)
                self.assertEqual(camera_row.source_sec, 100)
                self.assertEqual(sensors[camera_row.sensor_topic, camera_row.sensor_wire_sequence].data, b'onetwo')
                imu_rows = [row for row in timings if row.stream_id == 'imu']
                self.assertTrue(imu_rows[1].source_duplicate)
                self.assertFalse(imu_rows[1].source_conflict)
                self.assertTrue(imu_rows[2].source_conflict)
                self.assertTrue(imu_rows[3].source_regression)
        finally:
            udp.close()
            for sub in subs: sub.unregister()

        publisher = rospy.Publisher('/timing_fixture/no_subscriber', CompressedImage, queue_size=1)
        try:
            message = CompressedImage()
            self.assertEqual(publish_sensor(publisher, message), (0, False))
        finally:
            publisher.unregister()


if __name__ == '__main__':
    rospy.init_node('frame_timing_contract_test')
    rostest.rosrun('asmc_bridge', 'frame_timing_wire', TimingWireTest)
