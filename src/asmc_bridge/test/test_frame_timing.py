#!/usr/bin/env python3
"""Raw-clock evidence and kernel receipt tests need neither ROS nor a GPU."""
import os
import socket
import struct
import sys
import threading
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from asmc_bridge.frame_timing import PacketReceipt, SourceClockTracker, publish_sensor, receipt_fields
from asmc_bridge.telemetry import BridgeTelemetry


class FrameTimingTest(unittest.TestCase):
    def test_duplicate_conflict_regression_and_invalid_keep_raw_clock_separate(self):
        tracker = SourceClockTracker(capacity=2)
        self.assertTrue(tracker.sec_nsec(1, 0, (1,))['source_valid'])
        self.assertTrue(tracker.sec_nsec(1, 0, (1,))['source_duplicate'])
        self.assertTrue(tracker.sec_nsec(1, 0, (2,))['source_conflict'])
        self.assertTrue(tracker.sec_nsec(1, 1, (3,))['source_valid'])
        self.assertTrue(tracker.sec_nsec(1, 0, (2,))['source_regression'])
        self.assertFalse(tracker.sec_nsec(0, 0)['source_valid'])
        self.assertFalse(tracker.sec_nsec(1, 1000000000)['source_valid'])
        for nsec in range(2, 100):
            tracker.sec_nsec(1, nsec)
        self.assertEqual(len(tracker.recent), 2)
        self.assertEqual(tracker.latest, 1000000099)

    def test_device_hour_rollover_and_regression_are_not_unix_time(self):
        tracker = SourceClockTracker()
        tracker.device([3599999000])
        result = tracker.device([10, 20])
        self.assertTrue(result['device_hour_rollover'])
        self.assertFalse(result['source_regression'])
        self.assertTrue(tracker.device([5])['source_regression'])
        self.assertFalse(tracker.device([None, 3600000000])['source_valid'])

    def test_realtime_clock_jump_does_not_change_monotonic_duration(self):
        result = receipt_fields([PacketReceipt(1000, 10, True), PacketReceipt(900, 20, False, True)])
        self.assertLess(result['rx_last_unix_ns'], result['rx_first_unix_ns'])
        self.assertEqual(result['rx_last_monotonic_ns'] - result['rx_first_monotonic_ns'], 10)
        self.assertEqual(result['rx_userspace_fallback_packets'], 1)
        self.assertEqual(result['ancillary_truncated_packets'], 1)
        with self.assertRaises(ValueError):
            receipt_fields([])

    def test_kernel_ancillary_drop_and_control_truncation(self):
        metrics = BridgeTelemetry(); metrics.register('cam1')
        sock = mock.Mock()
        control = [(socket.SOL_SOCKET, 64, struct.pack('=qq', 2, 30)),
                   (socket.SOL_SOCKET, 40, struct.pack('=I', 7))]
        sock.recvmsg.return_value = (b'image', control, 0, ('host', 1))
        data, _, receipt = metrics.receive_timed('cam1', sock, 100)
        self.assertEqual(data, b'image')
        self.assertEqual(receipt.unix_ns, 2000000030)
        self.assertTrue(receipt.kernel)
        sock.recvmsg.assert_called_with(100, socket.CMSG_SPACE(16) + socket.CMSG_SPACE(4))
        sock.recvmsg.return_value = (b'image', control, socket.MSG_CTRUNC, ('host', 1))
        _, _, receipt = metrics.receive_timed('cam1', sock, 100)
        self.assertFalse(receipt.kernel)
        self.assertTrue(receipt.ancillary_truncated)
        row = metrics.snapshot()[2]['cam1']
        self.assertEqual(row['socket_drops'], 7)
        self.assertEqual(row['ancillary_truncations'], 1)
        self.assertEqual(row['rx_timestamp_fallbacks'], 1)

    def test_real_linux_kernel_timestamp_and_legacy_two_value_receive(self):
        metrics = BridgeTelemetry(); metrics.register('cam1')
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            receiver.bind(('127.0.0.1', 0)); receiver.settimeout(1)
            metrics.configure_socket('cam1', receiver)
            sender.sendto(b'image', receiver.getsockname())
            data, _, receipt = metrics.receive_timed('cam1', receiver, 100)
            self.assertEqual(data, b'image')
            option = metrics.snapshot()[2]['cam1']['socket_timestamp_option']
            self.assertEqual(receipt.kernel, bool(option))
            sender.sendto(b'next', receiver.getsockname())
            self.assertEqual(len(metrics.receive('cam1', receiver, 100)), 2)

    def test_unsupported_timestamp_socket_is_explicit_userspace_fallback(self):
        metrics = BridgeTelemetry(); metrics.register('imu')
        sock = mock.Mock(); sock.setsockopt.side_effect = OSError('unsupported')
        sock.getsockopt.return_value = 4096
        metrics.configure_socket('imu', sock)
        sock.recvmsg.return_value = (b'imu', [], 0, ('host', 1))
        _, _, receipt = metrics.receive_timed('imu', sock, 100)
        self.assertFalse(receipt.kernel)
        row = metrics.snapshot()[2]['imu']
        self.assertEqual(row['socket_timestamp_option'], 0)
        self.assertEqual(row['rx_timestamp_fallbacks'], 1)

    def test_wire_sequence_is_observed_after_serialization_under_publisher_lock(self):
        class Impl:
            seq = 7
            lock = threading.RLock()
            def acquire(self): self.lock.acquire()
            def release(self): self.lock.release()
        publisher = mock.Mock(); publisher.impl = Impl()
        message = mock.Mock(); message.header.seq = 0
        # rospy publishes with no connections without serializing or advancing seq.
        self.assertEqual(publish_sensor(publisher, message), (0, False))
        def serialize(value):
            publisher.impl.seq += 1
            value.header.seq = publisher.impl.seq
        publisher.publish.side_effect = serialize
        self.assertEqual(publish_sensor(publisher, message), (8, True))


if __name__ == '__main__':
    unittest.main()
