#!/usr/bin/env python3
"""Health, concurrent reporting and physical rotation evidence regressions."""
import os
import socket
import struct
import sys
import threading
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from asmc_bridge.telemetry import BridgeTelemetry, PeriodicReporter, latency_summary


class TelemetryTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.metrics = BridgeTelemetry(clock=lambda: self.now, capacity=32)
        self.metrics.register('lidar')
        self.metrics.receiver('lidar', 'running')

    def row(self, channel='diagnostics'):
        return self.metrics.snapshot(channel)[2]['lidar']

    def test_waiting_stale_recovery_and_idle_are_distinct(self):
        self.assertEqual(self.row()['health'], 'WAITING')
        self.metrics.message('lidar')
        self.assertEqual(self.row()['health'], 'OK')
        self.now += .501
        self.assertEqual(self.row()['health'], 'STALE')
        self.metrics.message('lidar')
        self.assertEqual(self.row()['health'], 'OK')
        self.metrics.register('competition', optional=True)
        self.metrics.register('cmd', transmit=True)
        rows = self.metrics.snapshot()[2]
        self.assertEqual(rows['competition']['health'], 'IDLE')
        self.assertEqual(rows['cmd']['health'], 'IDLE')
        self.metrics.message('competition')
        self.now += 1
        self.assertEqual(self.metrics.snapshot()[2]['competition']['health'], 'STALE')

    def test_invalid_header_is_visible_even_on_optional_input(self):
        self.metrics.register('competition', optional=True)
        self.metrics.increment('competition', 'header_errors')
        self.assertEqual(self.metrics.snapshot()[2]['competition']['health'], 'INVALID')

    def test_recent_errors_warn_but_historical_errors_do_not_hide_recovery(self):
        self.metrics.message('lidar')
        self.metrics.increment('lidar', 'parse_errors')
        self.assertEqual(self.row()['health'], 'DEGRADED')
        row = self.row()
        self.assertEqual(row['health'], 'OK')
        self.assertEqual(row['parse_errors'], 1)
        self.metrics.receiver('lidar', 'failed', 'bind failed')
        self.assertEqual(self.row()['health'], 'ERROR')

    def test_diagnostic_and_console_rates_use_independent_actual_windows(self):
        for _ in range(12):
            self.metrics.message('lidar')
        self.now += 1
        self.assertEqual(self.row()['interval_hz'], 12)
        for _ in range(48):
            self.metrics.message('lidar')
        self.now += 4
        self.assertEqual(self.row()['interval_hz'], 12)
        self.assertEqual(self.row('console')['interval_hz'], 12)
        self.assertEqual(self.row()['messages'], 60)

    def test_concurrent_samples_and_snapshots_are_bounded_and_immutable(self):
        failures = []
        start = threading.Barrier(3)
        def writer():
            try:
                start.wait()
                for value in range(6000):
                    self.metrics.sample('lidar', 'processing', value)
                    self.metrics.message('lidar')
            except Exception as error:
                failures.append(error)
        threads = [threading.Thread(target=writer) for _ in range(2)]
        for thread in threads:
            thread.start()
        start.wait()
        for _ in range(150):
            row = self.row()
            self.assertLessEqual(len(row['processing']), 32)
            latency_summary(row['processing'])
        for thread in threads:
            thread.join(timeout=5)
            self.assertFalse(thread.is_alive())
        self.assertEqual(failures, [])
        row = self.row()
        self.assertEqual(row['messages'], 12000)
        retained = row['processing']
        self.metrics.sample('lidar', 'processing', -1)
        self.assertEqual(row['processing'], retained)
        self.assertNotIn(-1, row['processing'])

    def test_socket_overflow_is_reported_and_truncated_packet_is_not_parsed(self):
        sock = mock.Mock()
        sock.recvmsg.return_value = (b'partial', [(socket.SOL_SOCKET, 40, struct.pack('=I', 3))], socket.MSG_TRUNC, ('host', 1))
        data, _ = self.metrics.receive('lidar', sock, 4)
        self.assertIsNone(data)
        row = self.row()
        self.assertEqual(row['socket_drops'], 3)
        self.assertEqual(row['raw_packets'], 1)
        self.assertEqual(row['messages'], 0)
        self.assertEqual(row['parse_errors'], 1)

    def test_device_rollover_and_wall_cadence_are_separate(self):
        self.metrics.rotation('lidar', 3599950000)
        self.now += .1
        self.metrics.rotation('lidar', 16667)
        row = self.row()
        self.assertAlmostEqual(row['wall_rotation'][0], 100.)
        self.assertAlmostEqual(row['device_rotation'][0], 66.667)
        self.assertEqual(row['device_regressions'], 0)
        self.metrics.rotation('lidar', 10)
        self.assertEqual(self.row()['device_regressions'], 1)

    def test_reporter_failure_does_not_disable_next_report(self):
        reporter = PeriodicReporter.__new__(PeriodicReporter)
        reporter.callback = mock.Mock(side_effect=[RuntimeError('report failure'), None])
        reporter.on_error = mock.Mock()
        reporter.failures = 0
        reporter.tick()
        reporter.tick()
        self.assertEqual(reporter.callback.call_count, 2)
        self.assertEqual(reporter.failures, 1)
        reporter.on_error.assert_called_once()

    def test_reporter_stops_while_ros_clock_is_not_running(self):
        happened = threading.Event()
        reporter = PeriodicReporter(lambda _: happened.set(), .01, lambda error: self.fail(str(error)))
        try:
            self.assertTrue(happened.wait(timeout=1))
        finally:
            reporter.stop()
        self.assertFalse(reporter.thread.is_alive())

    def test_invalid_diagnostic_threshold_is_rejected(self):
        with self.assertRaises(ValueError):
            self.metrics.register('bad', stale_after=float('nan'))


if __name__ == '__main__':
    unittest.main()
