#!/usr/bin/env python3
"""Exercise real UDP→ROS health transitions while /clock is paused."""
import socket
import struct
import threading
import time
import unittest

import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray


class ReportingRosTest(unittest.TestCase):
    def test_pause_stale_and_recovery(self):
        lock = threading.Lock()
        received = []
        def callback(message):
            for status in message.status:
                if status.name == 'morai_sensor_bridge/cam1':
                    values = {value.key: value.value for value in status.values}
                    with lock:
                        received.append((time.monotonic(), values, message.header.stamp.to_nsec()))
        subscriber = rospy.Subscriber('/morai_sensor_bridge/diagnostics', DiagnosticArray, callback, queue_size=1)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sequence = 0
        def wait_for(state, send=False):
            nonlocal sequence
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline:
                with lock:
                    if received and received[-1][1]['health'] == state:
                        return received[-1]
                if send:
                    sequence += 1
                    payload = b'opaque-jpeg-transport-fixture'
                    sender.sendto(b'MOR' + struct.pack('<IIii', 1, sequence, 0, len(payload)) + payload + b'EI', ('127.0.0.1', 39194))
                time.sleep(.02)
            self.fail('did not observe ' + state)
        try:
            wait_for('WAITING')
            first = wait_for('OK', send=True)
            stale = wait_for('STALE')
            resumed = wait_for('OK', send=True)
            self.assertGreaterEqual(int(stale[1]['messages']), int(first[1]['messages']))
            self.assertGreater(float(stale[1]['last_message_age_sec']), .5)
            self.assertGreater(int(resumed[1]['messages']), int(stale[1]['messages']))
            with lock:
                rows = tuple(received)
            self.assertGreaterEqual(len(rows), 4)
            self.assertTrue(all(row[2] == 0 for row in rows))
            self.assertTrue(all(.75 < b[0] - a[0] < 1.25 for a, b in zip(rows, rows[1:])))
            self.assertEqual(resumed[1]['reporter_failures'], '0')
        finally:
            sender.close()
            subscriber.unregister()


if __name__ == '__main__':
    rospy.init_node('bridge_reporting_ros_test')
    rostest.rosrun('asmc_bridge', 'bridge_reporting_ros', ReportingRosTest)
