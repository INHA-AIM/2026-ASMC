#!/usr/bin/env python3
"""Real node publication: GPS-zero, no-message timeout and fresh recovery.

Runs in rostest's isolated master with synthetic inputs, no simulator/control.
"""
from collections import deque
import threading
import time
import unittest

import rospy
import rostest
from asmc_msgs.msg import VehicleState
from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import Imu


class VehicleStateRosTest(unittest.TestCase):
    def test_gps_validity_and_recovery(self):
        lock = threading.Lock()
        rows = deque(maxlen=1000)

        def callback(message):
            with lock:
                rows.append((time.monotonic(), message))

        sub = rospy.Subscriber('/bridge_fixture/state', VehicleState, callback, queue_size=1)
        gps_pub = rospy.Publisher('/bridge_fixture/gps', GPSMessage, queue_size=1)
        imu_pub = rospy.Publisher('/bridge_fixture/imu', Imu, queue_size=1)

        def phase(duration, gps_mode, expected, final_window=.08):
            start = time.monotonic()
            next_gps = start
            sent = False
            while time.monotonic() - start < duration and not rospy.is_shutdown():
                now = time.monotonic()
                stamp = rospy.Time.now()
                imu = Imu()
                imu.header.stamp = stamp
                imu.orientation.w = 1.0
                imu_pub.publish(imu)
                if gps_mode != 'missing' and now >= next_gps and not (gps_mode == 'first' and sent):
                    gps = GPSMessage()
                    gps.header.stamp = stamp
                    gps.status = 1
                    gps.latitude, gps.longitude = ((0., 0.) if gps_mode == 'zero' else (37.239, 126.773))
                    gps.eastOffset, gps.northOffset = 302595., 4124145.
                    gps_pub.publish(gps)
                    next_gps = now + .2
                    sent = True
                time.sleep(.01)
            end = time.monotonic()
            with lock:
                selected = [m for t, m in rows if end - final_window <= t <= end]
            self.assertGreaterEqual(len(selected), 2, 'state publisher stopped during '+gps_mode)
            self.assertTrue(all(m.valid == expected for m in selected), gps_mode)
            for message in selected:
                self.assertEqual(message.header.frame_id, 'map')
                self.assertEqual(message.source, VehicleState.SOURCE_GPS_IMU)
                if not expected:
                    self.assertEqual((message.x, message.y, message.speed), (0., 0., 0.))

        try:
            deadline = time.monotonic() + 5
            while not (gps_pub.get_num_connections() and imu_pub.get_num_connections()):
                self.assertLess(time.monotonic(), deadline, 'fixture inputs not connected')
                time.sleep(.02)
            phase(1.0, 'normal', True)
            phase(.6, 'zero', False)
            phase(.16, 'first', False)
            phase(.6, 'normal', True)
            phase(.8, 'missing', False)
            phase(.16, 'first', False)
            phase(.6, 'normal', True)
        finally:
            sub.unregister()
            gps_pub.unregister()
            imu_pub.unregister()


if __name__ == '__main__':
    rospy.init_node('vehicle_state_recovery_test')
    rostest.rosrun('asmc_bridge', 'vehicle_state_recovery', VehicleStateRosTest)
