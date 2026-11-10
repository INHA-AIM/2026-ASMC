#!/usr/bin/env python3
"""A448 preparation/25kmh is distinct from actual GPS-loss selection."""
import json
import time
import unittest
import rospy
import rostest
from std_msgs.msg import Bool, String, UInt8
from asmc_msgs.msg import ControlHandoff, VehicleState
from morai_msgs.msg import GPSMessage


class SpeedStages(unittest.TestCase):
    def test_upstream_link_entry_and_actual_loss(self):
        prepares, traces, modes, requests = [], [], [], []
        rospy.Subscriber('/planning/localization/prepare', Bool, lambda m: prepares.append(m.data))
        rospy.Subscriber('/planning/localization/approach_trace', String, lambda m: traces.append(json.loads(m.data)))
        rospy.Subscriber('/planning/mode', UInt8, lambda m: modes.append(m.data))
        rospy.Subscriber('/planning/localization/handoff_request', ControlHandoff, lambda m: requests.append(m))
        state = rospy.Publisher('/planning/vehicle/state', VehicleState, queue_size=1)
        gps = rospy.Publisher('/gps', GPSMessage, queue_size=1)
        candidate = rospy.Publisher('/planning/localization/handoff_candidate', ControlHandoff, queue_size=1)

        def wait(predicate):
            end = time.monotonic() + 5
            while not predicate():
                self.assertLess(time.monotonic(), end)
                time.sleep(.005)

        wait(lambda: prepares and traces and modes and state.get_num_connections() and gps.get_num_connections())
        gps.publish(GPSMessage(latitude=37., longitude=126., status=1))

        def send(x, speed=57.5/3.6):
            message = VehicleState(valid=True, x=x, y=0, yaw=0, speed=speed)
            message.header.frame_id = 'map'
            message.header.stamp = rospy.Time.now()
            state.publish(message)
            time.sleep(.24)

        send(0)
        self.assertFalse(prepares[-1])
        self.assertEqual(traces[-1]['effective_cap_kph'], 0., 'no extra upstream speed cap')
        send(19.)
        self.assertFalse(prepares[-1], 'braking-distance lookahead must not trigger before A448')
        send(20.)
        wait(lambda: prepares[-1] and traces[-1]['effective_cap_kph'] == 25.)
        self.assertEqual(modes[-1], 0, 'link entry prepares but does not select jamming')
        send(55., 25./3.6)
        self.assertEqual(modes[-1], 0, 'known tunnel station with valid GPS still stays NORMAL')
        self.assertEqual(traces[-1]['effective_cap_kph'], 25.)
        gps.publish(GPSMessage())
        wait(lambda: requests and requests[-1].valid)
        self.assertEqual(modes[-1], 0, 'command preparation precedes selection')
        request = requests[-1]
        evidence = ControlHandoff(epoch=request.epoch, target_mode=1, valid=True, longl_cmd_type=1, accel=.1)
        evidence.header.stamp = rospy.Time.now()
        evidence.header.frame_id = '/localization_controller'
        evidence.state_stamp = evidence.trajectory_stamp = evidence.header.stamp
        evidence.state_frame = 'local/stages'
        candidate.publish(evidence)
        wait(lambda: modes[-1] == 1)


if __name__ == '__main__':
    rospy.init_node('localization_speed_stages_test')
    rostest.rosrun('asmc_planner', 'localization_speed_stages', SpeedStages)
