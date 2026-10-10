#!/usr/bin/env python3
"""Offline ROS ownership and GPS transition check; never starts the MORAI bridge."""
import argparse
import os
import signal
import subprocess
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from morai_msgs.msg import CtrlCmd, GPSMessage
from rosgraph.masterapi import Master
from std_msgs.msg import String, UInt8


def wait_for(predicate, description, seconds=20):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError('timed out waiting for ' + description)


def owners(master, topic):
    return sorted(dict(master.getSystemState()[0]).get(topic, []))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=11407)
    args = parser.parse_args()
    env = os.environ.copy()
    env['ROS_MASTER_URI'] = 'http://127.0.0.1:{}'.format(args.port)
    env['ROS_IP'] = '127.0.0.1'
    env.pop('ROS_HOSTNAME', None)
    os.environ.update(env)
    core = subprocess.Popen(['roscore', '-p', str(args.port)], env=env,
                            stdout=open('/tmp/asmc_mode_stack_core.log', 'w'),
                            stderr=subprocess.STDOUT, start_new_session=True)
    launch = None
    try:
        time.sleep(2)
        launch = subprocess.Popen(
            ['roslaunch', 'asmc_planner', 'mode_control_integration.launch',
             'traffic_light:=false'], env=env,
            stdout=open('/tmp/asmc_mode_stack_launch.log', 'w'),
            stderr=subprocess.STDOUT, start_new_session=True)
        rospy.init_node('asmc_mode_stack_offline_check', anonymous=True)
        seen = {'mode': None, 'source': '', 'jamming': '', 'control': None}

        def jamming_status(message):
            if message.status:
                values = {entry.key: entry.value for entry in message.status[0].values}
                seen['jamming'] = values.get('mode_active', '')

        rospy.Subscriber('/planning/mode', UInt8,
                         lambda message: seen.__setitem__('mode', message.data))
        rospy.Subscriber('/planning/control_selector/diagnostics', String,
                         lambda message: seen.__setitem__('source', message.data))
        rospy.Subscriber('/planning/camera_bbox_lane_change/diagnostics',
                         DiagnosticArray, jamming_status)
        rospy.Subscriber('/ctrl_cmd', CtrlCmd,
                         lambda message: seen.__setitem__('control', message))
        gps = rospy.Publisher('/gps', GPSMessage, queue_size=1)
        wait_for(lambda: seen['mode'] == 0 and seen['jamming'] == 'false' and
                 gps.get_num_connections() >= 1, 'initial NORMAL stack')

        master = Master(rospy.get_name())
        expected = {'/ctrl_cmd': ['/asmc_control_selector'],
                    '/ctrl_cmd/normal': ['/mpc_controller'],
                    '/ctrl_cmd/jamming': ['/camera_bbox_lane_change'],
                    '/planning/mode': ['/asmc_planner']}
        for topic, names in expected.items():
            actual = owners(master, topic)
            assert actual == names, (topic, actual, names)

        jammed = GPSMessage(latitude=0.0, longitude=0.0, altitude=0.0)
        wait_for(lambda: (gps.publish(jammed) or False) or
                 (seen['mode'] == 1 and seen['jamming'] == 'true' and
                  'selected_source=/ctrl_cmd/jamming' in seen['source']),
                 'JAMMING after raw GPS 0/0/0')
        wait_for(lambda: seen['control'] is not None,
                 'selected jamming STOP command')
        recovered = GPSMessage(latitude=37.0, longitude=126.0, altitude=12.0)
        wait_for(lambda: (gps.publish(recovered) or False) or
                 (seen['mode'] == 0 and seen['jamming'] == 'false' and
                  'selected_source=/ctrl_cmd/normal' in seen['source']),
                 'NORMAL after nonzero GPS recovery')
        for topic, names in expected.items():
            assert owners(master, topic) == names, topic
        print('PASS: NORMAL -> 0/0/0 JAMMING -> recovery NORMAL; sole /ctrl_cmd owner')
    finally:
        for process in (launch, core):
            if process and process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)


if __name__ == '__main__':
    main()
