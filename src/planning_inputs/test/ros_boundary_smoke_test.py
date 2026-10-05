#!/usr/bin/env python3
"""Exercise actual private topic subscriptions and legacy ROS wire conversion."""
import os
import signal
import subprocess
import threading
import time

import rosgraph
import rospy
from asmc_msgs.msg import TrackedObject, TrackedObjectArray
from geometry_msgs.msg import Point
from planning_inputs.msg import DetectedObject, DetectedObjectArray


class LegacyTrackedObjectArray(TrackedObjectArray):
    # Main's nested message has the frozen canonical fields and MD5.
    _type = 'Msg/TrackedObjectArray'


def require(predicate, description, timeout=30):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            if predicate():
                print('PASS ' + description, flush=True)
                return
        except (OSError, rosgraph.MasterException):
            pass
        time.sleep(0.1)
    raise AssertionError(description)


def main():
    launch = subprocess.Popen([
        'roslaunch', 'behavior_planner', 'autonomy_no_bridges_v2.launch',
        'objects_source:=lidar', 'dashboard:=false'], start_new_session=True)
    try:
        master = rosgraph.Master('/planning_boundary_test')
        require(lambda: bool(master.getPid()), 'ROS master started')
        rospy.init_node('planning_boundary_test', anonymous=True)

        def subscriptions():
            return dict(master.getSystemState()[1])

        require(lambda: {'/behavior_planner', '/frenet_planner',
                         '/mpc_controller'}.issubset(set(
                             subscriptions().get('/planning/vehicle/state', []))),
                'Behavior, Frenet and MPC share private vehicle state')
        require(lambda: {'/behavior_planner', '/frenet_planner'}.issubset(set(
                    subscriptions().get('/planning/objects', []))),
                'Behavior and Frenet subscribe to private objects')
        assert '/frenet_planner' not in subscriptions().get('/perception/objects', [])
        print('PASS Frenet does not consume frozen public DetectedObject type', flush=True)

        received = []
        ready = threading.Event()
        def callback(message):
            if message.objects and message.objects[0].id == 731:
                received.append(message)
                ready.set()
        sub = rospy.Subscriber('/planning/objects', DetectedObjectArray,
                               callback, queue_size=2)
        pub = rospy.Publisher('/last_lidar', LegacyTrackedObjectArray, queue_size=2)
        require(lambda: pub.get_num_connections() > 0, 'legacy LiDAR wire subscriber connects')
        frame = LegacyTrackedObjectArray()
        frame.header.frame_id = 'map'
        obj = TrackedObject()
        obj.id, obj.age, obj.miss_count = 731, 1, 0
        obj.classification = 'car'
        obj.cluster_center = Point(100, 100, 0.5)
        obj.obb_center = Point(100, 100, 0.5)
        obj.length, obj.width, obj.height = 4, 2, 1
        obj.velocity_mps.x, obj.speed_kph = 2.5, 9
        obj.obb_corners = [Point(x, y, z) for z in (0, 1)
                           for x, y in ((98, 99), (102, 99), (102, 101), (98, 101))]
        frame.objects = [obj]
        deadline = time.monotonic() + 10
        while not ready.is_set() and time.monotonic() < deadline:
            frame.header.stamp = rospy.Time.now()
            pub.publish(frame)
            ready.wait(0.1)
        assert ready.is_set(), 'legacy wire frame did not reach private planning output'
        converted = received[0]
        assert converted.header.frame_id == 'map' and converted.header.stamp.to_sec() > 0
        output = converted.objects[0]
        assert output.id == obj.id and output.center == obj.obb_center
        assert output.obb_corners == obj.obb_corners and output.vx == 2.5
        assert output.motion_state == DetectedObject.MOTION_UNKNOWN
        print('PASS legacy wire -> private objects preserves header, OBB and unknown velocity', flush=True)
        pub.unregister()
        sub.unregister()
    finally:
        if launch.poll() is None:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait(timeout=5)


if __name__ == '__main__':
    main()
