#!/usr/bin/env python3
"""Exercise actual private topic subscriptions and legacy ROS wire conversion."""
import copy
import json
import os
import signal
import subprocess
import threading
import time

import rosgraph
import rospy
from asmc_msgs.msg import TrackedObject, TrackedObjectArray
from geometry_msgs.msg import Point
from std_msgs.msg import String
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
        'objects_source:=lidar', 'lidar_source_policy:=main_raw',
        'lidar_input_topic:=/lidar/tracked_objects_map', 'dashboard:=false', 'traffic_light:=false'], start_new_session=True)
    extra_processes = []
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
        pub = rospy.Publisher('/lidar/tracked_objects_map', LegacyTrackedObjectArray, queue_size=2)
        require(lambda: pub.get_num_connections() > 0, 'legacy LiDAR wire subscriber connects')
        frame = LegacyTrackedObjectArray()
        frame.header.frame_id = 'map'
        obj = TrackedObject()
        obj.id, obj.age, obj.miss_count = 731, 1, 0
        obj.classification = 'person'
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
        # Exercise private publication filtering with the real pinned catalog.
        reports = []
        dbg = rospy.Subscriber('/planning/objects_debug', String,
                               lambda msg: reports.append(json.loads(msg.data)), queue_size=20)
        guard = copy.deepcopy(obj)
        guard.id = 732
        guard.classification = guard.yolo_class = 'unknown'
        dx, dy = -4.0633 - obj.cluster_center.x, 134.2587 - obj.cluster_center.y
        guard.cluster_center.x += dx; guard.cluster_center.y += dy
        guard.obb_center.x += dx; guard.obb_center.y += dy
        for point in guard.obb_corners:
            point.x += dx; point.y += dy
        frame.objects = [obj, guard]
        deadline = time.monotonic() + 10
        matched = None
        while time.monotonic() < deadline:
            frame.header.stamp = rospy.Time.now()
            pub.publish(frame)
            time.sleep(0.1)
            for report in reports:
                match = next((r for r in report['objects'] if r['id'] == 732), None)
                if match is not None:
                    matched = match
            if matched is not None:
                break
        assert matched is not None, 'private filter diagnostics missing'
        assert not matched['keep'] and not matched['known_static_keep']
        assert matched['reference_id'] >= 0
        assert all(732 not in [o.id for o in msg.objects] for msg in received)
        print('PASS real known guardrail is excluded at private ROS publication boundary', flush=True)
        dbg.unregister()
        # A separate validated boundary must preserve the upstream decisions.
        extra_processes.append(subprocess.Popen([
            'rosrun','planning_inputs','tracked_wire_bridge.py',
            '__name:=validated_wire_test','_input_topic:=/test/last_lidar',
            '_output_topic:=/test/validated_tracks','_motion_debug_topic:=/test/motion_debug',
            '_source_policy:=validated_lidar'], start_new_session=True))
        extra_processes.append(subprocess.Popen([
            'rosrun','planning_inputs','planning_objects_adapter',
            '__name:=validated_objects_test','_input_topic:=/test/validated_tracks',
            '_output_topic:=/test/validated_objects','_source_policy:=validated_lidar'],
            start_new_session=True))
        validated_received=[]
        validated_sub=rospy.Subscriber('/test/validated_objects',DetectedObjectArray,
                                       validated_received.append,queue_size=10)
        validated_pub=rospy.Publisher('/test/last_lidar',LegacyTrackedObjectArray,queue_size=2)
        debug_pub=rospy.Publisher('/test/motion_debug',String,queue_size=2)
        require(lambda: validated_pub.get_num_connections()>0 and debug_pub.get_num_connections()>0,
                'validated frame and diagnostic subscribers connect')
        frame.objects=[obj]
        frame.header.stamp=rospy.Time.now()
        obj.velocity_mps.x=0.0
        validated_pub.publish(frame)
        time.sleep(0.3)
        assert not validated_received, 'missing motion diagnostics must not fabricate output'
        report={'stamp_secs':frame.header.stamp.secs,
                'stamp_nsecs':frame.header.stamp.nsecs,
                'objects':[{'id':731,'motion_state_code':1,'final_keep':True},
                           {'id':732,'motion_state_code':1,'final_keep':False}]}
        wrong=copy.deepcopy(report)
        wrong['stamp_nsecs']=(wrong['stamp_nsecs']+1)%1000000000
        debug_pub.publish(String(data=json.dumps(wrong)))
        time.sleep(0.3)
        assert not validated_received, 'mismatched timestamp must not supply a state'
        debug_pub.publish(String(data=json.dumps(report)))
        require(lambda: bool(validated_received), 'exact source-aligned validated input reaches planner')
        restored=validated_received[-1]
        assert len(restored.objects)==1 and restored.objects[0].id==731
        assert restored.objects[0].motion_state==DetectedObject.MOTION_LOW_THREAT
        assert restored.objects[0].obb_corners==obj.obb_corners
        assert restored.objects[0].center==obj.obb_center and restored.objects[0].vx==0.0
        print('PASS validated state, geometry, velocity and upstream guardrail deletion preserved',flush=True)
        frame.header.stamp=rospy.Time.now()
        frame.objects=[]
        report={'stamp_secs':frame.header.stamp.secs,'stamp_nsecs':frame.header.stamp.nsecs,
                'objects':[]}
        debug_pub.publish(String(data=json.dumps(report)))
        validated_pub.publish(frame)
        require(lambda: len(validated_received)>1 and not validated_received[-1].objects,
                'empty validated frame clears prior objects')
        validated_pub.unregister();debug_pub.unregister();validated_sub.unregister()
        pub.unregister()
        sub.unregister()
    finally:
        for process in extra_processes:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try: process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
        if launch.poll() is None:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait(timeout=5)


if __name__ == '__main__':
    main()
