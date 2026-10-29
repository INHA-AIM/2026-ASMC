#!/usr/bin/env python3
"""Expanded launch ownership and real selector/LOCAL switching on a private master."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time


def check_launches():
    import roslaunch.config
    import rospkg

    path = rospkg.RosPack().get_path('asmc_planner') + '/launch/autonomy_no_bridges_v2.launch'
    def load(*args):
        return roslaunch.config.load_config_default([(path, list(args))], None)
    baseline = load('dashboard:=false')
    local = load('jamming_planner:=local', 'dashboard:=false')
    for config in (baseline, local):
        names = [n.namespace + n.name for n in config.nodes]
        assert len(names) == len(set(names)), names
        assert names.count('/asmc_control_selector') == 1
        assert names.count('/mpc_controller') == 1
        assert not any(n.type in ('lidar_pipeline_node', 'morai_udp_bridge.py',
                                 'morai_sensor_bridge.py') for n in config.nodes)
    assert not any(n.name in ('local_planner', 'local_controller', 'semantic_space_trial')
                   for n in baseline.nodes)
    assert any(n.package == 'lane_boundary_adapter' for n in baseline.nodes)
    assert not any(n.package == 'lane_boundary_adapter' for n in local.nodes)
    assert sum(n.name == 'local_controller' for n in local.nodes) == 1
    assert sum(n.name == 'local_planner' for n in local.nodes) == 1
    assert sum(n.name == 'semantic_space_trial' for n in local.nodes) == 1
    for key, value in {'/local_controller/enable_control': True,
                       '/local_controller/normal_controller_node': '/mpc_controller',
                       '/local_controller/selector_node': '/asmc_control_selector',
                       '/local_planner/active_in_jamming_only': True,
                       '/local_planner/speed_cap_kph': 10,
                       '/asmc_control_selector/smooth_handoff': True,
                       '/planning_mode/approach_enabled': True}.items():
        assert local.params[key].value == value, (key, local.params[key].value)
    external = load('jamming_planner:=local', 'local_semantic_input:=false')
    assert not any(n.name == 'semantic_space_trial' for n in external.nodes)
    try:
        load('jamming_planner:=typo')
    except (roslaunch.RLException, KeyError):
        pass
    else:
        raise AssertionError('unknown mode was accepted')
    full_path = rospkg.RosPack().get_path('asmc_planner') + '/launch/autonomy.launch'
    for args in ([], ['jamming_planner:=local']):
        full = roslaunch.config.load_config_default([(full_path,args)],None)
        names = [n.namespace+n.name for n in full.nodes]
        assert len(names) == len(set(names)), names
        for name in ('/morai_udp_bridge','/morai_sensor_bridge','/camera_objects',
                     '/camera_lane','/perception/lidar/lidar_pipeline',
                     '/perception/lidar/lidar_monitor','/behavior_dashboard_server',
                     '/asmc_control_selector','/planning_gps_imu_state_adapter'):
            assert names.count(name) == 1, (name,names)
        assert '/gps_imu_vehicle_state_adapter' not in names
        assert full.params['/behavior_dashboard_server/port'].value == 8089
        assert full.params['/perception/lidar/lidar_monitor/port'].value == 8097
        assert ('/local_controller' in names) == bool(args)
        assert any(n.package == 'lane_boundary_adapter' for n in full.nodes) == (not args)
    print('Full single-container composition: canonical ports and exactly one stack PASS')
    print('Expanded autonomy: exclusive follower/LOCAL, semantic input, defaults and typo rejection PASS')


def main():
    check_launches()
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    os.environ['ROS_HOSTNAME'] = '127.0.0.1'
    import rosgraph
    import rospy
    import rospkg
    from asmc_msgs.msg import LocalTrajectory, TrajectoryPoint
    from morai_msgs.msg import CtrlCmd
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, UInt8

    children, logs = [], []
    with tempfile.TemporaryDirectory(prefix='local-autonomy-') as folder:
        def start(args, name):
            f = open(Path(folder) / name, 'w')
            logs.append(f)
            child = subprocess.Popen(args, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
            children.append(child)
            return child
        try:
            start(['roscore', '-p', str(port)], 'master.log')
            master = rosgraph.Master('/local_autonomy_test')
            for _ in range(80):
                try:
                    master.getPid()
                    break
                except Exception:
                    time.sleep(.1)
            else:
                raise RuntimeError('private master unavailable')
            rospy.init_node('local_autonomy_test', disable_signals=True)
            mode = rospy.Publisher('/planning/mode', UInt8, queue_size=1, latch=True)
            normal = rospy.Publisher('/ctrl_cmd/normal', CtrlCmd, queue_size=1)
            states = rospy.Publisher('/planning/local/odometry', Odometry, queue_size=1)
            plans = rospy.Publisher('/planning/local/trajectory', LocalTrajectory, queue_size=1)
            commands, output, traces = [], [], []
            subs = [rospy.Subscriber('/ctrl_cmd/jamming', CtrlCmd, commands.append, queue_size=100),
                    rospy.Subscriber('/ctrl_cmd', CtrlCmd, output.append, queue_size=100),
                    rospy.Subscriber('/planning/local/control_trace', String,
                                     lambda m: traces.append(json.loads(m.data)), queue_size=100)]
            start(['rosrun', 'asmc_planner', 'asmc_control_selector_node'], 'selector.log')
            config = rospkg.RosPack().get_path('asmc_controller') + '/config/controller.yaml'
            controller = start(['rosrun', 'asmc_controller', 'asmc_local_controller_node',
                                '_enable_control:=true', '_normal_controller_node:=/local_autonomy_test',
                                '_selector_node:=/asmc_control_selector', '_controller_config:=' + config],
                               'controller.log')
            time.sleep(1.3)
            assert controller.poll() is None
            seq = 0
            def feed(seconds):
                nonlocal seq
                until = time.monotonic() + seconds
                while time.monotonic() < until:
                    seq += 1
                    now = rospy.Time.now()
                    state = Odometry()
                    state.header.stamp, state.header.frame_id = now, 'local/test/0'
                    state.child_frame_id = 'base_link'
                    state.pose.pose.orientation.w = 1.
                    state.twist.twist.linear.x = 1.
                    plan = LocalTrajectory()
                    plan.header, plan.ego_at_plan, plan.scene_stamp = state.header, state, now
                    plan.valid, plan.reason, plan.plan_sequence, plan.dt = True, 'ok', seq, .05
                    for i in range(100):
                        point = TrajectoryPoint()
                        point.x, point.velocity = i * .05, 1.
                        plan.points.append(point)
                    states.publish(state)
                    plans.publish(plan)
                    normal.publish(CtrlCmd(longlCmdType=1, accel=.123, steering=.456))
                    time.sleep(.03)
            mode.publish(UInt8(0))
            time.sleep(.15)
            commands.clear(); output.clear(); traces.clear()
            feed(.8)
            assert not commands and any(r['reason'] == 'inactive_normal_mode' for r in traces)
            assert any(abs(m.steering - .456) < 1e-9 for m in output)
            assert all(not r['command_published'] for r in traces)
            print('NORMAL: local solver/publication idle, normal command selected PASS')
            mode.publish(UInt8(1))
            time.sleep(.15)
            commands.clear(); output.clear(); traces.clear()
            feed(1.3)
            assert any(r['valid'] and r['command_published'] for r in traces), traces[-5:]
            assert commands and output
            assert all(abs(m.steering - .456) > 1e-6 for m in output)
            assert dict(master.getSystemState()[0])['/ctrl_cmd'] == ['/asmc_control_selector']
            print('JAMMING: real LOCAL MPC selected; normal command blocked; one wire owner PASS')
            mode.publish(UInt8(0))
            time.sleep(.15)
            commands.clear(); output.clear(); traces.clear()
            feed(.7)
            assert not commands and any(abs(m.steering - .456) < 1e-9 for m in output)
            print('Return to NORMAL: LOCAL stops publishing and normal resumes PASS')
            # Allowing a named NORMAL controller must not allow the old follower.
            foreign = rospy.Publisher('/ctrl_cmd/jamming', CtrlCmd, queue_size=1)
            controller.wait(timeout=5)
            log = (Path(folder) / 'controller.log').read_text()
            assert '/ctrl_cmd/jamming owned by /local_autonomy_test' in log, log
            foreign.unregister()
            print('Integrated mode still rejects competing jamming publisher PASS')
        finally:
            for child in reversed(children):
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGINT)
                    try:
                        child.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGTERM)
                        child.wait(timeout=5)
            for f in logs:
                f.close()

if __name__ == '__main__':
    main()
