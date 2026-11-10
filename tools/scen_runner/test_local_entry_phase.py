#!/usr/bin/env python3
"""Reproduce a prepared source expiring at the next unrelated MPC timer phase."""
import json
import copy
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from scen_runner.runtime.private_master import PrivateMaster


def main():
    with tempfile.TemporaryDirectory(prefix='local-entry-phase-') as folder:
        with PrivateMaster(Path(folder),11348):
            import rospy
            import rospkg
            from asmc_msgs.msg import LocalTrajectory,TrajectoryPoint
            from morai_msgs.msg import CtrlCmd
            from nav_msgs.msg import Odometry
            from std_msgs.msg import String,UInt8
            rospy.init_node('entry_phase_test',disable_signals=True)
            modes=rospy.Publisher('/planning/mode',UInt8,queue_size=1,latch=True)
            states=rospy.Publisher('/planning/local/odometry',Odometry,queue_size=1)
            plans=rospy.Publisher('/planning/local/trajectory',LocalTrajectory,queue_size=1)
            normal=rospy.Publisher('/ctrl_cmd/normal',CtrlCmd,queue_size=1)
            traces=[];commands=[]
            subs=[rospy.Subscriber('/planning/local/control_trace',String,
                    lambda m:traces.append((time.monotonic(),json.loads(m.data))),queue_size=100),
                  rospy.Subscriber('/ctrl_cmd/jamming',CtrlCmd,
                    lambda m:commands.append((time.monotonic(),m)),queue_size=100)]
            children=[];logs=[]
            def start(cmd,name):
                f=(Path(folder)/name).open('w');logs.append(f)
                child=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
                children.append(child);return child
            def wait_for(check,seconds=3):
                until=time.monotonic()+seconds
                while not check():
                    if time.monotonic()>until:raise AssertionError('timeout: '+str(traces[-3:]))
                    time.sleep(.001)
            seq=0
            def feed(age=0.,future_state=0.):
                nonlocal seq
                seq+=1
                now=rospy.Time.now()
                state=Odometry();state.header.stamp=now;state.header.frame_id='local/entry/0'
                state.child_frame_id='base_link';state.pose.pose.orientation.w=1.;state.twist.twist.linear.x=1.
                plan=LocalTrajectory();plan.header=copy.deepcopy(state.header);plan.ego_at_plan=copy.deepcopy(state)
                plan.valid=True;plan.reason='ok';plan.plan_sequence=seq;plan.dt=.05
                plan.scene_stamp=now-rospy.Duration(age)
                for i in range(100):
                    q=TrajectoryPoint();q.x=i*.05;q.velocity=1.;plan.points.append(q)
                state.header.stamp=now+rospy.Duration(future_state)
                states.publish(state);plans.publish(plan)
                normal.publish(CtrlCmd(longlCmdType=1,accel=.1))
            try:
                start(['rosrun','asmc_planner','asmc_control_selector_node','_local_handoff_enabled:=true'],'selector.log')
                config=rospkg.RosPack().get_path('asmc_controller')+'/config/controller.yaml'
                start(['rosrun','asmc_controller','asmc_local_controller_node','_enable_control:=true',
                       '_normal_controller_node:=/entry_phase_test','_selector_node:=/asmc_control_selector',
                       '_controller_config:='+config],'controller.log')
                wait_for(lambda:states.get_num_connections() and plans.get_num_connections() and modes.get_num_connections()>=2)
                modes.publish(UInt8(0))
                for _ in range(60):feed();time.sleep(.025)
                latencies=[]
                for case in range(6):
                    modes.publish(UInt8(0));feed();time.sleep(.065)
                    # Observe an idle timer tick, then switch just after it.
                    count=len(traces)
                    wait_for(lambda:len(traces)>count and traces[-1][1]['reason']=='inactive_normal_mode')
                    # Last three entries have a tiny future motion sample:
                    # wait for its time, never solve from that future state.
                    feed(.323,.004 if case>=3 else 0.);time.sleep(.001 if case>=3 else .004)
                    commands.clear();traces.clear();begin=time.monotonic();modes.publish(UInt8(1))
                    wait_for(lambda:bool(commands) and any(r[1]["valid"] for r in traces),seconds=.1)
                    latency=(commands[0][0]-begin)*1000
                    assert commands[0][1].brake<.9,(latency,traces)
                    assert latency<30,(latency,traces)
                    assert any(r['valid'] and r['scene_age_s']<=.35 for _,r in traces),traces
                    assert all(r['state_age_s']>=0 for _,r in traces if r['valid']),traces
                    latencies.append(latency)
                    feed();time.sleep(.06)
                # Expired preparation must still fail; first-solve scheduling
                # cannot re-stamp or extend the ordinary350ms contract.
                modes.publish(UInt8(0));feed();time.sleep(.065)
                feed(.38);time.sleep(.005);commands.clear();traces.clear();modes.publish(UInt8(1))
                wait_for(lambda:bool(commands),seconds=.15)
                assert commands[0][1].brake>.9
                assert any(r['reason']=='local_source_stale' for _,r in traces),traces
                print(json.dumps({'mode_to_first_command_ms':latencies,'expired_source_rejected':True}))
            finally:
                for child in reversed(children):
                    if child.poll() is None:
                        os.killpg(child.pid,signal.SIGINT)
                        try:child.wait(timeout=8)
                        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
                for f in logs:f.close()

if __name__=='__main__':main()
