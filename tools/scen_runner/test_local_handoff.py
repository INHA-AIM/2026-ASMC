#!/usr/bin/env python3
"""Real selector: prepared entry, bounded wait, return, expiry and legacy fallback."""
import os
from pathlib import Path
import tempfile
import time
import subprocess
import signal
from scen_runner.runtime.private_master import PrivateMaster

with tempfile.TemporaryDirectory(prefix='local-handoff-') as folder:
    with PrivateMaster(Path(folder), 11348):
        import rospy
        from std_msgs.msg import UInt8
        from morai_msgs.msg import CtrlCmd
        from asmc_msgs.msg import LocalTrajectory, TrajectoryPoint
        rospy.init_node('handoff_test', disable_signals=True)
        output=[]
        sub=rospy.Subscriber('/ctrl_cmd',CtrlCmd,lambda m: output.append((time.monotonic(),m)),queue_size=100)
        mode=rospy.Publisher('/planning/mode',UInt8,queue_size=1,latch=True)
        normal=rospy.Publisher('/ctrl_cmd/normal',CtrlCmd,queue_size=1)
        local=rospy.Publisher('/ctrl_cmd/jamming',CtrlCmd,queue_size=1)
        plans=rospy.Publisher('/planning/local/trajectory',LocalTrajectory,queue_size=1)
        log=open(Path(folder)/'selector.log','w')
        child=subprocess.Popen(['rosrun','asmc_planner','asmc_control_selector_node',
            '_local_handoff_enabled:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            time.sleep(1)
            mode.publish(UInt8(0))
            for i in range(12):
                p=LocalTrajectory();p.valid=True;p.header.stamp=rospy.Time.now();p.header.frame_id='local/fixture/0';p.scene_stamp=p.header.stamp;p.plan_sequence=i
                p.points=[TrajectoryPoint(),TrajectoryPoint()]
                plans.publish(p)
                normal.publish(CtrlCmd(longlCmdType=1,accel=.1,steering=.2))
                time.sleep(.02)
            output.clear()
            mode.publish(UInt8(1));time.sleep(.065)
            assert not output, [(t,m.brake) for t,m in output]
            local.publish(CtrlCmd(longlCmdType=1,accel=.1,steering=.15));time.sleep(.025)
            assert output and all(m.brake<.9 for _,m in output), output
            print('Prepared entry: no inserted stop or old-command republish PASS')
            # Normal commands may arrive while unselected; only the selected
            # local command refreshes the active-source watchdog.
            normal.publish(CtrlCmd(longlCmdType=1,accel=.1,steering=.2));time.sleep(.01)
            output.clear();mode.publish(UInt8(0));time.sleep(.055)
            assert not output, output
            normal.publish(CtrlCmd(longlCmdType=1,accel=.1,steering=.2));time.sleep(.025)
            assert output and all(m.brake<.9 for _,m in output), output
            print('Prepared return: next NORMAL command takes over without an inserted stop PASS')
            # Fresh preparation exists, but the selected controller never responds.
            p.header.stamp=rospy.Time.now();p.scene_stamp=p.header.stamp;plans.publish(p)
            normal.publish(CtrlCmd(longlCmdType=1,accel=.1));time.sleep(.025)
            output.clear();begin=time.monotonic();mode.publish(UInt8(1));time.sleep(.19)
            stops=[t for t,m in output if m.brake>.9]
            assert stops and .075 <= stops[0]-begin <= .18, (begin,stops)
            print('Missing LOCAL command: bounded100ms wait, watchdog stop PASS')
            # Unselected traffic cannot keep a selected source alive.
            for _ in range(15):
                normal.publish(CtrlCmd(longlCmdType=1,accel=.9));time.sleep(.02)
            assert output[-1][1].brake>.9
            mode.publish(UInt8(0));time.sleep(.05)
            normal.publish(CtrlCmd(longlCmdType=1,accel=.1));time.sleep(.02)
            p.valid=False;plans.publish(p);time.sleep(.02)
            output.clear();begin=time.monotonic();mode.publish(UInt8(1));time.sleep(.06)
            assert any(m.brake>.9 for _,m in output), output
            print('Invalid preparation: no transition grace; no watchdog refresh by unselected traffic PASS')
        finally:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT);child.wait(timeout=8)
            log.close()
