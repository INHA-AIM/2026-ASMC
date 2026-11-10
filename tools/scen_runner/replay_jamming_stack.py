#!/usr/bin/env python3
"""Run all three production stacks against raw bags, without a physical bridge.

Recorded ego motion is fixed: this verifies wiring and planning, not closed-loop
completion, collision freedom, physical handoff gap or comparative drive time.
"""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time

RAW_TOPICS=('/gps','/imu','/Ego_topic','/velodyne_points',
            '/image_jpeg/compressed','/cam_left/image_jpeg/compressed',
            '/cam_right/image_jpeg/compressed','/cam_extra/image_jpeg/compressed',
            '/morai_sensor_bridge/frame_timing')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('root',type=Path);p.add_argument('bag',type=Path)
    p.add_argument('--trial-prefix',required=True)
    p.add_argument('--master-port',type=int,default=11374)
    p.add_argument('--dashboard-port',type=int,default=8087)
    p.add_argument('--lidar-port',type=int,default=8098)
    p.add_argument('--rate',type=float,default=1.)
    p.add_argument('--strategy',choices=('camera','local','localization'),action='append')
    args=p.parse_args()
    if not .5<=args.rate<=1 or Path(args.trial_prefix).name!=args.trial_prefix:
        raise ValueError('bounded replay rate/prefix required')
    from localization_run import active_registration
    record=active_registration(args.root)
    checkout=Path(__file__).resolve().parents[2]
    output=args.root/'replays'/args.trial_prefix
    output.parent.mkdir(exist_ok=True);output.mkdir(exist_ok=False)
    for port in (args.master_port,args.dashboard_port,args.lidar_port):
        if not 1024<=port<=65535 or port==11311:raise ValueError('explicit private ports required')
        with socket.socket() as sock:
            if sock.connect_ex(('127.0.0.1',port))==0:raise RuntimeError('port occupied; owner left untouched: '+str(port))
    uri='http://127.0.0.1:'+str(args.master_port)
    os.environ.update(ROS_MASTER_URI=uri,ROS_HOSTNAME='127.0.0.1')
    import rosbag,rosgraph,rospy
    from std_msgs.msg import String,UInt8
    from morai_msgs.msg import CtrlCmd
    from diagnostic_msgs.msg import DiagnosticArray
    from asmc_msgs.msg import InferenceStatus
    from localization_observer import ExperimentReporter,ensure_dashboard
    with rosbag.Bag(str(args.bag)) as bag:
        first=bag.get_start_time();duration=bag.get_end_time()-first
        present=bag.get_type_and_topic_info()[1]
        missing=set(RAW_TOPICS[:-1])-set(present)
        if missing:raise ValueError('missing raw replay inputs: '+str(sorted(missing)))
    digest=hashlib.sha256()
    with args.bag.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):digest.update(chunk)
    provenance=dict(input_bag=str(args.bag),input_sha256=digest.hexdigest(),
        revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=checkout,text=True).strip(),
        duration_s=duration,rate=args.rate,physical_bridge=False,closed_loop_acceptance=False,
        dashboard_url='http://localhost:%d/web/index_v2.html'%args.dashboard_port,
        lidar_url='http://localhost:%d'%args.lidar_port,master_uri=uri,
        monitoring=True,resource_policy='offline container CPU/memory limits recorded separately')
    (output/'input.json').write_text(json.dumps(provenance,indent=2)+'\n')
    (output/'source.patch').write_bytes(subprocess.check_output(['git','diff','HEAD','--binary'],cwd=checkout))
    processes=[];logs=[]
    def start(command,path):
        stream=path.open('x');logs.append(stream)
        child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(child);return child
    def stop(child):
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(15)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM);child.wait(5)
    master_process=start(['roscore','-p',str(args.master_port)],output/'master.log')
    reporter=None
    try:
        master=rosgraph.Master('/jamming_raw_replay',master_uri=uri)
        deadline=time.monotonic()+10
        while True:
            try:master.getPid();break
            except OSError:
                if time.monotonic()>deadline:raise
                time.sleep(.1)
        master.setParam('/use_sim_time',True)
        rospy.init_node('jamming_raw_replay',disable_signals=True)
        from rosgraph_msgs.msg import Clock
        clock=rospy.Publisher('/clock',Clock,queue_size=1,latch=True)
        clock.publish(Clock(rospy.Time.from_sec(first)))
        monitoring=ensure_dashboard(args.root,args.dashboard_port,uri)
        viewer=start(['roslaunch','asmc_lidar','view.launch','dashboard_port:='+str(args.lidar_port)],output/'lidar-view.log')
        summaries=[]
        for strategy in args.strategy or ('camera','local','localization'):
            trial=output/strategy;trial.mkdir(exist_ok=False)
            occupied={t:n for t,n in master.getSystemState()[0] if n and t in
                      ('/gps','/velodyne_points','/ctrl_cmd','/planning/mode')}
            if occupied:raise RuntimeError('unexpected replay producer; not stopped: '+str(occupied))
            clock.publish(Clock(rospy.Time.from_sec(first)))
            reporter=ExperimentReporter(trial,record['run_id'],args.trial_prefix+'-'+strategy,
                'raw_bag_fixed_motion_not_driving',monitoring,
                conditions=dict(planner=strategy,input_bag=str(args.bag),rate=args.rate))
            rows=(trial/'observations.jsonl').open('x');counts=Counter();modes=[];reasons=Counter()
            record_lock=threading.Lock()
            def record_message(topic,message,rows=rows,counts=counts,modes=modes,reasons=reasons):
                counts[topic]+=1
                if isinstance(message,CtrlCmd):value=dict(accel=message.accel,brake=message.brake,steering=message.steering)
                elif isinstance(message,DiagnosticArray):
                    status=next((s for s in message.status if s.name=='camera_bbox_lane_change'),None)
                    value=dict(reason=status.message,valid=status.level==0,
                               values={e.key:e.value for e in status.values}) if status else {}
                elif isinstance(message,InferenceStatus):
                    value=dict(model=message.model,loaded=message.loaded,warmed=message.warmed_up,
                               successful_batches=message.successful_batches)
                elif isinstance(message,UInt8):
                    value=message.data
                    if not modes or modes[-1]!=value:modes.append(value)
                else:
                    try:value=json.loads(message.data)
                    except (ValueError,AttributeError):value={}
                    if isinstance(value,dict) and 'reason' in value:reasons[str(value['reason'])]+=1
                rows.write(json.dumps(dict(topic=topic,ros_time=rospy.Time.now().to_sec(),monotonic=time.monotonic(),value=value))+'\n')
            def observe(topic,message,record=record_message,lock=record_lock,rows=rows):
                with lock:
                    if not rows.closed:record(topic,message)
            subscribers=[rospy.Subscriber(t,k,lambda m,t=t,callback=observe:callback(t,m),queue_size=50) for t,k in
                (('/ctrl_cmd',CtrlCmd),('/planning/mode',UInt8),('/planning/mode/status',String),
                 ('/planning/jamming/handoff_trace',String),('/planning/local/trace',String),
                 ('/planning/local/control_trace',String),('/planning/localization/trace',String),
                 ('/planning/localization/control_trace',String),
                 ('/planning/camera_bbox_lane_change/diagnostics',DiagnosticArray),
                 ('/perception/camera/objects/status',InferenceStatus),('/perception/camera/lane/status',InferenceStatus))]
            camera=start(['roslaunch','asmc_camera','camera.launch'],trial/'camera.log')
            lidar=start(['roslaunch','asmc_lidar','lidar.launch','debug:=true'],trial/'lidar.log')
            # Recorded transport is deliberately a named replay producer.
            # Authorize it only in this private wrapper; production retains
            # /morai_udp_bridge and all source validity/epoch checks.
            wrapper=trial/'replay.launch'
            wrapper.write_text('''<launch>
<include file="$(find asmc_planner)/launch/autonomy_no_bridges_v2.launch">
<arg name="jamming_planner" value="%s"/><arg name="dashboard" value="false"/>
<arg name="shadow_run_id" value="%s"/></include>
<param name="/planning_competition_speed_state/speed_callerid" value="/jamming_raw_input"/>
<param name="/local_planner/speed_callerid" value="/jamming_raw_input"/>
<param name="/localization_motion/speed_callerid" value="/jamming_raw_input"/>
</launch>'''%(strategy,record['run_id']))
            autonomy=start(['roslaunch',str(wrapper)],trial/'autonomy.log')
            player=None
            try:
                # The CPU-limited offline runtime must finish loading its map
                # before playback; a sleeping roslaunch is not planner readiness.
                deadline=time.monotonic()+45
                while not dict(master.getSystemState()[0]).get('/planning/mode'):
                    if time.monotonic()>deadline:raise RuntimeError('normal planner/mode source startup deadline')
                    if any(child.poll() is not None for child in (camera,lidar,autonomy)):
                        raise RuntimeError('production launch exited during startup')
                    time.sleep(.1)
                graph=master.getSystemState()
                (trial/'graph-before.json').write_text(json.dumps(graph,indent=2))
                expected={'camera':'/camera_bbox_lane_change','local':'/local_controller','localization':'/localization_controller'}[strategy]
                pubs=dict(graph[0]);nodes={n for _,ns in graph[0]+graph[1] for n in ns}
                if pubs.get('/ctrl_cmd')!=['/asmc_control_selector'] or expected not in nodes:
                    raise RuntimeError('unexpected selected control graph')
                if not pubs.get('/planning/mode') or '/asmc_planner' not in nodes:
                    raise RuntimeError('normal planner/mode source failed; inspect assets and startup log')
                unwanted={'/camera_bbox_lane_change','/local_controller','/localization_controller'}-{expected}
                if unwanted.intersection(nodes):raise RuntimeError('unselected jamming controller is running')
                reporter.phase('running','fixed recorded ego motion; no physical bridge')
                player=start(['rosbag','play','__name:=jamming_raw_input','--clock','--rate',str(args.rate),'--topics',*RAW_TOPICS,
                              '--bags',str(args.bag)],trial/'play.log')
                deadline=time.monotonic()+duration/args.rate+30
                captures=set()
                while player.poll() is None:
                    if any(child.poll() is not None for child in (camera,lidar,autonomy)):
                        raise RuntimeError('replay production child exited')
                    if time.monotonic()>deadline:raise RuntimeError('bounded raw replay deadline')
                    if not dict(master.getSystemState()[0]).get('/planning/mode'):
                        raise RuntimeError('normal planner/mode source disappeared during replay')
                    elapsed=duration/args.rate+30-(deadline-time.monotonic())
                    for checkpoint in (5,20,40):
                        if elapsed>=checkpoint and checkpoint not in captures:
                            from urllib.request import urlopen
                            for reconnect in range(2):
                                with urlopen('http://127.0.0.1:%d/api/snapshot'%args.dashboard_port,timeout=2) as response:
                                    snapshot=json.load(response)
                                (trial/('web-%ds-%d.json'%(checkpoint,reconnect))).write_text(json.dumps(snapshot))
                                identity=snapshot.get('experiment') or {}
                                if identity.get('trial_id')!=args.trial_prefix+'-'+strategy or identity.get('master_uri')!=uri:
                                    raise RuntimeError('observer reconnect identity mismatch')
                            captures.add(checkpoint)
                    time.sleep(.2)
                if player.returncode:raise RuntimeError('raw bag player failed')
                reporter.phase('completed','replay ended; closed-loop driving remains untested')
            finally:
                for child in (player,autonomy,lidar,camera):
                    if child is not None:stop(child)
                for subscriber in subscribers:subscriber.unregister()
                with record_lock:rows.close()
                reporter.finish('stopped','replay processes ended');reporter=None
            summary=dict(strategy=strategy,counts=dict(counts),modes=modes,reasons=dict(reasons),
                         input_sha256=provenance['input_sha256'],closed_loop_acceptance=False)
            (trial/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');summaries.append(summary)
        (output/'summary.json').write_text(json.dumps(summaries,indent=2)+'\n')
    finally:
        if reporter:reporter.finish('failed','replay interrupted')
        # Only the observer this process started on its initially empty port.
        try:
            import rosnode
            rosnode.kill_nodes(['/behavior_dashboard_server'])
        except Exception:pass
        for child in reversed(processes):stop(child)
        for stream in logs:stream.close()
    print(json.dumps(dict(output=str(output),closed_loop_acceptance=False)))


if __name__=='__main__':main()
