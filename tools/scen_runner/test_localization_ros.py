#!/usr/bin/env python3
"""Real ROS boundary checks, exclusively in a network-none test container.

Launches its own master and position node; --integrated adds controllers and
the selector. Never launches a bridge or connects to the shared master.
Requires an unused loopback port and a fresh registered-run subdirectory.
"""
import argparse,json,math,os,signal,socket,subprocess,time
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--integrated',action='store_true');a=p.parse_args()
    if os.environ.get('ROS_MASTER_URI')!='http://127.0.0.1:11361':
        raise RuntimeError('isolated test ROS_MASTER_URI required')
    with socket.socket() as s:
        if s.connect_ex(('127.0.0.1',11361))==0:raise RuntimeError('test master port occupied')
    from localization_run import active_registration
    registration=active_registration(a.output.parent)
    a.output.mkdir(exist_ok=False);processes=[];logs=[];checks=[]
    def launch(args,name):
        f=(a.output/(name+'.log')).open('x');logs.append(f)
        q=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(q);return q
    def check(condition,name):
        checks.append(dict(name=name,passed=bool(condition)));print(('PASS ' if condition else 'FAIL ')+name,flush=True)
        if not condition:raise AssertionError(name)
    try:
        launch(['roscore','-p','11361'],'master');time.sleep(2)
        import rospy,yaml
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from morai_msgs.msg import GPSMessage,CtrlCmd
        from asmc_msgs.msg import LidarTrackingObservation,LocalTrajectory,LidarObject
        from std_msgs.msg import String,UInt8
        from geometry_msgs.msg import Point
        from pyproj import Transformer
        from scen_runner.morai.signal_trial_coordinates import Coordinates
        rospy.init_node('position_boundary_fixture',disable_signals=True)
        config=yaml.safe_load(Path('/root/ws/src/asmc_planner/config/frenet/params.yaml').read_text())
        config['waypoint_file']='/root/ws/R_KR_PG_KATRI/2026_molit_comp_global_path.txt'
        config['avoid_hd_map']['directory']='/root/ws/src/asmc_planner/config/frenet/hd_map'
        cfg=a.output/'parameters.yaml';cfg.write_text(yaml.safe_dump(config))
        trace=[];plans=[];normal_commands=[];shadow_commands=[];control_trace=[]
        mode=rospy.Publisher('/planning/mode',UInt8,queue_size=1,latch=True) if a.integrated else None
        if mode:mode.publish(UInt8(0))
        rospy.Subscriber('/planning/localization/trace',String,lambda m:trace.append(json.loads(m.data)),queue_size=300)
        rospy.Subscriber('/planning/localization/trajectory',LocalTrajectory,lambda m:plans.append(m),queue_size=100)
        state=rospy.Publisher('/planning/local/odometry',Odometry,queue_size=4)
        imu=rospy.Publisher('/imu',Imu,queue_size=4);gps=rospy.Publisher('/gps',GPSMessage,queue_size=4)
        scan=rospy.Publisher('/perception/lidar/tracks_local',LidarTrackingObservation,queue_size=4)
        launch(['/devel/lib/asmc_planner/asmc_shadow_localization_node','_parameters:='+str(cfg),
                '_run_id:='+registration['run_id'],'_validate_recovery_window:=true',
                '_journal_path:='+str(a.output/'trace.jsonl'),'_mode_gated:='+str(a.integrated).lower()],'position')
        deadline=time.monotonic()+15
        while not all(pub.get_num_connections() for pub in (state,imu,gps,scan)):
            if time.monotonic()>deadline:raise RuntimeError('position subscriptions unavailable')
            time.sleep(.05)
        native_x,native_y,enu_yaw=43.76026,-476.4096,-136.55374
        yaw=Coordinates([302595.,4124145.]).grid_yaw(native_x,native_y,enu_yaw)
        inverse=Transformer.from_crs(32652,4326,always_xy=True)
        serial=0;last_scan=None;speed=0;shift=0;session='lidar-fixture-a';frame='local/fixture-a';config_id='c1';pitch=0
        def phase(seconds,fix=True,scan_mode='normal',pose_mode='normal'):
            nonlocal serial,last_scan
            start=len(trace);until=time.monotonic()+seconds
            while time.monotonic()<until:
                t=rospy.Time.now()-rospy.Duration(.025);serial+=1
                m=Odometry();m.header.stamp=t;m.header.frame_id=frame;m.child_frame_id='base_link';m.pose.pose.orientation.w=1
                m.twist.twist.linear.x=speed
                if pose_mode=='backward':m.header.stamp=t-rospy.Duration(1)
                state.publish(m)
                q=Imu();q.header.stamp=t;q.header.frame_id='imu';yaw_half=math.radians(enu_yaw)/2
                q.orientation.x=-math.sin(pitch/2)*math.sin(yaw_half);q.orientation.y=math.sin(pitch/2)*math.cos(yaw_half)
                q.orientation.z=math.cos(pitch/2)*math.sin(yaw_half);q.orientation.w=math.cos(pitch/2)*math.cos(yaw_half);imu.publish(q)
                if fix:
                    g=GPSMessage();g.header.stamp=t;g.status=1;g.eastOffset=302595.;g.northOffset=4124145.
                    mount_forward=.35*math.cos(pitch)+1.25*math.sin(pitch)
                    g.longitude,g.latitude=inverse.transform(native_x+shift+g.eastOffset+mount_forward*math.cos(yaw),native_y+g.northOffset+mount_forward*math.sin(yaw));gps.publish(g)
                if scan_mode=='duplicate' and last_scan is not None:scan.publish(last_scan)
                else:
                    o=LidarTrackingObservation();b=o.observation;b.producer_session_id=session;b.generation=1;b.scan_sequence=serial
                    b.config_id=config_id;b.calibration_id='k1';b.objects.header.stamp=t;b.objects.header.frame_id='base_link';b.objects.status=b.objects.OBSERVED_EMPTY
                    if scan_mode in ('box','predicted'):
                        obj=LidarObject();obj.track_id=12;obj.observation_state=obj.OBSERVED if scan_mode=='box' else obj.PREDICTED
                        obj.obb_corners=[Point(x,y,z) for x in (-8.5,-7.5) for y in (-.5,.5) for z in (.5,1.5)]
                        b.objects.status=b.objects.OBSERVED;b.objects.objects=[obj]
                    if scan_mode=='invalid':b.objects.status=b.objects.INPUT_INVALID
                    if scan_mode=='future':b.objects.header.stamp=t+rospy.Duration(20);b.scan_sequence+=100000
                    scan.publish(o)
                    if scan_mode=='normal':last_scan=o
                time.sleep(.05)
            time.sleep(.02)
            return trace[start:]
        if a.integrated:
            from rosgraph.masterapi import Master
            rospy.Subscriber('/ctrl_cmd/normal',CtrlCmd,lambda m:normal_commands.append(m),queue_size=100)
            rospy.Subscriber('/ctrl_cmd/jamming',CtrlCmd,lambda m:shadow_commands.append(m),queue_size=100)
            rospy.Subscriber('/planning/local/control_trace',String,lambda m:control_trace.append(json.loads(m.data)),queue_size=100)
            launch(['/devel/lib/asmc_controller/mpc_controller_node','_mpc/config_path:=/root/ws/src/asmc_controller/config/controller.yaml','_suspend_in_jamming:=true'],'normal-controller')
            local=launch(['/devel/lib/asmc_controller/asmc_local_controller_node','__name:=localization_controller',
                '_enable_control:=true','_integrated_normal_controller:=/mpc_controller',
                '_controller_config:=/root/ws/src/asmc_controller/config/controller.yaml',
                '/planning/local/trajectory:=/planning/localization/trajectory'],'shadow-controller')
            launch(['/devel/lib/asmc_planner/asmc_control_selector_node','__name:=asmc_control_selector'],'selector')
            rows=phase(2)
            check(any(r['anchor_ready'] for r in rows),'anchor warms while NORMAL owns control')
            check(rows and all(r['reason']=='inactive_normal' and r['speed_profiles']==0 for r in rows),'unselected shadow reference calculation suspended')
            check(normal_commands and not shadow_commands,'NORMAL commands alone before transition')
            check(local.poll() is None,'named NORMAL peer accepted without disabling duplicate protection')
            mode.publish(UInt8(1));phase(.3);n=len(normal_commands);rows=phase(1)
            check(len(normal_commands)==n,'NORMAL MPC publishing suspended in JAMMING')
            check(shadow_commands and any(r['valid'] for r in control_trace),'shadow MPC accepts real LocalTrajectory in JAMMING')
            publishers=dict(Master(rospy.get_name()).getSystemState()[0])
            expected={'/ctrl_cmd':['/asmc_control_selector'],'/ctrl_cmd/normal':['/mpc_controller'],'/ctrl_cmd/jamming':['/localization_controller']}
            check(all(sorted(publishers.get(t,[]))==v for t,v in expected.items()),'one final selector and one controller per routing slot')
            (a.output/'owners.json').write_text(json.dumps(expected,indent=2)+'\n')
            mode.publish(UInt8(0));phase(.3);n=len(shadow_commands);c=len(control_trace);m=len(normal_commands);rows=phase(.5)
            check(len(shadow_commands)==n and len(control_trace)==c,'shadow MPC solving and publishing suspended after NORMAL return')
            check(len(normal_commands)>m and all(r['reason']=='inactive_normal' for r in rows),'NORMAL resumes while shadow anchor stays warm')
            launch(['python3','-c',"import rospy;from morai_msgs.msg import CtrlCmd;rospy.init_node('unexpected_jamming_owner');p=rospy.Publisher('/ctrl_cmd/jamming',CtrlCmd,queue_size=1);rospy.sleep(4)"],'conflict-probe')
            phase(2)
            check(local.poll() is not None,'a second JAMMING owner terminates only the research controller')
            return 0
        rows=phase(1.6);check(any(r['valid'] and r['anchor_ready'] for r in rows),'initial aligned warmup releases real LocalTrajectory')
        check(all(not r['valid'] for r in rows if r['stamp']<rows[0]['stamp']+.9),'no premature valid plan during warmup')
        speed=-.004;before_plans=len(plans);rows=phase(.3)
        check(rows and all(r['valid'] for r in rows) and any(r['speed_kph']<0 for r in rows) and
              any(p.valid and all(q.velocity>=0 for q in p.points) for p in plans[before_plans:]),
              'signed standstill rollback remains observed while forward plan samples stay nonnegative')
        speed=1
        rows=phase(1.2,fix=False);check(any(r['source']=='dead_reckoning' and r['valid'] for r in rows),'GPS loss retains valid DR trajectory')
        rows=phase(2.6);check({'recovery_verify','recovery_blend','gps_anchored'}<=set(r['source'] for r in rows),'real ROS GPS recovery verifies then blends then reconnects')
        shift=.3;rows=phase(.1)
        check(any(r['source']=='recovery_verify' and r['valid'] for r in rows),
              'noisy tracking fix retains the validated DR pose while verifying')
        shift=0;rows=phase(2.6)
        check(all(r['valid'] for r in rows) and rows[-1]['source']=='gps_anchored',
              'isolated tracking outlier validates and blends without a stop')
        shift=.3;rows=phase(1.3)
        check(rows and all(not r['valid'] for r in rows[-3:]),
              'persistent tracking candidate over 25cm invalidates while moving')
        speed=0;shift=0;rows=phase(1.5)
        check(rows[-1]['valid'],'persistent jump requires stopped stable reinitialization')
        speed=1;shift=.7;rows=phase(.4)
        check(rows and all(not r['valid'] for r in rows[2:]),'gross tracking jump stops immediately')
        speed=0;shift=0;rows=phase(1.5);check(rows[-1]['valid'],'stopped one second reinitialization restores valid plan')
        rows=phase(.2,scan_mode='invalid');check(any(not r['valid'] and r['objects_reason']=='invalid_scan_contract' for r in rows),'invalid LiDAR is not observed free space')
        phase(.5);rows=phase(.15,scan_mode='duplicate');check(len({r['obb_scan_transforms'] for r in rows})==1,'duplicate scan does not transform OBB again')
        phase(.3);phase(.1,scan_mode='future');rows=phase(.5);check(rows[-1]['valid'],'future scan does not poison producer watermark')
        pitch=.2;rows=phase(.3,scan_mode='box');expected=-8.5*math.cos(.2)+.5*math.sin(.2)
        check(any(abs(r['first_obb_world_xmin']-expected)<1e-6 and r['objects']==1 for r in rows),'raw body OBB receives measured pitch exactly once in ROS path')
        rows=phase(.2,scan_mode='predicted');check(any(not r['valid'] and r['objects_reason']=='object_not_observed' for r in rows),'predicted object cannot become observed free space')
        pitch=0;phase(.3)
        speed=1;config_id='c2';phase(.2,scan_mode='invalid');rows=phase(.3);check(any(not r['valid'] and r['recovery_state']=='reanchor_stop' for r in rows),'configuration epoch requires stopped reanchor')
        speed=0;phase(1.5);session='lidar-fixture-b';rows=phase(.2);check(any(not r['valid'] for r in rows),'LiDAR restart invalidates old anchor/cache')
        phase(1.5);session='lidar-fixture-a';rows=phase(.15);check(rows[-1]['object_session']=='lidar-fixture-b','retired producer cannot replace current identity')
        session='lidar-fixture-b';phase(.3);rows=phase(.1,pose_mode='backward');rows+=phase(.3)
        check(any(not r['valid'] and r['recovery_state']=='reanchor_stop' for r in rows),'backward odometry creates invalid alignment epoch')
        check(all(r['integration_calls']==0 and r['inference_calls']==0 for r in trace),'consumer adds no motion integrator or inference')
        check(all(r['obb_attitude_applications']==1 for r in trace),'single attitude application declared throughout trace')
        check(bool(plans) and all(p.header.frame_id.startswith('local/') and p.dt==.05 for p in plans),'LocalTrajectory frame and time-step contract preserved')
    finally:
        for q in reversed(processes):
            if q.poll() is None:
                os.killpg(q.pid,signal.SIGINT)
                try:q.wait(5)
                except subprocess.TimeoutExpired:os.killpg(q.pid,signal.SIGTERM);q.wait(5)
        for f in logs:f.close()
        (a.output/'results.json').write_text(json.dumps(checks,indent=2)+'\n')
    return 0

if __name__=='__main__':raise SystemExit(main())
