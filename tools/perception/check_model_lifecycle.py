#!/usr/bin/env python3
"""GPU lifecycle acceptance on an isolated ROS master, without vehicle control."""
import argparse
import json
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import rospy
from sensor_msgs.msg import Image
from asmc_msgs.msg import CameraObjectBoxArray, TrafficLightDetection, LaneBoundaryFamily
from asmc_camera.msg import InferenceStatus
from asmc_camera.srv import SetInferenceMode

NODES = dict(twinlite='/camera_semantic_inference', yolo='/camera_object_detector',
             resnet18='/traffic_light_infer')


def rate_report(rows, started, ended):
    normal = [(t, stamp, valid) for t, stamp, valid in rows if started <= t <= ended]
    stamps = [stamp for _, stamp, _ in normal]
    times = [t for t, _, _ in normal]
    window_counts = [sum(a <= t < a + 5 for t in times)
                     for a in [started, started + 5] if a + 5 <= ended + .01]
    return dict(messages=len(times), hz=len(times)/(ended-started),
                valid_messages=sum(valid for _,_,valid in normal),
                first_second=sum(t < started+1 for t in times),
                five_second_hz=[n/5 for n in window_counts],
                max_gap_sec=max([b-a for a,b in zip([started]+times, times+[ended])] or [ended-started]),
                duplicate_stamps=len(stamps)-len(set(stamps)),
                reversed_stamps=sum(b<a for a,b in zip(stamps,stamps[1:])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--start-sec', type=float, default=44.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rospy.init_node('model_lifecycle_acceptance', anonymous=True)
    rospy.set_param('/use_sim_time', True)
    lock = threading.RLock()
    statuses, rows = {}, {}
    subscribers, processes, handles = [], [], []
    def status(model, msg):
        with lock:
            statuses[model] = msg
    def output(name, msg):
        with lock:
            rows.setdefault(name, []).append((time.monotonic(), msg.header.stamp.to_nsec(), bool(getattr(msg,"valid",True))))
    for model, node in NODES.items():
        subscribers.append(rospy.Subscriber(node+'/inference_status', InferenceStatus,
            lambda msg, model=model: status(model,msg), queue_size=50))
    topics = dict(yolo=('/perception/camera/objects',CameraObjectBoxArray),
                  resnet18=('/perception/traffic_light/detection',TrafficLightDetection),
                  family=('/perception/lane/boundary_family',LaneBoundaryFamily))
    for view in ['front','left','right']:
        topics[view] = ('/perception/camera/'+view+'/road_marking/class_id',Image)
    for name,(topic,kind) in topics.items():
        subscribers.append(rospy.Subscriber(topic,kind,lambda msg,name=name:output(name,msg),queue_size=100))
    def run(label, cmd):
        handle = open(args.output/(label+'.log'),'w')
        handles.append(handle)
        p = subprocess.Popen(cmd, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p)
        return p
    report = dict(stages=[], checks={}, failures=[])
    def wait_ready(model, timeout=30):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if model in statuses and statuses[model].loaded and statuses[model].warmed_up:
                return
            if camera.poll() is not None:
                raise RuntimeError('camera launch exited during model load')
            time.sleep(.1)
        raise RuntimeError('model did not load: '+model)
    def request(model,mode,hz):
        service=NODES[model]+'/set_inference_mode'
        rospy.wait_for_service(service,timeout=30)
        result=rospy.ServiceProxy(service,SetInferenceMode)(mode,hz)
        if not result.accepted:raise RuntimeError(result.reason)
    def stage(name,duration):
        start=time.monotonic()
        time.sleep(duration)
        end=time.monotonic()
        with lock:
            samples={k:rate_report(v,start,end) for k,v in rows.items()}
            states={k:dict(mode=v.mode,prepared=v.prepared,pipeline_ready=v.pipeline_ready,
                generation=v.generation,successful_batches=v.successful_batches,errors=v.errors,
                processing_hz=v.processing_hz,reason=v.reason) for k,v in statuses.items()}
        item=dict(name=name,started_monotonic=start,ended_monotonic=end,outputs=samples,statuses=states)
        report['stages'].append(item)
        return item
    try:
        camera=run('camera',['roslaunch','asmc_camera','perception.launch','profile:=planning',
            'start_camera_objects:=true','start_traffic_light:=true','traffic_light_wait_first:=0',
            'lane_enable_shared_family_debug:=false'])
        for model in NODES:wait_ready(model)
        run('bag',['rosbag','play',args.bag,'--clock','-s',str(args.start_sec),'-u','75','--quiet'])
        a=stage('all_standby',2)
        report['checks']['standby_no_compute']=all(s['successful_batches']==0 for s in a['statuses'].values())
        request('yolo',1,20)
        a=stage('yolo_prepare',1.5)
        report['checks']['yolo_prepare_no_operational_output']=a['outputs'].get('yolo',{}).get('messages',0)==0
        report['checks']['yolo_prepared']=a['statuses']['yolo']['prepared']
        request('yolo',2,20)
        a=stage('yolo_active',10)
        report['checks']['yolo_active_18hz']=a['outputs'].get('yolo',{}).get('hz',0)>=18
        report['checks']['other_models_remain_standby']=all(a['statuses'][k]['successful_batches']==0 for k in ('twinlite','resnet18'))
        request('yolo',0,0)
        stage('yolo_stop',.5)
        request('resnet18',1,10)
        a=stage('resnet_prepare',1.5)
        report['checks']['resnet_prepare_no_operational_output']=a['outputs'].get('resnet18',{}).get('messages',0)==0
        report['checks']['resnet_prepared']=a['statuses']['resnet18']['prepared']
        request('resnet18',2,10)
        a=stage('resnet_active',10)
        report['checks']['resnet_active_10hz']=all(x>=9.5 for x in a['outputs'].get('resnet18',{}).get('five_second_hz',[0])) and a['outputs'].get('resnet18',{}).get('first_second',0)>=9 and a['outputs'].get('resnet18',{}).get('max_gap_sec',1)<=.25
        request('resnet18',0,0)
        stage('resnet_stop',.5)
        request('twinlite',1,10)
        a=stage('twinlite_prepare',2)
        report['checks']['twinlite_prepared_all_views']=a['statuses']['twinlite']['prepared']
        request('twinlite',2,10)
        a=stage('twinlite_active',10)
        for name in ['front','left','right','family']:
            values=a['outputs'].get(name,{})
            report['checks']['twinlite_'+name+'_10hz']=all(x>=9.5 for x in values.get('five_second_hz',[0])) and values.get('first_second',0)>=9 and values.get('max_gap_sec',1)<=.25
        request('twinlite',0,0)
        stage('twinlite_stop_drain',.5)
        before={k:v['successful_batches'] for k,v in report['stages'][-1]['statuses'].items()}
        a=stage('all_standby_again',1)
        report['checks']['stop_compute']=all(a['statuses'][k]['successful_batches']==v for k,v in before.items())
        report['checks']['stop_operational_outputs']=all(v['messages']==0 for v in a['outputs'].values())
        report['checks']['duplicate_and_reverse_zero']=all(v['duplicate_stamps']==0 and v['reversed_stamps']==0
            for item in report['stages'] if item['name'].endswith('_active') for v in item['outputs'].values())
    except Exception as exc:
        report['failures'].append(str(exc))
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=15)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for handle in handles:handle.close()
        report['passed']=not report['failures'] and bool(report['checks']) and all(report['checks'].values())
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(passed=report['passed'],checks=report['checks'],failures=report['failures'])))
    return 0 if report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
