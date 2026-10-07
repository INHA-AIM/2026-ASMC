#!/usr/bin/env python3
"""GPU lifecycle acceptance on an isolated master; never publishes vehicle control.

Trace runs diagnose server-accepted transitions. --no-trace measures the same
output gates without buffered instrumentation and uses the conservative RPC
start as its timing anchor. Preparation requires the acknowledged generation.
"""
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
from lifecycle_measurements import percentile, rate_report, trace_trial, trace_integrity, ten_hz_pass

NODES = dict(twinlite='/camera_semantic_inference', yolo='/camera_object_detector',
             resnet18='/traffic_light_infer')
OUTPUTS = dict(twinlite=['front','left','right','family'], yolo=['yolo'], resnet18=['resnet18'])
HZ = dict(twinlite=10., yolo=20., resnet18=10.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--start-sec', type=float, default=44.)
    parser.add_argument('--prepare-trials', type=int, default=10)
    parser.add_argument('--prepare-timeout', type=float, default=10.)
    parser.add_argument('--no-trace', action='store_true')
    parser.add_argument('--input-stop', action='store_true', help='freeze bag input/clock, reject preparation, then test ACTIVE reentry')
    args = parser.parse_args()
    if args.prepare_trials < 1 or args.prepare_timeout <= 0:
        parser.error('positive trial count and timeout required')
    args.output.mkdir(parents=True, exist_ok=False)
    rospy.set_param('/use_sim_time', True)
    rospy.init_node('model_lifecycle_acceptance', anonymous=True)
    lock = threading.RLock()
    statuses, rows, status_rows = {}, {}, []
    subscribers, processes, handles = [], [], []
    requests, stages = [], []
    report = dict(checks={}, failures=[], trace_enabled=not args.no_trace,
                  timing_anchor='server_acceptance' if not args.no_trace else 'rpc_start_conservative')
    camera = playback = None
    def status(model, msg):
        with lock:
            statuses[model] = msg
            status_rows.append(dict(model=model, arrival=time.monotonic(), mode=msg.mode,
                generation=msg.generation, prepared=msg.prepared, pipeline_ready=msg.pipeline_ready,
                successful_batches=msg.successful_batches, reason=msg.reason,
                source_cutoff_ns=msg.source_cutoff.to_nsec(), source_stamp_ns=msg.last_source_stamp.to_nsec()))
    def output(name, msg):
        with lock:
            rows.setdefault(name, []).append((time.monotonic(), msg.header.stamp.to_nsec(),
                                               bool(getattr(msg,'valid',True))))
    for model, node in NODES.items():
        subscribers.append(rospy.Subscriber(node+'/inference_status', InferenceStatus,
            lambda msg, model=model: status(model,msg), queue_size=100))
        rospy.set_param(node+'/lifecycle_trace_path', '' if args.no_trace else
                        str((args.output/(model+'.lifecycle.jsonl')).resolve()))
    topics = dict(yolo=('/perception/camera/objects',CameraObjectBoxArray),
                  resnet18=('/perception/traffic_light/detection',TrafficLightDetection),
                  family=('/perception/lane/boundary_family',LaneBoundaryFamily))
    for view in ['front','left','right']:
        topics[view] = ('/perception/camera/'+view+'/road_marking/class_id',Image)
    for name,(topic,kind) in topics.items():
        subscribers.append(rospy.Subscriber(topic,kind,lambda msg,name=name:output(name,msg),queue_size=100))
    def run(label, cmd):
        handle = (args.output/(label+'.log')).open('w')
        handles.append(handle)
        process = subprocess.Popen(cmd,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process)
        return process
    def wait(predicate, description, timeout):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            with lock:
                if predicate(): return
            if camera and camera.poll() is not None:
                raise RuntimeError('camera launch exited: '+description)
            if playback and playback.poll() is not None:
                raise RuntimeError('bag ended before check completed: '+description)
            time.sleep(.005)
        raise RuntimeError('timed out: '+description)
    def request(model,mode,purpose='transition'):
        proxy = rospy.ServiceProxy(NODES[model]+'/set_inference_mode', SetInferenceMode)
        started = time.monotonic()
        result = proxy(mode,HZ[model])
        returned = time.monotonic()
        if not result.accepted: raise RuntimeError(result.reason)
        record = dict(model=model, mode=mode, generation=result.generation, purpose=purpose,
                      call_started=started,call_returned=returned)
        requests.append(record)
        return record
    def prepared(record):
        model = record['model']
        wait(lambda: model in statuses and statuses[model].generation == record['generation']
             and statuses[model].mode == 1 and statuses[model].prepared
             and statuses[model].pipeline_ready, model+' current-generation prepare',args.prepare_timeout)
        record['ready_observed_monotonic'] = time.monotonic()
    def observe(name,duration,record=None):
        started = time.monotonic()
        deadline = started+duration
        while time.monotonic() < deadline:
            if camera.poll() is not None or playback.poll() is not None:
                raise RuntimeError('launch/playback ended during '+name)
            time.sleep(.005)
        with lock:
            state = {k:dict(mode=v.mode,generation=v.generation,prepared=v.prepared,
                           pipeline_ready=v.pipeline_ready,successful_batches=v.successful_batches)
                     for k,v in statuses.items()}
        stages.append(dict(name=name,start=started,end=deadline,request=record,statuses=state))
    try:
        launch_started=time.monotonic()
        camera=run('camera',['roslaunch','asmc_camera','perception.launch','profile:=planning',
            'traffic_light_wait_first:=0','lane_enable_shared_family_debug:=false'])
        for model in NODES:
            rospy.wait_for_service(NODES[model]+'/set_inference_mode',timeout=60)
            wait(lambda:model in statuses and statuses[model].loaded and statuses[model].warmed_up,
                 model+' load/dummy warmup',60)
        report['launch_to_all_loaded_and_dummy_warmed_sec']=time.monotonic()-launch_started
        playback=run('bag',['rosbag','play',args.bag,'--clock','-s',str(args.start_sec),'--quiet'])
        observe('all_standby',2)
        report['checks']['standby_no_compute']=all(s['successful_batches']==0 for s in stages[-1]['statuses'].values())
        for model in ['yolo','resnet18','twinlite']:
            for trial in range(args.prepare_trials):
                if trial: request(model,0); observe(model+'_standby_'+str(trial),.1)
                record=request(model,1,'prepare_sample')
                prepared(record)
            active=request(model,2)
            observe(model+'_active',10.,active)
            report['checks'][model+'_other_models_standby']=all(
                s['mode']==0 for k,s in stages[-1]['statuses'].items() if k!=model)
            if args.input_stop:
                os.killpg(playback.pid,signal.SIGSTOP)
                try:
                    observe(model+'_input_stopped',.8)
                    report['checks'][model+'_input_stop_clears_ready']=not (
                        statuses[model].prepared or statuses[model].pipeline_ready)
                    request(model,0)
                    failed_prepare=request(model,1,'input_stop')
                    observe(model+'_prepare_without_input',.6)
                    report['checks'][model+'_no_input_cannot_prepare']=(
                        statuses[model].generation==failed_prepare['generation']
                        and not statuses[model].prepared and not statuses[model].pipeline_ready)
                finally:
                    os.killpg(playback.pid,signal.SIGCONT)
                prepared(failed_prepare)
                reentry=request(model,2,'reentry')
                observe(model+'_reentry_active',10.,reentry)
            request(model,0)
            observe(model+'_stop_drain',.5)
        before={k:s['successful_batches'] for k,s in stages[-1]['statuses'].items()}
        observe('all_standby_again',1)
        report['checks']['stop_compute']=all(stages[-1]['statuses'][k]['successful_batches']==v for k,v in before.items())
    except Exception as exc:
        report['failures'].append(str(exc))
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        for handle in handles:handle.close()
        for subscriber in subscribers:subscriber.unregister()
        traces={}
        if not args.no_trace:
            for model in NODES:
                path=args.output/(model+'.lifecycle.jsonl')
                traces[model]=[json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
            report['checks']['complete_traces']=all(trace_integrity(trace) for trace in traces.values())
        def correlate(record, ended=None):
            if not args.no_trace:
                return trace_trial(record,traces[record['model']],rows if ended else None,ended)
            result=dict(record,accepted_monotonic=record['call_started'])
            if ended:
                matching=[s for s in status_rows if s['model']==record['model'] and s['generation']==record['generation']]
                if not matching:raise ValueError('missing acknowledged-generation status')
                result['outputs']={name:rate_report(values,record['call_started'],ended,matching[0]['source_cutoff_ns'])
                                   for name,values in rows.items()}
            return result
        try:
            report['transitions']=[correlate(record) for record in requests]
            report['stages']=[]
            for stage in stages:
                if stage['request']:
                    measured=correlate(stage['request'],stage['end'])
                    stage['measurement']=measured
                    for name in OUTPUTS[stage['request']['model']]:
                        metric=measured['outputs'].get(name,{})
                        passed=(metric.get('hz',0)>=18. if name=='yolo' else ten_hz_pass(metric))
                        report['checks'][name+'_active_rate']=report['checks'].get(name+'_active_rate',True) and passed
                        clean=not(metric.get('duplicate_stamps',1) or metric.get('reversed_stamps',1))
                        report['checks'][name+'_no_duplicate_reverse']=report['checks'].get(name+'_no_duplicate_reverse',True) and clean
                else:
                    stage['outputs']={name:rate_report(values,stage['start'],stage['end']) for name,values in rows.items()}
                report['stages'].append(stage)
            for model in ('yolo','resnet18'):
                ranges=[(r['call_returned'],next(n['call_started'] for n in requests[i+1:] if n['model']==model))
                        for i,r in enumerate(requests) if r['model']==model and r['mode']==1]
                report['checks'][model+'_prepare_no_operational_output']=not any(
                    a<=t<b for t,_,_ in rows.get(model,[]) for a,b in ranges)
            last=report['stages'][-1]
            report['checks']['stop_operational_outputs']=last['name']=='all_standby_again' and all(
                not r['messages'] for r in last['outputs'].values())
            report['readiness']={}
            if not args.no_trace:
                for model in NODES:
                    samples=[r for r in report['transitions'] if r['model']==model and r['purpose']=='prepare_sample']
                    stats={key:dict(samples=len(samples),p95_sec=percentile([r[key] for r in samples if r[key] is not None],95),
                                    p99_sec=percentile([r[key] for r in samples if r[key] is not None],99))
                           for key in ('actual_image_ready_sec','pipeline_ready_sec')}
                    report['readiness'][model]=stats
                    report['checks'][model+'_all_prepare_trials_ready']=len(samples)==args.prepare_trials and all(
                        r['actual_image_ready_sec'] is not None and r['pipeline_ready_sec'] is not None for r in samples)
                report['startup']={model:[r for r in trace if r['event']=='startup'] for model,trace in traces.items()}
        except Exception as exc:
            report['failures'].append('analysis: '+str(exc))
        report['passed']=not report['failures'] and bool(report['checks']) and all(report['checks'].values())
        (args.output/'observations.json').write_text(json.dumps(dict(outputs=rows,statuses=status_rows,requests=requests))+'\n')
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(passed=report['passed'],checks=report['checks'],failures=report['failures'])))
    return 0 if report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
