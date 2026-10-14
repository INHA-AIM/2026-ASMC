#!/usr/bin/env python3
"""Live MORAI matrix: fresh launches, independent generations and all failed trials retained."""
import argparse
from collections import defaultdict
import hashlib
import json
import os
from pathlib import Path
import subprocess
import threading
import time
import yaml
from b0_measurements import output_report, resource_report, quantiles
from b0_resources import ResourceSampler
from run_b0_matrix import stop

ROOT = Path(__file__).resolve().parents[2]
NODES = dict(yolo='/camera_object_detector', twinlite='/camera_semantic_inference', resnet18='/traffic_light_infer')
PROFILES = dict(C1=['yolo'], C4=['yolo','twinlite'], C5=['yolo','resnet18'], C6=['yolo','twinlite','resnet18'])
RATES = dict(all20=dict(front=20.,left=20.,right=20.), all15=dict(front=15.,left=15.,right=15.),
             all10=dict(front=10.,left=10.,right=10.), front20_side10=dict(front=20.,left=10.,right=10.))


def debug_masks_enabled(job):
    return bool(job.get('debug',False) and job.get('debug_masks',True))


def debug_topics(job):
    if not job.get('debug'): return []
    topics=['/perception/camera/objects/debug/image/compressed',
        '/perception/camera/objects/debug/mosaic/compressed',
        '/perception/camera/objects/debug/left/image/compressed',
        '/perception/camera/objects/debug/right/image/compressed',
        '/perception/camera/debug/mosaic/compressed',
        '/perception/bev/debug/comparison/compressed',
        '/perception/traffic_light/debug/image/compressed']
    if 'seg' in job['model'] and debug_masks_enabled(job):
        topics += ['/perception/camera/objects/debug/instances/compressed',
            '/perception/camera/objects/debug/left/instances/compressed',
            '/perception/camera/objects/debug/right/instances/compressed']
    return topics


def measurement_gate(row, minimum=9.5):
    return bool(row.get('complete') and row.get('five_second_hz') and
        all(hz >= minimum for hz in row['five_second_hz']) and
        row['valid_messages']==row['messages'] and not row['duplicate_stamps'] and not row['reversed_stamps'] and
        row['max_gap_sec'] <= .25 and row['input_anchor_missing']==0 and
        row['input_to_output_wall_ms']['p95'] is not None and row['input_to_output_wall_ms']['p95'] <= 150 and
        row['input_to_output_wall_ms']['p99'] <= 200)


def deadline_report(rows, begin, end, maximum_gap, minimum_hz):
    kept = [r for r in rows if begin <= r['arrival'] < end]
    times = [begin]+[r['arrival'] for r in kept]+[end]
    gap = max(b-a for a,b in zip(times,times[1:]))
    return dict(messages=len(kept), hz=len(kept)/(end-begin), max_gap_sec=gap,
                valid_messages=sum(r.get('valid', True) for r in kept),
                passed=bool(kept) and gap<=maximum_gap and len(kept)/(end-begin)>=minimum_hz)


def sensor_only_trial(output, seconds):
    """Read the existing bridge without starting models, actors or control."""
    import rospy
    from sensor_msgs.msg import CompressedImage
    from diagnostic_msgs.msg import DiagnosticArray
    from analyze_yoloe_selection import input_pauses
    inputs=defaultdict(list);diagnostics=[];subs=[]
    topics=dict(front='/image_jpeg/compressed',left='/cam_left/image_jpeg/compressed',
                right='/cam_right/image_jpeg/compressed',extra='/cam_extra/image_jpeg/compressed')
    for view,topic in topics.items():
        subs.append(rospy.Subscriber(topic,CompressedImage,lambda m,v=view:inputs[v].append(
            dict(arrival=time.monotonic(),stamp_ns=m.header.stamp.to_nsec())),queue_size=100,buff_size=2**24))
    for diagnostic_topic in ('/diagnostics','/morai_sensor_bridge/diagnostics','/morai_udp_bridge/diagnostics'):
        subs.append(rospy.Subscriber(diagnostic_topic,DiagnosticArray,lambda m:diagnostics.append(dict(
            arrival=time.monotonic(),values=[dict(name=s.name,**{v.key:v.value for v in s.values})
                for s in m.status if s.name.startswith(('morai_sensor_bridge/','morai_udp_bridge/'))])),queue_size=20))
    sampler=ResourceSampler(os.environ['ROS_MASTER_URI'],['/morai_sensor_bridge','/morai_udp_bridge'],system_metrics=True)
    sampler.start();begin=time.monotonic();end=begin+seconds
    try:
        while time.monotonic()<end and not rospy.is_shutdown():time.sleep(.1)
    finally:
        completed=time.monotonic()>=end
        samples=sampler.close()
        for s in subs:s.unregister()
        report=dict(mode='sensor_bridge_only',start=begin,end=end,seconds=seconds,complete=completed,
            timestamp_policy=dict(sensor_bridge=rospy.get_param('/morai_sensor_bridge/timestamp_mode','unknown')),
            views={v:input_pauses(rows,begin,end) for v,rows in inputs.items()},resources=resource_report(samples),
            interpretation='Diagnostic control only; does not waive any failed full-stack trial.')
        for filename,data in [('report',report),('inputs',dict(inputs)),('diagnostics',diagnostics),('resources',samples)]:
            (output/(filename+'.json')).write_text(json.dumps(data,indent=2)+'\n')


def provenance(models, engine, revision):
    # Include dirty source digests: HEAD alone does not identify an experiment.
    files = sorted(set(ROOT.glob('tools/perception/*.py')) | set(ROOT.glob('src/asmc_camera/src/asmc_camera/**/*.py')) |
                   set(ROOT.glob('src/asmc_camera/config/camera/**/*.yaml')) |
                   set(ROOT.glob('src/lidar+sensorfusion/src/Main/src/**/*.*')) |
                   set(ROOT.glob('src/lidar+sensorfusion/src/Main/scripts/*.py')) |
                   set(ROOT.glob('src/lidar+sensorfusion/src/Main/**/*.launch')) |
                   set(ROOT.glob('src/asmc_camera/launch/*.launch')))
    hashes = {str(f.relative_to(ROOT)): hashlib.sha256(f.read_bytes()).hexdigest() for f in files if f.is_file()}
    if len(revision)!=40 or any(c not in '0123456789abcdef' for c in revision):
        raise ValueError('full host checkout revision required')
    binaries={}
    for relative in ('Main/main','asmc_planner/asmc_planner_node','asmc_controller/mpc_controller_node'):
        for prefix in os.environ.get('CMAKE_PREFIX_PATH','').split(':'):
            path=Path(prefix)/'lib'/relative
            if path.is_file():
                binaries[relative]=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                break
    return dict(head=revision,
                source_sha256=hashes, engine_sha256=hashlib.sha256(engine.read_bytes()).hexdigest(),
                built_binaries=binaries,
                model_provenance={str(f):json.loads(f.read_text()) for f in
                    sorted(set(models.glob(engine.stem+'*.json')) | set(engine.parent.glob(engine.stem+'*.json')))},
                renderer='MORAI left running with user sensor/render settings; preset not independently verified',
                windows_background_cpu_ram='unavailable; global NVIDIA GPU includes simulator and desktop')


def main():
    import rosgraph
    import rospy
    from asmc_camera.msg import InferenceStatus
    from asmc_camera.srv import SetInferenceMode
    from asmc_msgs.msg import CameraObjectBoxArray, TrafficLightDetection, LaneBoundaryFamily, Trajectory, VehicleState, BehaviorCommand
    from planning_inputs.msg import DetectedObjectArray
    from sensor_msgs.msg import Image, CompressedImage
    from morai_msgs.msg import CtrlCmd
    from Msg.msg import Yolo_bbox_arr
    from diagnostic_msgs.msg import DiagnosticArray
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--jobs',type=Path,required=True)
    p.add_argument('--models',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--revision',required=True,help='Full host checkout SHA; container does not mount worktree Git metadata')
    p.add_argument('--sensor-only-seconds',type=float,help='Read existing sensors without launching models or preview')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    jobs=json.loads(a.jobs.read_text());(a.output/'protocol.json').write_text(json.dumps(jobs,indent=2)+'\n')
    rospy.init_node('coco7_live_measurements',anonymous=True)
    if a.sensor_only_seconds is not None:
        if a.sensor_only_seconds<5:p.error('sensor-only measurement requires at least five seconds')
        sensor_only_trial(a.output,a.sensor_only_seconds)
        return
    master=rosgraph.Master('/coco7_trial')
    index=[]
    for j in jobs:
        name='{model}-{width}-{rates}-{batch}-{profile}-r{repeat}'.format(**j)+('-debug' if j.get('debug') else '')
        out=a.output/name;out.mkdir()
        report=dict(job=j, failures=[],requests=[], preparation={}, observation_window_completed=False,
                    interpretation='MORAI rendering active; wall input arrival anchors; no driving')
        arrivals={};inputs=defaultdict(list);outputs=defaultdict(list);statuses={};status_rows=[];diagnostics=[];controls=[];bbox=[];planning=defaultdict(list);debug_rows=defaultdict(list)
        lock=threading.RLock();subs=[];processes=[];handles=[];sampler=None;begin=end=None;trace_parameters={}
        def save(): (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        def receive_input(view,msg):
            now=time.monotonic(); stamp=msg.header.stamp.to_nsec()
            with lock:
                arrivals[view,stamp]=now;inputs[view].append(dict(arrival=now,stamp_ns=stamp))
                if len(arrivals)>12000:
                    for key in list(arrivals)[:3000]:arrivals.pop(key)
        def receive_output(label,msg):
            now=time.monotonic();stamp=msg.header.stamp.to_nsec()
            if label=='yolo': label='yolo_'+msg.header.frame_id.removeprefix('camera_') if hasattr(str,'removeprefix') else 'yolo_'+msg.header.frame_id.replace('camera_','',1)
            view=label[5:] if label.startswith('yolo_') else 'extra' if label=='resnet18' else label
            with lock:
                anchor=arrivals.get((view,stamp))
                row=dict(arrival=now,stamp_ns=stamp,valid=bool(getattr(msg,'valid',True)),
                    source_age_ms=(rospy.Time.now().to_nsec()-stamp)/1e6,
                    wall_latency_ms=(now-anchor)*1000 if anchor is not None else None,
                    input_anchor='matched' if anchor is not None else 'missing')
                if label.startswith('yolo_'):
                    row['boxes']=[dict(label=b.class_name,source_label=b.source_class_name,confidence=b.confidence,
                                      xyxy=[b.xmin,b.ymin,b.xmax,b.ymax]) for b in msg.boxes]
                outputs[label].append(row)
        def receive_status(model,msg):
            with lock:
                statuses[model]=msg;status_rows.append(dict(model=model,arrival=time.monotonic(),mode=msg.mode,
                    generation=msg.generation,prepared=msg.prepared,pipeline_ready=msg.pipeline_ready,errors=msg.errors,reason=msg.reason,
                    source_cutoff_ns=msg.source_cutoff.to_nsec()))
        def launch(command,label):
            h=(out/(label+'.log')).open('w');handles.append(h)
            child=subprocess.Popen(command,stdout=h,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(child);report.setdefault('commands',[]).append(command);return child
        def wait_for(predicate,timeout,reason):
            deadline=time.monotonic()+timeout
            while not predicate():
                if any(c.poll() is not None for c in processes):raise RuntimeError('launch exited: '+reason)
                if rospy.is_shutdown() or time.monotonic()>deadline:raise RuntimeError(reason)
                time.sleep(.03)
        def request(model,mode):
            proxy=rospy.ServiceProxy(NODES[model]+'/set_inference_mode',SetInferenceMode)
            started=time.monotonic();reply=proxy(mode,0.)
            row=dict(model=model,mode=mode,generation=reply.generation,accepted=reply.accepted,reason=reply.reason,
                     call_started=started,call_returned=time.monotonic());report['requests'].append(row)
            if not reply.accepted:raise RuntimeError('mode request rejected '+model)
            return row
        try:
            sampler=ResourceSampler(os.environ['ROS_MASTER_URI'],list(NODES.values())+['/camera_bev','/camera_bev_fusion','/camera_lane_vectorizer','/Main','/asmc_planner','/mpc_controller','/morai_sensor_bridge','/morai_udp_bridge'],system_metrics=True)
            sampler.start()
            graph=master.getSystemState()
            if any(topic in ('/ctrl_cmd','/ctrl_cmd_0') and names for topic,names in graph[0]):
                raise RuntimeError('public control publisher present before preview trial')
            running={n for section in graph for _,names in section for n in names}
            if running & set(NODES.values()):raise RuntimeError('existing model launch; refusing duplicate inference')
            if rospy.get_param('/use_sim_time',False):raise RuntimeError('live trial requires wall ROS time')
            report['trace_enabled']=bool(j.get('diagnostic_trace',False))
            if report['trace_enabled']:
                for owner,node in NODES.items():
                    param=node+'/lifecycle_trace_path'
                    trace_parameters[param]=rospy.get_param(param,None)
                    rospy.set_param(param,str(out/(owner+'.lifecycle.jsonl')))
            report['timestamp_policy']=dict(
                sensor_bridge=rospy.get_param('/morai_sensor_bridge/timestamp_mode','unknown'),
                udp_bridge=rospy.get_param('/morai_udp_bridge/timestamp_mode','unknown'),
                interpretation='Receive stamps are bridge timestamps, not simulator acquisition timestamps. Input/output wall anchors use monotonic time.')
            model=j['model']
            custom=yaml.safe_load(Path(j['runtime_config']).read_text()) if j.get('runtime_config') else None
            engine=Path(custom['weights']) if custom else a.models/(model+'.engine')
            if not engine.is_file():raise RuntimeError('missing engine '+str(engine))
            report['provenance']=provenance(a.models,engine,a.revision)
            config=dict(weights=str(engine),expected_sha256=hashlib.sha256(engine.read_bytes()).hexdigest(),
                runtime_backend='tensorrt',task='segment' if 'seg' in model else 'detect',front_width=j['width'],
                batch_mode=j['batch'],batch_wait_ms=10.,batched_runtime=True,label_mode='source',
                allowed_names=['person','dog','bicycle','car','motorcycle','bus','truck'],
                conf_thres=j.get('confidence',.7),device='cuda:0',require_cuda=True,views=['front','left','right'],
                target_hz_by_view=RATES[j['rates']])
            if custom:
                if custom.get('label_mode')!='source' or custom.get('front_width')!=j['width'] or custom.get('model_id')!=j['model']:
                    raise ValueError('custom experiment must preserve raw labels and selected width')
                config=dict(custom,conf_thres=j.get('confidence',.4),target_hz_by_view=RATES[j['rates']],batch_mode=j['batch'])
            (out/'runtime.yaml').write_text(yaml.safe_dump(config));report['runtime']=config
            topics=dict(front='/image_jpeg/compressed',left='/cam_left/image_jpeg/compressed',right='/cam_right/image_jpeg/compressed',extra='/cam_extra/image_jpeg/compressed')
            for view,topic in topics.items():subs.append(rospy.Subscriber(topic,CompressedImage,lambda m,v=view:receive_input(v,m),queue_size=100,buff_size=2**24))
            subs.append(rospy.Subscriber('/perception/camera/objects',CameraObjectBoxArray,lambda m:receive_output('yolo',m),queue_size=100))
            for view in ('front','left','right'):
                subs.append(rospy.Subscriber('/perception/camera/'+view+'/road_marking/class_id',Image,lambda m,v=view:receive_output(v,m),queue_size=100,buff_size=2**24))
            subs.append(rospy.Subscriber('/perception/traffic_light/detection',TrafficLightDetection,lambda m:receive_output('resnet18',m),queue_size=100))
            subs.append(rospy.Subscriber('/perception/lane/boundary_family',LaneBoundaryFamily,lambda m:receive_output('family',m),queue_size=100))
            for model,node in NODES.items():subs.append(rospy.Subscriber(node+'/inference_status',InferenceStatus,lambda m,n=model:receive_status(n,m),queue_size=100))
            subs.append(rospy.Subscriber('/perception/camera/objects/diagnostics',DiagnosticArray,lambda m:diagnostics.append(dict(arrival=time.monotonic(),values=[{v.key:v.value for v in s.values} for s in m.status])),queue_size=100))
            subs.append(rospy.Subscriber('/diagnostics',DiagnosticArray,lambda m:diagnostics.append(dict(
                arrival=time.monotonic(),kind='bridge',values=[dict(name=s.name,**{v.key:v.value for v in s.values})
                    for s in m.status if s.name.startswith(('morai_sensor_bridge/','morai_udp_bridge/'))])),queue_size=20))
            if j.get('debug'):
                # Consume debug outputs without GUI/recording. Subscriber-aware
                # publishers must really encode images for the ON cost trial.
                topics=debug_topics(j)
                for topic in topics:
                    subs.append(rospy.Subscriber(topic,CompressedImage,lambda m,t=topic:debug_rows[t].append(
                        dict(arrival=time.monotonic(),bytes=len(m.data))),queue_size=1,buff_size=2**24))
            subs.append(rospy.Subscriber('/YOLO/BBox',Yolo_bbox_arr,lambda m:bbox.append(dict(arrival=time.monotonic(),source_stamp_ns=m.header.stamp.to_nsec(),count=m.objc)),queue_size=100))
            subs.append(rospy.Subscriber('/planning/trajectory',Trajectory,lambda m:planning['trajectory'].append(dict(arrival=time.monotonic(),valid=m.valid,failure_reason=m.failure_reason,points=len(m.points))),queue_size=100))
            subs.append(rospy.Subscriber('/planning/vehicle/state',VehicleState,lambda m:planning['state'].append(dict(arrival=time.monotonic(),valid=m.valid)),queue_size=100))
            subs.append(rospy.Subscriber('/behavior/command_v2',BehaviorCommand,lambda m:planning['behavior'].append(dict(arrival=time.monotonic())),queue_size=100))
            subs.append(rospy.Subscriber('/planning/objects',DetectedObjectArray,lambda m:planning['objects'].append(dict(arrival=time.monotonic())),queue_size=100))
            for topic in ('/evaluation/ctrl_cmd','/evaluation/ctrl_cmd_0','/ctrl_cmd','/ctrl_cmd_0'):
                subs.append(rospy.Subscriber(topic,CtrlCmd,lambda m,t=topic:controls.append(dict(topic=t,arrival=time.monotonic(),velocity=m.velocity,accel=m.accel,brake=m.brake)),queue_size=100))
            launch(['roslaunch','asmc_camera','perception.launch','profile:=planning',
                'camera_objects_config:='+str(out/'runtime.yaml'),'objects_views:=[front, left, right]',
                'start_visualization:='+str(j.get('debug',False)).lower(),
                'objects_debug_masks:='+str(debug_masks_enabled(j)).lower(),
                'lane_target_hz:=10','traffic_light_hz:=10','objects_target_hz:=20','traffic_light_wait_first:=0'], 'camera')
            wait_for(lambda:all(n in statuses and statuses[n].loaded and statuses[n].warmed_up for n in NODES),90,'model load timeout')
            time.sleep(2.)
            selected=PROFILES[j['profile']]
            for model in selected:
                req=request(model,1)
                wait_for(lambda:statuses[model].generation==req['generation'] and statuses[model].prepared,10,'actual-image prepare timeout '+model)
                report['preparation'][model]=dict(seconds=time.monotonic()-req['call_started'],pipeline_ready=statuses[model].pipeline_ready)
            if j.get('full_stack'):
                launch(['roslaunch',str(ROOT/'tools/perception/coco7_preview_stack.launch'),
                    'fusion_yolo_policy:='+j.get('fusion_policy','unique'),
                    'camera_objects_conf_thres:='+str(j.get('confidence',.7))], 'preview')
                # Planner startup is separate from model ACTIVE transition. Do
                # not let its first path/map loading gap masquerade as a YOLO
                # scheduling regression, or accept a STOP heartbeat as ready.
                preview_started=time.monotonic()
                def preview_ready():
                    now=time.monotonic()
                    return all(len([r for r in planning[label] if r['arrival']>preview_started and r.get('valid',True)])>=3 and
                               now-planning[label][-1]['arrival']<age
                               for label,age in [('trajectory',.25),('state',.20),('behavior',.5),('objects',.3)])
                wait_for(preview_ready,30,'preview pipeline did not become ready with real inputs')
                report['preview_prepare_seconds']=time.monotonic()-preview_started
            active={model:request(model,2) for model in selected}
            begin=min(r['call_started'] for r in active.values())
            end=max(r['call_started'] for r in active.values())+float(j.get('seconds',20))
            while time.monotonic()<end:
                if rospy.is_shutdown():raise RuntimeError('ACTIVE observation window interrupted')
                graph=master.getSystemState()
                if any(topic in ('/ctrl_cmd','/ctrl_cmd_0') and names for topic,names in graph[0]):raise RuntimeError('unexpected public control publisher')
                if any(c.poll() is not None for c in processes):raise RuntimeError('launch exited in measurement')
                time.sleep(.2)
            report['observation_window_completed']=True
            report['observed_window_end']=time.monotonic()
            report['measurements']={}
            for label,rows in outputs.items():
                owner='yolo' if label.startswith('yolo_') else 'resnet18' if label=='resnet18' else 'twinlite'
                if owner not in selected:continue
                req=active[owner];status=statuses[owner]
                report['measurements'][label]=output_report(rows,req['call_started'],req['call_started']+float(j.get('seconds',20)),status.source_cutoff.to_nsec(),float(j.get('seconds',20)))
            for view in ('front','left','right'):
                row=report['measurements'].get('yolo_'+view,{})
                if not measurement_gate(row):report['failures'].append('yolo '+view+' supply/latency gate')
            for owner, labels in [('twinlite',['front','left','right']),('resnet18',['resnet18'])]:
                if owner in selected:
                    for label in labels:
                        row=report['measurements'].get(label,{})
                        if (not row.get('five_second_hz') or any(hz<9.5 for hz in row['five_second_hz']) or
                                row.get('first_second',0)<9 or row.get('max_gap_sec',1)>.25 or row.get('duplicate_stamps') or row.get('reversed_stamps')):
                            report['failures'].append(owner+' '+label+' supply gate')
            report['target_attainment']={v:dict(target_hz=RATES[j['rates']][v],
                observed_hz=report['measurements'].get('yolo_'+v,{}).get('hz'),
                met=bool(report['measurements'].get('yolo_'+v,{}).get('five_second_hz')) and all(hz>=RATES[j['rates']][v]*.95 for hz in report['measurements'].get('yolo_'+v,{}).get('five_second_hz',[])))
                for v in ('front','left','right')}
            if j.get('full_stack'):
                report['planning']={label:deadline_report(planning[label],begin,end,gap,rate)
                    for label,gap,rate in [('trajectory',.25,19.),('state',.20,9.),('behavior',.5,9.),('objects',.3,0.)]}
                report['planning']['control']=deadline_report([r for r in controls if r['topic']=='/evaluation/ctrl_cmd'],begin,end,.25,19.)
                report['planning']['trajectory_failure_reasons']={str(reason):sum(begin<=r['arrival']<end and r['failure_reason']==reason for r in planning['trajectory'])
                    for reason in sorted({r['failure_reason'] for r in planning['trajectory']})}
                for label in ('trajectory','state','behavior','objects','control'):
                    if not report['planning'][label]['passed']:report['failures'].append('planning '+label+' deadline gate')
                if not report['planning']['trajectory']['valid_messages']:
                    report['failures'].append('planner did not produce any valid trajectory; STOP heartbeat alone is not whole-stack validation')
            report['input_hz']={v:sum(begin<=r['arrival']<end for r in rows)/(end-begin) for v,rows in inputs.items()}
            if j.get('debug'):
                report['debug_consumption']={topic:dict(messages=sum(begin<=r['arrival']<end for r in rows),
                    bytes=sum(r['bytes'] for r in rows if begin<=r['arrival']<end)) for topic,rows in debug_rows.items()}
                report['debug_scope']='Compressed outputs consumed without GUI or extra recorder; each output must actually encode.'
                for topic in topics:
                    if not report['debug_consumption'].get(topic,{}).get('messages'):
                        report['failures'].append('debug output absent '+topic)
            report['graph']=master.getSystemState()
            report['passed']=not report['failures']
        except Exception as exc:
            report['failures'].append(str(exc));report['passed']=False
        finally:
            if sampler:
                resources=sampler.close()
                steady=[r for r in resources if begin is not None and begin<=r['arrival']<end]
                report['resources']=resource_report(steady)
                report['resources']['system']={name:quantiles([r['system'].get(name) for r in steady if r.get('system')])
                    for name in ('cpu_percent','memory_used_mib')}
                report['resources_startup_and_active']=resource_report(resources)
                report['resource_scope']='Startup/load/prepare and ACTIVE recorded; resources contains ACTIVE only; 90% gate covers all phases.'
                (out/'resources.json').write_text(json.dumps(resources)+'\n')
                samples=[r['gpu'] for r in resources if r.get('gpu')]
                if not samples or max(r['memory_used_mib']/r['memory_total_mib'] for r in samples)>.9:
                    report['failures'].append('GPU memory unavailable or above 90%');report['passed']=False
            for child in reversed(processes):stop(child)
            for param,previous in trace_parameters.items():
                if previous is None:
                    if rospy.has_param(param):rospy.delete_param(param)
                else:rospy.set_param(param,previous)
            for handle in handles:handle.close()
            for sub in subs:sub.unregister()
            for filename,rows in [('inputs',dict(inputs)),('outputs',dict(outputs)),('status',status_rows),('diagnostics',diagnostics),('controls',controls),('bbox',bbox),('planning',dict(planning)),('debug',dict(debug_rows))]:
                (out/(filename+'.json')).write_text(json.dumps(rows)+'\n')
            save();index.append(dict(name=name,job=j,passed=report['passed'],failures=report['failures']))
            (a.output/'index.json').write_text(json.dumps(index,indent=2)+'\n')
            print(json.dumps(index[-1]),flush=True)
        if rospy.is_shutdown():
            (a.output/'interrupted.json').write_text(json.dumps(dict(completed=len(index),requested=len(jobs),accepted=False))+'\n')
            return 1
    return 0 if all(row['passed'] for row in index) else 1


if __name__=='__main__':raise SystemExit(main())
