#!/usr/bin/env python3
"""Serial camera experiments; uses existing frame/resource measurement helpers.

Run in the selected overlay (frozen main for R0, candidate for R1/R2). No vehicle
control, actors, training or producer defaults are changed. A saved-image replay
can run on a separate ROS master; live trials use the existing bridge.
"""
from asmc_camera.runtime.parameters import canonical_control_path
import argparse
import hashlib
from collections import defaultdict
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import threading
import time

from lifecycle_measurements import percentile, rate_report
from b0_resources import ResourceSampler, qt_viewers


def acceptance_eligible(report):
    return (report.get('acceptance_eligible',True) and
            not report.get('allocation_profile',False) and not report.get('profile_lane_fit',False))


def active_window(start, planned_end, finished, shutdown_at=None):
    end=min(planned_end,shutdown_at) if shutdown_at is not None else planned_end
    return dict(start=start,end=max(start,end),planned_end=planned_end,
                complete=finished>=planned_end and (shutdown_at is None or shutdown_at>=planned_end))


def shutdown_observer(previous, record):
    def handle(signum, frame):
        record()  # roslaunch pmon may block before invoking rospy shutdown hooks.
        if callable(previous):return previous(signum,frame)
        if previous==signal.SIG_DFL:raise KeyboardInterrupt()
    return handle


def transition_ready(status, expected, mode):
    return all(role in status and status[role].generation==generation and
               status[role].mode==mode and status[role].prepared and
               (role!='lane' or status[role].pipeline_ready)
               for role,generation in expected.items())


def measurements(rows, start, end, target):
    reports = {}
    streams = defaultdict(list)
    for row in rows:
        if row['event'] == 'output' and not row.get('stale') and row['received'] is not None:
            if row['topic'] == '/perception/camera/objects' and not row['valid']:
                continue
            streams[row['topic'] + ':' + row['frame']].append(row)
    for key, values in streams.items():
        values.sort(key=lambda r: r['at'])
        report = rate_report([(r['at'], r['stamp'], r['valid']) for r in values], start, end)
        times = [r['wall_ms'] for r in values if start <= r['at'] < end]
        report['wall_ms'] = {f'p{q}': percentile(times, q) for q in (50, 95, 99)}
        hz = target if key.startswith('/perception/camera/objects:') else 10.
        report['target_hz'] = hz
        report['target_95pct_pass'] = bool(report['five_second_hz']) and min(report['five_second_hz']) >= hz*.95
        report['minimum_9_5_pass'] = bool(report['five_second_hz']) and min(report['five_second_hz']) >= 9.5
        report['minimum_10_pass'] = bool(report['five_second_hz']) and min(report['five_second_hz']) >= 10.
        report['timing_pass'] = bool(times) and report['wall_ms']['p95'] <= 150 and report['wall_ms']['p99'] <= 200 and report['max_gap_sec'] <= .25
        reports[key] = report
    return reports


def resource_summary(resources, start, end, interval=.5):
    resources=[r for r in resources if start<=r['arrival']<end]
    gpu=[r['gpu'] for r in resources if r['gpu']]
    cpu=[sum(n['cpu_percent'] or 0 for n in r['nodes'].values()) for r in resources
         if any(n['cpu_percent'] is not None for n in r['nodes'].values())]
    system_cpu=[r['system']['cpu_percent'] for r in resources if r.get('system',{}).get('cpu_percent') is not None]
    times=[start]+[r['arrival'] for r in resources]+[end]
    coverage=(len(resources)>=(end-start)/interval*.8 and len(cpu)>=len(resources)*.9 and
              len(gpu)>=len(resources)*.9 and max((b-a for a,b in zip(times,times[1:])),default=end-start)<=3.)
    return dict(camera_cpu_mean=sum(cpu)/len(cpu) if cpu else None,
        camera_rss_peak_mib=max((sum(n['rss_mib'] for n in r['nodes'].values()) for r in resources),default=None),
        vram_peak_mib=max((r['memory_used_mib'] for r in gpu),default=None),
        vram_peak_fraction=max((r['memory_used_mib']/r['memory_total_mib'] for r in gpu),default=None),
        wsl_system_cpu_mean=sum(system_cpu)/len(system_cpu) if system_cpu else None,
        sample_count=len(resources),cpu_sample_count=len(cpu),gpu_sample_count=len(gpu),resource_coverage_pass=coverage,
        scope='Summed camera/debug/preview ROS processes; rqt included in WSL system CPU, not this process sum. Whole GPU includes MORAI/Windows.')


def main():
    import rospy
    import roslaunch
    import rospkg
    from asmc_msgs.msg import InferenceStatus
    from asmc_msgs.srv import SetInferenceMode
    from sensor_msgs.msg import Image, CompressedImage
    from asmc_msgs.msg import CameraObjectBoxArray, LaneBoundaryFamily
    from diagnostic_msgs.msg import DiagnosticArray
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--variant', choices=('R0', 'R1', 'R2'), required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=60)
    p.add_argument('--active', default='objects,lane,traffic_light')
    p.add_argument('--lane-hz', type=float, default=10)
    p.add_argument('--saved-input', type=Path, help='JSON {front,left,right,extra: [JPEG path,...]}')
    p.add_argument('--gui', action='store_true', help='Visualization publishers with connected sinks')
    p.add_argument('--view-models', default='all', help='view.launch models selection; recorded separately from inference ACTIVE roles')
    p.add_argument('--visual-outputs', choices=('all','family','bev'), default='all',
        help='Internal demand comparison: full views, published family only, or fused BEV only')
    p.add_argument('--qt', action='store_true', help='Also open the actual rqt viewer for this trial')
    p.add_argument('--nms-iou', type=float, default=.7)
    p.add_argument('--confidence', type=float, default=.4)
    p.add_argument('--lane-batch-wait-ms', type=float)
    p.add_argument('--fusion-hz', type=float)
    p.add_argument('--legacy-lidar-root', type=Path)
    p.add_argument('--profile-lane-fit', action='store_true')
    p.add_argument('--allocation-profile', action='store_true',help='Diagnostic TwinLite allocation/deserialize run; excluded from acceptance')
    p.add_argument('--shared-gpu-input', action='store_true')
    p.add_argument('--lane-runtime-config', type=Path)
    a = p.parse_args()
    if a.allocation_profile and (a.variant=='R2' or a.profile_lane_fit):
        p.error('allocation diagnosis uses an independent lane worker without fit profiling')
    if a.qt and not a.gui:
        p.error('--qt requires --gui')
    if a.visual_outputs!='all' and not a.gui:
        p.error('explicit visual output demand requires --gui')
    # roslaunch starts node processes in ROS_HOME; probes need an absolute path.
    a.output = a.output.resolve()
    a.output.mkdir(parents=True, exist_ok=False)
    rospy.init_node('camera_runtime_comparison', anonymous=True)
    package = Path(rospkg.RosPack().get_path('asmc_camera'))
    root = Path(__file__).resolve().parents[2]
    legacy = dict(objects='/camera_object_detector', lane='/camera_semantic_inference', traffic_light='/traffic_light_infer')
    active = a.active.split(',')
    from camera_measurement_records import RecordedRows, instrument_consumer
    records=[RecordedRows(a.output/('consumer-'+name+'.jsonl')) for name in
             ('statuses','diagnostics','boxes','families','debug')]
    status_rows,diagnostics,outputs,families,debug_outputs=records
    status, subscribers = {}, []
    close_consumer_probe=instrument_consumer(a.output/'consumer-gc.jsonl',LaneBoundaryFamily)
    debug_samples, latest_debug, failure_samples = {}, {}, {}
    start = end = None
    def receive_debug(message, role):
        at = time.monotonic()
        debug_outputs.append(dict(role=role,at=at,stamp=message.header.stamp.to_nsec(),bytes=len(message.data)))
        latest_debug[role] = message
        if start is not None and start <= at < end and role not in debug_samples:
            debug_samples[role] = message
    def receive_status(role, message):
        status[role] = message
        row=dict(role=role,at=time.monotonic(),generation=message.generation,mode=message.mode,
            prepared=message.prepared,pipeline_ready=message.pipeline_ready,errors=message.errors,reason=message.reason)
        status_rows.append(row)
        if role=='lane' and not message.pipeline_ready and start is not None and start<=row['at']<end:
            for name,sample in list(latest_debug.items()):
                failure_samples.setdefault(name,(sample,row))
    for role, node in legacy.items():
        subscribers.append(rospy.Subscriber(canonical_control_path(node, 'status'), InferenceStatus,
            lambda m, r=role: receive_status(r, m), queue_size=20))
    subscribers.append(rospy.Subscriber('/perception/camera/objects', CameraObjectBoxArray,
        lambda m: outputs.append(dict(frame=m.header.frame_id, stamp=m.header.stamp.to_nsec(), seq=m.header.seq,
            valid=m.valid, boxes=[dict(label=b.source_class_name, group=b.class_name, confidence=b.confidence,
                                    xyxy=[b.xmin,b.ymin,b.xmax,b.ymax]) for b in m.boxes])), queue_size=30))
    subscribers.append(rospy.Subscriber('/perception/lane/boundary_family', LaneBoundaryFamily,
        lambda m: families.append(dict(at=time.monotonic(),stamp=m.header.stamp.to_nsec(),
            valid=m.valid,reason=m.status_reason)),queue_size=1))
    # Keep the same normal lane consumers as the real stack. No GUI consumers
    # are added unless explicitly requested, so lazy output stays comparable.
    for topic in ('/perception/camera/objects/diagnostics', '/perception/camera/diagnostics', '/morai_sensor_bridge/diagnostics',
                  '/perception/bev/diagnostics', '/perception/bev/debug/diagnostics', '/perception/lane/diagnostics'):
        subscribers.append(rospy.Subscriber(topic, DiagnosticArray, lambda m: diagnostics.append(
            dict(at=time.monotonic(), statuses=[dict(name=s.name, values={v.key:v.value for v in s.values}) for s in m.status])), queue_size=10))
    launch = package/'launch'/('perception.launch' if a.variant == 'R0' else '_runtime.launch')
    args = ['profile:=planning', 'objects_debug_raw:=false', 'lane_enable_shared_family_debug:=false',
            'lane_target_hz:='+str(a.lane_hz), 'traffic_light_device:=cuda'] if a.variant == 'R0' else [
            'mode:=standby', 'implementation:='+('shared' if a.variant == 'R2' else 'separate'), 'lane_hz:='+str(a.lane_hz), 'nms_iou:='+str(a.nms_iou)]
    if a.variant != 'R0':
        if a.lane_runtime_config:
            args += ['lane_runtime_config:='+str(a.lane_runtime_config.resolve())]
        if a.shared_gpu_input:
            if a.variant != 'R2':
                raise ValueError('shared GPU input requires R2')
            args += ['shared_gpu_input:=true']
        args += ['confidence:='+str(a.confidence)]
        if a.lane_batch_wait_ms is not None:
            args += ['lane_batch_wait_ms:='+str(a.lane_batch_wait_ms)]
        if a.fusion_hz is not None:
            args += ['lane_fusion_hz:='+str(a.fusion_hz)]
    probe = root/'tools/perception/camera_runtime_probe.py'
    class MeasuredLaunch(roslaunch.parent.ROSLaunchParent):
        def _load_config(self):
            super()._load_config()
            for node in self.config.nodes:
                if node.type in ('lane_vectorizer_node.py','semantic_inference_node.py'):
                    key='/'+node.name+'/lifecycle_trace_path'
                    self.config.params[key]=roslaunch.core.Param(key,str(a.output/(node.name+'-supply.trace')))
                if node.type in ('object_detection_node.py','semantic_inference_node.py','traffic_light_node.py','camera_runtime_node.py'):
                    diagnostic = ' --allocation-profile' if a.allocation_profile and node.type=='semantic_inference_node.py' else ''
                    node.launch_prefix = 'python3 '+shlex.quote(str(probe))+diagnostic+' --output '+shlex.quote(str(a.output/(node.name+'.jsonl')))+' --'
                elif node.type=='lane_vectorizer_node.py' and a.profile_lane_fit:
                    node.launch_prefix = 'python3 '+shlex.quote(str(probe))+' --lane-fit-profile --output '+shlex.quote(str(a.output/'lane-fit.profile'))+' --'
                elif node.type=='lane_vectorizer_node.py':
                    # Record family-process GC separately from model GC. The
                    # normal probe does not trace allocations or alter policy.
                    node.launch_prefix = 'python3 '+shlex.quote(str(probe))+' --output '+shlex.quote(str(a.output/'family-gc.jsonl'))+' --'
    class MeasuredViewLaunch(roslaunch.parent.ROSLaunchParent):
        def _load_config(self):
            super()._load_config()
            for node in self.config.nodes:
                if node.type in ('object_view_node.py','object_visualizer_node.py'):
                    node.launch_prefix = 'python3 '+shlex.quote(str(probe))+' --visual-codecs --output '+shlex.quote(str(a.output/('visual-'+node.name+'.jsonl')))+' --'
                elif node.type=='family_view_node.py':
                    # The original ROS uint8[] wire decoder rejected bytes.
                    # Use the same repaired renderer for both fitters; leave
                    # the frozen baseline inference/fitting sources untouched.
                    node.launch_prefix='python3 '+shlex.quote(str(probe))+' --visual-codecs --entrypoint '+shlex.quote(str(root/'src/asmc_camera/scripts/family_view_node.py'))+' --output '+shlex.quote(str(a.output/'family-view.jsonl'))+' --'
                    node.env_args.append(('PYTHONPATH',str(root/'devel/lib/python3/dist-packages')+':'+os.environ.get('PYTHONPATH','')))
    parent = MeasuredLaunch(roslaunch.rlutil.get_or_generate_uuid(None, False), [(str(launch), args)])
    roslaunch.configure_logging(parent.run_id)
    replay_stop, replay = threading.Event(), None
    replay_sources = {}
    if a.saved_input:
        blobs = {v:[Path(f).read_bytes() for f in files] for v,files in json.loads(a.saved_input.read_text()).items()}
        topics = dict(front='/image_jpeg/compressed', left='/cam_left/image_jpeg/compressed',
                      right='/cam_right/image_jpeg/compressed', extra='/cam_extra/image_jpeg/compressed')
        publishers = {v:rospy.Publisher(topics[v], CompressedImage, queue_size=1) for v in blobs}
        def replay_frames():
            deadline, sequence = time.monotonic(), 0
            while not replay_stop.is_set() and not rospy.is_shutdown():
                stamp = rospy.Time.now()
                for view, pub in publishers.items():
                    msg = CompressedImage(format='jpeg', data=blobs[view][(sequence//4) % len(blobs[view])])
                    msg.header.stamp, msg.header.seq, msg.header.frame_id = stamp, sequence, 'camera_'+view
                    replay_sources[str(stamp.to_nsec())+':camera_'+view] = (sequence//4) % len(blobs[view])
                    pub.publish(msg)
                sequence += 1
                deadline += .05
                replay_stop.wait(max(0, deadline-time.monotonic()))
        replay = threading.Thread(target=replay_frames, daemon=True)
        replay.start()
    main_process = qt_process = None
    sampler, report, start, end = None, dict(variant=a.variant, active=active, lane_hz=a.lane_hz,
        saved_input=str(a.saved_input) if a.saved_input else None, gui=a.qt, visualization_sinks=a.gui, nms_iou=a.nms_iou, failures=[],
        confidence=a.confidence, view_models=a.view_models if a.gui else None, profile_lane_fit=a.profile_lane_fit,
        visual_outputs=a.visual_outputs,allocation_profile=a.allocation_profile,
        acceptance_eligible=not(a.allocation_profile or a.profile_lane_fit),
        mode_transitions=[],
        timestamp_mode=rospy.get_param('/morai_sensor_bridge/timestamp_mode','unset (bridge default receive)'),
        use_sim_time=rospy.get_param('/use_sim_time',False),
        latency_definition='Actual model/broker compressed input callback entry -> ROS publish return; monotonic wall.',
        aggregate_definition='Per-run raw frames, linear interpolated percentiles; no mean of run medians.'), None, None
    report['consumer_recording']='bounded async JSONL; report materialization after ACTIVE'
    report['family_subscription_queue']=1
    shutdown_at=[None]; signal_handlers={}
    def record_shutdown():
        if shutdown_at[0] is None:shutdown_at[0]=time.monotonic()
    rospy.on_shutdown(record_shutdown)
    if a.lane_runtime_config:
        report['lane_runtime_config'] = str(a.lane_runtime_config)
        report['lane_runtime_config_sha256'] = hashlib.sha256(Path(a.lane_runtime_config).read_bytes()).hexdigest()
    try:
        report['qt_inventory_before']=qt_viewers()
        if any(v['owned_trial'] for v in report['qt_inventory_before']):
            raise RuntimeError('unclosed Qt from an earlier trial; do not start a new comparison')
        hashes = {str(f.relative_to(package)):hashlib.sha256(f.read_bytes()).hexdigest()
                  for pattern in ('src/**/*.py','launch/*.launch','config/**/*.yaml') for f in package.glob(pattern)}
        report['source_sha256'] = hashlib.sha256(json.dumps(hashes,sort_keys=True).encode()).hexdigest()
        report['measurement_sha256']={f:hashlib.sha256((root/'tools/perception'/f).read_bytes()).hexdigest()
            for f in ('compare_camera_runtime.py','camera_runtime_probe.py','b0_resources.py',
                      'run_camera_structure_matrix.py','run_b0_matrix.py','camera_measurement_records.py')}
        report['family_view_source_sha256']={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (root/'src/asmc_camera/src/asmc_camera/bev/family_view.py',
                      root/'src/asmc_camera/src/asmc_camera/nodes/family_view_node.py',
                      root/'src/asmc_camera/scripts/family_view_node.py')}
        (a.output/'source-hashes.json').write_text(json.dumps(hashes,indent=2))
        parent.start()
        report['effective_lane_batch_wait_ms'] = rospy.get_param('/camera_lane/batch_wait_ms',None)
        report['effective_fusion_hz'] = rospy.get_param('/camera_bev_fusion/max_fused_hz',None)
        deadline = time.monotonic()+120
        while not all(r in status and status[r].loaded and status[r].warmed_up for r in legacy):
            if time.monotonic() > deadline:
                raise RuntimeError('model load timeout')
            parent.spin_once(); time.sleep(.05)
        def transition(mode):
            record=dict(mode=mode,started=time.monotonic(),expected={},ready=False)
            report['mode_transitions'].append(record)
            for role in active:
                rospy.wait_for_service(canonical_control_path(legacy[role], 'set_mode'),5)
                response=rospy.ServiceProxy(canonical_control_path(legacy[role], 'set_mode'),SetInferenceMode)(mode,0.)
                if not response.accepted: raise RuntimeError(response.reason)
                record['expected'][role]=response.generation
            deadline=time.monotonic()+30
            while not transition_ready(status,record['expected'],mode):
                if time.monotonic()>deadline:
                    record['elapsed_sec']=time.monotonic()-record['started']
                    raise RuntimeError('mode {} prepare/family timeout: {}'.format(mode,
                        {r:status[r].reason for r in active}))
                parent.spin_once(); time.sleep(.05)
            record.update(ready=True,elapsed_sec=time.monotonic()-record['started'])
        transition(1)
        transition(2)
        if a.gui:
            # Start only visualization and connect compressed debug sinks. The
            # sink cost is recorded separately from a real rqt window's cost.
            view_launch = MeasuredViewLaunch(parent.run_id,
                [(str(package/'launch/view.launch'), ['models:='+a.view_models])])
            view_launch.start()
            debug_topics = dict(objects='/perception/camera/objects/debug/mosaic/compressed',
                lane='/perception/camera/debug/mosaic/compressed', traffic_light='/perception/traffic_light/debug/image/compressed')
            lane_views=dict(lane_family='/perception/lane/debug/shared_family_bev/compressed',
                            bev='/perception/bev/debug/fused/compressed')
            if a.visual_outputs=='all':
                debug_topics.update(lane_views)
            else:
                debug_topics={a.visual_outputs:lane_views['lane_family' if a.visual_outputs=='family' else 'bev']}
            for role, topic in debug_topics.items():
                selected_role='lane' if role in ('lane_family','family','bev') else role
                if a.view_models != 'all' and selected_role not in a.view_models.split(','):
                    continue
                subscribers.append(rospy.Subscriber(topic, CompressedImage,
                    lambda m,r=role:receive_debug(m,r),queue_size=1))
        if a.legacy_lidar_root:
            env = dict(os.environ)
            nested = a.legacy_lidar_root
            for key, value in [('ROS_PACKAGE_PATH',nested/'src'), ('CMAKE_PREFIX_PATH',nested/'devel'),
                               ('LD_LIBRARY_PATH',nested/'devel/lib'), ('PYTHONPATH',nested/'devel/lib/python3/dist-packages')]:
                env[key] = str(value)+':'+env.get(key,'')
            main_process = subprocess.Popen(['roslaunch',str(root/'tools/perception/b0_main.launch'),
                'camera_class_profile:=mission16_v1','camera_objects_conf_thres:=0.0'], env=env,
                stdout=(a.output/'legacy-lidar.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
            time.sleep(3)
            if main_process.poll() is not None:
                raise RuntimeError('legacy lidar launch exited')
        if a.qt:
            qt_topic = (lane_views['lane_family'] if a.visual_outputs=='family' else lane_views['bev']
                        if a.visual_outputs=='bev' else '/perception/camera/objects/debug/mosaic/compressed')
            qt_process = subprocess.Popen(['rosrun','rqt_image_view','rqt_image_view','--clear-config',
                qt_topic,'__name:=camera_runtime_viewer'],
                stdout=(a.output/'qt.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
            time.sleep(3)
            if qt_process.poll() is not None:
                report['failures'].append('Qt viewer exited; do not claim GUI-ON acceptance')
        time.sleep(2)
        report['qt_inventory_during']=qt_viewers()
        report['expected_owned_qt']=int(a.qt)
        if sum(v['owned_trial'] for v in report['qt_inventory_during'])!=int(a.qt):
            report['failures'].append('actual trial Qt count differs from requested GUI condition')
        names = ['/camera_bev','/camera_bev_fusion','/camera_lane_vectorizer'] + (
            ['/camera_runtime'] if a.variant == 'R2' else list(legacy.values()) if a.variant == 'R0'
            else ['/camera_objects','/camera_lane','/camera_traffic_light'])
        if a.gui:
            names += ['/'+n.name for n in view_launch.config.nodes]
        if a.legacy_lidar_root: names += ['/Main','/camera_objects_yolo_bbox_adapter']
        for sig in (signal.SIGINT,signal.SIGTERM):
            signal_handlers[sig]=signal.getsignal(sig)
            signal.signal(sig,shutdown_observer(signal_handlers[sig],record_shutdown))
        sampler = ResourceSampler(os.environ.get('ROS_MASTER_URI','http://127.0.0.1:11311'), names, interval=.5, system_metrics=True)
        sampler.start()
        start = time.monotonic(); end = start+a.seconds
        print('MEASURE '+str(a.output), flush=True)
        while time.monotonic() < end and not rospy.is_shutdown():
            parent.spin_once(); time.sleep(.05)
        window=active_window(start,end,time.monotonic(),shutdown_at[0]);end=window['end']
        report.update(window,final_status={r:dict(generation=m.generation, prepared=m.prepared, pipeline_ready=m.pipeline_ready, errors=m.errors, reason=m.reason) for r,m in status.items()})
        if not window['complete']:report['failures'].append('interrupted ACTIVE measurement; cleanup excluded')
        if qt_process and qt_process.poll() is not None:
            report['failures'].append('Qt viewer terminated during measurement')
    except Exception as exc:
        report['failures'].append(str(exc))
    finally:
        # GUI/preview have independent process groups. Reap them before ROS
        # launch shutdown, which can block while model workers join.
        from run_b0_matrix import stop
        if qt_process: stop(qt_process,timeout=5)
        if main_process: stop(main_process,timeout=10)
        report['qt_inventory_after']=qt_viewers()
        if any(v['owned_trial'] for v in report['qt_inventory_after']):
            report['failures'].append('trial Qt remains after cleanup')
        if sampler:
            resources = sampler.close()
            (a.output/'resources.json').write_text(json.dumps(resources))
            report['resources'] = resource_summary(resources,start,end)
            if not report['resources']['resource_coverage_pass']:
                report['failures'].append('insufficient resource sampling coverage; no resource acceptance')
        if a.gui and 'view_launch' in locals():
            view_launch.shutdown()
        parent.shutdown()
        for sig,handler in signal_handlers.items():signal.signal(sig,handler)
        replay_stop.set()
        if replay:
            replay.join(2)
        for sub in subscribers:
            sub.unregister()
        try:
            close_consumer_probe()
        except Exception as error:
            report['failures'].append('consumer timing journal failed: '+str(error))
        retained=[]
        for record in records:
            try:
                retained.append(record.finish())
            except Exception as error:
                report['failures'].append('consumer journal failed: '+record.path.name+': '+str(error))
                retained.append([])
        status_rows,diagnostics,outputs,families,debug_outputs=retained
        rows = []
        for path in a.output.glob('*.jsonl'):
            loaded = [dict(json.loads(s), probe=path.stem) for s in path.read_text().splitlines() if s]
            rows.extend(loaded)
            ends = [r for r in loaded if r.get('event') == 'journal_end']
            if not ends:
                report['failures'].append('measurement journal did not finish: '+path.name)
            if ends and any(r['dropped'] for r in ends):
                report['failures'].append('measurement journal dropped events: '+path.name)
        if start is not None and not rows:
            report['failures'].append('missing model measurement traces; no latency/supply claim')
        if start is not None:
            unmatched=[r for r in rows if r['event']=='output' and start<=r['at']<end and
                       r.get('received') is None and r.get('valid',True) and not r.get('stale') and
                       (r['topic'] in ('/perception/camera/objects','/perception/traffic_light/detection') or '/road_marking/' in r['topic'])]
            report['unmatched_model_output_stamps']=len(unmatched)
            if unmatched:
                report['failures'].append('model input/output stamp mismatch')
            report['streams'] = measurements(rows, start, end, 15)
            required = dict(objects='/perception/camera/objects:',
                lane='/perception/camera/front/road_marking/model_grid/class_id:',
                traffic_light='/perception/traffic_light/detection:')
            for role in active:
                if not any(key.startswith(required[role]) for key in report['streams']):
                    report['failures'].append('missing fresh measured output for '+role)
            report['input'] = {frame:rate_report([(r['at'],r['stamp'],True) for r in rows if r['event']=='input' and r['probe']+':'+r['frame']==frame],start,end)
                               for frame in {r['probe']+':'+r['frame'] for r in rows if r['event']=='input'}}
            for key, stream in report['streams'].items():
                if '/road_marking/' in key:
                    stream['target_hz'] = a.lane_hz
                    stream['target_95pct_pass'] = min(stream['five_second_hz'],default=0) >= a.lane_hz*.95
        if a.saved_input:
            (a.output/'replay-sources.json').write_text(json.dumps(replay_sources))
        (a.output/'boxes.json').write_text(json.dumps(outputs))
        (a.output/'statuses.json').write_text(json.dumps(status_rows))
        (a.output/'diagnostics.json').write_text(json.dumps(diagnostics))
        family_diagnostics=[s['values'] for row in diagnostics for s in row['statuses']
            if s['name']=='drivable_bev/lane_vectorizer']
        report['family_error_counters']={key:max((int(v.get(key,'0')) for v in family_diagnostics),default=0)
            for key in ('errors','shared_family_errors')}
        if any(report['family_error_counters'].values()):
            report['failures'].append('family calculation errors')
        (a.output/'lane-families.json').write_text(json.dumps(families))
        (a.output/'debug-outputs.json').write_text(json.dumps(debug_outputs))
        for role, message in debug_samples.items():
            (a.output/('sample-'+role+'.jpg')).write_bytes(bytes(message.data))
        failure_metadata={}
        for role,(message,context) in failure_samples.items():
            (a.output/('failure-'+role+'.jpg')).write_bytes(bytes(message.data))
            failure_metadata[role]=dict(context,debug_header_stamp_ns=message.header.stamp.to_nsec(),
                note='Most recent debug image when lane status became not ready; not a synchronized source GT image.')
        (a.output/'failure-samples.json').write_text(json.dumps(failure_metadata,indent=2))
        if start is not None:
            report['debug_outputs'] = {role:rate_report([(r['at'],r['stamp'],True) for r in debug_outputs if r['role']==role],start,end)
                for role in {r['role'] for r in debug_outputs}}
            report['family_view_stamped_messages']=sum(r['role']=='lane_family' and r['stamp']>0 and start<=r['at']<end for r in debug_outputs)
            if a.gui and a.visual_outputs in ('all','family') and not report['family_view_stamped_messages']:
                report['failures'].append('family visualization never received a renderable source stamp')
        if start is not None:
            report['lane_family'] = rate_report([(r['at'],r['stamp'],True) for r in families],start,end)
            report['lane_family']['valid_supply'] = rate_report(
                [(r['at'],r['stamp'],True) for r in families if r['valid']],start,end)
            active_families = [r for r in families if start <= r['at'] < end]
            report['lane_family']['fit_valid_count'] = sum(r['valid'] for r in active_families)
            inputs = {}
            for row in rows:
                if row['event']=='input' and 'camera_lane' in row['probe']:
                    inputs[row['stamp']] = min(row['at'],inputs.get(row['stamp'],row['at']))
            times = [(r['at']-inputs[r['stamp']])*1000 for r in active_families if r['stamp'] in inputs]
            report['lane_family']['input_to_subscriber_ms'] = {f'p{q}':percentile(times,q) for q in (50,95,99)}
            report['lane_family']['latency_sample_count'] = len(times)
            produced=[r for r in rows if r['event']=='output' and r['topic']=='/perception/lane/boundary_family' and r['valid']]
            report['lane_family']['producer_supply']=rate_report([(r['at'],r['stamp'],True) for r in produced],start,end)
            by_stamp=defaultdict(list)
            for row in produced:
                by_stamp[row['stamp']].append(row)
            transport=[]; unmatched=0
            for row in active_families:
                if not row['valid']: continue
                matches=[p for p in by_stamp[row['stamp']] if p.get('publish_begin',p['at'])<=row['at']]
                if not matches:
                    unmatched+=1
                    continue
                published=max(matches,key=lambda p:p['at'])
                transport.append(1000*(row['at']-published.get('publish_begin',published['at'])))
            report['lane_family']['publish_begin_to_subscriber_ms']={f'p{q}':percentile(transport,q) for q in (50,95,99)}
            report['lane_family']['unmatched_publisher_stamps']=unmatched
            if unmatched: report['failures'].append('family consumer/publisher stamp mismatch')
            supply_path=a.output/'camera_lane_vectorizer-supply.trace'
            if supply_path.exists():
                trace=[json.loads(line) for line in supply_path.read_text().splitlines()]
                endings=[r for r in trace if r['event']=='trace_end']
                if not endings or any(r['dropped'] for r in endings):
                    report['failures'].append('family supply trace incomplete/dropped')
                compute=[r for r in trace if r['event']=='family_compute_end' and start<=r['monotonic']<end]
                gates=[r for r in trace if r['event']=='family_publish_start' and start<=r['monotonic']<end]
                report['lane_family']['compute_wall_ms']={f'p{q}':percentile([r['wall_ms'] for r in compute],q) for q in (50,95,99)}
                report['lane_family']['compute_cpu_ms']={f'p{q}':percentile([r['cpu_ms'] for r in compute],q) for q in (50,95,99)}
                report['lane_family']['publish_lock_wait_ms']={f'p{q}':percentile([r['lock_wait_ms'] for r in gates],q) for q in (50,95,99)}
        (a.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print('RESULT '+json.dumps(report.get('resources', report['failures'])), flush=True)
    return int(bool(report['failures']))


if __name__ == '__main__':
    raise SystemExit(main())
