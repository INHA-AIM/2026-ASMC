#!/usr/bin/env python3
"""Serial camera experiments; uses existing frame/resource measurement helpers.

Run in the selected overlay (frozen main for R0, candidate for R1/R2). No vehicle
control, actors, training or producer defaults are changed. A saved-image replay
can run on a separate ROS master; live trials use the existing bridge.
"""
import argparse
import hashlib
from collections import defaultdict
import json
import os
from pathlib import Path
import shlex
import subprocess
import threading
import time

from lifecycle_measurements import percentile, rate_report
from b0_resources import ResourceSampler, qt_viewers


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
    from asmc_camera.msg import InferenceStatus
    from asmc_camera.srv import SetInferenceMode
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
    p.add_argument('--qt', action='store_true', help='Also open the actual rqt viewer for this trial')
    p.add_argument('--nms-iou', type=float, default=.7)
    p.add_argument('--confidence', type=float, default=.4)
    p.add_argument('--lane-batch-wait-ms', type=float)
    p.add_argument('--fusion-hz', type=float)
    p.add_argument('--legacy-lidar-root', type=Path)
    p.add_argument('--profile-lane-fit', action='store_true')
    a = p.parse_args()
    # roslaunch starts node processes in ROS_HOME; probes need an absolute path.
    a.output = a.output.resolve()
    a.output.mkdir(parents=True, exist_ok=False)
    rospy.init_node('camera_runtime_comparison', anonymous=True)
    package = Path(rospkg.RosPack().get_path('asmc_camera'))
    root = Path(__file__).resolve().parents[2]
    legacy = dict(objects='/camera_object_detector', lane='/camera_semantic_inference', traffic_light='/traffic_light_infer')
    active = a.active.split(',')
    status, status_rows, diagnostics, subscribers, outputs, families, debug_outputs = {}, [], [], [], [], [], []
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
        subscribers.append(rospy.Subscriber(node+'/inference_status', InferenceStatus,
            lambda m, r=role: receive_status(r, m), queue_size=20))
    subscribers.append(rospy.Subscriber('/perception/camera/objects', CameraObjectBoxArray,
        lambda m: outputs.append(dict(frame=m.header.frame_id, stamp=m.header.stamp.to_nsec(), seq=m.header.seq,
            valid=m.valid, boxes=[dict(label=b.source_class_name, group=b.class_name, confidence=b.confidence,
                                    xyxy=[b.xmin,b.ymin,b.xmax,b.ymax]) for b in m.boxes])), queue_size=30))
    subscribers.append(rospy.Subscriber('/perception/lane/boundary_family', LaneBoundaryFamily,
        lambda m: families.append(dict(at=time.monotonic(),stamp=m.header.stamp.to_nsec(),
            valid=m.valid,reason=m.status_reason)),queue_size=30))
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
                if node.type in ('object_detection_node.py','semantic_inference_node.py','traffic_light_node.py','camera_runtime_node.py'):
                    node.launch_prefix = 'python3 '+shlex.quote(str(probe))+' --output '+shlex.quote(str(a.output/(node.name+'.jsonl')))+' --'
                elif node.type=='lane_vectorizer_node.py' and a.profile_lane_fit:
                    node.launch_prefix = 'python3 '+shlex.quote(str(probe))+' --lane-fit-profile --output '+shlex.quote(str(a.output/'lane-fit.profile'))+' --'
    class MeasuredViewLaunch(roslaunch.parent.ROSLaunchParent):
        def _load_config(self):
            super()._load_config()
            for node in self.config.nodes:
                if node.type in ('object_view_node.py','object_visualizer_node.py'):
                    node.launch_prefix = 'python3 '+shlex.quote(str(probe))+' --visual-codecs --output '+shlex.quote(str(a.output/('visual-'+node.name+'.jsonl')))+' --'
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
        timestamp_mode=rospy.get_param('/morai_sensor_bridge/timestamp_mode','unset (bridge default receive)'),
        use_sim_time=rospy.get_param('/use_sim_time',False),
        latency_definition='Actual model/broker compressed input callback entry -> ROS publish return; monotonic wall.',
        aggregate_definition='Per-run raw frames, linear interpolated percentiles; no mean of run medians.'), None, None
    try:
        report['qt_inventory_before']=qt_viewers()
        if any(v['owned_trial'] for v in report['qt_inventory_before']):
            raise RuntimeError('unclosed Qt from an earlier trial; do not start a new comparison')
        hashes = {str(f.relative_to(package)):hashlib.sha256(f.read_bytes()).hexdigest()
                  for pattern in ('src/**/*.py','launch/*.launch','config/**/*.yaml') for f in package.glob(pattern)}
        report['source_sha256'] = hashlib.sha256(json.dumps(hashes,sort_keys=True).encode()).hexdigest()
        report['measurement_sha256']={f:hashlib.sha256((root/'tools/perception'/f).read_bytes()).hexdigest()
            for f in ('compare_camera_runtime.py','camera_runtime_probe.py','b0_resources.py',
                      'run_camera_structure_matrix.py','run_b0_matrix.py')}
        (a.output/'source-hashes.json').write_text(json.dumps(hashes,indent=2))
        parent.start()
        report['effective_lane_batch_wait_ms'] = rospy.get_param('/camera_lane/batch_wait_ms',None)
        report['effective_fusion_hz'] = rospy.get_param('/camera_bev_fusion/max_fused_hz',None)
        deadline = time.monotonic()+120
        while not all(r in status and status[r].loaded and status[r].warmed_up for r in legacy):
            if time.monotonic() > deadline:
                raise RuntimeError('model load timeout')
            parent.spin_once(); time.sleep(.05)
        for role in active:
            rospy.wait_for_service(legacy[role]+'/set_inference_mode', 5)
            response = rospy.ServiceProxy(legacy[role]+'/set_inference_mode', SetInferenceMode)(1, 0.)
            if not response.accepted:
                raise RuntimeError(response.reason)
        deadline = time.monotonic()+30
        while not all(status[r].prepared for r in active):
            if time.monotonic() > deadline:
                raise RuntimeError('PREPARE timeout: '+repr({r:status[r].reason for r in active}))
            parent.spin_once(); time.sleep(.05)
        for role in active:
            rospy.ServiceProxy(legacy[role]+'/set_inference_mode', SetInferenceMode)(2, 0.)
        if a.gui:
            # Start only visualization and connect compressed debug sinks. The
            # sink cost is recorded separately from a real rqt window's cost.
            view_launch = MeasuredViewLaunch(parent.run_id,
                [(str(package/'launch/view.launch'), ['models:='+a.view_models])])
            view_launch.start()
            debug_topics = dict(objects='/perception/camera/objects/debug/mosaic/compressed',
                lane='/perception/camera/debug/mosaic/compressed', traffic_light='/perception/traffic_light/debug/image/compressed')
            for role, topic in debug_topics.items():
                if a.view_models != 'all' and role not in a.view_models.split(','):
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
            qt_process = subprocess.Popen(['rosrun','rqt_image_view','rqt_image_view','--clear-config',
                '/perception/camera/objects/debug/mosaic/compressed','__name:=camera_runtime_viewer'],
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
        sampler = ResourceSampler(os.environ.get('ROS_MASTER_URI','http://127.0.0.1:11311'), names, interval=.5, system_metrics=True)
        sampler.start()
        start = time.monotonic(); end = start+a.seconds
        print('MEASURE '+str(a.output), flush=True)
        while time.monotonic() < end and not rospy.is_shutdown():
            parent.spin_once(); time.sleep(.05)
        report.update(start=start, end=end, complete=time.monotonic()>=end,
                      final_status={r:dict(generation=m.generation, prepared=m.prepared, pipeline_ready=m.pipeline_ready, errors=m.errors, reason=m.reason) for r,m in status.items()})
        if qt_process and qt_process.poll() is not None:
            report['failures'].append('Qt viewer terminated during measurement')
    except Exception as exc:
        report['failures'].append(str(exc))
    finally:
        if sampler:
            resources = sampler.close()
            (a.output/'resources.json').write_text(json.dumps(resources))
            report['resources'] = resource_summary(resources,start,end)
            if not report['resources']['resource_coverage_pass']:
                report['failures'].append('insufficient resource sampling coverage; no resource acceptance')
        if a.gui and 'view_launch' in locals():
            view_launch.shutdown()
        from run_b0_matrix import stop
        if qt_process: stop(qt_process,timeout=5)
        if main_process: stop(main_process,timeout=10)
        parent.shutdown()
        report['qt_inventory_after']=qt_viewers()
        if any(v['owned_trial'] for v in report['qt_inventory_after']):
            report['failures'].append('trial Qt remains after cleanup')
        replay_stop.set()
        if replay:
            replay.join(2)
        for sub in subscribers:
            sub.unregister()
        rows = []
        for path in a.output.glob('*.jsonl'):
            rows.extend(dict(json.loads(s), probe=path.stem) for s in path.read_text().splitlines() if s)
        if start is not None and not rows:
            report['failures'].append('missing model measurement traces; no latency/supply claim')
        if start is not None:
            report['streams'] = measurements(rows, start, end, 15)
            required = dict(objects='/perception/camera/objects:',
                lane='/perception/camera/front/road_marking/class_id:',
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
        (a.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print('RESULT '+json.dumps(report.get('resources', report['failures'])), flush=True)
    return int(bool(report['failures']))


if __name__ == '__main__':
    raise SystemExit(main())
