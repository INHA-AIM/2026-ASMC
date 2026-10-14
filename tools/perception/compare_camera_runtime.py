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
from b0_resources import ResourceSampler


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
        report['timing_pass'] = bool(times) and report['wall_ms']['p95'] <= 150 and report['wall_ms']['p99'] <= 200 and report['max_gap_sec'] <= .25
        reports[key] = report
    return reports


def main():
    import rospy
    import roslaunch
    import rospkg
    from asmc_camera.msg import InferenceStatus
    from asmc_camera.srv import SetInferenceMode
    from sensor_msgs.msg import Image, CompressedImage
    from asmc_msgs.msg import CameraObjectBoxArray
    from diagnostic_msgs.msg import DiagnosticArray
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--variant', choices=('R0', 'R1', 'R2'), required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=60)
    p.add_argument('--active', default='objects,lane,traffic_light')
    p.add_argument('--lane-hz', type=float, default=10)
    p.add_argument('--saved-input', type=Path, help='JSON {front,left,right,extra: [JPEG path,...]}')
    p.add_argument('--gui', action='store_true', help='Visualization publishers with connected sinks')
    p.add_argument('--qt', action='store_true', help='Also open the actual rqt viewer for this trial')
    p.add_argument('--nms-iou', type=float, default=.7)
    p.add_argument('--legacy-lidar-root', type=Path)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    rospy.init_node('camera_runtime_comparison', anonymous=True)
    package = Path(rospkg.RosPack().get_path('asmc_camera'))
    root = Path(__file__).resolve().parents[2]
    legacy = dict(objects='/camera_object_detector', lane='/camera_semantic_inference', traffic_light='/traffic_light_infer')
    active = a.active.split(',')
    status, status_rows, diagnostics, subscribers, outputs = {}, [], [], [], []
    def receive_status(role, message):
        status[role] = message
        status_rows.append(dict(role=role,at=time.monotonic(),generation=message.generation,mode=message.mode,
            prepared=message.prepared,pipeline_ready=message.pipeline_ready,errors=message.errors,reason=message.reason))
    for role, node in legacy.items():
        subscribers.append(rospy.Subscriber(node+'/inference_status', InferenceStatus,
            lambda m, r=role: receive_status(r, m), queue_size=20))
    subscribers.append(rospy.Subscriber('/perception/camera/objects', CameraObjectBoxArray,
        lambda m: outputs.append(dict(frame=m.header.frame_id, stamp=m.header.stamp.to_nsec(), seq=m.header.seq,
            valid=m.valid, boxes=[dict(label=b.source_class_name, group=b.class_name, confidence=b.confidence,
                                    xyxy=[b.xmin,b.ymin,b.xmax,b.ymax]) for b in m.boxes])), queue_size=30))
    # Keep the same normal lane consumers as the real stack. No GUI consumers
    # are added unless explicitly requested, so lazy output stays comparable.
    for topic in ('/perception/camera/objects/diagnostics', '/perception/camera/diagnostics', '/morai_sensor_bridge/diagnostics'):
        subscribers.append(rospy.Subscriber(topic, DiagnosticArray, lambda m: diagnostics.append(
            dict(at=time.monotonic(), statuses=[dict(name=s.name, values={v.key:v.value for v in s.values}) for s in m.status])), queue_size=10))
    launch = package/'launch'/('perception.launch' if a.variant == 'R0' else '_runtime.launch')
    args = ['profile:=planning', 'objects_debug_raw:=false', 'lane_enable_shared_family_debug:=false',
            'lane_target_hz:='+str(a.lane_hz), 'traffic_light_device:=cuda'] if a.variant == 'R0' else [
            'mode:=standby', 'implementation:='+('shared' if a.variant == 'R2' else 'separate'), 'lane_hz:='+str(a.lane_hz), 'nms_iou:='+str(a.nms_iou)]
    probe = root/'tools/perception/camera_runtime_probe.py'
    class MeasuredLaunch(roslaunch.parent.ROSLaunchParent):
        def _load_config(self):
            super()._load_config()
            for node in self.config.nodes:
                if node.type in ('object_detection_node.py','semantic_inference_node.py','traffic_light_node.py','camera_runtime_node.py'):
                    node.launch_prefix = 'python3 '+shlex.quote(str(probe))+' --output '+shlex.quote(str(a.output/(node.name+'.jsonl')))+' --'
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
        timestamp_mode=rospy.get_param('/morai_sensor_bridge/timestamp_mode','unset (bridge default receive)'),
        use_sim_time=rospy.get_param('/use_sim_time',False),
        latency_definition='Actual model/broker compressed input callback entry -> ROS publish return; monotonic wall.',
        aggregate_definition='Per-run raw frames, linear interpolated percentiles; no mean of run medians.'), None, None
    try:
        hashes = {str(f.relative_to(package)):hashlib.sha256(f.read_bytes()).hexdigest()
                  for pattern in ('src/**/*.py','launch/*.launch','config/**/*.yaml') for f in package.glob(pattern)}
        report['source_sha256'] = hashlib.sha256(json.dumps(hashes,sort_keys=True).encode()).hexdigest()
        (a.output/'source-hashes.json').write_text(json.dumps(hashes,indent=2))
        parent.start()
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
            view_launch = roslaunch.parent.ROSLaunchParent(parent.run_id, [str(package/'launch/view.launch')])
            view_launch.start()
            for topic in ('/perception/camera/objects/debug/mosaic/compressed', '/perception/camera/debug/mosaic/compressed',
                          '/perception/traffic_light/debug/image/compressed'):
                subscribers.append(rospy.Subscriber(topic, CompressedImage, lambda m: None, queue_size=1))
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
        names = ['/camera_bev','/camera_bev_fusion','/camera_lane_vectorizer'] + (
            ['/camera_runtime'] if a.variant == 'R2' else list(legacy.values()) if a.variant == 'R0'
            else ['/camera_objects','/camera_lane','/camera_traffic_light'])
        if a.gui:
            names += ['/camera_object_visualizer','/camera_object_visualizer_left','/camera_object_visualizer_right',
                      '/camera_object_mosaic','/camera_semantic_visualizer','/camera_bev_visualizer',
                      '/camera_traffic_light_visualizer','/lane_vector_visualizer']
        if a.qt: names.append('/camera_runtime_viewer')
        if a.legacy_lidar_root: names += ['/Main','/camera_objects_yolo_bbox_adapter']
        sampler = ResourceSampler(os.environ.get('ROS_MASTER_URI','http://127.0.0.1:11311'), names, interval=.5, system_metrics=True)
        sampler.start()
        start = time.monotonic(); end = start+a.seconds
        print('MEASURE '+str(a.output), flush=True)
        while time.monotonic() < end and not rospy.is_shutdown():
            parent.spin_once(); time.sleep(.05)
        report.update(start=start, end=end, complete=time.monotonic()>=end,
                      final_status={r:dict(generation=m.generation, prepared=m.prepared, pipeline_ready=m.pipeline_ready, errors=m.errors, reason=m.reason) for r,m in status.items()})
    except Exception as exc:
        report['failures'].append(str(exc))
    finally:
        if sampler:
            resources = sampler.close()
            (a.output/'resources.json').write_text(json.dumps(resources))
            gpu = [r['gpu'] for r in resources if r['gpu']]
            cpu = [sum(n['cpu_percent'] or 0 for n in r['nodes'].values()) for r in resources]
            report['resources'] = dict(camera_cpu_mean=sum(cpu)/len(cpu) if cpu else None,
                camera_rss_peak_mib=max((sum(n['rss_mib'] for n in r['nodes'].values()) for r in resources),default=None),
                vram_peak_mib=max((r['memory_used_mib'] for r in gpu),default=None),
                vram_peak_fraction=max((r['memory_used_mib']/r['memory_total_mib'] for r in gpu),default=None))
        if a.gui and 'view_launch' in locals():
            view_launch.shutdown()
        from run_b0_matrix import stop
        if qt_process: stop(qt_process)
        if main_process: stop(main_process)
        parent.shutdown()
        replay_stop.set()
        if replay:
            replay.join(2)
        for sub in subscribers:
            sub.unregister()
        rows = []
        for path in a.output.glob('*.jsonl'):
            rows.extend(dict(json.loads(s), probe=path.stem) for s in path.read_text().splitlines() if s)
        if start is not None:
            report['streams'] = measurements(rows, start, end, 15)
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
        (a.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print('RESULT '+json.dumps(report.get('resources', report['failures'])), flush=True)
    return int(bool(report['failures']))


if __name__ == '__main__':
    raise SystemExit(main())
