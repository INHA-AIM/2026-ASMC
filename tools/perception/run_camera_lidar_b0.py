#!/usr/bin/env python3
"""One isolated B0 trial. Keeps all models resident; never commands a vehicle."""
import argparse
from collections import defaultdict
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from urllib.parse import urlparse

import rosbag
import rosgraph
import rospy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import CompressedImage, Image, Imu, NavSatFix, PointCloud2
from morai_msgs.msg import GPSMessage, EgoVehicleStatus, CtrlCmd
from Msg.msg import TrackedObjectArray, Yolo_bbox_arr
from asmc_msgs.msg import CameraObjectBoxArray, TrafficLightDetection, LaneBoundaryFamily
from asmc_camera.msg import InferenceStatus
from asmc_camera.srv import SetInferenceMode

from b0_measurements import output_report, output_gate, resource_report, fusion_report, gps_phase_report
from b0_resources import ResourceSampler
from analyze_fusion_trace import summarize as summarize_trace
from lifecycle_measurements import trace_trial, trace_integrity

NODES = dict(twinlite='/camera_semantic_inference', yolo='/camera_object_detector', resnet18='/traffic_light_infer')
MODEL_OUTPUTS = dict(twinlite=['front', 'left', 'right', 'family'], yolo=['yolo'], resnet18=['resnet18'])
IMAGE_TOPICS = dict(front='/image_jpeg/compressed', left='/cam_left/image_jpeg/compressed',
                    right='/cam_right/image_jpeg/compressed', extra='/cam_extra/image_jpeg/compressed')


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[2]/'config/experiments/camera_lidar_b0_v1.yaml')
    parser.add_argument('--phase', choices=['camera', 'fusion', 'gps'], required=True)
    parser.add_argument('--profile', choices=['C1', 'C2', 'C3', 'C4', 'C5', 'C6'], required=True)
    parser.add_argument('--bag-id', choices=['camera', 'primary', 'shadow'], required=True)
    parser.add_argument('--repeat', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--diagnostic', action='store_true')
    parser.add_argument('--gps-gap', action='store_true')
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    master_uri = urlparse(os.environ.get('ROS_MASTER_URI', ''))
    if master_uri.hostname not in ('localhost', '127.0.0.1') or master_uri.port != config['conditions']['ros_master_port']:
        parser.error('use the isolated configured master; never the shared live master')
    if args.phase == 'camera' and args.bag_id != 'camera':
        parser.error('camera phase requires camera bag')
    if args.phase != 'camera' and args.bag_id == 'camera':
        parser.error('fusion requires Ego-equipped primary/shadow bag')
    if args.phase != 'camera' and args.profile not in config['fusion']['profiles']:
        parser.error('fusion/GPS profiles require ACTIVE YOLO')
    if args.gps_gap and args.phase != 'gps':
        parser.error('GPS injection is diagnostic only')
    args.output.mkdir(parents=True, exist_ok=False)
    bag_path = config['camera']['bag'] if args.bag_id == 'camera' else config['fusion']['bags'][args.bag_id]
    with rosbag.Bag(bag_path, 'r') as bag:
        bag_start_ns = int(round(bag.get_start_time()*1e9))
        bag_seconds = bag.get_end_time()-bag.get_start_time()
        bag_topics = sorted(bag.get_type_and_topic_info().topics)
    rospy.set_param('/use_sim_time', True)
    rospy.init_node('camera_lidar_b0_trial', anonymous=True)
    lock = threading.RLock()
    inputs, outputs = defaultdict(list), defaultdict(list)
    arrivals, statuses, status_rows = {}, {}, []
    requests, tracks, bbox, rosout, diagnostics, graphs, relays = [], [], [], [], [], [], []
    controls, processes, streams, subscribers, commands = [], [], [], [], []
    active_models = config['profiles'][args.profile]
    traced = args.diagnostic or args.phase == 'gps'
    report = dict(schema_version=1, profile=args.profile, phase=args.phase, bag_id=args.bag_id,
                  bag=bag_path, bag_duration_sec=bag_seconds, repeat=args.repeat,
                  all_models_loaded=True, active_models=active_models, trace_enabled=traced,
                  camera_debug=args.debug, main_intrinsic_fusion_debug=args.phase != 'camera',
                  gps_gap=args.gps_gap, commands=commands, failures=[], checks={},
                  source_config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
                  timing_anchor='server_acceptance' if traced else 'rpc_start_conservative')
    report['execution_source_sha256'] = {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
        for path in Path(__file__).resolve().parent.glob('*b0*.py')}
    camera = main_process = playback = None
    started = ended = None

    def receive_input(name, message):
        now = time.monotonic()
        stamp = message.header.stamp.to_nsec() if hasattr(message, 'header') else 0
        row = dict(arrival=now, ros_ns=rospy.Time.now().to_nsec(), stamp_ns=stamp)
        with lock:
            inputs[name].append(row)
            key = stamp//1000000*1000000 if name == 'lidar' else stamp
            arrivals[name, key] = now

    def receive_output(name, message):
        now, stamp = time.monotonic(), message.header.stamp.to_nsec()
        source = dict(yolo='front', resnet18='extra', lidar='lidar').get(name, name)
        with lock:
            matches = [arrivals[v, stamp] for v in ('front', 'left', 'right') if (v, stamp) in arrivals] if name == 'family' else (
                [arrivals[source, stamp]] if (source, stamp) in arrivals else [])
            row = dict(arrival=now, ros_ns=rospy.Time.now().to_nsec(), stamp_ns=stamp,
                       frame_id=message.header.frame_id, valid=bool(getattr(message, 'valid', True)),
                       source_age_ms=(rospy.Time.now().to_nsec()-stamp)/1e6,
                       wall_latency_ms=(now-matches[0])*1000 if len(matches) == 1 else None,
                       input_anchor='matched' if len(matches) == 1 else 'ambiguous' if matches else 'missing')
            outputs[name].append(row)
            if name == 'lidar':
                tracks.append(dict(row, objects=[[o.id, o.classification, o.yolo_class] for o in message.objects],
                                   centers={str(o.id): [o.cluster_center.x, o.cluster_center.y, o.cluster_center.z] for o in message.objects},
                                   geometry=[dict(id=o.id, length=o.length, width=o.width, height=o.height,
                                                  age=o.age, miss_count=o.miss_count) for o in message.objects]))
            elif name == 'yolo':
                row['boxes'] = [dict(label=b.source_class_name, group=b.class_name, confidence=b.confidence,
                                     xyxy=[b.xmin, b.ymin, b.xmax, b.ymax]) for b in message.boxes]

    def receive_status(model, message):
        with lock:
            statuses[model] = message
            status_rows.append(dict(model=model, arrival=time.monotonic(), ros_ns=rospy.Time.now().to_nsec(),
                mode=message.mode, generation=message.generation, loaded=message.loaded,
                warmed_up=message.warmed_up, prepared=message.prepared, pipeline_ready=message.pipeline_ready,
                successful_batches=message.successful_batches, errors=message.errors, reason=message.reason,
                processing_hz=message.processing_hz, source_cutoff_ns=message.source_cutoff.to_nsec()))

    def receive_log(message):
        with lock:
            if message.name == '/Main':
                rosout.append(dict(name=message.name, message=message.msg, arrival=time.monotonic(),
                                   ros_ns=rospy.Time.now().to_nsec()))

    def receive_bbox(message):
        with lock:
            bbox.append(dict(arrival=time.monotonic(), stamp_ns=message.header.stamp.to_nsec(),
                             frame_id=message.header.frame_id, objc=message.objc))

    for view, topic in IMAGE_TOPICS.items():
        subscribers.append(rospy.Subscriber(topic, CompressedImage, lambda m, v=view: receive_input(v, m), queue_size=100, buff_size=2**24))
    for name, topic, kind in [('lidar', '/velodyne_points', PointCloud2), ('gps', '/gps', GPSMessage),
                             ('gps_fix', '/gps/fix', NavSatFix), ('imu', '/imu', Imu), ('ego', '/Ego_topic', EgoVehicleStatus)]:
        subscribers.append(rospy.Subscriber(topic, kind, lambda m, n=name: receive_input(n, m), queue_size=100, buff_size=2**24))
    topics = dict(yolo=('/perception/camera/objects', CameraObjectBoxArray),
                  resnet18=('/perception/traffic_light/detection', TrafficLightDetection),
                  family=('/perception/lane/boundary_family', LaneBoundaryFamily),
                  lidar=('/lidar/tracked_objects_map', TrackedObjectArray))
    topics.update({v: ('/perception/camera/'+v+'/road_marking/class_id', Image) for v in ('front', 'left', 'right')})
    for name, (topic, kind) in topics.items():
        subscribers.append(rospy.Subscriber(topic, kind, lambda m, n=name: receive_output(n, m), queue_size=100, buff_size=2**24))
    for model, node in NODES.items():
        rospy.set_param(node+'/lifecycle_trace_path', str(args.output/(model+'.lifecycle.jsonl')) if traced else '')
        subscribers.append(rospy.Subscriber(node+'/inference_status', InferenceStatus,
                           lambda m, n=model: receive_status(n, m), queue_size=100))
    subscribers.append(rospy.Subscriber('/rosout', Log, receive_log, queue_size=1000))
    subscribers.append(rospy.Subscriber('/YOLO/BBox', Yolo_bbox_arr, receive_bbox, queue_size=100))
    subscribers.append(rospy.Subscriber('/perception/camera/objects/diagnostics', DiagnosticArray,
        lambda m: diagnostics.append(dict(arrival=time.monotonic(), status=[dict(level=s.level, message=s.message,
                                     values={v.key: v.value for v in s.values}) for s in m.status])), queue_size=100))
    for topic in ('/ctrl_cmd', '/ctrl_cmd_0'):
        subscribers.append(rospy.Subscriber(topic, CtrlCmd, lambda m, t=topic: controls.append(dict(topic=t, arrival=time.monotonic())), queue_size=100))
    if args.debug:
        debug_topics = ['/perception/camera/objects/debug/image/compressed', '/perception/camera/debug/mosaic/compressed',
                        '/perception/bev/debug/comparison/compressed', '/perception/traffic_light/debug/image/compressed']
        for topic in debug_topics:
            subscribers.append(rospy.Subscriber(topic, CompressedImage,
                lambda m, n=topic: receive_output(n, m), queue_size=10, buff_size=2**24))
        subscribers.append(rospy.Subscriber('/perception/camera/objects/debug/raw', CameraObjectBoxArray,
                           lambda m: receive_output('debug_raw', m), queue_size=100))
    if args.phase == 'gps':
        for topic, kind in [('/gps', GPSMessage), ('/gps/fix', NavSatFix)]:
            publisher = rospy.Publisher(topic, kind, queue_size=100)
            def relay(message, topic=topic, publisher=publisher):
                elapsed = (rospy.Time.now().to_nsec()-bag_start_ns)/1e9
                delivered = not(args.gps_gap and config['gps']['gap_start_sec'] <= elapsed < config['gps']['gap_end_sec'])
                with lock:
                    relays.append(dict(topic=topic, elapsed_sec=elapsed, delivered=delivered,
                                       stamp_ns=message.header.stamp.to_nsec() if hasattr(message, 'header') else None))
                if delivered:
                    publisher.publish(message)
            subscribers.append(rospy.Subscriber('/b0/source'+topic, kind, relay, queue_size=100))

    master = rosgraph.Master('/b0_probe')
    sampler = ResourceSampler(master, list(NODES.values())+['/camera_bev', '/camera_bev_fusion', '/camera_lane_vectorizer', '/Main'], config['resource_sample_sec'])
    sampler.start()
    def launch(label, command):
        stream = (args.output/(label+'.log')).open('w')
        streams.append(stream)
        commands.append(dict(label=label, argv=command, launched_monotonic=time.monotonic()))
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        commands[-1]['pid'] = process.pid
        processes.append(process)
        return process
    def wait(predicate, description, timeout):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            with lock:
                if predicate():
                    return
            if any(p.poll() is not None for p in (camera, main_process) if p is not None):
                raise RuntimeError('node exited while waiting: '+description)
            if playback and playback.poll() is not None:
                raise RuntimeError('bag ended while waiting: '+description)
            time.sleep(.005)
        raise RuntimeError('timeout: '+description)
    def request(model, mode):
        called = time.monotonic()
        reply = rospy.ServiceProxy(NODES[model]+'/set_inference_mode', SetInferenceMode)(mode, config['conditions']['target_hz'][model])
        row = dict(model=model, mode=mode, generation=reply.generation, call_started=called,
                   call_returned=time.monotonic(), accepted=reply.accepted, reason=reply.reason)
        requests.append(row)
        if not reply.accepted:
            raise RuntimeError('request rejected: '+str(row))
        return row
    def loaded():
        return all(m in statuses and statuses[m].loaded and statuses[m].warmed_up for m in NODES)
    try:
        launch_start = time.monotonic()
        initial = {m: 2 if m in active_models and args.phase != 'camera' else 0 for m in NODES}
        command = ['roslaunch', 'asmc_camera', 'perception.launch', 'profile:=planning',
                   'start_visualization:='+str(args.debug).lower(), 'objects_debug_raw:='+str(args.debug).lower(),
                   'lane_enable_shared_family_debug:='+str(args.debug).lower(), 'traffic_light_wait_first:=0',
                   'lane_initial_mode:='+str(initial['twinlite']), 'objects_initial_mode:='+str(initial['yolo']),
                   'traffic_light_initial_mode:='+str(initial['resnet18'])]
        camera = launch('camera', command)
        for node in NODES.values():
            rospy.wait_for_service(node+'/set_inference_mode', timeout=60)
        wait(loaded, 'all resident models loaded and dummy warmed', 60)
        report['load_dummy_warmup_sec'] = time.monotonic()-launch_start
        if args.phase != 'camera':
            main_command = ['roslaunch', 'Main', 'main.launch', 'start_yolo:=false', 'rviz:=false', 'start_velodyne_driver:=false']
            if traced:
                main_command.append('fusion_trace_path:='+str(args.output/'fusion.jsonl'))
            main_process = launch('main', main_command)
            wait(lambda: len(dict(master.getSystemState()[0]).get('/YOLO/BBox', [])) == 1, 'single adapter publisher', 30)
        play = ['rosbag', 'play', bag_path, '--clock', '--quiet', '--rate=1.0', '--delay=1.0']
        if args.phase == 'camera':
            play += ['--start='+str(config['camera']['start_sec']), '--duration='+str(config['camera']['duration_sec'])]
        if args.phase == 'gps':
            play += ['/gps:=/b0/source/gps', '/gps/fix:=/b0/source/gps/fix']
        playback = launch('bag', play)
        if args.phase == 'camera':
            wait(lambda: all(inputs[v] for v in IMAGE_TOPICS), 'four input cameras', 10)
            time.sleep(config['camera']['standby_sec'])
            report['checks']['standby_no_compute'] = all(s.successful_batches == 0 for s in statuses.values())
            prepare = [request(m, 1) for m in active_models]
            wait(lambda: all(r['model'] in statuses and statuses[r['model']].generation == r['generation']
                 and statuses[r['model']].mode == 1 and statuses[r['model']].prepared
                 and statuses[r['model']].pipeline_ready for r in prepare), 'all acknowledged PREPARE generations', config['camera']['prepare_timeout_sec'])
            report['prepare_ready_monotonic'] = time.monotonic()
            active = [request(m, 2) for m in active_models]
            started = min(r['call_started'] for r in active)
            common_start = max(r['call_returned'] for r in active)
            report['common_active_started'] = common_start
            wait(lambda: time.monotonic() >= common_start+config['camera']['active_sec'], 'complete ACTIVE window', config['camera']['active_sec']+2)
        else:
            wait(lambda: bool(inputs['front']) and bool(inputs['lidar']), 'camera and LiDAR input', 10)
            started = min(inputs['front'][0]['arrival'], inputs['lidar'][0]['arrival'])
            deadline = time.monotonic()+bag_seconds+30
            while playback.poll() is None:
                if time.monotonic() > deadline:
                    raise RuntimeError('bag timeout')
                if camera.poll() is not None or main_process.poll() is not None:
                    raise RuntimeError('inference/Main exited during replay')
                graphs.append(dict(arrival=time.monotonic(), publishers=dict(master.getSystemState()[0]).get('/YOLO/BBox', [])))
                time.sleep(.25)
            if playback.returncode:
                raise RuntimeError('bag playback failed')
            time.sleep(.6)
        ended = time.monotonic()
        with lock:
            report['checks']['other_models_standby'] = all(statuses[m].mode == 0 for m in NODES if m not in active_models)
    except Exception as exc:
        report['failures'].append(str(exc))
    finally:
        for process in reversed(processes):
            stop(process)
        resource_rows = sampler.close()
        for stream in streams:
            stream.close()
        for subscriber in subscribers:
            subscriber.unregister()
        trace_rows = {}
        if traced:
            for model in NODES:
                path = args.output/(model+'.lifecycle.jsonl')
                trace_rows[model] = [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
            report['checks']['complete_model_traces'] = all(trace_integrity(rows) for rows in trace_rows.values())
        report['measurements'] = {}
        try:
            for model in active_models:
                record = next((r for r in requests if r['model'] == model and r['mode'] == 2), None)
                begin, cutoff = started, 0
                if record:
                    matching = [s for s in status_rows if s['model'] == model and s['generation'] == record['generation']]
                    if not matching:
                        raise ValueError('no status for acknowledged ACTIVE generation')
                    cutoff, begin = matching[0]['source_cutoff_ns'], record['call_started']
                    if traced:
                        begin = trace_trial(record, trace_rows[model])['accepted_monotonic']
                for name in MODEL_OUTPUTS[model]:
                    measurement = output_report(outputs[name], begin, ended, cutoff,
                                                config['camera']['active_sec'] if args.phase == 'camera' else None) if begin and ended else dict(complete=False)
                    report['measurements'][name] = measurement
                    report['checks'][name+'_rate'] = output_gate(name, measurement, lifecycle=args.phase == 'camera')
            report['checks']['no_control_messages'] = not controls
            report['resources'] = resource_report([r for r in resource_rows if started and ended and started <= r['arrival'] <= ended])
            report['resource_sampler_cost_ms'] = __import__('b0_measurements').quantiles([r['sampling_ms'] for r in resource_rows])
            report['resources']['all_phases'] = resource_report(resource_rows)
            report['last_diagnostics'] = diagnostics[-1] if diagnostics else None
            report['checks']['gpu_sample_available'] = report['resources']['gpu_available']
            if args.phase != 'camera':
                main_log = (args.output/'main.log').read_text()
                report['fusion'] = fusion_report(main_log, rosout, inputs['lidar'], tracks)
                f = report['fusion']
                report['checks']['single_bbox_publisher'] = bool(graphs) and all(g['publishers'] == ['/camera_objects_yolo_bbox_adapter'] for g in graphs)
                report['checks']['fusion_complete_accounting'] = f['accounting_complete']
                report['checks']['fusion_offset'] = f['offset_p95_max_ms'] is not None and f['offset_p95_max_ms'] <= 50.
                if traced:
                    path = args.output/'fusion.jsonl'
                    rows = [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
                    report['fusion_trace'] = summarize_trace(rows)
                if args.phase == 'gps':
                    report['gps_phases'] = gps_phase_report(inputs, tracks, relays, bag_start_ns,
                        config['gps']['gap_start_sec'], config['gps']['gap_end_sec'])
                    report['gps_pose_messages'] = [r for r in rosout if 'pose' in r['message'].lower() or 'gps' in r['message'].lower()]
        except Exception as exc:
            report['failures'].append('analysis: '+str(exc))
        report['passed'] = not report['failures'] and bool(report['checks']) and all(report['checks'].values())
        report['classification_or_count_acceptance'] = 'computed_across_repeats_by_matrix_analysis'
        observation = dict(inputs=inputs, outputs=outputs, statuses=status_rows, requests=requests,
                           tracks=tracks, bbox=bbox, rosout=rosout, diagnostics=diagnostics,
                           graph_samples=graphs, gps_relays=relays, control_messages=controls)
        (args.output/'observations.json').write_text(json.dumps(observation)+'\n')
        (args.output/'resources.json').write_text(json.dumps(resource_rows)+'\n')
        report['execution_return_codes'] = {commands[i]['label']: p.returncode for i, p in enumerate(processes)}
        (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(dict(output=str(args.output), passed=report['passed'], checks=report['checks'], failures=report['failures'])), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
