#!/usr/bin/env python3
"""Own a preview launch, a bounded drive gate, and independent model re-entry.

Driving and stationary repeat measurements remain separate experiments. This
runner never overrides a planner STOP, changes existing actors, or sets an Ego
velocity to simulate a successful drive.
"""
import argparse
from collections import defaultdict
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

from b0_measurements import output_report, resource_report
from b0_resources import ResourceSampler
from run_b0_matrix import stop
from run_coco7_live_trials import NODES, measurement_gate, provenance

ROOT = Path(__file__).resolve().parents[2]


def main():
    import rosgraph
    import rospy
    import yaml
    from asmc_camera.msg import InferenceStatus
    from asmc_camera.srv import SetInferenceMode
    from asmc_msgs.msg import CameraObjectBoxArray, TrafficLightDetection, Trajectory
    from sensor_msgs.msg import CompressedImage, Image

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime', type=Path, required=True)
    p.add_argument('--models', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--revision', required=True)
    p.add_argument('--drive-seconds', type=float, default=10.)
    p.add_argument('--reentry-seconds', type=float, default=20.)
    a = p.parse_args()
    if not 0 < a.drive_seconds <= 30 or a.reentry_seconds < 5:
        p.error('drive at most30s; re-entry at least5s')
    if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11311':
        p.error('live master11311 required')
    a.output.mkdir(parents=True, exist_ok=False)
    rospy.init_node('coco7_drive_rehearsal', anonymous=True)
    master = rosgraph.Master(rospy.get_name())
    runtime = yaml.safe_load(a.runtime.read_text())
    report = dict(scope='bounded simulator drive and stationary model re-entry; not a competition driving approval',
                  failures=[], requests=[], provenance=provenance(a.models, Path(runtime['weights']), a.revision))
    statuses = {}; anchors = {}; outputs = defaultdict(list); trajectories = []
    lock = threading.RLock(); subs = []; processes = []; handles = []
    sampler = None

    def save():
        (a.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')

    def launch(command, name):
        handle = (a.output/(name+'.log')).open('w'); handles.append(handle)
        child = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(child); report.setdefault('commands', []).append(command)
        return child

    def wait(predicate, timeout, reason):
        end = time.monotonic()+timeout
        while not predicate():
            if rospy.is_shutdown() or time.monotonic() > end or any(c.poll() is not None for c in processes):
                raise RuntimeError(reason)
            time.sleep(.03)

    def request(model, mode):
        started = time.monotonic()
        response = rospy.ServiceProxy(NODES[model]+'/set_inference_mode', SetInferenceMode)(mode, 0.)
        row = dict(model=model, mode=mode, generation=response.generation,
                   call_started=started, call_returned=time.monotonic(), accepted=response.accepted, reason=response.reason)
        report['requests'].append(row); save()
        if not response.accepted:
            raise RuntimeError('mode request rejected: '+model)
        return row

    def transition(mode, ready):
        result = {}
        for model in NODES:
            req = request(model, mode); result[model] = req
            wait(lambda: model in statuses and statuses[model].generation == req['generation'] and
                 statuses[model].mode == mode and
                 ((statuses[model].prepared and statuses[model].pipeline_ready) if ready else not statuses[model].prepared),
                 10., 'model transition failed: '+model)
        return result

    def status(model, message):
        with lock: statuses[model] = message

    def image(view, message):
        with lock:
            anchors[view, message.header.stamp.to_nsec()] = time.monotonic()
            if len(anchors) > 12000:
                for key in list(anchors)[:3000]: anchors.pop(key)

    def output(label, view, message):
        now = time.monotonic(); stamp = message.header.stamp.to_nsec()
        with lock:
            anchor = anchors.get((view, stamp))
            outputs[label].append(dict(arrival=now, stamp_ns=stamp, valid=bool(getattr(message, 'valid', True)),
                                       wall_latency_ms=(now-anchor)*1000 if anchor is not None else None,
                                       source_age_ms=(rospy.Time.now().to_nsec()-stamp)/1e6,
                                       input_anchor='matched' if anchor is not None else 'missing'))

    try:
        graph = master.getSystemState()
        if any(t in ('/ctrl_cmd', '/ctrl_cmd_0') and names for t, names in graph[0]):
            raise RuntimeError('public control publisher already present')
        running = {n for entries in graph for _, names in entries for n in names}
        if running & set(NODES.values()) or rospy.get_param('/use_sim_time', False):
            raise RuntimeError('existing inference or simulated clock; refusing mixed live trial')
        for model, node in NODES.items():
            subs.append(rospy.Subscriber(node+'/inference_status', InferenceStatus, lambda m,k=model:status(k,m), queue_size=10))
        topics = dict(front='/image_jpeg/compressed', left='/cam_left/image_jpeg/compressed',
                      right='/cam_right/image_jpeg/compressed', extra='/cam_extra/image_jpeg/compressed')
        for view, topic in topics.items():
            subs.append(rospy.Subscriber(topic, CompressedImage, lambda m,v=view:image(v,m), queue_size=50, buff_size=2**24))
        subs.append(rospy.Subscriber('/perception/camera/objects', CameraObjectBoxArray,
                    lambda m:output('yolo_'+m.header.frame_id.replace('camera_', '', 1), m.header.frame_id.replace('camera_', '', 1), m), queue_size=50))
        for view in ('front', 'left', 'right'):
            subs.append(rospy.Subscriber('/perception/camera/'+view+'/road_marking/class_id', Image,
                        lambda m,v=view:output('twinlite_'+v,v,m), queue_size=50, buff_size=2**24))
        subs.append(rospy.Subscriber('/perception/traffic_light/detection', TrafficLightDetection,
                    lambda m:output('resnet18','extra',m), queue_size=50))
        subs.append(rospy.Subscriber('/planning/trajectory', Trajectory,
                    lambda m:trajectories.append((time.monotonic(), m.valid)), queue_size=20))
        sampler = ResourceSampler(os.environ['ROS_MASTER_URI'], list(NODES.values())+['/Main','/frenet_planner','/mpc_controller'], system_metrics=True)
        sampler.start()
        launch(['roslaunch','asmc_camera','perception.launch','profile:=planning',
                'camera_objects_config:='+str(a.runtime),'objects_views:=[front, left, right]',
                'start_visualization:=false','traffic_light_wait_first:=0'], 'camera')
        wait(lambda: all(k in statuses and statuses[k].loaded and statuses[k].warmed_up for k in NODES), 90., 'models failed to load')
        transition(1, True)
        launch(['roslaunch',str(ROOT/'tools/perception/coco7_preview_stack.launch'),
                'camera_objects_conf_thres:='+str(runtime.get('conf_thres', .7))], 'preview')
        transition(2, True)
        wait(lambda: len([t for t,v in trajectories if v]) >= 3 and trajectories[-1][1] and
             time.monotonic()-trajectories[-1][0] < .25, 30., 'preview not ready with actual inputs')
        drive = [sys.executable, str(ROOT/'tools/perception/drive_coco7_preview.py'),
                 '--output',str(a.output/'drive'),'--seconds',str(a.drive_seconds)]
        with (a.output/'drive.log').open('w') as handle:
            child = subprocess.Popen(drive, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
            # The gate owns its public publisher and restoration. Give its
            # finally block time to finish even when a guard stops the trial.
            try: child.wait(timeout=a.drive_seconds+60.)
            except subprocess.TimeoutExpired:
                stop(child); raise RuntimeError('drive gate timeout')
        report['drive_return_code'] = child.returncode
        report['drive'] = json.loads((a.output/'drive/report.json').read_text()) if (a.output/'drive/report.json').exists() else None
        if not report['drive'] or not report['drive']['passed']:
            report['failures'].append('bounded drive failed; original gate report retained')
        if any(t in ('/ctrl_cmd','/ctrl_cmd_0') and names for t,names in master.getSystemState()[0]):
            raise RuntimeError('drive public publisher remained after restoration')
        # Re-entry is useful evidence even when actual motion was blocked.
        transition(0, False)
        report['standby_checked'] = time.monotonic()
        transition(1, True)
        active = transition(2, True)
        end = max(r['call_started'] for r in active.values())+a.reentry_seconds
        wait(lambda: time.monotonic() >= end, a.reentry_seconds+2., 're-entry input interrupted')
        report['reentry'] = {}
        with lock:
            for label, rows in outputs.items():
                owner = 'yolo' if label.startswith('yolo_') else 'twinlite' if label.startswith('twinlite_') else 'resnet18'
                start = active[owner]['call_started']
                row = output_report(rows,start,start+a.reentry_seconds,statuses[owner].source_cutoff.to_nsec(),a.reentry_seconds)
                passed = measurement_gate(row) and (owner == 'yolo' or row['first_second'] >= 9)
                report['reentry'][label] = dict(passed=passed, measurements=row)
                if not passed: report['failures'].append('re-entry supply: '+label)
        expected = {'yolo_'+v for v in ('front','left','right')} | {'twinlite_'+v for v in ('front','left','right')} | {'resnet18'}
        if set(report['reentry']) != expected:
            report['failures'].append('missing re-entry stream')
    except Exception as exc:
        report['failures'].append(str(exc))
    finally:
        if sampler:
            samples = sampler.close(); report['resources'] = resource_report(samples)
            (a.output/'resources.json').write_text(json.dumps(samples)+'\n')
            gpu = [r['gpu'] for r in samples if r.get('gpu')]
            if not gpu or max(g['memory_used_mib']/g['memory_total_mib'] for g in gpu) > .9:
                report['failures'].append('GPU memory unavailable or above90%')
        for child in reversed(processes): stop(child)
        for handle in handles: handle.close()
        for sub in subs: sub.unregister()
        (a.output/'outputs.json').write_text(json.dumps(dict(outputs))+'\n')
        report['passed'] = not report['failures']; save()
    print(json.dumps(dict(passed=report['passed'], failures=report['failures'])))
    return 0 if report['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
