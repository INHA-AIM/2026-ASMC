#!/usr/bin/env python3
"""Run the fixed B0 protocol serially with a fresh isolated master per trial.

Run inside the network-none GPU container after sourcing ROS, outer, nested.
Rate failures are experimental results, not reasons to discard or rerun a cell.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.parse import urlparse
from xmlrpc.client import ServerProxy

import rosbag
import yaml

from b0_measurements import make_protocol


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(4*1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def stop(process, timeout=25):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def manifest(root, config, revision):
    import cv2
    import numpy
    import torch
    import ultralytics
    bags = {'camera': config['camera']['bag'], **config['fusion']['bags']}
    bag_info = {}
    for name, path in bags.items():
        with rosbag.Bag(path, 'r') as bag:
            bag_info[name] = dict(path=path, sha256=sha(Path(path)),
                duration_sec=bag.get_end_time()-bag.get_start_time(),
                topics={topic: dict(type=info.msg_type, frames=info.message_count)
                        for topic, info in bag.get_type_and_topic_info().topics.items()})
    paths = list((root/'src/asmc_camera/src/asmc_camera').rglob('*.py'))
    for folder, suffixes in [('src/asmc_camera/launch', ['*.launch']), ('src/asmc_camera/config', ['*.yaml']),
                             ('src/lidar+sensorfusion/src/Main/src', ['*.cpp', '*.hpp']),
                             ('src/asmc_msgs/msg', ['*.msg']), ('src/asmc_camera/msg', ['*.msg']),
                             ('src/asmc_camera/srv', ['*.srv']), ('tools/perception', ['*b0*.py'])]:
        for suffix in suffixes:
            paths.extend((root/folder).rglob(suffix))
    paths += [root/'src/lidar+sensorfusion/src/Main/main.launch', root/'config/experiments/camera_lidar_b0_v1.yaml',
              root/'tools/perception/b0_main.launch']
    weights = [root/'src/asmc_camera/models/object_detection/yolo11m.pt',
               root/'src/asmc_camera/models/traffic_light/best.pt',
               root/'artifacts/perception_eval/runs/twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/best_mean.pt']
    return dict(revision=revision, created_unix=time.time(),
                interpreter=sys.executable, python=sys.version, torch=torch.__version__,
                ultralytics=ultralytics.__version__, cv2=cv2.__version__, numpy=numpy.__version__,
                gpu=subprocess.check_output(['nvidia-smi', '--query-gpu=name,memory.total,driver_version', '--format=csv,noheader'], text=True).strip(),
                source_sha256={str(p.relative_to(root)): sha(p) for p in sorted(set(paths))},
                weight_sha256={str(p.relative_to(root)): sha(p) for p in weights}, bags=bag_info)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--revision', required=True)
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[2]/'config/experiments/camera_lidar_b0_v1.yaml')
    parser.add_argument('--phases', default='camera,fusion,gps,debug')
    parser.add_argument('--manifest-only', action='store_true')
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    port = config['conditions']['ros_master_port']
    uri = urlparse(os.environ.get('ROS_MASTER_URI', ''))
    if uri.hostname not in ('localhost', '127.0.0.1') or uri.port != port:
        parser.error('isolated master URI required')
    args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[2]
    print('Fingerprinting inputs and frozen source', flush=True)
    (args.output/'manifest.json').write_text(json.dumps(manifest(root, config, args.revision), indent=2)+'\n')
    if args.manifest_only:
        return 0
    phases = args.phases.split(',')
    if set(phases)-{'camera', 'fusion', 'gps', 'debug'}:
        parser.error('unknown phase')
    jobs = make_protocol(config, phases)
    (args.output/'protocol.json').write_text(json.dumps(jobs, indent=2)+'\n')
    results, diagnostic_keys = [], set()
    def run(job, diagnostic=False):
        label = '{phase}-{bag_id}-{profile}-r{repeat}'.format(**job)
        label += '-gap' if job.get('gps_gap') else '-control' if job['phase'] == 'gps' else ''
        label += '-debug' if job.get('debug') else ''
        label += '-trace' if diagnostic else ''
        command = [sys.executable, str(root/'tools/perception/run_camera_lidar_b0.py'),
                   '--config', str(args.config), '--phase', job['phase'], '--profile', job['profile'],
                   '--bag-id', job['bag_id'], '--repeat', str(job['repeat']), '--output', str(args.output/label)]
        for key, flag in [('gps_gap', '--gps-gap'), ('debug', '--debug')]:
            if job.get(key):
                command.append(flag)
        if diagnostic:
            command.append('--diagnostic')
        core = child = None
        core_log = (args.output/(label+'.core.log')).open('w')
        stdout = (args.output/(label+'.trial.log')).open('w')
        print('START '+label, flush=True)
        try:
            # Refuse to own or terminate an already existing master.
            try:
                ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/b0_matrix')
            except (OSError, ConnectionError):
                pass
            else:
                raise RuntimeError('configured master already exists; do not reuse another session')
            core = subprocess.Popen(['roscore', '-p', str(port)], stdout=core_log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+15
            while True:
                try:
                    ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/b0_matrix')
                    break
                except (OSError, ConnectionError):
                    if core.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError('isolated roscore startup failed')
                    time.sleep(.1)
            child = subprocess.Popen(command, stdout=stdout, stderr=subprocess.STDOUT, start_new_session=True)
            child.wait()
            report = args.output/label/'report.json'
            row = dict(label=label, job=job, diagnostic=diagnostic, return_code=child.returncode,
                       report_exists=report.exists(), passed=json.loads(report.read_text())['passed'] if report.exists() else False)
            results.append(row)
            print('END '+json.dumps(row), flush=True)
            (args.output/'index.json').write_text(json.dumps(results, indent=2)+'\n')
            if child.returncode and not diagnostic and job['phase'] != 'gps':
                diagnostic_keys.add((job['phase'], job['bag_id'], job['profile'], bool(job.get('debug'))))
        finally:
            if child:
                stop(child)
            if core:
                stop(core)
            stdout.close()
            core_log.close()
    for job in jobs:
        run(job)
    for phase, bag_id, profile, debug in sorted(diagnostic_keys):
        run(dict(phase=phase, bag_id=bag_id, profile=profile, repeat=1, debug=debug), diagnostic=True)
    print(json.dumps(dict(completed=len(results), missing_reports=sum(not r['report_exists'] for r in results),
                          performance_failures=sum(not r['passed'] for r in results))), flush=True)
    return 0 if all(r['report_exists'] for r in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
