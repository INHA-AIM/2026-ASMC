#!/usr/bin/env python3
"""Serial, isolated YOLO-only class comparisons using the existing B0 observer."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from urllib.parse import urlparse
from xmlrpc.client import ServerProxy

import yaml
from run_b0_matrix import manifest, stop
from camera_coco_measurements import make_protocol, validate_profiles


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--revision', required=True)
    parser.add_argument('--config', type=Path, default=root/'config/experiments/camera_coco_classes_v1.yaml')
    parser.add_argument('--phases', default='camera,debug,fusion')
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    port = config['conditions']['ros_master_port']
    uri = urlparse(os.environ.get('ROS_MASTER_URI', ''))
    if uri.hostname not in ('localhost', '127.0.0.1') or uri.port != port:
        parser.error('configured isolated master URI required')
    phases = args.phases.split(',')
    if set(phases)-{'camera', 'debug', 'fusion'}:
        parser.error('unknown phase')
    baseline = yaml.safe_load((root/config['object_profiles']['baseline']['config']).read_text())
    expanded = yaml.safe_load((root/config['object_profiles']['all']['config']).read_text())
    groups = yaml.safe_load((root/config['object_profiles']['all']['groups']).read_text())['class_groups']
    from ultralytics import YOLO
    # CPU metadata inspection; actual nodes perform CUDA preflight and warm-up.
    model_names = dict(YOLO(str(root/'src/asmc_camera/models/object_detection/yolo11m.pt')).names)
    errors = validate_profiles(baseline, expanded, groups, model_names)
    if errors:
        parser.error('; '.join(errors))
    args.output.mkdir(parents=True, exist_ok=False)
    print('Fingerprinting frozen inputs', flush=True)
    metadata = manifest(root, config, args.revision)
    metadata.update(all_models_loaded=False, loaded_models=['yolo'], class_names=model_names,
                    source_config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest())
    paths = [args.config, Path(__file__).resolve(), Path(__file__).with_name('camera_coco_measurements.py')]
    for entry in config['object_profiles'].values():
        paths += [root/entry['config'], root/entry['groups']]
    for path in paths:
        path = path.resolve()
        metadata['source_sha256'][str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
    (args.output/'manifest.json').write_text(json.dumps(metadata, indent=2)+'\n')
    jobs = make_protocol(config, phases)
    (args.output/'protocol.json').write_text(json.dumps(jobs, indent=2)+'\n')
    results = []
    for job in jobs:
        label = '{phase}-{bag_id}-{classes}-r{repeat}'.format(**job)+('-debug' if job['debug'] else '')
        profile = config['object_profiles'][job['classes']]
        command = [sys.executable, str(root/'tools/perception/run_camera_lidar_b0.py'),
                   '--config', str(args.config), '--phase', job['phase'], '--profile', 'C1',
                   '--bag-id', job['bag_id'], '--repeat', str(job['repeat']),
                   '--output', str(args.output/label), '--objects-only',
                   '--objects-config', str(root/profile['config']),
                   '--objects-class-groups-config', str(root/profile['groups'])]
        if job['debug']:
            command.append('--debug')
        core = child = None
        print('START '+label, flush=True)
        with (args.output/(label+'.core.log')).open('w') as core_log, (args.output/(label+'.trial.log')).open('w') as log:
            try:
                try:
                    ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/coco_matrix')
                except (OSError, ConnectionError):
                    pass
                else:
                    raise RuntimeError('master already exists; never take over another session')
                core = subprocess.Popen(['roscore', '-p', str(port)], stdout=core_log, stderr=subprocess.STDOUT, start_new_session=True)
                deadline = time.monotonic()+15
                while True:
                    try:
                        ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/coco_matrix')
                        break
                    except (OSError, ConnectionError):
                        if core.poll() is not None or time.monotonic()>deadline:
                            raise RuntimeError('isolated master startup failed')
                        time.sleep(.1)
                child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                child.wait()
                path = args.output/label/'report.json'
                report = json.loads(path.read_text()) if path.exists() else None
                row = dict(label=label, job=job, command=command, return_code=child.returncode,
                           report_exists=report is not None, passed=bool(report and report['passed']))
                results.append(row)
                (args.output/'index.json').write_text(json.dumps(results, indent=2)+'\n')
                print('END '+json.dumps(row), flush=True)
            finally:
                if child:
                    stop(child)
                if core:
                    stop(core)
    # Performance failure is a result. Missing evidence is a protocol failure.
    return 0 if all(row['report_exists'] for row in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
