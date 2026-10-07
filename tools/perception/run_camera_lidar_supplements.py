#!/usr/bin/env python3
"""Run predeclared extra diagnostics, retaining the fixed 51-trial protocol."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
from xmlrpc.client import ServerProxy

import yaml
from run_b0_matrix import manifest, stop


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--revision', required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    config_path = root/'config/experiments/camera_lidar_b0_v1.yaml'
    config = yaml.safe_load(config_path.read_text())
    uri = 'http://127.0.0.1:%d' % config['conditions']['ros_master_port']
    if os.environ.get('ROS_MASTER_URI') != uri:
        parser.error('isolated configured master required')
    jobs = json.loads(args.jobs.read_text())
    labels = [j['label'] for j in jobs]
    if len(set(labels)) != len(labels) or any(not re.fullmatch('[A-Za-z0-9_-]+', s) for s in labels):
        parser.error('unique plain labels required')
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'protocol.json').write_text(json.dumps(jobs, indent=2)+'\n')
    (args.output/'manifest.json').write_text(json.dumps(manifest(root, config, args.revision), indent=2)+'\n')
    results = []
    for job in jobs:
        label = job['label']
        core = child = None
        print('START '+label, flush=True)
        with (args.output/(label+'.core.log')).open('w') as core_log, (args.output/(label+'.trial.log')).open('w') as trial_log:
            try:
                try:
                    ServerProxy(uri).getPid('/b0_supplement')
                except (OSError, ConnectionError):
                    pass
                else:
                    raise RuntimeError('existing master; refuse to reuse another session')
                core = subprocess.Popen(['roscore', '-p', str(config['conditions']['ros_master_port'])],
                    stdout=core_log, stderr=subprocess.STDOUT, start_new_session=True)
                deadline = time.monotonic()+15
                while True:
                    try:
                        ServerProxy(uri).getPid('/b0_supplement')
                        break
                    except (OSError, ConnectionError):
                        if core.poll() is not None or time.monotonic() > deadline:
                            raise RuntimeError('supplement master startup failed')
                        time.sleep(.05)
                command = [sys.executable, str(root/'tools/perception/run_camera_lidar_b0.py'),
                    '--config', str(config_path), '--output', str(args.output/label),
                    '--phase', job['phase'], '--bag-id', job['bag_id'], '--profile', job['profile'],
                    '--repeat', str(job['repeat'])]
                if job.get('trace'):
                    command.append('--diagnostic')
                child = subprocess.Popen(command, stdout=trial_log, stderr=subprocess.STDOUT, start_new_session=True)
                child.wait()
                path = args.output/label/'report.json'
                row = dict(label=label, job=job, diagnostic=bool(job.get('trace')), return_code=child.returncode,
                           report_exists=path.exists(), passed=json.loads(path.read_text())['passed'] if path.exists() else False)
                results.append(row)
                (args.output/'index.json').write_text(json.dumps(results, indent=2)+'\n')
                print('END '+json.dumps(row), flush=True)
            finally:
                if child:
                    stop(child)
                if core:
                    stop(core)
    return 0 if all(r['report_exists'] for r in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
