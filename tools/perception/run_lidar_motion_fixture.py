#!/usr/bin/env python3
"""Run all synthetic motion profiles with production functions and retain failures."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from xmlrpc.client import ServerProxy
from run_b0_matrix import stop


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--binary',type=Path,required=True);args=p.parse_args()
    uri='http://127.0.0.1:11314'
    if os.environ.get('ROS_MASTER_URI')!=uri:p.error('isolated master 11314 required')
    try:ServerProxy(uri).getPid('/motion_guard')
    except (OSError,ConnectionError):pass
    else:p.error('refuse to reuse a master')
    args.output.mkdir(parents=True,exist_ok=False)
    tools=Path(__file__).parent
    subprocess.run([sys.executable,str(tools/'make_lidar_motion_fixture.py'),'--output',str(args.output/'fixture')],check=True)
    index=[];core=None
    with (args.output/'master.log').open('w') as log:
        try:
            core=subprocess.Popen(['roscore','-p','11314'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+15
            while True:
                try:ServerProxy(uri).getPid('/motion_guard');break
                except (OSError,ConnectionError):
                    if core.poll() is not None or time.monotonic()>deadline:raise RuntimeError('master startup failed')
                    time.sleep(.05)
            for profile in ('legacy','common_pitch','common_roll_pitch'):
                label=args.output/profile
                command=[str(args.binary),str(args.output/'fixture/synthetic.bag'),profile,str(label)+'.jsonl',str(label)+'.trace.jsonl']
                with Path(str(label)+'.log').open('w') as trial:
                    rc=subprocess.run(command,stdout=trial,stderr=subprocess.STDOUT).returncode
                index.append(dict(profile=profile,return_code=rc))
                (args.output/'index.json').write_text(json.dumps(index,indent=2)+'\n')
                if rc:raise RuntimeError('preserved failed profile '+profile)
                subprocess.run([sys.executable,str(tools/'analyze_lidar_motion_fixture.py'),
                    '--truth',str(args.output/'fixture/truth.json'),'--profile',profile,
                    '--trace',str(label)+'.trace.jsonl','--output',str(label)+'.analysis.json'],check=True)
        finally:
            if core:stop(core)
    manifest=dict(binary_sha256=hashlib.sha256(args.binary.read_bytes()).hexdigest(),
        source_sha256={str(f.name):hashlib.sha256(f.read_bytes()).hexdigest() for f in tools.glob('*lidar*motion*py')},
        fixture=json.loads((args.output/'fixture/manifest.json').read_text()),
        scope='synthetic ground/object and straight speed GT; later attitude stress uses scalar CAN and is diagnostic')
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')

if __name__=='__main__':main()
