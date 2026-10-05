#!/usr/bin/env python3
"""Bounded review-node measurement; preserves bridge and existing ROS nodes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

p=argparse.ArgumentParser();p.add_argument('profile',type=Path);p.add_argument('output',type=Path)
p.add_argument('--seconds',type=float,default=70);p.add_argument('--simulator-smoke',action='store_true')
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
repo=Path(__file__).resolve().parents[3];review=repo/'tools/lidar/ground_live_review'
existing=subprocess.check_output(['rosnode','list'],text=True).splitlines()
if '/ground_compare_live' in existing:raise RuntimeError('review node already running; will not replace it')
report={'profile':a.profile.name,'profile_sha256':hashlib.sha256(a.profile.read_bytes()).hexdigest(),
        'seconds':a.seconds,'existing_nodes':existing,'smoke_requested':a.simulator_smoke}
launch=None;monitor=None
try:
    with (a.output/'launch.log').open('x') as log,(a.output/'monitor.log').open('x') as mon:
        launch=subprocess.Popen(['roslaunch','asmc_lidar','ground_compare_live.launch','rviz:=false',
            'comparison_config:='+str(a.profile.resolve())],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        time.sleep(3)
        if launch.poll() is not None:raise RuntimeError('review launch exited')
        monitor=subprocess.Popen([sys.executable,str(review/'monitor.py'),str(a.output/'monitor.json'),
            '--seconds',str(a.seconds)],stdout=mon,stderr=subprocess.STDOUT)
        if a.simulator_smoke:
            time.sleep(12)
            with (a.output/'simulator_smoke.log').open('x') as smoke:
                # The existing helper refuses moving Ego / nearby forward actors;
                # all actual control requests are bounded and restore the original pose/mode.
                run=subprocess.run([sys.executable,str(review/'simulator_smoke.py'),str(a.output/'simulator_smoke.json'),
                    '--confirm-simulator-move'],stdout=smoke,stderr=subprocess.STDOUT)
                report['smoke_exit_code']=run.returncode
        report['monitor_exit_code']=monitor.wait(timeout=a.seconds+20)
finally:
    if monitor and monitor.poll() is None:
        monitor.terminate();monitor.wait(timeout=10)
    if launch and launch.poll() is None:
        os.killpg(launch.pid,signal.SIGINT)
        try:launch.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=10)
    report['remaining_nodes']=subprocess.check_output(['rosnode','list'],text=True).splitlines()
    (a.output/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
if report.get('monitor_exit_code')!=0:raise SystemExit('no successful live measurement')
