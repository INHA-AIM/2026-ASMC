#!/usr/bin/env python3
"""Reuse the existing ROS field/clock/drop contract with a one-method candidate."""
import argparse
import itertools
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import hashlib
import rospy

p=argparse.ArgumentParser();p.add_argument('input',type=Path);p.add_argument('v1',type=Path)
p.add_argument('candidate',type=Path);p.add_argument('method',choices=['linefit','patchwork'])
p.add_argument('--output',type=Path)
a=p.parse_args()
if a.output and a.output.exists():raise FileExistsError(a.output)
active=Path(rospy.get_param('/ground_compare_live/comparison_config'))
active_sha=hashlib.sha256(active.read_bytes()).hexdigest()
with a.v1.open() as v1,a.candidate.open() as candidate,tempfile.NamedTemporaryFile(mode='w+',suffix='.jsonl') as result:
    count=0
    for base,changed in itertools.islice(zip(v1,candidate),12):
        x,y=json.loads(base),json.loads(changed)
        if y['config_sha256']!=active_sha:raise ValueError('active ROS configuration differs from frozen candidate')
        if x['input_id']!=y['input_id'] or x['stamp_ns']!=y['stamp_ns'] or x['raw_to_voxel']!=y['raw_to_voxel']:
            raise ValueError('unaligned candidate')
        x['methods'][a.method]=y['methods'][a.method];result.write(json.dumps(x)+'\n')
        count+=1
    if count!=12:raise ValueError('requires twelve complete comparison observations')
    result.flush()
    run=subprocess.run([sys.executable,str(Path(__file__).resolve().parents[1]/'ground_live_review/test_ros_contract.py'),str(a.input),result.name],check=True,stdout=subprocess.PIPE,text=True)
    evidence=json.loads(run.stdout.strip().splitlines()[-1]);evidence['config_sha256']=active_sha
    evidence['candidate']=str(a.candidate);evidence['method']=a.method
    if a.output:
        with a.output.open('x') as f:json.dump(evidence,f,indent=2)
    print(json.dumps(evidence))
