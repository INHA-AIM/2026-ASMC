#!/usr/bin/env python3
"""Continuous CPU ground/cluster/tracker replay, isolated from vehicle control."""
import argparse
import hashlib
import gzip
import rosbag
import json
import os
from pathlib import Path
import subprocess
import sys
from xmlrpc.client import ServerProxy
import time
import yaml
from run_b0_matrix import stop


def stream_sha(stream):
    h=hashlib.sha256()
    for block in iter(lambda:stream.read(1024*1024),b""):h.update(block)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--fixtures',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--binary',type=Path,required=True)
    p.add_argument('--revision',required=True)
    p.add_argument('--trace',action='store_true')
    args=p.parse_args()
    uri='http://127.0.0.1:11314'
    if os.environ.get('ROS_MASTER_URI')!=uri:p.error('isolated master 11314 required')
    try:ServerProxy(uri).getPid('/cpu_replay_guard')
    except (OSError,ConnectionError):pass
    else:p.error('refuse to reuse a master')
    root=Path(__file__).resolve().parents[2]
    cfg=yaml.safe_load((root/'config/experiments/lidar_camera_frame_boundaries_v1.yaml').read_text())
    args.output.mkdir(parents=True,exist_ok=False)
    sources=list((root/'src/asmc_lidar').rglob('*'))+list((root/'src/lidar+sensorfusion/src/Main').rglob('*'))
    sources+=list((root/'tools/perception').glob('*lidar*continuous*.py'))
    manifest=dict(revision=args.revision,clock='frozen bag availability; not actual callback or performance measurement',
        binary_sha256=hashlib.sha256(args.binary.read_bytes()).hexdigest(),
        sources={str(x.relative_to(root)):hashlib.sha256(x.read_bytes()).hexdigest() for x in sources if x.is_file()},
        fixture_manifest=json.loads((args.fixtures/'manifest.json').read_text()),
        retained_history=True,random_seed=0,thresholds_changed=False)
    manifest["expected_scans"]={}
    for bag_id in cfg["bags"]:
        with rosbag.Bag(str(args.fixtures/(bag_id+"-normal.bag"))) as bag:
            manifest["expected_scans"][bag_id]=bag.get_message_count(topic_filters=["/velodyne_points"])
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    index=[];core=None
    with (args.output/'master.log').open('w') as log:
        try:
            core=subprocess.Popen(['roscore','-p','11314'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+15
            while True:
                try:ServerProxy(uri).getPid('/cpu_replay_guard');break
                except (OSError,ConnectionError):
                    if time.monotonic()>deadline or core.poll() is not None:raise RuntimeError('master startup failed')
                    time.sleep(.05)
            for bag, settings in cfg['bags'].items():
                for profile in cfg['profiles']:
                    for mode in (['normal'] if profile=='legacy' else ['normal','invalid','missing']):
                        for repeat in range(1,3 if mode=='normal' else 2):
                            label=f'{bag}-{profile}-{mode}-r{repeat}'
                            target=args.output/(label+'.jsonl')
                            trace=str(args.output/(label+'.trace.jsonl')) if args.trace and mode=='normal' and repeat==1 else '-'
                            command=[str(args.binary),str(args.fixtures/(bag+'-normal.bag')),profile,str(target),trace,mode,
                                *map(str,settings['injection_window_sec'])]
                            print('START '+label,flush=True)
                            with (args.output/(label+'.log')).open('w') as trial:
                                rc=subprocess.run(command,stdout=trial,stderr=subprocess.STDOUT).returncode
                            index.append(dict(label=label,bag=bag,profile=profile,gps_mode=mode,repeat=repeat,return_code=rc))
                            (args.output/'index.json').write_text(json.dumps(index,indent=2)+'\n')
                            if trace!='-':
                                trace_path=Path(trace)
                                if trace_path.exists():
                                    with trace_path.open("rb") as src,gzip.open(str(trace_path)+".gz","wb",compresslevel=3) as dst:
                                        import shutil
                                        shutil.copyfileobj(src,dst)
                                    with gzip.open(str(trace_path)+".gz","rb") as src:
                                        compressed_sha=stream_sha(src)
                                    with trace_path.open("rb") as src:
                                        original_sha=stream_sha(src)
                                    if original_sha is not None and compressed_sha==original_sha:trace_path.unlink()
                            print('END '+label+' '+str(rc),flush=True)
                            if rc:raise RuntimeError('preserved failed replay '+label)
        finally:
            if core:stop(core)
    return 0

if __name__=='__main__':raise SystemExit(main())
