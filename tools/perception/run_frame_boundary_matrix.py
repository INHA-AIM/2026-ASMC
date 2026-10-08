#!/usr/bin/env python3
"""Run current Main frame-boundary trials in an isolated master; retain every attempt."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from xmlrpc.client import ServerProxy
import yaml
import roslib.packages
from b1a_protocol import require_stopped_simulator
from frame_boundary_measurements import jobs
from run_b0_matrix import stop


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fixtures',type=Path,required=True)
    parser.add_argument('--source',type=Path,default=Path('/root/ws'))
    parser.add_argument('--suites',default='state,cost')
    parser.add_argument('--binary-dir',type=Path,required=True)
    parser.add_argument('--smoke',action='store_true')
    parser.add_argument('--normal-only',action='store_true',help='three profiles x two full bags x three repeats')
    parser.add_argument('--revision',required=True,help='host worktree commit; recorded alongside tool SHA')
    parser.add_argument('--host-state',type=Path,required=True,help='fresh Windows Simulator check, updated by host monitor')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    config=yaml.safe_load((root/'config/experiments/lidar_camera_frame_boundaries_v1.yaml').read_text())
    port=config['conditions']['ros_master_port']; uri='http://127.0.0.1:%d'%port
    if os.environ.get('ROS_MASTER_URI')!=uri: parser.error('isolated master required')
    def host_check():
        state=json.loads(args.host_state.read_text())
        require_stopped_simulator(state,time.time())
        return state
    initial_host_state=host_check()
    installed=roslib.packages.find_node('Main','main') or []
    expected_binary=(args.binary_dir/'main').resolve()
    if not installed or any(Path(path).resolve()!=expected_binary for path in installed):
        parser.error('resolved Main executable differs from --binary-dir')
    args.output.mkdir(parents=True,exist_ok=False)
    sources=[root/'config/experiments/lidar_camera_frame_boundaries_v1.yaml']
    sources+=list((root/'src/asmc_lidar').rglob('*'))+list((root/'src/lidar+sensorfusion/src/Main').rglob('*'))
    sources+=list((root/'tools/perception').glob('*frame_boundary*.py'))
    manifest=dict(config=config,revision=args.revision,host_state=initial_host_state,
        code_sha256={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources if p.is_file()},
        fixtures=json.loads((args.fixtures/'manifest.json').read_text()),
        binary_sha256=hashlib.sha256((args.binary_dir/'main').read_bytes()).hexdigest(),
        library_sha256=hashlib.sha256((args.binary_dir.parents[0]/'libmyHeader.so').read_bytes()).hexdigest())
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    protocol=[job for suite in args.suites.split(',') for job in jobs(config,suite)]
    if args.normal_only: protocol=[job for job in protocol if job['gps_mode']=='normal']
    if args.smoke: protocol=[dict(phase='realtime',suite='state',bag_id='shadow',gps_mode='missing',profile='common_pitch',subscribers='both',repeat=0)]
    (args.output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    rows=[]
    for job in protocol:
        label='{suite}-{bag_id}-{profile}-{gps_mode}-{subscribers}-r{repeat}'.format(**job)
        print('START '+label,flush=True); core=child=None
        with (args.output/(label+'.core.log')).open('w') as corelog,(args.output/(label+'.trial.log')).open('w') as triallog:
            try:
                host_check()
                try: ServerProxy(uri).getPid('/frame_boundary_matrix')
                except (OSError,ConnectionError): pass
                else: raise RuntimeError('refuse to reuse an existing master')
                core=subprocess.Popen(['roscore','-p',str(port)],stdout=corelog,stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+15
                while True:
                    try: ServerProxy(uri).getPid('/frame_boundary_matrix'); break
                    except (OSError,ConnectionError):
                        if core.poll() is not None or time.monotonic()>deadline: raise RuntimeError('master startup failed')
                        time.sleep(.05)
                command=[sys.executable,str(root/'tools/perception/run_frame_boundary_trial.py'),'--job',json.dumps(job),
                         '--output',str(args.output/label),'--fixtures',str(args.fixtures),'--source',str(args.source),
                         '--host-state',str(args.host_state)]
                child=subprocess.Popen(command,stdout=triallog,stderr=subprocess.STDOUT,start_new_session=True)
                while child.poll() is None:
                    host_check()
                    time.sleep(.2)
                report=args.output/label/'report.json'
                rows.append(dict(label=label,job=job,return_code=child.returncode,report_exists=report.exists(),
                                 complete=json.loads(report.read_text())['complete'] if report.exists() else False))
            except Exception as exc:
                rows.append(dict(label=label,job=job,error=str(exc),report_exists=False,complete=False))
            except KeyboardInterrupt:
                rows.append(dict(label=label,job=job,error='matrix interrupted',report_exists=False,complete=False))
                raise
            finally:
                if child: stop(child)
                if core: stop(core)
                (args.output/'index.json').write_text(json.dumps(rows,indent=2)+'\n')
        print('END '+json.dumps(rows[-1]),flush=True)
        if rows[-1].get('error'): break
    return 0 if all(r['complete'] for r in rows) else 1


if __name__=='__main__': raise SystemExit(main())
