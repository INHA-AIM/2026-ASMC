#!/usr/bin/env python3
"""Execute explicit R0/R1/R2 jobs serially; preserve failures and restore weather."""
import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time
from xmlrpc.client import ServerProxy
from run_b0_matrix import stop


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--jobs', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--baseline-root', type=Path, required=True)
    p.add_argument('--candidate-root', type=Path, required=True)
    p.add_argument('--saved-input', type=Path)
    p.add_argument('--grpc-host', default='host.docker.internal')
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    jobs = json.loads(a.jobs.read_text())
    (a.output/'jobs.json').write_text(json.dumps(jobs, indent=2))
    index, child, original, api = [], None, None, None
    def interrupt(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupt)
    root = Path(__file__).resolve().parents[2]
    if any('weather' in j for j in jobs):
        sys.path.insert(0, str(root/'tools/grpc_inha_univ/src/proto'))
        import grpc
        from morai.environment.environment_pb2_grpc import EnvironmentStub
        from morai.environment.weather_pb2 import Weather
        from morai.common.type_pb2 import Empty
        api = EnvironmentStub(grpc.insecure_channel(a.grpc_host+':7789'))
        original = api.GetWeather(Empty(), timeout=5)
        (a.output/'original-weather.json').write_text(json.dumps(dict(type=original.type, instantly=original.instantly)))
    def change_weather(requested):
        response = api.SetWeather(Weather(type=requested, instantly=True), timeout=5)
        deadline = time.monotonic()+10
        observed = api.GetWeather(Empty(), timeout=5)
        while observed.type != requested and time.monotonic() < deadline:
            time.sleep(.1)
            observed = api.GetWeather(Empty(), timeout=5)
        return dict(requested=requested, observed=observed.type,
                    status=response.status, success=response.status==1 and observed.type==requested)
    try:
        for job in jobs:
            label = job['name']
            if api:
                requested = {'sunny':0,'foggy':2}[job['weather']]
                transition = change_weather(requested)
                (a.output/(label+'-weather.json')).write_text(json.dumps(transition))
                if not transition['success']:
                    raise RuntimeError('weather change not confirmed: '+repr(transition))
                time.sleep(3)
            overlay = a.baseline_root if job['variant'] == 'R0' or job.get('baseline') else a.candidate_root
            command = ['python3', str(root/'tools/perception/compare_camera_runtime.py'),
                '--variant',job['variant'],'--seconds',str(job.get('seconds',60)),
                '--active',job.get('active','objects,lane,traffic_light'),
                '--lane-hz',str(job.get('lane_hz',10)), '--output',str(a.output/label)]
            if a.saved_input:
                command += ['--saved-input',str(a.saved_input)]
            command += ['--nms-iou',str(job.get('nms_iou',.7))]
            command += ['--confidence',str(job.get('confidence',.4))]
            if job.get('lane_runtime_config'):
                command += ['--lane-runtime-config', job['lane_runtime_config']]
            if 'lane_batch_wait_ms' in job:
                command += ['--lane-batch-wait-ms',str(job['lane_batch_wait_ms'])]
            if 'fusion_hz' in job:
                command += ['--fusion-hz',str(job['fusion_hz'])]
            if job.get('gui'):
                command += ['--gui']
                command += ['--view-models',job.get('view_models','all')]
                command += ['--visual-outputs',job.get('visual_outputs','all')]
            if job.get('qt'):
                command += ['--qt']
            if job.get('legacy_lidar_root'):
                command += ['--legacy-lidar-root', job['legacy_lidar_root']]
            if job.get('profile_lane_fit'):
                command += ['--profile-lane-fit']
            if job.get('allocation_profile'):
                command += ['--allocation-profile']
            if job.get('shared_gpu_input'):
                command += ['--shared-gpu-input']
            script = 'source '+shlex.quote(str(overlay/'devel/setup.bash'))+'\nexec '+shlex.join(command)
            print('START '+label, flush=True)
            with (a.output/(label+'.log')).open('w') as stream:
                child = subprocess.Popen(['bash','-c',script], stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
                returncode = child.wait()
            path = a.output/label/'report.json'
            report = json.loads(path.read_text()) if path.exists() else {}
            index.append(dict(job=job, returncode=returncode, report_exists=bool(report), failures=report.get('failures', ['missing report'])))
            (a.output/'index.json').write_text(json.dumps(index, indent=2))
            print('END '+label+' '+str(returncode), flush=True)
            child = None
            time.sleep(2)
    finally:
        if child:
            # The child owns roslaunch and its own Qt session. Allow its bounded
            # nested cleanup to finish before killing the benchmark process.
            stop(child,timeout=60)
        if api and original is not None:
            restored = change_weather(original.type)
            (a.output/'weather-restored.json').write_text(json.dumps(restored))
    return int(any(r['returncode'] or r['failures'] for r in index))


if __name__ == '__main__':
    raise SystemExit(main())
