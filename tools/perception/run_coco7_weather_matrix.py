#!/usr/bin/env python3
"""Stationary shared-container measurements; owned actors and verified weather restoration."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools/grpc_inha_univ/src/proto'))


def main():
    import grpc
    from morai.environment.environment_pb2_grpc import EnvironmentStub
    from morai.environment.weather_pb2 import Weather
    from morai.environment.environment_enum_pb2 import WeatherType
    from morai.common.type_pb2 import Empty
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--candidates', type=Path, required=True, help='Frozen model/width/confidence list')
    p.add_argument('--revision', required=True)
    p.add_argument('--models', type=Path, default=Path('/data/models/camera_coco7_v1'))
    p.add_argument('--repeats', type=int, default=3)
    p.add_argument('--seconds', type=float, default=20.)
    p.add_argument('--profile', choices=['C1', 'C6'], default='C6')
    p.add_argument('--weather', nargs='+', choices=['sunny', 'foggy'], default=['sunny', 'foggy'])
    p.add_argument('--grpc-host', default='host.docker.internal')
    a = p.parse_args()
    if a.repeats < 1 or a.seconds < 5:
        p.error('positive repeat count and complete 5-second windows required')
    candidates = json.loads(a.candidates.read_text())
    if not candidates or len({(c['model'], c['width']) for c in candidates}) != len(candidates):
        p.error('empty or duplicate candidates')
    a.output.mkdir(parents=True, exist_ok=False)
    api = EnvironmentStub(grpc.insecure_channel(a.grpc_host+':7789'))
    original = api.GetWeather(Empty(), timeout=5)
    report = dict(original_weather=WeatherType.Name(original.type), candidates=candidates,
                  runs=[], preview_control_only=True, revision=a.revision)
    registry = a.output/'owned-actors.json'

    def save():
        (a.output/'weather-protocol.json').write_text(json.dumps(report, indent=2)+'\n')

    def weather(code):
        api.SetWeather(Weather(type=code, instantly=True), timeout=10)
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            if api.GetWeather(Empty(), timeout=5).type == code:
                time.sleep(2)
                if api.GetWeather(Empty(), timeout=5).type == code:
                    return WeatherType.Name(code)
            time.sleep(.2)
        raise RuntimeError('weather transition/restore not verified')

    save()
    try:
        subprocess.run([sys.executable, str(ROOT/'tools/perception/coco7_scene.py'), 'create',
                        '--registry', str(registry), '--grpc-host', a.grpc_host], check=True)
        actors = json.loads(registry.read_text())['actors']
        if len(actors) != 9 or any(not r.get('created') or r.get('error') for r in actors):
            raise RuntimeError('incomplete owned-actor fixture')
        for label, code in [('sunny', 0), ('foggy', 2)]:
            if label not in a.weather:
                continue
            entry = dict(weather=label, observed_before=weather(code))
            report['runs'].append(entry)
            save()
            jobs = []
            for repeat in range(1, a.repeats+1):
                order = candidates if repeat % 2 else list(reversed(candidates))
                for c in order:
                    jobs.append(dict(c, rates='all15', batch='2plus1', profile=a.profile,
                                     repeat=repeat, seconds=a.seconds, full_stack=a.profile == 'C6',
                                     fusion_policy='unique', weather_requested=label))
            jf = a.output/(label+'-jobs.json')
            jf.write_text(json.dumps(jobs, indent=2)+'\n')
            run = subprocess.run([sys.executable, str(ROOT/'tools/perception/run_coco7_live_trials.py'),
                                  '--jobs', str(jf), '--models', str(a.models),
                                  '--output', str(a.output/label), '--revision', a.revision])
            entry.update(exit_code=run.returncode, observed_after=WeatherType.Name(api.GetWeather(Empty(), timeout=5).type))
            save()
            index = json.loads((a.output/label/'index.json').read_text())
            if len(index) != len(jobs) or any(not json.loads((a.output/label/r['name']/'report.json').read_text()).get(
                    'observation_window_completed') for r in index):
                raise RuntimeError('incomplete measurement matrix; evidence retained')
            if entry['observed_after'] != WeatherType.Name(code):
                raise RuntimeError('weather changed during measurements')
            # Exit 1 records failed acceptance gates; it is not an execution abort.
    finally:
        errors = []
        try:
            if registry.exists():
                subprocess.run([sys.executable, str(ROOT/'tools/perception/coco7_scene.py'), 'cleanup',
                                '--registry', str(registry), '--grpc-host', a.grpc_host], check=True)
                data = json.loads(registry.read_text())
                owned = {r['id'] for r in data['actors'] if r.get('created')}
                cleaned = {r['id'] for r in data.get('cleanup', []) if r['response'].get('status') == 'STATUS_CODE_SUCCESS'}
                if owned != cleaned:
                    errors.append('owned actor cleanup incomplete')
        except Exception as exc:
            errors.append(str(exc))
        try:
            report['restored_weather'] = weather(original.type)
        except Exception as exc:
            errors.append(str(exc))
        report['restoration_errors'] = errors
        save()
        if errors:
            raise RuntimeError(str(errors))


if __name__ == '__main__':
    main()
