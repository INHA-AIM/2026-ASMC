#!/usr/bin/env python3
"""Stationary owned-actor range captures with verified weather restoration."""
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--extra-representatives',type=Path)
    parser.add_argument('--side-only',action='store_true')
    parser.add_argument('--assets',nargs='+',default=['2014_Kia_K7','2013_Hyundai_Xcient(P520)','2016_Hyundai_Universe','Man1','Dog1','Bicycle_city','SportBike'])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--grpc-host', default='host.docker.internal')
    parser.add_argument('--weather', choices=['sunny','foggy'], nargs='+',default=['sunny','foggy'])
    parser.add_argument('--front-distances', type=float, nargs='+', default=[20,40,60],
                        help='Forward Ego-anchor distances; actor placement and cleanup remain checked')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    api = EnvironmentStub(grpc.insecure_channel(args.grpc_host+':7789'))
    original = api.GetWeather(Empty(), timeout=5)
    report = dict(original_weather=WeatherType.Name(original.type), runs=[],
                  distances_m=args.front_distances, pixel_ground_truth=False)
    def save():
        (args.output/'protocol.json').write_text(json.dumps(report, indent=2)+'\n')
    def weather(code):
        response = api.SetWeather(Weather(type=code, instantly=True), timeout=10)
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            actual = api.GetWeather(Empty(), timeout=5)
            if actual.type == code:
                time.sleep(2)
                if api.GetWeather(Empty(), timeout=5).type == code:
                    return dict(type=WeatherType.Name(code), response=str(response))
            time.sleep(.25)
        raise RuntimeError('weather transition/restore timeout')
    save()
    try:
        for label, code in [('sunny',0),('foggy',2)]:
            if label not in args.weather: continue
            entry=dict(weather=weather(code), started_at=time.time())
            report['runs'].append(entry); save()
            cmd=[sys.executable, str(ROOT/'tools/perception/capture_coco7_actors.py'),
                 '--output', str(args.output/label), '--grpc-host', args.grpc_host,
                 '--frames-per-view','5',
                 '--asset',*args.assets]
            if args.side_only:cmd+=['--side-only']
            else:cmd+=['--front-distances',*[str(d) for d in args.front_distances]]
            if args.extra_representatives:cmd+=['--extra-representatives',str(args.extra_representatives)]
            with (args.output/(label+'.log')).open('w') as log:
                run=subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
            entry.update(exit_code=run.returncode, ended_at=time.time(),
                         weather_after=WeatherType.Name(api.GetWeather(Empty(),timeout=5).type))
            save()
            if run.returncode:
                raise RuntimeError('capture failed; see preserved log')
            if entry['weather_after'] != WeatherType.Name(code):
                raise RuntimeError('weather changed during capture')
    finally:
        try:
            report['restoration']=weather(original.type)
        except Exception as exc:
            report['restoration_error']=str(exc)
            raise
        finally:
            save()


if __name__ == '__main__': main()
