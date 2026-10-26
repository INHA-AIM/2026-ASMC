#!/usr/bin/env python3
"""Run a fixed-setting natural-signal survey; retain every failed trial.

Requires the existing bridge/perception stack. Each child owns its planner,
recorder and generated actors. Cleanup failure stops the batch.
"""
import argparse
import json
import signal
import subprocess
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--physical-road', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[42, 43, 44, 45, 46])
    parser.add_argument('--deadline', type=float, default=650)
    parser.add_argument('--park-static', action='append', default=[])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    (args.output / 'survey-config.json').write_text(json.dumps(vars(args), default=str, indent=2))
    runner = Path(__file__).with_name('run_yellow_signal_trial.py')
    for index, seed in enumerate(args.seeds, 1):
        name = 'run-{:02d}-seed{}'.format(index, seed)
        trial = args.output / name
        command = [sys.executable, str(runner), '--production-stack', '--host', args.host,
                   '--physical-road', str(args.physical_road.resolve()), '--output', str(trial.resolve()),
                   '--case', 'corridor', '--full-route', '--natural-signals', '--scenario-seed', str(seed),
                   '--record-bag', '--deadline', str(args.deadline)]
        for fixture in args.park_static:
            command += ['--park-static', fixture]
        print(json.dumps(dict(event='starting', run=index, seed=seed, output=str(trial))), flush=True)
        with (args.output / (name + '.log')).open('w') as log:
            child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = child.wait()
            except KeyboardInterrupt:
                child.send_signal(signal.SIGINT)
                child.wait()  # Child's finally holds Ego and closes the recording.
                raise
        path = trial / 'result.json'
        if not path.exists():
            raise RuntimeError('trial has no cleanup/result record; batch stopped: ' + name)
        result = json.loads(path.read_text())
        row = dict(run=index, seed=seed, returncode=code, completed=result.get('completed', False),
                   passed=result.get('passed', False), error=result.get('error'),
                   samples_recorded=bool(result.get('recording')))
        results.append(row)
        (args.output / 'survey-results.json').write_text(json.dumps(results, indent=2))
        print(json.dumps(dict(event='finished', **row)), flush=True)
        errors = [key for key in result if key.endswith('error') and key != 'error']
        if errors or result.get('source_changed_during_trial') or not row['samples_recorded']:
            raise RuntimeError('batch stopped after incomplete evidence/cleanup: ' + str(errors))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
