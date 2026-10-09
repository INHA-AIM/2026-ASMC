#!/usr/bin/env python3
"""Compare every paired policy repeat without selecting favorable executions."""
import argparse
import hashlib
import json
from pathlib import Path

from analyze_fusion_identity import compare, compare_stable_observations


def compare_trials(root):
    if (root/'REJECTED_PROTOCOL.json').exists():
        raise ValueError('rejected experiment protocol cannot supply acceptance evidence')
    index = json.loads((root/'index.json').read_text())
    measured = [r for r in index if not r['job']['diagnostic']]
    report = dict(scope='matching and stable common LiDAR classification; not independent object recall',
                  index_sha256=hashlib.sha256((root/'index.json').read_bytes()).hexdigest(),
                  measured_trials=len(measured), failed_trials=[r['path'] for r in measured if not r['passed']],
                  bags={})
    for bag in ('primary', 'shadow'):
        rows = {policy: sorted([r for r in measured if r['job']['bag_id'] == bag and r['job']['policy'] == policy],
                               key=lambda r: r['job']['repeat']) for policy in ('unique', 'bounded_reuse')}
        for policy, trials in rows.items():
            if [r['job']['repeat'] for r in trials] != [1, 2, 3] or not all(r['complete'] for r in trials):
                raise ValueError('three complete repeats required: '+bag+'/'+policy)
        tracks = {policy: [json.loads((root/r['path']/'observations.json').read_text())['tracks']
                           for r in trials] for policy, trials in rows.items()}
        report['bags'][bag] = dict(
            trials={policy: [dict(path=r['path'], passed=r['passed'], totals=r['fusion']['totals'],
                                 processed_lidar=len(r['fusion'].get('published_processed_lidar_stamps_ns', [])),
                                 offset_p95_max_ms=r['fusion']['offset_p95_max_ms'], checks=r['checks'])
                             for r in trials] for policy, trials in rows.items()},
            stable=compare_stable_observations(tracks['unique'], tracks['bounded_reuse']),
            paired=[compare(a,b) for a,b in zip(tracks['unique'], tracks['bounded_reuse'])])
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=compare_trials(args.directory)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    for bag,row in report['bags'].items():
        print(bag, 'matched', {p:[t['totals']['matched'] for t in ts] for p,ts in row['trials'].items()},
              'stable',row['stable']['old_stable'],'classification differences',len(row['stable']['regressions']))
    print('failed measured trials:', report['failed_trials'])


if __name__=='__main__':main()
