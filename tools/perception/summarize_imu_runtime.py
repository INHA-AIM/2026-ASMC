#!/usr/bin/env python3
"""Reuse shadow runtime gates; do not waive source gaps or baseline failures."""
import argparse
import json
from pathlib import Path
from summarize_camera_structure import shadow_acceptance_failures
from summarize_camera_visualization import viewer_condition_verified


def main(args):
    trials = []
    for path in sorted(args.results.rglob('report.json')):
        report = json.loads(path.read_text())
        if 'variant' not in report: continue
        statuses = json.loads(path.with_name('statuses.json').read_text())
        failures = shadow_acceptance_failures(report, statuses)
        if report.get('gui') and not viewer_condition_verified(report): failures.append('Qt inventory')
        imu = report.get('imu_application', {})
        if report.get('imu_mode') == 'source':
            if not imu.get('frames') or imu.get('corrected', 0) / imu['frames'] < .99:
                failures.append('source IMU application below99%')
            if imu.get('joint_frames') != imu.get('corrected'):
                failures.append('joint policy/IMU count mismatch')
        trials.append(dict(trial=path.parent.name, path=str(path), passed=not failures, failures=failures,
            imu=imu, family=report.get('lane_family'), resources=report.get('resources'), streams=report.get('streams'),
            complete=report.get('complete'), seconds=report.get('end', 0) - report.get('start', 0),
            mode=report.get('imu_mode')))
    candidates = [r for r in trials if r['mode'] == 'source']
    summary = dict(schema='imu_joint_runtime_acceptance_v1', trials=trials,
        all_candidate_passed=bool(candidates) and all(r['passed'] for r in candidates),
        candidate_runs=len(candidates), ten_minute_pass=any(r['passed'] and r['seconds'] >= 599.99 for r in candidates))
    args.output.write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(dict(trials=len(trials), candidate_runs=len(candidates),
        all_candidate_passed=summary['all_candidate_passed'], ten_minute_pass=summary['ten_minute_pass'],
        failures={r['trial']: r['failures'] for r in trials if not r['passed']})), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--results', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    main(p.parse_args())
