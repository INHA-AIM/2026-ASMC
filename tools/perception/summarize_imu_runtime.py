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
        status_path = path.with_name('statuses.json')
        statuses = json.loads(status_path.read_text()) if status_path.exists() else []
        failures = shadow_acceptance_failures(report, statuses)
        if not status_path.exists(): failures.append('missing status journal')
        if report.get('gui') and not viewer_condition_verified(report): failures.append('Qt inventory')
        imu = report.get('imu_application', {})
        transitions = None
        trace_path = path.with_name('camera_lane_vectorizer-supply.trace')
        if trace_path.exists() and report.get('start') is not None and report.get('end') is not None:
            from imu_runtime_metrics import imu_application
            trace = [json.loads(line) for line in trace_path.read_text().splitlines()]
            imu = imu_application(trace, report['start'], report['end'], report.get('imu_mode') == 'source')
            if imu['unmatched']: failures.append('IMU/fit publication trace mismatch')
            family_path = path.with_name('consumer-families.jsonl')
            if family_path.exists():
                from imu_runtime_metrics import fallback_transitions
                families = [r['record'] for r in map(json.loads, family_path.read_text().splitlines())
                            if r['event'] == 'consumer_record']
                transitions = fallback_transitions(trace, families, report['start'], report['end'])
        if report.get('imu_mode') == 'source':
            if not imu.get('frames') or imu.get('corrected', 0) / imu['frames'] < .99:
                failures.append('source IMU application below99%')
            if imu.get('joint_frames') != imu.get('corrected'):
                failures.append('joint policy/IMU count mismatch')
        start, end = report.get('start'), report.get('end')
        seconds = end - start if start is not None and end is not None else 0.
        from imu_runtime_metrics import normalize_supply_targets
        trials.append(dict(trial=path.parent.name, path=str(path), passed=not failures, failures=failures,
            imu=imu, family=report.get('lane_family'), resources=report.get('resources'),
            streams=normalize_supply_targets(report.get('streams', {}), report.get('lane_hz', 10.)),
            complete=report.get('complete'), seconds=seconds,
            mode=report.get('imu_mode'), effective_imu_config=report.get('effective_imu_config'),
            fallback_transitions=transitions))
    candidates = [r for r in trials if r['mode'] == 'source' and args.candidate_contains in r['trial']]
    summary = dict(schema='imu_joint_runtime_acceptance_v1', trials=trials,
        all_candidate_passed=bool(candidates) and all(r['passed'] for r in candidates),
        candidate_runs=len(candidates), candidate_name_filter=args.candidate_contains,
        interrupted_conditions=[dict(path=str(p), **json.loads(p.read_text()))
                                for p in sorted(args.results.rglob('fixture-condition-failure.json'))],
        ten_minute_pass=any(r['passed'] and r['seconds'] >= 599.99 for r in candidates))
    args.output.write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(dict(trials=len(trials), candidate_runs=len(candidates),
        all_candidate_passed=summary['all_candidate_passed'], ten_minute_pass=summary['ten_minute_pass'],
        failures={r['trial']: r['failures'] for r in trials if not r['passed']})), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--results', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--candidate-contains', default='', help='Select final revision by recorded run name; all earlier trials remain in evidence')
    main(p.parse_args())
