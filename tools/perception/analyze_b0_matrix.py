#!/usr/bin/env python3
"""Preserve every B0 repeat; separate protocol completion from performance gates."""
import argparse
from collections import defaultdict
import json
from pathlib import Path

try:
    from .analyze_fusion_identity import compare_stable_observations
except ImportError:
    from analyze_fusion_identity import compare_stable_observations


def analyze(root):
    index = json.loads((root/'index.json').read_text())
    groups, rows = defaultdict(list), []
    for entry in index:
        path = root/entry['label']/'report.json'
        row = dict(entry, report=json.loads(path.read_text()) if path.exists() else None)
        rows.append(row)
        j = entry['job']
        groups[j['phase'], j['bag_id'], j['profile'], bool(j.get('debug')), bool(entry['diagnostic']), bool(j.get('gps_gap'))].append(row)
    comparisons = {}
    for bag_id in ('primary', 'shadow'):
        baseline = groups.get(('fusion', bag_id, 'C1', False, False, False), [])
        if len(baseline) != 3:
            continue
        old = [json.loads((root/r['label']/'observations.json').read_text())['tracks'] for r in baseline]
        ref_counts = [(r['report'].get('fusion', {}).get('totals') or {}).get('matched') for r in baseline]
        for profile in ('C1', 'C4', 'C5', 'C6'):
            candidates = groups.get(('fusion', bag_id, profile, False, False, False), [])
            if len(candidates) != 3:
                continue
            new = [json.loads((root/r['label']/'observations.json').read_text())['tracks'] for r in candidates]
            counts = [(r['report'].get('fusion', {}).get('totals') or {}).get('matched') for r in candidates]
            try:
                identity = compare_stable_observations(old, new)
            except ValueError as exc:
                identity = dict(error=str(exc))
            comparisons[bag_id+'-'+profile] = dict(reference_matched=ref_counts, candidate_matched=counts,
                matched_not_degraded=all(v is not None for v in ref_counts+counts) and min(counts) >= min(ref_counts),
                stable_observations=identity,
                classification_gate_pass=bool(identity.get('old_stable')) and not identity.get('regressions') and not identity.get('error'),
                previously_approved_two_observations_automatically_waived=False,
                independent_object_recall_or_fragmentation_gt=False)
    protocol = json.loads((root/'protocol.json').read_text())
    regular = [r for r in rows if not r['diagnostic']]
    return dict(schema_version=1, protocol_expected_trials=len(protocol),
        protocol_completed_trials=len(regular), diagnostic_trials=len(rows)-len(regular),
        protocol_complete=len(regular) == len(protocol) and all(r['report'] is not None for r in regular),
        performance_passed_trials=sum(bool(r['report'] and r['report']['passed']) for r in regular),
        performance_failed_trials=[r['label'] for r in regular if not r['report'] or not r['report']['passed']],
        configuration_adoption='none_operational_defaults_unchanged',
        fusion_comparisons=comparisons,
        trials=[dict(label=r['label'], diagnostic=r['diagnostic'], return_code=r['return_code'],
                     report=r['report']) for r in rows])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    summary = analyze(args.root)
    path = args.output or args.root/'summary.json'
    path.write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps({k: summary[k] for k in ('protocol_complete', 'protocol_expected_trials', 'protocol_completed_trials', 'diagnostic_trials', 'performance_failed_trials')}))
    return 0 if summary['protocol_complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
