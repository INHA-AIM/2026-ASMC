#!/usr/bin/env python3
"""Keep every COCO trial and distinguish protocol completion from adoption."""
import argparse
from collections import defaultdict
import json
from pathlib import Path

try:
    from .analyze_fusion_identity import compare_stable_observations, compare
except ImportError:
    from analyze_fusion_identity import compare_stable_observations, compare


def key(job):
    return (job['phase'], job['bag_id'], job['classes'], job['repeat'], bool(job['debug']))


def analyze(root):
    protocol = json.loads((root/'protocol.json').read_text())
    index = json.loads((root/'index.json').read_text())
    expected, observed = [key(j) for j in protocol], [key(r['job']) for r in index]
    groups, rows = defaultdict(list), []
    for entry in index:
        path = root/entry['label']/'report.json'
        report = json.loads(path.read_text()) if path.exists() else None
        row = dict(entry, report=report)
        rows.append(row)
        j = entry['job']
        groups[j['phase'], j['bag_id'], j['classes'], bool(j['debug'])].append(row)
    comparisons = {}
    for bag in ('primary', 'shadow'):
        old = groups.get(('fusion', bag, 'baseline', False), [])
        new = groups.get(('fusion', bag, 'all', False), [])
        if len(old) != 2 or len(new) != 2 or any(not r['report'] for r in old+new):
            comparisons[bag] = dict(complete=False, reason='missing_two_repeats_per_profile')
            continue
        old_tracks = [json.loads((root/r['label']/'observations.json').read_text())['tracks'] for r in old]
        new_tracks = [json.loads((root/r['label']/'observations.json').read_text())['tracks'] for r in new]
        old_counts = [(r['report'].get('fusion', {}).get('totals') or {}).get('matched') for r in old]
        new_counts = [(r['report'].get('fusion', {}).get('totals') or {}).get('matched') for r in new]
        try:
            identity = compare_stable_observations(old_tracks, new_tracks)
            pairwise = [compare(old_tracks[0], tracks) for tracks in old_tracks[1:]+new_tracks]
        except ValueError as exc:
            identity, pairwise = dict(error=str(exc)), []
        comparisons[bag] = dict(complete=True, baseline_matched=old_counts, all_matched=new_counts,
            # Match the existing repeated-run gate; retain the stronger comparison as well.
            matched_not_degraded=all(c is not None for c in old_counts+new_counts) and min(new_counts) >= min(old_counts),
            all_runs_at_least_best_baseline=all(c is not None for c in old_counts+new_counts) and min(new_counts) >= max(old_counts),
            stable_observations=identity, pairwise=pairwise,
            classification_gate_pass=bool(identity.get('old_stable')) and not identity.get('regressions') and not identity.get('error'),
            prior_classification_exceptions_automatically_applied=False,
            object_recall_or_fragmentation_gt_available=False)
    complete = (len(set(expected)) == len(expected) and len(set(observed)) == len(observed)
                and set(expected) == set(observed) and all(r['report'] for r in rows))
    return dict(schema_version=1, protocol_complete=complete,
        protocol_expected_trials=len(expected), protocol_completed_trials=len(rows),
        performance_passed_trials=sum(bool(r['report'] and r['report']['passed']) for r in rows),
        performance_failed_trials=[r['label'] for r in rows if not r['report'] or not r['report']['passed']],
        operational_defaults_changed=False, all_classes_default_promoted=False,
        fusion_comparisons=comparisons, trials=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = analyze(args.root)
    (args.output or args.root/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: result[k] for k in ('protocol_complete', 'protocol_expected_trials',
        'protocol_completed_trials', 'performance_failed_trials')}))
    return 0 if result['protocol_complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
