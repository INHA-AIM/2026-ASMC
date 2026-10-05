#!/usr/bin/env python3
"""Summarize L2-A1 replay coverage and costs without treating labels as GT."""

import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import sys


def lines(path):
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def percentiles(values):
    if not values:
        return None
    ordered = sorted(values)
    def at(p):
        rank = (len(ordered) - 1) * p
        low = int(rank)
        high = min(low + 1, len(ordered) - 1)
        return ordered[low] + (ordered[high] - ordered[low]) * (rank - low)
    return {key: round(at(p), 4) for key, p in [('p50', .5), ('p95', .95), ('p99', .99)]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--snapshots', type=Path, required=True)
    parser.add_argument('--expect-measured', type=int, default=210)
    args = parser.parse_args()
    snapshots = {row['input_id']: row for row in lines(args.snapshots)}
    measured = {key for key, row in snapshots.items() if row.get('measurement')}
    if len(measured) != args.expect_measured:
        raise ValueError('frozen measured count changed')
    counts = Counter()
    states = defaultdict(Counter)
    costs = defaultdict(list)
    seen = set()
    for row in lines(args.candidate):
        key = row['input_id']
        if key not in snapshots:
            raise ValueError('unfrozen input ' + key)
        if key in seen:
            raise ValueError('duplicate input ' + key)
        seen.add(key)
        if key not in measured:
            continue
        counts['measured'] += 1
        state = snapshots[key]['controlled_camera_state']
        states[state]['frames'] += 1
        status = row['status']
        counts['status_' + status] += 1
        states[state]['status_' + status] += 1
        if status != 'valid':
            continue
        counts['raw_points'] += row['raw_points']
        counts['roi_points'] += row['roi_points']
        counts['voxel_points'] += row['voxel_points']
        for name, value in row['timing_ms'].items():
            costs[name].append(float(value))
        classes = row['patchwork_classes']
        if not classes:
            continue
        roi = row.get('patchwork_roi_to_original')
        if roi is None:
            raise ValueError('Patchwork++ lineage missing at ' + key)
        if len(classes) != row['raw_points'] or len(roi) != row['roi_points']:
            raise ValueError('Patchwork++ dimensions mismatch at ' + key)
        if len(set(roi)) != len(roi) or any(i < 0 or i >= len(classes) for i in roi):
            raise ValueError('Patchwork++ ROI index error at ' + key)
        local = Counter(classes[i] for i in roi)
        if any(cls not in (0, 1, 2) for cls in local):
            raise ValueError('unknown Patchwork++ class code at ' + key)
        counts['pp_ground_points'] += local[1]
        counts['pp_nonground_points'] += local[2]
        counts['pp_unassigned_roi_points'] += local[0]
        counts['pp_outside_roi_points'] += len(classes) - len(roi)
        states[state]['pp_unassigned_roi_points'] += local[0]
        counts['pp_' + row['patchwork_reason']] += 1
    if counts['measured'] != args.expect_measured or len(seen) != len(snapshots):
        raise ValueError('incomplete replay: seen=%d expected=%d measured=%d' %
                         (len(seen), len(snapshots), counts['measured']))
    report = {
        'meaning': 'paired classification coverage and offline processing cost; not detection accuracy',
        'counts': dict(sorted(counts.items())),
        'camera_states': {key: dict(sorted(value.items())) for key, value in sorted(states.items())},
        'timing_ms': {key: percentiles(value) for key, value in sorted(costs.items())},
    }
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
