#!/usr/bin/env python3
"""Check L2-A1 snapshot parity; this does not score detection accuracy."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import sys


def rows(path):
    with path.open() as handle:
        for line in handle:
            if line.strip():
                yield json.loads(line)


def fnv_indices(indices):
    digest = 14695981039346656037
    for item in indices:
        for byte in struct.pack('<i', item):
            digest = (digest ^ byte) * 1099511628211 & ((1 << 64) - 1)
    return f'{digest:016x}'


def transform(point, leveling):
    x, y, z = point
    return [sum(leveling[3 * k + i] * point[k] for k in range(3)) + offset
            for i, offset in enumerate((0.8, 0, 1.45))]


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--snapshots', type=Path, required=True)
    parser.add_argument('--expect-measured', type=int, default=210)
    args = parser.parse_args()
    fixed = {r['input_id']: r for r in rows(args.baseline) if r.get('measurement')}
    snapshots = {r['input_id']: r for r in rows(args.snapshots) if r.get('measurement')}
    if len(fixed) != args.expect_measured or len(snapshots) != args.expect_measured:
        raise ValueError('fixed measured input count changed')
    summary = {'measured': 0, 'valid': 0, 'voxel_parity': 0, 'lineage_parity': 0,
               'label_parity': 0, 'obb_count_parity': 0, 'obb_geometry_parity': 0,
               'max_center_error_m': 0.0, 'max_corner_error_m': 0.0,
               'failures': []}
    observed = set()
    for row in rows(args.candidate):
        key = row['input_id']
        if key not in fixed:
            continue
        if key in observed:
            raise ValueError(f'duplicate measured row {key}')
        observed.add(key)
        target = fixed[key]
        source = snapshots[key]
        summary['measured'] += 1
        summary['valid'] += row['status'] == 'valid'
        voxel = row['voxel_points'] == target['voxel_points']
        if row.get('frozen_voxel_fnv1a64'):
            voxel &= row['frozen_voxel_fnv1a64'] == target['frozen_voxel_fnv1a64']
        summary['voxel_parity'] += voxel
        lineage = fnv_indices(row['raw_to_voxel']) == target['raw_to_voxel_fnv1a64']
        summary['lineage_parity'] += lineage
        labels = row['reference_labels'] == target['baseline']['labels']
        summary['label_parity'] += labels
        old = [obj for obj in target['baseline']['objects'] if not obj['removed']]
        new = row['reference_objects']
        count = len(old) == len(new)
        summary['obb_count_parity'] += count
        same_geometry = count
        for original, current in zip(old, new):
            center = transform(original['center'], target['baseline']['leveling'])
            error = distance(center, current['center_base'])
            summary['max_center_error_m'] = max(summary['max_center_error_m'], error)
            same_geometry &= error <= 1e-4
            for corner, replayed in zip(original['corners'], current['corners_base']):
                error = distance(transform(corner, target['baseline']['leveling']), replayed)
                summary['max_corner_error_m'] = max(summary['max_corner_error_m'], error)
                same_geometry &= error <= 1e-4
        summary['obb_geometry_parity'] += same_geometry
        camera = 'missing' if source['controlled_camera_state'] == 'missing' else 'valid_empty_or_positive_supplied'
        ok = (row['status'] == 'valid' and voxel and lineage and labels and count and same_geometry
              and row['stamp_ns'] == source['scan_effective_stamp_ns'] and row['camera_state'] == camera)
        if not ok and len(summary['failures']) < 20:
            summary['failures'].append({'input_id': key, 'voxel': voxel, 'lineage': lineage,
                                        'labels': labels, 'obb_count': count, 'obb_geometry': bool(same_geometry),
                                        'status': row['status'], 'camera': row['camera_state']})
    print(json.dumps(summary, indent=2, sort_keys=True))
    required = ['measured', 'valid', 'voxel_parity', 'lineage_parity', 'label_parity',
                'obb_count_parity', 'obb_geometry_parity']
    if any(summary[k] != args.expect_measured for k in required):
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
