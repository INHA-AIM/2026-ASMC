#!/usr/bin/env python3
"""Compare numerical replay records; print JSON to stdout (no output files).

Only top-level producer IDs, configuration identity and measured timing differ
between builds. Nested geometry, source stamps, KF state, classes, counts,
cloud hashes, validity and rejection reasons must match exactly.
"""
import argparse
import json
from pathlib import Path

TELEMETRY = {'producer', 'producer_session_id', 'generation', 'config_id',
             'calibration_id', 'timings_ms'}


def numerical(record):
    return {key: value for key, value in record.items() if key not in TELEMETRY}


def compare(before, after):
    old, new = before.open(), after.open()
    count = 0
    try:
        while True:
            a, b = old.readline(), new.readline()
            if not a or not b:
                return {'equal': a == b, 'scans': count,
                        'first_difference': None if a == b else count}
            if numerical(json.loads(a)) != numerical(json.loads(b)):
                return {'equal': False, 'scans': count, 'first_difference': count}
            count += 1
    finally:
        old.close()
        new.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    args = parser.parse_args()
    if args.baseline.is_dir() and args.candidate.is_dir():
        before = {p.name: p for p in args.baseline.glob('*.jsonl')}
        after = {p.name: p for p in args.candidate.glob('*.jsonl')}
        if not before or before.keys() != after.keys():
            parser.error('non-empty directories with the same JSONL names are required')
        results = {name: compare(before[name], after[name]) for name in sorted(before)}
    else:
        results = {args.baseline.name: compare(args.baseline, args.candidate)}
    print(json.dumps(results, indent=2))
    return 0 if all(row['equal'] for row in results.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
