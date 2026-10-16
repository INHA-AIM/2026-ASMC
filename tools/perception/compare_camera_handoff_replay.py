#!/usr/bin/env python3
"""Compare every field of two logical offline Main replay outputs."""
import argparse
import json
import math
from pathlib import Path


def differences(reference, candidate, path=''):
    if type(reference) != type(candidate):
        return [path + ':type']
    if isinstance(reference, dict):
        if reference.keys() != candidate.keys():
            return [path + ':keys']
        return [error for key in reference for error in differences(reference[key], candidate[key], path+'/'+key)]
    if isinstance(reference, list):
        if len(reference) != len(candidate):
            return [path + ':length']
        return [error for index, (a, b) in enumerate(zip(reference, candidate))
                for error in differences(a, b, path+'/'+str(index))]
    if isinstance(reference, float):
        equal = ((math.isnan(reference) and math.isnan(candidate))
                 or math.isclose(reference, candidate, rel_tol=0, abs_tol=1e-6))
        return [] if equal else [path + ':value']
    return [] if reference == candidate else [path + ':value']


def summarize(reference, candidate):
    errors = differences(reference, candidate)
    return dict(passed=bool(reference) and not errors, frames=len(candidate),
                ready=sum(row['ready'] for row in candidate),
                objects=sum(len(row['objects']) for row in candidate),
                differences=errors, tolerance=1e-6,
                scope='logical offline output parity, not real-time or GT quality acceptance')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference', type=Path)
    parser.add_argument('candidate', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rows = [[json.loads(line) for line in path.read_text().splitlines()]
            for path in (args.reference, args.candidate)]
    report = summarize(*rows)
    text = json.dumps(report, indent=2)+'\n'
    if args.output:
        args.output.write_text(text)
    print(text, end='')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
