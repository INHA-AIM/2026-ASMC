#!/usr/bin/env python3
"""Measure independent simulator speed, including every recorded stop.

A partial drive is never labelled a full-lap average. This report does not
replace the separate solid-paint/wheel audit or certify road safety.
"""
import argparse
import json
import math
from pathlib import Path


def measure(rows, completed=False):
    if len(rows) < 2:
        raise ValueError('at least two independent observations are required')
    elapsed = distance = stopped = 0.0
    maximum_gap = 0.0
    bins = {}
    for row in rows:
        if not all(math.isfinite(row[k]) for k in ('monotonic', 'speed_kmh', 'station')):
            raise ValueError('non-finite observation')
        if row['speed_kmh'] < 0:
            raise ValueError('speed magnitude must be nonnegative')
    for a, b in zip(rows, rows[1:]):
        dt = b['monotonic'] - a['monotonic']
        if dt <= 0:
            raise ValueError('observations must be strictly ordered')
        ds = (a['speed_kmh'] + b['speed_kmh']) / 7.2 * dt
        elapsed += dt
        distance += ds
        maximum_gap = max(maximum_gap, dt)
        if max(a['speed_kmh'], b['speed_kmh']) < 0.36:
            stopped += dt
        key = int((a['station'] + b['station']) / 200) * 100
        bucket = bins.setdefault(key, dict(elapsed_s=0.0, distance_m=0.0))
        bucket['elapsed_s'] += dt
        bucket['distance_m'] += ds
    for bucket in bins.values():
        bucket['mean_kmh'] = bucket['distance_m'] / bucket['elapsed_s'] * 3.6
    observed = maximum_gap <= 0.3 and all(r['observation_complete'] for r in rows)
    mean = distance / elapsed * 3.6
    maximum = max(r['speed_kmh'] for r in rows)
    return dict(
        scope='full_lap_observed_window' if completed else 'partial_drive',
        samples=len(rows), elapsed_s=elapsed, distance_m=distance,
        mean_including_stops_kmh=mean, full_lap_mean_kmh=mean if completed and observed else None,
        maximum_kmh=maximum, stopped_below_036_kmh_s=stopped,
        maximum_observation_gap_s=maximum_gap, observation_complete=observed,
        observed_within_59_kmh_ceiling=bool(observed and maximum <= 59),
        original_50_kmh_goal_met=bool(completed and observed and maximum <= 59 and mean >= 50),
        station_bins_100m=bins,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    args = parser.parse_args()
    result = json.loads((args.trial / 'result.json').read_text())
    rows = [json.loads(line) for line in (args.trial / 'oracle.jsonl').open() if line.strip()]
    report = measure(rows, result.get('completed', False))
    report['trial_passed'] = result.get('passed', False)
    report['independent_evaluation'] = result.get('independent_evaluation')
    report['lane_paint_audit_required'] = True
    path = args.trial / 'speed-summary.json'
    path.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items()
                      if k not in ('station_bins_100m', 'independent_evaluation')}, indent=2))


if __name__ == '__main__':
    main()
