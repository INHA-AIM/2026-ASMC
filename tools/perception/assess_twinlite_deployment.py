#!/usr/bin/env python3
"""Assess deployment gates; execution success alone is not performance acceptance."""
import argparse
import json
from pathlib import Path


def assess(report, statuses):
    failures = list(report.get('failures', []))
    if not report.get('complete') or not report.get('acceptance_eligible'):
        failures.append('incomplete or diagnostic-only measurement')
    streams = report.get('streams', {})
    if not streams:
        failures.append('missing model streams')
    for name, stream in streams.items():
        minimum = 9.5 if 'traffic_light' in name else 10.
        if not stream.get('five_second_hz') or min(stream['five_second_hz']) < minimum:
            failures.append(name + ': rate')
        if not stream.get('timing_pass'):
            failures.append(name + ': latency/gap')
        if stream.get('duplicate_stamps') or stream.get('reversed_stamps'):
            failures.append(name + ': stamps')
    family = report.get('lane_family', {})
    valid = family.get('valid_supply', {})
    producer = family.get('producer_supply', {})
    if not valid.get('five_second_hz') or min(valid['five_second_hz']) < 10:
        failures.append('valid family rate')
    if max(valid.get('max_gap_sec', float('inf')),
           producer.get('max_gap_sec', float('inf'))) > .25:
        failures.append('family gap')
    if family.get('duplicate_stamps') or family.get('reversed_stamps') or family.get('unmatched_publisher_stamps'):
        failures.append('family correspondence')
    resources = report.get('resources', {})
    if not resources.get('resource_coverage_pass') or resources.get('vram_peak_fraction', 1) > .9:
        failures.append('resources')
    if any(report.get('family_error_counters', {}).values()):
        failures.append('family errors')
    active = [row for row in statuses
              if report['start'] <= row.get('at', -1) < report['end']]
    lane = [row for row in active if row.get('role') == 'lane']
    drops = sum(not row['pipeline_ready'] for row in lane)
    if not lane or drops or any(row.get('errors', 0) for row in active):
        failures.append('normal ACTIVE readiness/errors')
    if (report.get('qt_inventory_before') or report.get('qt_inventory_after')
            or len(report.get('qt_inventory_during', [])) != 1):
        failures.append('Qt inventory')
    return dict(pass_=not failures, failures=failures, normal_lane_readiness_drops=drops)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    results = []
    for item in json.loads((args.matrix/'index.json').read_text()):
        folder = args.matrix/item['job']['name']
        report = json.loads((folder/'report.json').read_text())
        statuses = [json.loads(line).get('record', {}) for line in
                    (folder/'consumer-statuses.jsonl').read_text().splitlines() if line]
        results.append(dict(name=item['job']['name'], **assess(report, statuses)))
    output = dict(schema='twinlite_deployment_acceptance_v1', trials=results,
                  all_pass=all(row['pass_'] for row in results),
                  scope='Parked shadow ODD; not closed-loop driving acceptance')
    args.output.write_text(json.dumps(output, indent=2)+'\n')
    print(json.dumps(output))
    return 0 if output['all_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
