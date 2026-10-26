#!/usr/bin/env python3
"""Alternate two native replay builds; compare objects, not diagnostic wording.

No ROS master, publishers or Python geometry. The two executables must use their
own matching libraries. Whole replay wall time includes bag IO/JSON serialization;
this is not a live input-to-publication latency or frame-rate measurement.
"""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import subprocess
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from audit_frame_time import ROOT, invariant, read, sha, stats, write_json


def behavior(row):
    # Selection metadata/reason priority intentionally changed. Preserve all
    # geometry, track/filter results and timestamps in the equality comparison.
    value = invariant(row)
    for key in ('input_selection', 'imu_ns', 'can_ns', 'reason'):
        value.pop(key, None)
    return value


def main(args):
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = json.loads(args.manifest.read_text())
    report = {'baseline_sha': sha(args.before), 'candidate_sha': sha(args.after),
              'baseline_libraries': {p.name: sha(p) for p in args.before_lib.glob('*.so')},
              'manifest_sha': sha(args.manifest), 'runs': [], 'passed': True,
              'limits': ['No live supply/latency claim. Wall time includes bag IO and trace writing.',
                         'Native selection reason/metadata changes excluded; object/track/filter data compared.']}
    for item in manifest['inputs']:
        if item['id'] not in ('normal', 'tunnel'):
            continue
        bag = args.storage_root / item['path']
        if sha(bag) != item['sha256']:
            raise ValueError('input hash mismatch: ' + str(bag))
        for repeat in range(3):
            outputs = {}
            for name in (('before', 'after') if repeat % 2 == 0 else ('after', 'before')):
                directory = args.output / ('%s-%s-%s' % (item['id'], repeat, name))
                directory.mkdir()
                env = os.environ.copy()
                if name == 'before':
                    env['LD_LIBRARY_PATH'] = str(args.before_lib) + ':' + env.get('LD_LIBRARY_PATH', '')
                command = ['/usr/bin/time', '-f', '{"wall_s":%e,"user_s":%U,"system_s":%S,"rss_kib":%M}',
                           '-o', str(directory / 'resource.json'), str(getattr(args, name)),
                           '--bag', str(bag), '--config', str(ROOT / 'src/asmc_lidar/config/lidar_pipeline.yaml'),
                           '--package-share', str(ROOT / 'src/asmc_lidar'), '--output', str(directory / 'trace.jsonl')]
                with (directory / 'replay.log').open('xb') as log:
                    subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
                rows = list(read(directory / 'trace.jsonl'))
                outputs[name] = [behavior(r) for r in rows]
                report['runs'].append(dict(dataset=item['id'], repeat=repeat, variant=name, scans=len(rows),
                    reasons=dict(Counter(r['reason'] for r in rows)),
                    pipeline_ms=stats(r['timings_ms']['total'] for r in rows if r.get('timings_ms')),
                    **json.loads((directory / 'resource.json').read_text())))
            equal = outputs['before'] == outputs['after']
            report['passed'] &= equal
            if not equal:
                write_json(args.output / ('difference-%s-%s.json' % (item['id'], repeat)),
                    [{'index': i, 'before': a, 'after': b} for i, (a, b) in
                     enumerate(zip(outputs['before'], outputs['after'])) if a != b][:3])
    write_json(args.output / 'summary.json', report)
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('before', 'after', 'before-lib', 'manifest', 'storage-root', 'output'):
        p.add_argument('--' + name, required=True, type=Path)
    sys.exit(main(p.parse_args()))
