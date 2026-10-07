#!/usr/bin/env python3
"""Offline B0 source coverage and logs; never changes the frozen trial gates."""
import argparse
import ast
import json
from pathlib import Path
import re

try:
    from .analyze_fusion_identity import compare
    from .b0_measurements import quantiles
except ImportError:
    from analyze_fusion_identity import compare
    from b0_measurements import quantiles


def native_semantic_log(text):
    pattern = r"semantic fps=[\d.]+ output=(\d+) batches=(\d+) replaced=(\{[^\n]+?\}) decode_errors=(\d+) inference_errors=(\d+) avg_ms=(\{[^\n]+?\})"
    rows = re.findall(pattern, text)
    if not rows:
        return None
    count, batches, replaced, decode, inference, stages = rows[-1]
    return dict(output_frames_at_last_periodic_log=int(count), interval_batches=int(batches),
                replaced_by_view_at_last_periodic_log=ast.literal_eval(replaced),
                decode_errors=int(decode), inference_errors=int(inference),
                average_stage_ms_at_last_periodic_log=ast.literal_eval(stages),
                final_shutdown_counter=False)


def input_coverage(observation, report):
    result = {}
    for name, measured in report.get('measurements', {}).items():
        source = dict(yolo='front', resnet18='extra').get(name, name)
        if source not in ('front', 'left', 'right', 'extra'):
            continue
        rows = observation['outputs'].get(name, [])
        if report['phase'] == 'camera':
            request = next(r for r in observation['requests'] if r['mode'] == 2 and
                           r['model'] == dict(front='twinlite', left='twinlite', right='twinlite').get(name, name))
            start = request['call_started']
            # This input coverage is diagnostic and uses the conservative RPC
            # start even when a trace trial's rate gate uses server acceptance.
        else:
            start = min(r['arrival'] for key in ('front', 'lidar') for r in observation['inputs'][key])
        end = measured.get('window_end_monotonic', start+measured.get('duration_sec', 0))
        supplied = [r for r in observation['inputs'][source] if start <= r['arrival'] < end]
        returned = [r for r in rows if start <= r['arrival'] < end]
        supplied_stamps = {r['stamp_ns'] for r in supplied}
        processed_stamps = {r['stamp_ns'] for r in returned}
        arrivals = [r['arrival'] for r in supplied]
        result[name] = dict(input_frames=len(supplied), input_unique_stamps=len(supplied_stamps),
                            output_frames=len(returned), output_unique_stamps=len(processed_stamps),
                            unprocessed_source_stamps=sorted(supplied_stamps-processed_stamps),
                            input_max_interarrival_sec=max((b-a for a, b in zip(arrivals, arrivals[1:])), default=None),
                            interpretation='includes intentional target-Hz selection; not all unprocessed input is a fault')
    return result


def summarize(root):
    index = json.loads((root/'index.json').read_text())
    trials, sources = {}, {}
    for entry in index:
        path = root/entry['label']
        if not (path/'report.json').exists() or not (path/'observations.json').exists():
            trials[entry['label']] = dict(incomplete_evidence=True)
            continue
        report = json.loads((path/'report.json').read_text())
        observation = json.loads((path/'observations.json').read_text())
        if entry['job']['phase'] == 'fusion' and not entry['diagnostic']:
            sources[entry['label']] = observation['tracks']
        prepared = {}
        for request in observation['requests']:
            if request['mode'] != 1:
                continue
            ready = next((r for r in observation['statuses'] if r['model'] == request['model'] and
                          r['generation'] == request['generation'] and r['prepared'] and r['pipeline_ready']), None)
            prepared[request['model']] = ((ready['arrival']-request['call_started']) if ready else None)
        trials[entry['label']] = dict(
            source_coverage=input_coverage(observation, report),
            prepare_to_pipeline_status_sec=prepared,
            semantic_native_last_periodic_log=native_semantic_log((path/'camera.log').read_text(errors='replace')),
            lidar_wall_latency_ms=quantiles([r['wall_latency_ms'] for r in observation['tracks']]),
            qualitative_track_miss_observations=sum(g['miss_count'] > 0 for r in observation['tracks'] for g in r['geometry']),
            miss_count_is_not_independent_object_recall=True)
    pairwise = {}
    for bag in ('primary', 'shadow'):
        for repeat in (1, 2, 3):
            reference = sources.get('fusion-%s-C1-r%d' % (bag, repeat))
            if reference is None:
                continue
            for profile in ('C4', 'C5', 'C6'):
                label = 'fusion-%s-%s-r%d' % (bag, profile, repeat)
                if label in sources:
                    pairwise[label] = compare(reference, sources[label])
    return dict(trials=trials, pairwise_identity_diagnostic=pairwise,
                acceptance_source='summary.json; pairwise differences are not stable-repeat acceptance')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    path = args.root/'supplemental-analysis.json'
    path.write_text(json.dumps(summarize(args.root), indent=2)+'\n')
    print(str(path))


if __name__ == '__main__':
    main()
