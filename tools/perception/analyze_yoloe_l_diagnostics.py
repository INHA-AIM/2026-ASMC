#!/usr/bin/env python3
"""Account for lifecycle admission and fusion separately from trace-OFF rates."""
import argparse
import json
from pathlib import Path

import numpy as np

from analyze_fusion_identity import compare
from analyze_fusion_trace import summarize


def quantiles(values):
    return {k: float(np.percentile(values, q)) if values else None
            for k, q in [('p50', 50), ('p95', 95), ('p99', 99)]}


def camera_admission(report, rows):
    start = report['common_active_started']
    active = [r for r in rows if start <= r['monotonic'] < start + 20]
    result = {}
    for view in ['front', 'left', 'right']:
        events = {}
        for event in ['input', 'begin', 'complete', 'published', 'rate_wait']:
            selected = [r for r in active if r['event'] == event and
                        (r.get('view') == view or view in r.get('stamps', {}))]
            events[event] = len(selected)
        published = [r for r in active if r['event'] == 'published' and view in r['stamps']]
        stages = sorted({key for r in published for key in r.get('timings', {}) if key.endswith('_ms')})
        result[view] = dict(events=events, timings_ms={key: quantiles(
            [r['timings'][key] for r in published if key in r['timings']]) for key in stages})
    return dict(window='server ACTIVE acceptance to +20s; no time-origin shift', views=result,
                trace_complete=sum(r['event'] == 'trace_end' for r in rows) == 1,
                counter_scope='buffer replacement counters in last_diagnostics include STANDBY/PREPARE too',
                batch_timing_scope='per-result shared batch time; do not sum across same-batch views',
                last_diagnostics=report.get('last_diagnostics'))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--results', type=Path, required=True)
    a = p.parse_args(); root = a.results
    camera = root/'diagnostic-trials-v1/camera-camera-rich42-2plus1-C1-r1'
    fusion = root/'diagnostic-trials-v1/fusion-primary-rich42-2plus1-C6-r1'
    load_rows = lambda path: [json.loads(line) for line in path.read_text().splitlines()]
    camera_report = json.loads((camera/'report.json').read_text())
    fusion_rows = load_rows(fusion/'fusion.jsonl')
    accounted = summarize(fusion_rows)
    matches = [r['offset_ns']/1e6 for r in fusion_rows if r['event'] == 'match_decision' and r['action'] == 'match']
    ends = [r for r in fusion_rows if r['event'] == 'lidar_process_end']
    accounted['processing_ms_interpolated_quantiles'] = quantiles([r['duration_ms'] for r in ends])
    accounted['matched_offset_ms_pooled_trace_only'] = quantiles(matches)
    accounted['drop_wait_ns'] = [r.get('waited_ns') for r in fusion_rows
                                if r['event'] == 'match_decision' and r['action'] == 'drop']
    repeat_comparisons = {}
    for bag in ['primary', 'shadow']:
        observations = [json.loads((root/'followup-trials-v1'/
            ('fusion-'+bag+'-rich42-2plus1-C6-r'+str(i))/'observations.json').read_text())['tracks']
            for i in [1, 2, 3]]
        repeat_comparisons[bag] = {str(i): compare(observations[0], observations[i-1]) for i in [2, 3]}
    result = dict(camera=camera_admission(camera_report, load_rows(camera/'yolo.lifecycle.jsonl')),
                  fusion_trace=accounted, fusion_repeat_comparisons=repeat_comparisons,
                  acceptance_scope='trace ON diagnostic only; does not replace normal OFF runs or prove old/new fusion parity',
                  prior_classification_exceptions_applied=False)
    (root/'diagnostic-analysis.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(camera=result['camera']['views'],
        fusion={k:v for k,v in accounted.items() if k not in ['processed_lidar_stamps','events']},
        repeat_comparison={bag:{key:{k:len(v) if isinstance(v,list) else v for k,v in value.items()}
                               for key,value in runs.items()} for bag,runs in repeat_comparisons.items()})))


if __name__ == '__main__':
    main()
