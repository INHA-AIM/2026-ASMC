#!/usr/bin/env python3
"""Compare the two frozen finalists; retain failures and distinguish GT from inspection."""
import argparse
import hashlib
import json
from pathlib import Path
from statistics import mean

from analyze_coco7_trials import temporal_report
from analyze_coco7_weather import load_jsonl, stratify

FINALISTS = [('yolo26m', 1280), ('yolo26l-seg', 960)]


def live_summary(reports, expected_repeats=3):
    """Incomplete matrices fail closed; planning failures are never discarded."""
    results = []
    for weather in ('sunny', 'foggy'):
        for model, width in FINALISTS:
            rows = [r for r in reports if r['job']['model'] == model and
                    r['job']['width'] == width and r['job']['weather_requested'] == weather]
            repeats = [r['job']['repeat'] for r in rows]
            if len(set(repeats)) != len(repeats):
                raise ValueError('duplicate finalist repeat')
            complete = (set(repeats) == set(range(1, expected_repeats+1)) and
                        all(r.get('observation_window_completed') for r in rows))
            measurements = [r['measurements']['yolo_'+v] for r in rows
                            if r.get('observation_window_completed') for v in ('front', 'left', 'right')]
            entry = dict(weather=weather, model=model, width=width, complete=complete,
                         trials=len(rows), failures=[dict(repeat=r['job']['repeat'], reasons=r['failures'])
                                                   for r in rows if r['failures']],
                         whole_stack_passed=complete and all(r['passed'] for r in rows),
                         camera_supply_passed=complete and all(all(
                             x.startswith('planning ') for x in r['failures']) for r in rows),
                         target_15hz_passed=complete and all(all(t['met'] for t in r.get(
                             'target_attainment', {}).values()) and len(r.get('target_attainment', {})) == 3 for r in rows))
            if measurements:
                entry.update(hz_min=min(m['hz'] for m in measurements), hz_max=max(m['hz'] for m in measurements),
                    wall_p95_ms_max=max(m['input_to_output_wall_ms']['p95'] for m in measurements),
                    wall_p99_ms_max=max(m['input_to_output_wall_ms']['p99'] for m in measurements),
                    maximum_gap_ms=max(m['max_gap_sec']*1000 for m in measurements),
                    peak_whole_vram_mib=max(r['resources_startup_and_active']['gpu_memory_mib']['maximum'] for r in rows
                                            if r.get('observation_window_completed')))
            results.append(entry)
    return results


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--live', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        p.error('refusing to overwrite prior evidence')
    manifest = args.root/'gt-v2/frames.jsonl'
    gt = [r for r in load_jsonl(manifest) if r['split'] == 'selection' and r['annotation_complete']]
    result = dict(scope='Model comparison first at equal confidence; selection-only threshold tradeoff afterwards. '
                       'Capture verified box GT uses class-aware one-to-one IoU >=0.5. Missing GT classes are not zero recall.',
                  gt_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(), frames=len(gt), quality=[], live=[])
    evidence = [manifest]
    for model, width in FINALISTS:
        path = args.root/'quality'/f'{model}-{width}-selection'
        source = json.loads((path/'summary.json').read_text())
        predictions = load_jsonl(path/'predictions.jsonl')
        entry = dict(model=model, width=width, engine_sha256=source['model']['weights_sha256'],
                     missing_gt_classes=source['missing_gt_classes'], threshold_tradeoff=[])
        for confidence in (.25, .3, .35, .4, .45, .5, .55, .6, .65, .7):
            pooled = next(t for t in source['thresholds'] if t['confidence'] == confidence)
            weather = stratify(gt, predictions, confidence)
            temporal = temporal_report({r['image']: r for r in gt}, predictions, confidence)
            stable = [r for r in temporal['objects'] if r['stability_evaluable'] and r['first_stable_frame'] is not None]
            entry['threshold_tradeoff'].append(dict(confidence=confidence, pooled=pooled, weather=weather,
                temporal=dict(evaluable=sum(r['stability_evaluable'] for r in temporal['objects']),
                              acquired=len(stable), not_stably_detected=temporal['not_stably_detected'],
                              insufficient_contiguous_gt=temporal['insufficient_contiguous_gt'],
                              mean_stable_input_delay=mean(r['stable_input_delay'] for r in stable) if stable else None),
                objects=temporal['objects']))
        result['quality'].append(entry)
        evidence.extend([path/'summary.json', path/'predictions.jsonl'])
    reports = []
    for weather in ('sunny', 'foggy'):
        index = args.live/weather/'index.json'
        evidence.append(index)
        for row in json.loads(index.read_text()):
            path = args.live/weather/row['name']/'report.json'
            reports.append(json.loads(path.read_text()))
            evidence.append(path)
    result['live'] = live_summary(reports)
    result['evidence'] = [dict(path=str(p.relative_to(args.root)),
                               sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in evidence]
    args.output.write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
