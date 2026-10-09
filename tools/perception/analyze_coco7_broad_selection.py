#!/usr/bin/env python3
"""Compare all eight models with explicit reviewed exclusions and retained failures."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

from analyze_coco7_trials import temporal_report
from analyze_coco7_weather import load_jsonl, weather_tag
from evaluate_coco7_models import COCO7, evaluate, summary

COMBINATIONS = [(m, w) for m in ('yolo26m', 'yolo26l', 'yolo26m-seg', 'yolo26l-seg')
                for w in (960, 1280)]


def confirmation_summary(reports, candidates, expected_repeats=3):
    result = []
    for candidate in candidates:
        for weather in ('sunny', 'foggy'):
            rows = [r for r in reports if all(r['job'][key] == candidate[key]
                    for key in ('model', 'width', 'confidence')) and
                    r['job']['weather_requested'] == weather]
            repeats = [r['job']['repeat'] for r in rows]
            if len(repeats) != len(set(repeats)):
                raise ValueError('duplicate confirmation repeat')
            complete = set(repeats) == set(range(1, expected_repeats+1)) and all(
                r.get('observation_window_completed') for r in rows)
            camera_complete = complete and all(set(('front', 'left', 'right', 'resnet18',
                'yolo_front', 'yolo_left', 'yolo_right')) <= set(r.get('measurements', {})) for r in rows)
            entry = dict(candidate=candidate, weather=weather, complete=complete,
                camera_supply_passed=camera_complete and all(all(f.startswith('planning ')
                    for f in r['failures']) for r in rows),
                whole_stack_passed=complete and all(r['passed'] and not r['failures'] for r in rows),
                target_15hz_passed=complete and all(set(r.get('target_attainment', {})) ==
                    {'front', 'left', 'right'} and all(t['met'] for t in r['target_attainment'].values()) for r in rows),
                failures=[dict(repeat=r['job']['repeat'], reasons=r['failures']) for r in rows if r['failures']])
            metrics = [r['measurements']['yolo_'+v] for r in rows if r.get('observation_window_completed')
                       for v in ('front', 'left', 'right') if 'yolo_'+v in r.get('measurements', {})]
            if metrics:
                entry.update(hz_min=min(m['hz'] for m in metrics), hz_max=max(m['hz'] for m in metrics),
                    wall_p95_ms_max=max(m['input_to_output_wall_ms']['p95'] for m in metrics),
                    wall_p99_ms_max=max(m['input_to_output_wall_ms']['p99'] for m in metrics),
                    maximum_gap_ms=max(m['max_gap_sec']*1000 for m in metrics),
                    duplicate_stamps=sum(m['duplicate_stamps'] for m in metrics),
                    reversed_stamps=sum(m['reversed_stamps'] for m in metrics),
                    peak_whole_vram_mib=max(r['resources_startup_and_active']['gpu_memory_mib']['maximum'] for r in rows))
            result.append(entry)
    return result


def selected_frames(rows, predictions, exclusions):
    gt = {r['image']: r for r in rows}
    if len(gt) != len(rows):
        raise ValueError('duplicate GT image')
    excluded = {r['image']: r for r in exclusions}
    if len(excluded) != len(exclusions) or set(excluded)-set(gt):
        raise ValueError('duplicate or foreign reviewed exclusion')
    for key, item in excluded.items():
        if item['sha256'] != gt[key]['source_sha256']['intensity']:
            raise ValueError('exclusion RGB SHA mismatch')
    seen = set()
    for p in predictions:
        key = p['image']
        if key in seen or key not in gt:
            raise ValueError('duplicate or foreign prediction')
        seen.add(key)
        if p['source_sha256']['intensity'] != gt[key]['source_sha256']['intensity']:
            raise ValueError('prediction RGB SHA mismatch')
    if seen != set(gt):
        raise ValueError('incomplete predictions')
    return {k: r for k, r in gt.items() if k not in excluded}, [p for p in predictions if p['image'] not in excluded]


def score(gt, predictions, confidence):
    counts = {label: Counter() for label in COCO7}
    by_weather, sizes = {}, {name: Counter() for name in ('below10px', '10to19px', '20px_or_more')}
    for p in predictions:
        r = gt[p['image']]
        measured, _, pairs, known, _ = evaluate(r['annotations'], p['predictions'], confidence)
        weather = weather_tag(r['run'])
        if weather not in by_weather:
            by_weather[weather] = {label: Counter() for label in COCO7}
        for label in COCO7:
            counts[label].update(measured[label])
            by_weather[weather][label].update(measured[label])
        matched = {g for g, _, _ in pairs}
        for n, a in enumerate(known):
            height = a['xyxy'][3]-a['xyxy'][1]
            bucket = 'below10px' if height < 10 else ('10to19px' if height < 20 else '20px_or_more')
            sizes[bucket].update(gt=1, tp=int(n in matched))
    pooled = summary(counts)
    pooled['total_tp'] = sum(c['tp'] for c in counts.values())
    temporal = temporal_report(gt, predictions, confidence)
    return dict(confidence=confidence, pooled=pooled,
        weather={k: summary(v) for k, v in by_weather.items()},
        size_recall={k: dict(v, recall=v['tp']/v['gt'] if v['gt'] else None) for k, v in sizes.items()},
        temporal_summary={k: v for k, v in temporal.items() if k != 'objects'},
        temporal_objects=temporal['objects'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--conditions', type=Path, required=True)
    parser.add_argument('--live', type=Path)
    parser.add_argument('--confirmation', type=Path)
    parser.add_argument('--confirmation-candidates', type=Path)
    parser.add_argument('--actual-selected', type=Path, help='Selected actual-cutoff selection result directory')
    parser.add_argument('--output', type=Path, required=True)
    a = parser.parse_args()
    if a.output.exists():
        parser.error('refusing to overwrite prior evidence')
    if bool(a.confirmation) != bool(a.confirmation_candidates):
        parser.error('confirmation and its frozen candidates must be supplied together')
    conditions = json.loads(a.conditions.read_text())
    manifest = a.root/'gt-v2/frames.jsonl'
    rows = [r for r in load_jsonl(manifest) if r['split'] == 'selection' and r['annotation_complete']]
    report = dict(scope='All8 combinations; selection only; reviewed exclusions do not rewrite original metrics.',
                  conditions=conditions, quality=[], live=[], source_sha256={})
    def pin(path):
        report['source_sha256'][str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    pin(manifest)
    pin(a.conditions)
    for model, width in COMBINATIONS:
        directory = a.root/'quality'/f'{model}-{width}-selection'
        source = json.loads((directory/'summary.json').read_text())
        if source['gt_manifest_sha256'] != hashlib.sha256(manifest.read_bytes()).hexdigest():
            raise ValueError('GT manifest changed since original prediction')
        predictions = load_jsonl(directory/'predictions.jsonl')
        gt, included = selected_frames(rows, predictions, conditions['reviewed_exclusions'])
        pin(directory/'summary.json')
        pin(directory/'predictions.jsonl')
        report['quality'].append(dict(model=model, width=width, included_frames=len(gt),
            excluded_frames=len(rows)-len(gt), engine_sha256=source['model']['weights_sha256'],
            original_predictor_confidence=source['model']['conf_thres'],
            score_method='Saved actual0.25 predictions thresholded at0.50/0.70; selected winner needs actual cutoff validation.',
            results=[score(gt, included, c) for c in (.5, .7)]))
    if a.live:
        for p in sorted(a.live.glob('*/*/report.json')):
            r = json.loads(p.read_text())
            pin(p)
            report['live'].append(dict(path=str(p), job=r['job'], complete=r.get('observation_window_completed', False),
                passed=r['passed'], failures=r['failures'], measurements=r.get('measurements'),
                target_attainment=r.get('target_attainment'), resources=r.get('resources_startup_and_active'),
                planning=r.get('planning')))
        if {(r['job']['model'], r['job']['width']) for r in report['live']} != set(COMBINATIONS):
            raise ValueError('incomplete8model screen')
    if a.confirmation:
        candidates = json.loads(a.confirmation_candidates.read_text())
        pin(a.confirmation_candidates)
        pin(a.confirmation/'weather-protocol.json')
        reports = []
        for p in sorted(a.confirmation.glob('*/*/report.json')):
            pin(p)
            reports.append(json.loads(p.read_text()))
        report['confirmation'] = confirmation_summary(reports, candidates)
    if a.actual_selected:
        directory = a.actual_selected
        source = json.loads((directory/'summary.json').read_text())
        confidence = source['frozen_confidence']
        if source['split'] != 'selection' or confidence is None or source['model']['conf_thres'] != confidence:
            raise ValueError('actual selected evidence must use its frozen selection predictor cutoff')
        if source['gt_manifest_sha256'] != hashlib.sha256(manifest.read_bytes()).hexdigest():
            raise ValueError('actual selected GT changed')
        pin(directory/'summary.json')
        pin(directory/'predictions.jsonl')
        gt, predictions = selected_frames(rows, load_jsonl(directory/'predictions.jsonl'), conditions['reviewed_exclusions'])
        report['actual_selected'] = dict(original_frames=source['frames'], original_thresholds=source['thresholds'],
            reviewed_frames=len(gt), reviewed_result=score(gt, predictions, confidence))
    a.output.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')


if __name__ == '__main__':
    main()
