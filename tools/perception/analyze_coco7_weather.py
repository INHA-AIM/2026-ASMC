#!/usr/bin/env python3
"""Stratify saved predictions by Capture weather without retuning thresholds."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import statistics

from evaluate_coco7_models import COCO7, evaluate, summary


def weather_tag(run):
    tags = set(re.findall(r'(?:^|_)(sunny|foggy|cloudy)(?=_|$)', run.lower()))
    if len(tags) != 1:
        raise ValueError('missing or ambiguous Capture weather tag: ' + run)
    return tags.pop()


def stratify(rows, predictions, confidence):
    gt = {r['image']: r for r in rows if r['annotation_complete']}
    if len(gt) != sum(bool(r['annotation_complete']) for r in rows):
        raise ValueError('duplicate GT image')
    counts, frames, ignored, seen = {}, Counter(), Counter(), set()
    for prediction in predictions:
        key = prediction['image']
        if key in seen:
            raise ValueError('duplicate prediction image: ' + key)
        seen.add(key)
        if key not in gt:
            raise ValueError('prediction outside complete GT: ' + key)
        row = gt[key]
        if prediction['source_sha256']['intensity'] != row['source_sha256']['intensity']:
            raise ValueError('RGB SHA mismatch: ' + key)
        if prediction['run'] != row['run'] or prediction['view'] != row['view']:
            raise ValueError('prediction identity mismatch: ' + key)
        weather = weather_tag(row['run'])
        if weather not in counts:
            counts[weather] = {label: Counter() for label in COCO7}
        measured, excluded, _, _, _ = evaluate(row['annotations'], prediction['predictions'], confidence)
        for label in COCO7:
            counts[weather][label].update(measured[label])
        frames[weather] += 1
        ignored[weather] += excluded
    if seen != set(gt):
        raise ValueError('incomplete prediction coverage: %d missing' % len(set(gt) - seen))
    return {weather: dict(frames=frames[weather], ignored_unknown_subtype=ignored[weather],
                          **summary(values)) for weather, values in sorted(counts.items())}


def load_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gt', type=Path, required=True)
    parser.add_argument('--quality', type=Path, nargs='+', required=True)
    parser.add_argument('--split', choices=['selection', 'heldout'], default='selection')
    parser.add_argument('--candidates', type=Path,
                        help='Previously selected pooled-selection confidences; do not tune on weather')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('refusing to overwrite prior evidence')
    manifest = args.gt / 'frames.jsonl'
    rows = [r for r in load_jsonl(manifest) if r['split'] == args.split and r['annotation_complete']]
    if args.split == 'selection' and args.candidates is None:
        parser.error('selection requires the previously selected candidates file')
    candidates = {(c['model'], c['width']): c['confidence']
                  for c in json.loads(args.candidates.read_text())} if args.candidates else {}
    results = []
    directories = sorted(d for root in args.quality for d in root.glob('*-' + args.split))
    if not directories:
        parser.error('no saved evaluations for requested split')
    for directory in directories:
        source = json.loads((directory / 'summary.json').read_text())
        model, width = directory.name.rsplit('-', 2)[:2]
        width = int(width)
        predictions = load_jsonl(directory / 'predictions.jsonl')
        if args.split == 'heldout':
            if source.get('frozen_confidence') is None:
                raise ValueError('heldout predictions must use a selection-frozen threshold')
            thresholds = [source['frozen_confidence']]
        else:
            thresholds = sorted({.7, candidates.get((model, width), .7)})
        for confidence in thresholds:
            results.append(dict(model=model, front_width=width, confidence=confidence,
                                previously_selected=candidates.get((model, width)) == confidence,
                                source_summary_sha256=hashlib.sha256((directory / 'summary.json').read_bytes()).hexdigest(),
                                source_predictions_sha256=hashlib.sha256((directory / 'predictions.jsonl').read_bytes()).hexdigest(),
                                source_gt_manifest_sha256=source['gt_manifest_sha256'],
                                weather=stratify(rows, predictions, confidence)))
            if source['gt_manifest_sha256'] != hashlib.sha256(manifest.read_bytes()).hexdigest():
                raise ValueError('GT manifest SHA mismatch: ' + str(directory))
    heights = {}
    for row in rows:
        weather = weather_tag(row['run'])
        heights.setdefault(weather, [])
        heights[weather].extend(a['xyxy'][3] - a['xyxy'][1] for a in row['annotations']
                               if a.get('subtype_verified') and a['source_class_name'] in COCO7)
    geometry = {w: dict(gt_objects=len(h), median_pixel_height=statistics.median(h) if h else None,
                        below_10_pixels=sum(v < 10 for v in h)) for w, h in heights.items()}
    report = dict(schema_version=1, split=args.split, thresholds_retuned=False, gt_geometry=geometry,
                  weather_provenance='Capture run-name tags; not live-simulator weather or independently verified weather telemetry',
                  interpretation='Sunny and foggy have different GT label coverage; compare models within each weather. '
                                 'No dog/bicycle/motorcycle pixel GT. No new model or operating-default acceptance.',
                  results=results)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
