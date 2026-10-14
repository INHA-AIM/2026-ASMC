#!/usr/bin/env python3
"""Frozen low-cutoff predictions -> confidence/NMS grid; never run heldout tuning."""
import argparse
import hashlib
import importlib
import json
from pathlib import Path
import sys


def read(path):
    return [json.loads(s) for s in Path(path).read_text().splitlines() if s]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--predictions', type=Path, required=True)
    p.add_argument('--evaluator-dir', type=Path, default=Path(__file__).resolve().parent)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--split', choices=('selection','heldout'), default='selection')
    p.add_argument('--confirm-nms', type=float)
    p.add_argument('--confirm-confidence', type=float)
    p.add_argument('--confirm-exclusive', type=float)
    a = p.parse_args()
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/asmc_camera/src'))
    sys.path.insert(0, str(a.evaluator_dir))
    ev = importlib.import_module('evaluate_camera_bbox_pilot')
    from asmc_camera.detection.mission import GROUPS, group_of
    from asmc_camera.detection.output_nms import suppress, iou
    ev.accepted_name = lambda name: group_of(name) or 'unknown'
    gt = [r for r in read(a.manifest) if r['split'] == a.split]
    raw = [r for r in read(a.predictions) if r['id'] in {r['id'] for r in gt}]
    def evaluate(conf, threshold, exclusive):
        reduced = [dict(r, predictions=suppress(r['predictions'], conf, threshold, exclusive)[0]) for r in raw]
        labels = ev.summarize(gt, reduced, conf, pixel_scope=True)
        groups = ev.summarize(gt, reduced, conf, level='accepted', pixel_scope=True)
        groups['macro_f1_fixed3'] = sum(groups['classes'][g]['f1'] for g in GROUPS) / len(GROUPS)
        duplicates = wrong_name = background = 0
        by = {r['id']: r for r in reduced}
        for row in gt:
            if not ev.scoped_complete(row, True):
                continue
            predictions = by[row['id']]['predictions']
            _, pairs = ev.frame_counts(row, predictions, conf, pixel_scope=True)
            matched = {i for _, i, _ in pairs}
            positives = [x for x in row['annotations'] if x.get('supervision') not in ('ignore', 'excluded') and ev.training_label(x)]
            ignored = [x for x in row['annotations'] if x.get('supervision') == 'ignore']
            excluded = ev.ignored_prediction_indices(predictions, ignored, matched, dict(ignore_prediction_ioa=.5))
            for i, prediction in enumerate(predictions):
                if i in matched or i in excluded:
                    continue
                nearby = [x for x in positives if iou(x['xyxy'], prediction['xyxy']) >= .5]
                if any(ev.training_label(x) == prediction['class_name'] for x in nearby):
                    duplicates += 1
                elif nearby:
                    wrong_name += 1
                else:
                    background += 1
        return dict(confidence=conf, nms_iou=threshold, exclusive_iou=exclusive,
                    labels=labels, groups=groups, duplicate_fp=duplicates,
                    wrong_name_fp=wrong_name, background_or_geometry_fp=background)
    coordinate_policy = 'actual packed tensor coordinates (runtime equivalent)' if all('nms_xyxy' in p for r in raw for p in r['predictions']) else 'legacy image-space cache; clipping may differ from runtime'
    baseline = evaluate(.4, .7, None)
    if a.split == 'heldout':
        if a.confirm_nms is None or a.confirm_confidence is None:
            p.error('heldout requires frozen confirm settings; no grid search allowed')
        candidate = evaluate(a.confirm_confidence, a.confirm_nms, a.confirm_exclusive)
        a.output.parent.mkdir(parents=True, exist_ok=True)
        a.output.write_text(json.dumps(dict(baseline=baseline, confirmation=candidate,
            scope='Previously collected heldout; fixed settings, no retuning.', nms_coordinate_policy=coordinate_policy,
            manifest_sha256=hashlib.sha256(a.manifest.read_bytes()).hexdigest()),indent=2)+'\n')
        print(json.dumps(dict(baseline_f1=baseline['groups']['macro_f1_fixed3'],
                             candidate_f1=candidate['groups']['macro_f1_fixed3'])))
        return
    grid = []
    for conf in [round(.1 + .05 * i, 2) for i in range(13)]:
        for threshold in (.5, .6, .7, .8):
            for exclusive in (None, .9):
                row = evaluate(conf, threshold, exclusive)
                row['recall_gate'] = all(row['groups']['classes'][g]['recall'] >=
                    baseline['groups']['classes'][g]['recall'] - .01 for g in GROUPS
                    if baseline['groups']['classes'][g]['complete_gt'] >= 20)
                grid.append(row)
    ranked = sorted((r for r in grid if r['recall_gate']), key=lambda r: (
        -r['groups']['macro_f1_fixed3'], -r['labels']['macro_f1_fixed16'],
        r['exclusive_iou'] is not None, abs(r['confidence']-.4), abs(r['nms_iou']-.7)))
    winner = ranked[0]
    report = dict(baseline=baseline, selection_winner=winner, grid=grid,
        hashes={str(f): hashlib.sha256(f.read_bytes()).hexdigest() for f in
                (a.manifest, a.predictions, a.evaluator_dir/'evaluate_camera_bbox_pilot.py')},
        scope='Reused val205 development selection; no unseen-test claim.', nms_coordinate_policy=coordinate_policy,
        runtime_default_changed=False, confirmation_required=True,
        stable_detection='Not derivable from shuffled val205; requires temporal confirmation.')
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: dict({m: v[m] for m in ('confidence','nms_iou','exclusive_iou','duplicate_fp',
        'wrong_name_fp','background_or_geometry_fp')}, f1_3=v['groups']['macro_f1_fixed3'],
        f1_16=v['labels']['macro_f1_fixed16']) for k,v in [('baseline',baseline),('selection_winner',winner)]}))


if __name__ == '__main__':
    main()
