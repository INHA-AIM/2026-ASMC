#!/usr/bin/env python3
"""Rescore saved predictions; never present a post-filter as a new GPU trial."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from seg_measurements import GROUPS, INFRASTRUCTURE, frame_metrics, summarize, choose_threshold


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def filter_predictions(predicted, allowed, confidence):
    """Return original indices: removing a box must not shift its mask identity."""
    return [i for i, box in enumerate(predicted)
            if box['confidence'] >= confidence and box['source_class_name'] not in INFRASTRUCTURE
            and (allowed is None or box['source_class_name'] in allowed)]


def packed_masks(root, relative):
    if not relative:
        return None
    with np.load(root / relative, allow_pickle=False) as data:
        return np.unpackbits(data['masks'], axis=2)[:, :, :int(data['width'])].astype(bool)


def score_frame(truth, predicted, gt_masks, prediction_masks, allowed, confidence):
    indices = filter_predictions(predicted, allowed, confidence)
    return frame_metrics(truth, [predicted[i] for i in indices], gt_masks,
                         None if prediction_masks is None else [prediction_masks[i] for i in indices])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gt', type=Path, required=True)
    parser.add_argument('--accuracy', type=Path, required=True)
    parser.add_argument('--policy', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    policy = yaml.safe_load(args.policy.read_text())
    allowed = frozenset(policy['allowed_names'])
    if not allowed or len(allowed) != len(policy['allowed_names']):
        raise ValueError('mission names must be explicit and unique')
    gt = {(r['run'], r['view'], r['frame']): r for r in
          (json.loads(line) for line in (args.gt / 'frames.jsonl').read_text().splitlines())
          if r['split'] == 'selection' and r['annotation_complete']}
    result = dict(allowed_names=policy['allowed_names'], policy_sha256=sha(args.policy),
                  gt_manifest_sha256=sha(args.gt / 'frames.jsonl'), selection_frames=len(gt),
                  scope='post-filter of saved original predictions; all selection GT retained',
                  fresh_inference=False, runtime_validated=False, yoloe_prompt_cache_changed=False,
                  heldout_evaluated=False, training_performed=False, models={})
    for model in ('baseline', 'yolo26', 'yoloe'):
        directory = args.accuracy / model
        rows = [json.loads(line) for line in (directory / 'predictions.jsonl').read_text().splitlines()]
        keys = [(r['run'], r['view'], r['frame']) for r in rows]
        if len(set(keys)) != len(keys) or set(keys) != set(gt):
            raise ValueError('duplicate, missing or non-selection prediction rows')
        sweep = [round(.10 + .05 * i, 2) for i in range(13)]
        states = {(profile, conf): dict(counts={g: Counter() for g in GROUPS}, masks=[], names=Counter())
                  for profile in ('all', 'mission') for conf in sweep}
        removed = Counter()
        for row in rows:
            truth = gt[(row['run'], row['view'], row['frame'])]
            original = row['predictions']
            gt_masks = packed_masks(args.gt, truth.get('masks'))
            masks = packed_masks(directory, row.get('masks'))
            for box in original:
                if box['confidence'] >= .7 and box['source_class_name'] not in allowed:
                    removed[box['source_class_name']] += 1
            for profile in ('all', 'mission'):
                for confidence in sweep:
                    counts, values, names, _ = score_frame(truth['annotations'], original, gt_masks, masks,
                                                           None if profile == 'all' else allowed, confidence)
                    state = states[profile, confidence]
                    for group in GROUPS:
                        state['counts'][group].update(counts[group])
                    state['masks'].extend(values)
                    state['names'].update(names)
        profiles = {}
        for profile in ('all', 'mission'):
            thresholds = [dict(confidence=c, metrics=summarize(states[profile, c]['counts'],
                           states[profile, c]['masks'], states[profile, c]['names'])) for c in sweep]
            profiles[profile] = dict(thresholds=thresholds, selected_threshold=choose_threshold(thresholds),
                                    fixed_07=next(row for row in thresholds if row['confidence'] == .7))
        old = json.loads((directory / 'summary.json').read_text())
        if profiles['all']['thresholds'] != old['thresholds']:
            raise ValueError('full-class rescoring differs from preserved original evidence')
        result['models'][model] = dict(profiles=profiles, removed_labels_at_07=dict(removed),
                                      predictions_sha256=sha(directory / 'predictions.jsonl'),
                                      preserved_source_config_sha256=old['source_config_sha256'])
        print(model, 'all', profiles['all']['fixed_07']['metrics']['macro_f1'],
              'mission', profiles['mission']['fixed_07']['metrics']['macro_f1'], flush=True)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
