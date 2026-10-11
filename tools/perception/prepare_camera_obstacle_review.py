#!/usr/bin/env python3
"""Select deferred training obstacles for independent RGB class-only review."""
import argparse
import copy
import hashlib
import json
from pathlib import Path

from camera_data_audit import read_jsonl, save, sha, write_jsonl
from prepare_camera_direct_review import prepare


def class_reviews(samples, crops, codes, previous):
    """Convert explicit per-crop RGB decisions; '?' never changes a label."""
    names = dict(B='wooden_box', C='concrete_barrier', S='steel_barricade',
                 D='traffic_drum', P='plastic_barrier')
    changes = {}
    inspected = {}
    for number in range((len(crops)+31)//32):
        batch = crops[number*32:(number+1)*32]
        decisions = codes[str(number)].split()
        if len(decisions) != len(batch):
            raise ValueError('crop decision count mismatch')
        for entry, code in zip(batch, decisions):
            if code not in names and code != '?':
                raise ValueError('unknown crop decision')
            sample = samples[entry['index']]
            if sample['id'] != entry['id'] or sample['image_sha256'] != entry['image_sha256']:
                raise ValueError('crop identity mismatch')
            inspected.setdefault(entry['id'], []).append(dict(
                annotation_id=entry['annotation']['id'], sheet=entry['sheet'], code=code))
            if code != '?':
                changes.setdefault(entry['id'], {})[str(entry['annotation']['id'])] = names[code]
    result = copy.deepcopy(previous)
    for sample in samples:
        ident = sample['id']
        if ident not in inspected:
            continue
        old = result.get(ident, {})
        if old and old.get('image_sha256') != sample['image_sha256']:
            raise ValueError('previous review SHA mismatch')
        # Prior verified labels already live in the previous curated rows.
        # Do not promote unresolved proposals from an old pending review.
        if any('xyxy' in a for a in old.get('overrides', [])) or old.get('additions'):
            raise ValueError('geometry review requires separate handling')
        overrides = {key: dict(id=key, canonical=value)
                     for key, value in changes.get(ident, {}).items()}
        result[ident] = dict(image_sha256=sample['image_sha256'], decision='pending',
            all_targets_annotated=False, human_confirmed=False,
            reviewer='Codex independent RGB annotation crop inspection',
            notes='Class-only RGB crop review of original boxes; completeness is not asserted. '
                  'Question-mark entries keep the previous unresolved class.',
            overrides=list(overrides.values()),
            evidence=dict(crops=inspected[ident], source=sample['source'],
                          previous_review=old))
    return result


def select(rows, config):
    selected = {}
    for selection in config['selection']:
        matching = [r for r in rows if not r['eligible'] and r['split'] == 'train'
            and r['source'] == selection['source']
            and not any('split' in s or 'duplicate' in s or 'representative' in s for s in r['reasons'])
            and any(a['original_label'] in selection['labels'] for a in r['annotations'])]
        matching.sort(key=lambda r: hashlib.sha256(
            f"{config['seed']}:{r['id']}".encode()).hexdigest())
        selected.update((r['id'], r) for r in matching[:selection['limit']])
    return list(selected.values())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('previous', 'config', 'output'):
        parser.add_argument('--'+key, type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    rows = select(list(read_jsonl(args.previous)), config)
    args.output.mkdir(parents=True, exist_ok=False)
    stage = args.output/'selection'
    stage.mkdir()
    write_jsonl(stage/'curated.jsonl', rows)
    buckets = list(dict.fromkeys(s['source']+'|'+label for s in config['selection'] for label in s['labels']))
    save(stage/'review-progress.json', dict(buckets=[dict(bucket=b) for b in buckets]))
    labels = list(dict.fromkeys(label for s in config['selection'] for label in s['labels']))
    report = prepare(stage, buckets, args.output/'rgb', all_images=True, crop_labels=labels)
    save(args.output/'selection.json', dict(config_sha256=sha(args.config),
        previous_sha256=sha(args.previous), image_ids=[r['id'] for r in rows],
        generated=report, approved_images=0, annotation_completeness_asserted=False))
    print(json.dumps(report))


if __name__ == '__main__':
    main()
