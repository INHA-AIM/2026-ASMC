#!/usr/bin/env python3
"""Independently re-read Capture training pairs and check frozen evaluation leakage."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/data_collection/src'))
from data_collection.capture_collection import CLASSES


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_pair(row, image_path, label_path):
    if digest(image_path) != row['image_sha256']:
        raise ValueError('image SHA mismatch')
    if digest(label_path) != row['annotation_sha256']:
        raise ValueError('annotation SHA mismatch')
    with Image.open(image_path) as im:
        im = im.convert('RGB')
        width, height = im.size
        pixel_sha = hashlib.sha256(im.tobytes()).hexdigest()
    if pixel_sha != row['pixel_sha256']:
        raise ValueError('decoded pixel SHA mismatch')
    actual = [line.split() for line in label_path.read_text().splitlines() if line.strip()]
    if len(actual) != len(row['annotations']):
        raise ValueError('object count mismatch')
    for fields, ann in zip(actual, row['annotations']):
        if len(fields) != 5 or fields[0] != str(CLASSES.index(ann['canonical'])):
            raise ValueError('class ID mismatch')
        x1, y1, x2, y2 = ann['xyxy']
        if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
            raise ValueError('bbox outside image')
        expected = ((x1 + x2) / (2 * width), (y1 + y2) / (2 * height),
                    (x2 - x1) / width, (y2 - y1) / height)
        if any(abs(float(v) - target) > 1e-8 for v, target in zip(fields[1:], expected)):
            raise ValueError('bbox normalization mismatch')
    return pixel_sha


def evaluation_hashes(rows):
    pixels, files = set(), set()
    for row in rows:
        if row.get('split', row.get('frozen_split')) in ('val', 'test', 'selection', 'heldout'):
            if row.get('pixel_sha256'):
                pixels.add(row['pixel_sha256'])
            if row.get('image_sha256'):
                files.add(row['image_sha256'])
    return pixels, files


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--frozen-manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('choose a new immutable verification file')
    rows = read_rows(args.dataset / 'manifest.jsonl')
    before = digest(args.frozen_manifest)
    eval_pixels, eval_files = evaluation_hashes(read_rows(args.frozen_manifest))
    counts, seen, failures = Counter(), set(), []
    for row in rows:
        try:
            if row['split'] != 'train' or row['split_group'] != row['layout_id']:
                raise ValueError('Capture layout split mismatch')
            pixel = verify_pair(row, args.dataset / 'images/train' / (row['key'] + '.png'),
                                args.dataset / 'labels/train' / (row['key'] + '.txt'))
            if pixel in seen:
                raise ValueError('duplicate pixel image')
            seen.add(pixel)
            if pixel in eval_pixels or row['image_sha256'] in eval_files:
                raise ValueError('frozen evaluation leakage')
            counts.update(a['canonical'] for a in row['annotations'])
        except (ValueError, KeyError, OSError) as exc:
            failures.append(dict(key=row.get('key'), reason=str(exc)))
    if digest(args.frozen_manifest) != before:
        raise ValueError('frozen manifest changed during verification')
    result = dict(images=len(rows), objects=sum(counts.values()), class_objects=dict(counts),
        passed=bool(rows) and not failures, failures=failures, fixed_class_ids=list(CLASSES),
        manifest_sha256=digest(args.dataset/'manifest.jsonl'), frozen_manifest_sha256=before,
        frozen_evaluation_pixel_hashes=len(eval_pixels), frozen_evaluation_file_hashes=len(eval_files),
        checks=['real image and annotation SHA', 'decoded pixels', 'class IDs', 'bbox coordinates',
                'object count', 'layout split', 'exact duplicates', 'frozen evaluation leakage'],
        near_duplicate_leakage='not claimed; separate scene/layout review required')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
