#!/usr/bin/env python3
"""Read-only Capture GT size audit in original and letterboxed tensor pixels."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
from PIL import Image


def tensor_size(xyxy, image_size, target, scaleup=True):
    width, height = image_size
    if min(width, height, target) <= 0:
        raise ValueError('positive image and tensor dimensions required')
    x1, y1, x2, y2 = map(float, xyxy)
    if not all(math.isfinite(x) for x in (x1, y1, x2, y2)) or x2 <= x1 or y2 <= y1:
        raise ValueError('finite positive bbox required')
    gain = target / max(width, height)
    if not scaleup:
        gain = min(gain, 1.0)
    return ((x2 - x1) * gain, (y2 - y1) * gain)


def category(width, height):
    short, long = sorted((width, height))
    if short < 2 or width * height < 16:
        return 'near_point'
    if short < 4 or long < 8:
        return 'very_small'
    if short < 8 or long < 16:
        return 'small'
    return 'larger'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--training-size', type=int, default=960)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('new immutable audit output required')
    totals, by_class, by_view = Counter(), defaultdict(Counter), defaultdict(Counter)
    image_types, datasets, examples = Counter(), [], defaultdict(list)
    class_images = defaultdict(Counter)
    for dataset in args.dataset:
        manifest = dataset / 'manifest.jsonl'
        dataset_counts = Counter()
        for line in manifest.read_text().splitlines():
            row = json.loads(line)
            split = row.get('split', 'train')
            path = dataset / 'images' / split / (row['key'] + '.png')
            with Image.open(path) as image:
                size = image.size
            kinds = set()
            per_image = defaultdict(set)
            for ann in row['annotations']:
                width, height = tensor_size(ann['xyxy'], size, args.training_size)
                kind = category(width, height)
                totals[kind] += 1
                dataset_counts[kind] += 1
                by_class[ann['canonical']][kind] += 1
                by_view[row['view']][kind] += 1
                kinds.add(kind)
                per_image[ann['canonical']].add(kind)
                if len(examples[kind]) < 12:
                    examples[kind].append(dict(dataset=str(dataset), key=row['key'],
                        canonical=ann['canonical'], xyxy=ann['xyxy'], original_size=size,
                        tensor_wh=[width, height], source_image=str(path.resolve())))
            image_types['total'] += 1
            image_types['only_near_point'] += kinds == {'near_point'}
            image_types['mixed_near_point_and_larger'] += 'near_point' in kinds and len(kinds) > 1
            for label, present in per_image.items():
                class_images[label]['any'] += 1
                class_images[label]['non_point'] += bool(present - {'near_point'})
                class_images[label]['small_or_larger'] += bool(present & {'small', 'larger'})
        datasets.append(dict(path=str(dataset), manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),
                             objects=dict(dataset_counts)))
    report = dict(training_size=args.training_size, scaleup=True, scope='square letterbox before random augmentation; inference uses view-specific tensors',
        thresholds='diagnostic only: near_point short<2px or area<16px2; very_small short<4 or long<8; small short<8 or long<16',
        labels_modified=False, training_ignore_implemented=False, totals=dict(totals),
        class_counts={k:dict(v) for k,v in by_class.items()}, view_counts={k:dict(v) for k,v in by_view.items()},
        class_images={k:dict(v) for k,v in class_images.items()},
        images=dict(image_types), datasets=datasets, examples=dict(examples))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('totals','images','labels_modified')}))


if __name__ == '__main__':
    main()
