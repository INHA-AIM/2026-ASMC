#!/usr/bin/env python3
"""Read-only storage preflight for YOLO exports and native/cache TwinLite rows."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/data_collection/src'))
from data_collection.data_locations import load_locations


def check_dataset(dataset, data_root):
    missing = []
    counts = {}
    def check(path):
        if not path.is_file():
            missing.append(str(path))
    if (dataset/'images').is_dir():
        for split in ('train', 'val', 'test'):
            images = list((dataset/'images'/split).iterdir())
            counts[split] = len(images)
            for image in images:
                check(image)
                check(dataset/'labels'/split/(image.stem+'.txt'))
    else:
        for split in ('train', 'val', 'test'):
            manifest = dataset/(split+'.jsonl')
            counts[split] = 0
            with manifest.open() as stream:
                for line in stream:
                    row = json.loads(line)
                    counts[split] += 1
                    paths = [row['image']]
                    for key in ('targets', 'valid_masks', 'auxiliary_targets'):
                        paths.extend(row.get(key, {}).values())
                    if row.get('valid_mask'):
                        paths.append(row['valid_mask'])
                    for value in paths:
                        path = Path(value)
                        check(path if path.is_absolute() else data_root/path)
    manifests = [dataset/'manifest.jsonl'] if (dataset/'manifest.jsonl').is_file() else list(dataset.glob('*.jsonl'))
    return dict(dataset=str(dataset), samples=counts, missing_count=len(missing),
                missing_examples=missing[:20],
                manifest_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in manifests})


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data-root', type=Path, default=ROOT.parent/'data')
    p.add_argument('--dataset', type=Path, action='append', required=True)
    args = p.parse_args()
    locations = load_locations()
    result = dict(locations=locations, roots_available={
        key: Path(value).is_dir() for key, value in locations.items() if key.endswith('_wsl')},
        datasets=[check_dataset(d, args.data_root) for d in args.dataset])
    print(json.dumps(result, indent=2))
    return int(any(d['missing_count'] for d in result['datasets']))


if __name__ == '__main__':
    sys.exit(main())
