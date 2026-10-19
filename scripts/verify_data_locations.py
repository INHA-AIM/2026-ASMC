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
from data_collection.storage_paths import default_data_root


def check_dataset(dataset, data_root):
    dataset, data_root = Path(dataset).expanduser(), Path(data_root).expanduser()
    missing = []
    counts, errors, checked = {}, [], []
    def check(path):
        if not path.is_file():
            missing.append(str(path))
    if (dataset/'images').is_dir():
        for split in ('train', 'val', 'test'):
            directory = dataset/'images'/split
            if not directory.is_dir():
                counts[split] = 0
                continue
            checked.append(split)
            images = sorted(directory.iterdir())
            counts[split] = len(images)
            for image in images:
                check(image)
                check(dataset/'labels'/split/(image.stem+'.txt'))
    else:
        for split in ('train', 'val', 'test'):
            manifest = dataset/(split+'.jsonl')
            counts[split] = 0
            if not manifest.is_file():
                continue
            checked.append(split)
            with manifest.open() as stream:
                for number, line in enumerate(stream, 1):
                    if not line.strip():
                        continue
                    try:
                        row = json.loads(line)
                        paths = [row['image']]
                        for key in ('targets', 'valid_masks', 'auxiliary_targets'):
                            paths.extend(row.get(key, {}).values())
                        if row.get('valid_mask'):
                            paths.append(row['valid_mask'])
                        for value in paths:
                            if not isinstance(value, str) or not value:
                                raise ValueError('path must be a non-empty string')
                            path = Path(value).expanduser()
                            check(path if path.is_absolute() else data_root/path)
                        counts[split] += 1
                    except (ValueError, TypeError, KeyError, AttributeError) as error:
                        errors.append('{}:{}: {}'.format(manifest, number, error))
    if not checked or not sum(counts.values()):
        errors.append('No dataset samples found in image splits or split JSONL manifests')
    manifests = [dataset/'manifest.jsonl'] if (dataset/'manifest.jsonl').is_file() else list(dataset.glob('*.jsonl'))
    hashes = {}
    for manifest in sorted(manifests):
        digest = hashlib.sha256()
        with manifest.open('rb') as stream:
            for block in iter(lambda: stream.read(1024*1024), b''):
                digest.update(block)
        hashes[manifest.name] = digest.hexdigest()
    return dict(dataset=str(dataset), samples=counts, missing_count=len(missing),
                checked_splits=checked, errors=errors, error_count=len(errors),
                missing_examples=missing[:20],
                manifest_sha256=hashes)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data-root', type=Path, default=Path(default_data_root()))
    p.add_argument('--dataset', type=Path, action='append', required=True)
    args = p.parse_args(argv)
    try:
        locations = load_locations()
        result = dict(locations=locations, roots_available={
            key: bool(value) and Path(value).is_dir()
            for key, value in locations.items() if key.endswith('_wsl')},
            datasets=[check_dataset(d, args.data_root) for d in args.dataset])
    except (OSError, ValueError, RuntimeError) as error:
        print(json.dumps({'error': str(error)}, indent=2))
        return 1
    print(json.dumps(result, indent=2))
    return int(any(d['missing_count'] or d['error_count'] for d in result['datasets']))


if __name__ == '__main__':
    sys.exit(main())
