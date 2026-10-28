#!/usr/bin/env python3
"""Validate approved history-only assets and the models still shipped for driving.

Normal CI checks the retained inventory without downloading historical blobs.
--verify-history additionally checks the original bytes in a full local clone.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
REGISTRY = 'config/artifacts/git-history-assets-v1.json'


def validate_inventory(registry):
    if registry.get('schema_version') != 1 or not registry.get('assets'):
        raise ValueError('nonempty history asset inventory v1 required')
    if not re.fullmatch(r'[0-9a-f]{40}', registry.get('history_commit', '')):
        raise ValueError('history must use an immutable full commit SHA')
    seen = set()
    for asset in registry['assets']:
        path = PurePosixPath(asset['path'])
        if path.is_absolute() or '..' in path.parts or str(path) != asset['path']:
            raise ValueError('unsafe asset path: '+asset['path'])
        if asset['path'] in seen:
            raise ValueError('duplicate history asset: '+asset['path'])
        seen.add(asset['path'])
        if (not re.fullmatch(r'[0-9a-f]{64}', asset.get('sha256', '')) or
                not re.fullmatch(r'[0-9a-f]{40}', asset.get('git_blob', '')) or
                not isinstance(asset.get('bytes'), int) or asset['bytes'] <= 0):
            raise ValueError('missing byte identity: '+asset['path'])
    return seen


def check_retired(root, registry, verify_history=False):
    validate_inventory(registry)
    errors = []
    for asset in registry['assets']:
        if (root/asset['path']).exists() or (root/asset['path']).is_symlink():
            errors.append('retired asset reintroduced: '+asset['path'])
        if verify_history:
            source = registry['history_commit']+':'+asset['path']
            blob = subprocess.check_output(['git', 'rev-parse', source], cwd=str(root), text=True).strip()
            data = subprocess.check_output(['git', 'show', source], cwd=str(root))
            if (blob != asset['git_blob'] or len(data) != asset['bytes'] or
                    hashlib.sha256(data).hexdigest() != asset['sha256']):
                errors.append('historical bytes changed: '+asset['path'])
    return errors


def check_models(root):
    errors = []
    manifest = json.loads((root/'config/models/camera_object_models_v3.json').read_text())
    files = manifest['files']
    if ({row['profile'] for row in files} != {'collect_e3', 'finetuned_e8'} or
            len(files) != 4 or manifest['default_profile'] != 'collect_e3'):
        errors.append('current object deployment must contain E3 and E8 PT/engine pairs')
    model_root = root/'src/asmc_camera/models'
    expected = [(model_root/row['path'], row['sha256']) for row in files]
    lane = json.loads((model_root/'lane_segmentation/manifest-large-v7-e8.json').read_text())
    expected.append((model_root/'lane_segmentation'/lane['checkpoint'], lane['checkpoint_sha256']))
    expected.extend((model_root/'lane_segmentation'/row['file'], row['sha256']) for row in lane['engines'])
    for path, sha in expected:
        if not path.is_file() or path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest() != sha:
            errors.append('current deployment missing or changed: '+str(path.relative_to(root)))
    if not (model_root/'traffic_light/best.pt').is_file():
        errors.append('traffic-light deployment missing')
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify-history', action='store_true')
    args = parser.parse_args()
    registry = json.loads((ROOT/REGISTRY).read_text())
    errors = check_retired(ROOT, registry, args.verify_history) + check_models(ROOT)
    for error in errors:
        print('FAIL '+error)
    if not errors:
        print('PASS %d history-only assets (%d bytes); E3/E8, Large E8 and traffic light intact' %
              (len(registry['assets']), sum(row['bytes'] for row in registry['assets'])))
    return bool(errors)


if __name__ == '__main__':
    raise SystemExit(main())
