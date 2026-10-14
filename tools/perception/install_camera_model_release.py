#!/usr/bin/env python3
"""Verify a downloaded model release and install its pinned files atomically."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def install(manifest, release_dir, models_root, verify_only=False):
    if manifest.get('schema_version') != 1 or not manifest.get('files'):
        raise ValueError('unsupported or empty model manifest')
    release_dir, models_root = Path(release_dir), Path(models_root).resolve()
    verified = []
    destinations = set()
    # Validate every source and existing target before making any changes.
    for item in manifest['files']:
        asset, relative = Path(item['asset']), Path(item['relative_path'])
        if asset.name != str(asset) or relative.is_absolute() or '..' in relative.parts:
            raise ValueError('unsafe release path')
        source, target = release_dir / asset, (models_root / relative).resolve()
        if models_root not in target.parents or target in destinations:
            raise ValueError('unsafe or duplicate model destination')
        destinations.add(target)
        if source.stat().st_size != item['bytes'] or sha256(source) != item['sha256']:
            raise ValueError('release file mismatch: ' + str(asset))
        if target.exists() and sha256(target) != item['sha256']:
            raise ValueError('refusing to replace a different existing model: ' + str(target))
        verified.append((source, target, item))
    if not verify_only:
        for source, target, item in verified:
            if target.exists():
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as temporary:
                temporary_path = Path(temporary.name)
            try:
                shutil.copyfile(source, temporary_path)
                if sha256(temporary_path) != item['sha256']:
                    raise ValueError('copied model SHA mismatch')
                os.replace(temporary_path, target)
                target.chmod(0o644)
            finally:
                temporary_path.unlink(missing_ok=True)
    return dict(verified_files=len(verified), installed=not verify_only,
                models_root=str(models_root), release_tag=manifest['release_tag'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path,
                        default=Path(__file__).resolve().parents[2] / 'config/models/camera_object_models_v1.json')
    parser.add_argument('--release-dir', type=Path, required=True)
    parser.add_argument('--models-root', type=Path, required=True)
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    print(json.dumps(install(json.loads(args.manifest.read_text()), args.release_dir,
                             args.models_root, args.verify_only), indent=2))


if __name__ == '__main__':
    main()
