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


def link_package_models(manifest, models_root, package_models):
    """Expose verified external artifacts without duplicating large weights.

    Links are local conveniences, not runtime dependencies or Git artifacts.
    Never replace an unrelated existing package file, including a broken link.
    """
    models_root, package_models = Path(models_root).resolve(), Path(package_models).resolve()
    entries = []
    destinations = set()
    for item in manifest['files']:
        asset, relative = Path(item['asset']), Path(item['relative_path'])
        if asset.name != str(asset) or relative.is_absolute() or '..' in relative.parts:
            raise ValueError('unsafe model link path')
        source, link = (models_root/relative).resolve(), package_models/asset
        if link in destinations:
            raise ValueError('duplicate package model link')
        destinations.add(link)
        if models_root not in source.parents:
            raise ValueError('model link source outside models root')
        if source.stat().st_size != item['bytes'] or sha256(source) != item['sha256']:
            raise ValueError('installed model mismatch: '+str(source))
        if (link.exists() or link.is_symlink()) and (not link.is_symlink() or link.resolve() != source):
            raise ValueError('refusing to replace package model: '+str(link))
        entries.append((source, link))
    package_models.mkdir(parents=True, exist_ok=True)
    for source, link in entries:
        if not link.is_symlink():
            link.symlink_to(os.path.relpath(source,package_models))
    return dict(links=[str(link) for _,link in entries], copied_bytes=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path,
                        default=Path(__file__).resolve().parents[2] / 'config/models/camera_object_models_v1.json')
    parser.add_argument('--release-dir', type=Path, required=True)
    parser.add_argument('--models-root', type=Path, required=True)
    parser.add_argument('--verify-only', action='store_true')
    parser.add_argument('--package-models', type=Path, help='Optional local links to verified installed files')
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    result = install(manifest, args.release_dir, args.models_root, args.verify_only)
    if args.package_models and not args.verify_only:
        result['package_models'] = link_package_models(manifest,args.models_root,args.package_models)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
