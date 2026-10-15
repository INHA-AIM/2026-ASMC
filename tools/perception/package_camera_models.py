#!/usr/bin/env python3
"""Create inference-only package weights; never modify training checkpoints."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import os


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''):
            h.update(b)
    return h.hexdigest()


def inference_checkpoint(state):
    model = state.get('ema') if state.get('ema') is not None else state.get('model')
    if model is None:
        raise ValueError('checkpoint has neither EMA nor model')
    # Explicit allowlist also removes custom training_model/scheduler_state.
    return dict(model=model, ema=None, epoch=-1, optimizer=None, scaler=None,
                updates=None, best_fitness=None,
                **{k: state[k] for k in ('version', 'license', 'docs', 'date') if k in state},
                deployment_source_epoch=int(state.get('epoch', -1)))


def package(manifest, models_root, package_dir):
    import torch
    package_dir, models_root = Path(package_dir), Path(models_root)
    package_dir.mkdir(parents=True, exist_ok=True)
    verified = []
    for row in manifest['files']:
        source = models_root / row['relative_path']
        target = package_dir / row['asset']
        if source.stat().st_size != row['bytes'] or sha(source) != row['sha256']:
            raise ValueError('source mismatch: ' + str(source))
        if target.is_symlink() and target.resolve() != source.resolve():
            raise ValueError('unrelated package link: ' + str(target))
        verified.append((row, source, target))
    rows = []
    for row, source, target in verified:
        with tempfile.NamedTemporaryFile(dir=package_dir, delete=False) as f:
            temporary = Path(f.name)
        try:
            if row['profile'] == 'finetuned_e8' and source.suffix == '.pt':
                state = torch.load(str(source), map_location='cpu')
                checkpoint = inference_checkpoint(state)
                expected = checkpoint['model'].state_dict()
                # A file object keeps the ZIP root deterministic across temp names.
                with temporary.open('wb') as stream:
                    torch.save(checkpoint, stream)
                restored = torch.load(str(temporary), map_location='cpu')['model']
                actual = restored.state_dict()
                if restored.names != checkpoint['model'].names or actual.keys() != expected.keys():
                    raise ValueError('deployment classes/state keys differ')
                if any(not torch.equal(expected[k], actual[k]) for k in expected):
                    raise ValueError('deployment parameter/buffer differs')
                transformation = 'EMA tensors unchanged; training state removed by allowlist'
            else:
                shutil.copyfile(source, temporary)
                if sha(temporary) != row['sha256']:
                    raise ValueError('copied artifact mismatch')
                transformation = 'byte-for-byte copy'
            if temporary.stat().st_size >= 100 * 1024 * 1024:
                raise ValueError('deployment file exceeds GitHub file limit')
            digest = sha(temporary)
            if target.exists() and not target.is_symlink():
                if sha(target) != digest:
                    raise ValueError('refusing to overwrite different deployment file: ' + str(target))
            else:
                os.replace(temporary, target)
                target.chmod(0o644)
            rows.append(dict(profile=row['profile'], path='object_detection/' + row['asset'],
                sha256=digest, bytes=target.stat().st_size, source_sha256=row['sha256'],
                source_relative_path=row['relative_path'], transformation=transformation))
        finally:
            temporary.unlink(missing_ok=True)
    return dict(schema_version=2, deployment='package-local ordinary Git files',
                weights_license=manifest.get('weights_license','AGPL-3.0; upstream Ultralytics weights'),
                source_release=manifest['release_tag'], files=rows,
                runtime=manifest['runtime'], checkpoint_provenance=dict(manifest['checkpoint_provenance'],
                    finetuned_e8='v11 complete 8epoch epoch7.pt; deployment contains unchanged EMA and no resume state'),
                engine_environment=dict(gpu='RTX 4060 Ti 8GB', tensorrt='10.13.0.35',
                                        platform='Linux x86_64', container='asmc-ros-noetic'))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--models-root', type=Path, required=True)
    p.add_argument('--package-dir', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    result = package(json.loads(a.manifest.read_text()), a.models_root, a.package_dir)
    a.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
