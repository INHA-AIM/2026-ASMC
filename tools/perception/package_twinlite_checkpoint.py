#!/usr/bin/env python3
"""Package a completed managed EMA for the existing MORAI inference loader."""
import argparse
import hashlib
import json
from pathlib import Path

import torch


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def deployment_state(source, source_sha):
    if source.get('schema_version') != 'twinlite-managed-checkpoint-1.0.0':
        raise ValueError('expected a managed training checkpoint')
    epoch = source.get('epoch')
    if (source.get('epoch_complete') is not True or not isinstance(epoch, int)
            or epoch < 1 or source.get('completed_epochs') != epoch):
        raise ValueError('only a complete saved epoch may be deployed')
    if not source.get('ema_state_dict') or not source.get('validation'):
        raise ValueError('validated EMA is required')
    spec = source.get('task_spec') or {}
    expected = {'drivable': ['background', 'drivable'],
                'road_marking': ['background', 'white_lane', 'yellow_lane', 'stopline']}
    if (spec.get('class_names') != expected
            or spec.get('heads') != ['drivable', 'road_marking']
            or spec.get('class_counts') != {'drivable': 2, 'road_marking': 4}):
        raise ValueError('MORAI drivable/road-marking class order differs')
    config = (source.get('args') or {}).get('config')
    if config not in ('nano', 'small', 'medium', 'large') or not source.get('upstream_commit'):
        raise ValueError('missing model scale/upstream provenance')
    return dict(schema_version='twinlite-morai-checkpoint-1.1.0', epoch=epoch,
                args={'config': config}, upstream_commit=source['upstream_commit'],
                task_spec=spec, ema_state_dict=source['ema_state_dict'],
                initialization=source.get('initialization'),
                dataset_provenance=source.get('dataset_provenance'),
                deployment_source={'schema_version': source['schema_version'],
                                   'checkpoint_sha256': source_sha,
                                   'epoch': epoch, 'weights': 'ema_state_dict'})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or args.output.with_suffix('.source.json').exists():
        raise ValueError('deployment destination already exists')
    source = torch.load(args.checkpoint, map_location='cpu')
    source_sha = sha(args.checkpoint)
    result = deployment_state(source, source_sha)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix('.pt.part')
    torch.save(result, temporary)
    loaded = torch.load(temporary, map_location='cpu')
    tensors = source['ema_state_dict']
    if set(tensors) != set(loaded['ema_state_dict']):
        raise ValueError('EMA tensor names changed')
    for name, expected in tensors.items():
        actual = loaded['ema_state_dict'][name]
        if actual.dtype != expected.dtype or not torch.equal(actual, expected):
            raise ValueError('EMA tensor/buffer changed: ' + name)
    temporary.replace(args.output)
    report = dict(source_checkpoint=str(args.checkpoint), source_sha256=source_sha,
                  checkpoint=args.output.name, checkpoint_sha256=sha(args.output),
                  bytes=args.output.stat().st_size, source_bytes=args.checkpoint.stat().st_size,
                  epoch=result['epoch'], model_config=result['args']['config'],
                  weights='ema', tensor_count=len(tensors), all_tensors_equal=True,
                  task_spec=result['task_spec'], upstream_commit=result['upstream_commit'],
                  optimizer_state_included=False)
    args.output.with_suffix('.source.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
