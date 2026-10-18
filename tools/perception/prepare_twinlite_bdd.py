#!/usr/bin/env python3
"""Wrap the official BDD Large state dict without changing any model tensors."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/asmc_camera/src'))

SOURCE_SHA = '5605d1a8ce762c1b6f7d23f4c2e7ec9384bfd6e6887b040b43e1f8902a817e22'
SOURCE_URL = 'https://drive.google.com/file/d/1H8P-GrOUBOaVs5LEqXBfz0dC9gguUUio/view'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    import torch
    from asmc_camera.training.twinlite import load_external_twinlite
    from asmc_camera.training.twinlite.tasks import BINARY_LANE_TASK
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--twinlite-root')
    a = p.parse_args()
    if sha(a.source) != SOURCE_SHA:
        raise ValueError('official source SHA mismatch')
    if a.output.exists():
        raise FileExistsError(a.output)
    original = torch.load(str(a.source), map_location='cpu', weights_only=True)
    model, commit = load_external_twinlite(config='large', root=a.twinlite_root)
    model.load_state_dict(original, strict=True)
    state = dict(schema_version='twinlite-morai-checkpoint-1.1.0', epoch=-1,
                 model_state_dict=original, ema_state_dict=None, upstream_commit=commit,
                 args=dict(config='large', training_heads=['drivable', 'lane']),
                 task_spec=BINARY_LANE_TASK.as_dict(),
                 provenance=dict(dataset='BDD100K', source_url=SOURCE_URL,
                                 source_sha256=SOURCE_SHA, training_performed=False))
    a.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(state, str(a.output))
    restored = torch.load(str(a.output), map_location='cpu', weights_only=True)['model_state_dict']
    if set(original) != set(restored) or any(not torch.equal(v, restored[k]) for k, v in original.items()):
        raise ValueError('packaging changed a tensor')
    manifest = dict(schema='twinlite_bdd_pretrained_v1', created_at=datetime.now(timezone.utc).isoformat(),
                    source_url=SOURCE_URL, source_sha256=SOURCE_SHA,
                    official_readme='https://github.com/chequanghuy/TwinLiteNetPlus',
                    upstream_commit=commit, model_config='large', task_spec=BINARY_LANE_TASK.as_dict(),
                    checkpoint=a.output.name, checkpoint_sha256=sha(a.output), bytes=a.output.stat().st_size,
                    tensor_count=len(restored), parameter_count=sum(x.numel() for x in model.parameters()),
                    all_source_tensors_identical=True, training_performed=False,
                    official_preprocessing=dict(input_hw=[384, 640], color='RGB', normalization='uint8 / 255',
                                                resize='OpenCV INTER_LINEAR', padding=114),
                    live_input_hw_by_view=dict(front=[544, 960], left=[480, 640], right=[480, 640]),
                    deployment_status='experimental binary semantic visualization; no family/planning output')
    a.output.with_suffix('.source.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
