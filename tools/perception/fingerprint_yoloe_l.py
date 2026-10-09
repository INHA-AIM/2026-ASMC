#!/usr/bin/env python3
"""Fingerprint the actual local inputs after serial measurements finish."""
import argparse
import json
from pathlib import Path

import tensorrt
import yaml

from run_b0_matrix import manifest, sha


def main():
    root = Path(__file__).resolve().parents[2]
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--results', type=Path, required=True)
    p.add_argument('--prepared', type=Path, required=True)
    p.add_argument('--revision', required=True, help='Runtime source base; per-trial byte hashes remain authoritative')
    p.add_argument('--image-digest', required=True)
    a = p.parse_args()
    config = root/'config/experiments/camera_yoloe_l_first_v1.yaml'
    data = manifest(root, yaml.safe_load(config.read_text()), a.revision)
    data.update(tensorrt=tensorrt.__version__, frozen_image_digest=a.image_digest,
                experiment_sha256=sha(config), measurements_revision=a.revision,
                revision_scope='Runtime base commit; every report also stores actual execution source hashes',
                prepared_sha256={f.name: sha(f) for f in sorted(a.prepared.iterdir())
                                  if f.is_file() and f.suffix in ['.pt', '.ts', '.npz', '.engine', '.onnx', '.json']},
                first_run_tools_sha256={f.name: sha(f) for f in sorted((root/'tools/perception').glob('*yoloe_l*.py'))},
                gt_manifest_sha256=sha(a.results/'gt-v2/frames.jsonl'),
                gallery_scope='Local raw Capture imagery; not uploaded with numeric PR evidence')
    (a.results/'manifest.json').write_text(json.dumps(data, indent=2)+'\n')
    print(json.dumps(dict(bags={k:v['sha256'] for k,v in data['bags'].items()},
                         tensorrt=data['tensorrt'], experiment_sha256=data['experiment_sha256'])))


if __name__ == '__main__':
    main()
