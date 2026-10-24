#!/usr/bin/env python3
"""Export a verified A1 map into immutable C++ input arrays (not a new map authority).

Use a registered storage run for --output. Refuses existing output directories.
Source paths stay provenance; the runtime needs only this manifest and two arrays.
"""
import argparse
import hashlib
import json
from pathlib import Path
import yaml
from compare_hd_map_filters import MapAudit, REPO, canonical, sha256


def prepare(config, output):
    cfg = yaml.safe_load(Path(config).read_text())
    args = {k: REPO / cfg[k] for k in ('product', 'grid', 'metadata', 'source')}
    audit = MapAudit(**args)
    if cfg['expected_map_id'] != audit.product['map_id']:
        raise ValueError('expected_map_id_mismatch')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    arrays = {}
    for name, values in [('occupancy', audit.occupancy.astype('int8')), ('exceptions', audit.exceptions)]:
        target = output / (name + '.bin')
        target.write_bytes(values.tobytes(order='C'))
        arrays[name] = {'file': target.name, 'sha256': sha256(target), 'bytes': target.stat().st_size}
    policy = {k: cfg[k] for k in ('expected_map_id', 'map_origin', 'pose_age_limit_ms',
                                'candidate_guard_m', 'exception_protection_m')}
    manifest = {'schema_version': 'asmc.lidar.hd_map_audit_bundle/v1',
                'frame_id': 'map', 'map_id': audit.product['map_id'],
                'map_origin': cfg['map_origin'], 'height': audit.occupancy.shape[0],
                'width': audit.occupancy.shape[1], 'resolution_m': audit.resolution,
                'origin_xy': audit.origin.tolist(), 'arrays': arrays,
                'policy': policy, 'policy_sha256': hashlib.sha256(canonical(policy).encode()).hexdigest(),
                'policy_canonical': canonical(policy), 'source_sha256': audit.shas,
                'source_paths': audit.paths, 'missions': audit.mission_audit,
                'mission_scope_verified': False,
                'note': 'Audit only. No position bound or external approach approval.'}
    target = output / 'manifest.json'
    target.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    return {'manifest': str(target), 'manifest_sha256': sha256(target)}

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', required=True)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    print(json.dumps(prepare(args.config, args.output)))
