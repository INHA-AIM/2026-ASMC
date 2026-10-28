#!/usr/bin/env python3
"""Export/check the single committed HD-map product used by every planner mode."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/asmc_hd_map/src'))
from asmc_hd_map.planning_export import write_free_grid_rle


def export_bundle(root, check=False):
    root = Path(root)
    source = root / 'src/asmc_hd_map/config/kcity2025_compact_drivable_map.json'
    payload = source.read_bytes()
    product = json.loads(payload)
    if product['build_id'] != 'kcity2025_reviewed_high_speed_adjacent_drivable_v7':
        raise ValueError('expected the reviewed v7 HD-map product')
    core = {k: v for k, v in product.items() if k != 'product_sha256'}
    digest = hashlib.sha256(json.dumps(core, ensure_ascii=False, sort_keys=True,
                                      separators=(',', ':')).encode()).hexdigest()
    if digest != product['product_sha256']:
        raise ValueError('HD-map product digest mismatch')
    grid = product['grid']
    grid_path = root / grid['path']
    if hashlib.sha256(grid_path.read_bytes()).hexdigest() != grid['sha256']:
        raise ValueError('HD-map NPZ digest mismatch')
    with np.load(str(grid_path), allow_pickle=False) as archive:
        if (float(archive['resolution_m']) != grid['resolution_m'] or
                not np.array_equal(archive['origin_xy'], [grid['origin_x'], grid['origin_y']]) or
                list(archive['frame_id']) != [product['frame_id']]):
            raise ValueError('HD-map NPZ frame/origin/resolution mismatch')
        stream = io.StringIO()
        stream.write('# asmc.drivable_grid/v1 %s %s\n' %
                     (product['product_sha256'], grid['sha256']))
        write_free_grid_rle(stream, archive['occupancy'], grid)
        raster = stream.getvalue().encode()
    destination = root / 'src/asmc_planner/config/frenet/hd_map'
    outputs = {'drivable.json': payload, 'drivable_grid.rle': raster}
    for name, data in outputs.items():
        path = destination / name
        if check:
            if not path.is_file() or path.read_bytes() != data:
                raise ValueError('planner HD-map bundle is stale: %s' % name)
        else:
            destination.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
    return product['build_id']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    try:
        identity = export_bundle(ROOT, args.check)
    except (OSError, KeyError, ValueError) as exc:
        parser.exit(1, 'HD-MAP FAILED: %s\n' % exc)
    print('HD-MAP OK: %s; vector and raster identical for every planner mode' % identity)


if __name__ == '__main__':
    main()
