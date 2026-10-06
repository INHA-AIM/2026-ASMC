#!/usr/bin/env python3
"""Export a validated, coherent global HD snapshot for planning file readers."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import numpy as np


def main():
    package = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(package / 'src'))
    from asmc_hd_map.runtime_bundle import load_runtime_bundle
    from asmc_hd_map.planning_export import write_free_grid_rle
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--data-root', type=Path, default=Path(os.environ.get('ASMC_DATA', '/data')))
    args = parser.parse_args()
    map_path = package / 'reports/kcity2025_route_curation_20260917/route_curated_candidate_map.json'
    metadata_path = package / 'config/kcity2025_competition_metadata.yaml'
    product_path = package / 'config/kcity2025_compact_drivable_map.json'
    bundle = load_runtime_bundle(map_path, metadata_path, data_root=args.data_root)
    product = json.loads(product_path.read_text(encoding='utf-8'))
    payload = json.dumps({k: v for k, v in product.items() if k != 'product_sha256'},
                         ensure_ascii=False, sort_keys=True, separators=(',', ':')).encode('utf-8')
    if hashlib.sha256(payload).hexdigest() != product['product_sha256']:
        raise ValueError('compact product self-hash mismatch')
    if product['reviewed_geometry_sha256'] != bundle['geometry_sha256']:
        raise ValueError('mixed vector geometry identities')
    grid_path = product_path.parent / Path(product['grid']['path']).name
    if not grid_path.is_file():
        grid_path = package.parents[1] / product['grid']['path']
    if hashlib.sha256(grid_path.read_bytes()).hexdigest() != product['grid']['sha256']:
        raise ValueError('occupancy SHA mismatch')
    with np.load(grid_path, allow_pickle=False) as archive:
        occupancy = archive['occupancy']
    # Refuse a nonempty destination; consumers replace the whole bundle together.
    out = args.output_dir
    if out.exists() and any(out.iterdir()):
        raise ValueError('output directory must be empty')
    out.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(product_path, out / 'drivable.json')
    shutil.copyfile(map_path, out / 'lines.json')
    shutil.copyfile(metadata_path, out / 'competition_metadata.yaml')
    with (out / 'drivable_grid.rle').open('w', encoding='ascii') as stream:
        write_free_grid_rle(stream, occupancy, product['grid'])
    route = dict(route_id=bundle['metadata']['route']['route_id'],
                 route_sha256=bundle['metadata']['sources']['official_global_path']['sha256'],
                 frame_id='map', station_definition='official_path_xy_arc_length',
                 length_m=bundle['route_stations'][-1], points=bundle['route_points'],
                 stations_m=bundle['route_stations'])
    (out / 'route.json').write_text(json.dumps(route, sort_keys=True) + '\n', encoding='utf-8')
    manifest = dict(build_id=product['build_id'], geometry_sha256=bundle['geometry_sha256'],
                    product_sha256=product['product_sha256'], files={
                        p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.iterdir())})
    (out / 'manifest.json').write_text(json.dumps(manifest, sort_keys=True, indent=2) + '\n', encoding='utf-8')
    print('exported %s: build=%s files=%d' % (out, manifest['build_id'], len(manifest['files'])))


if __name__ == '__main__':
    main()
