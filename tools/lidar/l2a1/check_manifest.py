#!/usr/bin/env python3
"""Verify frozen L2-A1 code, sensor configuration and replay input hashes."""
import argparse
import hashlib
from pathlib import Path
import sys

import yaml


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo-root', type=Path, required=True)
    parser.add_argument('--data-root', type=Path, required=True)
    args = parser.parse_args()
    root = args.repo_root.resolve()
    manifest = yaml.safe_load((root / 'config/baselines/lidar_l2a1_runtime_20261002.yaml').read_text())
    files = {
        'reference.tunnel_core_header_sha256': root / 'src/asmc_lidar/vendor/jaeho/Lidar/TunnelFilter/TunnelFilterCore.hpp',
        'reference.tunnel_core_source_sha256': root / 'src/asmc_lidar/vendor/jaeho/Lidar/TunnelFilter/TunnelFilterCore.cpp',
        'reference.coordinate_time_core_header_sha256': root / 'tools/lidar/frame_time/core.hpp',
        'reference.coordinate_time_core_source_sha256': root / 'tools/lidar/frame_time/core.cpp',
        'patchwork_plus_plus.core_header_sha256': root / 'src/asmc_lidar/vendor/patchwork/include/patchwork/patchworkpp.h',
        'patchwork_plus_plus.core_source_sha256': root / 'src/asmc_lidar/vendor/patchwork/src/patchworkpp.cpp',
        'patchwork_plus_plus.plane_header_sha256': root / 'src/asmc_lidar/vendor/patchwork/include/patchwork/plane_fit.h',
        'patchwork_plus_plus.plane_source_sha256': root / 'src/asmc_lidar/vendor/patchwork/src/plane_fit.cpp',
        'patchwork_plus_plus.types_header_sha256': root / 'src/asmc_lidar/vendor/patchwork/include/patchwork/types.h',
        'sensor.sha256': root / manifest['sensor']['source'],
        'frozen_capture.input_sha256': args.data_root / manifest['frozen_capture']['input_relative_to_ASMC_DATA'],
        'frozen_capture.reference_sha256': args.data_root / manifest['frozen_capture']['reference_relative_to_ASMC_DATA'],
    }
    bad = []
    for name, path in files.items():
        section, key = name.split('.')
        expected = manifest[section][key]
        actual = sha(path) if path.is_file() else 'missing'
        if actual != expected:
            bad.append(f'{name}: {actual} != {expected} ({path})')
    if bad:
        print('\n'.join(bad), file=sys.stderr)
        return 1
    print(f'validated {len(files)} L2-A1 source and input SHA256 values')
    return 0


if __name__ == '__main__':
    sys.exit(main())
