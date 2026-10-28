#!/usr/bin/env python3
"""Materialize the pinned official route for the HD-map handover trial.

The repository route may have Git-normalized line endings. Only a byte sequence
matching the metadata's original SHA-256 and size is accepted; coordinates,
metadata pins and the map publisher are never modified.
"""
import argparse
import hashlib
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[2]


def prepare(repo_root, data_root, source_path=None):
    repo_root = Path(repo_root).resolve()
    data_root = Path(data_root).expanduser().resolve()
    metadata = yaml.safe_load((repo_root / 'src/asmc_hd_map/config/kcity2025_competition_metadata.yaml').read_text(encoding='utf-8'))
    source = metadata['sources']['official_global_path']
    prefix = '$ASMC_DATA/'
    if not source['path'].startswith(prefix):
        raise ValueError('official route must be below $ASMC_DATA')
    relative = Path(source['path'][len(prefix):])
    if relative.is_absolute() or '..' in relative.parts:
        raise ValueError('invalid official-route relative path')
    target = data_root / relative
    expected, size = source['sha256'], int(source['bytes'])

    def matches(raw):
        return len(raw) == size and hashlib.sha256(raw).hexdigest() == expected

    def existing():
        if not target.is_file() or not matches(target.read_bytes()):
            raise ValueError('existing destination differs from the official route; preserved: %s' % target)
        return target, expected, 'already verified'

    if target.exists() or target.is_symlink():
        return existing()
    if source_path is None:
        config = yaml.safe_load((repo_root / 'src/asmc_planner/config/frenet/params.yaml').read_text(encoding='utf-8'))
        source_path = Path(config['waypoint_file']).expanduser()
        if not source_path.is_absolute():
            # Same relative-path rule as FrenetPlanner::Init.
            source_path = repo_root / 'src/asmc_planner/src' / source_path
    raw = Path(source_path).expanduser().read_bytes()
    lf = raw.replace(b'\r\n', b'\n')
    # Accept only exact pinned bytes, including the original EOF convention.
    candidates = (raw, lf, lf.replace(b'\n', b'\r\n'),
                  lf.rstrip(b'\n'), lf.rstrip(b'\n').replace(b'\n', b'\r\n'))
    payload = next((value for value in candidates if matches(value)), None)
    if payload is None:
        raise ValueError('source cannot reproduce the pinned official route; provide the official original with --source')
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        stream = target.open('xb')  # Never overwrite another dataset.
    except FileExistsError:
        return existing()
    try:
        with stream:
            stream.write(payload)
    except BaseException:
        target.unlink()
        raise
    return target, expected, 'prepared and verified'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', type=Path, default=Path('/data'))
    parser.add_argument('--source', type=Path, help='Optional official route file; default: configured Frenet waypoint file')
    args = parser.parse_args()
    try:
        target, digest, status = prepare(ROOT, args.data_root, args.source)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        print('ERROR: %s' % error, file=sys.stderr)
        return 1
    print('%s: %s\nSHA-256: %s' % (status, target, digest))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
