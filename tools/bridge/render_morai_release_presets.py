#!/usr/bin/env python3
"""Render the user-facing release/transport presets from the shared contracts."""
from __future__ import annotations
import argparse
import json
import re
from pathlib import Path
from render_morai_network_profiles import load_contract, render_profile, rendered_bytes, REPO_ROOT
from render_morai_sensor_profiles import render_sensor_profile, iter_udp_configs

CATALOG = REPO_ROOT / 'config/morai/catalog.json'
VERSIONS = {'26r1': '26.R1.H3', 'molitcomp03': '25.S4.MolitComp03'}
TRANSPORTS = {'local': 'Local', 'lan2lan': 'LAN2LAN'}


def render_catalog(contract, catalog):
    date = catalog['revision_date']
    if not re.fullmatch(r'\d{6}', date):
        raise ValueError('Preset revision date must be YYMMDD')
    if set(catalog['versions']) != set(VERSIONS) or catalog['versions'] != VERSIONS:
        raise ValueError('Catalog release/version mapping does not match the verified versions')
    result = {}
    for release, version in VERSIONS.items():
        for transport, title in TRANSPORTS.items():
            for kind in ('Network', 'Sensor'):
                filename = ('network/' if kind == 'Network' else 'sensors/') + f'{date}_ASMC_{title}_{kind}_{version}.json'
                document = (render_profile(contract, release, transport) if kind == 'Network'
                            else render_sensor_profile(contract, 'v3', transport))
                entry = catalog['files'].get(filename, {})
                expected = {'release': release, 'version': version, 'transport': transport,
                            'kind': kind, 'sensor_set': None if kind == 'Network' else 'v3'}
                if any(entry.get(key) != value for key, value in expected.items()):
                    raise ValueError('Mismatched release/transport metadata: ' + filename)
                result[filename] = rendered_bytes(document)
        # GT is an explicit local development preset; it is not a competition LAN input.
        filename = 'sensors/' + f'{date}_ASMC_Local_SensorGT_{version}.json'
        gt = json.loads((REPO_ROOT / contract['sensor_sets']['GT']['path']).read_text())
        for udp in iter_udp_configs(gt):
            if (udp['hostIP'], udp['destinationIP']) != ('127.0.0.1', '127.0.0.1'):
                raise ValueError('GT source must remain local-only')
        entry = catalog['files'].get(filename, {})
        expected_gt = {'release': release, 'version': version, 'transport': 'local',
                       'kind': 'SensorGT', 'sensor_set': 'GT'}
        if any(entry.get(key) != value for key, value in expected_gt.items()):
            raise ValueError('GT presets must be local and release-specific')
        result[filename] = rendered_bytes(gt)
    if set(catalog['files']) != set(result):
        raise ValueError('Catalog filenames do not match YYMMDD_ASMC_Transport_Kind_Version')
    return result


def check_all(contract=None, catalog=None):
    contract = contract or load_contract()
    catalog = catalog or json.loads(CATALOG.read_text())
    for name, expected in render_catalog(contract, catalog).items():
        path = CATALOG.parent / name
        if not path.is_file() or path.read_bytes() != expected:
            raise ValueError('Missing or stale versioned MORAI preset: ' + name)
    print('Versioned MORAI presets: 8 runtime + 2 local GT; names and contents current')


def write_all(contract, catalog, output_root=None):
    root = CATALOG.parent if output_root is None else Path(output_root)
    for name, content in render_catalog(contract, catalog).items():
        output = root / name
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_bytes(content)
        print('wrote ' + name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument('--write', action='store_true')
    action.add_argument('--check', action='store_true')
    args = parser.parse_args()
    contract = load_contract()
    catalog = json.loads(CATALOG.read_text())
    if args.check:
        check_all(contract, catalog)
    else:
        write_all(contract, catalog)


if __name__ == '__main__':
    main()
