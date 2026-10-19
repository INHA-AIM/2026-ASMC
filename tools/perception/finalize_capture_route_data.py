#!/usr/bin/env python3
"""Preserve incomplete Capture records and export only independently verified GT."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/data_collection/src'))
from data_collection.capture_sync import (CaptureSyncError, load_successful_records,
                                         preflight_run, sync_run)
from data_collection.data_locations import resolve_sensor_root
from data_collection.storage_paths import default_data_root, validate_bulk_output


def classify_records(sensor_root, records):
    accepted, excluded = [], []
    for row in records:
        try:
            preflight_run(sensor_root, [row], profile='bbox3_partial')
            accepted.append(row)
        except (CaptureSyncError, OSError, ValueError) as exc:
            excluded.append(dict(sequence=row['sequence'], custom_name=row['custom_name'], reason=str(exc)))
    return accepted, excluded


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-id', required=True)
    p.add_argument('--data-root', type=Path, default=Path(default_data_root()))
    p.add_argument('--sensor-root', type=Path, help='Override configured native SensorData roots')
    p.add_argument('--ledger', type=Path, required=True, help='Independently reviewed placement ledger')
    p.add_argument('--layout-id', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--gallery', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise ValueError('choose a new immutable export version')
    validate_bulk_output(a.output)
    validate_bulk_output(a.gallery)
    run = a.data_root/'capture_runs'/a.run_id
    validation = json.loads((run/'validation.json').read_text())
    if validation.get('exit_reason') != 'route_complete' or not validation.get('tracking_passed'):
        raise ValueError('full-route tracking has not passed; preserve raw run for diagnosis')
    restoration = json.loads((run/'restoration.json').read_text())
    if not all(restoration.get(k) is True for k in ('stopped', 'gear_matches', 'mode_matches')):
        raise ValueError('stop/actuator restoration has not passed')
    if json.loads((run/'summary.json').read_text()).get('failed') != 0:
        raise ValueError('Capture RPC failure requires diagnosis before finalization')
    records = load_successful_records(run/'capture_manifest.jsonl')
    a.sensor_root = resolve_sensor_root(a.sensor_root, records)
    accepted, excluded = classify_records(a.sensor_root, records)
    if not accepted:
        raise ValueError('no structurally complete RGB/GT records')
    # Write outside root-owned runtime logs; source data and prior decisions stay intact.
    evidence = a.output.with_name(a.output.name+'-preflight.json')
    report = dict(run_id=a.run_id, profile='bbox3_partial',
        source_manifest_sha256=hashlib.sha256((run/'capture_manifest.jsonl').read_bytes()).hexdigest(),
        accepted_capture_records=len(accepted), excluded_capture_records=excluded,
        missing_bbox_policy='preserve view; quarantine in export; never synthesize empty GT')
    if evidence.exists() and json.loads(evidence.read_text()) != report:
        raise ValueError('existing preflight evidence differs; choose a new revision')
    evidence.parent.mkdir(parents=True, exist_ok=True)
    evidence.write_text(json.dumps(report, indent=2)+'\n')
    sync_run(a.sensor_root, a.data_root/'capture_runs', a.data_root/'datasets', a.run_id,
             jobs=1, excluded_sequences={r['sequence'] for r in excluded}, profile='bbox3_partial')
    dataset = a.data_root/'datasets'/a.run_id
    components = a.output.with_name(a.output.name+'-components.jsonl')
    subprocess.run([sys.executable, str(ROOT/'tools/perception/resolve_capture_route_components.py'),
        '--run', str(dataset), '--ledger', str(a.ledger), '--output', str(components)], check=True)
    subprocess.run([sys.executable, str(ROOT/'tools/perception/export_capture_route_bbox.py'),
        '--run', str(dataset), '--ledger', str(components), '--output', str(a.output),
        '--gallery', str(a.gallery), '--layout-id', a.layout_id], check=True)


if __name__ == '__main__':
    main()
