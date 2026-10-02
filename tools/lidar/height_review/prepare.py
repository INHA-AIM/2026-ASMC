#!/usr/bin/env python3
"""Freeze H1 inputs, or recreate bag snapshots with the frozen historical extractor.

The old extractor tracks the last valid *arriving* IMU (not the complete history).
Keep that conservative selection for parity; do not silently add newly eligible scans.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from contextlib import ExitStack

def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(8*1024*1024), b''):
            h.update(block)
    return h.hexdigest()

def rows(path):
    return [json.loads(x) for x in path.read_text().splitlines() if x]

def dump(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--data', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--legacy', type=Path, help='optional exact old snapshot cache; otherwise reextract')
    a = p.parse_args()
    out = a.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    data = a.data.resolve()
    previous = data / 'lidar_assessment/20261003_l2a2_imu_patchwork_review'
    sources = {}
    for name, old_name, stride in [('manual', 'manual_all', 1), ('stationary', 'stationary', 18)]:
        source = json.loads((previous / old_name / 'source.json').read_text())
        assert source['bag'].startswith('/data/')
        bag_rel = source['bag'][len('/data/'):]
        bag = data / bag_rel
        if sha(bag) != source['bag_sha256']:
            raise ValueError('bag SHA drift: '+str(bag))
        target = out / name
        with ExitStack() as stack:
            if a.legacy:
                cached = a.legacy / old_name
            else:
                cached = Path(stack.enter_context(tempfile.TemporaryDirectory(prefix='h1-reextract-'))) / old_name
                cmd = [sys.executable, str(previous / 'scripts/extract_sequences.py'),
                       '--bag', str(bag), '--output', str(cached), '--stride', str(stride), '--prefix', old_name]
                if name == 'stationary':
                    cmd += ['--scene-manifest', str(data / 'lidar_assessment/20260926/stationary_v2/manifest.json')]
                subprocess.run(cmd, check=True)
            meta = rows(cached / 'snapshots.jsonl')
            excluded = [r['input_id'] for r in meta if r['scan_arrival_ns'] < r['scan_effective_stamp_ns']]
            selected = [r for r in meta if r['input_id'] not in excluded]
            assert len(selected) == (644 if name == 'manual' else 129), (name, len(selected))
            assert excluded == (['manual_all_000258'] if name == 'manual' else [])
            target.mkdir()
            (target / 'clouds').mkdir()
            for row in selected:
                shutil.copy2(cached / row['cloud_path'], target / row['cloud_path'])
        # Synthetic Capture timestamps are never used here; keep every bag ns unchanged.
        (target / 'snapshots.jsonl').write_text(''.join(json.dumps(r, sort_keys=True)+'\n' for r in selected))
        sources[name] = {'bag': bag_rel, 'sha256': source['bag_sha256'], 'bytes': bag.stat().st_size,
                         'selected': len(selected), 'excluded': excluded, 'stride': stride,
                         'selection': 'frozen last-arriving valid IMU; <=100ms; no future arrival/stamp',
                         'extractor_sha256': sha(previous / 'scripts/extract_sequences.py')}
    stage = data / 'lidar_assessment/20260926/ground_benchmark_v1/stage'
    selection = data / 'lidar_assessment/20261002_l2a2_general_actors/route_selection.json'
    selected = {r['frame'] for r in json.loads(selection.read_text())['records'] if r['admitted']}
    manifest = json.loads((stage / 'manifest.json').read_text())
    capture = out / 'capture'; capture.mkdir()
    records = []
    samples = [s for s in manifest['samples'] if s['frame'] in selected]
    assert len(samples) == 62
    for i, sample in enumerate(samples):
        frame = sample['frame']
        for suffix in ('.bin', '.labels.npz'):
            shutil.copy2(stage / (frame+suffix), capture / (frame+suffix))
        records.append({'input_id': frame, 'capture_key': frame, 'phase': frame,
                        'cloud_path': frame+'.bin', 'roll': 0.0, 'pitch': 0.0,
                        'geometry_valid': True, 'scan_effective_stamp_ns': (i+1)*1000000000,
                        'scan_arrival_ns': (i+1)*1000000000, 'controlled_camera_state': 'missing',
                        'attitude_provenance': 'no_IMU_zero_attitude_legacy_capture_geometry_only',
                        'stamp_provenance': 'synthetic_sequence_index_not_sensor_time'})
    (capture / 'snapshots.jsonl').write_text(''.join(json.dumps(r, sort_keys=True)+'\n' for r in records))
    dump(capture / 'labels.json', samples)
    sources['capture'] = {'frames': 62, 'source_manifest_sha256': sha(stage / 'manifest.json'),
                          'selection_sha256': sha(selection), 'independent_test_set': False,
                          'sensor': 'historical 10Hz profile; not current v3'}
    if a.legacy:
        oracle = out / 'oracle'; oracle.mkdir()
        for src, name in [('manual_all/patch_strict.jsonl', 'manual.jsonl'),
                          ('stationary/patch_pitch180.jsonl', 'stationary.jsonl'),
                          ('capture_patch_baseline.jsonl', 'capture.jsonl'),
                          ('manual_all/compare_strict.jsonl', 'manual_fourway.jsonl')]:
            shutil.copy2(a.legacy / src, oracle / name)
    else:
        # Frozen original outputs are now durable; regeneration never needs /tmp.
        oracle = data / 'lidar_assessment/20261003_l2a2_h1/inputs/oracle'
        shutil.copytree(oracle, out / 'oracle')
    files = {str(f.relative_to(out)): {'sha256': sha(f), 'bytes': f.stat().st_size}
             for f in sorted(out.rglob('*')) if f.is_file()}
    dump(out / 'manifest.json', {'sources': sources, 'files': files,
         'basis_commit': 'cb0ccb97de180f6ddbe5065863d99423c0568799',
         'height_m': 1.80, 'mount_z_m': 1.45, 'measured_level_road_height_approx_m': [1.77,1.78],
         'min_range_m': 1.0, 'max_range_m': 60.0, 'ring0_end_m': 4.6875,
         'rnr': False, 'voxel_m': .10, 'no_Gt_in_algorithm': True})
    print(json.dumps({'output': str(out), 'frames': [644,129,62], 'files': len(files)}))

if __name__ == '__main__':
    main()
