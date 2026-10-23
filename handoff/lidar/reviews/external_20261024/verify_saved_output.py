#!/usr/bin/env python3
"""Verify frozen data and re-evaluate SAVED predictions. Does not run inference."""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import platform
import re
import subprocess
import sys


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def equal(a, b):
    if isinstance(a, float) and isinstance(b, float):
        if math.isnan(a) and math.isnan(b):
            return True
    return a == b


def comparison(a, b):
    exact = equal(a, b)
    error = abs(a - b) if math.isfinite(a) and math.isfinite(b) else None
    # Cross-environment evaluator arithmetic only. Integer counts remain exact.
    # This does NOT change the C++ replay equality gate or EVAL_CONTRACT.json.
    close = exact or (isinstance(a, float) and isinstance(b, float) and
                      error is not None and error <= 1e-12)
    return {'actual': a, 'reference': b, 'exact_equal': exact,
            'absolute_difference': error, 'within_roundoff_tolerance': close}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[4]
    if args.out.exists():
        raise SystemExit('Refusing existing output: ' + str(args.out))
    args.out.mkdir(parents=True)
    manifest = json.loads((repo / 'handoff/lidar/DATA_MANIFEST.json').read_text())
    observed_manifest = json.loads((args.data / 'MANIFEST.json').read_text())
    if manifest != observed_manifest:
        raise SystemExit('Dataset manifest differs from the frozen repository manifest')
    verified = []
    for name, expected in manifest['files'].items():
        path = args.data / name
        actual = {'size': path.stat().st_size, 'sha256': digest(path)}
        if actual != expected:
            raise SystemExit('Input integrity failure: ' + name)
        verified.append({'path': name, **actual})

    tests = []
    for name, count in [('test_alignment.py', 12), ('test_response_eval.py', 11)]:
        script = repo / 'handoff/lidar/evaluation_tests' / name
        command = [sys.executable, '-B', str(script)]
        result = subprocess.run(command, capture_output=True, text=True)
        log = result.stdout + result.stderr
        (args.out / (name + '.log')).write_text(log)
        found = re.search(r'Ran (\d+) tests?', log)
        actual_count = int(found.group(1)) if found else None
        passed = result.returncode == 0 and actual_count == count
        tests.append({'test': name, 'count': actual_count,
                      'returncode': result.returncode, 'passed': passed})

    module_path = repo / 'tools/experiments/lidar_obb_quality_temporal/eval_lib.py'
    spec = importlib.util.spec_from_file_location('frozen_eval', module_path)
    ev = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ev)
    contract = json.loads((module_path.parent / 'EVAL_CONTRACT.json').read_text())
    original_segment = ev.segment_gt

    def segment(gt, frames, c):
        rows = original_segment(gt, frames, c)
        if 'gt_generation' in rows:
            rows['seg'] = rows.seg + '_g' + rows.gt_generation.astype(int).astype(str)
        return rows

    # Identical evaluation-only generation handling to handoff/lidar/run.py.
    ev.segment_gt = segment
    sample = ev.eval_scene(contract, str(args.data / 'evaluation'),
                           str(args.data / 'expected.jsonl'), 'actual_lap', None)
    metrics = ev.compute_metrics([sample], contract)
    reference = json.loads((repo / 'handoff/lidar/evidence/'
                            'DOWNLOADED_PREFIX_EVALUATION.json').read_text())['expected']
    keys = ['n_input_scans', 'tp', 'fn', 'fp', 'velocity_n',
            'velocity_error_mae', 'velocity_error_p95', 'center_mae_vehicle',
            'center_p95_vehicle', 'long_mae_vehicle', 'short_mae_vehicle',
            'yaw_mae_vehicle_deg', 'yaw_p95_vehicle_deg', 'idsw',
            'fragmentations', 'scans_emitted_local', 'scans_dropped']
    checks = {k: comparison(metrics[k], reference[k]) for k in keys}
    event_tp = sample['tp'][sample['tp'].idx.between(1086, 1155)]
    finite = ev.np.isfinite(event_tp[['pvx', 'pvy', 'gvx', 'gvy']]).all(axis=1)
    valid = event_tp[(event_tp.kfc >= 2) & finite]
    errors = ev.np.hypot(valid.pvx - valid.gvx, valid.pvy - valid.gvy)
    event = {'TP': len(event_tp), 'velocity_n': len(valid),
             'velocity_mae': float(errors.mean()), 'velocity_max': float(errors.max())}
    event_checks = {k: equal(v, reference['event_window_1086_1155'][k])
                    for k, v in event.items()}
    exact = all(v['exact_equal'] for v in checks.values()) and all(event_checks.values())
    passed = (all(t['passed'] for t in tests) and
              all(v['within_roundoff_tolerance'] for v in checks.values()) and
              all(event_checks.values()))
    result = {
        'scope': 'Frozen saved-output re-evaluation; NOT a C++/ROS inference replay',
        'reviewed_source_commit': '6c5dba83333c908a8d0d265d64d0f79754523291',
        'data_commit': '446cf839f66e45641274fd4e5669b865108e429d',
        'environment': {'python': platform.python_version(),
                        'numpy': ev.np.__version__, 'pandas': ev.pd.__version__},
        'inputs': verified, 'portable_tests': tests, 'metrics': checks,
        'event_1086_1155': event, 'event_equal': event_checks,
        'all_selected_metrics_exact': exact,
        'floating_metric_absolute_tolerance': 1e-12,
        'discrete_count_tolerance': 0,
        'result': ('EXACT_MATCH' if exact else 'MATCH_WITH_ROUNDOFF') if passed else 'FAIL',
        'not_verified': ['C++ build and unit tests', '1156-scan inference replay',
                         'point-level split/overmerge (no replay points.bin)',
                         'live MORAI', 'full-load real-time operation',
                         'physical correctness of provisional GT']}
    (args.out / 'VERIFICATION.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
