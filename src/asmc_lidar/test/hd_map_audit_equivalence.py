#!/usr/bin/env python3
"""Compare C++ against A1, including raster boundary semantics. No ROS needed."""
import argparse
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import numpy as np
import yaml

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))
from compare_hd_map_filters import MapAudit, REPO, normalize, sha256
from prepare_hd_map_audit import prepare


def run(probe, inputs):
    cfg_path = TOOLS / 'filter_audit/audit.yaml'
    cfg = yaml.safe_load(cfg_path.read_text())
    audit = MapAudit(*(REPO / cfg[k] for k in ('product', 'grid', 'metadata', 'source')))
    counts = {}; times = []; cases = 0
    with tempfile.TemporaryDirectory(prefix='lidar-hd-map-audit-') as td:
        bundle = prepare(cfg_path, Path(td) / 'bundle')
        p = subprocess.Popen([str(probe), bundle['manifest'], bundle['manifest_sha256']],
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        try:
            for source in inputs:
                for number, line in enumerate(Path(source).open(), 1):
                    frame = normalize(json.loads(line), Path(source).name, number, cfg)
                    for obj in frame['objects']:
                        expected = audit.evaluate(frame, obj, cfg)
                        p.stdin.write(json.dumps({'frame': frame, 'object': obj}, allow_nan=False) + '\n')
                        p.stdin.flush()
                        value = p.stdout.readline()
                        assert value, 'probe closed unexpectedly'
                        actual = json.loads(value)
                        for key in ('decision', 'reason', 'spatial'):
                            assert actual[key] == expected[key], (source, number, key, expected, actual)
                        assert actual['exception_types'] == expected.get('exception_types', []), (source, number, actual, expected)
                        assert actual['input_state'] == obj.get('observation_state', 'unknown')
                        counts[actual['decision']] = counts.get(actual['decision'], 0) + 1
                        times.append(actual['comparison_ms']); cases += 1
            p.stdin.close()
            assert p.wait(timeout=30) == 0
        finally:
            if p.poll() is None:
                p.kill(); p.wait()
        zipped = prepare(cfg_path, Path(td) / 'compressed', compressed=True)
        request=json.dumps({'frame':frame,'object':obj},allow_nan=False)+'\n'
        result=subprocess.run([str(probe),zipped['manifest'],zipped['manifest_sha256']],input=request,capture_output=True,text=True,check=True)
        compressed=json.loads(result.stdout)
        for key in ('decision','reason','spatial'):assert compressed[key]==actual[key],key
        compressed_array=Path(td)/'compressed/occupancy.bin.gz'
        with compressed_array.open('r+b') as stream:stream.seek(15);stream.write(b'\xff')
        bad=subprocess.run([str(probe),zipped['manifest'],zipped['manifest_sha256']],capture_output=True,text=True)
        assert bad.returncode!=0 and 'audit_array_sha_mismatch' in bad.stderr
        # Manifest pinning and array corruption must be detected at preparation.
        bad = subprocess.run([str(probe), bundle['manifest'], '0' * 64], capture_output=True, text=True)
        assert bad.returncode != 0 and 'audit_manifest_sha_mismatch' in bad.stderr
        array = Path(td) / 'bundle/occupancy.bin'
        with array.open('r+b') as out:
            out.seek(10); out.write(b'\x01')
        bad = subprocess.run([str(probe), bundle['manifest'], bundle['manifest_sha256']], capture_output=True, text=True)
        assert bad.returncode != 0 and 'audit_array_sha_mismatch' in bad.stderr
    return {'objects': cases, 'counts': counts, 'exact_python_cpp_decisions': True,
            'array_and_manifest_sha_checks': True,
            'cpp_object_ms': {str(q): float(np.percentile(times, q)) for q in (50, 95, 99)} if times else {}}

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--probe', type=Path, required=True)
    p.add_argument('--input', action='append')
    args = p.parse_args()
    inputs = args.input or [Path(__file__).parent / 'fixtures/hd_map_filter_audit/cases.jsonl']
    print(json.dumps(run(args.probe, inputs), indent=2))
