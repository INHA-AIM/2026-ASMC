#!/usr/bin/env python3
"""Explicit integration check; both standalone C++ binaries are required, no skips.

Usage: python3 test_legacy_probes.py /tmp/lidar-a1-probe-build
Creates only temporary unit-test outputs. Never reads a bag or publishes ROS.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('bin_dir',type=Path)
args=parser.parse_args()
repo=Path(__file__).resolve().parents[3]
script=repo/'src/asmc_lidar/tools/compare_hd_map_filters.py'
fixture=repo/'src/asmc_lidar/test/fixtures/hd_map_filter_audit/cases.jsonl'
config=script.parent/'filter_audit/audit.yaml'
with tempfile.TemporaryDirectory(prefix='lidar-a1-contract-') as tmp:
    summaries=[]
    for n in (1,2):
        destination=Path(tmp)/str(n)
        subprocess.run([sys.executable,str(script),'--run-id','unit-'+str(n),'--config',str(config),
                        '--input',str(fixture),'--legacy-bin-dir',str(args.bin_dir),'--output',str(destination)],check=True)
        summaries.append(json.loads((destination/'summary.json').read_text()))
        rows=[json.loads(x) for x in (destination/'decisions.jsonl').read_text().splitlines()]
        assert len(rows)==len(fixture.read_text().splitlines()) and not summaries[-1]['failures']
        assert all(set(r['legacy'])=={'main','planning'} for r in rows)
        assert any(r['legacy']['main'].get('full_main_chain') for r in rows)
        assert any(r['legacy']['main'].get('reason')=='obstacle_confirmation_pending' for r in rows), {r['legacy']['main'].get('reason') for r in rows}
    assert summaries[0]['stable_decisions_sha256']==summaries[1]['stable_decisions_sha256']
    assert summaries[0]['input_objects_sha256']==summaries[1]['input_objects_sha256']
print('PASS: 36 fixtures, both current C++ implementations, complete Main chain and deterministic decisions')
