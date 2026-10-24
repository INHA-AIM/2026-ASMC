#!/usr/bin/env python3
"""Refresh/check the small Git deployment bundle derived from asmc_hd_map.

This is a deployment export, not an experiment or a second map authority.
Experiment profiles and completed runs are never overwritten.
"""
import argparse
import re
from pathlib import Path
import tempfile
from prepare_hd_map_audit import prepare

PKG=Path(__file__).resolve().parents[1]

def sync(check=False):
    target=PKG/'config/hd_map_audit'
    config=PKG/'config/lidar_pipeline.yaml'
    with tempfile.TemporaryDirectory(prefix='lidar-map-deploy-') as temporary:
        exported=Path(temporary)/'bundle'
        result=prepare(PKG/'tools/filter_audit/audit.yaml',exported,compressed=True)
        text=config.read_text()
        expected=re.sub(r'(hd_map_audit:\n)  mode: [^\n]+\n  manifest: [^\n]+\n  manifest_sha256: [^\n]+',
            r'\1  mode: "audit"\n  manifest: "config/hd_map_audit/manifest.json"\n  manifest_sha256: "'+result['manifest_sha256']+'"',text)
        if expected==text and 'mode: "audit"' not in text:raise ValueError('default audit block not found')
        files={p.name:p.read_bytes() for p in exported.iterdir()}
        changed=expected!=text or not target.exists() or any(not (target/n).is_file() or (target/n).read_bytes()!=b for n,b in files.items())
        if check and changed:raise ValueError('default HD map bundle is stale; run sync_default_hd_map.py')
        if not check:
            target.mkdir(exist_ok=True)
            for n,b in files.items():(target/n).write_bytes(b)
            config.write_text(expected)
        print('default HD map bundle '+('verified' if check else 'updated')+': '+result['manifest_sha256'])

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true')
    sync(parser.parse_args().check)
