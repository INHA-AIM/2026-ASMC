#!/usr/bin/env python3
"""Create a new audit-preview YAML; does not enable ROS object exclusion."""
import argparse
import hashlib
from pathlib import Path
import yaml
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--template',required=True);p.add_argument('--manifest',required=True);p.add_argument('--output',required=True)
a=p.parse_args();output=Path(a.output)
if output.exists():raise SystemExit('output exists; choose a new profile path')
manifest=Path(a.manifest);cfg=yaml.safe_load(Path(a.template).read_text())
cfg.setdefault('hd_map_audit',{}).update(mode='audit',manifest=str(manifest.resolve()),
    manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest())
with output.open('x') as f:yaml.safe_dump(cfg,f,sort_keys=False)
print(str(output))
