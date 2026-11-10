"""Check the authoritative workspace registry before append-only trial output."""
import json
import os
from pathlib import Path


def storage_minimum_free_bytes():
    workspace=Path(__file__).resolve().parents[2].parent
    policy=json.loads((workspace/'.storage/policy.json').read_text())
    value=int(policy.get('minimum_free_bytes',10*2**30))
    if value<=0:raise RuntimeError('invalid authoritative storage floor')
    return value


def active_registration(root):
    root=Path(root)
    record=json.loads((root/'registration.json').read_text())
    workspace=Path(__file__).resolve().parents[2].parent
    current=json.loads((workspace/'.storage/runs'/record['run_id']/'run.json').read_text())
    if current['status']!='active' or any(current[k]!=record[k] for k in ('run_id','output','config_sha256')):
        raise RuntimeError('registered run is frozen or registration identity differs')
    if os.statvfs(root).f_bavail*os.statvfs(root).f_frsize<storage_minimum_free_bytes():
        raise RuntimeError('storage free space below authoritative minimum')
    return current
