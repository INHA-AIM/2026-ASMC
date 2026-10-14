#!/usr/bin/env python3
"""Rotate owned obstacles; recreate pedestrian and vehicle actors at the next yaw.

Every stage has an immutable ledger. New actor IDs need independent Capture review.
A failed stage never starts driving or claims rollback of recreated identities.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ledger',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--grpc-host',default='host.docker.internal')
    a=p.parse_args()
    a.output_dir.mkdir(parents=True,exist_ok=False)
    rows=[json.loads(l) for l in a.ledger.read_text().splitlines() if l.strip()]
    active=[r for r in rows if r.get('created') and not r.get('removed')]
    if not active or any(not r.get('geometry_gate') for r in active):
        raise ValueError('numerically verified owned layout required')
    # Vehicle SetTransform also showed deferred motion in the native stability
    # check. Fresh static spawn reuses the measured support alignment path.
    ped=[r for r in active if r['actor_type'] in ('pedestrian','vehicle')]
    obstacles=[r for r in active if r['actor_type']=='obstacle']
    def save(name,value):
        path=a.output_dir/name
        path.write_text(''.join(json.dumps(r)+'\n' for r in value))
        return path
    def run(tool,*args):
        subprocess.run([sys.executable,str(Path(__file__).with_name(tool)),*map(str,args),'--grpc-host',a.grpc_host],check=True)
    # Validate ownership through the existing cleanup tool, including moved actors.
    cleanup=save('respawn-cleanup.jsonl',ped)
    plan=[]
    keys=('slot','station_m','target_class','api_name','actor_type','route_surface_z','location','height_basis','route_sha256','station_phase','side_phase','allocation_policy_sha256')
    for r in ped:
        n={k:r[k] for k in keys if k in r}
        n.update(yaw_deg=(r['yaw_deg']+60)%360,created=False,placement_approved=False,identity_approved=False,native_ids=[],
                 recreated_from_actor_id=r['id'],recreation_reason='native SetTransform stability failure; fresh static spawn at next orientation')
        plan.append(n)
    fresh=save('respawn-new.jsonl',plan)
    original_obstacles=save('obstacles-before.jsonl',obstacles)
    # Pending RGB review does not waive numeric geometry or ownership checks.
    for r in obstacles:
        if not r.get('placement_approved'):
            raise ValueError('obstacle positions require existing RGB approval before rotation')
    run('capture_route_layout.py','cleanup','--ledger',cleanup)
    rotated=a.output_dir/'obstacles-rotated.jsonl'
    run('rotate_capture_route_layout.py','--ledger',original_obstacles,'--output',rotated,'--degrees',60,'--pause-static')
    run('capture_route_layout.py','create','--ledger',fresh,'--pause-static')
    new=[json.loads(l) for l in fresh.read_text().splitlines() if l.strip()]
    obs=[json.loads(l) for l in rotated.read_text().splitlines() if l.strip()]
    combined=save('combined.jsonl',sorted(obs+new,key=lambda r:r['slot']))
    print(json.dumps(dict(output=str(combined),recreated=sum(r.get('created') and not r.get('removed') for r in new),rotated=len(obs),new_identity_rgb_review_pending=True)))

if __name__=='__main__':main()
