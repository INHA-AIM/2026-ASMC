#!/usr/bin/env python3
"""Read-only local/map rigid consistency and causal Ego pose comparison.

Ego is an audit reference only. No operational pose or object GT is injected.
Nearest preceding Ego introduces a recorded age and cannot prove full accuracy.
"""
import argparse
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
import numpy as np

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observations',type=Path,required=True)
    parser.add_argument('--bag',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    import rosbag
    ego=[]
    with rosbag.Bag(str(args.bag)) as bag:
        for _,m,_ in bag.read_messages(topics=['/Ego_topic']):
            ego.append(dict(stamp_ns=m.header.stamp.to_nsec(),position=[m.position.x,m.position.y,m.position.z],heading_deg=m.heading))
    ego.sort(key=lambda row:row['stamp_ns']);stamps=[row['stamp_ns'] for row in ego]
    observed=json.loads(args.observations.read_text())
    local={r['stamp_ns']:r for r in observed['outputs']['typed_local']}
    rows=[];unavailable=0;degenerate=0
    for target in observed['outputs']['typed_map']:
        source=local.get(target['stamp_ns']);index=bisect_right(stamps,target['stamp_ns'])-1
        if source is None or source['status']!=1 or target['status']!=1 or index<0:
            unavailable+=1;continue
        reference=ego[index];age=(target['stamp_ns']-reference['stamp_ns'])/1e9
        if age>.12:unavailable+=1;continue
        objects={o['id']:o for o in source['objects']}
        pair=next(((objects[o['id']],o) for o in target['objects'] if o['id'] in objects and
            min(o['dimensions'])>1e-5),None)
        if pair is None:degenerate+=1;continue
        x,y=(np.array(o['corners']) for o in pair)
        xc,yc=x.mean(axis=0),y.mean(axis=0)
        u,s,vt=np.linalg.svd((x-xc).T@(y-yc))
        if s[1]<1e-10:degenerate+=1;continue
        rotation=vt.T@np.diag([1.,1.,np.linalg.det(vt.T@u.T)])@u.T
        translation=yc-rotation@xc
        residual=float(np.max(np.linalg.norm(x@rotation.T+translation-y,axis=1)))
        delta=translation-np.array(reference['position'])
        yaw=math.atan2(rotation[1,0],rotation[0,0]);heading=math.radians(reference['heading_deg'])
        rows.append(dict(stamp_ns=target['stamp_ns'],ego_age_sec=age,
            rigid_residual_m=residual,derived_map_from_base_translation=translation.tolist(),
            ego_reference_position=reference['position'],delta_m=delta.tolist(),
            xy_error_m=float(np.linalg.norm(delta[:2])),z_error_m=float(abs(delta[2])),
            yaw_error_deg=abs(math.degrees(math.atan2(math.sin(yaw-heading),math.cos(yaw-heading))))))
    metrics={key:{'p50':float(np.quantile([r[key] for r in rows],.5)),
        'p95':float(np.quantile([r[key] for r in rows],.95)),
        'maximum':max(r[key] for r in rows)} for key in
        ('rigid_residual_m','xy_error_m','z_error_m','yaw_error_deg','ego_age_sec')} if rows else {}
    result=dict(samples=len(rows),unavailable=unavailable,degenerate=degenerate,metrics=metrics,rows=rows,
        observation_sha256=hashlib.sha256(args.observations.read_bytes()).hexdigest(),
        tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),reference_bag=str(args.bag),
        reference='causal nearest preceding Ego position assumed rear-axle map; heading assumed ENU degrees',
        scope='rigid consistency and pose-convention audit only; no object recall, absolute accuracy guarantee or default promotion')
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'}))
    return 0 if rows and max(r['rigid_residual_m'] for r in rows)<1e-8 else 1

if __name__=='__main__':raise SystemExit(main())
