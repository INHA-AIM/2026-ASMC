"""Offline native-box clearance. Evaluation only; never a planner input.

MORAI vehicle/prop reports transverse extent.x and longitudinal extent.y;
pedestrian native boxes keep Unity's vertical extent.y, forward extent.z.
The same yaw
convention used for rear-axle vehicle footprints is applied to all eight box
corners. This is a conservative SDK box reference, not an identified mesh.
"""
import argparse
import json
import math
from pathlib import Path
from .analyze_local import quantiles


def footprint(state):
    import numpy as np
    from shapely.geometry import MultiPoint
    e=state['box_extents'];center=state['box_center']
    if len(e)!=3 or any(not math.isfinite(v) or v<=0 for v in e):
        raise ValueError('finite positive native box required')
    r,p,y=map(math.radians,state.get('box_rotation',[state['pose'][k] for k in ('roll','pitch','yaw')]))
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    R=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
    forward,vertical=(e[2],e[1]) if state.get('kind')==2 else (e[1],e[2])
    corners=[R@np.array([x,z,h])+center for x in (-forward,forward) for z in (-e[0],e[0]) for h in (-vertical,vertical)]
    return MultiPoint([p[:2] for p in corners]).convex_hull


def analyze(root):
    root=Path(root)
    events=[json.loads(l) for l in (root/'events.jsonl').open()]
    fixed=next((e.get('fixture_states',[]) for e in events if e.get('event')=='fixture_ready'),[])
    fixed=[s for s in fixed if s['kind']==3 and s['speed_kmh']<.1]
    rows=[];omitted=0
    for e in events:
        if e.get('event')!='native_sample':continue
        actors=e.get('actor_states',fixed)
        if not actors:omitted+=1;continue
        own=footprint(e['ego'])
        for other in actors:
            body=footprint(other)
            d=own.distance(body)
            # Focus percentile on actual encounter; far approach cannot hide
            # a zero-gap pass in a long route's clearance distribution.
            if d<=5.:
                rows.append(dict(time=e['ros_time'],actor=other['id'],gap_m=d,
                                 overlap_area_m2=own.intersection(body).area,
                                 speed_kph=e['ego']['speed_kmh']))
    return dict(run=root.name,scope='SDK box hull XY, roll/pitch included; static fallback only. Not mesh/contact certification.',
                encounter_gap_m=quantiles([r['gap_m'] for r in rows]),
                minimum_gap_m=min((r['gap_m'] for r in rows),default=None),
                frames_below_15cm=sum(r['gap_m']<.15 for r in rows),
                overlapping_box_frames=sum(r['overlap_area_m2']>1e-5 for r in rows),
                missing_actor_frames=omitted,series=rows)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args()
    with a.output.open('x') as f:json.dump(analyze(a.root),f,indent=2)
