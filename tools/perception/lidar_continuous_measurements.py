"""Strict continuous-state comparison, with incomplete trials kept as failures."""
import math


def table(rows):
    previous=0
    for row in rows:
        stamp=row['stamp_ns']
        if stamp<=previous:raise ValueError('zero, duplicate or regressing stamp')
        previous=stamp
        if not row['ready'] and row['objects']:raise ValueError('invalid observation with objects')
        ids=[o['id'] for o in row['objects']]
        if len(ids)!=len(set(ids)):raise ValueError('duplicate track IDs')
        for o in row['objects']:
            values=o['center']+o.get('kf_ca',[])+o.get('obb',[])+[v for c in o.get('corners',[]) for v in c]
            if not all(math.isfinite(x) for x in values):raise ValueError('nonfinite geometry or state')
    return {x['stamp_ns']:x for x in rows}


def compare(a,b):
    ta,tb=table(a),table(b)
    common=ta.keys()&tb.keys()
    changed=[ns for ns in sorted(common) if ta[ns]!=tb[ns]]
    missing=sorted(ta.keys()-tb.keys());extra=sorted(tb.keys()-ta.keys())
    return dict(exact=not changed and not missing and not extra,
        common_frames=len(common),changed_ns=changed,missing_ns=missing,extra_ns=extra)


def speed_events(rows):
    """Diagnostic changes only; neither stability nor KF output is speed GT."""
    previous={};events=[]
    for row in rows:
        if row.get('event')!='tracking_state':continue
        dt=row['source_dt_sec']
        for o in row['objects']:
            s=o['semantic'];key=s['id'];before=previous.get(key)
            if before:
                old,old_dt=before
                v=o['kf_ca'][2:4];old_v=old['kf_ca'][2:4]
                jump=math.hypot(v[0]-old_v[0],v[1]-old_v[1])
                if jump>2:
                    events.append(dict(stamp_ns=row['stamp_ns'],id=key,jump_mps=jump,
                        dt_sec=dt,kf_init_count=o['kf_init_count'],age=o['age'],misses=s['lidar_misses'],
                        center_delta_m=math.dist(s['center'],old['semantic']['center']),
                        cause=('initialization transition candidate' if o['age']<=2 or
                            o['kf_init_count']!=old['kf_init_count'] else
                            'requires geometry/association/attitude review')))
            previous[key]=(o,dt)
    return events
