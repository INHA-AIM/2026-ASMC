"""Native evaluation against frozen route segments; never a control pose source."""
import argparse
import hashlib
import json
from pathlib import Path
from .analyze_local import quantiles


def project(route, stations, xy):
    import numpy as np
    a = np.asarray(route, float)[:-1, :2]
    d = np.diff(np.asarray(route, float)[:, :2], axis=0)
    length2 = (d*d).sum(axis=1)
    t = np.clip(((np.asarray(xy)-a)*d).sum(axis=1)/np.maximum(length2, 1e-12), 0., 1.)
    offset = np.asarray(xy) - (a+d*t[:, None])
    distance2 = (offset*offset).sum(axis=1)
    distance2[length2 < 1e-12] = float('inf')
    i = int(distance2.argmin())
    error = float(distance2[i]**.5)
    cross = d[i, 0]*offset[i, 1]-d[i, 1]*offset[i, 0]
    station = float(stations[i]+t[i]*(stations[i+1]-stations[i]))
    return station, error, error if cross >= 0 else -error


def build(reference, output, roots):
    output=Path(output)
    if not (output/'registration.json').is_file():
        raise ValueError('output requires a registered run')
    source=Path(reference)
    ref=json.loads(source.read_text())
    route,station=ref['route_xyz'],ref['route_station']
    # Catch nearest-vertex quantisation before reporting any centimetre metrics.
    a,b=route[0],route[1]
    _,e,_=project(route,station,[(a[0]+b[0])*.5,(a[1]+b[1])*.5])
    if e>1e-8:raise ValueError('mid-segment regression')
    runs=[];omitted=[]
    for root in map(Path,roots):
        events=[json.loads(l) for l in (root/'events.jsonl').open()] if (root/'events.jsonl').exists() else []
        modes=[e for e in events if e.get('event')=='mode']
        samples=[e for e in events if e.get('event')=='native_sample']
        if not samples:
            omitted.append(dict(run=root.name,reason='no_native_drive_samples; preparation failure is not a successful trial'))
            continue
        rows=[]
        from bisect import bisect_right
        times=[m['ros_time'] for m in modes]
        for e in samples:
            pose=e['ego']['pose']; s,error,signed=project(route,station,[pose['x'],pose['y']])
            j=bisect_right(times,e['ros_time'])-1
            rows.append(dict(time=e['ros_time'],x=pose['x'],y=pose['y'],station=s,error=error,signed=signed,
                             speed=e['ego']['speed_kmh'],mode=modes[j]['value'] if j>=0 else None))
        runs.append(dict(run=root.name,series=rows,
            fixtures=next((e.get('fixture_states',[]) for e in events if e.get('event')=='fixture_ready'),[]),
            native_segment_active_error_m=quantiles([r['error'] for r in rows if r['mode']==1]),
            native_segment_final2s_error_m=quantiles([r['error'] for r in rows if r['time']>=rows[-1]['time']-2.])))
    result=dict(route=ref['route'],route_sha256=ref['route_sha256'],route_xyz=route,route_station=station,
        reference_evidence_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),mid_segment_test_passed=True,
        evidence='Frozen route/native evaluation only; active avoidance deviation is intentional, final2s is return metric, not survey GT.',runs=runs,omitted_runs=omitted)
    with (output/'native-segment-review.json').open('x') as f:json.dump(result,f)
    template=Path(__file__).with_name('templates')/'local_route_review.html'
    with (output/'native-review.html').open('x') as f:f.write(template.read_text())


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('reference',type=Path);p.add_argument('output',type=Path);p.add_argument('roots',nargs='+',type=Path)
    a=p.parse_args();build(a.reference,a.output,a.roots)
