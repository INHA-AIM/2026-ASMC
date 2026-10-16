#!/usr/bin/env python3
"""Pin existing six-condition tunnel RGB/pose inputs; does not create lane GT."""
import argparse
from bisect import bisect_left
import hashlib
import json
from pathlib import Path
import numpy as np

TOPICS={'front':'/image_jpeg/compressed','left':'/cam_left/image_jpeg/compressed',
        'right':'/cam_right/image_jpeg/compressed'}


def sha256(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''): h.update(chunk)
    return h.hexdigest()


def nearest_index(stamps, target, maximum_age_ns):
    index=bisect_left(stamps,target)
    candidates=[i for i in (index-1,index) if 0<=i<len(stamps)]
    if not candidates: return None
    chosen=min(candidates,key=lambda i:(abs(stamps[i]-target),i))
    return chosen if abs(stamps[chosen]-target)<=maximum_age_ns else None


def phase(station, start, end):
    return 'entry' if station<start else 'exit' if station>end else 'interior'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--shadow-start',type=float,default=1877.744798966)
    p.add_argument('--shadow-end',type=float,default=1996.730791531)
    a=p.parse_args()
    import rosbag
    from asmc_camera.offline.bev.pose_time_alignment import align_ego_pose
    from asmc_camera.offline.bev.rgb_bev import StampedEgoPose
    a.output.mkdir(parents=True,exist_ok=False)
    route_path=a.data_root/'R_KR_PG_K-city_2025/2026_molit_comp_global_path.txt'
    route=np.loadtxt(route_path)
    stations=np.r_[0,np.cumsum(np.linalg.norm(np.diff(route[:,:2],axis=0),axis=1))]
    runs=sorted((a.data_root/'rosbags/raw').glob('20260912_*shadow_static_avoid_v1*'))
    if len(runs)!=6: raise ValueError('expected the fixed six SUNNY/FOGGY x 11/13/15 runs')
    manifest, summaries=[],[]
    for run in runs:
        bags=sorted((run/'bags').glob('*.bag'))
        if len(bags)!=1: raise ValueError('expected one bag: '+run.name)
        bag_path=bags[0]
        images={v:[] for v in TOPICS}; poses=[]
        with rosbag.Bag(str(bag_path)) as bag:
            for topic,message,recorded in bag.read_messages(topics=list(TOPICS.values())+['/Ego_topic']):
                header_ns=message.header.stamp.to_nsec()
                stamp=header_ns or recorded.to_nsec()
                if topic=='/Ego_topic':
                    poses.append(StampedEgoPose(stamp,message.position.x,message.position.y,message.heading))
                else:
                    view=next(v for v,t in TOPICS.items() if t==topic)
                    images[view].append(dict(topic=topic,source_ns=stamp,header_ns=header_ns,
                        bag_ns=recorded.to_nsec(),stamp_policy='header' if header_ns else 'bag_fallback',
                        frame_id=message.header.frame_id,sha256=hashlib.sha256(bytes(message.data)).hexdigest()))
        for values in images.values(): values.sort(key=lambda x:x['source_ns'])
        rows=[];used={v:set() for v in ('left','right')}; rejected=0
        for front in images['front']:
            selected={'front':front}
            for view in ('left','right'):
                values=images[view]
                i=nearest_index([r['source_ns'] for r in values],front['source_ns'],35000000)
                if i is None or i in used[view]: break
                selected[view]=values[i]
            if len(selected)!=3 or max(r['source_ns'] for r in selected.values())-min(r['source_ns'] for r in selected.values())>35000000:
                rejected+=1; continue
            alignment=align_ego_pose(poses,front['source_ns'],50000000,100000000,'interpolate')
            if alignment.pose is None: rejected+=1; continue
            pose=alignment.pose
            i=int(np.argmin(np.linalg.norm(route[:,:2]-[pose.x_m,pose.y_m],axis=1)))
            station=float(stations[i])
            for view in ('left','right'):
                used[view].add(images[view].index(selected[view]))
            row=dict(run_id=run.name,bag=str(bag_path.relative_to(a.data_root)),
                views=selected,pose=dict(x=pose.x_m,y=pose.y_m,heading_deg=pose.heading_deg),
                alignment=alignment.to_dict(),route_station_m=station,
                phase=phase(station,a.shadow_start,a.shadow_end))
            rows.append(row)
        manifest.extend(rows)
        summaries.append(dict(run_id=run.name,bag=str(bag_path.relative_to(a.data_root)),
            bag_sha256=sha256(bag_path),triplets=len(rows),rejected_front_inputs=rejected,
            phases={k:sum(r['phase']==k for r in rows) for k in ('entry','interior','exit')}))
    if any(not all(s['phases'].values()) for s in summaries):
        raise ValueError('fixture lacks entry/interior/exit coverage')
    path=a.output/'manifest.jsonl'
    path.write_text(''.join(json.dumps(r,sort_keys=True)+'\n' for r in manifest))
    summary=dict(schema='shadow_family_fixture_v1',runs=summaries,triplets=len(manifest),
        manifest_sha256=sha256(path),route_sha256=sha256(route_path),
        shadow_station_bounds=[a.shadow_start,a.shadow_end],
        boundary_source='existing morai_camera_runtime_trial SHADOW_START/END; development ODD fixture',
        note='Existing development bag RGB and interpolated pose. No new GT or capture-time guarantee; keep original header/fallback policy. Adjacent frames are not independent scenes.')
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__': main()
