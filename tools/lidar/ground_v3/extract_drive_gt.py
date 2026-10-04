#!/usr/bin/env python3
"""Admit continuous UDP GT only with exact XYZ correspondence and causal IMU."""
import argparse
from collections import Counter, defaultdict, deque
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from extract_tunnel_bag import digest, rpy, xyzi


TOPICS = {'/velodyne_points_semantic':'semantic', '/velodyne_points_instance':'instance'}


def field(msg, name, kind):
    found = next((f for f in msg.fields if f.name==name),None)
    if found is None or found.count!=1 or found.datatype != (7 if kind=='<f4' else 6):
        raise ValueError('wrong field '+name)
    if msg.is_bigendian: raise ValueError('unsupported endian')
    return np.ndarray((msg.height,msg.width),dtype=kind,buffer=msg.data,offset=found.offset,
        strides=(msg.row_step,msg.point_step)).reshape(-1).copy()


def geometry(msg):
    return np.column_stack([field(msg,k,'<f4') for k in 'xyz'])


def key(xyz):
    return hashlib.sha256(np.asarray(xyz,dtype='<f4').tobytes()).digest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--max-label-arrival-ms',type=float,default=250)
    a=p.parse_args()
    import rosbag
    meta=json.loads((a.capture/'manifest.json').read_text())
    if not meta.get('complete') or not meta.get('recording_complete'):
        raise ValueError('collection incomplete')
    bags=[a.capture/x['path'] for x in meta['bags']]
    start=round(meta['measurement_start_wall_s']*1e9)
    end=round(meta['measurement_end_wall_s']*1e9)
    tables={k:defaultdict(list) for k in TOPICS.values()}; counts=Counter(); skipped=Counter()
    for bagpath in bags:
        with rosbag.Bag(str(bagpath)) as bag:
            for topic,msg,arrival in bag.read_messages(topics=list(TOPICS)):
                kind=TOPICS[topic]; counts[kind]+=1; xyz=geometry(msg)
                if not np.isfinite(xyz).all():
                    skipped[kind+'_nonfinite']+=1;continue
                tables[kind][key(xyz)].append((arrival.to_nsec(),msg.header.stamp.to_nsec(),
                    field(msg,kind+'_id','<u4'),counts[kind]))
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'clouds').mkdir();(a.output/'labels').mkdir()
    history=deque(maxlen=300);used={'semantic':set(),'instance':set()}
    last_stamp=0; deltas=[]; admitted=0
    with (a.output/'snapshots.jsonl').open('x') as dst:
        for bagpath in bags:
            with rosbag.Bag(str(bagpath)) as bag:
                for topic,msg,arrival in bag.read_messages(topics=['/imu','/velodyne_points']):
                    receipt=arrival.to_nsec(); stamp=msg.header.stamp.to_nsec()
                    if topic=='/imu':
                        q=[getattr(msg.orientation,k) for k in 'xyzw']
                        if stamp>0 and all(math.isfinite(v) for v in q) and .9<sum(v*v for v in q)<1.1:
                            history.append((stamp,receipt,*rpy(msg)))
                        continue
                    counts['intensity']+=1
                    if not start<=receipt<=end:skipped['outside_measurement']+=1;continue
                    if stamp<=0 or stamp<=last_stamp:skipped['invalid_scan_stamp']+=1;continue
                    last_stamp=stamp
                    eligible=[i for i in history if 0<=stamp-i[0]<=100_000_000 and i[1]<=receipt]
                    if not eligible:skipped['no_causal_imu_100ms']+=1;continue
                    attitude=max(eligible,key=lambda x:(x[0],x[1]))
                    cloud=xyzi(msg)
                    if not np.isfinite(cloud).all():skipped['cloud_nonfinite']+=1;continue
                    identity=key(cloud[:,:3]); selected={}
                    for kind in tables:
                        available=[v for v in tables[kind].get(identity,[]) if v[3] not in used[kind]
                            and abs(v[0]-receipt)<=a.max_label_arrival_ms*1e6]
                        if available:selected[kind]=min(available,key=lambda v:(abs(v[0]-receipt),v[0]))
                    if len(selected)!=2:skipped['no_exact_xyz_triplet']+=1;continue
                    for kind,v in selected.items():used[kind].add(v[3])
                    name='drive_'+str(counts['intensity']).zfill(6)
                    rel=Path('clouds')/(name+'.bin');cloud.tofile(a.output/rel)
                    np.savez_compressed(a.output/'labels'/(name+'.npz'),
                        **{kind:v[2] for kind,v in selected.items()})
                    delta={kind:(v[0]-receipt)/1e6 for kind,v in selected.items()};deltas.append(delta)
                    row={'input_id':name,'phase':a.capture.name,'cloud_path':str(rel),
                        'scan_effective_stamp_ns':stamp,'scan_arrival_ns':receipt,
                        'roll':attitude[2],'pitch':attitude[3],'imu_stamp_ns':attitude[0],
                        'imu_arrival_ns':attitude[1],'geometry_valid':True,'measurement':True,
                        'controlled_camera_state':'missing','source_frame':msg.header.frame_id,
                        'gt_xyz_sha256':identity.hex(),'label_receipt_delta_ms':delta,
                        'label_stamp_ns':{k:v[1] for k,v in selected.items()}}
                    dst.write(json.dumps(row,sort_keys=True)+'\n');admitted+=1
    record={'schema':'ground-v3-drive-gt-v1','capture_manifest_sha256':digest(a.capture/'manifest.json'),
        'bags':[{'path':str(f),'sha256':digest(f)} for f in bags],
        'counts':dict(counts),'selected':admitted,'rejected':dict(skipped),
        'matching':'exact SHA256 of identical ordered finite XYZ; one use per label scan; closest receipt within bound',
        'max_label_arrival_ms':a.max_label_arrival_ms,
        'label_abs_arrival_ms_p99':{k:float(np.percentile([abs(v[k]) for v in deltas],99)) if deltas else None for k in tables},
        'snapshots_sha256':digest(a.output/'snapshots.jsonl'),
        'limits':['Receive stamp is not ray firing time.','GT can arrive later for scoring; algorithm uses only arrival-causal IMU.',
                  'Exact XYZ admits geometry correspondence, not independent real-world physical synchronization.']}
    (a.output/'source.json').write_text(json.dumps(record,indent=2,sort_keys=True)+'\n')
    print(json.dumps(record),flush=True)
    if not admitted:raise ValueError('no admitted GT frames')


if __name__=='__main__':main()
