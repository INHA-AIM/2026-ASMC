#!/usr/bin/env python3
"""Read immutable ROS1 PointCloud2 XYZI into per-scan research inputs."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores,get_typestore


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--baseline',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False);store=get_typestore(Stores.ROS1_NOETIC)
    rows=[];intensities=[]
    with Reader(a.bag) as reader:
        connection=next(c for c in reader.connections if c.topic=='/velodyne_points')
        for _,recorded,data in reader.messages(connections=[connection]):
            message=store.deserialize_ros1(data,connection.msgtype)
            fields={f.name:f for f in message.fields}
            if message.is_bigendian or message.point_step!=16 or message.height!=1:
                raise ValueError('unsupported PointCloud2 layout; do not silently reinterpret')
            if any(fields[k].offset!=4*i or fields[k].datatype!=7 for i,k in enumerate(('x','y','z','intensity'))):
                raise ValueError('unexpected XYZI fields')
            raw=np.frombuffer(message.data,dtype='<f4').reshape(-1,4).copy()
            valid=np.isfinite(raw).all(axis=1)&(np.linalg.norm(raw[:,:3],axis=1)>1e-3)
            stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
            with np.load(a.baseline/'inputs/clouds'/f'{stamp}.npz') as baseline:
                base=np.isfinite(raw[:,:3]).all(axis=1)
                if not np.array_equal(raw[base,:3],baseline['xyz']):raise ValueError('baseline XYZ differs')
            selected=raw[valid];intensities.extend(selected[:,3].tolist())
            np.savez_compressed(a.output/f'{stamp}.npz',xyzi=selected,raw_index=np.flatnonzero(valid))
            rows.append(dict(stamp_ns=stamp,bag_ns=recorded,points=len(selected),raw_points=len(raw),
                             discarded_nonfinite_or_zero=int((~valid).sum()),frame=message.header.frame_id))
    digest=hashlib.sha256(a.bag.read_bytes()).hexdigest()
    summary=dict(bag_sha256=digest,scans=len(rows),rows=rows,
        intensity_percentiles={str(q):float(np.percentile(intensities,q)) for q in (0,1,50,95,99,100)},
        baseline_xyz_verified=True,features='raw x/y/z/intensity; no ground removal, no accumulation, no synthetic per-point time')
    (a.output/'manifest.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps({k:v for k,v in summary.items() if k!='rows'}))


if __name__=='__main__':main()
