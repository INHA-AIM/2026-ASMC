#!/usr/bin/env python3
"""Fit the empirical Molit turn gain from pre-loss GNSS/IMU/allowed motion.

Input motion must use the declared constant-offset model (gain zero). Recovery
fixes, native actor poses, controls and perception are never calibration inputs.
"""
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np


def preloss_basis(path,existing_offset=1.4,speed_floor=1.):
    import rosbag
    from pyproj import Transformer
    from scen_runner.morai.signal_trial_coordinates import Coordinates
    gps=[];motion=[];imu=[];lost=None
    with rosbag.Bag(str(path)) as bag:
        for topic,m,_ in bag.read_messages(topics=['/gps','/imu','/planning/localization/motion']):
            t=m.header.stamp.to_sec()
            if topic=='/gps':
                if m.status and m.latitude and m.longitude:
                    if lost is None:gps.append((t,m))
                elif len(gps)>10 and lost is None:lost=t
            elif topic=='/planning/localization/motion':
                p=m.pose.pose.position;q=m.pose.pose.orientation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                motion.append((t,p.x,p.y,yaw,m.twist.twist.linear.x))
            else:
                q=m.orientation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
                pitch=math.asin(max(-1,min(1,2*(q.w*q.y-q.z*q.x))))
                imu.append((t,roll,pitch,yaw))
    if lost is None or len(motion)<10 or len(imu)<10:raise ValueError('complete pre-loss inputs required')
    ma=np.asarray(sorted(set(motion)));ia=np.asarray(sorted(set(imu)))
    ma[:,3]=np.unwrap(ma[:,3]);ia[:,3]=np.unwrap(ia[:,3])
    gps=[(t,m) for t,m in gps if max(ma[0,0],ia[0,0])+.25<t<lost-.25]
    if len(gps)<10:raise ValueError('insufficient pre-loss paired GNSS')
    c=np.cos(ma[:,3]);s=np.sin(ma[:,3]);one=np.c_[c-c[0],s-s[0]]
    weights=1./np.maximum(.5*(ma[:-1,4]+ma[1:,4]),speed_floor)
    inverse=np.vstack(([0.,0.],np.cumsum(np.c_[np.diff(c),np.diff(s)]*weights[:,None],axis=0)))
    projection=Transformer.from_crs(4326,32652,always_xy=True);coordinates=Coordinates([302595.,4124145.])
    targets=[];bases=[]
    for t,g in gps:
        mx,my,myaw=[np.interp(t,ma[:,0],ma[:,j]) for j in (1,2,3)]
        roll,pitch,yaw=[np.interp(t,ia[:,0],ia[:,j]) for j in (1,2,3)]
        x,y=projection.transform(g.longitude,g.latitude);x-=g.eastOffset;y-=g.northOffset
        gyaw=coordinates.grid_yaw(x,y,math.degrees(yaw));phi=gyaw-myaw
        c,s=math.cos(gyaw),math.sin(gyaw)
        bx=.35*math.cos(pitch)+1.25*math.sin(pitch)*math.cos(roll);by=-1.25*math.sin(roll)
        measured=np.array([x-c*bx+s*by,y-s*bx-c*by])
        rotation=np.array([[math.cos(phi),-math.sin(phi)],[math.sin(phi),math.cos(phi)]])
        old=np.array([np.interp(t,ma[:,0],one[:,j]) for j in (0,1)])
        new=np.array([np.interp(t,ma[:,0],inverse[:,j]) for j in (0,1)])
        targets.append(measured-rotation@(np.array([mx,my])-existing_offset*old));bases.append(rotation@new)
    target=np.asarray(targets);basis=np.asarray(bases)
    # The per-run translation is a nuisance map anchor, not native truth fitting.
    target-=target.mean(axis=0);basis-=basis.mean(axis=0)
    return basis.reshape(-1,1),target.ravel(),dict(samples=len(gps),first_fix=gps[0][0],last_fix=gps[-1][0],loss_stamp=lost)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('root',type=Path);p.add_argument('--trial',required=True)
    p.add_argument('--existing-offset',type=float,default=1.4)
    p.add_argument('--confirm-constant-input',action='store_true',required=True,
                   help='confirm each input motion uses constant offset, turn gain zero')
    p.add_argument('bags',nargs='+',type=Path);a=p.parse_args()
    from localization_run import active_registration
    active_registration(a.root)
    if Path(a.trial).name!=a.trial or not math.isfinite(a.existing_offset):raise ValueError('invalid calibration request')
    output=a.root/'calibrations'/a.trial;output.mkdir(parents=True,exist_ok=False)
    datasets=[];inputs=[]
    for path in a.bags:
        x,y,details=preloss_basis(path,a.existing_offset);datasets.append((x,y))
        inputs.append(dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),**details))
    design=np.vstack([x for x,y in datasets]);target=np.concatenate([y for x,y in datasets])
    if float(np.linalg.norm(design))<.01:raise ValueError('heading excitation is insufficient')
    gain=float(np.linalg.lstsq(design,target,rcond=None)[0][0])
    result=dict(gain_m2ps=gain,speed_floor_mps=1.,existing_constant_offset_m=a.existing_offset,
                scope='pre-loss GNSS/IMU/allowed motion only; empirical Molit model; no physical acceptance',
                inputs=inputs,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                rms_m=[float(np.sqrt(np.mean(np.sum((y-x[:,0]*gain).reshape(-1,2)**2,axis=1)))) for x,y in datasets])
    (output/'fit.json').open('x').write(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':main()
