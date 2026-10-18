#!/usr/bin/env python3
"""Read-only stationary LiDAR/IMU calibration and exact RGB/model-grid capture.

No control, scenario, source-timestamp mode or operational geometry is changed.
The fitted plane is a candidate until stationarity and independent checks pass.
"""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import sys
import threading
import time

import numpy as np


def fit_plane(points, seed=20261011):
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<40 or not np.isfinite(points).all():
        raise ValueError('at least 40 XYZ points required')
    rng=np.random.RandomState(seed);matrix=np.column_stack((points[:,:2],np.ones(len(points))))
    best=None;best_count=0
    for _ in range(100):
        indices=rng.choice(len(points),3,replace=False)
        try:coeff=np.linalg.solve(matrix[indices],points[indices,2])
        except np.linalg.LinAlgError:continue
        if np.linalg.norm(coeff[:2])>.15:continue
        inliers=np.abs(matrix@coeff-points[:,2])<.025
        if inliers.sum()>best_count:best,best_count=inliers,int(inliers.sum())
    if best is None or best_count<40:raise ValueError('no supported near-horizontal plane')
    for _ in range(3):
        coeff=np.linalg.lstsq(matrix[best],points[best,2],rcond=None)[0]
        best=np.abs(matrix@coeff-points[:,2])<.025
        if best.sum()<40:raise ValueError('plane lost support')
    residual=np.abs(matrix@coeff-points[:,2]);normal=np.r_[-coeff[:2],1.]
    normal/=np.linalg.norm(normal)
    return dict(coefficients=coeff.tolist(),normal=normal.tolist(),
                normal_distance_m=float(-coeff[2]/np.linalg.norm(np.r_[-coeff[:2],1.])),
                inlier_ratio=float(best.mean()),inlier_residual_p95_m=float(np.percentile(residual[best],95)),
                points=len(points),inliers=int(best.sum()))


def mean_quaternion(values):
    q=np.asarray(values,dtype=float)
    if q.ndim!=2 or q.shape[1]!=4 or not len(q) or not np.isfinite(q).all() or np.any(np.linalg.norm(q,axis=1)<1e-9):
        raise ValueError('nonempty finite unit-quaternion samples required')
    q=q/np.linalg.norm(q,axis=1)[:,None]
    _,vectors=np.linalg.eigh(q.T@q);result=vectors[:,-1]
    return result if result[3]>=0 else -result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=12.)
    parser.add_argument('--grpc-root',type=Path,default=Path('/root/ws/tools/grpc_inha_univ/src'))
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    sys.path[:0]=[str(args.grpc_root),str(args.grpc_root/'api'),str(args.grpc_root/'proto')]
    import grpc,message_filters,rospy
    from google.protobuf.json_format import MessageToDict
    from morai.actor.actor_pb2_grpc import ActorStub
    from morai.sensor.sensor_pb2_grpc import SensorStub
    from morai.common.object_info_pb2 import ObjectInfo
    from morai.common.enum_pb2 import OBJECT_TYPE_VEHICLE
    from sensor_msgs.msg import Imu,PointCloud2,CompressedImage,Image
    from tf2_msgs.msg import TFMessage
    try:
        from tools.perception.review_family_motion import rotation,rpy
    except ImportError:
        from review_family_motion import rotation,rpy
    rospy.init_node('stationary_lane_calibration',anonymous=True,disable_signals=True)
    actor=ActorStub(grpc.insecure_channel('host.docker.internal:7789'))
    sensor=SensorStub(grpc.insecure_channel('host.docker.internal:7789'))
    ego=ObjectInfo(object_type=OBJECT_TYPE_VEHICLE);ego.id.value='Ego'
    spec=MessageToDict(actor.GetVehicleSpec(ego,timeout=3))
    infos=MessageToDict(sensor.GetSensorInfos(ego,timeout=3))
    data=defaultdict(list);lock=threading.Lock();last={};subs=[];filters=[];pairs=[]
    start=time.monotonic()
    def imu_callback(msg):
        with lock:
            if len(data['imu'])<1500:
                data['imu'].append(dict(source_ns=msg.header.stamp.to_nsec(),
                    q_xyzw=[getattr(msg.orientation,k) for k in 'xyzw'],
                    gyro_xyz=[getattr(msg.angular_velocity,k) for k in 'xyz']))
    def cloud_callback(msg):
        now=time.monotonic()
        with lock:
            if len(data['clouds'])<50 and now-last.get('cloud',0)>.25:
                data['clouds'].append(msg);last['cloud']=now
    def tf_callback(msg):
        with lock:
            for t in msg.transforms:
                if t.header.frame_id.lstrip('/')=='base_link' and t.child_frame_id.lstrip('/')=='velodyne':
                    data['lidar_tf']=[dict(translation=[getattr(t.transform.translation,k) for k in 'xyz'],
                        rotation_xyzw=[getattr(t.transform.rotation,k) for k in 'xyzw'])]
    def frame_callback(rgb,classes,confidence,drivable,view):
        now=time.monotonic()
        with lock:
            if len(data[view])<24 and now-last.get(view,0)>.7:
                data[view].append((rgb,classes,confidence,drivable));last[view]=now
    subs.extend([rospy.Subscriber('/imu',Imu,imu_callback,queue_size=10),
                 rospy.Subscriber('/velodyne_points',PointCloud2,cloud_callback,queue_size=1),
                 rospy.Subscriber('/tf_static',TFMessage,tf_callback,queue_size=10)])
    for view,topic in dict(front='/image_jpeg/compressed',left='/cam_left/image_jpeg/compressed',right='/cam_right/image_jpeg/compressed').items():
        base='/perception/camera/'+view+'/road_marking/model_grid'
        group=[message_filters.Subscriber(topic,CompressedImage,queue_size=2,buff_size=16<<20),
               message_filters.Subscriber(base+'/class_id',Image,queue_size=2,buff_size=16<<20),
               message_filters.Subscriber(base+'/confidence',Image,queue_size=2,buff_size=16<<20),
               message_filters.Subscriber('/perception/camera/'+view+'/drivable/probability',Image,queue_size=2,buff_size=16<<20)]
        sync=message_filters.TimeSynchronizer(group,12);sync.registerCallback(frame_callback,view)
        filters.extend(group);pairs.append(sync)
    try:
        while time.monotonic()-start<args.duration and not rospy.is_shutdown():
            state=actor.GetActorState(ego,timeout=3)
            data['ego'].append(dict(receive_monotonic=time.monotonic(),
                xyz=[getattr(state.transform.location,k) for k in 'xyz'],
                rotation_deg=[getattr(state.transform.rotation,k) for k in 'xyz'],
                velocity=[getattr(state.global_velocity,k) for k in 'xyz']))
            time.sleep(.2)
    finally:
        for sub in subs+filters:sub.unregister()
        rospy.signal_shutdown('stationary read-only capture complete')
    for name in ('imu','ego'):
        (args.output/(name+'-raw.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in data[name]))
    if not data['lidar_tf']:raise RuntimeError('live raw LiDAR -> base_link transform not confirmed')
    tf=data['lidar_tf'][0];R=rotation(tf['rotation_xyzw']);translation=np.asarray(tf['translation'])
    cloud_fits=[]
    for i,msg in enumerate(data['clouds']):
        fields={f.name:f for f in msg.fields}
        if any(fields[k].datatype!=7 for k in ('x','y','z')):raise ValueError('expected float32 XYZ')
        dtype=np.dtype(dict(names=['x','y','z'],formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
                            offsets=[fields[k].offset for k in ('x','y','z')],itemsize=msg.point_step))
        values=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step)).reshape(-1)
        xyz=np.column_stack([values[k] for k in ('x','y','z')]).astype(float)@R.T+translation
        good=np.isfinite(xyz).all(axis=1)&(xyz[:,0]>4)&(xyz[:,0]<16)&(np.abs(xyz[:,1])<4)&(xyz[:,2]>-.8)&(xyz[:,2]<.15)
        points=xyz[good]
        try:fit=fit_plane(points)
        except ValueError as e:fit=dict(error=str(e))
        fit.update(source_ns=msg.header.stamp.to_nsec(),frame_id=msg.header.frame_id)
        cloud_fits.append(fit)
        np.savez_compressed(args.output/('ground-%03d.npz'%i),points_base=points)
    poses=data['ego'];xyz=np.array([r['xyz'] for r in poses]);vel=np.array([r['velocity'] for r in poses])
    im=sorted(data['imu'],key=lambda s:s['source_ns']);quaternions=np.array([r['q_xyzw'] for r in im])
    mean_q=mean_quaternion(quaternions);angles=np.rad2deg(np.array([rpy(q) for q in quaternions]));gyro=np.array([r['gyro_xyz'] for r in im])
    valid_fits=[f for f in cloud_fits if 'error' not in f]
    accepted=[f for f in valid_fits if f['inlier_ratio']>=.7]
    if not valid_fits or not im or not poses:
        raise RuntimeError('missing ground/IMU/Ego input; cannot calibrate')
    # Keep an unsupported candidate visible, but never promote it. A failed
    # support gate must not disappear into an empty-array reduction.
    normals=np.array([f['normal'] for f in valid_fits]);normal=np.median(normals,axis=0);normal/=np.linalg.norm(normal)
    h=float(np.median([f['normal_distance_m'] for f in valid_fits]))
    movement=float(np.max(np.linalg.norm(xyz-xyz[0],axis=1)))
    speed=float(np.max(np.linalg.norm(vel,axis=1)))
    gates=dict(displacement_le_10cm=movement<=.1,speed_le_5cmps=speed<=.05,
               imu_gyro_le_0p01=float(np.max(np.linalg.norm(gyro,axis=1)))<=.01,
               attitude_std_le_0p05deg=bool(np.max(np.std(angles[:,:2],axis=0))<=.05),
               supported_ground_frames=len(accepted)>=20,height_plausible=.2<=h<=.6)
    imu_up=rotation(mean_q).T@np.array([0.,0.,1.]);normal_error=float(np.rad2deg(np.arccos(np.clip(normal@imu_up,-1,1))))
    frames=[]
    for view in ('front','left','right'):
        for i,(rgb,cls,confidence,drivable) in enumerate(data[view]):
            def mono(msg):return np.ndarray((msg.height,msg.width),np.uint8,buffer=msg.data,strides=(msg.step,1)).copy()
            stem=view+'-%03d'%i;blob=bytes(rgb.data)
            (args.output/(stem+'.jpg')).write_bytes(blob)
            np.savez_compressed(args.output/(stem+'.npz'),classes=mono(cls),confidence=mono(confidence),drivable=mono(drivable))
            frames.append(dict(view=view,source_ns=rgb.header.stamp.to_nsec(),frame_id=rgb.header.frame_id,
                               image=stem+'.jpg',grid=stem+'.npz',rgb_sha256=hashlib.sha256(blob).hexdigest()))
    report=dict(schema='stationary_plane_calibration_v1',vehicle_spec=spec,sensors=infos,lidar_tf=tf,
        duration_s=args.duration,ego_displacement_m=movement,max_speed_mps=speed,
        attitude_mean_rpy_deg=np.rad2deg(rpy(mean_q)).tolist(),attitude_std_rp_deg=np.std(angles[:,:2],axis=0).tolist(),
        reference_quaternion_xyzw=mean_q.tolist(),ground_normal_base=normal.tolist(),height_normal_m=h,
        ground_coefficients=[float(-normal[0]/normal[2]),float(-normal[1]/normal[2]),float(-h/normal[2])],
        imu_up_vs_lidar_normal_angle_deg=normal_error,ground_frames=len(accepted),fitted_frames=len(valid_fits),gates=gates,
        stationary_gates_pass=all(gates.values()),adopted=False,operational_changes=False,
        limitations=['Static local plane/IMU candidate, not surveyed GT or moving validation.',
                     'Current ROS header receive-time policy retained; no source-time rewrite.',
                     'No vehicle actuation; read-only Actor/Sensor RPCs only.'])
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (args.output/'cloud-fits.json').write_text(json.dumps(cloud_fits,indent=2)+'\n')
    for name,rows in (('imu',im),('ego',poses),('frames',frames)):
        (args.output/(name+'.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in rows))
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
