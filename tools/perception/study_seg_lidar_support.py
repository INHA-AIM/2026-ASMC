#!/usr/bin/env python3
"""Offline raw-cloud support inside the same box vs mask. No 3D GT accuracy claim."""
import argparse
from bisect import bisect_left
import hashlib
import html
import json
from pathlib import Path
import cv2
import numpy as np
import rosbag
from sensor_msgs import point_cloud2
import yaml
from asmc_camera.bev.calibration import load_calibration_snapshot, validate_sensor_set_snapshot, _rotation_x, _rotation_y, _rotation_z
from asmc_camera.detection.segmentation import SegmentationDetector
from asmc_camera.detection.hood_mask import HoodMask


def project_raw(points, lidar, camera):
    angles=np.radians([float(lidar['rot'][k]) for k in ('roll','pitch','yaw')])
    rotation=_rotation_z(angles[2])@_rotation_y(angles[1])@_rotation_x(angles[0])
    translation=np.array([float(lidar['pos'][k]) for k in ('x','y','z')])
    base=points@rotation.T+translation
    optical=camera.ground_to_optical(base)
    h=optical@camera.intrinsic.T
    pixels=np.full((len(points),2),np.nan)
    forward=optical[:,2]>0
    pixels[forward]=h[forward,:2]/h[forward,2,None]
    visible=(forward & np.isfinite(pixels).all(axis=1) & (pixels[:,0]>=0)&(pixels[:,0]<camera.width)
             &(pixels[:,1]>=0)&(pixels[:,1]<camera.height))
    return pixels,visible,optical[:,2]


def point_support(pixels,visible,xyxy,mask):
    x1,y1,x2,y2=xyxy
    inside=visible&(pixels[:,0]>=x1)&(pixels[:,0]<x2)&(pixels[:,1]>=y1)&(pixels[:,1]<y2)
    selected=np.flatnonzero(inside)
    kept=selected[mask[pixels[selected,1].astype(int),pixels[selected,0].astype(int)]]
    return selected,kept


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();root=Path(__file__).resolve().parents[2]
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'gallery').mkdir()
    protocol=yaml.safe_load(args.config.read_text())
    calibration=root/'src/asmc_camera/config/cameras_v3.yaml'
    document,cameras=load_calibration_snapshot(calibration)
    sensor_path=root/'tools/bridge/templates/sensor_v3.json'
    sensor=json.loads(sensor_path.read_text())
    validate_sensor_set_snapshot(document,cameras,sensor_path)
    lidar=sensor['Lidar3DList'][0];camera=cameras['front']
    hood=HoodMask.load(True,root/'src/asmc_camera/config/camera/yolo_hood_mask_v1.yaml')
    data={}
    for name,path in protocol['fusion']['bags'].items():
        with rosbag.Bag(path) as bag:
            scans=[(m.header.stamp.to_nsec(),m) for _,m,_ in bag.read_messages(topics=['/velodyne_points'])]
            images=[];next_sec=0.;start=bag.get_start_time()
            for _,m,t in bag.read_messages(topics=['/image_jpeg/compressed']):
                elapsed=t.to_sec()-start
                if elapsed>=next_sec:images.append((m.header.stamp.to_nsec(),m));next_sec=elapsed+1.
        data[name]=(sorted(scans,key=lambda value:value[0]),images)
    records=[];gallery=[]
    for model in ('yolo26','yoloe'):
        cfg=yaml.safe_load((root/protocol['object_profiles'][model]['config']).read_text())
        resolve=lambda s:s.replace('$(optenv ASMC_SEG_MODELS /data/models/camera_seg_v1)','/data/models/camera_seg_v1')
        backend=SegmentationDetector(weights=resolve(cfg['weights']),expected_sha256=cfg['expected_sha256'],
            allowed_names=cfg['allowed_names'],conf_thres=.7,predict_overrides=cfg['predict_overrides'],
            model_kind=cfg['model_kind'],prompt_embeddings=resolve(cfg.get('prompt_embeddings','')))
        for bag_name,(scans,images) in data.items():
            stamps=[s for s,_ in scans]
            for i,(stamp,message) in enumerate(images):
                j=bisect_left(stamps,stamp);candidates=[k for k in (j-1,j) if 0<=k<len(stamps)]
                k=min(candidates,key=lambda k:abs(stamps[k]-stamp))
                offset=abs(stamps[k]-stamp)/1e6
                if offset>50:
                    records.append(dict(model=model,bag=bag_name,camera_stamp_ns=stamp,reason='no_raw_scan_within_50ms',offset_ms=offset));continue
                scan=scans[k][1]
                if scan.header.frame_id!='velodyne':raise ValueError('expected raw sensor frame velodyne')
                points=np.array(list(point_cloud2.read_points(scan,field_names=('x','y','z'),skip_nans=True)),float)
                pixels,visible,depth=project_raw(points,lidar,camera)
                image=cv2.imdecode(np.frombuffer(message.data,np.uint8),cv2.IMREAD_COLOR)
                if image.shape[:2]!=(camera.height,camera.width):raise ValueError('camera/sensor calibration dimensions disagree')
                result=backend.predict_frame(hood.apply(image));boxes=[]
                annotated=image.copy()
                for d,mask in zip(result.detections,result.masks):
                    if mask is None:raise ValueError('seg mask missing')
                    xyxy=[d.xmin,d.ymin,d.xmax,d.ymax]
                    inside,kept=point_support(pixels,visible,xyxy,mask)
                    boxes.append(dict(label=d.source_class_name,confidence=d.confidence,xyxy=xyxy,
                                      box_points=len(inside),mask_points=len(kept),removed_points=len(inside)-len(kept),
                                      box_depth_p95=float(np.percentile(depth[inside],95)) if len(inside) else None,
                                      mask_depth_p95=float(np.percentile(depth[kept],95)) if len(kept) else None))
                    annotated[mask]=(annotated[mask]*.7+np.array([40,230,40])*.3).astype(np.uint8)
                    for indices,color in ((inside,(0,0,255)),(kept,(0,255,255))):
                        for index in indices:cv2.circle(annotated,tuple(pixels[index].astype(int)),1,color,-1)
                    x1,y1,x2,y2=map(int,xyxy);cv2.rectangle(annotated,(x1,y1),(x2,y2),(255,255,0),1)
                records.append(dict(model=model,bag=bag_name,camera_stamp_ns=stamp,lidar_stamp_ns=stamps[k],offset_ms=offset,
                                    source_frame=scan.header.frame_id,boxes=boxes))
                if i%8==0:
                    name=model+'-'+bag_name+'-%03d.jpg'%i
                    cv2.imwrite(str(args.output/'gallery'/name),annotated)
                    gallery.append('<figure><figcaption>'+html.escape(model+' '+bag_name+' '+str(stamp))+'</figcaption><img width="960" loading="lazy" src="'+name+'"></figure>')
        del backend
    (args.output/'observations.json').write_text(json.dumps(records)+'\n')
    summary=dict(samples=len(records),models={m:dict(
        boxes=sum(len(r.get('boxes',[])) for r in records if r['model']==m),
        box_points=sum(b['box_points'] for r in records if r['model']==m for b in r.get('boxes',[])),
        mask_points=sum(b['mask_points'] for r in records if r['model']==m for b in r.get('boxes',[]))) for m in ('yolo26','yoloe')},
        coordinate='raw velodyne -> fixed lidar mount -> base_link -> front optical; no leveling inverse',
        calibration_sha256=hashlib.sha256(calibration.read_bytes()).hexdigest(),
        sensors_sha256=hashlib.sha256(sensor_path.read_bytes()).hexdigest(),
        operational_fusion_changed=False,gt_accuracy_evaluable=False,
        scope='offline nearest raw scan within 50ms; 1s camera samples; no arrival policy, depth visibility or GT semantic labels',
        warning='fewer points can remove object points; support reduction is not wall-removal or object-preservation accuracy')
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    (args.output/'gallery/index.html').write_text('<!doctype html><meta charset="utf-8"><title>Raw LiDAR box/mask support</title><h1>Red: box support / Yellow: mask support · no 3D GT accuracy claim</h1>'+''.join(gallery))
    print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
