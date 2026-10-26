"""Rendering and fixed-OBB sensitivity, never modifies a runtime object.

Transforms are executed by context_probe. Camera projection calls the production
calibration class. The map illustration uses its pinned raster, not lane centers.
"""
import gzip
import html
import json
from pathlib import Path
import numpy as np
import cv2
from asmc_camera.bev.calibration import load_calibration_snapshot
from asmc_camera.runtime.ground_projection import attitude_camera


def project(camera, points):
    p=np.asarray(points,float)
    q=(camera.optical_from_body_rotation@p.T).T+camera.optical_from_body_translation
    h=(camera.intrinsic@q.T).T
    return h[:,:2]/h[:,2:3],q[:,2]


def paint(image,camera,points,color):
    uv,z=project(camera,points)
    for pixel,depth in zip(uv,z):
        if depth>.1 and np.isfinite(pixel).all() and (pixel>=0).all() and pixel[0]<image.shape[1] and pixel[1]<image.shape[0]:
            cv2.circle(image,tuple(np.rint(pixel).astype(int)),3,color,-1)


def render(args,inputs,camera_rows):
    from audit_frame_time import ROOT, read, sha, json_probe, write_rows, stats
    _,cameras=load_calibration_snapshot(ROOT/'src/asmc_camera/config/cameras_v3.yaml')
    out=args.output; sections=[];metrics={}
    # Only selected RGBs are decoded; retain original front/side calibration sizes.
    import rosbag
    topics={'/image_jpeg/compressed':'front','/cam_left/image_jpeg/compressed':'left','/cam_right/image_jpeg/compressed':'right'}
    bundle=json.loads((ROOT/'src/asmc_lidar/config/hd_map_audit/manifest.json').read_text())
    root=ROOT/'src/asmc_lidar/config/hd_map_audit'
    desc=bundle['arrays']['occupancy'];occupancy=np.frombuffer(gzip.decompress((root/desc['file']).read_bytes()),np.uint8).reshape(bundle['height'],bundle['width'])
    origin=np.array(bundle['origin_xy']);res=bundle['resolution_m']
    for name in ('normal','tunnel'):
        scans=list(read(out/name/'lidar.jsonl'));states=list(read(out/name/'planning.jsonl'))
        requests=[];keys=[]
        for scan,state in zip(scans,states):
            if not(scan.get('map',{}).get('valid') and state['valid']):continue
            for obj in scan['typed']['local'].get('objects',[]):
                requests.append(dict(map_from_base=scan['map']['map_from_base'],alternative=state,points=obj['corners']))
                keys.append((scan['seq'],obj['track_id'],scan['stamp_ns'],state['state_ns']))
        transformed=json_probe(args.probes/'context_probe',requests,(root/'manifest.json',sha(root/'manifest.json'))) if requests else []
        # Actual HdMapAudit is called in the context probe, including the configured inset.
        records=[]
        for key,row in zip(keys,transformed):
            delta=np.linalg.norm(np.array(row['original'])-np.array(row['alternative']),axis=1)
            records.append(dict(seq=key[0],track_id=key[1],scan_ns=key[2],state_ns=key[3],
                same_time=key[2]==key[3],corner_delta_m=float(delta.max()),**row))
        write_rows(out/name/'obb-sensitivity.jsonl',records)
        metrics[name]=dict(objects=len(records),corner_delta_m=stats(r['corner_delta_m'] for r in records),
                           roundtrip_m=stats(r['roundtrip_m'] for r in records),
                           relation_changes=sum(r.get('original_relation',{}).get('reason')!=r.get('alternative_relation',{}).get('reason') for r in records),
                           note='Fixed delivered OBB, alternative xy/yaw only; supports not recovered from trace, no filter adoption')
        # Sample nine scans spanning the whole recording, not cherry-picked by validity.
        selected=[scans[int(i)] for i in np.linspace(0,len(scans)-1,min(9,len(scans))).astype(int)]
        wanted={}
        for scan in selected:
            for view in cameras:wanted[(scan['seq'],view)]=None
        with rosbag.Bag(str(inputs[name])) as bag:
            for topic,msg,arrival in bag.read_messages(topics=list(topics)):
                view=topics[topic]; stamp=msg.header.stamp.to_nsec()
                for scan in selected:
                    if arrival.to_nsec()>scan['arrival_ns']:continue
                    delta=abs(stamp-scan['stamp_ns']);key=scan['seq'],view
                    if delta<=50e6 and (wanted[key] is None or delta<wanted[key][0]):wanted[key]=(delta,bytes(msg.data),stamp)
        section='<h2>'+name+' — receive-header RGB overlay</h2><p>Green: delivered local OBB corners. Map: green current, orange asynchronous planning pose sensitivity. Corner points are not semantic GT.</p>'
        for scan in selected:
            panels=[]
            for view in ('left','front','right'):
                match=wanted[scan['seq'],view]
                if match:
                    image=cv2.imdecode(np.frombuffer(match[1],np.uint8),1)
                    for obj in scan['typed']['local'].get('objects',[]):paint(image,cameras[view],obj['corners'],(0,255,0))
                    cv2.putText(image,view+' dt=%.1fms'%((match[2]-scan['stamp_ns'])/1e6),(8,25),0,.65,(0,220,255),2)
                    panels.append(cv2.resize(image,(480,270)))
                else:panels.append(np.zeros((270,480,3),np.uint8))
            file=name+'-%04d.jpg'%scan['seq'];cv2.imwrite(str(out/file),np.hstack(panels))
            section+='<p>scan '+str(scan['seq'])+' / '+html.escape(scan.get('map',{}).get('reason','input invalid'))+'</p><img src="'+file+'">'
            if scan.get('map',{}).get('valid'):
                m=scan['map']['map_from_base'];center=np.array([m[3],m[7]])
                # Map raster row direction follows the manifest's increasing map y.
                size=300;xy=(center-origin)/res;ix,iy=np.rint(xy).astype(int)
                canvas=np.zeros((size,size,3),np.uint8)
                for y in range(size):
                    yy=iy+y-size//2
                    x0=max(0,ix-size//2);x1=min(occupancy.shape[1],ix+size//2)
                    if 0<=yy<occupancy.shape[0] and x0<x1:
                        v=occupancy[yy,x0:x1];start=x0-(ix-size//2)
                        canvas[y,start:start+len(v)][v>0]=(75,60,55)
                for row in records:
                    if row['seq']!=scan['seq']:continue
                    for key,col in [('original',(0,255,0)),('alternative',(0,140,255))]:
                        pts=np.rint((np.array(row[key])[:,:2]-center)/res+size/2).astype(np.int32)
                        cv2.polylines(canvas,[cv2.convexHull(pts)],True,col,1)
                cv2.circle(canvas,(size//2,size//2),3,(255,255,255),-1)
                file=name+'-%04d-map.png'%scan['seq'];cv2.imwrite(str(out/file),cv2.flip(canvas,0))
                section+='<img width="500" src="'+file+'">'
        sections.append(section)
    # Separate source-time recording: nominal vs attitude plane projected reference grid.
    frames=list(read(inputs['camera_frames']));rows={(r['index'],r['view']):r for r in camera_rows if r['query_after_publish_ms']==100}
    section='<h2>Separate source-time camera recording</h2><p>Reference ground points only (green nominal / orange IMU plane). No LiDAR from the other bags is inserted into these images. 100ms causal availability is an offline sensitivity, not a runtime wait recommendation.</p>'
    for frame in frames[::max(1,len(frames)//8)]:
        panels=[]
        for view in ('left','front','right'):
            f=frame['views'][view];path=inputs['camera_frames'].parent/f['rgb']
            if sha(path)!=f['rgb_sha256']:raise ValueError('RGB SHA mismatch')
            image=cv2.imread(str(path));cam=cameras[view];q=rows[(frame['index'],view)]['quaternion']
            if q:
                adjusted=attitude_camera(cam,q)
                # Both objects project the same metric reference points through production camera methods.
                xy=np.array([[x,y] for x in (3,5,10,15) for y in (-5,-2,0,2,5)])
                for camera,col in ((cam,(0,255,0)),(adjusted,(0,140,255))):
                    uv,z=camera.project_ground(xy)
                    for pixel,depth in zip(uv,z):
                        if depth>.1 and np.isfinite(pixel).all() and (pixel>=0).all() and pixel[0]<image.shape[1] and pixel[1]<image.shape[0]:cv2.circle(image,tuple(np.rint(pixel).astype(int)),3,col,-1)
            panels.append(cv2.resize(image,(480,270)))
        file='source-%03d.jpg'%frame['index'];cv2.imwrite(str(out/file),np.hstack(panels));section+='<img src="'+file+'">'
    sections.append(section)
    (out/'gallery.html').write_text('<!doctype html><meta charset="utf-8"><title>C0 gallery</title><style>body{background:#181d24;color:#eee;font:16px sans-serif;margin:2em}img{max-width:100%;margin:1em 0}a{color:#8df}</style><a href="index.html">C0 report</a>'+''.join(sections))
    return metrics
