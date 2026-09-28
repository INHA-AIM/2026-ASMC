#!/usr/bin/env python3
"""Read-only AIM metadata/selected-payload audit; never grants training approval."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import os
from pathlib import Path
import struct

import numpy as np

PRUNE={'raw','chunks','software_snapshot','runtime','ros_logs','training_dataset_v1'}
REPORTS=('capture_contract.json','episode_000/motion_time_v1/report.json',
         'episode_000/native_audit/report.json','episode_000/gt_timeline_v1/report.json',
         'episode_000/object_geometry_audit_v2/report.json','episode_000/object_geometry_audit_v1/report.json',
         'episode_000/review_closeout_v2/report.json','episode_000/review_closeout/report.json',
         'episode_000/exposure_v1/report.json','episode_000/native_associations/report.json')


def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()


def canonical(o):return json.dumps(o,sort_keys=True,separators=(',',':'),allow_nan=False)


def safe_path(root,relative):
    result=(root/relative).resolve()
    result.relative_to(root.resolve())
    return result


def file_evidence(root,p):
    return {'path':str(p.relative_to(root)),'bytes':p.stat().st_size,'sha256':sha(p)}


def contracts(root):
    found=[]
    for directory,dirs,files in os.walk(root):
        dirs[:]=[d for d in dirs if d not in PRUNE and not d.startswith(('CAMERA_','LIDAR_','GPS_','IMU_'))]
        if 'sensor_contract.json' in files and Path(directory).name not in ('quality','episode_000'):
            found.append(Path(directory)/'sensor_contract.json')
    return sorted(found)


def profile(d):
    poses={}
    for k,v in sorted(d.get('transforms',{}).items()):
        if k.startswith('/base_link->'):
            poses[k]=v['transform']['transform']
    value={k:d.get(k) for k in ('raw_lidar_axes','encoder_lidar_axes','lidar_reference','lidar_to_base','lidar_channel_count','camera_fov_degrees','map_id')}
    value['sensor_poses']=poses
    return value


def decode_cloud(payload):
    """Decode serialized ROS1 PointCloud2, including fields/row stride/endian."""
    pos=0
    def unpack(fmt):
        nonlocal pos
        count=struct.calcsize(fmt);result=struct.unpack_from(fmt,payload,pos);pos+=count
        return result
    def string():
        nonlocal pos
        n,=unpack('<I');s=payload[pos:pos+n].decode();pos+=n;return s
    seq,secs,nsecs=unpack('<III');frame=string();height,width,nfields=unpack('<III')
    fields={}
    for _ in range(nfields):
        name=string();offset,datatype,count=unpack('<IBI');fields[name]=(offset,datatype,count)
    big,point_step,row_step,nbytes=unpack('<BIII')
    if nbytes!=height*row_step:raise ValueError('PointCloud2 byte length mismatch')
    data=payload[pos:pos+nbytes]
    if len(data)!=nbytes:raise ValueError('truncated PointCloud2')
    values=[]
    for name in ('x','y','z','intensity'):
        offset,datatype,count=fields[name]
        if (datatype,count)!=(7,1):raise ValueError('not scalar FLOAT32 '+name)
        values.append(np.ndarray((height,width),dtype='>f4' if big else '<f4',buffer=data,
                                  offset=offset,strides=(row_step,point_step)).reshape(-1))
    return np.column_stack(values),{'frame_id':frame,'stamp_ns':secs*10**9+nsecs,'seq':seq}


def selected_payloads(root,session,native):
    # Use recorded exact-pair candidates as identifiers, then independently check
    # native SHA, raw bytes, shape and all XYZI. Their simultaneity claim is unused.
    pairs=[c for c in native.get('comparison',[]) if c.get('role')=='lidar']
    if not pairs:return {'status':'no_recorded_raw_pair','samples':[]}
    chosen=[pairs[i] for i in sorted({0,len(pairs)//4,len(pairs)//2,3*len(pairs)//4,len(pairs)-1})]
    needed={p['ros_record_id'] for p in chosen};records={}
    index=session/'episode_000/quality/record_index.jsonl'
    with index.open() as stream:
        for line in stream:
            r=json.loads(line)
            if r['record_id'] in needed:records[r['record_id']]=r
    assets={a['relative_path']:a for a in native.get('assets',[])}
    rows=[]
    for pair in chosen:
        path=safe_path(session,pair['relative_path']);r=records[pair['ros_record_id']]
        if r.get('ros_type')!='sensor_msgs/PointCloud2' or r.get('encoding')!='ros1':
            raise ValueError('wrong raw record type')
        chunk=safe_path(session/'episode_000/raw',r['chunk_path'])
        with chunk.open('rb') as stream:
            stream.seek(r['payload_offset']);payload=stream.read(r['payload_bytes'])
        points,header=decode_cloud(payload)
        native_points=np.fromfile(path,dtype='<f4').reshape(-1,4)
        exact=points.shape==native_points.shape and np.array_equal(points,native_points,equal_nan=True)
        hashed=file_evidence(root,path)
        xyz=native_points[:,:3];valid=np.isfinite(xyz).all(axis=1)&(np.linalg.norm(xyz,axis=1)>1)
        lower=xyz[valid&(xyz[:,0]>3)&(xyz[:,0]<15)&(abs(xyz[:,1])<2)&(xyz[:,2]<-.5),2]
        rows.append({'native':hashed,'source_sha_matches':hashed['sha256']==assets[pair['relative_path']]['sha256'],
            'raw_record_id':r['record_id'],'raw_chunk':str(chunk.relative_to(root)),
            'raw_payload_sha256':hashlib.sha256(payload).hexdigest(),'raw_payload_offset':r['payload_offset'],
            'raw_payload_bytes':r['payload_bytes'],'header':header,'native_ros_xyzi_exact':bool(exact),
            'points':len(points),'finite_points':int(valid.sum()),
            'xyz_min':xyz[valid].min(axis=0).tolist(),'xyz_max':xyz[valid].max(axis=0).tolist(),
            'near_lower_surface_median_z':float(np.median(lower)) if len(lower) else None,
            'filename_minus_ros_stamp_ns':pair.get('filename_minus_ros_source_ns')})
    image_samples=[]
    for role in ('camera_1','camera_2','camera_3','camera_4'):
        candidates=[a for a in native['assets'] if a['role']==role]
        if not candidates:continue
        a=candidates[len(candidates)//2];p=safe_path(session,a['relative_path'])
        with p.open('rb') as f:header=f.read(24)
        if header[:8]!=b'\x89PNG\r\n\x1a\n':raise ValueError('bad native PNG')
        size=list(struct.unpack('>II',header[16:24]));h=file_evidence(root,p)
        image_samples.append({'role':role,'native':h,'resolution':size,
                              'matches_catalog':size==a['resolution'] and h['sha256']==a['sha256']})
    return {'status':'sample_native_ros_geometry_verified' if all(r['native_ros_xyzi_exact'] and
               r['source_sha_matches'] for r in rows) else 'mismatch',
            'samples':rows,'camera_samples':image_samples,
            'scope':'sample equality establishes binary format and ROS equivalence, not optical exposure or rear-axle datum'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--aim-root',type=Path,default=os.environ.get('ASMC_AIM_DATA'))
    p.add_argument('--autonav-sensors',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not a.aim_root or not a.aim_root.is_dir():p.error('set ASMC_AIM_DATA or --aim-root')
    if a.output.exists():p.error('output exists; never overwrite evidence')
    root=a.aim_root.resolve();rows=[];sources=[];profiles={};selected={}
    for i,c in enumerate(contracts(root)):
        session=c.parent;rel=str(session.relative_to(root));d=json.loads(c.read_text())
        normalized=profile(d);key=hashlib.sha256(canonical(normalized).encode()).hexdigest()
        profiles[key]=normalized;sources.append(file_evidence(root,c))
        reports={}
        for name in REPORTS:
            q=session/name
            if q.is_file():
                reports[name]=json.loads(q.read_text());sources.append(file_evidence(root,q))
        capture=reports.get('capture_contract.json',{});motion=reports.get('episode_000/motion_time_v1/report.json',{})
        native=reports.get('episode_000/native_audit/report.json',{})
        geo=reports.get('episode_000/object_geometry_audit_v2/report.json',reports.get('episode_000/object_geometry_audit_v1/report.json',{}))
        camera_geometry=reports.get('episode_000/exposure_v1/report.json',{}).get('calibration',{})
        row={'session':rel,'campaign_group':Path(rel).parts[0],'sensor_profile_sha256':key,
            'map_id':d.get('map_id'),'map_route_sha':d.get('capture',{}).get('traffic_light_ground_truth',{}).get('source_sha256',{}),
            'capture_hz_declared':capture.get('expected_capture_cadence_hz'),
            'native_counts':native.get('counts',{}),'native_bundles':native.get('complete_bundles'),
            'native_files_valid_reported':native.get('files_valid'),
            'motion_consistent_reported':motion.get('all_native_intervals_consistent'),
            'physical_simultaneity_verified_reported':native.get('physical_simultaneity_verified',False),
            'object_training_label_valid_reported':geo.get('object_training_label_valid',False),
            'object_geometry_complete_reported':geo.get('complete_relevant_geometry_scope',False),
            'object_models_reported':geo.get('observed_models',[]),
            'camera_geometry_provenance':camera_geometry,
            'source_snapshot_declared':bool(capture.get('producer_software_snapshot')),
            'roles_present':sorted(native.get('counts',{})),
            'lidar_gt_assets_declared':sorted({x['relative_path'].rsplit('_',1)[-1] for x in native.get('assets',[]) if x.get('role')=='lidar'}),
            'permission_for_accuracy_or_training':False}
        rows.append(row)
        # First audited session of each top-level campaign: deterministic source
        # inspection, not a score-driven selection or an accuracy holdout.
        group=row['campaign_group']
        if group not in selected and native.get('files_valid') and native.get('comparison'):
            try:
                checks=selected_payloads(root,session,native)
                sw=session/'software_snapshot';verified=[];mismatched=[]
                declared=capture.get('producer_software_snapshot',{}).get('files',{})
                for name in ('morai_pretraining/sensor_record_manifest.py','tools/audit_sensor_record_probe.py',
                             'tools/audit_sensor_record_exposure.py','async_capture/camera_geometry.py'):
                    q=sw/name
                    if q.exists():
                        evidence=file_evidence(root,q);sources.append(evidence)
                        (verified if declared.get(name)==evidence['sha256'] else mismatched).append(name)
                checks['software_sha_verified']=verified;checks['software_sha_mismatch']=mismatched
                timeline=session/'episode_000/gt_timeline_v1/frames.json'
                if timeline.exists():
                    frames=json.loads(timeline.read_text());sources.append(file_evidence(root,timeline))
                    positive=next((j for j,f in enumerate(frames) if f.get('object_gt',{}).get('objects')),len(frames)//2)
                    start=max(0,positive-50);segment=frames[start:start+100]
                    stamps=[f['lidar_source_ns'] for f in segment]
                    checks['candidate_segment']={'start_index':start,'frame_count':len(segment),
                        'lidar_start_ns':stamps[0] if stamps else None,'lidar_end_ns':stamps[-1] if stamps else None,
                        'positive_object_frames':sum(bool(f.get('object_gt',{}).get('objects')) for f in segment),
                        'strict_stamp_order':all(b>a for a,b in zip(stamps,stamps[1:])),
                        'source':'episode_000/gt_timeline_v1/frames.json',
                        'use':'geometry/reference only; motion/GT association needs per-frame admission before tracking scores'}
                selected[group]={'session':rel,**checks}
            except (ValueError,KeyError,OSError,struct.error) as e:
                selected[group]={'session':rel,'status':'inspection_failed','reason':str(e)}
        if i%25==0:print('metadata sessions audited',i+1,flush=True)
    autonav=json.loads(a.autonav_sensors.read_text())
    summary={'schema':'asmc-aim-data-readiness-v1','runtime_connected':False,
        'root_variable':'ASMC_AIM_DATA','sessions':len(rows),'sensor_profiles':profiles,
        'campaign_roots_without_contract':[p.name for p in sorted(root.iterdir()) if p.is_dir()
            and p.name not in {r['campaign_group'] for r in rows}],
        'campaign_sessions':dict(Counter(r['campaign_group'] for r in rows)),
        'metadata_bundles_reported':sum(r['native_bundles'] or 0 for r in rows),
        'motion_consistent_sessions_reported':sum(r['motion_consistent_reported'] is True for r in rows),
        'validated_training_label_sessions_reported':sum(r['object_training_label_valid_reported'] is True for r in rows),
        'physically_simultaneous_sessions_reported':sum(r['physical_simultaneity_verified_reported'] is True for r in rows),
        'selected_payload_checks':selected,
        'autonav_sensor_file':{'sha256':sha(a.autonav_sensors),'lidar_pos':autonav['Lidar3DList'][0]['pos'],
            'gps_pos':autonav['GPSList'][0]['pos'],'camera4_pos':autonav['cameraList'][3]['pos'],
            'camera4_rot':autonav['cameraList'][3]['rot']},
        'mission_coverage':{m:{'status':'candidate_campaign_only_needs_frame_route_verification',
            'candidate_campaign_groups':[g for g in sorted({r['campaign_group'] for r in rows})
                if (g.startswith('static') if m=='static_obstacle' else
                    g=='dynamic_obstacle' if m=='dynamic_crossing' else
                    g=='basic_driving' or g.startswith('full loop_'))]}
            for m in ('general_road','static_obstacle','dynamic_crossing','roundabout','highway','tollgate','gps_shadow')},
        'use_policy':{'single_cloud':'source-native geometry if sample provenance passes',
            'tracking':'conditional on per-frame motion/identity/clock checks',
            'camera_fusion':'measured projection and per-frame association required; camera4 excluded',
            'arrival_delay':'real YOLO arrival not established; injected delays must be labelled synthetic',
            'accuracy':'verified boxes only; unlabelled detections are not automatically false positives'},
        'limitations':['Metadata counts/status are producer reports, not full payload certification',
            'No Autonav performance equivalence: mounting/view/10Hz differ',
            'Raw/native float4 is distinct from FRU encoder data; no FRU conversion applied',
            'Shared map name does not prove equal route geometry, map offset or rear-axle datum',
            'Source data untouched; no training approval or model/runtime change'],
    }
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'sessions.jsonl').write_text(''.join(canonical(r)+'\n' for r in rows))
    (a.output/'summary.json').write_text(json.dumps(summary,sort_keys=True,indent=2)+'\n')
    (a.output/'manifest.json').write_text(json.dumps({'schema':'asmc-aim-audit-input-v1',
        'input_root':'ASMC_AIM_DATA','inputs':sorted(sources,key=lambda x:x['path']),
        'tool_sha256':sha(Path(__file__))},sort_keys=True,indent=2)+'\n')
    print(canonical({k:v for k,v in summary.items() if k in ('sessions','campaign_sessions','metadata_bundles_reported',
        'motion_consistent_sessions_reported','validated_training_label_sessions_reported','physically_simultaneous_sessions_reported')}))


if __name__=='__main__':main()
