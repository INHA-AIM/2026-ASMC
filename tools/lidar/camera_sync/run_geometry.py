#!/usr/bin/env python3
"""Freeze arrival-time inputs, run pinned geometry, write reproducible JSONL."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import subprocess

import numpy as np
import yaml
from run_evaluation import digest_file, scene_at, percentile

MODES = ('baseline', 'candidate', 'runtime_off', 'old_offline_no_protection')


def dump(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def geometry_only(value):
    if isinstance(value, dict):
        return {k: geometry_only(v) for k, v in value.items()
                if not k.endswith('_ms') and k != 'timing_ms'}
    if isinstance(value, list):
        return [geometry_only(v) for v in value]
    return value


def mode_geometry(value):
    """Geometry comparison must not count availability-only changes as motion."""
    return {k:value[k] for k in ('labels','kept_indices','objects','leveling',
        'leveled_xyzi_fnv1a64','clusters','final_objects')}


def float4_cloud(message):
    if message.is_bigendian:
        raise ValueError('big-endian input needs explicit validation')
    fields = {f.name: f for f in message.fields}
    if any(name not in fields or fields[name].datatype != 7 or fields[name].count != 1
           for name in ('x', 'y', 'z', 'intensity')):
        raise ValueError('expected scalar FLOAT32 x/y/z/intensity')
    return np.column_stack([np.ndarray((message.height, message.width), dtype='<f4',
        buffer=message.data, offset=fields[name].offset,
        strides=(message.row_step, message.point_step)).reshape(-1)
        for name in ('x', 'y', 'z', 'intensity')]).astype('<f4')


def legacy_rpy(message):
    x,y,z,w = [np.float32(getattr(message.orientation, k)) for k in ('x','y','z','w')]
    # Same float quaternion storage and formula as pinned IMUParser_Callback.
    return [float(np.float32(math.atan2(2.0*(w*x+y*z), 1.0-2.0*(x*x+y*y)))),
            float(np.float32(math.asin(2.0*(w*y-z*x)))),
            float(np.float32(math.atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z))))]


def snapshot(bag, scenes, decisions, output):
    import rosbag
    output.mkdir(parents=True, exist_ok=False)
    (output/'raw').mkdir()
    decision_rows = {r['baseline']['source_index']: r for r in
                     map(json.loads, decisions.read_text().splitlines())}
    poses, boxes, rows = {}, {}, []
    invalid_gps = False
    tunnel = False
    valid_count = invalid_count = 0
    source_index = 0
    with rosbag.Bag(str(bag), 'r') as source:
        topics=['/velodyne_points','/YOLO/BBox','/imu','/gps','/Ego_topic']
        for topic, message, arrival in source.read_messages(topics=topics):
            at = arrival.to_nsec()
            if topic in ('/imu','/gps','/Ego_topic'):
                poses[topic] = (message, at)
                if topic == '/gps':
                    invalid_gps = message.status == 0 or (message.latitude == 0 and
                        message.longitude == 0 and message.altitude == 0)
                    if invalid_gps:
                        invalid_count=min(invalid_count+1,3);valid_count=0
                        if invalid_count>=3:tunnel=True
                    else:
                        valid_count=min(valid_count+1,3);invalid_count=0
                        if valid_count>=3:tunnel=False
                continue
            idx=source_index;source_index+=1
            if topic == '/YOLO/BBox':
                boxes[idx]=[{'class':b.classes,'xmin':float(b.xMin),'ymin':float(b.yMin),
                             'xmax':float(b.xMax),'ymax':float(b.yMax)} for b in message.yolo_bbox_arr]
                continue
            d=decision_rows[idx]
            if message.header.stamp.to_nsec()!=d['baseline']['scan_stamp_ns']:
                raise ValueError('scan identity mismatch')
            phases=[s for s in scenes if int(s['transition_wall_s']*1e9)<=at]
            phase=phases[-1]['name'] if phases else 'before_initial'
            scene=scene_at(at,scenes)
            row={'input_id':d['input_id'],'scene':scene,'phase':phase,
                 'measurement':scene!='transition_or_unassigned',
                 'scan_stamp_ns':message.header.stamp.to_nsec(),
                 'scan_effective_stamp_ns':d['baseline']['scan_effective_stamp_ns'],
                 'scan_arrival_ns':at, 'scan_message_sha256':d['baseline']['scan_digest'],
                 'baseline':d['baseline'],'candidate':d['candidate'],
                 'geometry_valid':False,'already_preprocessed':False,'pose_evidence':{}}
            reason = ('invalid_scan_stamp' if message.header.stamp.to_nsec()<=0 else
                      'gps_normal_filter_inactive' if not (invalid_gps or tunnel) else
                      'missing_state' if any(t not in poses for t in ('/imu','/gps','/Ego_topic')) else None)
            if reason is None:
                try:
                    rpy=legacy_rpy(poses['/imu'][0]);ego=poses['/Ego_topic'][0]
                    speed=float(np.float32(math.sqrt(sum(getattr(ego.velocity,k)**2 for k in ('x','y','z')))))
                    if not all(math.isfinite(x) for x in rpy+[speed]):raise ValueError('nonfinite')
                    row.update(roll=rpy[0],pitch=rpy[1],yaw=rpy[2],speed_mps=speed)
                    for name,(m,a) in sorted(poses.items()):
                        stamp=m.header.stamp.to_nsec() if hasattr(m,'header') else None
                        row['pose_evidence'][name]={'arrival_ns':a,'stamp_ns':stamp,
                            'age_at_scan_arrival_ns':at-a,'used_future':a>at}
                    if any(v['used_future'] for v in row['pose_evidence'].values()):raise ValueError('future_pose')
                    cloud=float4_cloud(message)
                    if not np.all(np.isfinite(cloud)):raise ValueError('nonfinite_points')
                    path=output/'raw'/ (row['input_id']+'.bin');cloud.tofile(path)
                    row.update(geometry_valid=True,cloud_path=str(path.relative_to(output)),
                               cloud_bytes=path.stat().st_size,cloud_sha256=digest_file(path))
                except (ValueError,OverflowError) as e:
                    reason='invalid_state_or_cloud:'+str(e)
            row['geometry_reject_reason']=reason
            rows.append(row)
    if len(rows)!=len(decision_rows):raise ValueError('scan count mismatch')
    with (output/'snapshots.jsonl').open('w') as stream:
        for row in rows:
            for mode in ('baseline','candidate'):
                selected=row[mode]['selected_yolo_index']
                row[mode]['boxes']=boxes[selected] if selected is not None else []
            stream.write(dump(row)+'\n')
    return rows


def convert_geometry(path, output):
    loader=getattr(yaml,'CSafeLoader',yaml.SafeLoader)
    hashes=[];rows=[]
    with path.open() as source,(output/'geometry.jsonl').open('w') as target:
        for line in source:
            try:
                row=json.loads(line)
            except ValueError:
                row=yaml.load(line,Loader=loader)  # pre-JSON historical evaluator output
            # yaml-cpp emits e.g. 8e-05, while YAML 1.1 resolves that as text.
            # Convert only schema-declared float fields, never IDs or digests.
            for mode in MODES:
                if mode not in row:continue
                m=row[mode]
                m['timing_ms']={k:float(v) for k,v in m['timing_ms'].items()}
                m['leveling']=[float(v) for v in m['leveling']]
                for obb in m['objects']:
                    for key in ('center','dimensions'):obb[key]=[float(v) for v in obb[key]]
                    obb['yaw']=float(obb['yaw'])
                    obb['corners']=[[float(v) for v in c] for c in obb['corners']]
            row['geometry_sha256']=hashlib.sha256(dump(geometry_only(row)).encode()).hexdigest()
            hashes.append(row['geometry_sha256']);rows.append(row)
            target.write(dump(row)+'\n')
    return rows,hashlib.sha256(''.join(hashes).encode()).hexdigest()


def summarize(rows, snapshots):
    groups=defaultdict(list)
    for r in rows:
        if 'baseline' in r and r['measurement']:groups[r['scene']].append(r)
    info={}
    for scene,group in sorted(groups.items()):
        summary={'scans':len(group),'changed_geometry':sum(
            mode_geometry(r['baseline'])!=mode_geometry(r['candidate']) for r in group)}
        for mode in MODES:
            summary[mode]={k:{str(p):percentile([r[mode][k] for r in group],p) for p in (50,95,99)}
                for k in ('protected_points','ground_removed','wall_removed','ceiling_removed','kept_points','clusters','final_objects','wall_ambiguous_skipped')}
            summary[mode]['timing_ms']={k:{str(p):percentile([r[mode]['timing_ms'][k] for r in group],p) for p in (50,95,99)}
                for k in ('protection','pipeline','obb','obb_filter')}
        summary['runtime_off_vs_old_offline_changed']=sum(
            mode_geometry(r['runtime_off'])!=mode_geometry(r['old_offline_no_protection']) for r in group)
        summary['candidate_minus_baseline']={k:{str(p):percentile([r['candidate'][k]-r['baseline'][k] for r in group],p) for p in (50,95,99)}
            for k in ('wall_removed','ground_removed','kept_points','clusters','final_objects')}
        info[scene]=summary
    valid=[r for r in rows if 'baseline' in r]
    return {'scans':len(rows),'geometry_scans':len(valid),'skipped':dict(Counter(
                r.get('skipped_reason') for r in rows if 'baseline' not in r)),
            'by_scene':info,'point_count_errors':sum(not r[m]['point_count_preserved'] for r in valid for m in MODES),
            'intensity_errors':sum(r[m]['intensity_preservation_errors'] for r in valid for m in MODES),
            'future_pose_uses':sum(e['used_future'] for s in snapshots for e in s['pose_evidence'].values()),
            'candidate_geometry_changed':sum(mode_geometry(r['baseline'])!=mode_geometry(r['candidate']) for r in valid),
            'timing_scope':'offline CPU stage timing; no ROS publication, tracker or live full-stack latency',
            'ground_truth_wall_accuracy_available':False,'deployment_accepted':False}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('bag','scene-manifest','decisions','setting','params','executable','output'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--snapshot-input',type=Path,help='Reuse immutable frozen inputs; no second bag reinterpretation')
    a=p.parse_args()
    for name in ('bag','scene_manifest','decisions','setting','params','executable'):
        if not getattr(a,name).is_file():p.error('missing '+name)
    if a.output.exists():p.error('output exists; preserve previous evidence')
    scenes=json.loads(a.scene_manifest.read_text())['scenes']
    if a.snapshot_input:
        snapshots=[json.loads(line) for line in a.snapshot_input.read_text().splitlines()]
        for s in snapshots:
            if s['geometry_valid'] and digest_file(a.snapshot_input.parent/s['cloud_path'])!=s['cloud_sha256']:
                raise ValueError('snapshot cloud SHA mismatch')
        a.output.mkdir(parents=True,exist_ok=False)
        snapshot_path=a.snapshot_input
    else:
        snapshots=snapshot(a.bag,scenes,a.decisions,a.output)
        snapshot_path=a.output/'snapshots.jsonl'
    subprocess.run([str(a.executable),str(snapshot_path),str(a.setting),str(a.params),
        str(a.output/'geometry.yaml_lines'),str(a.output/'voxel')],check=True,
        stdout=(a.output/'cpp.log').open('w'),stderr=subprocess.STDOUT)
    rows,geometry_hash=convert_geometry(a.output/'geometry.yaml_lines',a.output)
    summary=summarize(rows,snapshots)
    if summary['point_count_errors'] or summary['intensity_errors'] or summary['future_pose_uses']:
        raise AssertionError('input preservation/causality failed')
    (a.output/'summary.json').write_text(json.dumps(summary,sort_keys=True,indent=2)+'\n')
    manifest={'schema':'asmc-lidar-camera-geometry-v1','runtime_connected':False,
        'source_inputs':{k:{'path':str(getattr(a,k)),'sha256':digest_file(getattr(a,k))}
            for k in ('bag','scene_manifest','decisions','setting','params','executable')},
        'geometry_sha256':geometry_hash,'snapshots_sha256':digest_file(snapshot_path),
        'snapshots_path':str(snapshot_path),
        'point_origin':'sensor origin FLU from ROS PointCloud2, no mounting translation applied before legacy filter',
        'state_policy':'latest bag-arrived state frozen at scan arrival, no future or nearest IMU lookup',
        'limitations':['Explicit reprocessing, not exact restoration of unrecorded live internal state',
            'All scans preserved; legacy live callback skips are not recreated',
            'Stationary data cannot certify moving-object detection/tracking accuracy',
            'Filter labels are not ground truth'],
        'source_tools_sha256':{p.name:digest_file(p) for p in Path(__file__).parent.glob('*') if p.is_file()},
    }
    (a.output/'manifest.json').write_text(json.dumps(manifest,sort_keys=True,indent=2)+'\n')
    print(dump({k:v for k,v in summary.items() if k!='by_scene'}))


if __name__=='__main__':main()
