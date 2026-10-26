#!/usr/bin/env python3
"""C0 offline orchestration. No publishers, model inference or runtime policy edits.

Requires the standalone C++ probes, current lidar_replay and a registered output.
All geometry uses current C++ or camera production cores. Pose differences are
policy sensitivity, never independent accuracy. File outputs refuse overwrite.
"""
import argparse
from collections import Counter, defaultdict
import copy
import hashlib
import html
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/asmc_camera/src'))


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    with Path(path).open() as f:
        for line in f:
            yield json.loads(line)


def stats(values):
    v = sorted(float(x) for x in values if x is not None and math.isfinite(x))
    def percentile(q):
        at = (len(v)-1)*q; lo = int(at); hi = min(lo+1, len(v)-1)
        return v[lo] + (v[hi]-v[lo])*(at-lo)
    result = {'count': len(v)}
    if v:
        result.update({f'p{q}': percentile(q/100) for q in (50,95,99)})
        result['max'] = v[-1]
    return result


def timing_eligibility(a, b):
    """Eligibility only. In particular, no receive-source clock subtraction."""
    if not a or not b:
        return 'missing'
    if a.get('recording') != b.get('recording'):
        return 'different_recording'
    if not a.get('clock') or a.get('clock') != b.get('clock'):
        return 'different_clock'
    if a.get('session') != b.get('session'):
        return 'different_session'
    for key in ('duplicate', 'regression', 'conflict'):
        if a.get(key) or b.get(key):
            return key
    if not a.get('valid') or not b.get('valid'):
        return 'invalid'
    return 'comparable'


def pose_comparison(scan, state):
    if not scan.get('map', {}).get('valid') or not state.get('valid'):
        return {'reason':'pose_unavailable'}
    if state['arrival_ns'] != scan['arrival_ns'] or state['scan_ns'] != scan['stamp_ns']:
        raise ValueError('scan identity mismatch')
    m=scan['map']['map_from_base']; yaw=math.atan2(m[4],m[0])
    delta=math.atan2(math.sin(state['yaw']-yaw),math.cos(state['yaw']-yaw))
    dt=(state['state_ns']-scan['stamp_ns'])/1e6
    return dict(reason='same_time' if dt == 0 else 'asynchronous_policy_difference',
                dt_ms=dt, position_m=math.hypot(state['x']-m[3],state['y']-m[7]), yaw_rad=abs(delta))


def invariant(row):
    # Process identities/timing costs vary between runs; none are object behavior.
    result=copy.deepcopy(row)
    for key in ('producer','producer_session_id','session_id','timing','cost_ms'):
        result.pop(key,None)
    def clean(x):
        if isinstance(x,dict):
            for k in list(x):
                if k.endswith('_ms') and k in ('calculation_ms','comparison_ms','total_ms','wait_ms'):
                    del x[k]
                else: clean(x[k])
        elif isinstance(x,list):
            for a in x: clean(a)
    clean(result)
    return result


def run(command, log):
    with Path(log).open('xb') as f:
        subprocess.run([str(x) for x in command], stdout=f, stderr=subprocess.STDOUT, check=True)


def json_probe(executable, inputs, extra=()):
    result=subprocess.run([str(executable), *map(str,extra)],
                          input=''.join(json.dumps(x)+'\n' for x in inputs),
                          text=True,capture_output=True,check=True)
    rows=[json.loads(x) for x in result.stdout.splitlines()]
    if len(rows)!=len(inputs):raise ValueError('probe cardinality mismatch')
    return rows


def projection_cases(probe):
    import numpy as np
    import yaml
    from asmc_camera.bev.calibration import load_calibration_snapshot
    _, cameras=load_calibration_snapshot(ROOT/'src/asmc_camera/config/cameras_v3.yaml')
    mount=yaml.safe_load((ROOT/'src/asmc_lidar/config/autonav_v3.yaml').read_text())['sensor_pose_base_link']
    t=np.array([mount[k+'_m'] for k in ('x','y','z')]); cases=[]; lines=[]
    for view,cam in cameras.items():
        for angles in ([0,0,0],[4,-3,8]):
            r=cam.__class__(**{**vars(cam),'rotation_deg':tuple(angles)}).body_from_mount_rotation
            for roll,pitch in ((0,0),(9,-6),(-7,5)):
                for distance,y,z in ((5,0,-.35),(12,-1,.5),(30,2,1)):
                    base=np.array(cam.translation_m)+cam.body_from_mount_rotation@np.array([distance,y,z])
                    raw=r.T@(base-t);k=cam.intrinsic
                    expected=k@(cam.optical_from_body_rotation@base+cam.optical_from_body_translation)
                    cases.append((view,distance,roll,pitch,expected[:2]/expected[2]))
                    lines.append(' '.join(map(str,[*cam.translation_m,*cam.rotation_deg,*t,*angles,roll,pitch,*raw,k[0,0],k[1,1],k[0,2],k[1,2]])))
    p=subprocess.run([str(probe)],input='\n'.join(lines)+'\n',text=True,capture_output=True,check=True)
    values=np.array([[float(v) for v in l.split()] for l in p.stdout.splitlines()]).reshape(-1,5,3)
    records=[]
    for c,v in zip(cases,values):
        errors=np.linalg.norm(v[:,:2]/v[:,2:3]-c[4],axis=1)
        records.append(dict(view=c[0],distance_m=c[1],roll=c[2],pitch=c[3],errors_px=dict(zip(
            ['raw','leveled','missing_inverse','double_inverse','wrong_attitude'],errors.tolist()))))
    positive=max(r['errors_px'][k] for r in records for k in ('raw','leveled'))
    negative={k:min(r['errors_px'][k] for r in records if r['roll'] or r['pitch']) for k in ('missing_inverse','double_inverse','wrong_attitude')}
    return dict(passed=positive<1e-8 and min(negative.values())>1,maximum_error_px=positive,
                negative_control_min_px=negative,cases=records)


def bag_inventory(path):
    import rosbag
    with rosbag.Bag(str(path)) as bag:
        return {t:dict(type=v.msg_type,count=v.message_count) for t,v in bag.get_type_and_topic_info().topics.items()}


def header_matches(path, scans):
    """Causal RGB availability at scan receipt. ±50ms header window, no future RGB."""
    import rosbag
    topics={'/image_jpeg/compressed':'front','/cam_left/image_jpeg/compressed':'left','/cam_right/image_jpeg/compressed':'right'}
    from collections import deque
    queues={v:deque(maxlen=8) for v in topics.values()}; out=[]
    with rosbag.Bag(str(path)) as bag:
        events=iter(bag.read_messages(topics=list(topics))); current=next(events,None)
        for s in scans:
            arrival=s['arrival_ns']
            while current and current.timestamp.to_nsec()<=arrival:
                msg=current.message;queues[topics[current.topic]].append((msg.header.stamp.to_nsec(),current.timestamp.to_nsec()))
                current=next(events,None)
            for view,q in queues.items():
                best=min(q,key=lambda p:abs(p[0]-s['stamp_ns'])) if q else None
                delta=(best[0]-s['stamp_ns'])/1e6 if best else None
                out.append(dict(seq=s['seq'],view=view,dt_ms=delta,reason='missing_view' if best is None else
                    ('header_match_only' if abs(delta)<=50 else 'outside_50ms'),
                    acquisition_time_comparable=False,object_matching='no_camera_observations_in_bag'))
    return out


def state_summary(scans,states):
    if len(scans)!=len(states):raise ValueError('state/scan cardinality')
    rows=[dict(seq=s['seq'],**pose_comparison(s,p)) for s,p in zip(scans,states)]
    return rows,dict(reasons=dict(Counter(r['reason'] for r in rows)),
                    sample_delta_ms=stats(r.get('dt_ms') for r in rows),
                    asynchronous_position_m=stats(r.get('position_m') for r in rows if r['reason']=='asynchronous_policy_difference'),
                    asynchronous_yaw_rad=stats(r.get('yaw_rad') for r in rows if r['reason']=='asynchronous_policy_difference'),
                    same_time_position_m=stats(r.get('position_m') for r in rows if r['reason']=='same_time'))


def sensor_message(raw):
    class Stamp(int):
        def to_nsec(self):return int(self)
    def convert(k,v):
        if isinstance(v,dict):return SimpleNamespace(**{x:convert(x,y) for x,y in v.items()})
        if k in ('stamp','published_stamp'):return Stamp(v)
        return v
    return SimpleNamespace(**{k:convert(k,v) for k,v in raw.items()})


def camera_review(frames_path,events_path):
    from asmc_camera.runtime.sensor_timing import ImuTimeline
    frames=list(read(frames_path));queries=[]
    for frame in frames:
        for view,row in frame['views'].items():
            timing=row['lineage']['sensor_timing']
            for delay in (0,10,100):
                queries.append((timing['publish_monotonic_ns']/1e9+delay/1000,frame['index'],view,delay,timing))
    events=iter(read(events_path));event=next(events,None);clock=[0.]
    timeline=ImuTimeline(clock=lambda:clock[0]);records=[];last_arrival=-1.
    for when,index,view,delay,timing in sorted(queries):
        while event and event['received_monotonic']<=when:
            t=event['received_monotonic']
            if t<last_arrival:raise ValueError('camera arrival regression; do not sort away')
            last_arrival=t;clock[0]=t;msg=sensor_message(event['message'])
            if event['kind']=='imu_timing':timeline.add_timing(msg)
            elif event['kind']=='imu':
                h=msg.header;q=msg.orientation
                timeline.add_imu(('/imu',h.frame_id,h.stamp.to_nsec(),h.seq),(q.x,q.y,q.z,q.w),h.frame_id)
            event=next(events,None)
        clock[0]=when;pose,reason=timeline.attitude(sensor_message(timing))
        records.append(dict(index=index,view=view,query_after_publish_ms=delay,reason=reason,
                            source_ns=timing['source_sec']*10**9+timing['source_nsec'],
                            session=timing['bridge_session_id'],clock=timing['source_clock_domain'],
                            bracket_ms=pose.bracket_ms if pose else None,
                            quaternion=list(pose.quaternion) if pose else None))
    return records,dict(frames=len(frames),views=len(frames)*3,query='bridge publication + delay; not recorded worker start',
        delays={str(d):dict(reasons=dict(Counter(r['reason'] for r in records if r['query_after_publish_ms']==d)),
                           bracket_ms=stats(r['bracket_ms'] for r in records if r['query_after_publish_ms']==d)) for d in (0,10,100)},
        counters=timeline.snapshot()['counters'])


def write_json(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')


def write_rows(path,rows):
    with Path(path).open('x') as f:
        for row in rows:f.write(json.dumps(row,allow_nan=False)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--storage-root',type=Path,required=True)
    p.add_argument('--run-id',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--probes',type=Path,required=True);p.add_argument('--replay',type=Path,required=True)
    args=p.parse_args(); manifest=json.loads(args.manifest.read_text())
    if args.output.name!=args.run_id:raise ValueError('run ID/output mismatch')
    if args.output.exists():raise ValueError('output exists; use a newly registered run')
    for path,digest in manifest['configs'].items():
        if sha(ROOT/path)!=digest:raise ValueError('changed config: '+path)
    inputs={}
    for item in manifest['inputs']:
        path=args.storage_root/item['path']
        path.resolve().relative_to(args.storage_root.resolve())
        if sha(path)!=item['sha256']:raise ValueError('input SHA mismatch: '+item['id'])
        inputs[item['id']]=path
    args.output.mkdir(parents=True,exist_ok=False)
    source=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip()
    dirty=subprocess.check_output(['git','-C',str(ROOT),'diff','HEAD','--binary'])
    (args.output/'source.patch').write_bytes(dirty)
    summary=dict(schema='asmc.frame_time_audit.v1',run_id=args.run_id,source_revision=source,
                 manifest_sha256=sha(args.manifest),patch_sha256=hashlib.sha256(dirty).hexdigest(),
                 manifest=manifest,bag={},limitations=['Receive-only bag headers are not measurement stamps.',
                 'No cross-recording joins. No independent GT.',
                 'Camera publication queries are causal sensitivity, not measured worker timing.'])
    summary['projection']=projection_cases(args.probes/'frame_projection_probe')
    for name in ('normal','tunnel'):
        path=inputs[name]; directory=args.output/name;directory.mkdir()
        inventory=bag_inventory(path)
        command=[args.replay,'--bag',path,'--config',ROOT/'src/asmc_lidar/config/lidar_pipeline.yaml',
                 '--package-share',ROOT/'src/asmc_lidar','--planning-output',directory/'planning.bag']
        run(command+['--output',directory/'lidar.jsonl'],directory/'replay.log')
        # Independent second execution: audit probes cannot alter runtime results.
        repeat=[x for x in command[:-2]]
        run(repeat+['--output',directory/'repeat.jsonl'],directory/'repeat.log')
        scans=list(read(directory/'lidar.jsonl')); repeats=list(read(directory/'repeat.jsonl'))
        same=len(scans)==len(repeats) and all(invariant(a)==invariant(b) for a,b in zip(scans,repeats))
        result=dict(topics=inventory,scans=len(scans),replay_equal=same,
                    local_status=dict(Counter(str(s['typed']['local'].get('status','not_accepted')) for s in scans)),
                    map_reasons=dict(Counter(s.get('map',{}).get('reason','input_unavailable') for s in scans)),
                    imu_age_ms=stats((s['stamp_ns']-s['imu_ns'])/1e6 for s in scans if s.get('imu_ns')),
                    gps_age_ms=stats((s['stamp_ns']-g)/1e6 for s in scans for g in s.get('map',{}).get('gps_stamps',[])))
        for kind,pkg in (('planning','planning_inputs'),('bridge','asmc_bridge')):
            run([args.probes/(kind+'_state_probe'),path,ROOT/'src'/pkg/'config/vehicle_state.yaml',directory/(kind+'.jsonl')],directory/(kind+'.log'))
            states=list(read(directory/(kind+'.jsonl')));rows,result[kind]=state_summary(scans,states)
            write_rows(directory/(kind+'-comparison.jsonl'),rows)
        matches=header_matches(path,scans);write_rows(directory/'image-matches.jsonl',matches)
        result['images']={v:dict(reasons=dict(Counter(r['reason'] for r in matches if r['view']==v)),
                                dt_ms=stats(r['dt_ms'] for r in matches if r['view']==v)) for v in ('front','left','right')}
        summary['bag'][name]=result
    camera_rows,summary['camera']=camera_review(inputs['camera_frames'],inputs['camera_events'])
    write_rows(args.output/'camera-timing.jsonl',camera_rows)
    # Optional rendering is part of the tool, with no ROS/GPU process.
    from frame_time.gallery import render
    summary['gallery']=render(args,inputs,camera_rows)
    summary['passed']=summary['projection']['passed'] and all(x['replay_equal'] for x in summary['bag'].values())
    write_json(args.output/'summary.json',summary)
    (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>C0 frame/time</title>'
        '<style>body{font:16px sans-serif;max-width:1200px;margin:2em auto;background:#181d24;color:#eee}a{color:#8df}img{max-width:100%}pre{white-space:pre-wrap}</style>'
        '<h1>C0 좌표·시각 대조</h1><p>정확도 GT가 없는 정책 민감도 비교. 서로 다른 기록은 연결하지 않음.</p>'
        '<p><a href="gallery.html">RGB·OBB·지도 갤러리</a> · <a href="summary.json">전체 수치</a></p><pre>'+html.escape(json.dumps(summary,indent=2,ensure_ascii=False))+'</pre>')
    print(json.dumps(dict(passed=summary['passed'],output=str(args.output))))
    return 0 if summary['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
