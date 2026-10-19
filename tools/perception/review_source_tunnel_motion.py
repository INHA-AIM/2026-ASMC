#!/usr/bin/env python3
"""Source-clock review of captured tunnel motion, reusing the fixed fit audit.

Raw IMU and metadata must join exactly on wire header. Native RPC timestamps are
kept as a development pose reference, never a competition motion input or GT.
"""
import argparse
from collections import Counter,defaultdict
from copy import deepcopy
import html
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from asmc_camera.bev.calibration import calibrations_from_dict
from asmc_camera.bev.imu_projection import attitude_camera,correct_evidence
from asmc_camera.bev.model_grid_projection import ModelGridGroundProjector,MetricEvidence
from asmc_camera.runtime.sensor_timing import quaternion_rotation
from review_family_motion import interpolate,rpy,sha,stats
from review_tunnel_imu_motion import read,fit,family_distances,temporal,render,VIEWS


def source_signals(capture,poses):
    messages={};timing={};counts=Counter();sessions=set()
    for event in read(capture/'events.jsonl'):
        m=event['message']
        if event['kind']=='imu':
            h=m['header'];key=h['stamp'],h['seq'],h['frame_id']
            if key in messages:raise ValueError('ambiguous IMU wire key')
            messages[key]=m
        elif event['kind']=='imu_timing':
            key=m['published_stamp'],m['sensor_wire_sequence'],m['sensor_frame_id']
            if key in timing:raise ValueError('ambiguous timing wire key')
            timing[key]=m;sessions.add(m['bridge_session_id'])
    if len(sessions)!=1:raise ValueError('motion review requires one bridge session')
    unique={}
    for key,m in messages.items():
        t=timing.get(key)
        if t is None:counts['missing_timing']+=1;continue
        if t['source_conflict'] or t['source_regression']:
            raise ValueError('source conflict/regression: split the recording before motion interpolation')
        if (not t['sensor_wire_sequence_valid'] or not t['source_valid'] or
            t['source_clock_domain']!='morai_sensor' or t['source_duplicate'] or
            t['source_sec']<=0 or not 0<=t['source_nsec']<10**9):counts['excluded_source']+=1;continue
        if m['orientation_covariance'][0]<0:counts['invalid_orientation']+=1;continue
        ns=t['source_sec']*10**9+t['source_nsec'];q=[m['orientation'][k] for k in 'xyzw']
        try:quaternion_rotation(q)
        except ValueError:counts['invalid_quaternion']+=1;continue
        if ns in unique and unique[ns]['q_xyzw']!=q:raise ValueError('conflicting source pose')
        unique[ns]=dict(source_ns=ns,q_xyzw=q)
    imu=sorted(unique.values(),key=lambda r:r['source_ns'])
    rpc={};offsets=[];attitude_error=[]
    for row in poses:
        ns=row['rpc_timestamp_raw']*1000
        offset=(ns-(row['ros_request_ns']+row['ros_receive_ns'])/2)/1e6
        if abs(offset)>500:raise ValueError('RPC timestamp is not compatible with observed Unix microseconds')
        offsets.append(offset);s=row['state'];p=s['transform']['location'];rotation=s['transform']['rotation']
        value=dict(source_ns=ns,position_xyz=[p.get(k,0.) for k in 'xyz'],heading_deg=rotation.get('z',0.),phase=row['phase'])
        if ns in rpc and (rpc[ns]['position_xyz']!=value['position_xyz'] or
                          rpc[ns]['heading_deg']!=value['heading_deg']):
            raise ValueError('conflicting RPC pose at same timestamp')
        rpc[ns]=value
        im=interpolate(imu,ns,'imu')
        if im:
            angle=np.rad2deg(rpy(im['q_xyzw']))
            attitude_error.append(np.abs((angle-np.array([rotation.get(k,0.) for k in 'xyz'])+180)%360-180))
    return imu,sorted(rpc.values(),key=lambda r:r['source_ns']),dict(
        unique_imu=len(imu),unique_rpc=len(rpc),counts=dict(counts),bridge_session_id=next(iter(sessions)),
        rpc_source_interpretation='Raw integer empirically compatible with Unix microseconds; development reference only.',
        rpc_timestamp_minus_request_midpoint_ms=stats(offsets),
        rpc_imu_attitude_abs_delta_deg={k:stats([v[i] for v in attitude_error]) for i,k in enumerate(('roll','pitch','yaw'))})


def main(args):
    args.output.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1)
    capture=args.trial/'capture';poses=read(args.trial/'poses.jsonl')
    imu,ego,signal_report=source_signals(capture,poses)
    frames=read(capture/'frames.jsonl');cfg=yaml.safe_load((args.config/'bev/lane_fitting.yaml').read_text())
    cameras=calibrations_from_dict(yaml.safe_load((args.config/'cameras_v3.yaml').read_text()))
    projectors={v:ModelGridGroundProjector(cameras[v],model_hw=(544,960) if v=='front' else (480,640),
        source_mask=1<<i,max_ground_range_m=cfg['max_ground_range_m'],
        x_bounds_m=(cfg['evidence_x_min_m'],cfg['evidence_x_max_m']),
        y_bounds_m=(cfg['evidence_y_min_m'],cfg['evidence_y_max_m'])) for i,v in enumerate(VIEWS)}
    configs={'M0':cfg,'M1_source':cfg}
    for name,key in [('F_width0','width_prior_strength'),('F_smooth0','spline_smoothing')]:
        configs[name]=deepcopy(cfg);configs[name]['shared_lane_family'][key]=0.
    if args.joint_width0:
        configs['M1_source_width0']=deepcopy(cfg)
        configs['M1_source_width0']['shared_lane_family']['width_prior_strength']=0.
    records=[];gallery=[];rgb_index={};skipped=Counter();phase_saved=set()
    for number,frame in enumerate(frames):
        source={v:frame['views'][v]['lineage']['sensor_timing'] for v in VIEWS}
        times={v:source[v]['source_sec']*10**9+source[v]['source_nsec'] for v in VIEWS}
        if any(not frame['views'][v]['lineage']['source_timing_valid'] for v in VIEWS):skipped['invalid_source']+=1;continue
        if any(s['bridge_session_id']!=signal_report['bridge_session_id'] for s in source.values()):raise ValueError('bridge session mismatch')
        if max(times.values())-min(times.values())>35e6:skipped['source_skew']+=1;continue
        quats={v:interpolate(imu,times[v],'imu') for v in VIEWS}
        if not all(quats.values()):skipped['no_source_bracket']+=1;continue
        phase=min(ego,key=lambda e:abs(e['source_ns']-times['front']))['phase']
        corrected={v:attitude_camera(cameras[v],quats[v]['q_xyzw']) for v in VIEWS}
        arrays=dict(np.load(capture/frame['grid']))
        maps={v:{label:cv2.distanceTransform((arrays[v+'_class']!=label).astype(np.uint8),cv2.DIST_L2,5) for label in (1,2)} for v in VIEWS}
        base={v:projectors[v].extract(arrays[v+'_class'],arrays[v+'_confidence'],{1:.5,2:.5}) for v in VIEWS}
        variants={}
        for name,config in configs.items():
            motion=name.startswith('M1_source')
            used=(base if not motion else {v:correct_evidence(projectors[v],base[v],corrected[v]) for v in VIEWS})
            family,nodes,audit=fit(MetricEvidence.concatenate(list(used.values())),config)
            family['semantic_model_px']=family_distances(family,corrected if motion else cameras,projectors,arrays,maps)
            variants[name]=family
        row=dict(run_id='live-motion-r1',phase=phase,index=number,stamp_ns=times['front'],
            roll_pitch_deg=np.rad2deg(rpy(quats['front']['q_xyzw']))[:2].tolist(),variants=variants,
            grid_sha256=sha(capture/frame['grid']),image_source_ns=times)
        records.append(row)
        if phase not in phase_saved and number>10:
            for v in VIEWS:rgb_index[row['run_id'],row['stamp_ns'],v]=dict(path=frame['views'][v]['rgb'],sha256=frame['views'][v]['rgb_sha256'])
            display=('M0','M1_source','M1_source_width0','F_width0') if args.joint_width0 else tuple(configs)
            gallery.append(render(args.trial,row,variants,cameras,corrected,arrays,None,rgb_index,args.output,
                                  names=display,rgb_root=capture));phase_saved.add(phase)
        if (number+1)%40==0:print('source tunnel review',number+1,'/',len(frames),flush=True)
    pairs=temporal(records,{'live-motion-r1':{'/Ego_topic':ego}},variants=tuple(configs))
    medians=defaultdict(lambda:defaultdict(list))
    for r in records:
        for name,value in r['variants'].items():
            for key,s in value['semantic_model_px'].items():
                if s['count']:medians[name][key].append(s['p50'])
    summary=dict(schema='source_time_tunnel_motion_review_v1',source_clock='morai_sensor',tool_sha256=sha(__file__),
        joint_width0=args.joint_width0,
        code_sha256={p:sha(Path(__file__).with_name(p)) for p in ('review_tunnel_imu_motion.py','review_family_motion.py','summarize_family_motion.py')},
        config_sha256={p:sha(args.config/p) for p in ('cameras_v3.yaml','bev/lane_fitting.yaml','camera/lane_runtime.yaml')},
        pins={str(p):sha(p) for p in [capture/'frames.jsonl',capture/'events.jsonl',args.trial/'poses.jsonl']},
        input_frames=len(frames),frames=len(records),skipped=dict(skipped),signals=signal_report,
        roll_pitch_deg={k:stats([r['roll_pitch_deg'][i] for r in records]) for i,k in enumerate(('roll','pitch'))},
        validity={n:dict(available=len(records),valid=sum(r['variants'][n]['valid'] for r in records)) for n in configs},
        family_pixel_distances={n:{k:stats(v) for k,v in d.items()} for n,d in medians.items()},
        matched_temporal_pairs=len(pairs),
        matched_temporal_lateral_change_m={phase:{n:stats([p['variants'][n]['p95'] for p in pairs
            if (phase=='all' or p['phase']==phase) and p['variants'][n].get('count')]) for n in configs}
            for phase in ('all','accelerate','hold','brake','rest','accelerate2','steer','brake2')},
        independent_gt=False,default_changed=False,
        note='Same-source camera/IMU interpolation; no latest/extrapolated pose. Common x/roles are valid in both frames and all variants. '
             'Semantic distances are internal consistency; RPC Ego is development reference with retained native timestamp, not competition input.')
    (args.output/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    (args.output/'temporal-pairs.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in pairs))
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    labels='M0 / source IMU / IMU+폭 사전값 제약 해제 / 고정 평면+폭 사전값 제약 해제' if args.joint_width0 else 'M0 / source IMU / 폭 사전값 제약 해제 / smoothing 해제'
    page='<!doctype html><meta charset="utf-8"><title>원본 시각 터널 운동·family 대조</title><style>body{background:#14202a;color:white;font:16px sans-serif;margin:24px;max-width:1920px}img{width:100%}pre{white-space:pre-wrap}a{color:#8df}</style><h1>원본 시각 터널 가감속·조향 대조</h1><p>A126, 약20km/h. 열: '+labels+'. 같은 RGB에 각 후보의 자세로 재투영했습니다. 독립 GT 정확도·운영 수락 시험은 아닙니다.</p>'
    page+=''.join('<figure><figcaption>'+html.escape(p)+'</figcaption><img loading="lazy" src="'+html.escape(p)+'"></figure>' for p in gallery)
    page+='<p><a href="summary.json">집계</a> · <a href="frames.jsonl">프레임별 비교</a></p><pre>'+html.escape(json.dumps(summary,indent=2,ensure_ascii=False))+'</pre>'
    (args.output/'index.html').write_text(page)
    print(json.dumps({k:summary[k] for k in ('frames','skipped','validity','matched_temporal_pairs','matched_temporal_lateral_change_m')},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trial',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--joint-width0',action='store_true')
    main(p.parse_args())
