#!/usr/bin/env python3
"""Same-input tunnel motion/fit ablations; old bag headers are RECEIVE times.

No ROS publishers, defaults, network calls, model runs, or training. Cached Large
grids are reused. Semantic distance is internal consistency, not independent GT.
"""
import argparse
from collections import defaultdict
from copy import deepcopy
import html
import json
from pathlib import Path
import time

import cv2
import numpy as np
import yaml

from asmc_camera.bev.calibration import calibrations_from_dict
from asmc_camera.bev.imu_projection import attitude_camera, correct_evidence
from asmc_camera.bev.model_grid_projection import MetricEvidence, ModelGridGroundProjector
from asmc_camera.bev.lane_evidence import reduce_evidence_to_nodes
from asmc_camera.bev.lane_tracking import associate_lane_nodes_with_audit
from asmc_camera.bev.boundary_candidates import select_shadow_lane_tracks_with_audit
from asmc_camera.bev.shared_lane_family import fit_shared_lane_family_with_audit
from review_family_motion import family_dict, interpolate, overlay, rpy, sha, stats
from summarize_family_motion import sample_boundary

VIEWS=('front','left','right')
MOTION=('M0','M1_receive','M1_minus40ms','M1_plus40ms')
ABLATIONS=('F_width0','F_smooth0')


def read(path):
    with path.open() as stream:return list(map(json.loads,stream))


def mask_distances(xy, class_id, camera, geometry, classes, distance_maps=None):
    """Distances at valid projected points, keeping each class separately."""
    points=np.asarray(xy,dtype=float).reshape(-1,2)
    ids=np.asarray(class_id,dtype=int)
    if not len(points):return {}
    uv,depth=camera.project_ground(points)
    uv=uv*np.asarray(geometry.scale_xy)+np.asarray(geometry.pad_ltrb[:2])
    finite=np.isfinite(uv).all(axis=1)
    integer=np.zeros(uv.shape,dtype=int);integer[finite]=np.rint(uv[finite]).astype(int)
    good=(finite&(depth>.1)&(integer[:,0]>=0)&(integer[:,1]>=0)&
          (integer[:,0]<classes.shape[1])&(integer[:,1]<classes.shape[0]))
    result={}
    for label in (1,2):
        if not np.any(classes==label):continue
        distance=(distance_maps[label] if distance_maps is not None else
                  cv2.distanceTransform((classes!=label).astype(np.uint8),cv2.DIST_L2,5))
        at=integer[good&(ids==label)]
        if len(at):result[str(label)]=distance[at[:,1],at[:,0]].astype(float).tolist()
    return result


def fit(evidence,cfg):
    nodes=reduce_evidence_to_nodes(evidence,cfg['x_bin_m'],cfg['lateral_cluster_gap_m'],cfg['evidence_x_min_m'])
    tracks,_=associate_lane_nodes_with_audit(nodes,**{key:cfg[key] for key in
        ('max_track_gap_m','max_lateral_residual_m','max_heading_change_deg','min_track_nodes','min_observed_span_m')})
    selected,_=select_shadow_lane_tracks_with_audit(tracks,cfg['shadow_boundary'])
    family,audit=fit_shared_lane_family_with_audit(selected,cfg['shared_lane_family'])
    return family_dict(family,audit),nodes,audit


def family_distances(family,cameras,projectors,arrays,distance_maps):
    measured=defaultdict(list)
    for state,label in ((1,'observed'),(2,'inferred')):
        for boundary in family['boundaries']:
            xy=np.asarray(boundary['xy']);support=np.asarray(boundary['support'])
            keep=(support==state)&(xy[:,0]>=0)&(xy[:,0]<=15)
            for view in VIEWS:
                distances=mask_distances(xy[keep],np.full(int(keep.sum()),boundary['class_id']),
                    cameras[view],projectors[view].geometry,arrays[view+'_class'],distance_maps[view])
                for cls,values in distances.items():measured[label+'/'+view+'/'+cls].extend(values)
    return {k:stats(v) for k,v in measured.items()}


def stage_distances(evidence_by_view,nodes,cameras,projectors,arrays,distance_maps):
    result={}
    for i,view in enumerate(VIEWS):
        # Reprojection of the original selected rays should land back on the
        # same segmentation. Merged centroid displacement is a separate stage.
        evidence=evidence_by_view[view]
        for cls,values in mask_distances(evidence.xy[::8],evidence.class_id[::8],cameras[view],
                projectors[view].geometry,arrays[view+'_class'],distance_maps[view]).items():result['raw/'+view+'/'+cls]=stats(values)
        chosen=[n for n in nodes if n.source_mask&(1<<i)]
        for cls,values in mask_distances([n.xy for n in chosen],[n.class_id for n in chosen],cameras[view],
                projectors[view].geometry,arrays[view+'_class'],distance_maps[view]).items():result['nodes/'+view+'/'+cls]=stats(values)
    return result


def temporal(records,signals,variants=MOTION):
    previous={};pairs=[];query=np.arange(.5,14.5001,.25)
    for row in records:
        key=row['run_id'],row['phase'];old=previous.get(key);previous[key]=row
        if old is None or not 0<row['stamp_ns']-old['stamp_ns']<=200e6:continue
        poses=[interpolate(signals[row['run_id']]['/Ego_topic'],r['stamp_ns'],'pose') for r in (old,row)]
        if any(p is None for p in poses):continue
        changes=defaultdict(list)
        for role in ('left_outer_boundary','yellow_centre_boundary','right_outer_boundary'):
            values={}
            for name in variants:
                if name not in old['variants'] or name not in row['variants']:break
                a=next((b for b in old['variants'][name]['boundaries'] if b['role']==role),None)
                b=next((b for b in row['variants'][name]['boundaries'] if b['role']==role),None)
                if a is None or b is None:break
                values[name]=np.abs(sample_boundary(b,query)-sample_boundary(a,query,*poses))
            if len(values)!=len(variants):continue
            keep=np.logical_and.reduce([np.isfinite(v) for v in values.values()])
            for name,v in values.items():changes[name].extend(v[keep].tolist())
        if changes:pairs.append(dict(run_id=row['run_id'],phase=row['phase'],
            dt_ms=(row['stamp_ns']-old['stamp_ns'])/1e6,variants={k:stats(v) for k,v in changes.items()}))
    return pairs


def render(directory,row,variants,cameras,corrected,arrays,nodes,index,output,
           names=('M0','M1_receive','F_width0','F_smooth0'),rgb_root=None):
    panels=[]
    for view in ('left','front','right'):
        rgb=index[row['run_id'],row['stamp_ns'],view]
        path=(directory/'inputs' if rgb_root is None else rgb_root)/rgb['path']
        if sha(path)!=rgb['sha256']:raise ValueError('RGB SHA mismatch')
        source=cv2.imread(str(path));line=[]
        for name in names:
            canvas=overlay(source,variants[name],corrected[view] if name.startswith('M1') else cameras[view])
            canvas=cv2.resize(canvas,(480,270))
            cv2.putText(canvas,view+' '+name,(5,23),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,120,255),2)
            line.append(canvas)
        panels.append(np.hstack(line))
    name=row['run_id']+'-'+row['phase']+'-%d.jpg'%row['index']
    cv2.imwrite(str(output/name),np.vstack(panels))
    return name


def main(args):
    cv2.setNumThreads(1);args.output.mkdir(parents=True,exist_ok=False)
    directory=args.inputs
    cfg=yaml.safe_load((args.config/'bev/lane_fitting.yaml').read_text())
    cameras=calibrations_from_dict(yaml.safe_load((args.config/'cameras_v3.yaml').read_text()))
    projectors={v:ModelGridGroundProjector(cameras[v],model_hw=(544,960) if v=='front' else (480,640),
        source_mask=1<<i,max_ground_range_m=cfg['max_ground_range_m'],
        x_bounds_m=(cfg['evidence_x_min_m'],cfg['evidence_x_max_m']),
        y_bounds_m=(cfg['evidence_y_min_m'],cfg['evidence_y_max_m'])) for i,v in enumerate(VIEWS)}
    pins=json.loads((directory/'inputs/summary.json').read_text())
    if sha(directory/'inputs/signals.jsonl')!=pins['signals_sha256']:raise ValueError('signals SHA mismatch')
    policies=json.loads((directory/'bag-stamp-policy.json').read_text())
    if any(stream.get('timestamp_mode')!='receive' for run in policies.values() for stream in run.values()):
        raise ValueError('this review explicitly expects receive-header legacy bags')
    signals=defaultdict(lambda:defaultdict(list))
    for row in read(directory/'inputs/signals.jsonl'):signals[row['run_id']][row['topic']].append(row)
    for topics in signals.values():
        for rows in topics.values():rows.sort(key=lambda r:r['source_ns'])
    frames=read(directory/'continuous-grids/manifest.jsonl')
    index={(r['run_id'],r['front_ns'],r['view']):r for r in read(directory/'inputs/rgb-index.jsonl')}
    configs={'M0':cfg,'M1_receive':cfg,'M1_minus40ms':cfg,'M1_plus40ms':cfg}
    for name,key in [('F_width0','width_prior_strength'),('F_smooth0','spline_smoothing')]:
        configs[name]=deepcopy(cfg);configs[name]['shared_lane_family'][key]=0.
    records=[];gallery=[];costs=defaultdict(list)
    for number,frame in enumerate(frames):
        path=directory/'continuous-grids'/frame['grid']
        if sha(path)!=frame['grid_sha256']:raise ValueError('grid SHA mismatch')
        arrays=dict(np.load(path));variants={};stage={};corrected=None;baseline_nodes=None
        distance_maps={v:{label:cv2.distanceTransform((arrays[v+'_class']!=label).astype(np.uint8),cv2.DIST_L2,5)
                         for label in (1,2)} for v in VIEWS}
        base={v:projectors[v].extract(arrays[v+'_class'],arrays[v+'_confidence'],{1:.5,2:.5}) for v in VIEWS}
        stamp=frame['views']['front']['header_ns']
        pose_rows={v:interpolate(signals[frame['run_id']]['/imu'],frame['views'][v]['header_ns'],'imu') for v in VIEWS}
        rollpitch=None
        if pose_rows['front']:rollpitch=np.rad2deg(rpy(pose_rows['front']['q_xyzw']))[:2].tolist()
        for name,variant_cfg in configs.items():
            used_cameras=cameras;used=base
            if name.startswith('M1'):
                offset= -40_000_000 if name=='M1_minus40ms' else 40_000_000 if name=='M1_plus40ms' else 0
                quats={v:interpolate(signals[frame['run_id']]['/imu'],frame['views'][v]['header_ns']+offset,'imu') for v in VIEWS}
                if not all(quats.values()):continue
                used_cameras={v:attitude_camera(cameras[v],quats[v]['q_xyzw']) for v in VIEWS}
                used={v:correct_evidence(projectors[v],base[v],used_cameras[v]) for v in VIEWS}
                if name=='M1_receive':corrected=used_cameras
            start=time.perf_counter();family,nodes,audit=fit(MetricEvidence.concatenate(list(used.values())),variant_cfg)
            costs[name].append(1000*(time.perf_counter()-start))
            family['semantic_model_px']=family_distances(family,used_cameras,projectors,arrays,distance_maps)
            variants[name]=family
            if name in ('M0','M1_receive'):
                stage[name]=stage_distances(used,nodes,used_cameras,projectors,arrays,distance_maps)
            if name=='M0':baseline_nodes=nodes
        row=dict(run_id=frame['run_id'],phase=frame['phase'],index=number,stamp_ns=stamp,
            grid_sha256=frame['grid_sha256'],roll_pitch_deg=rollpitch,variants=variants,stages=stage)
        records.append(row)
        if frame['sequence_index']==6 and corrected and all(n in variants for n in ('M0','M1_receive',*ABLATIONS)):
            gallery.append(render(directory,row,variants,cameras,corrected,arrays,baseline_nodes,index,args.output))
        if (number+1)%36==0:print('tunnel same-input review',number+1,'/',len(frames),flush=True)
    pairs=temporal(records,signals)
    def dist_summary(section,phase=None):
        accumulated=defaultdict(lambda:defaultdict(list))
        for row in records:
            if phase and row['phase']!=phase:continue
            for name,value in row[section].items():
                values=value['semantic_model_px'] if section=='variants' else value
                for k,s in values.items():
                    if s['count']:accumulated[name][k].append(s['p50'])
        return {n:{k:stats(v) for k,v in values.items()} for n,values in accumulated.items()}
    summary=dict(schema='tunnel_receive_time_imu_fit_review_v1',frames=len(records),tool_sha256=sha(__file__),
        timing_basis='legacy receive headers; no raw source timing; offset variants are sensitivity only',
        input_pins=pins,clock_policy_sha256=sha(directory/'bag-stamp-policy.json'),
        grid_manifest_sha256=sha(directory/'continuous-grids/manifest.jsonl'),
        configs={str(p):sha(args.config/p) for p in ('cameras_v3.yaml','bev/lane_fitting.yaml')},
        roll_pitch_deg={k:stats([r['roll_pitch_deg'][i] for r in records if r['roll_pitch_deg']]) for i,k in enumerate(('roll','pitch'))},
        validity={name:dict(available=sum(name in r['variants'] for r in records),
             valid=sum(r['variants'].get(name,{}).get('valid',False) for r in records)) for name in configs},
        same_input_fit_ms={k:stats(v) for k,v in costs.items()},
        stage_pixel_distances=dist_summary('stages'),family_pixel_distances=dist_summary('variants'),
        by_phase={phase:dict(family_pixel_distances=dist_summary('variants',phase)) for phase in ('entry','interior','exit')},
        matched_temporal_pairs=len(pairs),
        matched_temporal_lateral_change_m={phase:{name:stats([p['variants'][name]['p95'] for p in pairs
          if (phase=='all' or p['phase']==phase) and p['variants'].get(name,{}).get('count')]) for name in MOTION}
          for phase in ('all','entry','interior','exit')},
        independent_gt=False,default_changed=False,
        note='Each aggregate pixel statistic is a distribution of per-frame medians. Observed and inferred samples are separate. '
             'Temporal comparisons use identical x/roles valid for both frames and all four motion variants, aligned by development Ego poses.')
    (args.output/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    (args.output/'temporal-pairs.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in pairs))
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    page='<!doctype html><meta charset="utf-8"><title>터널 주행 IMU·family 밀착도 대조</title><style>body{font:16px sans-serif;max-width:1920px;margin:24px;background:#14202a;color:white}img{width:100%}pre{white-space:pre-wrap}a{color:#8df}</style><h1>터널 주행: IMU·family 밀착도 대조</h1>'
    page+='<p>6개 bag · SUNNY/FOGGY × 11/13/15시. 기존 수신 시각 기준이며 원본 시각 정밀 검증이 아닙니다. 같은 RGB·Large 그리드·mask를 사용합니다. 독립 GT가 없으므로 개선을 수락하지 않습니다.</p>'
    page+='<p>열: 현재 M0 / IMU M1 / 폭 사전값 제약만 해제 / smoothing만 해제. 초록·노랑은 관측, 주황은 구조 추정 구간입니다. 추정 선과 관측 픽셀을 같은 정답으로 취급하지 않습니다.</p>'
    page+=''.join('<figure><figcaption>'+html.escape(p)+'</figcaption><img loading="lazy" src="'+html.escape(p)+'"></figure>' for p in gallery)
    page+='<p><a href="summary.json">집계</a> · <a href="frames.jsonl">프레임별 결과</a></p><pre>'+html.escape(json.dumps(summary,indent=2,ensure_ascii=False))+'</pre>'
    (args.output/'index.html').write_text(page)
    print(json.dumps({k:summary[k] for k in ('frames','validity','matched_temporal_pairs','matched_temporal_lateral_change_m')},indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    main(parser.parse_args())
