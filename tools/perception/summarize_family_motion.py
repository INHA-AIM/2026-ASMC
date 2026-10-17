#!/usr/bin/env python3
"""Compare variants on identical supported x samples, excluding coverage bias.

Temporal differences are SE(2)-aligned curve changes, not independent GT error.
The common-query check supplements the per-variant native-sample statistics.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import numpy as np
if __package__:
    from .review_family_motion import read_rows, stats, interpolate, move_xy, sha
else:
    from review_family_motion import read_rows, stats, interpolate, move_xy, sha
from asmc_camera.bev.shared_lane_family import contiguous_sample_state_segments

VARIANTS=('M0','M1_absolute','M1_rest','M2_align_only','M2_align','M2_history')


def sample_boundary(boundary, query, source_pose=None, target_pose=None):
    result=np.full(len(query),np.nan)
    for state,xy in contiguous_sample_state_segments(boundary['xy'],boundary['support']):
        if state=='invalid':continue
        xy=move_xy(xy,source_pose,target_pose) if source_pose is not None else np.asarray(xy)
        xy=xy[np.argsort(xy[:,0])]
        # Do not bridge folded/duplicate forward coordinates silently.
        if np.any(np.diff(xy[:,0])<=1e-9):continue
        valid=(query>=xy[0,0])&(query<=xy[-1,0])
        result[valid]=np.interp(query[valid],xy[:,0],xy[:,1])
    return result


def matched_pairs(rows, signals):
    ego=defaultdict(list)
    for s in signals:
        if s['topic']=='/Ego_topic':ego[s['run_id']].append(s)
    for samples in ego.values():samples.sort(key=lambda x:x['source_ns'])
    previous={};pairs=[];query=np.arange(.5,14.5001,.25)
    for row in rows:
        if row['split']!='continuous':continue
        key=row['run_id'],row['phase'];old=previous.get(key);previous[key]=row
        if old is None or not 0<row['target_ns']-old['target_ns']<=200_000_000:continue
        if any(v not in r['variants'] for r in (old,row) for v in VARIANTS):continue
        poses=[interpolate(ego[row['run_id']],r['target_ns'],'pose') for r in (old,row)]
        if any(p is None for p in poses):continue
        diffs=defaultdict(list)
        for role in ('left_outer_boundary','yellow_centre_boundary','right_outer_boundary'):
            changes={}
            for v in VARIANTS:
                current=next((b for b in row['variants'][v]['boundaries'] if b['role']==role),None)
                prior=next((b for b in old['variants'][v]['boundaries'] if b['role']==role),None)
                if current is None or prior is None:break
                changes[v]=np.abs(sample_boundary(current,query)-sample_boundary(prior,query,*poses))
            if len(changes)!=len(VARIANTS):continue
            keep=np.logical_and.reduce([np.isfinite(d) for d in changes.values()])
            for v,d in changes.items():diffs[v].extend(d[keep].tolist())
        if not diffs:continue
        pairs.append(dict(run_id=row['run_id'],phase=row['phase'],source_ns=row['source_ns'],
                          dt_ms=(row['target_ns']-old['target_ns'])/1e6,
                          variants={v:stats(d) for v,d in diffs.items()}))
    return pairs


def summarize(rows,pairs):
    result={}
    for phase in ('all','entry','interior','exit'):
        chosen=pairs if phase=='all' else [r for r in pairs if r['phase']==phase]
        result[phase]={v:stats([r['variants'][v]['p95'] for r in chosen if r['variants'][v]['count']])
                       for v in VARIANTS}
    coverage={}
    continuous=[r for r in rows if r['split']=='continuous']
    for v in VARIANTS:
        values=[]
        for r in continuous:
            values.append(sum(r['variants'][v]['usability'][z]['available_length_m']-
                              r['variants']['M0']['usability'][z]['available_length_m']
                              for z in ('0-5','5-10','10-15')))
        coverage[v]=dict(delta_m=stats(values),loss_over_1cm_frames=sum(d<-.01 for d in values))
    return dict(schema='lane_motion_matched_queries_v1',matched_pair_p95_m=result,consumer_coverage_delta=coverage,
                query_x_m=[.5,14.5],spacing_m=.25,
                definition='Each pair uses identical current-x samples/roles valid in BOTH frames of ALL variants. Compare per-pair lateral-change p95; summaries are distributions of those pair p95 values.',
                independent_gt=False,full_se3_validated=False)


def render_rest_review(args, rows):
    """RGB checks for the rest-relative candidate, no refitting or GT creation."""
    import cv2,yaml
    from asmc_camera.bev.calibration import calibrations_from_dict
    from asmc_camera.bev.grid import BevGridSpec
    if __package__:
        from .review_family_motion import attitude_plane,overlay,render_family
    else:
        from review_family_motion import attitude_plane,overlay,render_family
    meta=json.loads((args.review/'report.json').read_text())
    cameras=calibrations_from_dict(yaml.safe_load((args.config/'cameras_v3.yaml').read_text()))
    grid=BevGridSpec(-5,20,-8,8,.1)
    signals=defaultdict(list)
    for row in read_rows(args.inputs/'signals.jsonl'):
        if row['topic']=='/imu':signals[row['run_id']].append(row)
    for samples in signals.values():samples.sort(key=lambda r:r['source_ns'])
    index={(r['run_id'],r['front_ns'],r['view']):r for r in read_rows(args.inputs/'rgb-index.jsonl')}
    groups=defaultdict(list)
    for row in rows:
        if row['split']=='representative':groups[row['run_id'],row['phase']].append(row)
    contact=[];gallery=[]
    for (run,phase),values in sorted(groups.items()):
        row=values[len(values)//2];panels=[]
        if 'M1_rest' not in row['variants']:continue
        for view in ('left','front','right'):
            src=index[run,row['source_ns'],view]
            blob=(args.inputs/src['path']).read_bytes()
            import hashlib
            if hashlib.sha256(blob).hexdigest()!=src['sha256']:raise ValueError('RGB SHA mismatch')
            image=cv2.imdecode(np.frombuffer(blob,np.uint8),cv2.IMREAD_COLOR)
            q=interpolate(signals[run],src['source_ns'],'imu')
            corrected=attitude_plane(cameras[view],q['q_xyzw'],meta['signals'][run]['rest_reference_q'])
            for variant,cam in (('M0',cameras[view]),('M1_rest',corrected)):
                panel=cv2.resize(overlay(image,row['variants'][variant],cam),(480,270))
                cv2.putText(panel,view+' '+variant,(5,22),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,100,255),2)
                panels.append(panel)
                if view=='front' and variant=='M1_rest':
                    cv2.putText(panel,run.split('_v1_')[-1]+' '+phase,(5,250),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,180,0),1)
                    contact.append(panel.copy())
        bev=[]
        for variant in ('M0','M1_rest'):
            canvas=render_family(row['variants'][variant],grid)
            origin=tuple(np.rint(grid.ego_pixel).astype(int))
            cv2.arrowedLine(canvas,origin,(origin[0],origin[1]-30),(0,255,0),2)
            cv2.putText(canvas,'LEFT +y',(3,95),cv2.FONT_HERSHEY_SIMPLEX,.3,(0,180,255),1)
            cv2.putText(canvas,'RIGHT -y',(100,95),cv2.FONT_HERSHEY_SIMPLEX,.3,(0,180,255),1)
            bev.append(cv2.resize(canvas,(480,750),interpolation=cv2.INTER_NEAREST))
        canvas=np.vstack([np.hstack(panels[i:i+2]) for i in range(0,6,2)]+[np.hstack(bev)])
        path='rest-'+run+'-'+phase+'.jpg';cv2.imwrite(str(args.review/path),canvas)
        gallery.append(dict(path=path,run_id=run,phase=phase))
    cv2.imwrite(str(args.review/'contact-M1-rest.jpg'),np.vstack([np.hstack(contact[i:i+3]) for i in range(0,len(contact),3)]))
    (args.review/'rest-gallery.json').write_text(json.dumps(gallery,indent=2)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--review',type=Path,required=True);p.add_argument('--inputs',type=Path,required=True)
    p.add_argument('--config',type=Path)
    args=p.parse_args();path=args.review/'frames.jsonl'
    rows=read_rows(path);pairs=matched_pairs(rows,read_rows(args.inputs/'signals.jsonl'))
    report=summarize(rows,pairs);report['frames_sha256']=sha(path)
    (args.review/'matched-pairs.jsonl').write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in pairs))
    (args.review/'matched-summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    if args.config:render_rest_review(args,rows)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
