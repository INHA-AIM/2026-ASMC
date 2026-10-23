#!/usr/bin/env python3
"""GT-defined common events, fixed query support, REAL time shifts; evaluation only.

No GT is passed to inference. Time shift is an error-minimising diagnostic estimate,
not a causal system delay. Missing/ambiguous/boundary events remain in the report.
"""
import argparse, json
from pathlib import Path
import numpy as np
import pandas as pd

DELAY=np.arange(-.5,.80001,.02)

def vector_world(x,y,yaw):
    c,s=np.cos(yaw),np.sin(yaw)
    return np.column_stack([c*x-s*y,s*x+c*y])

# Relative seconds: absolute endpoint/exact-match tolerance 10 ns; no relative tolerance.
TIME_ATOL = 1e-8
MAX_GAP = .30

def timestamp_quality(t):
    t=np.asarray(t,dtype=float)
    ordered=np.sort(t[np.isfinite(t)]) # audit only; never sorted for interpolation
    return {'duplicate_timestamps_n':int(np.sum(np.diff(ordered)<=TIME_ATOL)),
            'duplicate_edges':int(np.sum(np.abs(np.diff(t))<=TIME_ATOL)),
            'nonmonotonic_edges':int(np.sum(np.diff(t)<-TIME_ATOL)),
            'nonfinite_n':int(np.sum(~np.isfinite(t)))}

def interpolate(t,y,q,maxgap=MAX_GAP,groups=None):
    """Exact unique samples survive any preceding gap. Never sort malformed time.
    Only consecutive, increasing, same-generation edges can interpolate.
    Duplicate/overlapping support is ambiguous and returns NaN (counted separately).
    """
    t=np.asarray(t,dtype=float);y=np.asarray(y,dtype=float);q=np.asarray(q,dtype=float)
    out=np.full((len(q),y.shape[1]),np.nan)
    if not len(t):return out
    groups=np.zeros(len(t),dtype=int) if groups is None else np.asarray(groups)
    dt=np.diff(t)
    good=(dt>TIME_ATOL)&(dt<=maxgap+TIME_ATOL)&(groups[1:]==groups[:-1])
    if np.any(dt<=TIME_ATOL) or not np.isfinite(t).all():
        # Do not silently interpolate malformed trajectories, even after sorting.
        for k,v in enumerate(q):
            exact=np.flatnonzero(np.abs(t-v)<=TIME_ATOL)
            if len(exact)==1:out[k]=y[exact[0]]
        return out
    right=np.searchsorted(t,q)
    lo=np.clip(right-1,0,len(t)-1);hi=np.clip(right,0,len(t)-1)
    dl=np.abs(q-t[lo]);dh=np.abs(q-t[hi])
    exact=np.minimum(dl,dh)<=TIME_ATOL
    index=np.where(dl<=dh,lo,hi)
    out[exact]=y[index[exact]]
    ambiguous=(lo!=hi)&(dl<=TIME_ATOL)&(dh<=TIME_ATOL)
    out[ambiguous]=np.nan
    if len(t)>1:
        edge=np.clip(right-1,0,len(t)-2)
        valid=(~exact)&(right>0)&(right<len(t))&good[edge]
        e=edge[valid];w=(q[valid]-t[e])/dt[e]
        out[valid]=(1-w[:,None])*y[e]+w[:,None]*y[e+1]

    return out

def trajectory_groups(rows):
    # Explicit generation preferred; legacy samples expose age/observation count resets.
    cut=rows.pid.ne(rows.pid.shift())
    for c in ['generation','age','obs_count']:
        if c in rows:
            cut |= rows[c].ne(rows[c].shift()) if c=='generation' else rows[c].diff().le(0)
    return cut.cumsum().to_numpy()

def aligned_errors(pred,truth,mask):
    p=pred[mask];g=truth[mask]
    ps=np.linalg.norm(p,axis=1);gs=np.linalg.norm(g,axis=1)
    return {'zero_shift_vector_mae_mps':float(np.mean(np.linalg.norm(p-g,axis=1))),
            'zero_shift_speed_mae_mps':float(np.mean(np.abs(ps-gs))),
            'speed_overshoot_peak_mps':float(max(0,np.max(ps-gs))),
            'speed_overshoot_mean_positive_mps':float(np.mean(np.maximum(0,ps-gs))),
            'overshoot_definition':'positive speed residual at fixed zero-shift queries; not turn-angle overshoot'}

def estimate(pred,truth,t,q,mask,kind):
    costs=[]
    for d in DELAY:
        target=interpolate(t,truth,q-d)
        if kind=='speed': e=np.abs(np.linalg.norm(pred,axis=1)-np.linalg.norm(target,axis=1))
        else:e=np.linalg.norm(pred-target,axis=1)
        costs.append(float(np.mean(e[mask])))
    k=int(np.argmin(costs)); near=DELAY[np.asarray(costs)<=costs[k]+.02]
    return {'seconds':float(DELAY[k]),'cost_mps':costs[k],'boundary':k in (0,len(DELAY)-1),'near_optimal_width_s':float(np.ptp(near)),'ambiguous':bool(np.ptp(near)>.30)}

def evaluate(inputs,arms,scenes):
    events=[]
    for scene in scenes:
        fr=pd.read_csv(inputs/scene/'frames.csv').set_index('idx')
        gt=pd.read_csv(inputs/scene/'gt.csv')
        gt=gt[(gt.kind=='gt')&(gt.class_id==0)&(gt.x>=0)&(np.hypot(gt.x,gt.y)<=35)]
        tp={name:pd.read_pickle(run/'samples'/f'{scene}.pkl')['tp'] for name,run in arms.items()}
        for tid,g in gt.groupby('track_id'):
            g=g.sort_values('idx')
            for _,seg in g.groupby((g.idx.diff()!=1).cumsum()):
                if len(seg)<20:continue
                origin=int(seg.stamp_ns.iloc[0]);t=(seg.stamp_ns.to_numpy(dtype=np.int64)-origin)*1e-9
                if t[-1]<2:continue
                yaw=fr.loc[seg.idx,'ego_yaw'].to_numpy(); truth=vector_world(seg.vx.to_numpy(),seg.vy.to_numpy(),yaw)
                speed=np.linalg.norm(truth,axis=1); moving=speed>1
                if moving.sum()<10:continue
                speed_change=np.ptp(speed)>=2
                turn=np.ptp(np.unwrap(np.arctan2(truth[moving,1],truth[moving,0])))>=np.deg2rad(15)
                if not(speed_change or turn):continue
                q=np.arange(.8,t[-1]-.5,.05)
                if len(q)<12:continue
                entry={'scene':scene,'gt_id':int(tid),'idx_start':int(seg.idx.iloc[0]),'idx_end':int(seg.idx.iloc[-1]),'duration_s':float(t[-1]),'origin_stamp_ns':origin,'query_relative_s':q.tolist(),'speed_change':bool(speed_change),'turn':bool(turn),'query_n':len(q),'arms':{}}
                predictions={}; common=np.ones(len(q),dtype=bool)
                for name,rows in tp.items():
                    all_r=rows[(rows.gt_tid==tid)&(rows.idx>=seg.idx.iloc[0])&(rows.idx<=seg.idx.iloc[-1])].sort_values('idx',kind='stable')
                    r=all_r[all_r.kfc>=2]
                    r=r[np.isfinite(r.pvx)&np.isfinite(r.pvy)]
                    rt=(fr.loc[r.idx,'stamp_ns'].to_numpy(dtype=np.int64)-origin)*1e-9
                    if len(r)==0:pred=np.full((len(q),2),np.nan)
                    else:
                        rt=(fr.loc[r.idx,'stamp_ns'].to_numpy(dtype=np.int64)-origin)*1e-9
                        pyaw=fr.loc[r.idx,'ego_yaw'].to_numpy()
                        pred=interpolate(rt,vector_world(r.pvx.to_numpy(),r.pvy.to_numpy(),pyaw),q,groups=trajectory_groups(r))
                    valid=np.isfinite(pred).all(axis=1);common&=valid;predictions[name]=pred
                    entry['arms'][name]={'timestamp_quality':timestamp_quality(rt), 'generation_source':'pid + age/obs_count reset (legacy); explicit generation if present', 'initializing_rows_n':int((all_r.kfc<2).sum()), 'reacquisition_rows_n':int(all_r.prev_miss.astype(bool).sum()), 'missing_frame_n':int(len(seg)-all_r.idx.nunique()), 'unsupported_query_n':int((~valid).sum()), 'matched_n':len(r),'query_coverage':float(valid.mean()),'id_switches_in_event':int((r.pid.diff().iloc[1:]!=0).sum()) if len(r) else 0}
                # SAME truth support for EVERY delay as well as every arm.
                for d in DELAY:common&=np.isfinite(interpolate(t,truth,q-d)).all(axis=1)
                entry['truth_timestamp_quality']=timestamp_quality(t)
                entry['exclusion_reason']=None if common.sum()>=12 and common.mean()>=.8 else ('insufficient_common_queries' if common.sum()<12 else 'common_coverage_below_0.8')
                entry['common_query_mask']=common.tolist()
                entry['common_query_n']=int(common.sum()); entry['common_query_coverage']=float(common.mean())
                if common.sum()>=12 and common.mean()>=.8:
                    for name,pred in predictions.items():
                        entry['arms'][name].update(aligned_errors(pred,interpolate(t,truth,q),common))
                        entry['arms'][name]['vector_lag']=estimate(pred,truth,t,q,common,'vector')
                        if speed_change:entry['arms'][name]['speed_lag']=estimate(pred,truth,t,q,common,'speed')
                events.append(entry)
    summary={}
    for name in arms:
        valid_events=[e for e in events if e['exclusion_reason'] is None]
        total_q=sum(e['query_n'] for e in events)
        sm={'gt_defined_events':len(events),'valid_events':len(valid_events),
            'event_coverage':len(valid_events)/len(events) if events else 0,
            'query_total_n':total_q,'query_common_n':sum(e['common_query_n'] for e in events),
            'query_coverage':sum(e['common_query_n'] for e in events)/total_q if total_q else 0,
            'excluded_reasons':{k:sum(e['exclusion_reason']==k for e in events) for k in ['insufficient_common_queries','common_coverage_below_0.8']},
            'event_fractions_with_initialization_reacquisition_missing':{
                key:sum(e['arms'][name][key]>0 for e in events)/len(events) if events else 0
                for key in ['initializing_rows_n','reacquisition_rows_n','missing_frame_n']},
            'excluded_event_flags':{
                key:{'n':sum(e['exclusion_reason'] is not None and e['arms'][name][key]>0 for e in events),
                     'fraction_of_all_events':sum(e['exclusion_reason'] is not None and e['arms'][name][key]>0 for e in events)/len(events) if events else 0,
                     'fraction_of_excluded_events':sum(e['exclusion_reason'] is not None and e['arms'][name][key]>0 for e in events)/max(1,len(events)-len(valid_events))}
                for key in ['initializing_rows_n','reacquisition_rows_n','missing_frame_n']},
            'note':'excluded-event flags overlap; causal attribution of every missing query is not inferred'}

        for kind in ['speed_lag','vector_lag']:
            rows=[e['arms'][name][kind] for e in events if kind in e['arms'][name]]
            interior=[r for r in rows if not r['boundary'] and not r['ambiguous']]
            sm[kind]={'common_events':len(rows),'boundary_n':sum(r['boundary'] for r in rows),'ambiguous_n':sum(r['ambiguous'] for r in rows),'all_estimates_mean_s':float(np.mean([r['seconds'] for r in rows])) if rows else None,'interior_clear_n':len(interior),'interior_clear_mean_s':float(np.mean([r['seconds'] for r in interior])) if interior else None}
        summary[name]=sm
    paired={};reference=next(iter(arms))
    for name in arms:
        if name==reference:continue
        comparisons={}
        for kind in ['speed_lag','vector_lag']:
            rows=[e for e in events if kind in e['arms'][reference] and kind in e['arms'][name]]
            clear=[e for e in rows if all(not e['arms'][n][kind]['boundary'] and not e['arms'][n][kind]['ambiguous'] for n in [reference,name])]
            def delta(es):
                ds=[e['arms'][name][kind]['seconds']-e['arms'][reference][kind]['seconds'] for e in es]
                return {'n':len(ds),'mean_delta_s':float(np.mean(ds)) if ds else None,'median_delta_s':float(np.median(ds)) if ds else None,'improved':sum(d<-.01 for d in ds),'equal':sum(abs(d)<=.01 for d in ds),'worse':sum(d>.01 for d in ds)}
            comparisons[kind]={'all_common':delta(rows),'both_interior_clear':delta(clear)}
        paired[name]={'reference':reference,**comparisons}
    return {'schema_version':'response_v2.2_identity_safe','time_atol_s':TIME_ATOL,'max_interpolation_gap_s':MAX_GAP,'method':__doc__,'delay_grid_s':DELAY.tolist(),'summary':summary,'paired':paired,'events':events}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--inputs',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--scenes');p.add_argument('arms',nargs='+');a=p.parse_args()
    arms={n:Path(v) for n,v in (x.split('=',1) for x in a.arms)}
    manifest=json.loads((a.inputs/'INPUT_MANIFEST.json').read_text())
    scenes=a.scenes.split(',') if a.scenes else [s for s,m in manifest['scenes'].items() if m['split']=='primary']
    result=evaluate(a.inputs,arms,scenes);result['scenes']=scenes;result['arms']={n:str(v) for n,v in arms.items()}
    a.out.parent.mkdir(parents=True,exist_ok=True);
    if a.out.exists():raise FileExistsError(a.out)
    a.out.write_text(json.dumps(result,indent=2));print(json.dumps(result['summary'],indent=2))
