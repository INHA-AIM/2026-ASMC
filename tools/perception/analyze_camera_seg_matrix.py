#!/usr/bin/env python3
"""Preserve every trial, select a training candidate only after front gates."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
from analyze_fusion_identity import compare_stable_observations, compare
from seg_measurements import make_protocol


def key(job):
    return tuple(job[k] for k in ('phase','bag_id','model','views','profile','repeat','debug'))


def analyze(root, accuracy):
    protocol=json.loads((root/'protocol.json').read_text())
    entries=json.loads((root/'index.json').read_text())
    expected=[key(j) for j in protocol];observed=[key(r['job']) for r in entries]
    required={key(j) for j in make_protocol({'repeats':3},('camera','debug','fusion'))}
    groups=defaultdict(list);trials=[]
    for r in entries:
        path=root/r['label']/'report.json'
        report=json.loads(path.read_text()) if path.exists() else None
        row=dict(r,report=report);trials.append(row)
        j=r['job'];groups[j['phase'],j['bag_id'],j['model'],j['views'],j['profile'],j['debug']].append(row)
    complete=(set(expected)==required and len(set(expected))==len(expected) and len(set(observed))==len(observed)
              and set(expected)==set(observed) and all(r['report'] for r in trials))
    fronts={};candidates=[];scores={}
    for model in ('yolo26','yoloe'):
        rows=groups['camera','camera',model,'front','C1',False]
        fronts[model]=bool(len(rows)==3 and all(r['report'] and r['report'].get('passed') for r in rows))
        score_path=accuracy/model/'summary.json'
        score=json.loads(score_path.read_text()) if score_path.exists() else None
        scores[model]=score
        if fronts[model] and score and score['selected_threshold']:
            selected=score['selected_threshold']
            latency=max(r['report']['measurements']['yolo']['input_to_output_wall_ms']['p95'] for r in rows)
            if selected['metrics']['mask_iou'] is not None and latency is not None:
                candidates.append(dict(model=model,confidence=selected['confidence'],macro_f1=selected['metrics']['macro_f1'],
                                       mask_iou=selected['metrics']['mask_iou'],p95_latency_ms=latency))
    chosen=max(candidates,key=lambda c:(c['macro_f1'],c['mask_iou'],-c['p95_latency_ms'])) if candidates else None
    fusion={}
    for bag in ('primary','shadow'):
        old=groups['fusion',bag,'baseline','front','C1',False]
        for model in ('yolo26','yoloe'):
            new=groups['fusion',bag,model,'front','C1',False]
            name=bag+'-'+model
            if len(old)!=2 or len(new)!=2 or any(not r['report'] for r in old+new):
                fusion[name]=dict(complete=False);continue
            old_tracks=[json.loads((root/r['label']/'observations.json').read_text())['tracks'] for r in old]
            new_tracks=[json.loads((root/r['label']/'observations.json').read_text())['tracks'] for r in new]
            old_counts=[(r['report'].get('fusion',{}).get('totals') or {}).get('matched') for r in old]
            new_counts=[(r['report'].get('fusion',{}).get('totals') or {}).get('matched') for r in new]
            try:
                stable=compare_stable_observations(old_tracks,new_tracks)
                pairwise=[compare(old_tracks[0],tracks) for tracks in old_tracks[1:]+new_tracks]
            except ValueError as exc:stable=dict(error=str(exc));pairwise=[]
            fusion[name]=dict(complete=True,baseline_matched=old_counts,candidate_matched=new_counts,
                matched_not_degraded=all(x is not None for x in old_counts+new_counts) and min(new_counts)>=min(old_counts),
                all_runs_at_least_best_baseline=all(x is not None for x in old_counts+new_counts) and min(new_counts)>=max(old_counts),
                stable_observations=stable,pairwise=pairwise,
                classification_gate_pass=bool(stable.get('old_stable')) and not stable.get('regressions') and not stable.get('error'),
                prior_exceptions_applied=False,mask_used_by_operational_fusion=False)
    manifest=json.loads((root/'manifest.json').read_text()) if (root/'manifest.json').exists() else {}
    if manifest.get('smoke_only'):complete=False;chosen=None
    if not complete:chosen=None
    return dict(schema_version=1,protocol_complete=complete,expected_trials=len(expected),completed_trials=len(trials),
                passed_trials=sum(bool(r['report'] and r['report'].get('passed')) for r in trials),
                failed_trials=[r['label'] for r in trials if not r['report'] or not r['report'].get('passed')],
                front_eligible=fronts,training_candidate=chosen,
                selection_status='selected_for_training_only' if chosen else 'withheld_incomplete_or_front_gate_failed',
                accuracy=scores,fusion_comparisons=fusion,trials=trials,
                training_performed=False,operational_defaults_changed=False,mask_fusion_adoption=False)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('matrix',type=Path);p.add_argument('--accuracy',type=Path,required=True)
    p.add_argument('--output',type=Path)
    args=p.parse_args();result=analyze(args.matrix,args.accuracy)
    (args.output or args.matrix/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('protocol_complete','completed_trials','passed_trials','training_candidate')}))
    return 0 if result['protocol_complete'] else 1


if __name__=='__main__':raise SystemExit(main())
