#!/usr/bin/env python3
"""Retain every GUI trial; separate codec savings from supply/readiness gates."""
import argparse
import ast
from collections import Counter,defaultdict
import json
from pathlib import Path
from types import SimpleNamespace

from lifecycle_measurements import rate_report,percentile
from summarize_camera_structure import rate_followup,rows,parity
from compare_camera_runtime import resource_summary


def distribution(values):
    return dict(count=len(values),**{f'p{q}_ms':percentile(values,q) for q in (50,95,99)})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--results',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--matrix',default='final-matrix',help='Completed fixed-code comparison directory')
    p.add_argument('--recovery',default='resource-recovery',help='Coverage-only repeat; never replace a failed performance result')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    rate_followup(SimpleNamespace(results=a.results,output=a.output))
    summary=json.loads((a.output/'summary.json').read_text())
    originals={t['trial']:t for t in summary['trials'] if t['matrix']==a.matrix}
    recovered={}
    for t in summary['trials']:
        if t['matrix']!=a.recovery:continue
        jobs=json.loads((a.results/a.recovery/'jobs.json').read_text())
        job=next(j for j in jobs if j['name']==t['trial'])
        t['rerun_of']=job['rerun_of']
        if t['rerun_of'] in recovered:raise ValueError('Ambiguous repeated coverage recovery')
        recovered[t['rerun_of']]=t['trial']
    pooled=defaultdict(lambda:defaultdict(list));family_pooled=defaultdict(list)
    for trial in summary['trials']:
        root=a.results/trial['matrix']/trial['trial']
        if not trial.get('complete'):trial['minimum_acceptance_failures'].append('incomplete measurement')
        start,end=trial.get('start'),trial.get('end')
        if start is None:continue
        trial['resources']=resource_summary(json.loads((root/'resources.json').read_text()),start,end)
        resource_ok=trial['resources']['resource_coverage_pass']
        if not resource_ok:trial['minimum_acceptance_failures'].append('resource coverage invalid')
        trial['effective_final_trial']=trial['matrix']==a.matrix and resource_ok
        if trial['matrix']==a.recovery:
            original=originals[trial['rerun_of']]
            original_root=a.results/a.matrix/original['trial']
            original_resources=resource_summary(json.loads((original_root/'resources.json').read_text()),original['start'],original['end'])
            if original_resources['resource_coverage_pass']:raise ValueError('Cannot replace valid measurements based on performance')
            trial['effective_final_trial']=resource_ok
        if trial['matrix']==a.matrix and trial['trial'] in recovered:trial['resource_recovery_trial']=recovered[trial['trial']]
        family=json.loads((root/'lane-families.json').read_text())
        valid=rate_report([(r['at'],r['stamp'],True) for r in family if r['valid']],start,end)
        trial['lane_family']['valid_supply']=valid
        if not valid['five_second_hz'] or min(valid['five_second_hz'])<10 or valid['max_gap_sec']>.25:
            trial['minimum_acceptance_failures'].append('valid lane family supply/gap')
        reasons=Counter();source_ages=[];view=[];model_snapshots=defaultdict(list)
        for event in json.loads((root/'diagnostics.json').read_text()):
            if not start<=event['at']<end:continue
            for status in event['statuses']:
                v=status['values']
                if status['name']=='camera_semantic_perception/readiness':
                    if v['pipeline_ready']=='False':reasons[v['pipeline_ready_reason']]+=1
                    try:source_ages.append(float(v['family_source_age_ms']))
                    except (ValueError,KeyError):pass
                if status['name']=='camera_objects/view':view.append((event['at'],v))
                model_snapshots[status['name']].append((event['at'],v))
        trial['readiness_failure_reasons']=dict(reasons)
        trial['family_source_age_ms']=distribution(source_ages)
        trial['inference_counter_deltas']={}
        for name,snapshots in model_snapshots.items():
            if len(snapshots)<2:continue
            first,last=snapshots[0][1],snapshots[-1][1]
            values={}
            for key in ('received_frames','output_frames','replaced_frames','output_batches',
                        'decode_errors','inference_errors','header_errors','mask_errors','filter_errors',
                        'errors','fusion_errors','pair_errors','pending_fit_replacements'):
                if key in first and key in last:
                    values[key]=int(last[key])-int(first[key])
            for key in ('received_by_view','output_by_view','replaced_by_view','queue_drops_by_view'):
                if key in first and key in last:
                    a_values,b_values=ast.literal_eval(first[key]),ast.literal_eval(last[key])
                    values[key]={k:b_values[k]-a_values.get(k,0) for k in b_values}
            if values:trial['inference_counter_deltas'][name]=values
            if any(v for k,v in values.items() if k.endswith('errors') and isinstance(v,int)):
                trial['minimum_acceptance_failures'].append('diagnostic errors: '+name)
        trial['object_backends']=sorted({v['backend'] for _,v in model_snapshots.get('camera_objects/inference',[])})
        codecs={}
        for path in root.glob('visual-*.jsonl'):
            events=[r for r in rows(path) if start<=r['at']<end]
            codecs[path.stem]={k:dict(calls=sum(r['event']==k for r in events),
                elapsed_ms=sum(r['duration_ms'] for r in events if r['event']==k)) for k in ('decode','encode')}
        trial['object_visual_node_count']=len(codecs)
        trial['codecs']=codecs
        if len(view)>1:
            first,last=view[0],view[-1]
            keys=('decode','render','mosaic_encode','mosaic_publish','individual_encode','individual_publish',
                  'worker_error','decode_or_shape_error','render_error','encode_error','invalidated_render')
            delta={k:int(json.loads(last[1].get(k,'0')))-int(json.loads(first[1].get(k,'0'))) for k in keys}
            trial['view_counter_delta']=dict(delta,window_sec=last[0]-first[0])
            trial['codec_contract_pass']=(delta['mosaic_encode']==delta['mosaic_publish'] and
                delta['individual_encode']==0 and delta['decode']<=3*delta['mosaic_publish']+3 and
                not any(delta[k] for k in ('worker_error','decode_or_shape_error','render_error','encode_error')))
            trial['missing_fresh_view_snapshots']=sum(set(json.loads(v.get('fresh_views','[]')))!=set(('front','left','right')) for _,v in view)
            trial['max_image_cache_per_view']={k:max(json.loads(v.get('image_cache_sizes','{}')).get(k,0) for _,v in view) for k in ('front','left','right')}
            trial['pending_capacity_values']=sorted({int(v['pending_capacity']) for _,v in view})
            if max(trial['max_image_cache_per_view'].values())>8 or trial['pending_capacity_values']!=[1]:
                trial['minimum_acceptance_failures'].append('debug cache/pending bound')
            if trial['matrix'] in (a.matrix,a.recovery,'soak') and trial['missing_fresh_view_snapshots']:
                trial['minimum_acceptance_failures'].append('debug view missing/stale')
            if not trial['codec_contract_pass']:trial['minimum_acceptance_failures'].append('debug codec contract/errors')
        if trial['gui'] and 'objects' in trial['active']:
            debug=trial.get('debug_outputs',{}).get('objects',{})
            if trial['matrix'] in (a.matrix,a.recovery,'soak') and debug.get('messages',0)==0:
                trial['minimum_acceptance_failures'].append('missing actual object debug output')
        if trial['effective_final_trial'] and trial.get('complete') and '-control-' not in trial['trial']:
            group=('baseline' if '-baseline-' in trial['trial'] else 'candidate')+'-'+('foggy' if '-foggy-' in trial['trial'] else 'sunny')
            inputs={}
            for path in root.glob('*.jsonl'):
                for r in rows(path):
                    if r['event']=='input' and path.stem=='camera_lane':inputs[r['stamp']]=min(r['at'],inputs.get(r['stamp'],r['at']))
                    if r['event']=='output' and r.get('received') is not None and not r.get('stale') and start<=r['at']<end:
                        if r['topic']=='/perception/camera/objects' and not r['valid']:continue
                        pooled[group][r['topic']+':'+r['frame']].append(r['wall_ms'])
            for r in family:
                if start<=r['at']<end and r['stamp'] in inputs:family_pooled[group].append((r['at']-inputs[r['stamp']])*1000)
    summary.update(title='단일 객체 시각화·readiness 진단·전체 GUI 대조 — 2026-10-15',
        introduction='E8·confidence0.40·NMS0.50, YOLO/TwinLite15Hz·ResNet10Hz 유지. 같은 입력의 객체 overlay를 직접 합성하고 JPEG를 한 번 인코딩한다. 차선 350ms freshness를 유지하고 정상 ACTIVE 실패를 면제하지 않는다.',
        pooled_frame_latency={g:{k:distribution(v) for k,v in streams.items()} for g,streams in pooled.items()},
        pooled_family_latency={g:distribution(v) for g,v in family_pooled.items()},
        final_candidate_failures={r['trial']:r['minimum_acceptance_failures'] for r in summary['trials']
            if (r.get('effective_final_trial') or r['matrix']=='soak') and '-baseline-' not in r['trial'] and r['minimum_acceptance_failures']},
        invalid_resource_trials={r['matrix']+'/'+r['trial']:r.get('resource_recovery_trial') for r in summary['trials']
            if r['matrix'] in (a.matrix,a.recovery,'soak') and not r.get('resources',{}).get('resource_coverage_pass',False)},
        historical_failures={r['matrix']+'/'+r['trial']:r['minimum_acceptance_failures'] for r in summary['trials']
            if r['matrix'] not in (a.matrix,'soak') and r['minimum_acceptance_failures']})
    parity_root=a.results/'saved-parity'
    if all((parity_root/('complete-'+v)/'report.json').exists() for v in ('baseline','candidate')):
        summary['output_parity']=parity(parity_root,('baseline','candidate'))
    effective=[r for r in summary['trials'] if r.get('effective_final_trial')]
    candidate=[r for r in effective if '-candidate-' in r['trial']]
    baseline=[r for r in effective if '-baseline-' in r['trial']]
    controls=[r for r in effective if '-control-' in r['trial']]
    soaks=[r for r in summary['trials'] if r['matrix']=='soak']
    summary['runtime_completion_checks']=dict(
        fixed_candidate_source=len({r['source_sha256'] for r in candidate+soaks})==1,
        baseline_three_per_weather=all(sum('-'+w+'-' in r['trial'] for r in baseline)==3 for w in ('sunny','foggy')),
        candidate_three_per_weather=all(sum('-'+w+'-' in r['trial'] for r in candidate)==3 for w in ('sunny','foggy')),
        four_controls=len(controls)==4,
        completed_soak=len(soaks)==1 and soaks[0].get('complete') and soaks[0]['end']-soaks[0]['start']>=1800,
        candidate_and_control_gates=not any(r['minimum_acceptance_failures'] for r in candidate+controls+soaks),
        output_parity=bool(summary.get('output_parity')) and all(
            v['common']>0 and not v['mismatched'] and v['max_confidence_delta']<=.02
            for checks in summary.get('output_parity',{}).values() for v in checks.values()))
    summary['runtime_acceptance_pass']=all(summary['runtime_completion_checks'].values())
    summary['acceptance_complete']=False  # Final review also requires completed soak, parity and CI.
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(dict(trials=len(summary['trials']),candidate_failures=summary['final_candidate_failures'])))


if __name__=='__main__':main()
