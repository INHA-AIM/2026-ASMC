#!/usr/bin/env python3
"""Keep every first-run outcome; qualify clean repeats separately from metrics."""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--results',type=Path,required=True)
    a=p.parse_args();root=a.results
    deviations=json.loads((root/'condition-deviation.json').read_text()) if (root/'condition-deviation.json').exists() else {}
    rows=[];groups=defaultdict(list)
    for folder in ['batch-trials-v1','followup-trials-v1','condition-recheck-v1','diagnostic-trials-v1']:
        index=root/folder/'index.json'
        if not index.exists():continue
        for item in json.loads(index.read_text()):
            path=index.parent/item['label']/'report.json';r=json.loads(path.read_text())
            condition_ok=item['label']!=deviations.get('trial')
            diagnostic=item['job'].get('diagnostic',False)
            row=dict(folder=folder,label=item['label'],job=item['job'],normal_condition_eligible=condition_ok and not diagnostic,
                     feasibility=item['feasibility'],measurements=r.get('measurements',{}),resources=r.get('resources',{}),
                     fusion=r.get('fusion'),failures=r.get('failures'),last_diagnostics=r.get('last_diagnostics'),
                     last_statuses=r.get('last_statuses'),report_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            rows.append(row)
            if row['normal_condition_eligible']:
                j=item['job'];key=(j['phase'],j['bag_id'],j['prompt'],j['batch_mode'],j['profile'],j.get('debug',False),folder=='batch-trials-v1')
                groups[key].append(row)
    grouped=[]
    for key,items in groups.items():
        names=sorted({k for row in items for k in row['measurements']})
        metrics={}
        for name in names:
            values=[row['measurements'][name] for row in items if name in row['measurements']]
            quant=lambda metric,q:max((v.get(metric,{}).get(q) for v in values if v.get(metric,{}).get(q) is not None),default=None)
            metrics[name]=dict(hz_min=min(v.get('hz',0) for v in values),hz_max=max(v.get('hz',0) for v in values),
                complete_windows_min_hz=min((x for v in values for x in v.get('five_second_hz',[])),default=None),
                first_second_min=min(v.get('first_second',0) for v in values),
                worst_source_p95_ms=quant('source_age_ms','p95'),worst_source_p99_ms=quant('source_age_ms','p99'),
                worst_wall_p95_ms=quant('input_to_output_wall_ms','p95'),
                max_gap_sec=max(v.get('max_gap_sec',0) for v in values),
                duplicate_stamps=sum(v.get('duplicate_stamps',0) for v in values),reversed_stamps=sum(v.get('reversed_stamps',0) for v in values))
        grouped.append(dict(phase=key[0],bag=key[1],prompt=key[2],batch_mode=key[3],profile=key[4],debug=key[5],fixed07_batch_selection=key[6],
                            repeats=len(items),passed_repeats=sum(r['feasibility']['passed'] for r in items),
                            measurements=metrics,peak_vram_mib=max((r['resources'].get('gpu_memory_mib',{}).get('maximum',0) for r in items),default=0),
                            all_phases_peak_vram_mib=max((r['resources'].get('all_phases',{}).get('gpu_memory_mib',{}).get('maximum',0) for r in items),default=0),
                            fusion=[dict(repeat=r['job']['repeat'],**r['fusion']) for r in items if r['fusion']]))
    quality={}
    for profile in ['rich42','base27']:
        for runtime,path in [('pytorch','quality-'+profile+'-pytorch'+('-v2' if profile=='rich42' else '')),('tensorrt','quality-'+profile+'-tensorrt')]:
            r=json.loads((root/path/'summary.json').read_text());quality[profile+'-'+runtime]={k:r[k] for k in ['selection_frames','fixed_07','selected_threshold','input_latency_ms','labels','matched_to_missed_transitions','model','heldout_evaluated','training_performed']}
    summary=dict(schema_version=1,code_scope='opt-in YOLOE-l prompt/batch study; no training or operational promotion',
                 batch_selection=json.loads((root/'batch-selection.json').read_text()),condition_deviation=deviations,
                 quality=quality,groups=grouped,rows=rows,total_trials=len(rows),
                 performance_failed_trials=[r['label'] for r in rows if not r['feasibility']['passed']],
                 trace_diagnostics_excluded_from_normal_acceptance=True,
                 eligible_normal_trials=sum(r['normal_condition_eligible'] for r in rows),
                 strict_twenty_hz_met_in_every_normal_trial=all(all(v['twenty_hz_met'] for v in r['feasibility']['views'].values()) for r in rows if r['normal_condition_eligible']),
                 fusion_count_and_classification_parity_accepted=False,
                 fusion_offset_OFF_statistic='maximum of interval/tail p95, not pooled p95; pooled diagnostic statistic separately provided',
                 GT_limits=['animal GT absent','Obstacle/Vehicle subtype unverified where marked','selection only; heldout unused'],
                 fusion_scope='front bbox only; no 3-view or instance-mask fusion',
                 infrastructure_policy='observations only; consumer decides avoidance')
    (root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(dict(trials=len(rows),failed=summary['performance_failed_trials'],groups=[{k:v for k,v in g.items() if k not in ['measurements','fusion']} for g in grouped])),flush=True)


if __name__=='__main__':main()
