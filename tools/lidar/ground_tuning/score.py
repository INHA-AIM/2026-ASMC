#!/usr/bin/env python3
"""Joint GT graph scoring of all frozen variants; never uses detector clusters."""
import argparse
from collections import Counter, defaultdict
from contextlib import ExitStack
import gzip
import itertools
import json
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ground_live_review'))
from metrics import frame_metrics, CLASSES
import metrics as metric_module
from evaluate import digest

original_topology=metric_module.topology
def identical_group_topology(points,masks):
    """Same GT-group graph and arithmetic; compute identical subgroup masks once."""
    unique={};keys={};aliases={}
    for name,mask in masks.items():
        key=np.asarray(mask,dtype=np.uint8).tobytes()
        if key not in keys:keys[key]=name;unique[name]=mask
        aliases[name]=keys[key]
    results=original_topology(points,unique)
    return {name:results[aliases[name]] for name in masks}
metric_module.topology=identical_group_topology


def evaluate(root, results, output):
    output.mkdir(parents=True,exist_ok=True)
    if (output/'summary.json').exists() or (output/'frames.jsonl.gz').exists(): raise FileExistsError(output)
    files=sorted(results.glob('*.jsonl'))
    if len(files)!=14: raise ValueError('expected V1, coverage-only, twelve single-variable profiles')
    classes=defaultdict(lambda:defaultdict(Counter))
    objects=defaultdict(lambda:defaultdict(Counter))
    topology=defaultdict(lambda:defaultdict(lambda:defaultdict(Counter)))
    reasons=defaultdict(lambda:defaultdict(Counter));events=defaultdict(Counter)
    times=defaultdict(lambda:defaultdict(list));regress=defaultdict(Counter)
    identities={};seen=0
    with ExitStack() as stack:
        inputs=stack.enter_context((root/'snapshots.jsonl').open())
        streams=[stack.enter_context(f.open()) for f in files]
        out=stack.enter_context(gzip.open(output/'frames.jsonl.gz','wt'))
        failures=stack.enter_context((output/'object_regressions.jsonl').open('x'))
        for lines in itertools.zip_longest(inputs,*streams):
            if None in lines: raise ValueError('length mismatch')
            source,*rows=map(json.loads,lines)
            observed=source['scan_effective_stamp_ns'];names={};masks={}
            lineage=rows[0]['raw_to_voxel']
            for file,row in zip(files,rows):
                if row['input_id']!=source['input_id'] or row['stamp_ns']!=observed or row['raw_to_voxel']!=lineage:
                    raise ValueError('identity/stamp/lineage mismatch')
                if row['config_id']!=file.stem: raise ValueError('profile mismatch')
                identity=(row['config_id'],row['config_sha256'])
                if file.stem in identities and identities[file.stem]!=identity: raise ValueError('configuration changed mid-run')
                identities[file.stem]=identity
                for method,data in row['methods'].items():
                    name='v1_'+method if file.stem=='v1' else file.stem
                    names[name]=(method,data,row)
                    masks[name]=np.asarray(data['mask'],dtype=np.uint8)
            path=Path(source['cloud_path'])
            if path.is_absolute() or '..' in path.parts: raise ValueError('invalid input path')
            cloud=np.fromfile(root/path,dtype='<f4').reshape(-1,4)
            with np.load(root/'labels'/(source['input_id']+'.npz')) as gt:
                semantic=gt['semantic'];instance=gt['instance']
            # Identical masks share the exact same diagnostic graph computation.
            unique={};by_bytes={};alias={}
            for name,mask in masks.items():
                key=mask.tobytes()
                if key not in by_bytes: by_bytes[key]=name;unique[name]=mask
                alias[name]=by_bytes[key]
            unique_metrics=frame_metrics(cloud,semantic,instance,unique,source['pitch'])
            metrics={n:unique_metrics[alias[n]] for n in masks}
            out.write(json.dumps({'input_id':source['input_id'],'stamp_ns':observed,'methods':metrics},separators=(',',':'),allow_nan=False)+'\n')
            roi=np.asarray(lineage)>=0
            semantics=np.array([CLASSES.get(int(v),'unmapped_'+str(int(v))) for v in semantic])
            for name,data in metrics.items():
                method,raw,row=names[name]
                for category,c in data['classes'].items():
                    dest=classes[name][category]
                    dest.update({k:c[k] for k in ('input_points','retained_points','ground_points','unclassified_points')})
                    if 'regions' in c:
                        dest.update({k:v for k,v in c['regions'].items() if isinstance(v,int)})
                        for radius,stats in c['topology'].items():
                            topology[name][category][radius].update({k:v for k,v in stats.items() if isinstance(v,int)})
                base={ (o['semantic'],o['instance_id']):o for o in metrics['v1_'+method]['objects']}
                for o in data['objects']:
                    dest=objects[name][o['semantic']+('/sparse' if o['sparse'] else '/observed10plus')]
                    dest.update(observations=1,input_points=o['input_points'],retained_points=o['retained_points'],
                                complete_loss=int(o['complete_ground_loss']),unknown=o['unclassified_points'])
                    for band,counts in o['observed_height_bands'].items():
                        dest[band+'_input']+=counts['input_points'];dest[band+'_retained']+=counts['retained_points']
                    if o['sparse']: continue
                    b=base[(o['semantic'],o['instance_id'])]
                    loss=o['retained_points']-b['retained_points']
                    extra_loss=o['complete_ground_loss'] and not b['complete_ground_loss']
                    split_delta={r:o['topology'][r]['added_split_parts']-b['topology'][r]['added_split_parts'] for r in ('0.2','0.4','0.8')}
                    lower_delta=o['observed_height_bands']['lower']['retained_points']-b['observed_height_bands']['lower']['retained_points']
                    regress[name].update(additional_complete_loss=int(extra_loss),point_degraded_observations=int(loss<0),
                                         lower_degraded_observations=int(lower_delta<0),extra_split_observations=int(any(v>0 for v in split_delta.values())))
                    if loss<0 or lower_delta<0 or any(v>0 for v in split_delta.values()) or extra_loss:
                        failures.write(json.dumps({'profile':name,'input_id':source['input_id'],'class':o['semantic'],
                            'instance':o['instance_id'],'input_points':o['input_points'],'retained_before':b['retained_points'],
                            'retained_after':o['retained_points'],'lower_delta':lower_delta,'extra_complete_loss':bool(extra_loss),
                            'added_split_delta':split_delta},separators=(',',':'))+'\n')
                flags=np.asarray(raw['reasons'],dtype=np.uint32)
                if len(flags):
                    for category in set(semantics[roi]):
                        selected=roi&(semantics==category)&(masks[name]==2)
                        for bit in range(13):
                            count=int(np.count_nonzero(flags[selected]&(1<<bit)))
                            if count: reasons[name][category][str(1<<bit)]+=count
                events[name].update(raw['events'])
                for k,v in raw.items():
                    if k.endswith('_ms'):times[name][k].append(v)
                times[name]['prepare_ms'].append(row['prepare_ms'])
                times[name]['accounted_stage_sum_ms'].append(row['prepare_ms']+sum(raw[k] for k in ('copy_ms','segment_ms','guard_ms','restore_ms')))
            seen+=1
            if seen%100==0: print(json.dumps({'scored':seen,'last':source['input_id']}),flush=True)
    gates={}
    for name,obs in objects.items():
        if name.startswith('v1_'):continue
        method='patchwork' if name.startswith('pp_') else 'linefit';base=objects['v1_'+method]
        violations=[]
        if regress[name]['additional_complete_loss']:violations.append('additional_complete_loss')
        for c in ('vehicle','pedestrian','obstacle'):
            key=c+'/observed10plus';v,b=obs[key],base[key]
            for metric in ('retained_points','lower_retained'):
                if v[metric]<b[metric]:violations.append(c+'/'+metric)
        if sum(c['unclassified_points'] for c in classes[name].values()) > sum(c['unclassified_points'] for c in classes['v1_'+method].values()):
            violations.append('unclassified_increase')
        changes={}
        for c in ('asphalt','sidewalk'):
            changes[c]={r:{k:topology[name][c][r][k]-topology['v1_'+method][c][r][k]
                for k in ('added_split_parts','newly_isolated_points')} for r in ('0.2','0.4','0.8')}
        values=[v for c in changes.values() for r in c.values() for v in r.values()]
        structural_pass=all(v<=0 for v in values) and any(v<0 for v in values)
        gates[name]={'object_pass':not violations,'violations':violations,'background_deltas':changes,
                     'structural_pass':structural_pass,'eligible_on_this_recording':not violations and structural_pass}
    result={'frames':seen,'classes':classes,'objects':objects,'topology':topology,'regressions':regress,'gates':gates,
            'retained_reason_bits':reasons,'fit_events':events,'config_identities':identities,
            'timing_ms':{n:{k:{'p'+str(q):float(np.percentile(v,q)) for q in (50,95,99)} for k,v in t.items()} for n,t in times.items()},
            'sha256':{'snapshots':digest(root/'snapshots.jsonl'),**{f.stem:digest(f) for f in files}},
            'limitations':['Previously analysed recordings, not held out.','No verified temporal surface correspondence.',
                           'Foreground ground labels and semantic structures are not object detector false-positive truth.',
                           'Timing includes diagnostics and concurrent offline load, not real-time system latency.']}
    (output/'summary.json').write_text(json.dumps(result,indent=2,sort_keys=True,allow_nan=False)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('input',type=Path);p.add_argument('results',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();evaluate(a.input,a.results,a.output)
