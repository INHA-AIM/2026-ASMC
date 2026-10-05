#!/usr/bin/env python3
"""All-frame retention gates; actor topology all frames, background topology sampled explicitly."""
import argparse
from collections import Counter,defaultdict
import gzip
import itertools
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ground_live_review'))
from metrics import CLASSES,ACTORS,actor_shape,topology
from evaluate import digest

def leveled(cloud,roll,pitch):
    c,s=np.cos(np.float32(pitch)),np.sin(np.float32(pitch))
    cr,sr=np.cos(np.float32(roll)),np.sin(np.float32(roll))
    y=cr*cloud[:,1]-sr*cloud[:,2];z=sr*cloud[:,1]+cr*cloud[:,2]
    return np.column_stack([c*cloud[:,0]+s*z,y,-s*cloud[:,0]+c*z])

def main():
    p=argparse.ArgumentParser()
    for arg in ['input','baseline','candidate','output']:p.add_argument(arg,type=Path)
    p.add_argument('--background-stride',type=int,default=20)
    p.add_argument('--jaeho-profile',choices=['jaeho_height','jaeho_support'],default='jaeho_height')
    a=p.parse_args()
    if a.background_stride<1:raise ValueError('stride')
    a.output.mkdir(parents=True,exist_ok=False)
    classes=defaultdict(lambda:defaultdict(Counter));objects=defaultdict(lambda:defaultdict(Counter))
    graphs=defaultdict(lambda:defaultdict(lambda:defaultdict(Counter)))
    additional=defaultdict(Counter);degraded=defaultdict(Counter);events=defaultdict(Counter)
    timing=defaultdict(lambda:defaultdict(list));planes=defaultdict(Counter);previous={}
    paths=[a.input/'snapshots.jsonl',a.baseline,a.candidate/(a.jaeho_profile+'.jsonl'),a.candidate/'patchwork_support.jsonl']
    streams=[f.open() for f in paths];count=0
    with gzip.open(a.output/'frames.jsonl.gz','wt') as out:
        for values in itertools.zip_longest(*streams):
            if None in values:raise ValueError('stream length mismatch')
            source,base,j,pw=map(json.loads,values)
            for row in [base,j,pw]:
                if row['input_id']!=source['input_id'] or row['stamp_ns']!=source['scan_effective_stamp_ns']:
                    raise ValueError('input/stamp mismatch')
                if row['raw_to_voxel']!=base['raw_to_voxel']:raise ValueError('lineage mismatch')
            raw=np.fromfile(a.input/source['cloud_path'],dtype='<f4').reshape(-1,4)
            xyz=leveled(raw,source['roll'],source['pitch'])
            with np.load(a.input/'labels'/(source['input_id']+'.npz')) as lab:
                sem,instance=lab['semantic'],lab['instance']
            rows={'jaeho_baseline':base['methods']['jaeho'],'patchwork_baseline':base['methods']['patchwork'],
                'jaeho_height':j['methods']['jaeho'],'patchwork_support':pw['methods']['patchwork']}
            masks={n:np.asarray(v['mask'],np.uint8) for n,v in rows.items()}
            roi=np.asarray(base['raw_to_voxel'])>=0
            if any(len(x)!=len(raw) or not np.array_equal(x!=0,roi) for x in masks.values()):
                raise ValueError('point/ROI mismatch')
            names=np.asarray([CLASSES.get(int(x),'unmapped_'+str(int(x))) for x in sem])
            observations=defaultdict(list)
            for label in sorted(set(names[roi])):
                select=roi&(names==label)
                for n,mask in masks.items():
                    classes[n][label].update(input=int(select.sum()),retained=int(np.sum(mask[select]==2)),
                        uncertain=int(np.sum(mask[select]==3)))
                if label in ACTORS:
                    for identity in np.unique(instance[select]):
                        if not identity:continue
                        actor=select&(instance==identity);local={n:x[actor] for n,x in masks.items()}
                        shapes={n:actor_shape(xyz[actor],x) for n,x in local.items()}
                        graph=topology(xyz[actor],local)
                        for n,shape in shapes.items():
                            key=label+('/sparse' if shape['sparse'] else '/observed10plus')
                            lower=shape['observed_height_bands']['lower']
                            objects[n][key].update(observations=1,input=shape['input_points'],
                                retained=shape['retained_points'],lower_input=lower['input_points'],
                                lower_retained=lower['retained_points'],lost=int(shape['complete_ground_loss']),
                                no_retained=int(shape['no_known_retained_points']))
                            for radius,stats in graph[n].items():graphs[n][key][radius].update({k:v for k,v in stats.items() if isinstance(v,int)})
                            if not shape['sparse']:observations[n].append({'class':label,'instance':int(identity),
                                'retained':shape['retained_points'],'lower':lower['retained_points'],'input':shape['input_points']})
                        if len(xyz[actor])>=10:
                            for n,ref in [('jaeho_height','jaeho_baseline'),('patchwork_support','patchwork_baseline')]:
                                before,after=shapes[ref],shapes[n]
                                additional[n][label]+=int(before['retained_points']>0 and after['retained_points']==0)
                                degraded[n][label]+=int(after['retained_points']<before['retained_points'])
                elif count%a.background_stride==0:
                    graph=topology(xyz[select],{n:x[select] for n,x in masks.items()})
                    for n,rs in graph.items():
                        for radius,stats in rs.items():graphs[n][label][radius].update({k:v for k,v in stats.items() if isinstance(v,int)})
            for n,row in rows.items():
                events[n].update(row['events'])
                for k,v in row.items():
                    if k.endswith('_ms'):timing[n][k].append(v)
                if n.startswith('jaeho'):
                    selected=[x for x in row['plane_attempts'] if x['accepted_ground'] and x['height_valid']]
                    height=selected[-1]['height_at_origin_m'] if selected else None
                    planes[n]['frames']+=1;planes[n]['missing']+=int(height is None)
                    planes[n]['positive']+=int(height is not None and height>0)
                    if height is not None:
                        old=previous.get(n);stamp=source['scan_effective_stamp_ns']
                        if old and old[0]==source['phase'] and 0<stamp-old[1]<300000000 and abs(height-old[2])>.5:
                            planes[n]['height_jump_over_0p5m']+=1
                        previous[n]=(source['phase'],stamp,height)
                    else:previous.pop(n,None)
            out.write(json.dumps({'input_id':source['input_id'],'actors':observations},separators=(',',':'))+'\n')
            count+=1
            if count%500==0:print('scored '+str(count),flush=True)
    for stream in streams:stream.close()
    # Preserve stable score keys; profile is explicit and no prior output is overwritten.
    record={'frames':count,'jaeho_profile':a.jaeho_profile,'classes':classes,'objects':objects,'topology':graphs,
        'additional_complete_loss':additional,'degraded_actor_observations':degraded,
        'events':events,'plane_diagnostics':planes,'background_topology_stride':a.background_stride,
        'actor_topology_stride':1,'timing_ms':{n:{k:{'p'+str(q):float(np.percentile(v,q)) for q in [50,95,99]}
            for k,v in stages.items()} for n,stages in timing.items()},
        'sha256':{str(path):digest(path) for path in paths},
        'limits':['Previously analysed data, not held-out.','Plane jumps are diagnostics, not temporal world-surface GT.',
                  'No change to clustering or wall filters; structure mask is separate from ground score.',
                  'Timing of concurrent offline jobs is not an isolated runtime benchmark.']}
    (a.output/'summary.json').write_text(json.dumps(record,indent=2,sort_keys=True)+'\n')
    print(json.dumps({'frames':count,'additional':additional,'planes':planes}),flush=True)

if __name__=='__main__':main()
