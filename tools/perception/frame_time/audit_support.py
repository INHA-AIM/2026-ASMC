#!/usr/bin/env python3
"""C0 supplemental fixed-support and qualified-velocity evidence.

Native probe calls Runtime/HdMapAudit/DeliveryPreview. Never modifies operating
arrays. Reuses frozen C0 state estimates; reports their time difference explicitly.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from audit_frame_time import ROOT,read,stats,sha,run,write_json


def velocity_contract(bag,trace):
    import rosbag,math
    scans={s['stamp_ns']:s for s in read(trace)};counts=Counter();deltas=[];identity_ok=True
    with rosbag.Bag(str(bag)) as stream:
        for _,m,_ in stream.read_messages():
            array=m.observation.objects;original=scans[array.header.stamp.to_nsec()]['typed']['map']
            identity_ok &= [o.track_id for o in array.objects]==[o['track_id'] for o in original.get('objects',[])]
            identity_ok &= len(array.objects)==len(m.motion_states)
            for obj,raw,state in zip(array.objects,original.get('objects',[]),m.motion_states):
                counts['qualified_valid' if obj.absolute_velocity_valid else 'qualified_unknown']+=1
                counts['motion_'+str(state)]+=1
                if not obj.absolute_velocity_valid:identity_ok &= state==0
                if obj.absolute_velocity_valid and raw['absolute_velocity_valid']:
                    v=obj.absolute_velocity_mps;deltas.append(math.sqrt(sum((a-b)**2 for a,b in zip((v.x,v.y,v.z),raw['absolute_velocity']))))
    return dict(identity_and_validity_passed=identity_ok,counts=dict(counts),
        qualified_vs_classifier_speed_vector_delta_mps=stats(deltas),
        meaning='Different estimators; difference is not velocity error. No GT comparison.')


def analyze(manifest,storage,evidence,probes,out):
    result={'schema':'asmc.frame_time_support.v1','bags':{},'pins':{}}
    for item in manifest['inputs']:
        if item['id'] not in ('normal','tunnel'):continue
        name=item['id'];bag=storage/item['path'];states=evidence/name/'planning.jsonl';trace=evidence/name/'lidar.jsonl'
        if sha(bag)!=item['sha256']:raise ValueError('bag SHA')
        result['pins'][str(states)]=sha(states);result['pins'][str(trace)]=sha(trace)
        target=out/(name+'-support.jsonl')
        run([probes/'support_probe',bag,ROOT/'src/asmc_lidar/config/lidar_pipeline.yaml',states,target],out/(name+'-support.log'))
        scans=list(read(trace));rows=list(read(target));equal=len(scans)==len(rows) and all(s['typed']['local']==r['typed_local'] and s['typed']['map']==r['typed_map'] for s,r in zip(scans,rows))
        objects=[o for r in rows for o in r['objects']];changes=[]
        for r in rows:
            for o in r['objects']:
                if o['original']['preview']['decision']!=o['alternative']['preview']['decision']:
                    changes.append(dict(seq=r['seq'],track_id=o['track_id'],before=o['original']['preview']['reason'],after=o['alternative']['preview']['reason']))
        result['bags'][name]=dict(scans=len(rows),unchanged_delivery_arrays=equal,objects=len(objects),
            support_points=sum(o['point_count'] for o in objects),decision_changes=changes,
            interior_ratio_delta=stats(abs(o['original']['preview']['interior_road_ratio']-o['alternative']['preview']['interior_road_ratio']) for o in objects),
            velocity=velocity_contract(evidence/name/'planning.bag',trace))
    result['passed']=all(b['unchanged_delivery_arrays'] and b['velocity']['identity_and_validity_passed'] for b in result['bags'].values())
    result['limits']=['All observed pre-delivery objects, fixed OBB and full voxel cluster supports.',
        'Alternative xy/yaw from asynchronous planning state; preserves original roll/pitch/z.',
        'No new pose extrapolation, no independent positional bound.',
        'Camera protection absent in these bags; no simulated camera labels added.',
        'Map classification changes are sensitivity, never an operating policy recommendation.']
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('manifest','storage-root','evidence','probes','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--run-id',required=True);a=p.parse_args()
    if a.output.name!=a.run_id:raise ValueError('run mismatch')
    a.output.mkdir(parents=True,exist_ok=False)
    result=analyze(json.loads(a.manifest.read_text()),a.storage_root,a.evidence,a.probes,a.output)
    write_json(a.output/'summary.json',result);print(json.dumps(result,indent=2))
    sys.exit(0 if result['passed'] else 1)
