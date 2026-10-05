#!/usr/bin/env python3
"""GT Capture and unlabeled bag proxies are intentionally separate outputs."""
import argparse
from collections import defaultdict, Counter
import json
from pathlib import Path
import numpy as np
from prepare import rows, dump

POLICIES=('jaeho','original','singleton','fixed')
GROUPS={'ground':{85,127,128,136,144,170,255},'vehicle':{86,125,135,145,155,165,175,185},
        'pedestrian':{118},'obstacle':{164},'standing_structure_reference':{109}}

def summarize(values):
    return {'p'+str(q):float(np.percentile(values,q)) for q in (50,95,99)}

def increment(counter,mask,selected):
    counter['points']+=int(selected.sum())
    for k,v in [('ground_removed',1),('retained',2),('vertical_removed',3),('unclassified_or_outside',0)]:
        counter[k]+=int((selected&(mask==v)).sum())

def metrics(counters):
    return {name:{**dict(v),'retained_fraction':v['retained']/v['points'] if v['points'] else None,
                        'ground_removed_fraction':v['ground_removed']/v['points'] if v['points'] else None}
            for name,v in counters.items()}

def proxy_bands(pts):
    selected=(pts[:,0]>=3)&(pts[:,0]<=10)&(abs(pts[:,1])<=3)&(pts[:,2]>-2.3)&(pts[:,2]<-1.2)
    if selected.sum()<25:return {}
    selected&=abs(pts[:,2]-np.median(pts[selected,2]))<.12
    if selected.sum()<25:return {}
    abc=np.linalg.lstsq(np.column_stack((pts[selected,:2],np.ones(selected.sum()))),pts[selected,2],rcond=None)[0]
    dz=pts[:,2]-(pts[:,0]*abc[0]+pts[:,1]*abc[1]+abc[2])
    corridor=(pts[:,0]>=3)&(pts[:,0]<=20)&(abs(pts[:,1])<5)
    return {'road_proxy':corridor&(abs(dz)<=.06),'low_proxy':corridor&(dz>=.10)&(dz<.30),
            'elevated_proxy':corridor&(dz>=.30)&(dz<=1.20)}

def load_group(root,dataset,rep,att):
    policies=POLICIES if att=='pitch' else POLICIES[1:]
    return {p:rows(root/f'{dataset}_{rep}_{att}_{p}.jsonl') for p in policies}

def main():
    p=argparse.ArgumentParser();p.add_argument('inputs',type=Path);p.add_argument('results',type=Path)
    p.add_argument('output',type=Path);a=p.parse_args();a.output.mkdir(exist_ok=False)
    report={'parity':{},'groups':{},'objects':[], 'adoption':'HOLD',
            'limits':['Capture62 is a historical sensor, no measured IMU, not held-out data',
                      'Unlabeled bag road/low/elevated bands are geometric proxies, not semantic truth',
                      'jaeho timing includes existing clustering/OBB; PP segment timing is not like-for-like']}
    for dataset in ('manual','stationary','capture'):
        old={r['input_id']:r for r in rows(a.inputs/'oracle'/f'{dataset}.jsonl')}
        new=rows(a.results/f'{dataset}_voxel_pitch_original.jsonl')
        assert all(r['mask']==old[r['input_id']]['mask'] for r in new),dataset+' mask parity'
        assert all(abs(r['height_after']-old[r['input_id']]['height_after'])<1e-10 for r in new),dataset+' height parity'
        report['parity'][dataset]={'frames':len(new),'exact_mask':True,'height_tolerance_from_old_decimal_dump':1e-10}
    previous=rows(a.inputs/'oracle/manual_fourway.jsonl')
    jaeho=rows(a.results/'manual_voxel_pitch_jaeho.jsonl')
    assert len(previous)==len(jaeho)==644
    for old,new in zip(previous,jaeho):
        assert old['input_id']==new['input_id']
        assert old['ref_voxel']==[2 if v==3 else v for v in new['mask']]
    report['parity']['jaeho_manual']={'frames':644,'mask_after_recorded_wall_mapping':True}
    samples={r['frame']:r for r in json.loads((a.inputs/'capture/labels.json').read_text())}
    for dataset in ('manual','stationary','capture'):
      meta=rows(a.inputs/dataset/'snapshots.jsonl')
      for rep,att in [('voxel','pitch'),('raw','pitch'),('voxel','full')]:
        group=load_group(a.results,dataset,rep,att)
        counts={p:defaultdict(Counter) for p in group};phase_counts={p:defaultdict(lambda:defaultdict(Counter)) for p in group}
        object_stats={p:Counter() for p in group};object_rows=[]
        for i,m in enumerate(meta):
            pts=np.fromfile(a.inputs/dataset/m['cloud_path'],dtype='<f4').reshape(-1,4)
            masks={p:np.asarray(records[i]['mask'],dtype=np.uint8) for p,records in group.items()}
            for pol,records in group.items():
                r=records[i]
                assert r['input_id']==m['input_id'] and r['stamp_ns']==m['scan_effective_stamp_ns']
                assert r['arrival_ns']==m['scan_arrival_ns'] and len(masks[pol])==len(pts)
            if dataset!='capture':
                bands=proxy_bands(pts)
                for pol,mask in masks.items():
                    for name,selected in bands.items():
                        increment(counts[pol][name],mask,selected)
                        increment(phase_counts[pol][m['phase']][name],mask,selected)
                continue
            with np.load(a.inputs/'capture'/(m['input_id']+'.labels.npz')) as labels:
                sem=labels['semantic'];instance=labels['instance']
            # Use the same representation's reference ROI for all policies, also in full-RP sensitivity.
            reference=group.get('jaeho')
            if reference is None:
                if i==0:full_reference=rows(a.results/f'capture_{rep}_pitch_jaeho.jsonl')
                roi=np.asarray(full_reference[i]['mask'])!=0
            else:roi=masks['jaeho']!=0
            radial=np.linalg.norm(pts[:,:2],axis=1)
            asphalt=roi&(sem==127)&(radial>=3)&(radial<10)
            ground_z=float(np.median(pts[asphalt,2])) if asphalt.sum()>=20 else None
            for pol,mask in masks.items():
                for name,codes in GROUPS.items():
                    selected=roi&np.isin(sem,list(codes));increment(counts[pol][name],mask,selected)
                    for lo,hi in ((0,10),(10,20),(20,40),(40,60)):
                        increment(counts[pol][f'{name}_range_{lo}_{hi}'],mask,selected&(radial>=lo)&(radial<hi))
                    if name in ('vehicle','pedestrian','obstacle') and ground_z is not None:
                        increment(counts[pol][name+'_low_0_20cm'],mask,selected&(pts[:,2]>=ground_z)&(pts[:,2]<ground_z+.2))
            actors=set().union(*(GROUPS[k] for k in ('vehicle','pedestrian','obstacle')))
            for box in samples[m['input_id']]['boxes']:
                member=roi&(instance==int(box['id']))&np.isin(sem,list(actors))
                n=int(member.sum())
                if n<10:continue
                retained={pol:int((member&(mask==2)).sum()) for pol,mask in masks.items()}
                ob={'input_id':m['input_id'],'instance':int(box['id']),'class':box['class'],
                    'points':n,'distance_m':float(np.median(radial[member])),'retained':retained}
                object_rows.append(ob)
                for pol,num in retained.items():
                    object_stats[pol].update(observations=1,complete_loss=int(num==0),retained_80pct=int(num>=.8*n))
        key=f'{dataset}_{rep}_{att}'
        result={}
        for pol,records in group.items():
            heights=[r['height_after'] for r in records]
            result[pol]={'metrics':metrics(counts[pol]),'objects':dict(object_stats[pol]),
                'phase_proxy_metrics':{k:metrics(v) for k,v in phase_counts[pol].items()},
                'height':{'min':min(heights),'max':max(heights),'last':heights[-1],
                          'below_1_4m':sum(h<1.4 for h in heights)},
                'timing_ms':{stage:summarize([r['timing_ms'][stage] for r in records]) for stage in records[0]['timing_ms']},
                'update_reasons':dict(Counter(u['reason'] for r in records for u in r['updates']))}
        report['groups'][key]=result
        if 'jaeho' in result:
            # The runner instantiates PP for uniform plumbing, but Jaeho never uses its height.
            result['jaeho']['height']=None
        if dataset=='capture':report['objects'].append({'group':key,'observations':object_rows})
    report['quiet_primary_timing_ms']={}
    for pol in POLICIES[1:]:
        quiet=rows(a.results/f'manual_voxel_pitch_{pol}_quiet.jsonl')
        report['quiet_primary_timing_ms'][pol]={s:summarize([r['timing_ms'][s] for r in quiet]) for s in quiet[0]['timing_ms']}
    primary=next(x['observations'] for x in report['objects'] if x['group']=='capture_voxel_pitch')
    report['candidate_gate']={
        'fixed_height_stable':report['groups']['manual_voxel_pitch']['fixed']['height']['below_1_4m']==0,
        'additional_complete_loss_vs_original':sum(o['retained']['fixed']==0 and o['retained']['original']>0 for o in primary),
        'additional_complete_loss_vs_jaeho':sum(o['retained']['fixed']==0 and o['retained']['jaeho']>0 for o in primary),
        'note':'Capture reset each frame: adaptive update occurs after classification; cold-start equality is expected'}
    dump(a.output/'summary.json',report)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['axes.grid']=True
    edges=np.concatenate((np.linspace(1,8.375,3),np.linspace(8.375,15.75,5)[1:],
                          np.linspace(15.75,30.5,5)[1:],np.linspace(30.5,60,5)[1:]))
    hist=np.zeros(len(edges)-1,dtype=np.int64)
    for m in rows(a.inputs/'capture/snapshots.jsonl'):
        cloud=np.fromfile(a.inputs/'capture'/m['cloud_path'],dtype='<f4').reshape(-1,4)
        with np.load(a.inputs/'capture'/(m['input_id']+'.labels.npz')) as labels:
            asphalt=labels['semantic']==127
        hist+=np.histogram(np.linalg.norm(cloud[asphalt,:2],axis=1),edges)[0]
    dump(a.output/'ground_observability.json',{'sensor_origin_xy_radius_m':edges.tolist(),
        'asphalt_gt_counts':hist.tolist(),'profile':'historical Capture62, zero attitude, before ROI',
        'flat_road_visibility_m':1.8/np.tan(np.deg2rad(15))})
    fig,ax=plt.subplots(figsize=(12,4))
    ax.bar(range(len(hist)),hist,color=['orange']+['steelblue']*(len(hist)-1))
    ax.set_xticks(range(len(hist)))
    ax.set_xticklabels([f'{edges[i]:.2f}-{edges[i+1]:.2f}' for i in range(len(hist))],rotation=45,ha='right')
    ax.set(xlabel='CZM ring horizontal radius (m)',ylabel='asphalt GT points',yscale='log',
           title='Ground visibility: ring 0 supplies the height update; 61/62 scenes have no asphalt there')
    fig.tight_layout();fig.savefig(a.output/'ground_observability.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(2,1,figsize=(12,7))
    for pol in POLICIES[1:]:
        r=rows(a.results/f'manual_voxel_pitch_{pol}.jsonl')
        ax[0].plot([x['height_after'] for x in r],label=pol)
        ring0=[p for x in r for p in x['patches'] if p['ring']==0 and p['accepted']]
        ax[1].hist([-p['z'] for p in ring0],bins=np.linspace(0,2.3,40),alpha=.45,label=pol)
    ax[0].set(ylabel='sensor_height (m)',xlabel='ordered scan (644)',title='Same input: height policies')
    ax[1].set(xlabel='negative patch centroid z (m), NOT ground truth',ylabel='accepted ring-0 patches')
    for x in ax:x.legend()
    fig.tight_layout();fig.savefig(a.output/'height_and_evidence.png',dpi=150);plt.close(fig)
    obs=next(x['observations'] for x in report['objects'] if x['group']=='capture_voxel_pitch')
    failures=[o for o in obs if o['retained']['fixed']==0]
    for index,ob in enumerate(failures[:2]):
        frame=ob['input_id'];m=next(r for r in rows(a.inputs/'capture/snapshots.jsonl') if r['input_id']==frame)
        pts=np.fromfile(a.inputs/'capture'/m['cloud_path'],dtype='<f4').reshape(-1,4)
        with np.load(a.inputs/'capture'/(frame+'.labels.npz')) as lab:member=lab['instance']==ob['instance']
        fig,axes=plt.subplots(2,3,figsize=(13,7))
        for col,pol in enumerate(('jaeho','original','fixed')):
            rec=next(r for r in rows(a.results/f'capture_voxel_pitch_{pol}.jsonl') if r['input_id']==frame)
            mask=np.asarray(rec['mask']);center=pts[member,:2].mean(axis=0)
            visible=np.linalg.norm(pts[:,:2]-center,axis=1)<6
            for axis,dim in zip(axes[:,col],(1,2)):
                axis.scatter(pts[visible,0],pts[visible,dim],s=2,c='lightgray')
                for value,color in ((1,'red'),(2,'green'),(3,'orange'),(0,'black')):
                    selected=member&(mask==value)
                    axis.scatter(pts[selected,0],pts[selected,dim],s=15,c=color,label=f'label {value}')
                axis.set(xlabel='sensor x (m)',ylabel='sensor '+('y' if dim==1 else 'z')+' (m)',title=pol)
        fig.suptitle(f'GT instance {ob["instance"]}, {ob["points"]} ROI points, range {ob["distance_m"]:.1f}m\nred=removed ground; green=retained')
        fig.tight_layout();fig.savefig(a.output/f'far_loss_{index+1}.png',dpi=150);plt.close(fig)
    print(json.dumps({'parity':report['parity'],'capture_primary_objects':
        {k:v['objects'] for k,v in report['groups']['capture_voxel_pitch'].items()},'adoption':report['adoption']},indent=2))

if __name__=='__main__':main()
