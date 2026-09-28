#!/usr/bin/env python3
"""Known-label wall/object fixtures through the real pinned protection/filter."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import subprocess

import numpy as np
import yaml
from core import Event, evaluate
from run_geometry import convert_geometry, dump, geometry_only
from run_evaluation import digest_file


def rotation(r,p,y):
    from math import sin,cos
    r,p,y=np.deg2rad([r,p,y]);
    rx=np.array([[1,0,0],[0,cos(r),-sin(r)],[0,sin(r),cos(r)]])
    ry=np.array([[cos(p),0,sin(p)],[0,1,0],[-sin(p),0,cos(p)]])
    rz=np.array([[cos(y),-sin(y),0],[sin(y),cos(y),0],[0,0,1]])
    return rz@ry@rx


def projection(setting):
    l=setting['Vehicle']['LiDAR'];c=setting['Vehicle']['Camera']
    R=rotation(c['roll'],c['pitch'],c['yaw']).T@rotation(l['roll'],l['pitch'],l['yaw'])
    t=rotation(c['roll'],c['pitch'],c['yaw']).T@(np.array([l[k] for k in ('x','y','z')])-np.array([c[k] for k in ('x','y','z')]))
    optical=np.array([[0,-1,0],[0,0,-1],[1,0,0]])
    K=np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1]])
    return K@optical@np.column_stack([R,t])


def cloud_fixture(distance,gap,kind):
    ground=-1.45;wall_y=3.6
    floor=np.array([[x,y,ground,10] for x in np.arange(-20,45,.5) for y in np.arange(-3.5,3.6,.25)],dtype='<f4')
    walls=np.array([[x,y,z,20] for x in np.arange(-20,45,.25) for y in (-wall_y,wall_y)
                    for z in np.arange(ground+.3,2.6,.25)],dtype='<f4')
    ceiling=np.array([[x,y,3,30] for x in np.arange(0,40,1) for y in np.arange(-3.5,3.6,.5)],dtype='<f4')
    width,length,height={'car':(1.8,4,1.5),'person':(.5,.5,1.7),'low_unknown':(.6,.8,.3)}[kind]
    center_y=wall_y-width/2-gap
    xs=np.arange(distance-length/2,distance+length/2+.01,.15)
    ys=np.arange(center_y-width/2,center_y+width/2+.01,.1)
    zs=np.arange(ground+.21,ground+height+.01,.1)
    obj=np.array([[x,y,z,40] for x,y,z in itertools.product(xs,ys,zs)
                  if abs(x-xs[0])<.01 or abs(x-xs[-1])<.01 or abs(y-ys[0])<.01 or abs(y-ys[-1])<.01 or abs(z-zs[-1])<.01],dtype='<f4')
    points=np.concatenate([floor,walls,ceiling,obj])
    labels=np.concatenate([np.zeros(len(floor),np.uint8),np.ones(len(walls),np.uint8),
                           np.full(len(ceiling),2,np.uint8),np.full(len(obj),3,np.uint8)])
    return points,labels,obj


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('setting','params','executable','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('output exists')
    a.output.mkdir(parents=True);(a.output/'raw').mkdir()
    setting=yaml.safe_load(a.setting.read_text());P=projection(setting)
    cases=[(d,g,k,25,True) for d,g,k in itertools.product((5,15,25),(.1,.5,1),('car','person','low_unknown'))]
    cases.extend((15,.1,'car',delay,True) for delay in (0,10,26,50,130,200))
    # A valid camera frame with no boxes currently disables the wall filter's
    # ambiguous-point fallback. Keep this negative-detection condition visible
    # beside positive-box tests; it is pinned-code behavior, not a fix.
    cases.extend((15,.1,kind,10,False) for kind in ('car','person','low_unknown'))
    rows=[];truth={};base=10_000_000_000
    for ci,(distance,gap,kind,delay,has_box) in enumerate(cases):
        points,labels,obj=cloud_fixture(distance,gap,kind)
        name='case_{:02d}_{}_{}m_gap{}_delay{}'.format(ci,kind,distance,gap,delay)
        native=a.output/'raw'/(name+'.bin');points.tofile(native)
        uvh=np.column_stack([obj[:,:3],np.ones(len(obj))])@P.T
        uv=uvh[:,:2]/uvh[:,2:3]
        box={'class':kind if kind!='low_unknown' else 'unknown',
             'xmin':float(uv[:,0].min()),'xmax':float(uv[:,0].max()),
             'ymin':float(uv[:,1].min()),'ymax':float(uv[:,1].max())}
        for warm in range(5):
            stamp=base+(ci*1000+warm*100)*1_000_000
            if has_box:
                timely=Event('yolo',stamp,stamp,0,1)
                scan=Event('scan',stamp,stamp,1)
                if delay==0:events=[timely,scan]
                else:events=[Event('scan',stamp,stamp,0),Event('yolo',stamp+delay*1_000_000,stamp,1,1)]
                ref=evaluate([timely,scan],25_000_000)[0][0]
            else:
                scan=Event('scan',stamp,stamp,0)
                events=[scan,Event('yolo',stamp+delay*1_000_000,stamp,1,0)]
                ref=evaluate([scan],0)[0][0]
            trial=evaluate(events,25_000_000)[0][0]
            for d in (ref,trial):d['boxes']=[box] if has_box and d['selected_yolo_index'] is not None else []
            identity=name+'_frame'+str(warm)
            rows.append({'input_id':identity,'scene':name,'phase':name,'measurement':warm==4,
                'geometry_valid':True,'already_preprocessed':True,'roll':0,'pitch':0,'yaw':0,'speed_mps':0,
                'scan_stamp_ns':stamp,'scan_effective_stamp_ns':stamp,'cloud_path':str(native.relative_to(a.output)),
                'pose_evidence':{},'baseline':ref,'candidate':trial})
            if warm==4:truth[identity]={'labels':labels,'kind':kind,'delay_ms':delay,'distance_m':distance,
                'wall_gap_m':gap,'camera_result':'boxed_detection' if has_box else 'valid_empty_detection'}
    (a.output/'snapshots.jsonl').write_text(''.join(dump(r)+'\n' for r in rows))
    subprocess.run([str(a.executable),str(a.output/'snapshots.jsonl'),str(a.setting),str(a.params),
        str(a.output/'geometry.yaml_lines'),str(a.output/'voxel')],check=True,
        stdout=(a.output/'cpp.log').open('w'),stderr=subprocess.STDOUT)
    output,geometry_hash=convert_geometry(a.output/'geometry.yaml_lines',a.output)
    results=[]
    for row in output:
        if not row['measurement']:continue
        gt=truth[row['input_id']];obj=gt['labels']==3;wall=gt['labels']==1
        result={k:v for k,v in gt.items() if k!='labels'};result['input_id']=row['input_id']
        result['geometry_equal_to_timely']=geometry_only(row['baseline'])==geometry_only(row['candidate'])
        for mode in ('baseline','candidate'):
            labels=np.array(row[mode]['labels'])
            protected=np.zeros(len(labels),bool)
            protected[row[mode].get('protected_indices',[])]=True
            result[mode]={'object_points':int(obj.sum()),'object_points_kept':int(((labels==0)&obj).sum()),
                'object_points_removed_as_ground':int(((labels==2)&obj).sum()),
                'object_points_removed_as_wall':int(((labels==3)&obj).sum()),
                'object_points_protected':int((obj&protected).sum()),
                'protected_object_points_removed':int((obj&protected&(labels!=0)).sum()),
                'gt_wall_points_kept':int(((labels==0)&wall).sum()),'final_objects':row[mode]['final_objects']}
        if gt['camera_result']=='boxed_detection' and gt['delay_ms']<=25 and not result['geometry_equal_to_timely']:
            raise AssertionError('same box within wait changed geometry: '+row['input_id'])
        results.append(result)
    empty=[r for r in results if r['camera_result']=='valid_empty_detection']
    report={'schema':'asmc-camera-protection-synthetic-v2','cases':len(results),
        'same_box_within_wait_passed':all(r['geometry_equal_to_timely'] for r in results
            if r['camera_result']=='boxed_detection' and r['delay_ms']<=25),
        'valid_empty_detection_cases':len(empty),
        'valid_empty_geometry_changed':sum(not r['geometry_equal_to_timely'] for r in empty),
        'valid_empty_object_wall_removal_delta':{
            r['input_id']:r['candidate']['object_points_removed_as_wall']-
                          r['baseline']['object_points_removed_as_wall'] for r in empty},
        'results':results,'geometry_sha256':geometry_hash,
        'known_limits':['Synthetic geometry and analytic boxes; not simulated visibility or detection accuracy',
                       'Valid empty detections expose the pinned pipeline behavior when its ambiguity fallback is disabled',
                       'OBB and cluster counts are not object recall; clusters can merge with wall points'],
        'snapshot_sha256':digest_file(a.output/'snapshots.jsonl')}
    (a.output/'summary.json').write_text(json.dumps(report,sort_keys=True,indent=2)+'\n')
    tool_dir=Path(__file__).parent
    tool_names=('core.py','run_evaluation.py','run_geometry.py','run_synthetic_geometry.py')
    manifest={'schema':'asmc-camera-protection-synthetic-manifest-v1',
        'runtime_connected':False,
        'pinned_main_revision':'6435af10671235e735e4a9ed1bd747bd8d8389ac',
        'inputs':{name:{'path':str(path.resolve()),'sha256':digest_file(path)}
                  for name,path in (('setting',a.setting),('resolved_params',a.params),
                                    ('executable',a.executable))},
        'tools_sha256':{name:digest_file(tool_dir/name) for name in tool_names},
        'snapshot_sha256':digest_file(a.output/'snapshots.jsonl'),
        'geometry_sha256':geometry_hash,
        'summary_sha256':digest_file(a.output/'summary.json'),
        'limitations':['Analytic geometry is not simulator visibility or detector accuracy',
            'Pipeline class/cluster outputs are not object recall',
            'This synthetic study does not alter operating ROS code']}
    (a.output/'manifest.json').write_text(json.dumps(manifest,sort_keys=True,indent=2)+'\n')
    print(dump({k:v for k,v in report.items() if k not in ('results','known_limits')}))


if __name__=='__main__':main()
