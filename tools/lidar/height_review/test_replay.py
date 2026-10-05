#!/usr/bin/env python3
"""Synthetic rays are analytic fixtures, not a MORAI scan-time/ring reconstruction."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import numpy as np
from prepare import rows, dump
from run import comparable

def make_cloud(slope,roll,pitch,object_only=False):
    cr,sr=np.cos(roll),np.sin(roll);cp,sp=np.cos(pitch),np.sin(pitch)
    R=np.array([[cp,sp*sr,sp*cr],[0,cr,-sr],[-sp,cp*sr,cp*cr]])
    if object_only:
        x,y=np.meshgrid(np.linspace(2,3.5,25),np.linspace(1,2,25))
        leveled=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,-1.15)))
        cloud=leveled@R
    else:
        az,el=np.meshgrid(np.linspace(-np.pi,np.pi,720,endpoint=False),np.deg2rad(np.arange(-15,16,2)))
        ray=np.column_stack((np.cos(el.ravel())*np.cos(az.ravel()),
                             np.cos(el.ravel())*np.sin(az.ravel()),np.sin(el.ravel())))
        direction=ray@R.T
        denom=direction[:,2]-slope*direction[:,0]
        distance=np.divide(-1.8,denom,out=np.full(len(ray),np.nan),where=abs(denom)>1e-9)
        good=(distance>0)&(distance<60)
        cloud=ray[good]*distance[good,None]
    return np.column_stack((cloud,np.arange(len(cloud))%251)).astype('<f4')

def main():
    p=argparse.ArgumentParser();p.add_argument('binary',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();summary={'synthetic':[],'invalid_rejected':[]}
    with tempfile.TemporaryDirectory(prefix='lidar-h1-fixtures-') as tmp:
        root=Path(tmp);snapshots=[]
        for name,slope,roll,pitch,objects in [('flat_blind_ring',0,0,0,False),
            ('uphill',.08,0,0,False),('downhill',-.08,0,0,False),
            ('attitude',0,.08,-.05,False),('objects_without_ground',0,0,0,True)]:
            cloud=make_cloud(slope,roll,pitch,objects);cloud.tofile(root/(name+'.bin'))
            for repeat in range(3):
                snapshots.append({'input_id':name+str(repeat),'phase':name,'cloud_path':name+'.bin',
                    'roll':roll,'pitch':pitch,'geometry_valid':True,'controlled_camera_state':'missing',
                    'scan_effective_stamp_ns':(repeat+1)*1000000000,'scan_arrival_ns':(repeat+1)*1000000000,
                    'imu_stamp_ns':(repeat+1)*1000000000,'imu_arrival_ns':(repeat+1)*1000000000})
        source=root/'snapshots.jsonl'
        source.write_text(''.join(json.dumps(r)+'\n' for r in snapshots))
        for policy in ('original','singleton','fixed'):
            output=[]
            for repeat in range(2):
                dest=root/f'{policy}{repeat}.jsonl'
                subprocess.run([str(a.binary),str(source),'voxel','full',policy,'trace',str(dest)],check=True,stdout=subprocess.DEVNULL)
                output.append(rows(dest))
            assert [comparable(r) for r in output[0]]==[comparable(r) for r in output[1]]
            if policy=='fixed':assert all(r['height_after']==1.8 for r in output[0])
            summary['synthetic'].append({'policy':policy,'frames':len(output[0]),'deterministic':True,
                'outcomes':[{'id':r['input_id'],'height':r['height_after'],
                             'ground_points':r['mask'].count(1),'kept_points':r['mask'].count(2)} for r in output[0]]})
        base=snapshots[0]
        cases={'zero_stamp':{'scan_effective_stamp_ns':0},'arrival_before_stamp':{'scan_arrival_ns':1},
               'future_imu':{'imu_stamp_ns':2000000000},'stale_imu':{'imu_stamp_ns':1},
               'imu_arrives_later':{'imu_arrival_ns':2000000000},'invalid_imu':{'geometry_valid':False},
               'nan_roll':{'roll':float('nan')},'bad_camera':{'controlled_camera_state':'unsupported'}}
        for name,change in cases.items():
            source.write_text(json.dumps({**base,**change})+'\n')
            r=subprocess.run([str(a.binary),str(source),'voxel','pitch','fixed','quiet',str(root/(name+'.jsonl'))],capture_output=True)
            assert r.returncode!=0,name;summary['invalid_rejected'].append(name)
        for name,other in [('duplicate',base),('regression',{**base,'scan_effective_stamp_ns':999999999})]:
            source.write_text(json.dumps(base)+'\n'+json.dumps(other)+'\n')
            r=subprocess.run([str(a.binary),str(source),'voxel','pitch','fixed','quiet',str(root/(name+'.jsonl'))],capture_output=True)
            assert r.returncode!=0,name;summary['invalid_rejected'].append(name)
        for key in ('roll','pitch','geometry_valid'):
            invalid={k:v for k,v in base.items() if k!=key};source.write_text(json.dumps(invalid)+'\n')
            r=subprocess.run([str(a.binary),str(source),'voxel','pitch','fixed','quiet',str(root/(key+'.jsonl'))],capture_output=True)
            assert r.returncode!=0;summary['invalid_rejected'].append('missing_'+key)
    dump(a.output,summary);print(json.dumps({'synthetic_conditions':5,'policies':3,'repeats':2,'invalid_cases':len(summary['invalid_rejected'])}))

if __name__=='__main__':main()
