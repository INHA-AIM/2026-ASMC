#!/usr/bin/env python3
"""Compare fixed-input BEV/side views; no detector output is used as ground truth."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ground_live_review'))
from plot import find
from metrics import CLASSES,ACTORS

p=argparse.ArgumentParser();p.add_argument('input',type=Path);p.add_argument('results',type=Path)
p.add_argument('identity');p.add_argument('output',type=Path);p.add_argument('--profiles',nargs='+',required=True)
p.add_argument('--xlim',nargs=2,type=float,default=[-30,50]);p.add_argument('--ylim',nargs=2,type=float,default=[-20,20])
a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
s=find(a.input/'snapshots.jsonl',a.identity);cloud=np.fromfile(a.input/s['cloud_path'],dtype='<f4').reshape(-1,4)
xyz=cloud[:,:3].astype(float)+[.8,0,1.45];labels=a.input/'labels'/(a.identity+'.npz')
actor=np.zeros(len(cloud),bool)
if labels.exists():
    with np.load(labels) as gt: actor=np.isin(gt['semantic'],[k for k,v in CLASSES.items() if v in ACTORS])&(gt['instance']!=0)
fig,axes=plt.subplots(2,len(a.profiles),figsize=(5*len(a.profiles),7),squeeze=False,constrained_layout=True)
for col,profile in enumerate(a.profiles):
    method=profile[3:] if profile.startswith('v1_') else 'patchwork' if profile.startswith('pp_') else 'linefit'
    path=a.results/(('v1' if profile.startswith('v1_') else profile)+'.jsonl')
    r=find(path,a.identity)['methods'][method];mask=np.array(r['mask']);roi=mask!=0
    groups=[(roi&~actor&(mask==1),'#aaaaaa','ground'),(roi&~actor&(mask==2),'#298c52','retained background'),
            (roi&actor&(mask==2),'#2d77c7','retained actor'),(roi&actor&(mask==1),'#e12f44','actor removed'),
            (roi&(mask==3),'#ca75cd','unclassified')]
    for row,(x,y) in enumerate(((0,1),(0,2))):
        ax=axes[row,col]
        for select,color,label in groups:ax.scatter(xyz[select,x],xyz[select,y],s=3,c=color,label=label,rasterized=True)
        ax.set(xlabel='base_link x [m]',ylabel=('y' if row==0 else 'z')+' [m]',title=profile,
               xlim=a.xlim,ylim=a.ylim if row==0 else [-1,4]);ax.grid(alpha=.2)
axes[0,0].legend(fontsize=7);fig.suptitle(a.identity+(' / observed semantic + instance GT' if labels.exists() else ' / unlabelled IMU sequence; no accuracy claim'))
a.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=140);plt.close(fig)
