#!/usr/bin/env python3
"""Four panels of identical points and ground masks; labels are scoring-only."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ground_v3'))
from plot import find

def main():
    p=argparse.ArgumentParser()
    for n in ['input','baseline','candidate']:p.add_argument(n,type=Path)
    p.add_argument('input_id');p.add_argument('output',type=Path);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    src=find(a.input/'snapshots.jsonl',a.input_id);old=find(a.baseline,a.input_id)
    j=find(a.candidate/'jaeho_support.jsonl',a.input_id);pw=find(a.candidate/'patchwork_support.jsonl',a.input_id)
    cloud=np.fromfile(a.input/src['cloud_path'],dtype='<f4').reshape(-1,4)
    panels=[('Jaeho baseline',old['methods']['jaeho']),('Jaeho height + supported fallback',j['methods']['jaeho']),
        ('Patchwork flat guard',old['methods']['patchwork']),('Patchwork supported guard',pw['methods']['patchwork'])]
    with np.load(a.input/'labels'/(a.input_id+'.npz')) as labels:
        actor=np.isin(labels['semantic'],[86,118,125,135,145,155,164,165,175,185])&(labels['instance']!=0)
    fig,axes=plt.subplots(2,4,figsize=(17,8),constrained_layout=True)
    for col,(name,data) in enumerate(panels):
        mask=np.array(data['mask'])
        for row,coordinate in enumerate([1,2]):
            ax=axes[row,col]
            for sel,color,label in [(mask==1,'#bbbbbb','ground removed'),((mask==2)&~actor,'#21834d','background kept'),
                ((mask==2)&actor,'#1468c0','actor kept'),((mask==1)&actor,'#e13949','actor removed')]:
                ax.scatter(cloud[sel,0],cloud[sel,coordinate],s=1,c=color,label=label,rasterized=True)
            ax.set(xlim=(-30,50),ylim=(-20,20) if row==0 else (-3,5),title=name if row==0 else '',
                xlabel='sensor x [m]',ylabel='sensor '+('y' if row==0 else 'z')+' [m]');ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=7,markerscale=3)
    fig.suptitle(a.input_id+' / identical observed point lineage; no detector clustering')
    a.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=140)

if __name__=='__main__':main()
