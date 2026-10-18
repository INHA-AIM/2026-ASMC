#!/usr/bin/env python3
"""Export scientific figures from observed points and unvalidated tracker states."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);a=p.parse_args()
    raw=[json.loads(l) for l in (a.root/'tracker-common-pitch.jsonl').read_text().splitlines()]
    first=raw[0]['stamp_ns'];values=[]
    for row in raw:
        for obj in row['objects']:
            if obj['id']==76:
                values.append(((row['stamp_ns']-first)/1e9,np.linalg.norm(obj['kf_ca'][2:4]),obj['lidar_misses']))
    values=np.asarray(values)
    fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
    axes[0].plot(values[:,0],values[:,1],color='#777',lw=1)
    for condition,color,label in ((values[:,2]==0,'#277da8','LiDAR miss counter = 0'),
                                   (values[:,2]>0,'#dc7a26','LiDAR miss counter > 0')):
        axes[0].scatter(values[condition,0],values[condition,1],s=13,color=color,label=label)
    axes[0].set(ylabel='Internal relative KF speed (m/s)',title='Track ID 76: diagnostic output, not physical actor GT')
    axes[0].legend(fontsize=9);axes[1].plot(values[:,0],values[:,2],color='#dc7a26')
    axes[1].set(xlabel='Seconds since first scan',ylabel='LiDAR miss counter')
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle('Saved common_pitch snapshot; no camera boxes; velocity validity is not established',fontsize=10)
    fig.tight_layout();fig.savefig(a.root/'tracking-diagnostic.png',dpi=150);plt.close(fig)
    rows=[json.loads(l) for l in (a.root/'inputs/frames.jsonl').read_text().splitlines()]
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    for col,index in enumerate((64,112)):
        row=next(r for r in rows if r['scan_index']==index)
        stamp=row['stamp_ns']
        with np.load(a.root/'inputs/clouds'/f'{stamp}.npz') as data:
            xyz=data['xyz'];raw_index=data['raw_index']
        labels=np.full(len(xyz),-1)
        entries=np.loadtxt(a.root/'pcl-indices'/f'{stamp}.tsv',dtype=np.int64).reshape(-1,2)
        offsets=np.searchsorted(raw_index,entries[:,0])
        if not np.array_equal(raw_index[offsets],entries[:,0]):raise ValueError('lost raw lineage')
        labels[offsets]=entries[:,1]
        region=(xyz[:,0]>=-5)&(xyz[:,0]<=35)&(np.abs(xyz[:,1])<=15)&(xyz[:,2]>=-4)&(xyz[:,2]<=10)
        for label,color in ((-1,'#cccccc'),(0,'#2878b5'),(1,'#dc7a26'),(2,'#4b9951'),(3,'#9159a6')):
            chosen=region&(labels==label)
            axes[0,col].scatter(xyz[chosen,0],xyz[chosen,1],s=1,color=color,label='unknown' if label<0 else f'plane {label}')
            chosen &= (xyz[:,0]>=0)&(xyz[:,0]<=20)
            axes[1,col].scatter(xyz[chosen,1],xyz[chosen,2],s=2,color=color)
        axes[0,col].set(xlim=(-5,35),ylim=(-15,15),xlabel='raw sensor x (m)',ylabel='raw sensor y (m)',title=f'scan {index}: observed support only')
        axes[1,col].set(xlim=(-15,15),ylim=(-4,10),xlabel='raw sensor y (m)',ylabel='raw sensor z (m)',title='Cross-section: points at x = 0..20 m')
        for ax in axes[:,col]:ax.grid(alpha=.2)
    axes[0,0].legend(markerscale=4,fontsize=8)
    fig.suptitle('PCL RANSAC surfaces: no gravity alignment, semantic class or deletion decision',fontsize=11)
    fig.tight_layout();fig.savefig(a.root/'structure-evidence.png',dpi=150);plt.close(fig)


if __name__=='__main__':main()
