#!/usr/bin/env python3
"""Observed-point BEV and side diagnostics, not synthetic object ground truth."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from metrics import CLASSES,ACTORS


def find(path,identity):
    with path.open() as f:
        for text in f:
            row=json.loads(text)
            if row["input_id"]==identity:return row
    raise KeyError(identity)


def main():
    p=argparse.ArgumentParser()
    p.add_argument("input",type=Path);p.add_argument("results",type=Path)
    p.add_argument("identity");p.add_argument("output",type=Path)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    source=find(a.input/"snapshots.jsonl",a.identity)
    result=find(a.results,a.identity)
    cloud=np.fromfile(a.input/source["cloud_path"],dtype="<f4").reshape(-1,4)
    xyz=cloud[:,:3].astype(float)+[.8,0,1.45]
    with np.load(a.input/"labels"/(a.identity+".npz")) as gt:
        actor=np.isin(gt["semantic"],[k for k,v in CLASSES.items() if v in ACTORS])&(gt["instance"]!=0)
    fig,axes=plt.subplots(2,3,figsize=(14,7),constrained_layout=True)
    for col,method in enumerate(("jaeho","patchwork","linefit")):
        mask=np.asarray(result["methods"][method]["mask"]);roi=mask!=0
        groups=[(roi&~actor&(mask==1),"#aaaaaa","removed background"),
                (roi&~actor&(mask==2),"#298c52","retained background"),
                (roi&actor&(mask==2),"#2d77c7","retained actor"),
                (roi&actor&(mask==1),"#e12f44","actor removed as ground"),
                (roi&(mask==3),"#ca75cd","unclassified")]
        for row,(x,y) in enumerate(((0,1),(0,2))):
            ax=axes[row,col]
            for selected,color,label in groups:
                ax.scatter(xyz[selected,x],xyz[selected,y],s=1.5,c=color,label=label,rasterized=True)
            ax.set(xlabel="base_link x [m]",ylabel=("y" if row==0 else "z")+" [m]",
                   title=method+" — "+("BEV" if row==0 else "side"))
            ax.set_xlim(-30,50);ax.set_ylim((-20,20) if row==0 else (-1,7));ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=7,markerscale=3)
    fig.suptitle(a.identity+" / observed GT points only / no detector clusters")
    a.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(a.output,dpi=150);plt.close(fig)


if __name__=="__main__":
    main()
