#!/usr/bin/env python3
"""Render representative ground-stage object losses from the frozen L2-A2 replay."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

def rows(path):
    with Path(path).open() as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--comparison",required=True,type=Path)
    parser.add_argument("--admission",required=True,type=Path)
    parser.add_argument("--score",required=True,type=Path)
    parser.add_argument("--output",required=True,type=Path)
    args=parser.parse_args()
    if args.output.exists():
        raise ValueError("output exists")
    objects=list(rows(args.score/"objects.jsonl"))
    # Only box geometry admitted by the Capture audit is used as quantitative evidence.
    cases=sorted((o for o in objects if o["roi_points"]>=10 and o["box_geometry_admitted"]),
                 key=lambda o:o["retained"]["ref_voxel"]-o["retained"]["patch_voxel"],
                 reverse=True)
    selected=[];seen=set()
    for case in cases:
        if case["capture_key"] in seen:
            continue
        if case["retained"]["ref_voxel"]<=case["retained"]["patch_voxel"]:
            continue
        selected.append(case);seen.add(case["capture_key"])
        if len(selected)==3:
            break
    if not selected:
        raise ValueError("no worsened object case")
    wanted={c["capture_key"]:c for c in selected}
    snapshots={r["input_id"]:r for r in rows(args.admission/"snapshots.jsonl")}
    truth={r["capture_key"]:r for r in rows(args.admission/"truth.jsonl")}
    observed={}
    for result in rows(args.comparison):
        if result["measurement"] and result["camera"]=="missing" and result["capture_key"] in wanted:
            observed[result["capture_key"]]=result
    if set(observed)!=set(wanted):
        raise ValueError("missing selected comparison row")
    args.output.mkdir(parents=True)
    summary=[]
    for capture_key,case in wanted.items():
        result=observed[capture_key]
        source=snapshots[result["input_id"]]
        cloud=np.fromfile(args.admission/source["cloud_path"],dtype="<f4").reshape(-1,4)
        labels=np.fromfile(args.admission/truth[capture_key]["instance_path"],dtype="<i4")
        if len(cloud)!=len(labels):
            raise ValueError("cloud/instance mismatch")
        target=labels==case["instance_id"]
        roi=(cloud[:,0]>=-30)&(cloud[:,0]<=50)&(np.abs(cloud[:,1])<=20)
        bg=np.flatnonzero(roi)[::3]
        fig,axes=plt.subplots(2,2,figsize=(12,7),constrained_layout=True)
        for col,mode in enumerate(("ref_voxel","patch_voxel")):
            mask=np.asarray(result[mode],dtype=np.uint8)
            for row,(vertical,label) in enumerate(((1,"y (m)"),(2,"z (m)"))):
                ax=axes[row,col]
                ax.scatter(cloud[bg,0],cloud[bg,vertical],s=.5,c="0.7",alpha=.4,
                           rasterized=True,label="scene")
                for value,color,title in ((2,"#177245","target kept"),
                                          (1,"#c94039","target called ground"),
                                          (0,"#d59d23","target unassigned")):
                    chosen=target&(mask==value)
                    if chosen.any():
                        ax.scatter(cloud[chosen,0],cloud[chosen,vertical],s=9,c=color,
                                   label=title,alpha=.85)
                ax.set_xlabel("sensor x (m)");ax.set_ylabel(label)
                ax.set_title(mode+(" BEV" if row==0 else " side"))
                ax.set_xlim(0,45);ax.grid(alpha=.2)
                if row==0:ax.legend(loc="upper right",fontsize=7)
        fig.suptitle(f'{capture_key} ID {case["instance_id"]}: reference {case["retained"]["ref_voxel"]}/{case["roi_points"]}, '
                     f'Patchwork++ {case["retained"]["patch_voxel"]}/{case["roi_points"]}')
        name=f'{capture_key}_id{case["instance_id"]}.png'
        fig.savefig(args.output/name,dpi=140)
        plt.close(fig)
        summary.append({"capture_key":capture_key,"instance_id":case["instance_id"],
                        "reference_retained":case["retained"]["ref_voxel"],
                        "patchwork_retained":case["retained"]["patch_voxel"],
                        "file":name})
    (args.output/"cases.json").write_text(json.dumps(summary,sort_keys=True,indent=2)+"\n")
    print(json.dumps(summary))

if __name__=="__main__":
    main()
