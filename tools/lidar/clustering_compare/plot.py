#!/usr/bin/env python3
"""Post-hoc XY/XZ plots; no labels are passed to the clustering evaluator."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from tools.lidar.clustering_compare.projection import sha


def run(args):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    row = next(r for r in map(json.loads,(args.run/"geometry.jsonl").open()) if r["input_id"]==args.input_id)
    if args.output.exists():
        raise ValueError("output exists")
    modes = [m for m in ("baseline","raw_euclidean","travel_aos") if "objects" in row[m]]
    fig, axes = plt.subplots(2,len(modes),figsize=(7*len(modes),9),squeeze=False)
    for col, mode in enumerate(modes):
        file = row["voxel_path"] if mode=="baseline" else row["raw_path"]
        points = np.fromfile(args.run/"clouds"/file,dtype="<f4").reshape(-1,4)
        labels = np.asarray(row[mode]["labels"])
        ids = np.asarray(row[mode]["cluster_id"])
        for view, pair in enumerate(((0,1),(0,2))):
            ax=axes[view,col]
            ax.scatter(points[:,pair[0]],points[:,pair[1]],s=.4,c="#b9b9b9",alpha=.3)
            kept=labels==0
            ax.scatter(points[kept,pair[0]],points[kept,pair[1]],s=2,c=ids[kept],cmap="tab20",vmin=-1)
            for obj in row[mode]["objects"]:
                if obj["removed"]:
                    continue
                corners=np.asarray(obj["corners"])
                if view==0:
                    x,y=corners[[0,1,2,3,0],pair[0]],corners[[0,1,2,3,0],pair[1]]
                else:
                    x,y=corners[[0,1,5,4,0],pair[0]],corners[[0,1,5,4,0],pair[1]]
                ax.plot(x,y,color="#e56b19",linewidth=1)
            ax.set_title(mode+" "+("XY" if view==0 else "XZ")+" objects="+str(row[mode]["final_objects"]))
            ax.set_xlim(args.xmin,args.xmax)
            ax.set_ylim((-10,10) if view==0 else (-2,4))
            ax.set_aspect("equal"); ax.grid(alpha=.2)
    fig.suptitle(args.input_id+" | orange: surviving pinned OBB; colour: cluster; grey: raw input")
    fig.tight_layout(); args.output.parent.mkdir(parents=True,exist_ok=True); fig.savefig(args.output,dpi=130); plt.close(fig)
    args.output.with_suffix(".json").write_text(json.dumps({"input_id":args.input_id,"figure_sha256":sha(args.output),
        "run_manifest_sha256":sha(args.run/"manifest.json"),"plotter_sha256":sha(Path(__file__))},sort_keys=True,indent=2)+"\n")


if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run",type=Path,required=True); p.add_argument("--input-id",required=True)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--xmin",type=float,default=-10); p.add_argument("--xmax",type=float,default=30)
    run(p.parse_args())
