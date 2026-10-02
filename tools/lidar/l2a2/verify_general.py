#!/usr/bin/env python3
"""Compare the source-derived general ground path with the frozen PCL reference masks."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import numpy as np

def sha(path):
    digest=hashlib.sha256()
    with Path(path).open("rb") as stream:
        for part in iter(lambda: stream.read(1<<20), b""):
            digest.update(part)
    return digest.hexdigest()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary",required=True,type=Path)
    parser.add_argument("--stage",required=True,type=Path)
    parser.add_argument("--predictions",required=True,type=Path)
    parser.add_argument("--output",required=True,type=Path)
    args=parser.parse_args()
    if args.output.exists():
        raise ValueError("output exists")
    names=(args.stage/"frames.txt").read_text().splitlines()
    results=[]
    with tempfile.TemporaryDirectory(prefix="asmc-l2a2-general-") as temporary:
        for name in names:
            if "/" in name or ".." in name:
                raise ValueError("unsafe frame name")
            mask=Path(temporary)/(name+".mask")
            source=args.stage/(name+".bin")
            frozen=args.predictions/(name+".pcl_plane_reference.mask")
            subprocess.run([str(args.binary),"--general-mask",str(source),str(mask)],
                           check=True,stdout=subprocess.DEVNULL)
            old=np.fromfile(frozen,dtype=np.uint8)
            new=np.fromfile(mask,dtype=np.uint8)
            if len(old)!=len(new):
                raise ValueError("mask point count changed: "+name)
            results.append({"frame":name,"points":len(old),
                            "different":int((old!=new).sum()),
                            "frozen_sha256":sha(frozen),
                            "current_sha256":sha(mask)})
    summary={"schema":"asmc-lidar-general-ground-parity-v1",
             "role":"source-derived PCL RANSAC path, raw staged ROI at pitch=0; not full runtime parity",
             "frames":len(results),"points":sum(r["points"] for r in results),
             "different_points":sum(r["different"] for r in results),
             "frames_with_differences":sum(r["different"]>0 for r in results),
             "stage_manifest_sha256":sha(args.stage/"manifest.json"),
             "binary_sha256":sha(args.binary),
             "results":results}
    args.output.write_text(json.dumps(summary,sort_keys=True,indent=2)+"\n")
    print(json.dumps({k:summary[k] for k in
                      ("frames","points","different_points","frames_with_differences")}))
    return 0 if summary["different_points"]==0 else 1

if __name__=="__main__":
    raise SystemExit(main())
