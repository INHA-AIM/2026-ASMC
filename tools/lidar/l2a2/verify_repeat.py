#!/usr/bin/env python3
"""Check the repeated four-way replay without treating CPU timing as deterministic."""
import argparse
import hashlib
import json
from pathlib import Path
from itertools import zip_longest

FIELDS=("input_id","capture_key","phase","profile","camera","measurement",
        "stamp_ns","raw_points","roi_points","voxel_points","voxel_fnv1a64",
        "ref_voxel_ground_valid","ref_voxel_fallback",
        "ref_raw_ground_valid","ref_raw_fallback",
        "ref_voxel","ref_raw","patch_voxel","patch_raw")
def sha(path):
    h=hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda:stream.read(1<<20),b""):
            h.update(chunk)
    return h.hexdigest()
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--first",required=True,type=Path)
    parser.add_argument("--repeat",required=True,type=Path)
    parser.add_argument("--output",required=True,type=Path)
    args=parser.parse_args()
    if args.output.exists():
        raise ValueError("output exists")
    count=0
    with args.first.open() as left,args.repeat.open() as right:
        for a,b in zip_longest(left,right):
            if a is None or b is None:
                raise ValueError("row count mismatch")
            first=json.loads(a);repeat=json.loads(b)
            if any(first[key]!=repeat[key] for key in FIELDS):
                raise ValueError("non-deterministic result: "+first["input_id"])
            count+=1
    if count!=1050:
        raise ValueError("unexpected replay count")
    report={"schema":"asmc-lidar-l2a2-repeat-v1","rows":count,
            "first_sha256":sha(args.first),"repeat_sha256":sha(args.repeat),
            "timing_compared":False,"geometry_and_masks_identical":True}
    args.output.write_text(json.dumps(report,sort_keys=True,indent=2)+"\n")
    print(json.dumps({"rows":count,"geometry_and_masks_identical":True}))
if __name__=="__main__":
    main()
