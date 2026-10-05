#!/usr/bin/env python3
"""Add the immutable Jaeho ground mask to an otherwise unchanged sweep."""
import argparse
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument("sweep", type=Path)
    p.add_argument("reference", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError(a.output)
    count = 0
    with a.sweep.open() as left, a.reference.open() as right, a.output.open("w") as out:
        while True:
            l, r = left.readline(), right.readline()
            if not l and not r:
                break
            if not l or not r:
                raise ValueError("stream lengths differ")
            one, ref = json.loads(l), json.loads(r)
            if one["input_id"] != ref["input_id"] or one["stamp_ns"] != ref["stamp_ns"]:
                raise ValueError("reference identity/stamp differs")
            one["methods"]["jaeho_reference"] = {
                "mask": ref["mask"], "segment_ms": ref["segment_ms"]}
            out.write(json.dumps(one, separators=(",", ":")) + "\n")
            count += 1
    print(json.dumps({"joined_frames": count, "reference": "jaeho_reference"}))


if __name__ == "__main__":
    main()
