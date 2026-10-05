#!/usr/bin/env python3
"""Join two fixed-input method sweeps without altering any classification."""
import argparse
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument("first", type=Path)
    p.add_argument("second", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError(a.output)
    with a.first.open() as left, a.second.open() as right, a.output.open("w") as out:
        count = 0
        while True:
            l, r = left.readline(), right.readline()
            if not l and not r:
                break
            if not l or not r:
                raise ValueError("sweep lengths differ")
            one, two = json.loads(l), json.loads(r)
            if one["input_id"] != two["input_id"] or one["stamp_ns"] != two["stamp_ns"]:
                raise ValueError("frame identity/stamp differ")
            if set(one["methods"]) & set(two["methods"]):
                raise ValueError("duplicate method names")
            one["methods"].update(two["methods"])
            out.write(json.dumps(one, separators=(",", ":")) + "\n")
            count += 1
    print(json.dumps({"joined_frames": count, "methods": sorted(one["methods"])}))


if __name__ == "__main__":
    main()
