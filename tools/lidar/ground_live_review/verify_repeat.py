#!/usr/bin/env python3
"""Compare geometry/lineage exactly, ignoring non-deterministic timing values."""
import argparse
import itertools
import json


def main():
    p=argparse.ArgumentParser()
    p.add_argument("first");p.add_argument("second")
    a=p.parse_args();frames=0
    with open(a.first) as first,open(a.second) as second:
        for pair in itertools.zip_longest(first,second):
            if None in pair:
                raise AssertionError("different stream lengths")
            x,y=map(json.loads,pair)
            for key in ("input_id","stamp_ns","raw_to_voxel"):
                assert x[key]==y[key],(frames,key)
            for method,row in y["methods"].items():
                for key in ("mask","structural_removed"):
                    assert x["methods"][method][key]==row[key],(x["input_id"],method,key)
            frames+=1
    if not frames:
        raise AssertionError("no observations")
    print(json.dumps(dict(frames=frames,geometry_lineage_stamp_exact=True)))


if __name__=="__main__":
    main()
