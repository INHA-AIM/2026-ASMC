#!/usr/bin/env python3
"""Compare new runtime-core output against the previously frozen three masks."""
import argparse
import itertools
import json
from pathlib import Path
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument("candidate", type=Path)
    p.add_argument("jaeho", type=Path)
    p.add_argument("patchwork", type=Path)
    p.add_argument("linefit", type=Path)
    p.add_argument("--allow-ordered-linefit-delta", action="store_true",
                   help="Report only the documented insertion-race repair delta; Jaeho/PP stay strict")
    a = p.parse_args()
    streams = [x.open() for x in (a.candidate,a.jaeho,a.patchwork,a.linefit)]
    count = 0
    changes = []
    try:
        for rows in itertools.zip_longest(*streams):
            if None in rows:
                raise ValueError("length mismatch")
            candidate,jaeho,pp,lf = map(json.loads,rows)
            if len({x["input_id"] for x in (candidate,jaeho,pp,lf)}) != 1:
                raise ValueError("identity mismatch")
            expected = {"jaeho":jaeho["mask"],
                        "patchwork":pp["methods"]["pp_all_guard_25cm"]["mask"],
                        "linefit":lf["methods"]["lf_90x60_gap2"]["mask"]}
            for name,mask in expected.items():
                actual = candidate["methods"][name]["mask"]
                if not np.array_equal(actual,mask):
                    different = np.flatnonzero(np.asarray(actual)!=mask)
                    if name == "linefit" and a.allow_ordered_linefit_delta:
                        changes.append(dict(input_id=candidate['input_id'],points=len(different),
                                            indices=different.tolist()))
                    else:
                        raise AssertionError(f"{candidate['input_id']} {name}: {len(different)} mismatched points; first={different[:12].tolist()}")
            count += 1
    finally:
        for f in streams:
            f.close()
    print(json.dumps({"frames":count,"all_three_masks_exact":not changes,
                      "jaeho_patchwork_masks_exact":True,"linefit_changed_frames":len(changes),
                      "linefit_changed_points":sum(x['points'] for x in changes),
                      "linefit_changes":changes}))


if __name__ == "__main__":
    main()
