#!/usr/bin/env python3
import json, sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from lbc_bev import LBCRenderer
from lbc_bev.spec import MAP_SIZE, CROP_SIZE, PIXELS_PER_METER, PIXELS_AHEAD_VEHICLE, crop_birdview

def _log(msg: str) -> None:
    print(msg, flush=True)

def main():
    ok = True
    for k,v in [("MAP_SIZE",320),("CROP_SIZE",192),("PIXELS_PER_METER",5),("PIXELS_AHEAD_VEHICLE",100)]:
        if globals()[k]!=v: _log(f"FAIL {k}"); ok=False
        else: _log(f"OK {k} = {v}")
    d=np.zeros((320,320,7),np.uint8)
    if crop_birdview(d).shape!=(192,192,7): _log("FAIL crop"); ok=False
    else: _log("OK crop")
    _log("Loading HD map (road/lane bake, ~1-3 s)...")
    r=LBCRenderer()
    _log("Rendering test BEV...")
    from lbc_bev.ws_root import asmc_ws_root
    gi=asmc_ws_root() / "R_KR_PG_KATRI/global_info.json"
    wo=json.load(open(gi))["workspace_origin"] if gi.is_file() else None
    ex,ey=(float(wo[0]),float(wo[1])) if wo else (r.default_ego().x,r.default_ego().y)
    o=r.render(ex,ey,0.0)
    if o["birdview"].shape!=(320,320,7): _log("FAIL bv"); ok=False
    else:
        nz=[int(np.count_nonzero(o["birdview"][:,:,i])) for i in range(7)]
        _log(f"OK render nz {nz}")
    if o["cropped"].shape!=(192,192,7): _log("FAIL crop out"); ok=False
    else: _log("OK cropped")
    _log("ALL PASSED" if ok else "FAILED")
    return 0 if ok else 1
if __name__=="__main__": raise SystemExit(main())
