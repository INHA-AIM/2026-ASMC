#!/usr/bin/env python3
"""Re-execute one fixed comparison family and optional deterministic repetition."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace

from run import run
from score import run as score
from verify import verify


def execute(args):
    repo = Path(__file__).resolve().parents[3]
    root = args.data_root / "lidar_assessment"
    setting = root / "20260926/jaeho_source/src/lidar+sensorfusion/src/Main/setting.yaml"
    params = root / "20260926/offline_v2/resolved_params.yaml"
    config = repo / "config/baselines/lidar_l1b5a_clustering_20260929.yaml"
    options = dict(kind=args.kind, executable=args.build / "clustering_eval", setting=setting, params=params,
                   config=config, source_manifest=None, projection_audit=None, reference_geometry=None, decision="baseline")
    if args.kind == "capture":
        source = root / "20260929/l1b4b_capture_gt/inputs_v4"
        options.update(source_manifest=source / "manifest.json", projection_audit=root / "20260929/l1b5a_projection_v1",
                       reference_geometry=root / "20260929/l1b4b_capture_gt/run_v1/geometry.jsonl")
    elif args.kind == "synthetic":
        source = root / "20260929/l1b4_fixtures_v1"
    elif args.kind == "extra":
        source = root / "20260929/l1b5a_extra_fixtures_v1"
        options["kind"] = "synthetic"
    else:
        source = root / "20260928/l1b3g_geometry_v7"
    options["snapshots"] = source / "snapshots.jsonl"
    for suffix in ("run", "repeat") if args.repeat else ("run",):
        destination = args.output / suffix
        run(SimpleNamespace(**options, output=destination))
        if args.kind != "stationary":
            score(SimpleNamespace(kind=options["kind"],run=destination,truth=source/"truth.jsonl",
                                  output=args.output / (suffix+"_scores")))
    verify(args.output/"run", args.output/"repeat" if args.repeat else None)
    print(json.dumps({"campaign_verified":args.kind,"repeated":args.repeat,"output":str(args.output)},sort_keys=True))


if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--kind",choices=("capture","synthetic","extra","stationary"),required=True)
    p.add_argument("--data-root",type=Path,default=Path("/data"))
    p.add_argument("--build",type=Path,default=Path("/tmp/asmc-l1b5a-build"))
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--repeat",action="store_true")
    execute(p.parse_args())
