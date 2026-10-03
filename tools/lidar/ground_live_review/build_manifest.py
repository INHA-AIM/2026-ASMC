#!/usr/bin/env python3
"""Hash the frozen local experiment; original sensor data is read-only."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def digest(path):
    h=hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda:f.read(1024*1024),b""):
            h.update(block)
    return h.hexdigest()


def file_record(path,root):
    return {"path":str(path.relative_to(root)),"bytes":path.stat().st_size,"sha256":digest(path)}


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--repo",type=Path,required=True)
    p.add_argument("--data",type=Path,required=True)
    p.add_argument("--inventory-output",type=Path,required=True)
    p.add_argument("--source-base-revision",help="Explicit host revision when worktree gitdir is not mounted")
    p.add_argument("--branch",help="Explicit host branch when worktree gitdir is not mounted")
    a=p.parse_args();repo=a.repo.resolve();data=a.data.resolve()
    if a.inventory_output.exists():
        raise FileExistsError(a.inventory_output)
    inputs=data/"lidar_assessment/20261004_gt_dual_ground"
    outputs=data/"lidar_assessment/20261004_ground_live_v1"
    inventory=[]
    for split in ("sync","raw"):
        folder=inputs/split
        inventory.extend(file_record(folder/f,data) for f in ("snapshots.jsonl","source.json"))
        with (folder/"snapshots.jsonl").open() as f:
            for text in f:
                row=json.loads(text)
                inventory.append(file_record(folder/row["cloud_path"],data))
                inventory.append(file_record(folder/"labels"/(row["input_id"]+".npz"),data))
    a.inventory_output.write_text(json.dumps(inventory,indent=2,sort_keys=True)+"\n")
    files=[]
    for directory in ("src/asmc_lidar","tools/lidar/ground_live_review","tools/lidar/height_review","tools/lidar/frame_time"):
        for f in sorted((repo/directory).rglob("*")):
            if f.is_file() and "__pycache__" not in f.parts and f.suffix not in (".pyc",):
                files.append(file_record(f,repo))
    results=[]
    for f in ("sync_ordered.jsonl","raw_ordered.jsonl","raw_ordered_repeat.jsonl",
              "sync_parity.json","raw_parity.json","raw_repeat.json",
              "sync_final_metrics/frames.jsonl","sync_final_metrics/summary.json",
              "raw_final_metrics/frames.jsonl","raw_final_metrics/summary.json",
              "live_motion_monitor.json","live_bounded_monitor.json"):
        results.append(file_record(outputs/f,data))
    manifest={
      "schema":"asmc-lidar-l2a2-v1-review-1",
      "source_base_revision":a.source_base_revision or subprocess.check_output(["git","rev-parse","HEAD"],cwd=repo,text=True).strip(),
      "branch":a.branch or subprocess.check_output(["git","branch","--show-current"],cwd=repo,text=True).strip(),
      "source_identity":"file SHA set, including new/uncommitted files; base revision alone is not executable identity",
      "data_root_variable":"ASMC_DATA","input_inventory":file_record(a.inventory_output,data),
      "source_files":files,"results":results,
      "environment":{"compiler":subprocess.check_output(["c++","--version"],text=True).splitlines()[0],
                     "build_type":"Release", "live_backend_optimization":"-O2",
                     "ros":"Noetic", "backend_execution":"sequential on one common scan",
                     "cpu":subprocess.check_output(["uname","-m"],text=True).strip()},
      "settings":{"profile":"general","voxel_m":0.1,"leveling":"pitch_only",
          "imu_policy":"already arrived latest valid stamp <= scan; max age 100ms",
          "output_frame":"rear_axle_base_link","pending_scans":1,
          "patchwork":{"height_m":1.8,"height_policy":"fixed","rnr":False,"range_m":[1,60],"guard_m":0.25},
          "linefit":{"height_m":1.78,"sectors":90,"bins":60,"fit_assignment_threads":4,
              "ordered_insertion":True,"tail_fix_applied":False,"long_threshold_m":2,
              "max_dist_to_line_m":0.05},
          "gt_regions_m":2,"neighbor_radii_m":[0.2,0.4,0.8],"minimum_points":10},
      "frame_counts":{"sync":1439,"raw":4048},
      "acceptance":"comparison instrumentation only, no operating backend adopted",
      "live_validation":"stationary only; movement preflight declined due to nearby actors",
      "limitations":["no confirmed temporal surface correspondence","no detector-cluster ranking",
         "no wall/ceiling truth from StandingOBJ or Building","current data is not held out",
         "performance includes concurrent RViz/evaluation workload"]}
    print(json.dumps(manifest,indent=2,sort_keys=True))


if __name__=="__main__":
    main()
