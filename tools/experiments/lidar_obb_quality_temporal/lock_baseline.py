#!/usr/bin/env python3
"""Freeze the baseline (must run after the full baseline run and before ANY candidate file is written):
SHA256 of every baseline output / points dump / sample / metric file, of the evaluator, contract, input manifest,
static-bag scene and baseline binaries; writes RESULTS/BASELINE_LOCK.json, copies the frozen result files into
WORK/baseline_frozen and makes the baseline run directory and the frozen copies read-only.
usage: lock_baseline.py RUN_DIR RESULTS_DIR EVAL_DIR INPUTS_DIR STATIC_SCENE_DIR
"""
import sys

sys.dont_write_bytecode = True

import glob, hashlib, json, os, shutil, stat, subprocess, time  # noqa: E402


def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 22), b""):
            h.update(b)
    return h.hexdigest()


def main(run, results, evald, inputs, static_dir):
    files = {}
    for pat in ("out/*.jsonl", "out/*.pts.bin", "out/*.log", "out/*.rc", "samples/*.pkl", "METRICS.json", "params.yaml",
                "binary_sha256.txt", "udp_snmp.json", "run_meta.txt", "replay_totals.txt"):
        for p in sorted(glob.glob(os.path.join(run, pat))):
            files[os.path.relpath(p, run)] = sha(p)
    evf = {os.path.basename(p): sha(p) for p in sorted(glob.glob(os.path.join(evald, "*"))) if os.path.isfile(p)}
    head = subprocess.run(["git", "-C", evald, "rev-parse", "HEAD"], stdout=subprocess.PIPE,
                          env=dict(os.environ, GIT_OPTIONAL_LOCKS="0")).stdout.decode().strip()
    dirty = subprocess.run(["git", "-C", evald, "status", "--porcelain", "--", "."], stdout=subprocess.PIPE,
                           env=dict(os.environ, GIT_OPTIONAL_LOCKS="0")).stdout.decode()
    res_files = {}
    for name in ("EVAL_CONTRACT.json", "BASELINE_METRICS.json", "BASELINE_BY_SCENE.csv", "BASELINE_CLUSTER_EVENTS.json",
                 "CLUSTER_OBSERVABILITY_BASELINE.json", "CLUSTER_SPLIT_MERGE_ANALYSIS.json", "ROI_BOUNDARY_ANALYSIS.json",
                 "CALLGRAPH_AND_RISK_KO.md", "TRACK_BOX_STATE_AUDIT.json"):
        p = os.path.join(results, name)
        if os.path.exists(p):
            res_files[name] = sha(p)
    lock = {"locked_at_kst": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "run_dir": run, "evaluator_commit": head,
            "evaluator_dir_dirty": dirty, "evaluator_files_sha256": evf,
            "binary": open(os.path.join(run, "binary_sha256.txt")).read().split("\n"),
            "run_meta": open(os.path.join(run, "run_meta.txt")).read().split("\n"),
            "input_manifest_sha256": sha(os.path.join(inputs, "INPUT_MANIFEST.json")),
            "static_bag_scene_sha256": {f: sha(os.path.join(static_dir, f)) for f in sorted(os.listdir(static_dir))
                                        if os.path.isfile(os.path.join(static_dir, f))},
            "results_files_sha256": res_files, "n_run_files": len(files), "run_files_sha256": files}
    json.dump(lock, open(os.path.join(results, "BASELINE_LOCK.json"), "w"), indent=1)
    frozen = os.path.join(os.path.dirname(results), "work", "baseline_frozen")
    os.makedirs(frozen, exist_ok=True)
    for name in ("METRICS.json", "params.yaml", "binary_sha256.txt"):
        shutil.copy(os.path.join(run, name), os.path.join(frozen, name))
    for name in list(res_files) + ["BASELINE_LOCK.json"]:
        shutil.copy(os.path.join(results, name), os.path.join(frozen, name))
    ro = stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH
    for root, _, fs in os.walk(run):
        for f in fs:
            os.chmod(os.path.join(root, f), ro)
    for f in os.listdir(frozen):
        os.chmod(os.path.join(frozen, f), ro)
    print("locked %d run files, %d evaluator files, %d result files; BASELINE_LOCK.json written; evaluator_commit=%s dirty=%r"
          % (len(files), len(evf), len(res_files), head, dirty))


if __name__ == "__main__":
    main(*sys.argv[1:6])
