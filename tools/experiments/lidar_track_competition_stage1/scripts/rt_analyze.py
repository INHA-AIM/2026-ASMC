#!/usr/bin/env python3
"""Analyse one real-rate run directory (rt_job.sh output). Read-only.
usage: rt_analyze.py RUN_DIR [RUN_DIR ...] > summary.json
Clocks: feeder pub_mono_ns and observer recv_mono_ns are both CLOCK_MONOTONIC of this host (same clock);
stamp_ns (ROS wall at publish) is used ONLY as the join key, never subtracted from a monotonic time.
End-to-end latency = observer receive (consumer side) - feeder publish of the same scan.
Trace (if present): 'scan' events = scans that entered LidarProcess (scan id = callback generation),
'scan_end' process_ms = LidarProcess wall time (steady clock inside the node).
"""
import csv, json, os, sys
import numpy as np


def pct(a, p):
    a = np.asarray(a, float)
    return float(np.percentile(a, p)) if len(a) else None


def stats(a):
    a = np.asarray(a, float)
    if not len(a):
        return {"n": 0}
    return {"n": int(len(a)), "p50": pct(a, 50), "p95": pct(a, 95), "p99": pct(a, 99), "max": float(a.max()), "mean": float(a.mean())}


def one(d):
    fe = list(csv.DictReader(open(os.path.join(d, "feeder.csv"))))
    ob = list(csv.DictReader(open(os.path.join(d, "observer.csv")))) if os.path.isfile(os.path.join(d, "observer.csv")) else []
    pub = {int(r["stamp_ns"]): int(r["pub_mono_ns"]) for r in fe}
    t_pub = np.array([int(r["pub_mono_ns"]) for r in fe]) / 1e9
    late = [float(r["late_ms"]) for r in fe]
    res = {"run": d, "published_scans": len(fe)}
    if len(t_pub) > 1:
        res["input_rate_hz"] = (len(t_pub) - 1) / (t_pub[-1] - t_pub[0])
        res["input_duration_s"] = float(t_pub[-1] - t_pub[0])
        res["input_period_ms"] = stats(np.diff(t_pub) * 1e3)
    res["feeder_lateness_ms"] = stats(late)
    local = [r for r in ob if r["topic"] == "local"]
    lat = []; matched = set()
    for r in local:
        s = int(r["stamp_ns"])
        if s in pub:
            lat.append((int(r["recv_mono_ns"]) - pub[s]) / 1e6); matched.add(s)
    res["outputs_local"] = len(local)
    res["outputs_matched_to_input"] = len(matched)
    res["outputs_map"] = sum(1 for r in ob if r["topic"] == "map")
    res["dropped_scans_total"] = len(fe) - len(matched)
    res["drop_rate"] = (len(fe) - len(matched)) / max(1, len(fe))
    if len(local) > 1:
        tr = np.array([int(r["recv_mono_ns"]) for r in local]) / 1e9
        res["output_rate_hz"] = (len(tr) - 1) / (tr[-1] - tr[0])
        res["output_interval_ms"] = stats(np.diff(tr) * 1e3)
    res["e2e_latency_ms"] = stats(lat)
    # consecutive-drop runs
    seqs = [i for i, r in enumerate(fe) if int(r["stamp_ns"]) not in matched]
    runs = []; cur = 0; prev = None
    for s in seqs:
        cur = cur + 1 if prev is not None and s == prev + 1 else 1
        prev = s
        if cur == 1: runs.append(1)
        else: runs[-1] = cur
    res["drop_runs_max_consecutive"] = max(runs) if runs else 0
    tp = os.path.join(d, "track_trace.jsonl")
    if os.path.isfile(tp):
        scans, ends, gens = [], [], []
        for line in open(tp):
            if '"ev":"scan"' in line[:40]:
                e = json.loads(line); scans.append(e); gens.append(e["scan"])
            elif '"ev":"scan_end"' in line[:40]:
                ends.append(json.loads(line))
        res["trace_scans_processed"] = len(scans)
        if gens:
            g = np.array(gens)
            res["trace_generation_max"] = int(g.max())
            res["latest_frame_wins_skipped"] = int(np.sum(np.diff(g) - 1)) if len(g) > 1 else 0
            res["callbacks_not_reaching_processing_estimate"] = int(len(fe) - g.max())
        res["lidar_process_ms"] = stats([e["process_ms"] for e in ends])
        st = {}
        for k in ("make_cloud", "roi", "voxel", "imu_correct", "ransac", "cluster", "objectinfo", "framefilter", "motion", "marker"):
            st[k] = stats([e["stage_ms"][k] for e in ends])
        res["stage_ms"] = st
        res["trace_bytes"] = os.path.getsize(tp)
    ps = os.path.join(d, "proc_samples.csv")
    if os.path.isfile(ps):
        rows = list(csv.DictReader(open(ps)))
        if len(rows) > 2:
            hz = os.sysconf("SC_CLK_TCK")
            t = np.array([int(r["t_mono_ns"]) for r in rows]) / 1e9
            cpu = np.array([int(r["utime"]) + int(r["stime"]) for r in rows]) / hz
            res["main_cpu_cores_mean"] = float((cpu[-1] - cpu[0]) / (t[-1] - t[0]))
            res["main_rss_kb_max"] = int(max(int(r["rss_kb"] or 0) for r in rows))
            res["main_threads_max"] = int(max(int(r["threads"] or 0) for r in rows))
            res["load1"] = stats([float(r["load1"]) for r in rows])
            psi = [float(r["psi_some_avg10"]) for r in rows if r["psi_some_avg10"] not in ("", None)]
            res["cpu_pressure_some_avg10"] = stats(psi)
            rd = np.array([int(r["run_delay_ns_sum"] or 0) for r in rows])
            res["main_run_delay_ms_per_s"] = float((rd[-1] - rd[0]) / 1e6 / (t[-1] - t[0]))
    nt = os.path.join(d, "node_totals.txt")
    if os.path.isfile(nt):
        res["node_totals"] = open(nt).read().strip().splitlines()
    return res


print(json.dumps([one(d) for d in sys.argv[1:]], indent=1))
