#!/usr/bin/env python3
"""Side-by-side comparison of arms (frozen evaluator ARM.json + stage-1 supplementary metrics). Read-only.
usage: python3 -B compare_arms.py OUT.json OUT.md REF_ARM_NAME NAME=RUN_DIR[:SUPP_JSON] ...
  RUN_DIR must contain ARM.json (preholdout_eval.py arm over eval_lib v2 samples).
  SUPP_JSON (optional) = stage1_metrics.py output containing an arm with the same NAME.
Relative change is (arm - ref) / |ref|. Directions are NOT applied here (no pass/fail): the report states
for each metric whether lower or higher is better.
"""
import json, math, sys
sys.dont_write_bytecode = True

ROWS = [  # (set, key, better)
    ("primary", "tp", "higher"), ("primary", "fn", "lower"), ("primary", "fp", "lower"),
    ("primary", "recall", "higher"), ("primary", "recall_vehicle", "higher"), ("primary", "recall_pedestrian", "higher"),
    ("primary", "precision", "higher"),
    ("primary", "center_mae_vehicle", "lower"), ("primary", "center_p95_vehicle", "lower"),
    ("primary", "long_mae_vehicle", "lower"), ("primary", "short_mae_vehicle", "lower"), ("primary", "height_mae_vehicle", "lower"),
    ("primary", "yaw_mae_vehicle_deg", "lower"), ("primary", "corner_mae_vehicle", "lower"),
    ("primary", "center_mae_pedestrian", "lower"),
    ("primary", "center_temporal_p95_vehicle", "lower"), ("primary", "long_temporal_p95_vehicle", "lower"),
    ("primary", "short_temporal_p95_vehicle", "lower"), ("primary", "yaw_temporal_p95_vehicle_deg", "lower"),
    ("primary", "size_yaw_jump_rate_vehicle", "lower"),
    ("primary", "velocity_n", "info"), ("primary", "velocity_error_mae", "lower"), ("primary", "velocity_error_p95", "lower"),
    ("primary", "stationary_n", "info"), ("primary", "stationary_false_moving_rate", "lower"),
    ("primary", "idsw", "lower"), ("primary", "idsw_rate", "lower"), ("primary", "fragmentations", "lower"),
    ("primary", "fragmentation_rate", "lower"), ("primary", "track_ids_in_local", "lower"),
    ("primary", "split_rate_vehicle", "lower"), ("primary", "over_merge_rate", "lower"), ("primary", "over_merge_events", "lower"),
    ("primary", "scans_emitted_local", "higher"), ("primary", "scans_dropped", "lower"), ("primary", "scans_not_ready", "lower"),
    ("highspeed", "recall_vehicle", "higher"), ("highspeed", "center_mae_vehicle", "lower"),
    ("highspeed", "velocity_error_mae", "lower"), ("highspeed", "velocity_error_p95", "lower"),
    ("highspeed", "stationary_false_moving_rate", "lower"), ("highspeed", "idsw", "lower"),
    ("all", "velocity_error_mae", "lower"), ("all", "velocity_error_p95", "lower"), ("all", "idsw", "lower"),
    ("static_bag", "center_mae_vehicle", "lower"), ("static_bag", "yaw_temporal_p95_vehicle_deg", "lower"),
    ("static_bag", "velocity_error_mae", "lower"), ("static_bag", "stationary_false_moving_rate", "lower"),
    ("turn_lag.primary", "xcorr_lag_scans_weighted", "lower"), ("turn_lag.primary", "turn_yaw_mae_deg", "lower"),
    ("scen93.overall", "center_mae_vehicle", "lower"), ("scen93.overall", "yaw_mae_vehicle_deg", "lower"),
    ("scen93.overall", "velocity_error_mae", "lower"),
]
SUPP = [
    ("velocity_all", "p50"), ("velocity_all", "p95"), ("velocity_all", "p99"), ("velocity_all", "max"),
    ("velocity_gt_stationary", "p95"), ("velocity_gt_stationary", "p99"),
    ("velocity_gt_moving_vehicle", "mae"), ("velocity_gt_moving_vehicle", "p95"), ("velocity_gt_moving_vehicle", "p99"),
    ("stationary", "false_moving_rate_0p5"), ("stationary", "false_moving_rate_1p0"),
    ("stationary_by_ego_accel.ego_accel_or_brake", "false_moving_rate_0p5"),
    ("stationary_by_ego_accel.ego_const_speed", "false_moving_rate_0p5"),
    ("stationary_by_ego_motion.ego_turning", "false_moving_rate_0p5"),
    ("stationary_by_ego_motion.ego_straight", "false_moving_rate_0p5"),
    ("stationary_by_ego_motion.ego_stationary", "false_moving_rate_0p5"),
    ("velocity_by_ego_motion.ego_turning", "p95"), ("velocity_by_ego_motion.ego_straight", "p95"),
    ("speed_lag_scans", "mean"), ("speed_lag_scans", "p50"), ("speed_lag_scans", "n_segments"),
]


def get(d, path):
    for p in path.split("."):
        if not isinstance(d, dict) or p not in d:
            return None
        d = d[p]
    return d


def arm_value(arm, s, k):
    base = arm.get("sets", {}).get(s) if s in ("primary", "highspeed", "all") else get(arm, s)
    return None if base is None else base.get(k)


args = sys.argv[1:]
out_json, out_md, ref = args[0], args[1], args[2]
arms = {}
for a in args[3:]:
    name, rest = a.split("=", 1)
    run, supp = (rest.split(":", 1) + [None])[:2]
    arm = json.load(open(run + "/ARM.json"))
    sm = None
    if supp:
        sm = json.load(open(supp))["arms"].get(name, {}).get("metrics")
    arms[name] = (arm, sm)
names = list(arms)
table = []
for s, k, better in ROWS:
    row = {"set": s, "metric": k, "better": better}
    for n in names:
        row[n] = arm_value(arms[n][0], s, k)
    table.append(row)
for path, k in SUPP:
    row = {"set": "supp:" + path, "metric": k, "better": "lower" if "n_segments" not in k else "info"}
    for n in names:
        sm = arms[n][1]
        v = get(sm, path) if sm else None
        row[n] = v.get(k) if isinstance(v, dict) else None
    table.append(row)
for row in table:
    r = row.get(ref)
    for n in names:
        v = row.get(n)
        row[n + "_rel"] = (v - r) / abs(r) if isinstance(v, (int, float)) and isinstance(r, (int, float)) and r not in (0, None) and not (isinstance(r, float) and math.isnan(r)) else None
json.dump({"ref": ref, "arms": names, "rows": table}, open(out_json, "w"), indent=1)


def fmt(v):
    if v is None:
        return "–"
    if isinstance(v, float):
        if math.isnan(v):
            return "nan"
        return f"{v:.4g}"
    return str(v)


with open(out_md, "w") as f:
    f.write("| 집합 | 지표 | 방향 | " + " | ".join(names) + " |\n|---|---|---|" + "---|" * len(names) + "\n")
    for row in table:
        cells = []
        for n in names:
            v = row.get(n); rel = row.get(n + "_rel")
            cells.append(fmt(v) + ("" if n == ref or rel is None else f" ({rel:+.1%})"))
        f.write(f"| {row['set']} | {row['metric']} | {row['better']} | " + " | ".join(cells) + " |\n")
print("rows", len(table), "->", out_md)
