#!/usr/bin/env python3
"""Freeze the experiment input streams (read-only on the dataset).

Run as:  PYTHONDONTWRITEBYTECODE=1 python3 -B prepare_inputs.py OUT_DIR
This file itself also sets sys.dont_write_bytecode before importing anything from the dataset, so the
dataset's own loader (tools/replay_lidar_scene.py, imported by path and never modified) cannot leave a
__pycache__ next to it even if the interpreter flags are forgotten (experiment 01 incident).

For every scene of /home/autonav/Lidar_Data/{primary,highspeed_eval}:
  * frame-level lidar_primary == 1 and has_lidar_npy  (GT-empty frames are KEPT)
  * box-level correction_valid == 1 and vel_source == 'raw'  (= evaluated GT, kind=gt)
  * box-level correction_valid == 1 and vel_source != 'raw' -> kind=ignore (FP suppression only)
  * t_lidar strictly increasing; dt = t_lidar difference (never a fixed 66.7 ms)
Outputs (under OUT/<scene>/): frames.csv (harness input) and gt.csv (evaluation GT), plus
OUT/INPUT_MANIFEST.json with per-scene counts and SHA256 of every generated file.
"""
import sys

sys.dont_write_bytecode = True  # must precede the dataset-tool import below

import csv, hashlib, importlib.util, json, math, os  # noqa: E402

LIDAR_DATA = "/home/autonav/Lidar_Data"
TOOL = LIDAR_DATA + "/tools/replay_lidar_scene.py"
HOLDOUT = ["scen93", "scen98", "scen103", "scen108", "scen113"]


def sha_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def load_tool():
    assert sys.dont_write_bytecode, "bytecode writing must be disabled before importing the dataset tool"
    spec = importlib.util.spec_from_file_location("replay_lidar_scene_ro", TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def stamp_ns_of(t):
    ms = int(round(t * 1000.0))
    if abs(t * 1000.0 - ms) > 1e-3:
        raise SystemExit("t_lidar not on a millisecond grid: %r" % t)
    return ms * 1000000


def main(out_root):
    tool = load_tool()
    os.makedirs(out_root, exist_ok=True)
    manifest = {"tool": TOOL, "tool_sha256": sha_file(TOOL), "scenes": {}, "holdout": HOLDOUT, "rules": __doc__}
    for split in ("primary", "highspeed_eval"):
        root = os.path.join(LIDAR_DATA, split)
        scenes = sorted([d for d in os.listdir(root) if d.startswith("scen")], key=lambda s: int(s[4:]))
        for s in scenes:
            sd = os.path.join(root, s)
            frames = tool.load_scene(sd, require_primary=True, require_valid=True, only_raw_velocity=True)
            frames_all = tool.load_scene(sd, require_primary=True, require_valid=True, only_raw_velocity=False)
            assert [f["stem"] for f in frames] == [f["stem"] for f in frames_all]
            n_box_nonprimary = 0
            for fr in frames:
                p = os.path.join(sd, "labels_3d_lidar_v1", fr["stem"] + ".csv")
                if os.path.isfile(p):
                    with open(p, newline="") as f:
                        for r in csv.DictReader(f):
                            if r.get("correction_valid") == "1" and r.get("lidar_primary") != "1":
                                n_box_nonprimary += 1
            od = os.path.join(out_root, s)
            os.makedirs(od, exist_ok=True)
            n = len(frames)
            xs = [fr["ego"]["x"] for fr in frames]
            ys = [fr["ego"]["y"] for fr in frames]
            ts = [fr["t_lidar"] for fr in frames]
            yaws = [fr["ego"]["yaw"] for fr in frames]
            speeds = []
            for k in range(n):
                a, b = max(0, k - 1), min(n - 1, k + 1)
                if a == b:
                    speeds.append(0.0)
                    continue
                dx, dy, dt = xs[b] - xs[a], ys[b] - ys[a], ts[b] - ts[a]
                v = math.hypot(dx, dy) / dt
                if dx * math.cos(yaws[k]) + dy * math.sin(yaws[k]) < 0:
                    v = -v
                speeds.append(v)
            fpath = os.path.join(od, "frames.csv")
            with open(fpath, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["idx", "stem", "t_lidar", "stamp_ns", "dt_prev_s", "npy", "ego_x", "ego_y",
                            "ego_z", "ego_yaw", "ego_speed_mps", "n_gt", "n_ignore"])
                for k, fr in enumerate(frames):
                    dtp = "" if k == 0 else "%.6f" % (ts[k] - ts[k - 1])
                    w.writerow([k, fr["stem"], "%.6f" % ts[k], stamp_ns_of(ts[k]), dtp, fr["npy"],
                                "%.6f" % xs[k], "%.6f" % ys[k], "%.6f" % fr["ego"]["z"], "%.9f" % yaws[k],
                                "%.6f" % speeds[k], len(fr["boxes"]), len(frames_all[k]["boxes"]) - len(fr["boxes"])])
            gpath = os.path.join(od, "gt.csv")
            n_gt = n_ign = 0
            with open(gpath, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["idx", "stamp_ns", "kind", "track_id", "class_id", "x", "y", "z", "w", "l", "h",
                            "yaw", "vx", "vy", "gx", "gy", "yaw_global"])
                for k, (fr, fa) in enumerate(zip(frames, frames_all)):
                    keep = {(b["track_id"], b["x"], b["y"]) for b in fr["boxes"]}
                    for b in fa["boxes"]:
                        kind = "gt" if (b["track_id"], b["x"], b["y"]) in keep else "ignore"
                        n_gt += kind == "gt"
                        n_ign += kind == "ignore"
                        w.writerow([k, stamp_ns_of(ts[k]), kind, b["track_id"], b["class_id"],
                                    "%.4f" % b["x"], "%.4f" % b["y"], "%.4f" % b["z"], "%.4f" % b["w"],
                                    "%.4f" % b["l"], "%.4f" % b["h"], "%.6f" % b["yaw"], "%.4f" % b["vx"],
                                    "%.4f" % b["vy"], "%.4f" % b["gx"], "%.4f" % b["gy"], "%.6f" % b["yaw_global"]])
            dts = [ts[k] - ts[k - 1] for k in range(1, n)]
            role = "holdout" if s in HOLDOUT else ("highspeed" if split == "highspeed_eval" else "development")
            manifest["scenes"][s] = {
                "split": split, "role": role, "n_frames": n, "n_gt_boxes": n_gt, "n_ignore_boxes": n_ign,
                "n_gt_empty_frames": sum(1 for fr in frames if not fr["boxes"]),
                "n_box_rows_valid_but_not_lidar_primary": n_box_nonprimary,
                "dt_min_s": min(dts) if dts else None, "dt_max_s": max(dts) if dts else None,
                "n_dt_nonpositive": sum(1 for d in dts if d <= 0),
                "frames_csv": fpath, "frames_csv_sha256": sha_file(fpath),
                "gt_csv": gpath, "gt_csv_sha256": sha_file(gpath),
            }
            print("%-8s %-14s %-11s frames=%5d gt=%6d ignore=%4d empty=%4d nonprimary_rows=%d" % (
                s, split, role, n, n_gt, n_ign, manifest["scenes"][s]["n_gt_empty_frames"], n_box_nonprimary))
    tot = manifest["scenes"].values()
    manifest["totals"] = {
        "n_scenes": len(manifest["scenes"]), "n_frames": sum(v["n_frames"] for v in tot),
        "n_gt_boxes": sum(v["n_gt_boxes"] for v in tot), "n_ignore_boxes": sum(v["n_ignore_boxes"] for v in tot),
        "by_role": {r: {"scenes": sum(1 for v in tot if v["role"] == r),
                        "frames": sum(v["n_frames"] for v in tot if v["role"] == r)}
                    for r in ("development", "holdout", "highspeed")},
    }
    with open(os.path.join(out_root, "INPUT_MANIFEST.json"), "w") as f:
        json.dump(manifest, f, indent=1)
    print(json.dumps(manifest["totals"], indent=1))


if __name__ == "__main__":
    main(sys.argv[1])
