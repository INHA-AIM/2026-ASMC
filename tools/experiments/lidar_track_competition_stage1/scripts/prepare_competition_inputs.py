#!/usr/bin/env python3
"""Build Competition-restricted inference inputs (read-only on every source).

usage: python3 -B prepare_competition_inputs.py SRC_INPUTS_DIR STATIC_BAG_DIR OUT_DIR

SRC_INPUTS_DIR = experiment-03 frozen inputs (<scene>/frames.csv + gt.csv, INPUT_MANIFEST.json).
For every scene OUT_DIR/<scene>/ gets:
  frames.csv, gt.csv : symlinks to the frozen files (EVALUATION ONLY: ego pose for map-aligned temporal
                       differencing, GT boxes). The harness never reads them in competition mode.
  frames_comp.csv    : the inference stream. The competition-mode harness reads ONLY the allowed
                       columns listed in ALLOWED below; FORBIDDEN columns are written on purpose so the
                       isolation test can prove they have no effect.

Allowed channel emulation (the dataset has no Competition Vehicle Status, IMU or GPS recording):
  vel_x_mps    : Competition Vehicle Status vel_x (body forward speed, m/s, signed).
                 EMULATED from the evaluation ego pose: central difference of t_lidar positions
                 projected on the ego heading (lateral motion excluded, i.e. vel_y is NOT folded in).
                 Stamp = t_lidar (age 0) -> optimistic timing versus a real 50 Hz sample.
  imu_yaw_rad  : IMU orientation yaw (rad, ENU, CCW+). EMULATED from the evaluation ego heading
                 (MORAI IMU yaw vs Ego heading measured p95 0.2-0.8 deg on real bags).
  imu_roll_rad, imu_pitch_rad, imu_wz_rps : NOT AVAILABLE in this dataset -> NaN (not 0).
  GPS          : NOT AVAILABLE in this dataset -> no column; the harness feeds no GPS sample.
"""
import csv, hashlib, json, math, os, sys

sys.dont_write_bytecode = True

ALLOWED = ["idx", "stem", "stamp_ns", "npy", "vel_x_mps", "imu_yaw_rad", "imu_roll_rad", "imu_pitch_rad", "imu_wz_rps"]
FORBIDDEN = ["ego_x", "ego_y", "ego_z", "ego_speed_norm_mps", "vel_y_mps", "vel_z_mps", "accel_x", "accel_y", "accel_z"]


def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def build_scene(src_dir, out_dir):
    rows = list(csv.DictReader(open(os.path.join(src_dir, "frames.csv"))))
    n = len(rows)
    ts = [float(r["t_lidar"]) for r in rows]
    xs = [float(r["ego_x"]) for r in rows]
    ys = [float(r["ego_y"]) for r in rows]
    yaws = [float(r["ego_yaw"]) for r in rows]
    os.makedirs(out_dir, exist_ok=True)
    for name in ("frames.csv", "gt.csv"):
        dst = os.path.join(out_dir, name)
        if os.path.lexists(dst):
            os.remove(dst)
        os.symlink(os.path.join(src_dir, name), dst)
    out = os.path.join(out_dir, "frames_comp.csv")
    stats = {"n": n, "vel_x_negative": 0, "max_abs_vel_y": 0.0}
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(ALLOWED + FORBIDDEN)
        for k, r in enumerate(rows):
            a, b = max(0, k - 1), min(n - 1, k + 1)
            if a == b:
                vx = vy = 0.0
            else:
                dt = ts[b] - ts[a]
                dx, dy = xs[b] - xs[a], ys[b] - ys[a]
                c, s = math.cos(yaws[k]), math.sin(yaws[k])
                vx = (dx * c + dy * s) / dt          # body forward
                vy = (-dx * s + dy * c) / dt         # body left (FORBIDDEN, written for the isolation test)
            stats["vel_x_negative"] += vx < 0
            stats["max_abs_vel_y"] = max(stats["max_abs_vel_y"], abs(vy))
            w.writerow([r["idx"], r["stem"], r["stamp_ns"], r["npy"], "%.6f" % vx, "%.9f" % yaws[k],
                        "nan", "nan", "nan",
                        r["ego_x"], r["ego_y"], r["ego_z"], r["ego_speed_mps"], "%.6f" % vy, "nan", "nan", "nan", "nan"])
    stats["frames_comp_sha256"] = sha(out)
    stats["src_frames_sha256"] = sha(os.path.join(src_dir, "frames.csv"))
    return stats


def main(src, static_bag, out):
    os.makedirs(out, exist_ok=True)
    man = json.load(open(os.path.join(src, "INPUT_MANIFEST.json")))
    result = {"source_inputs": src, "source_manifest_sha256": sha(os.path.join(src, "INPUT_MANIFEST.json")),
              "allowed_columns": ALLOWED, "forbidden_columns": FORBIDDEN, "doc": __doc__, "scenes": {}}
    for sc, meta in sorted(man["scenes"].items(), key=lambda kv: int(kv[0][4:])):
        st = build_scene(os.path.join(src, sc), os.path.join(out, sc))
        st["role"] = meta["role"]
        result["scenes"][sc] = st
    st = build_scene(os.path.join(static_bag, "static_bag"), os.path.join(out, "static_bag"))
    st["role"] = "static_bag"
    result["scenes"]["static_bag"] = st
    # the evaluator manifest (roles, frame counts) is the frozen one: symlink it
    dst = os.path.join(out, "INPUT_MANIFEST.json")
    if os.path.lexists(dst):
        os.remove(dst)
    os.symlink(os.path.join(src, "INPUT_MANIFEST.json"), dst)
    with open(os.path.join(out, "COMP_INPUT_MANIFEST.json"), "w") as f:
        json.dump(result, f, indent=1)
    tot = result["scenes"].values()
    print(json.dumps({"scenes": len(result["scenes"]), "frames": sum(v["n"] for v in tot),
                      "vel_x_negative": sum(v["vel_x_negative"] for v in tot),
                      "max_abs_vel_y": max(v["max_abs_vel_y"] for v in tot)}))


if __name__ == "__main__":
    main(*sys.argv[1:4])
