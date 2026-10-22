#!/usr/bin/env python3
"""Forbidden-field isolation inputs (competition mode). Read-only on the sources.
usage: python3 -B make_isolation_inputs.py COMP_INPUTS_DIR OUT_DIR scene [scene...]
For each scene writes OUT_DIR/<variant>/<scene>/frames_comp.csv (+ symlinked frames.csv/gt.csv):
  orig            : unchanged copy
  forbid_perturb  : every FORBIDDEN column replaced by large deterministic random values
                    (pos_x/y/z +-500 m, |v| 0..40 m/s, vel_y/vel_z +-20 m/s, accel +-30 m/s^2)
  forbid_removed  : FORBIDDEN columns deleted from the file
  allowed_perturb : POSITIVE CONTROL - vel_x scaled by 1.1 (an allowed input) -> output must change
"""
import csv, os, random, sys
sys.dont_write_bytecode = True
FORBIDDEN = ["ego_x", "ego_y", "ego_z", "ego_speed_norm_mps", "vel_y_mps", "vel_z_mps", "accel_x", "accel_y", "accel_z"]
src, out, scenes = sys.argv[1], sys.argv[2], sys.argv[3:]
rng = random.Random(20261022)
for sc in scenes:
    rows = list(csv.DictReader(open(os.path.join(src, sc, "frames_comp.csv"))))
    header = list(rows[0].keys())
    for variant in ("orig", "forbid_perturb", "forbid_removed", "allowed_perturb"):
        d = os.path.join(out, variant, sc)
        os.makedirs(d, exist_ok=True)
        for name in ("frames.csv", "gt.csv"):
            dst = os.path.join(d, name)
            if os.path.lexists(dst):
                os.remove(dst)
            os.symlink(os.path.realpath(os.path.join(src, sc, name)), dst)
        cols = [c for c in header if not (variant == "forbid_removed" and c in FORBIDDEN)]
        with open(os.path.join(d, "frames_comp.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            for r in rows:
                r2 = {c: r[c] for c in cols}
                if variant == "forbid_perturb":
                    r2["ego_x"] = "%.6f" % rng.uniform(-500, 500); r2["ego_y"] = "%.6f" % rng.uniform(-500, 500)
                    r2["ego_z"] = "%.6f" % rng.uniform(-50, 50); r2["ego_speed_norm_mps"] = "%.6f" % rng.uniform(0, 40)
                    r2["vel_y_mps"] = "%.6f" % rng.uniform(-20, 20); r2["vel_z_mps"] = "%.6f" % rng.uniform(-20, 20)
                    for a in ("accel_x", "accel_y", "accel_z"):
                        r2[a] = "%.6f" % rng.uniform(-30, 30)
                if variant == "allowed_perturb":
                    r2["vel_x_mps"] = "%.6f" % (1.1 * float(r["vel_x_mps"]))
                w.writerow(r2)
print("ok", scenes)
