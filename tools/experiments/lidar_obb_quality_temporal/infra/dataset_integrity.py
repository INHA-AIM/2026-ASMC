#!/usr/bin/env python3
"""Read-only integrity snapshot of the protected datasets/bags (never writes to them).

For every protected root it records SEPARATELY
  * files: relative path, size, mtime                 (<name>_files_<tag>.tsv.gz)
  * dirs : relative path, mtime                       (<name>_dirs_<tag>.tsv.gz)
plus SHA256 of the key manifests/README/tools/bags and of a fixed-seed sample of lidar/label files,
and the list of bytecode caches (__pycache__ / *.pyc) present under each root at capture time.
usage: dataset_integrity.py OUT.json LISTING_DIR TAG
"""
import gzip, hashlib, json, os, random, sys, time

sys.dont_write_bytecode = True

SEED = 20261018
LIDAR_DATA = "/home/autonav/Lidar_Data"
DATASET = "/home/autonav/dataset"
TUNNEL_BAG = "/home/autonav/2026AIM/data/tunnel_yolo_fusion.bag"
NAS_AUDIT = "/home/autonav/2026AIM/audit_work/nas_bag_feasibility_20261018"
STATIC_BAG = (NAS_AUDIT + "/raw/raw/20260910_001230_kcity2025_gt_instance_smoke_foggy_1pm_morai_auto/"
              "bags/20260910_001230_kcity2025_gt_instance_smoke_foggy_1pm_morai_auto_0.bag")
ROOTS = (("dataset", DATASET), ("Lidar_Data", LIDAR_DATA), ("nas_bag_feasibility", NAS_AUDIT))


def sha_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 22), b""):
            h.update(b)
    return h.hexdigest()


def listing(root):
    files, dirs, caches = [], [], []
    for dp, dns, fns in os.walk(root):
        dns.sort()
        rel = os.path.relpath(dp, root)
        try:
            dirs.append("%s\t%.6f" % (rel, os.lstat(dp).st_mtime))
        except OSError as e:
            dirs.append("%s\tERR\t%s" % (rel, e))
        if os.path.basename(dp) == "__pycache__":
            caches.append(rel)
        for fn in sorted(fns):
            p = os.path.join(dp, fn)
            if fn.endswith(".pyc"):
                caches.append(os.path.relpath(p, root))
            try:
                st = os.lstat(p)
            except OSError as e:
                files.append("%s\tERR\t%s" % (os.path.relpath(p, root), e))
                continue
            files.append("%s\t%d\t%.6f" % (os.path.relpath(p, root), st.st_size, st.st_mtime))
    return files, dirs, caches


def scene_dirs(root):
    return sorted(d for d in os.listdir(root) if d.startswith("scen") and os.path.isdir(os.path.join(root, d)))


def write_gz(path, rows):
    blob = ("\n".join(rows) + "\n").encode("utf-8")
    with gzip.GzipFile(path, "wb", mtime=0) as f:
        f.write(blob)
    return hashlib.sha256(blob).hexdigest()


def main(out_json, listing_dir, tag):
    t0 = time.time()
    res = {"tag": tag, "seed": SEED, "captured_at_epoch": t0,
           "captured_at_kst": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime())}
    res["sha256"] = {
        "Lidar_Data/MANIFEST.json": sha_file(os.path.join(LIDAR_DATA, "MANIFEST.json")),
        "Lidar_Data/README.md": sha_file(os.path.join(LIDAR_DATA, "README.md")),
        "Lidar_Data/tools/replay_lidar_scene.py": sha_file(os.path.join(LIDAR_DATA, "tools/replay_lidar_scene.py")),
        "static_gt_bag": sha_file(STATIC_BAG),
        "tunnel_yolo_fusion.bag": sha_file(TUNNEL_BAG),
    }
    for extra in ("MANIFEST.json", "README.md"):
        p = os.path.join(DATASET, extra)
        if os.path.isfile(p):
            res["sha256"]["dataset/" + extra] = sha_file(p)
    counts = {}
    for name, root in (("dataset", DATASET), ("Lidar_Data/primary", LIDAR_DATA + "/primary"),
                       ("Lidar_Data/highspeed_eval", LIDAR_DATA + "/highspeed_eval")):
        sc = scene_dirs(root)
        per = {}
        for s in sc:
            d = {}
            for sub in ("lidar", "labels_3d_lidar_v1", "labels_3d", "ego_pose"):
                sp = os.path.join(root, s, sub)
                if os.path.isdir(sp):
                    d[sub] = len(os.listdir(sp))
            per[s] = d
        counts[name] = {"n_scenes": len(sc), "per_scene": per,
                        "total_lidar_files": sum(v.get("lidar", 0) for v in per.values())}
    res["counts"] = counts
    # fixed-seed sample: 100 lidar + 100 labels_3d_lidar_v1 (Lidar_Data), 50 lidar + 50 labels (dataset)
    rng = random.Random(SEED)
    samples = {}
    for name, roots, gtsub, n in (("Lidar_Data", [LIDAR_DATA + "/primary", LIDAR_DATA + "/highspeed_eval"],
                                    "labels_3d_lidar_v1", 100),
                                   ("dataset", [DATASET], "labels_3d", 50)):
        cands_l, cands_g = [], []
        for r in roots:
            for s in scene_dirs(r):
                for sub, acc in (("lidar", cands_l), (gtsub, cands_g)):
                    sp = os.path.join(r, s, sub)
                    if os.path.isdir(sp):
                        acc.extend(os.path.join(sp, f) for f in sorted(os.listdir(sp)))
        pick = rng.sample(cands_l, n) + rng.sample(cands_g, n)
        samples[name] = {os.path.relpath(p, "/home/autonav"): sha_file(p) for p in pick}
    res["sampled_file_sha256"] = samples
    res["sampled_file_count"] = sum(len(v) for v in samples.values())
    snaps = {}
    os.makedirs(listing_dir, exist_ok=True)
    for name, root in ROOTS:
        files, dirs, caches = listing(root)
        ff = os.path.join(listing_dir, "%s_files_%s.tsv.gz" % (name, tag))
        fd = os.path.join(listing_dir, "%s_dirs_%s.tsv.gz" % (name, tag))
        tot = 0
        for r in files:
            parts = r.split("\t")
            if len(parts) == 3 and parts[1] != "ERR":
                tot += int(parts[1])
        snaps[name] = {"root": root, "n_files": len(files), "n_dirs": len(dirs), "total_bytes": tot,
                       "files_listing_sha256": write_gz(ff, files), "files_listing": ff,
                       "dirs_listing_sha256": write_gz(fd, dirs), "dirs_listing": fd,
                       "bytecode_caches_present": caches}
    for name, p in (("static_gt_bag", STATIC_BAG), ("tunnel_yolo_fusion.bag", TUNNEL_BAG)):
        st = os.stat(p)
        snaps[name] = {"path": p, "size": st.st_size, "mtime": st.st_mtime}
    res["mtime_size_snapshot"] = snaps
    res["elapsed_s"] = time.time() - t0
    with open(out_json, "w") as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    print(json.dumps({"sha256": res["sha256"], "sampled": res["sampled_file_count"],
                      "snap": {k: {kk: vv for kk, vv in v.items() if kk in ("n_files", "n_dirs", "total_bytes",
                                                                         "bytecode_caches_present", "size")}
                               for k, v in snaps.items()}, "elapsed_s": res["elapsed_s"]}, indent=1))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3])
