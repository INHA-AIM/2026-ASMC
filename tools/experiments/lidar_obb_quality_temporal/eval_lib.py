#!/usr/bin/env python3
"""Evaluator for experiment 02 (observation-quality OBB size/yaw stabilisation). Contract: EVAL_CONTRACT.json.

  eval_lib.py scene   --contract C --inputs INPUT_DIR --pred PRED.jsonl --scene S --out OUT.pkl
                      [--points PTS.bin] [--counterfactual]
  eval_lib.py metrics --contract C --samples DIR --manifest INPUT_MANIFEST.json --out METRICS.json

All geometry in base_link (= GT ego/body frame) unless stated. GT is used only here, never at runtime.
No dataset file is ever written. Run with PYTHONDONTWRITEBYTECODE=1 python3 -B.
"""
import sys

sys.dont_write_bytecode = True

import argparse, gzip, json, math, os, struct  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from scipy.optimize import linear_sum_assignment  # noqa: E402
from scipy.sparse import coo_matrix  # noqa: E402
from scipy.sparse.csgraph import connected_components  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402

INF = 1e6
LOCAL_MODES = ("local", "local_and_map")  # output_gate::ModeName of kLocal and kBoth
HALF_PI = 0.5 * math.pi
SENSOR_BEV = (0.8, 0.0)


# ============================================================================ geometry
def box_corners(cx, cy, length, width, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = 0.5 * length, 0.5 * width
    return [(cx + dx * c - dy * s, cy + dx * s + dy * c) for dx, dy in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]


def _clip(subject, a, b):
    out = []

    def inside(p):
        return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= 0

    def inter(p, q):
        x1, y1, x2, y2 = a[0], a[1], b[0], b[1]
        x3, y3, x4, y4 = p[0], p[1], q[0], q[1]
        den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(den) < 1e-12:
            return q
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
        return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
    n = len(subject)
    for i in range(n):
        cur, prev = subject[i], subject[i - 1]
        if inside(cur):
            if not inside(prev):
                out.append(inter(prev, cur))
            out.append(cur)
        elif inside(prev):
            out.append(inter(prev, cur))
    return out


def _area(poly):
    s = 0.0
    for i in range(len(poly)):
        x1, y1 = poly[i - 1]
        x2, y2 = poly[i]
        s += x1 * y2 - x2 * y1
    return 0.5 * s


def bev_iou(a, b):
    """a, b = (cx, cy, length, width, yaw); exact rotated-rectangle IoU."""
    pa, pb = box_corners(*a), box_corners(*b)
    if _area(pa) < 0:
        pa = pa[::-1]
    if _area(pb) < 0:
        pb = pb[::-1]
    poly = pa
    for i in range(4):
        if not poly:
            break
        poly = _clip(poly, pb[i - 1], pb[i])
    inter = abs(_area(poly)) if len(poly) >= 3 else 0.0
    ua = a[2] * a[3] + b[2] * b[3] - inter
    return inter / ua if ua > 1e-9 else 0.0


def axis_err(a, b):
    """pi-periodic signed difference a-b in (-pi/2, pi/2]."""
    d = a - b
    return 0.5 * math.atan2(math.sin(2 * d), math.cos(2 * d))


def canon(length, width, yaw):
    """(long, short, axis yaw of the long side in (-pi/2, pi/2])."""
    if length >= width:
        L, S, ay = length, width, yaw
    else:
        L, S, ay = width, length, yaw + HALF_PI
    return L, S, axis_err(ay, 0.0)


def nearest_corner(cx, cy, length, width, yaw, sensor=SENSOR_BEV):
    cs = box_corners(cx, cy, length, width, yaw)
    return min(cs, key=lambda p: (p[0] - sensor[0]) ** 2 + (p[1] - sensor[1]) ** 2)


def in_mask(x, y, c):
    r = math.hypot(x, y)
    az = math.degrees(math.atan2(y, x))
    lo, hi = c["masks"]["azimuth_deg"]
    return r <= c["masks"]["max_range_m"] and lo <= az <= hi


def in_tracker_roi(x, y, c):
    r = c["masks"]["tracker_roi_base_link"]
    return r["x"][0] <= x <= r["x"][1] and r["y"][0] <= y <= r["y"][1]


def pred_class(s, c):
    m = c["matching"]["class_map_prediction"]
    return m.get(s, m["*"])


# ============================================================================ loading
def open_text(p):
    return gzip.open(p, "rt") if p.endswith(".gz") else open(p)


def load_pred(path):
    frames = {}
    with open_text(path) as f:
        for line in f:
            d = json.loads(line)
            tracks = {t["id"]: t for t in d.get("tracks", []) if t.get("miss", 1) == 0}
            frames[d["idx"]] = d, tracks
    return frames


def load_points(path):
    """harness --points-out file -> {idx: (valid, T(3x4), {track_id: float32 Nx3 in tracking input frame})}."""
    data = open(path, "rb").read()
    off, out = 0, {}
    while off < len(data):
        if data[off:off + 4] != b"PTS1":
            raise ValueError("bad points record at %d in %s" % (off, path))
        off += 4
        idx, = struct.unpack_from("<q", data, off); off += 8
        valid, n = struct.unpack_from("<ii", data, off); off += 8
        T = np.frombuffer(data, "<f8", 12, off).reshape(3, 4).copy(); off += 96
        tr = {}
        for _ in range(n):
            tid, m = struct.unpack_from("<ii", data, off); off += 8
            tr[tid] = np.frombuffer(data, "<f4", 3 * m, off).reshape(m, 3).astype(np.float64); off += 12 * m
        out[idx] = (bool(valid), T, tr)
    return out


def segment_gt(gt, frames, c):
    """Assign seg ids to kind=gt rows following contract gt_track_segmentation."""
    sc = c["gt_track_segmentation"]
    yaw_of = dict(zip(frames["idx"], frames["ego_yaw"]))
    t_of = dict(zip(frames["idx"], frames["stamp_ns"] * 1e-9))
    gt = gt.sort_values(["track_id", "idx"]).copy()
    seg = np.empty(len(gt), dtype=object)
    prev, counter = {}, {}
    for n, r in enumerate(gt.itertuples(index=False)):
        tid = r.track_id
        p = prev.get(tid)
        brk = p is None
        if p is not None:
            dt = t_of[r.idx] - t_of[p.idx]
            if dt > sc["break_on_gap_s"]:
                brk = True
            if sc["break_on_class_change"] and r.class_id != p.class_id:
                brk = True
            if max(abs(r.l - p.l), abs(r.w - p.w), abs(r.h - p.h)) > sc["break_on_dims_change_m"]:
                brk = True
            yy = yaw_of[p.idx]
            vgx = math.cos(yy) * p.vx - math.sin(yy) * p.vy
            vgy = math.sin(yy) * p.vx + math.cos(yy) * p.vy
            if math.hypot(r.gx - p.gx - vgx * dt, r.gy - p.gy - vgy * dt) > sc["break_on_position_jump_m"]:
                brk = True
        if brk:
            counter[tid] = counter.get(tid, -1) + 1
        seg[n] = "%d_%d" % (tid, counter[tid])
        prev[tid] = r
    gt["seg"] = seg
    return gt.sort_values(["idx", "track_id"])


# ============================================================================ matching (unchanged from experiment 01)
def match_frame(gts, igns, preds, c):
    gates = c["matching"]["center_gate_m"]
    gt_cls = c["matching"]["class_map_gt"]
    ng, npd = len(gts), len(preds)
    pc = [pred_class(p["cls"], c) for p in preds]
    pairs = {}
    if ng and npd:
        cost = np.full((ng, npd), INF)
        kind = np.zeros((ng, npd), dtype=np.int8)
        for i, g in enumerate(gts):
            gc = gt_cls[str(int(g["class_id"]))]
            for j, p in enumerate(preds):
                if pc[j] == gc:
                    k = 1
                elif pc[j] == "unknown":
                    k = 2
                else:
                    continue
                d = math.hypot(p["c"][0] - g["x"], p["c"][1] - g["y"])
                ok = d <= gates[gc]
                if not ok and d < 10.0:
                    ok = bev_iou((p["c"][0], p["c"][1], p["l"], p["w"], p["yaw"]),
                                 (g["x"], g["y"], g["l"], g["w"], g["yaw"])) >= 0.1
                if ok:
                    cost[i, j] = d
                    kind[i, j] = k
        used_g, used_p = set(), set()
        for stage in (1, 2):
            gi = [i for i in range(ng) if i not in used_g]
            pj = [j for j in range(npd) if j not in used_p]
            if not gi or not pj:
                continue
            sub = np.where(kind[np.ix_(gi, pj)] == stage, cost[np.ix_(gi, pj)], INF)
            r, cidx = linear_sum_assignment(sub)
            for a, b in zip(r, cidx):
                if sub[a, b] < INF:
                    pairs[gi[a]] = pj[b]
                    used_g.add(gi[a]); used_p.add(pj[b])
    used_p = set(pairs.values())
    ignored = set()
    if igns:
        cand = []
        for j, p in enumerate(preds):
            if j in used_p:
                continue
            for i, g in enumerate(igns):
                gc = gt_cls[str(int(g["class_id"]))]
                if pc[j] not in (gc, "unknown"):
                    continue
                d = math.hypot(p["c"][0] - g["x"], p["c"][1] - g["y"])
                ok = d <= gates[gc] or (d < 10.0 and bev_iou((p["c"][0], p["c"][1], p["l"], p["w"], p["yaw"]),
                                                               (g["x"], g["y"], g["l"], g["w"], g["yaw"])) >= 0.1)
                if ok:
                    cand.append((d, i, j))
        ui = set()
        for d, i, j in sorted(cand):
            if i in ui or j in ignored:
                continue
            ui.add(i); ignored.add(j)
    return pairs, ignored


# ============================================================================ observability features
def point_features(P, obb):
    """P: Nx3 points in the tracking input frame (sensor at origin); obb: harness tracks[].obb
    [cx, cy, cz, W(len along yaw), H, zmin, zmax, yaw] in the same frame."""
    n = len(P)
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    az = np.degrees(np.arctan2(y, x))
    m = float(np.median(az))
    d = (az - m + 180.0) % 360.0 - 180.0
    r = np.hypot(x, y)
    el = np.degrees(np.arctan2(z, np.maximum(r, 1e-6)))
    f = {"npts": n, "az_span_deg": float(d.max() - d.min()), "z_span_m": float(z.max() - z.min()),
         "az_bins": int(len(np.unique(np.floor(d / 0.5)))), "el_bins": int(len(np.unique(np.floor(el / 2.0)))),
         "sensor_range_m": float(np.hypot(x.mean(), y.mean()))}
    cx, cy, _, W, H, _, _, yaw = [float(v) if v is not None else float("nan") for v in obb]
    if not all(np.isfinite([cx, cy, W, H, yaw])):
        f.update({"visible_faces": -1, "fit_residual_m": np.nan, "inlier_ratio": np.nan, "aspect": np.nan,
                  "pca_ratio": np.nan, "n_facing": -1})
        return f
    c, s = math.cos(yaw), math.sin(yaw)
    u = (x - cx) * c + (y - cy) * s
    v = -(x - cx) * s + (y - cy) * c
    a, b = 0.5 * W, 0.5 * H
    dist = np.minimum(np.minimum(np.abs(u - a), np.abs(u + a)), np.minimum(np.abs(v - b), np.abs(v + b)))
    edges = [(np.abs(u - a) <= 0.15) & (np.abs(v) <= b + 0.15), (np.abs(u + a) <= 0.15) & (np.abs(v) <= b + 0.15),
             (np.abs(v - b) <= 0.15) & (np.abs(u) <= a + 0.15), (np.abs(v + b) <= 0.15) & (np.abs(u) <= a + 0.15)]
    lengths, coords = [H, H, W, W], [v, v, u, u]
    sup = []
    for e, L, cc in zip(edges, lengths, coords):
        k = int(e.sum())
        cov = float((cc[e].max() - cc[e].min()) / max(L, 0.05)) if k >= 2 else 0.0
        sup.append(k >= max(3.0, 0.15 * n) and cov >= 0.3)
    us, vs = -cx * c - cy * s, cx * s - cy * c   # sensor (origin) in the OBB frame
    facing = [us > a, us < -a, vs > b, vs < -b]
    if n >= 3:
        cv = np.cov(np.vstack([x, y]))
        ev = np.linalg.eigvalsh(cv)
        pca = float(math.sqrt(max(ev[1], 0.0) / max(ev[0], 1e-6)))
    else:
        pca = np.nan
    f.update({"visible_faces": int(sum(1 for fc, sp in zip(facing, sup) if fc and sp)), "n_facing": int(sum(facing)),
              "n_supported_edges": int(sum(sup)), "fit_residual_m": float(dist.mean()),
              "inlier_ratio": float((dist <= 0.15).mean()), "aspect": float(max(W, H) / max(min(W, H), 0.05)),
              "pca_ratio": pca})
    return f


# ============================================================================ point attribution to GT boxes
def _gt_box_arrays(rows):
    out = []
    for g in rows:
        L, S, ay = canon(g["l"], g["w"], g["yaw"])
        out.append((g["x"], g["y"], g["z"], L, S, g["h"], ay))
    return np.array(out, dtype=float).reshape(-1, 7)


def assign_points(Pb, boxes, kinds, margin=0.3):
    """Pb: Nx3 base_link points; boxes: Mx7 (cx,cy,cz,L,S,h,axis_yaw); kinds: M array (1 gt, 0 ignore).
    Returns per-point box index (-1 = none / ignore) with exclusive assignment by normalised depth, plus u,v."""
    N, M = len(Pb), len(boxes)
    if M == 0 or N == 0:
        return np.full(N, -1, dtype=int)
    dx = Pb[:, 0:1] - boxes[None, :, 0]
    dy = Pb[:, 1:2] - boxes[None, :, 1]
    c, s = np.cos(boxes[:, 6])[None, :], np.sin(boxes[:, 6])[None, :]
    u = dx * c + dy * s
    v = -dx * s + dy * c
    hl, hs, hh = 0.5 * boxes[None, :, 3], 0.5 * boxes[None, :, 4], 0.5 * boxes[None, :, 5]
    dz = Pb[:, 2:3] - boxes[None, :, 2]
    inside = (np.abs(u) <= hl + margin) & (np.abs(v) <= hs + margin) & (np.abs(dz) <= hh + margin)
    depth = np.maximum(np.abs(u) / np.maximum(hl, 0.05), np.abs(v) / np.maximum(hs, 0.05))
    depth = np.where(inside, depth, np.inf)
    best = np.argmin(depth, axis=1)
    ok = np.isfinite(depth[np.arange(N), best])
    idx = np.where(ok, best, -1)
    idx = np.where((idx >= 0) & (kinds[np.maximum(idx, 0)] == 1), idx, -1)
    return idx


def _extent_overlap(A, B):
    """A,B: Kx3 point sets (u,v,z): XY extent overlap / smaller area, z overlap / smaller height."""
    def ext(P):
        return P[:, 0].min(), P[:, 0].max(), P[:, 1].min(), P[:, 1].max(), P[:, 2].min(), P[:, 2].max()
    a, b = ext(A), ext(B)
    def side(lo, hi):
        return max(hi - lo, 0.1)
    ix = max(0.0, min(a[1], b[1]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[2], b[2]))
    area_a, area_b = side(a[0], a[1]) * side(a[2], a[3]), side(b[0], b[1]) * side(b[2], b[3])
    xy = (max(ix, 0.0) * max(iy, 0.0)) / min(area_a, area_b) if min(area_a, area_b) > 0 else 0.0
    iz = max(0.0, min(a[5], b[5]) - max(a[4], b[4]))
    zo = iz / min(side(a[4], a[5]), side(b[4], b[5]))
    return xy, zo


def adaptive_tol(r):
    return 0.55 + np.clip((r - 8.0) / 42.0, 0.0, 1.0) * 0.30


def components(P, tol=None, fixed=None, min_points=1):
    """Connected components of the symmetric-tolerance graph (production rule when tol=adaptive)."""
    n = len(P)
    if n == 0:
        return np.zeros(0, dtype=int), 0
    tree = cKDTree(P)
    rmax = fixed if fixed is not None else 0.85
    pairs = tree.query_pairs(r=rmax, output_type="ndarray")
    if len(pairs):
        d = np.linalg.norm(P[pairs[:, 0]] - P[pairs[:, 1]], axis=1)
        if fixed is None:
            t = adaptive_tol(np.hypot(P[:, 0], P[:, 1]))
            keep = d <= np.minimum(t[pairs[:, 0]], t[pairs[:, 1]])
        else:
            keep = d <= fixed
        pairs = pairs[keep]
    g = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(n, n)) if len(pairs) else coo_matrix((n, n))
    k, lab = connected_components(g, directed=False)
    sizes = np.bincount(lab, minlength=k)
    big = np.where(sizes >= min_points)[0]
    return lab, len(big)


# ============================================================================ per-scene evaluation
def eval_scene(c, inputs_dir, pred_path, scene, points_path=None, counterfactual=False):
    frames = pd.read_csv(os.path.join(inputs_dir, scene, "frames.csv"))
    gt_all = pd.read_csv(os.path.join(inputs_dir, scene, "gt.csv"))
    gt = segment_gt(gt_all[gt_all.kind == "gt"], frames, c)
    ign = gt_all[gt_all.kind == "ignore"]
    pred = load_pred(pred_path)
    pts = load_points(points_path) if points_path else None
    gt_by = {k: v.to_dict("records") for k, v in gt.groupby("idx")}
    ign_by = {k: v.to_dict("records") for k, v in ign.groupby("idx")}
    yaw_of = dict(zip(frames["idx"], frames["ego_yaw"]))
    gmap = c["matching"]["class_map_gt"]
    tp_rows, fn_rows, fp_rows, frame_rows, gtobs_rows, om_rows, cf_rows = [], [], [], [], [], [], []
    obs_count, last_obs = {}, {}
    for fr in frames.itertuples(index=False):
        k = fr.idx
        d, tracks = pred.get(k, (None, {}))
        # tracker-level observation bookkeeping (tracks[] with miss==0 of processed scans)
        prev_miss = {}
        for tid in tracks:
            prev_miss[tid] = (tid in last_obs) and last_obs[tid] != k - 1
            obs_count[tid] = obs_count.get(tid, 0) + 1
            last_obs[tid] = k
        emitted = d is not None and d.get("v2_mode") in LOCAL_MODES
        preds = sorted(d["local"], key=lambda o: o["id"]) if emitted else []
        prof = (d or {}).get("prof", {})
        frame_rows.append({"scene": scene, "idx": k, "stamp_ns": fr.stamp_ns, "present": d is not None,
                           "ready": bool(d and d["ready"]), "processed": bool(d and d["processed"]),
                           "v2_mode": d["v2_mode"] if d else "missing", "emitted": emitted, "n_local": len(preds),
                           "lidar_ms": d["timing"]["lidar_process_ms"] if d else np.nan,
                           "frame_ms": d["timing"]["frame_ms"] if d else np.nan,
                           "clusters": prof.get("clusters", np.nan), "det": prof.get("det", np.nan),
                           "plane_valid": prof.get("plane_valid", np.nan)})
        gts = sorted(gt_by.get(k, []), key=lambda g: g["track_id"])
        igs = ign_by.get(k, [])
        pairs, ignored = match_frame(gts, igs, preds, c)
        matched_p = set(pairs.values())
        cy_, sy_ = math.cos(yaw_of[k]), math.sin(yaw_of[k])
        # ---------------- point-level attribution (split / over-merge / boundary / counterfactual)
        gt_flags = {}
        pf = pts.get(k) if pts is not None else None
        if pf is not None and pf[0] and pf[2]:
            _, T, trk = pf
            R, t = T[:, :3], T[:, 3]
            allrows = gts + igs
            boxes = _gt_box_arrays(allrows)
            kinds = np.array([1] * len(gts) + [0] * len(igs))
            nin = {}         # (tid, gi) -> count
            frag_pts = {}    # (tid, gi) -> Kx3 (u,v,z) in GT frame
            ntrk = {}
            assign_of = {}
            for tid, P in trk.items():
                Pb = P @ R.T + t
                ntrk[tid] = len(Pb)
                a = assign_points(Pb, boxes, kinds)
                assign_of[tid] = a
                for gi in np.unique(a[a >= 0]):
                    sel = a == gi
                    nin[(tid, int(gi))] = int(sel.sum())
                    bx = boxes[gi]
                    cc, ss = math.cos(bx[6]), math.sin(bx[6])
                    q = Pb[sel]
                    frag_pts[(tid, int(gi))] = np.column_stack([(q[:, 0] - bx[0]) * cc + (q[:, 1] - bx[1]) * ss,
                                                                -(q[:, 0] - bx[0]) * ss + (q[:, 1] - bx[1]) * cc, q[:, 2]])
            # over-merge per track
            om_gt = {}
            for tid in trk:
                gl = [gi for (tt, gi), m in nin.items() if tt == tid and m >= 5 and m >= 0.1 * ntrk[tid]]
                if len(gl) < 2:
                    continue
                distinct = False
                for i1 in range(len(gl)):
                    for i2 in range(i1 + 1, len(gl)):
                        g1, g2 = gts[gl[i1]], gts[gl[i2]]
                        if bev_iou((g1["x"], g1["y"], g1["l"], g1["w"], g1["yaw"]),
                                   (g2["x"], g2["y"], g2["l"], g2["w"], g2["yaw"])) < 0.1:
                            distinct = True
                if distinct:
                    om_rows.append({"scene": scene, "idx": k, "pid": tid, "n_pts": ntrk[tid], "n_gt": len(gl),
                                    "gt_tids": [int(gts[g]["track_id"]) for g in gl], "gt_segs": [gts[g]["seg"] for g in gl],
                                    "gt_cls": [gmap[str(int(gts[g]["class_id"]))] for g in gl],
                                    "obs_count": obs_count.get(tid, 0),
                                    "range": float(min(math.hypot(gts[g]["x"], gts[g]["y"]) for g in gl))})
                    for g in gl:
                        om_gt.setdefault(g, tid)
            for gi, g in enumerate(gts):
                if not in_mask(g["x"], g["y"], c):
                    continue
                tot = sum(m for (tt, gg), m in nin.items() if gg == gi)
                if tot < 5:
                    continue
                frags = sorted([(m, tt) for (tt, gg), m in nin.items() if gg == gi and m >= 5 and m >= 0.5 * ntrk[tt]],
                               reverse=True)
                main = max([(m, tt) for (tt, gg), m in nin.items() if gg == gi])[1]
                split = len(frags) >= 2
                vertical = False
                xy_ov = z_ov = np.nan
                if split:
                    xy_ov, z_ov = _extent_overlap(frag_pts[(frags[0][1], gi)], frag_pts[(frags[1][1], gi)])
                    vertical = xy_ov >= 0.5 and z_ov <= 0.2
                L, S, ay = canon(g["l"], g["w"], g["yaw"])
                cs = box_corners(g["x"], g["y"], L, S, ay)
                ylim = c["masks"]["tracker_roi_base_link"]["y"]
                zlim = c["masks"]["tracker_roi_base_link"]["z"]
                boundary = any(p[1] < ylim[0] or p[1] > ylim[1] for p in cs) or (g["z"] + 0.5 * g["h"] > zlim[1])
                row = {"scene": scene, "idx": k, "seg": g["seg"], "gt_tid": int(g["track_id"]),
                       "cls": gmap[str(int(g["class_id"]))], "range": math.hypot(g["x"], g["y"]), "n_assigned": tot,
                       "n_frag": len(frags), "split": split, "vertical": bool(split and vertical),
                       "horizontal": bool(split and not vertical), "xy_overlap": xy_ov, "z_overlap": z_ov,
                       "frag_pids": [int(tt) for _, tt in frags], "main_pid": int(main),
                       "main_obs": obs_count.get(main, 0), "over_merge": gi in om_gt,
                       "om_pid": int(om_gt[gi]) if gi in om_gt else -1, "boundary": bool(boundary)}
                gt_flags[g["seg"]] = row
                gtobs_rows.append(row)
            # clustering counterfactuals (baseline diagnostics)
            if counterfactual:
                for tid, P in trk.items():
                    cen = P.mean(axis=0) @ R.T + t
                    if math.hypot(cen[0], cen[1]) > 40.0 or cen[0] < -1.0 or len(P) > 6000:
                        continue
                    lab_a, n_a = components(P, min_points=1)
                    lab_f, n_f = components(P, fixed=0.5, min_points=5)
                    a = assign_of[tid]
                    def dom(mask):
                        aa = a[mask]
                        aa = aa[aa >= 0]
                        if len(aa) < max(3, 0.5 * mask.sum()):
                            return -1
                        vals, cnt = np.unique(aa, return_counts=True)
                        return int(gts[int(vals[np.argmax(cnt)])]["track_id"]) if len(vals) else -1
                    frag_dom_a = []
                    for lab in np.unique(lab_a):
                        frag_dom_a.append(dom(lab_a == lab))
                    big_f = [lab for lab in np.unique(lab_f) if (lab_f == lab).sum() >= 5]
                    frag_dom_f = [dom(lab_f == lab) for lab in big_f]
                    vert = False
                    if n_a >= 2:
                        sizes = sorted([((lab_a == lab).sum(), lab) for lab in np.unique(lab_a)], reverse=True)
                        A, B = P[lab_a == sizes[0][1]], P[lab_a == sizes[1][1]]
                        xy, zo = _extent_overlap(A, B)
                        vert = xy >= 0.5 and zo <= 0.2
                    cf_rows.append({"scene": scene, "idx": k, "pid": tid, "n": len(P), "n_adapt": n_a, "n_fixed05": n_f,
                                    "dom_whole": dom(np.ones(len(P), bool)), "frag_dom_adapt": frag_dom_a,
                                    "frag_dom_fixed05": frag_dom_f, "merge_vertical": bool(vert),
                                    "range": float(math.hypot(cen[0], cen[1])), "obs_count": obs_count.get(tid, 0)})
        # ---------------- TP / FN / FP rows
        for i, g in enumerate(gts):
            if not in_mask(g["x"], g["y"], c):
                continue
            base = {"scene": scene, "idx": k, "t": fr.stamp_ns * 1e-9, "seg": g["seg"], "gt_tid": g["track_id"],
                    "cls": gmap[str(int(g["class_id"]))], "range": math.hypot(g["x"], g["y"]),
                    "gt_speed": math.hypot(g["vx"], g["vy"]), "in_roi": in_tracker_roi(g["x"], g["y"], c)}
            if i not in pairs:
                fn_rows.append(base)
                continue
            p = preds[pairs[i]]
            tk = tracks.get(p["id"])
            dx, dy = p["c"][0] - g["x"], p["c"][1] - g["y"]
            vb = tk["vb"] if tk else [np.nan, np.nan]
            pl, ps, pay = canon(p["l"], p["w"], p["yaw"])
            gl, gs, gay = canon(g["l"], g["w"], g["yaw"])
            pk = nearest_corner(p["c"][0], p["c"][1], p["l"], p["w"], p["yaw"])
            gk = nearest_corner(g["x"], g["y"], g["l"], g["w"], g["yaw"])
            kx, ky = pk[0] - gk[0], pk[1] - gk[1]
            row = dict(base)
            row.update({"pid": p["id"], "dx": dx, "dy": dy, "rx": cy_ * dx - sy_ * dy, "ry": sy_ * dx + cy_ * dy,
                        "cerr": math.hypot(dx, dy), "pl": pl, "ps": ps, "gl": gl, "gs": gs,
                        "dlong": pl - gl, "dshort": ps - gs, "dh": p["h"] - g["h"], "yerr": axis_err(pay, gay),
                        "kerr": math.hypot(kx, ky), "kxm": cy_ * kx - sy_ * ky, "kym": sy_ * kx + cy_ * ky,
                        "legacy_dl": p["l"] - g["l"], "legacy_dw": p["w"] - g["w"],
                        "legacy_yerr": axis_err(p["yaw"], g["yaw"]),
                        "pred_w_lt_h": bool(p["l"] < p["w"]),
                        "pvx": vb[0] if vb[0] is not None else np.nan, "pvy": vb[1] if vb[1] is not None else np.nan,
                        "gvx": g["vx"], "gvy": g["vy"], "kfc": tk["kfc"] if tk else -1, "mv": p["mv"], "age": p["age"],
                        "npts": p["npts"], "obs_count": obs_count.get(p["id"], 0),
                        "prev_miss": prev_miss.get(p["id"], np.nan) if obs_count.get(p["id"], 0) > 1 else np.nan})
            amb = np.nan
            if tk is not None and tk.get("pred") is not None and all(v is not None for v in tk["pred"]):
                q = tk["pred"]
                amb = int(sum(1 for tid2, t2 in tracks.items() if tid2 != p["id"] and t2.get("meas") is not None
                              and all(v is not None for v in t2["meas"])
                              and math.hypot(t2["meas"][0] - q[0], t2["meas"][1] - q[1]) <= 1.5) > 0)
            row["ambiguity"] = amb
            if pf is not None and pf[0] and p["id"] in pf[2] and tk is not None:
                for kk, vv in point_features(pf[2][p["id"]], tk["obb"]).items():
                    row["f_" + kk] = vv
            fl = gt_flags.get(g["seg"])
            row["split"] = bool(fl["split"]) if fl else False
            row["over_merge"] = bool(fl["over_merge"]) if fl else False
            row["boundary"] = bool(fl["boundary"]) if fl else False
            tp_rows.append(row)
        for j, p in enumerate(preds):
            if j in matched_p or j in ignored:
                continue
            if in_mask(p["c"][0], p["c"][1], c):
                fp_rows.append({"scene": scene, "idx": k, "pid": p["id"], "range": math.hypot(p["c"][0], p["c"][1])})
    tp, fn, fp, fm = pd.DataFrame(tp_rows), pd.DataFrame(fn_rows), pd.DataFrame(fp_rows), pd.DataFrame(frame_rows)
    # ---------------- temporal pairs
    prs = []
    if len(tp):
        t_of = dict(zip(frames["idx"], frames["stamp_ns"] * 1e-9))
        for seg, g in tp.sort_values(["seg", "idx"]).groupby("seg"):
            rows = g.to_dict("records")
            for a, b in zip(rows, rows[1:]):
                if b["idx"] != a["idx"] + 1 or t_of[b["idx"]] - t_of[a["idx"]] > 0.3:
                    continue
                dpl, dps = b["pl"] - a["pl"], b["ps"] - a["ps"]
                yd = abs(axis_err(b["yerr"], a["yerr"]))
                size_jump = (abs(dpl) >= 0.25 * a["pl"]) or (abs(dpl) >= 0.75) or (abs(dps) >= 0.25 * a["ps"]) or (abs(dps) >= 0.75)
                prs.append({"scene": scene, "idx": a["idx"], "seg": seg, "cls": a["cls"], "range": b["range"],
                            "obs_count": b["obs_count"], "gt_speed": a["gt_speed"], "same_pid": a["pid"] == b["pid"],
                            "cdelta": math.hypot(b["rx"] - a["rx"], b["ry"] - a["ry"]),
                            "kdelta": math.hypot(b["kxm"] - a["kxm"], b["kym"] - a["kym"]),
                            "dlong_res": abs(b["dlong"] - a["dlong"]), "dshort_res": abs(b["dshort"] - a["dshort"]),
                            "ydelta": yd, "flip": yd > math.radians(60.0), "size_jump": bool(size_jump),
                            "yaw_jump": yd > math.radians(15.0),
                            "legacy_ydelta": abs(axis_err(b["legacy_yerr"], a["legacy_yerr"])),
                            "flagged": bool(a["split"] or b["split"] or a["over_merge"] or b["over_merge"]),
                            "flag_split": bool(a["split"] or b["split"]), "flag_om": bool(a["over_merge"] or b["over_merge"]),
                            "flag_boundary": bool(a["boundary"] or b["boundary"])})
    tpairs = pd.DataFrame(prs)
    # ---------------- identity metrics per GT segment
    ids = []
    pres = pd.concat([tp.assign(tp=True) if len(tp) else pd.DataFrame(),
                      fn.assign(tp=False) if len(fn) else pd.DataFrame()], ignore_index=True)
    if len(pres):
        for seg, g in pres.sort_values(["seg", "idx"]).groupby("seg"):
            g = g.to_dict("records")
            idsw = frag = 0
            last_pid, was_tp, gap = None, False, False
            for r in g:
                if r["tp"]:
                    if last_pid is not None and r["pid"] != last_pid:
                        idsw += 1
                    if was_tp and gap:
                        frag += 1
                    last_pid, was_tp, gap = r["pid"], True, False
                elif was_tp:
                    gap = True
            ids.append({"scene": scene, "seg": seg, "cls": g[0]["cls"], "n_obs": len(g),
                        "n_tp": sum(1 for r in g if r["tp"]), "idsw": idsw, "frag": frag})
    idm = pd.DataFrame(ids)
    births = {}
    for k in sorted(pred):
        d, _ = pred[k]
        if d.get("v2_mode") in LOCAL_MODES:
            for o in d["local"]:
                births.setdefault(o["id"], [k, k])[1] = k
    life = pd.DataFrame([{"scene": scene, "pid": p, "first": a, "last": b} for p, (a, b) in births.items()])
    return {"scene": scene, "tp": tp, "fn": fn, "fp": fp, "frames": fm, "tpairs": tpairs, "ids": idm, "life": life,
            "gtobs": pd.DataFrame(gtobs_rows), "overmerge": pd.DataFrame(om_rows), "cf": pd.DataFrame(cf_rows),
            "n_input": len(frames), "has_points": pts is not None}


# ============================================================================ metrics
def pct(a, q):
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    return float(np.percentile(a, q)) if len(a) else float("nan")


def mean(a):
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    return float(a.mean()) if len(a) else float("nan")


def rmse(a):
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    return float(np.sqrt((a ** 2).mean())) if len(a) else float("nan")


def rate(b):
    b = np.asarray(b, dtype=float)
    return float(b.mean()) if len(b) else float("nan")


def cat(samples, key):
    parts = [s[key] for s in samples if key in s and len(s[key])]
    return pd.concat(parts, ignore_index=True) if parts else pd.DataFrame()


def events(df, cond):
    """Maximal runs of consecutive idx per (scene, seg) where cond is true -> (n_events, n_frames, n_repeated)."""
    if not len(df):
        return 0, 0, 0
    x = df[df[cond]].sort_values(["scene", "seg", "idx"])
    if not len(x):
        return 0, 0, 0
    ne = nrep = 0
    for _, g in x.groupby(["scene", "seg"]):
        idx = g.idx.to_numpy()
        brk = np.r_[True, np.diff(idx) != 1]
        starts = np.flatnonzero(brk)
        lens = np.diff(np.r_[starts, len(idx)])
        ne += len(lens)
        nrep += int((lens >= 2).sum())
    return ne, len(x), nrep


def compute_metrics(samples, c):
    tp, fn, fp = cat(samples, "tp"), cat(samples, "fn"), cat(samples, "fp")
    fm, tpairs, ids, life = cat(samples, "frames"), cat(samples, "tpairs"), cat(samples, "ids"), cat(samples, "life")
    gtobs = cat(samples, "gtobs")
    m = {"n_scenes": len(samples), "n_input_scans": int(sum(s["n_input"] for s in samples))}
    n_tp, n_fn, n_fp = len(tp), len(fn), len(fp)
    m.update({"tp": n_tp, "fn": n_fn, "fp": n_fp,
              "recall": n_tp / (n_tp + n_fn) if n_tp + n_fn else float("nan"),
              "precision": n_tp / (n_tp + n_fp) if n_tp + n_fp else float("nan")})
    for cl in ("vehicle", "pedestrian"):
        a = int((tp.cls == cl).sum()) if n_tp else 0
        b = int((fn.cls == cl).sum()) if n_fn else 0
        m["recall_" + cl] = a / (a + b) if a + b else float("nan")
        m["tp_" + cl], m["fn_" + cl] = a, b
    tr = tp[tp.in_roi] if n_tp else tp
    fr_ = fn[fn.in_roi] if n_fn else fn
    m["recall_roi"] = len(tr) / (len(tr) + len(fr_)) if len(tr) + len(fr_) else float("nan")
    for cl in ("vehicle", "pedestrian"):
        a = int((tr.cls == cl).sum()) if len(tr) else 0
        b = int((fr_.cls == cl).sum()) if len(fr_) else 0
        m["recall_roi_" + cl] = a / (a + b) if a + b else float("nan")
    if n_tp:
        groups = {"": np.ones(n_tp, bool), "_vehicle": (tp.cls == "vehicle").to_numpy(),
                  "_pedestrian": (tp.cls == "pedestrian").to_numpy()}
        for sfx, g in groups.items():
            t = tp[g]
            m["center_mae" + sfx], m["center_p95" + sfx] = mean(t.cerr), pct(t.cerr, 95)
            m["long_mae" + sfx], m["long_p95" + sfx] = mean(np.abs(t.dlong)), pct(np.abs(t.dlong), 95)
            m["short_mae" + sfx], m["short_p95" + sfx] = mean(np.abs(t.dshort)), pct(np.abs(t.dshort), 95)
            m["height_mae" + sfx] = mean(np.abs(t.dh))
            ye = np.degrees(np.abs(t.yerr))
            m["yaw_mae" + sfx + "_deg"], m["yaw_p95" + sfx + "_deg"] = mean(ye), pct(ye, 95)
            m["corner_mae" + sfx], m["corner_p95" + sfx] = mean(t.kerr), pct(t.kerr, 95)
            m["pred_length_lt_width_rate" + sfx] = rate(t.pred_w_lt_h)
        m["size_mae_vehicle"] = (m["long_mae_vehicle"] + m["short_mae_vehicle"]) / 2.0
        m["legacy_length_mae"], m["legacy_width_mae"] = mean(np.abs(tp.legacy_dl)), mean(np.abs(tp.legacy_dw))
        m["legacy_yaw_mae_vehicle_deg"] = mean(np.degrees(np.abs(tp.legacy_yerr[groups["_vehicle"]])))
        v = tp[(tp.kfc >= 2) & np.isfinite(tp.pvx) & np.isfinite(tp.pvy)]
        en = np.hypot(v.pvx - v.gvx, v.pvy - v.gvy)
        m.update({"velocity_n": len(v), "velocity_error_mae": mean(en), "velocity_error_p95": pct(en, 95)})
        gs_ = np.hypot(v.gvx, v.gvy)
        st = v[gs_ < 0.2]
        m["stationary_n"] = len(st)
        m["stationary_false_moving_rate"] = float((np.hypot(st.pvx, st.pvy) > 0.5).mean()) if len(st) else float("nan")
    if len(tpairs):
        for sfx, g in (("", np.ones(len(tpairs), bool)), ("_vehicle", (tpairs.cls == "vehicle").to_numpy()),
                       ("_pedestrian", (tpairs.cls == "pedestrian").to_numpy())):
            t = tpairs[g]
            yd = np.degrees(t.ydelta)
            m["temporal_pairs" + sfx] = int(len(t))
            m["long_temporal_p50" + sfx], m["long_temporal_p95" + sfx] = pct(t.dlong_res, 50), pct(t.dlong_res, 95)
            m["short_temporal_p50" + sfx], m["short_temporal_p95" + sfx] = pct(t.dshort_res, 50), pct(t.dshort_res, 95)
            m["yaw_temporal_p50" + sfx + "_deg"], m["yaw_temporal_p95" + sfx + "_deg"] = pct(yd, 50), pct(yd, 95)
            m["center_temporal_p50" + sfx], m["center_temporal_p95" + sfx] = pct(t.cdelta, 50), pct(t.cdelta, 95)
            m["corner_temporal_p95" + sfx] = pct(t.kdelta, 95)
            m["axis_flip_rate" + sfx] = rate(t.flip)
            m["size_jump_rate" + sfx] = rate(t.size_jump)
            m["yaw_jump_rate" + sfx] = rate(t.yaw_jump)
            m["size_yaw_jump_rate" + sfx] = rate(t.size_jump | t.yaw_jump)
        veh = (tpairs.cls == "vehicle").to_numpy()
        m["legacy_extreme_jump_rate"] = rate((tpairs.cdelta > 1.0) | (veh & (np.degrees(tpairs.legacy_ydelta) > 15.0)))
        jv = tpairs[veh]
        if len(jv):
            jj = jv.size_jump | jv.yaw_jump
            m["jump_pairs_vehicle"] = int(jj.sum())
            m["jump_pairs_vehicle_flagged_share"] = float((jj & jv.flagged).sum() / max(1, jj.sum()))
            m["jump_pairs_vehicle_boundary_share"] = float((jj & jv.flag_boundary).sum() / max(1, jj.sum()))
    if len(ids):
        m.update({"segments": len(ids), "segments_with_tp": int((ids.n_tp > 0).sum()), "idsw": int(ids.idsw.sum()),
                  "fragmentations": int(ids.frag.sum()), "idsw_rate": float(ids.idsw.sum() / max(1, n_tp)),
                  "fragmentation_rate": float(ids.frag.sum() / max(1, (ids.n_tp > 0).sum()))})
    if len(life):
        m["track_ids_in_local"] = len(life)
    if len(fm):
        m.update({"scans_emitted_local": int(fm.emitted.sum()), "scans_dropped": int((~fm.emitted).sum()),
                  "scans_not_ready": int((~fm.ready).sum()), "scans_missing_line": int((~fm.present).sum()),
                  "lidar_ms_p50": pct(fm.lidar_ms, 50), "lidar_ms_p95": pct(fm.lidar_ms, 95),
                  "lidar_ms_max": float(np.nanmax(fm.lidar_ms)) if fm.lidar_ms.notna().any() else float("nan"),
                  "frame_ms_p95": pct(fm.frame_ms, 95)})
        if "clusters" in fm:
            mm = fm.clusters - fm.det
            m["duplicate_merges_total"] = int(np.nansum(mm))
            m["scans_ground_plane_invalid"] = int((fm.plane_valid == 0).sum())
    if len(gtobs):
        for sfx, g in (("", np.ones(len(gtobs), bool)), ("_vehicle", (gtobs.cls == "vehicle").to_numpy()),
                       ("_pedestrian", (gtobs.cls == "pedestrian").to_numpy())):
            t = gtobs[g]
            m["eligible_gt_obs" + sfx] = int(len(t))
            m["split_rate" + sfx] = rate(t.split)
            m["vertical_split_rate" + sfx] = rate(t.vertical)
            m["horizontal_split_rate" + sfx] = rate(t.horizontal)
            m["over_merge_rate" + sfx] = rate(t.over_merge)
            m["boundary_rate" + sfx] = rate(t.boundary)
            for name in ("split", "vertical", "horizontal", "over_merge", "boundary"):
                ne, nf, nr = events(t, name)
                m[name + "_events" + sfx], m[name + "_frames" + sfx], m[name + "_repeated_events" + sfx] = ne, nf, nr
    return m


def _filtered(samples, f_tp, f_fn, f_pairs, f_gtobs, keep_ids=False):
    res = []
    for s in samples:
        d = dict(s)
        d["tp"] = s["tp"][f_tp(s["tp"])] if len(s["tp"]) else s["tp"]
        d["fn"] = s["fn"][f_fn(s["fn"])] if (f_fn is not None and len(s["fn"])) else (s["fn"] if f_fn is not None else s["fn"].iloc[0:0])
        d["tpairs"] = s["tpairs"][f_pairs(s["tpairs"])] if len(s["tpairs"]) else s["tpairs"]
        d["gtobs"] = s["gtobs"][f_gtobs(s["gtobs"])] if (f_gtobs is not None and len(s["gtobs"])) else s["gtobs"].iloc[0:0]
        d["fp"] = s["fp"].iloc[0:0] if len(s["fp"]) else s["fp"]
        d["ids"] = s["ids"].iloc[0:0] if len(s["ids"]) else s["ids"]
        d["frames"] = s["frames"].iloc[0:0]
        d["life"] = s["life"].iloc[0:0] if len(s["life"]) else s["life"]
        res.append(d)
    return res


def breakdowns(samples, c):
    out = {"by_class": {}, "by_range": {}, "by_age": {}, "by_speed": {}}
    for cl in c["breakdowns"]["class"]:
        f = lambda df, cl=cl: df.cls == cl
        out["by_class"][cl] = compute_metrics(_filtered(samples, f, f, f, f), c)
    for lo, hi in c["breakdowns"]["range_m"]:
        f = lambda df, lo=lo, hi=hi: (df["range"] >= lo) & (df["range"] < hi + (1e-9 if hi == 35 else 0))
        out["by_range"]["%d-%d" % (lo, hi)] = compute_metrics(_filtered(samples, f, f, f, f), c)
    for name, (lo, hi) in c["breakdowns"]["track_age_obs"].items():
        f = lambda df, lo=lo, hi=hi: (df["obs_count"] >= lo) & (df["obs_count"] <= hi)
        g = lambda df, lo=lo, hi=hi: (df["main_obs"] >= lo) & (df["main_obs"] <= hi)
        out["by_age"][name] = compute_metrics(_filtered(samples, f, None, f, g), c)   # recall undefined by age
    for name, (lo, hi) in c["breakdowns"]["gt_speed_mps"].items():
        f = lambda df, lo=lo, hi=hi: (df["gt_speed"] >= lo) & (df["gt_speed"] < hi)
        out["by_speed"][name] = compute_metrics(_filtered(samples, f, f, f, None), c)
    # pooled buckets used by the decision rule (bucket_no_large_regression)
    f = lambda df: (df["obs_count"] >= 1) & (df["obs_count"] <= 5)
    g = lambda df: (df["main_obs"] >= 1) & (df["main_obs"] <= 5)
    out["pooled_obs_1_5"] = compute_metrics(_filtered(samples, f, None, f, g), c)
    f = lambda df: (df["range"] >= 15) & (df["range"] <= 35 + 1e-9)
    out["pooled_range_15_35"] = compute_metrics(_filtered(samples, f, f, f, f), c)
    return out


def load_samples(d, scenes):
    return [pd.read_pickle(os.path.join(d, s + ".pkl")) for s in scenes]


def scene_sets(manifest):
    sc = manifest["scenes"]
    by = lambda r: sorted([s for s, v in sc.items() if v["role"] in r], key=lambda s: int(s[4:]))
    return {"primary": by(("development", "holdout")), "development": by(("development",)),
            "holdout": by(("holdout",)), "highspeed": by(("highspeed",)), "all": by(("development", "holdout", "highspeed"))}


KEY = {  # metric -> direction (+1 higher is better, -1 lower is better)
    "recall": +1, "recall_vehicle": +1, "recall_pedestrian": +1, "recall_roi": +1, "recall_roi_vehicle": +1,
    "recall_roi_pedestrian": +1, "precision": +1,
}
for _k in ("center_mae", "center_mae_vehicle", "long_mae_vehicle", "short_mae_vehicle", "yaw_mae_vehicle_deg",
           "size_mae_vehicle", "long_temporal_p95_vehicle", "short_temporal_p95_vehicle", "yaw_temporal_p95_vehicle_deg",
           "size_yaw_jump_rate_vehicle", "idsw_rate", "fragmentation_rate", "split_rate", "over_merge_rate",
           "split_rate_vehicle", "over_merge_rate_vehicle", "long_temporal_p95", "short_temporal_p95",
           "yaw_temporal_p95_deg", "stationary_false_moving_rate", "corner_mae_vehicle", "center_temporal_p95",
           "axis_flip_rate_vehicle", "size_jump_rate_vehicle", "yaw_jump_rate_vehicle", "height_mae_vehicle"):
    KEY[_k] = -1
BOOT_KEYS = ["long_temporal_p95_vehicle", "short_temporal_p95_vehicle", "yaw_temporal_p95_vehicle_deg",
             "size_yaw_jump_rate_vehicle", "recall", "center_mae_vehicle", "long_mae_vehicle", "short_mae_vehicle",
             "yaw_mae_vehicle_deg"]


def scene_arrays(sm):
    tp, tpairs = sm["tp"], sm["tpairs"]
    a = {"ntp": len(tp), "nfn": len(sm["fn"])}
    if len(tp):
        v = tp[tp.cls == "vehicle"]
        a["cerr_v"], a["long_v"], a["short_v"] = v.cerr.to_numpy(), np.abs(v.dlong).to_numpy(), np.abs(v.dshort).to_numpy()
        a["yaw_v"] = np.degrees(np.abs(v.yerr)).to_numpy()
    else:
        for k in ("cerr_v", "long_v", "short_v", "yaw_v"):
            a[k] = np.array([])
    if len(tpairs):
        v = tpairs[tpairs.cls == "vehicle"]
        a["tl_v"], a["ts_v"], a["ty_v"] = v.dlong_res.to_numpy(), v.dshort_res.to_numpy(), np.degrees(v.ydelta.to_numpy())
        a["jump_v"] = (v.size_jump | v.yaw_jump).to_numpy().astype(float)
    else:
        for k in ("tl_v", "ts_v", "ty_v", "jump_v"):
            a[k] = np.array([])
    return a


def fast_key_metrics(arrs, idx):
    g = lambda k: np.concatenate([arrs[i][k] for i in idx]) if len(idx) else np.array([])
    s = lambda k: sum(arrs[i][k] for i in idx)
    p95 = lambda x: float(np.percentile(x, 95)) if len(x) else np.nan
    mn = lambda x: float(x.mean()) if len(x) else np.nan
    out = {"long_temporal_p95_vehicle": p95(g("tl_v")), "short_temporal_p95_vehicle": p95(g("ts_v")),
           "yaw_temporal_p95_vehicle_deg": p95(g("ty_v")), "size_yaw_jump_rate_vehicle": mn(g("jump_v")),
           "center_mae_vehicle": mn(g("cerr_v")), "long_mae_vehicle": mn(g("long_v")), "short_mae_vehicle": mn(g("short_v")),
           "yaw_mae_vehicle_deg": mn(g("yaw_v"))}
    ntp, nfn = s("ntp"), s("nfn")
    out["recall"] = ntp / (ntp + nfn) if ntp + nfn else np.nan
    return out


def bootstrap(base_s, cand_s, n, seed):
    ab = [scene_arrays(s) for s in base_s]
    ac = [scene_arrays(s) for s in cand_s]
    rng = np.random.default_rng(seed)
    N = len(ab)
    rel = {k: [] for k in BOOT_KEYS}
    for _ in range(n):
        idx = rng.integers(0, N, N)
        mb, mc = fast_key_metrics(ab, idx), fast_key_metrics(ac, idx)
        for k in BOOT_KEYS:
            b, cc = mb[k], mc[k]
            rel[k].append((cc - b) / b if b and np.isfinite(b) and np.isfinite(cc) else np.nan)
    out = {}
    for k, v in rel.items():
        v = np.asarray(v, dtype=float)
        v = v[np.isfinite(v)]
        out[k] = {"n_valid": int(len(v)), "rel_change_mean": float(v.mean()) if len(v) else None,
                  "ci95": [float(np.percentile(v, 2.5)), float(np.percentile(v, 97.5))] if len(v) else None}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["scene", "metrics"])
    ap.add_argument("--contract", required=True)
    ap.add_argument("--inputs")
    ap.add_argument("--pred")
    ap.add_argument("--points")
    ap.add_argument("--counterfactual", action="store_true")
    ap.add_argument("--scene")
    ap.add_argument("--out")
    ap.add_argument("--samples")
    ap.add_argument("--manifest")
    ap.add_argument("--sets", default="primary,development,holdout,highspeed,all")
    a = ap.parse_args()
    c = json.load(open(a.contract))
    if a.cmd == "scene":
        r = eval_scene(c, a.inputs, a.pred, a.scene, a.points, a.counterfactual)
        pd.to_pickle(r, a.out)
        print(a.scene, "tp=%d fn=%d fp=%d pairs=%d gtobs=%d cf=%d" % (len(r["tp"]), len(r["fn"]), len(r["fp"]),
                                                                       len(r["tpairs"]), len(r["gtobs"]), len(r["cf"])))
    else:
        man = json.load(open(a.manifest))
        sets = scene_sets(man)
        res = {"contract_version": c["version"], "sets": {}}
        for name in a.sets.split(","):
            scenes = [s for s in sets[name] if os.path.exists(os.path.join(a.samples, s + ".pkl"))]
            if len(scenes) != len(sets[name]):
                res.setdefault("missing", {})[name] = sorted(set(sets[name]) - set(scenes))
            ss = load_samples(a.samples, scenes)
            res["sets"][name] = {"overall": compute_metrics(ss, c), **breakdowns(ss, c)}
        sb = os.path.join(a.samples, "static_bag.pkl")
        if os.path.exists(sb):
            res["static_bag"] = compute_metrics([pd.read_pickle(sb)], c)
        json.dump(res, open(a.out, "w"), indent=1, default=float)
        keys = ("recall", "center_mae_vehicle", "long_temporal_p95_vehicle", "short_temporal_p95_vehicle",
                "yaw_temporal_p95_vehicle_deg", "size_yaw_jump_rate_vehicle")
        print(json.dumps({k: {kk: res["sets"][k]["overall"].get(kk) for kk in keys} for k in res["sets"]}, indent=1))


if __name__ == "__main__":
    main()
