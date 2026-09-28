#!/usr/bin/env python3
"""Preserve legacy fixtures and add analytic first-return, sparse wall scenes."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "camera_sync"))
from run_synthetic_geometry import cloud_fixture, projection
from run_evaluation import digest_file

KINDS = {"car": (4., 1.8, 1.5), "person": (.5, .5, 1.7),
         "low_unknown": (.8, .6, .3)}


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def first_return_scene(kind="car", side=1, gap=.1, distance=15., ground=-1.45,
                       shape="straight", occluder=False):
    """16 elevation rays + 0.25 degree azimuth grid, NOT a VLP16 simulation.

    Intersect analytic floor/ceiling/wall rectangles and one axis-aligned box;
    only the nearest positive intersection is observable. No semantic/intensity
    input is available to the candidate beyond the resulting float4 point cloud.
    """
    elevations = np.deg2rad(np.arange(-15., 16., 2.))
    azimuths = np.deg2rad(np.arange(-180., 180., .25))
    az, el = np.meshgrid(azimuths, elevations)
    rays = np.column_stack((np.cos(el.ravel()) * np.cos(az.ravel()),
                            np.cos(el.ravel()) * np.sin(az.ravel()), np.sin(el.ravel())))
    distances = np.full(len(rays), np.inf)
    labels = np.full(len(rays), -1, dtype=np.int16)

    def offer(t, valid, label):
        select = valid & np.isfinite(t) & (t > .5) & (t < distances)
        distances[select] = t[select]
        labels[select] = label

    for z, label in ((ground, 0), (3., 2)):
        t = np.divide(z, rays[:, 2], out=np.full(len(rays), np.inf), where=abs(rays[:, 2]) > 1e-9)
        xyz = rays * np.where(np.isfinite(t), t, 0)[:, None]
        offer(t, (xyz[:, 0] >= -30) & (xyz[:, 0] <= 50) & (abs(xyz[:, 1]) <= 10), label)
    wall_path = ((-30., 3.6), (50., 3.6))
    if shape == "curve":
        wall_path = ((-30., 3.6), (0., 3.6), (20., 4.1), (50., 5.1))
    elif shape == "step":
        wall_path = ((-30., 3.6), (10., 3.6), (10., 3.95), (50., 3.95))
    for wall_side in (-1, 1):
        for p, q in zip(wall_path, wall_path[1:]):
            a, b = np.array([p[0], wall_side * p[1]]), np.array([q[0], wall_side * q[1]])
            tangent = b - a
            normal = np.array([-tangent[1], tangent[0]])
            denominator = rays[:, :2] @ normal
            t = np.divide(a @ normal, denominator, out=np.full(len(rays), np.inf),
                          where=abs(denominator) > 1e-9)
            xyz = rays * np.where(np.isfinite(t), t, 0)[:, None]
            along = ((xyz[:, :2] - a) @ tangent) / (tangent @ tangent)
            offer(t, (along >= 0) & (along <= 1) & (xyz[:, 2] >= ground) & (xyz[:, 2] <= 3), 1)
    box = None
    if kind:
        length, width, height = KINDS[kind]
        center_y = side * (3.6 - width / 2 - gap)
        lo = np.array([distance - length / 2, center_y - width / 2, ground + .21])
        hi = np.array([distance + length / 2, center_y + width / 2, ground + height])
        lows, highs = [], []
        for axis in range(3):
            direction = rays[:, axis]
            t1 = np.divide(lo[axis], direction, out=np.full(len(rays), np.inf), where=abs(direction) > 1e-9)
            t2 = np.divide(hi[axis], direction, out=np.full(len(rays), -np.inf), where=abs(direction) > 1e-9)
            parallel = abs(direction) <= 1e-9
            inside = lo[axis] <= 0 <= hi[axis]
            low, high = np.minimum(t1, t2), np.maximum(t1, t2)
            low[parallel] = -np.inf if inside else np.inf
            high[parallel] = np.inf if inside else -np.inf
            lows.append(low); highs.append(high)
        entry = np.max(lows, axis=0)
        leave = np.min(highs, axis=0)
        offer(np.where(entry > 0, entry, leave), leave >= np.maximum(entry, 0), 3)
        box = {"lo": lo.tolist(), "hi": hi.tolist()}
        if occluder:
            t = np.divide(lo[0] - 2., rays[:, 0], out=np.full(len(rays), np.inf), where=abs(rays[:, 0]) > 1e-9)
            xyz = rays * np.where(np.isfinite(t), t, 0)[:, None]
            offer(t, (xyz[:, 1] >= lo[1]) & (xyz[:, 1] <= center_y) &
                  (xyz[:, 2] >= lo[2]) & (xyz[:, 2] <= hi[2]), 4)
    valid = np.isfinite(distances) & (distances < 80)
    xyz = (rays[valid] * distances[valid, None]).astype("<f4")
    cloud = np.column_stack((xyz, np.full(len(xyz), 10, dtype="<f4"))).astype("<f4")
    return cloud, labels[valid].astype(np.uint8), box


def camera_decision(state, stamp, points, truth, setting, kind):
    selected = state in ("empty", "unsupported", "positive")
    boxes = []
    if state in ("unsupported", "positive") and np.any(truth == 3):
        obj = points[truth == 3]
        uvh = np.column_stack((obj[:, :3], np.ones(len(obj)))) @ projection(setting).T
        front = uvh[:, 2] > 0
        uv = uvh[front, :2] / uvh[front, 2:3]
        if len(uv):
            boxes = [{"class": kind if state == "positive" else "generic_obstacle",
                      "xmin": float(uv[:, 0].min()), "xmax": float(uv[:, 0].max()),
                      "ymin": float(uv[:, 1].min()), "ymax": float(uv[:, 1].max())}]
    return {"selected_yolo_index": 1 if selected else None, "boxes": boxes,
            "selected_yolo_stamp_ns": stamp if selected else None,
            "selected_yolo_arrival_ns": stamp - 1 if selected else None,
            "scan_stamp_ns": stamp, "scan_effective_stamp_ns": stamp,
            "scan_arrival_ns": stamp, "decision_ns": stamp, "wait_ns": 0,
            "protection_enabled": state != "off",
            "reason": "matched_boxes" if boxes else "matched_empty" if selected else
                      "protection_off" if state == "off" else "missing_yolo",
            "invalid_yolo_count_before_decision": 1 if state == "invalid" else 0,
            "nearest_arrived_stamp_offset_ns": None,
            "unmatched_cause": "not_yet_arrived" if not selected else None}


def generate(legacy_snapshots, legacy_summary, setting_path, output):
    if output.exists():
        raise ValueError("output exists; never overwrite fixed inputs")
    output.mkdir(parents=True)
    (output / "clouds").mkdir()
    setting = yaml.safe_load(setting_path.read_text())
    summary = json.loads(legacy_summary.read_text())
    specs = {r["input_id"].rsplit("_frame", 1)[0]: r for r in summary["results"]}
    rows, truths, sources = [], [], []
    # Preserve all 180 historical frames and both matching decisions verbatim.
    for line in legacy_snapshots.open():
        row = json.loads(line)
        spec = specs[row["input_id"].rsplit("_frame", 1)[0]]
        cloud_path = (legacy_snapshots.parent / row["cloud_path"]).resolve()
        cloud, labels, _ = cloud_fixture(spec["distance_m"], spec["wall_gap_m"], spec["kind"])
        stored = np.fromfile(cloud_path, dtype="<f4").reshape(-1, 4)
        if not np.array_equal(stored, cloud):
            raise ValueError("historical fixture reconstruction mismatch")
        row["cloud_path"] = str(cloud_path)
        row["fixture_family"] = "historical_dense"
        rows.append(row)
        truths.append({"input_id": row["input_id"], "labels": labels.tolist(), **{
            k: spec[k] for k in ("kind", "wall_gap_m", "distance_m", "camera_result", "delay_ms")},
            "key_gate": spec["camera_result"] == "valid_empty_detection", "object_box": None})
    shapes = ["straight", "step", "curve"]
    cases = [(kind, side, "straight", state) for kind in KINDS for side in (-1, 1)
             for state in ("empty", "missing", "unsupported", "off", "invalid", "positive")]
    cases += [(None, side, shape, "missing") for side in (-1, 1) for shape in shapes]
    cases += [(kind, side, shape, "missing") for kind in KINDS for side in (-1, 1)
              for shape in ("step", "curve")]
    # Ground-height and nonzero leveling exercises preserve semantic point lineage.
    cases += [("car", side, "lower_floor", "missing") for side in (-1, 1)]
    cases += [("car", side, "tilted", "missing") for side in (-1, 1)]
    cases += [(kind, side, "occluded", "missing") for kind in ("car", "person") for side in (-1, 1)]
    cases += [(kind, 1, "dense", state) for kind in KINDS
              for state in ("empty", "missing", "unsupported", "off", "invalid", "positive")]
    for ci, (kind, side, shape, state) in enumerate(cases):
        ground = -1.84 if shape == "lower_floor" else -1.45
        if shape == "dense":
            cloud, labels, _ = cloud_fixture(15., .1, kind)
            box = None
        else:
            cloud, labels, box = first_return_scene(kind, side, ground=ground,
                shape="straight" if shape in ("lower_floor", "tilted", "occluded") else shape,
                occluder=shape == "occluded")
        roll, pitch = (np.deg2rad(3), np.deg2rad(2)) if shape == "tilted" else (0., 0.)
        if shape == "tilted":
            from run_synthetic_geometry import rotation
            cloud[:, :3] = cloud[:, :3] @ rotation(3, 2, 0)
        name = "sparse_{:02d}_{}_{}_{}_{}".format(ci, kind or "wall_only", side, shape, state)
        cloud_path = output / "clouds" / (name + ".bin")
        cloud.tofile(cloud_path)
        sources.append({"path": str(cloud_path.relative_to(output)),
                        "sha256": digest_file(cloud_path), "bytes": cloud_path.stat().st_size})
        for warm in range(5):
            stamp = 100_000_000_000 + (ci * 1000 + warm * 100) * 1_000_000
            decision = camera_decision(state, stamp, cloud, labels, setting, kind)
            identity = name + "_frame" + str(warm)
            rows.append({"input_id": identity, "scene": name, "phase": name,
                         "measurement": warm == 4, "geometry_valid": True,
                         "already_preprocessed": True, "roll": roll, "pitch": pitch, "yaw": 0,
                         "speed_mps": 0, "scan_stamp_ns": stamp,
                         "scan_effective_stamp_ns": stamp, "scan_arrival_ns": stamp,
                         "cloud_path": str(cloud_path.resolve()), "pose_evidence": {},
                         "fixture_family": "dense_negative_variants" if shape == "dense" else "analytic_first_return", "baseline": decision,
                         "candidate": decision})
            truths.append({"input_id": identity, "labels": labels.tolist(), "kind": kind,
                           "side": side, "shape": shape, "camera_result": state,
                           "wall_gap_m": .1, "distance_m": 15., "key_gate": False,
                           "object_box": box})
    for filename, values in (("snapshots.jsonl", rows), ("truth.jsonl", truths)):
        (output / filename).write_text("".join(canonical(x) + "\n" for x in values))
    manifest = {"schema": "asmc-l1b4-fixtures-v1", "frames": len(rows), "historical_cases": 36,
                "additional_cases": len(cases), "runtime_connected": False,
                "source_sha256": {str(p.resolve()): digest_file(p) for p in
                    (legacy_snapshots, legacy_summary, setting_path)},
                "snapshots_sha256": digest_file(output / "snapshots.jsonl"),
                "truth_sha256": digest_file(output / "truth.jsonl"), "generated_clouds": sources,
                "limits": ["Analytic first-return ray grid is not MORAI or VLP16 timing/visibility",
                           "All fixtures are stationary; no tracking accuracy claim",
                           "Candidate executable does not receive truth.jsonl",
                           "Already-preprocessed legacy input bypasses voxel by original design"]}
    (output / "manifest.json").write_text(json.dumps(manifest, sort_keys=True, indent=2) + "\n")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("legacy-snapshots", "legacy-summary", "setting", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    result = generate(args.legacy_snapshots, args.legacy_summary, args.setting, args.output)
    print(canonical({k: result[k] for k in ("frames", "historical_cases", "additional_cases")}))
