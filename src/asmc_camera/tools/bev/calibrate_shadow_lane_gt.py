#!/usr/bin/env python3
"""Estimate shadow map pseudo-GT offsets from recorded LiDAR paint returns."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import rosbag
from sensor_msgs import point_cloud2

from asmc_camera.offline.bev.map_lane_calibration import (
    BOUNDARY_ORDER,
    BoundaryResidualProjector,
    calibrated_lane_geometry,
    estimate_residual_peak,
)


EGO_TOPIC = "/Ego_topic"
LIDAR_TOPIC = "/velodyne_points"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", required=True)
    parser.add_argument("--run-pattern", default="*shadow*")
    parser.add_argument("--pseudo-gt", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--frame-stride", type=int, default=5)
    parser.add_argument("--max-pose-age-ms", type=float, default=50.0)
    parser.add_argument("--intensity-threshold", type=float, default=40.0)
    parser.add_argument("--association-distance-m", type=float, default=0.9)
    parser.add_argument("--ego-heading-offset-deg", type=float, default=0.0)
    return parser.parse_args()


def _stamp_ns(message, bag_stamp):
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is not None and stamp.to_nsec() > 0:
        return int(stamp.to_nsec())
    return int(bag_stamp.to_nsec())


def _bag_paths(run_root: Path):
    paths = sorted((run_root / "bags").glob("*.bag"))
    if not paths:
        paths = sorted(run_root.glob("*.bag"))
    return [path for path in paths if not path.name.endswith(".active")]


def _inside_polygon(points_xy, polygon_xy):
    points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    polygon = np.asarray(polygon_xy, dtype=np.float64).reshape(-1, 2)
    edge = np.roll(polygon, -1, axis=0) - polygon
    relative = points[:, None, :] - polygon[None, :, :]
    cross = edge[None, :, 0] * relative[:, :, 1] - edge[None, :, 1] * relative[:, :, 0]
    return np.all(cross >= -1e-6, axis=1) | np.all(cross <= 1e-6, axis=1)


def _load_ego(paths):
    stamps = []
    values = []
    for path in paths:
        with rosbag.Bag(str(path), "r") as bag:
            for _, message, bag_stamp in bag.read_messages(topics=[EGO_TOPIC]):
                stamps.append(_stamp_ns(message, bag_stamp))
                values.append(
                    [message.position.x, message.position.y, float(message.heading)]
                )
    if not stamps:
        raise ValueError("run has no Ego topic")
    order = np.argsort(np.asarray(stamps, dtype=np.int64))
    return np.asarray(stamps, dtype=np.int64)[order], np.asarray(values, dtype=np.float64)[order]


def _nearest_pose(stamps, values, target_ns, max_age_ns):
    index = int(np.searchsorted(stamps, int(target_ns)))
    candidates = [value for value in (index - 1, index) if 0 <= value < len(stamps)]
    if not candidates:
        return None, None
    selected = min(candidates, key=lambda value: abs(int(stamps[value]) - int(target_ns)))
    delta = int(stamps[selected]) - int(target_ns)
    if abs(delta) > int(max_age_ns):
        return None, delta
    return values[selected], delta


def _base_to_map(points_xy, ego, heading_offset_deg=0.0):
    heading = np.radians(float(ego[2]) + float(heading_offset_deg))
    forward = np.asarray([np.cos(heading), np.sin(heading)])
    left = np.asarray([-np.sin(heading), np.cos(heading)])
    return (
        np.asarray(ego[:2], dtype=np.float64)
        + points_xy[:, 0:1] * forward
        + points_xy[:, 1:2] * left
    )


def _estimates_or_none(values):
    result = {}
    try:
        for name in BOUNDARY_ORDER:
            result[name] = estimate_residual_peak(np.concatenate(values[name]))
    except (ValueError, TypeError):
        return None
    return result


def _analyse_run(
    run_root,
    projector,
    polygon,
    frame_stride,
    max_pose_age_ns,
    intensity_threshold,
    association_distance_m,
    ego_heading_offset_deg,
):
    paths = _bag_paths(run_root)
    if not paths:
        return None
    ego_stamps, ego_values = _load_ego(paths)
    residuals = {name: [] for name in BOUNDARY_ORDER}
    counters = {
        "lidar_frames": 0,
        "sampled_frames": 0,
        "inside_shadow_frames": 0,
        "pose_age_drops": 0,
        "high_intensity_ground_points": 0,
    }
    pose_delta_ms = []
    sampled_index = 0
    for path in paths:
        with rosbag.Bag(str(path), "r") as bag:
            for _, message, bag_stamp in bag.read_messages(topics=[LIDAR_TOPIC]):
                counters["lidar_frames"] += 1
                sampled_index += 1
                if sampled_index % int(frame_stride) != 0:
                    continue
                counters["sampled_frames"] += 1
                stamp_ns = _stamp_ns(message, bag_stamp)
                ego, delta_ns = _nearest_pose(
                    ego_stamps, ego_values, stamp_ns, max_pose_age_ns
                )
                if ego is None:
                    counters["pose_age_drops"] += 1
                    continue
                pose_delta_ms.append(delta_ns / 1e6)
                if not _inside_polygon(ego[:2], polygon)[0]:
                    continue
                counters["inside_shadow_frames"] += 1
                fields = {field.name for field in message.fields}
                if not {"x", "y", "z", "intensity"}.issubset(fields):
                    raise ValueError("LiDAR schema lacks x,y,z,intensity")
                points = np.asarray(
                    list(
                        point_cloud2.read_points(
                            message,
                            field_names=("x", "y", "z", "intensity"),
                            skip_nans=True,
                        )
                    ),
                    dtype=np.float64,
                )
                if not len(points):
                    continue
                points[:, :3] += np.asarray([0.8, 0.0, 1.45])
                keep = (
                    (points[:, 2] >= -0.55)
                    & (points[:, 2] <= -0.15)
                    & (points[:, 0] >= 1.0)
                    & (points[:, 0] <= 25.0)
                    & (np.abs(points[:, 1]) <= 8.0)
                    & (points[:, 3] >= float(intensity_threshold))
                )
                points = points[keep]
                counters["high_intensity_ground_points"] += int(len(points))
                if not len(points):
                    continue
                map_xy = _base_to_map(
                    points[:, :2], ego, ego_heading_offset_deg
                )
                associated = projector.residuals(map_xy, association_distance_m)
                for name, values in associated.items():
                    if len(values):
                        residuals[name].append(values)
    estimates = _estimates_or_none(residuals)
    return {
        "run_id": run_root.name,
        "counters": counters,
        "pose_delta_ms": {
            "median": None if not pose_delta_ms else float(np.median(pose_delta_ms)),
            "p95_abs": None
            if not pose_delta_ms
            else float(np.percentile(np.abs(pose_delta_ms), 95)),
        },
        "estimates": None
        if estimates is None
        else {name: estimate.to_mapping() for name, estimate in estimates.items()},
        "_residuals": residuals,
    }


def main():
    args = parse_args()
    if args.frame_stride <= 0:
        raise ValueError("frame stride must be positive")
    pseudo_path = Path(args.pseudo_gt).resolve()
    with pseudo_path.open("r", encoding="utf-8") as stream:
        pseudo = json.load(stream)
    boundaries = {
        name: np.asarray(pseudo["boundaries"][name], dtype=np.float64)
        for name in BOUNDARY_ORDER
    }
    projector = BoundaryResidualProjector(boundaries)
    polygon = np.asarray(pseudo["shadow_area"]["polygon_xy_m"], dtype=np.float64)
    roots = sorted(Path(args.raw_root).resolve().glob(args.run_pattern))
    roots = [root for root in roots if root.is_dir() and _bag_paths(root)]
    if not roots:
        raise ValueError("no raw runs match the requested pattern")

    combined = {name: [] for name in BOUNDARY_ORDER}
    run_reports = []
    for index, root in enumerate(roots, 1):
        print("[{}/{}] {}".format(index, len(roots), root.name), flush=True)
        report = _analyse_run(
            root,
            projector,
            polygon,
            args.frame_stride,
            int(round(args.max_pose_age_ms * 1e6)),
            args.intensity_threshold,
            args.association_distance_m,
            args.ego_heading_offset_deg,
        )
        if report is None:
            continue
        for name, values in report.pop("_residuals").items():
            combined[name].extend(values)
        run_reports.append(report)

    estimates = {
        name: estimate_residual_peak(np.concatenate(combined[name]))
        for name in BOUNDARY_ORDER
    }
    geometry = calibrated_lane_geometry(
        float(pseudo["nominal_lane_width_m"]), estimates
    )
    output = {
        "schema_version": "asmc-shadow-lane-lidar-calibration-1.0.0",
        "status": "calibration_candidate",
        "deployment_allowed": False,
        "pseudo_gt": str(pseudo_path),
        "run_pattern": args.run_pattern,
        "parameters": {
            "frame_stride": int(args.frame_stride),
            "max_pose_age_ms": float(args.max_pose_age_ms),
            "intensity_threshold": float(args.intensity_threshold),
            "association_distance_m": float(args.association_distance_m),
            "ego_heading_offset_deg": float(args.ego_heading_offset_deg),
            "lidar_translation_m": [0.8, 0.0, 1.45],
            "ground_z_range_m": [-0.55, -0.15],
        },
        "combined_estimates": {
            name: estimate.to_mapping() for name, estimate in estimates.items()
        },
        "recommended_geometry": geometry,
        "runs": run_reports,
        "note": "LiDAR paint-return calibration candidate; confirm in raw RGB BEV before promotion.",
    }
    output_path = Path(args.output).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_suffix(output_path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(output, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    temporary.replace(output_path)
    print(json.dumps(output["combined_estimates"], indent=2, sort_keys=True))
    print(json.dumps(geometry, indent=2, sort_keys=True))
    print("wrote {}".format(output_path))


if __name__ == "__main__":
    main()
