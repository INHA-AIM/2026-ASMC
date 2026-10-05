#!/usr/bin/env python3
"""Validate shadow-lane map pseudo-GT against front RGB and LiDAR paint returns."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import random

import cv2
import numpy as np
import rosbag
from sensor_msgs import point_cloud2

from asmc_perception.bev.calibration import load_calibration_snapshot
from asmc_perception.offline.bev.shadow_map_gt import (
    lidar_paint_alignment,
    map_xy_to_base,
    sha256_file,
)


FRONT_TOPIC = "/image_jpeg/compressed"
EGO_TOPIC = "/Ego_topic"
LIDAR_TOPIC = "/velodyne_points"
TF_STATIC_TOPIC = "/tf_static"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-run-root", required=True)
    parser.add_argument("--pseudo-gt", required=True)
    parser.add_argument("--calibration", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--sample-count", type=int, default=48)
    parser.add_argument("--contact-sheet-count", type=int, default=12)
    return parser.parse_args()


def _write_json_atomic(path: Path, value) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    temporary.replace(path)


def _bag_paths(root: Path):
    paths = sorted((root / "bags").glob("*.bag"))
    if not paths:
        paths = sorted(root.glob("*.bag"))
    if not paths:
        raise ValueError("no closed bag below {}".format(root))
    return paths


def _stamp_ns(message, bag_stamp) -> int:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is not None and stamp.to_nsec() > 0:
        return int(stamp.to_nsec())
    return int(bag_stamp.to_nsec())


def _inside_convex_polygon(point_xy, polygon_xy) -> bool:
    point = np.asarray(point_xy, dtype=np.float64).reshape(2)
    polygon = np.asarray(polygon_xy, dtype=np.float64).reshape(-1, 2)
    edge = np.roll(polygon, -1, axis=0) - polygon
    relative = point - polygon
    cross = edge[:, 0] * relative[:, 1] - edge[:, 1] * relative[:, 0]
    return bool(np.all(cross >= -1e-6) or np.all(cross <= 1e-6))


class _Reservoir:
    def __init__(self, capacity: int, seed: int) -> None:
        if int(capacity) <= 0:
            raise ValueError("sample count must be positive")
        self.capacity = int(capacity)
        self.random = random.Random(int(seed))
        self.seen = 0
        self.values = []

    def add(self, value) -> None:
        self.seen += 1
        if len(self.values) < self.capacity:
            self.values.append(value)
            return
        index = self.random.randrange(self.seen)
        if index < self.capacity:
            self.values[index] = value


def _ego_value(message, stamp_ns: int) -> dict:
    return {
        "stamp_ns": int(stamp_ns),
        "xy": np.asarray([message.position.x, message.position.y], dtype=np.float64),
        "heading_deg": float(message.heading),
    }


def _transform_curves(curves, ego, heading_offset_deg=0.0):
    return {
        name: map_xy_to_base(
            points[:, :2],
            ego["xy"],
            ego["heading_deg"] + float(heading_offset_deg),
        )
        for name, points in curves.items()
    }


def _draw_curve_segments(image, pixels, valid, color, width=3):
    for index in range(len(pixels) - 1):
        if valid[index] and valid[index + 1]:
            cv2.line(
                image,
                tuple(np.rint(pixels[index]).astype(int)),
                tuple(np.rint(pixels[index + 1]).astype(int)),
                color,
                width,
                cv2.LINE_AA,
            )


def _camera_overlay(event, curves, calibration, heading_offset_deg=0.0):
    image = cv2.imdecode(
        np.frombuffer(event["message"].data, dtype=np.uint8), cv2.IMREAD_COLOR
    )
    if image is None:
        raise ValueError("failed to decode front camera JPEG")
    transformed = _transform_curves(curves, event["ego"], heading_offset_deg)
    visible_counts = {}
    style = {
        "yellow_center": (0, 255, 255),
        "white_positive_normal": (255, 80, 255),
        "white_negative_normal": (255, 255, 255),
    }
    for name, points in transformed.items():
        pixels, depth = calibration.project_ground(points)
        valid = (
            (depth > 0.1)
            & (pixels[:, 0] >= 0.0)
            & (pixels[:, 0] < image.shape[1])
            & (pixels[:, 1] >= 0.0)
            & (pixels[:, 1] < image.shape[0])
        )
        visible_counts[name] = int(np.count_nonzero(valid))
        _draw_curve_segments(image, pixels, valid, style[name])
    text = "map pseudo-GT | heading={:.1f}{:+.2f} deg".format(
        event["ego"]["heading_deg"], float(heading_offset_deg)
    )
    cv2.putText(
        image, text, (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (20, 20, 20), 4, cv2.LINE_AA
    )
    cv2.putText(
        image, text, (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 220, 80), 2, cv2.LINE_AA
    )
    return image, visible_counts


def _bev_pixel(
    points,
    x_range=(-5.0, 25.0),
    y_range=(-8.0, 8.0),
    shape=(600, 320),
):
    value = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    row = (x_range[1] - value[:, 0]) / (x_range[1] - x_range[0]) * (shape[0] - 1)
    col = (y_range[1] - value[:, 1]) / (y_range[1] - y_range[0]) * (shape[1] - 1)
    return np.column_stack((col, row))


def _lidar_overlay(event, curves, lidar_translation, heading_offset_deg=0.0):
    fields = {value.name for value in event["message"].fields}
    if not {"x", "y", "z", "intensity"}.issubset(fields):
        raise ValueError("LiDAR cloud must contain x,y,z,intensity")
    points = np.asarray(
        list(
            point_cloud2.read_points(
                event["message"],
                field_names=("x", "y", "z", "intensity"),
                skip_nans=True,
            )
        ),
        dtype=np.float64,
    )
    points[:, :3] += lidar_translation
    points = points[
        (points[:, 2] >= -0.55)
        & (points[:, 2] <= -0.15)
        & (points[:, 0] >= 0.0)
        & (points[:, 0] <= 20.0)
        & (np.abs(points[:, 1]) <= 7.0)
    ]
    transformed = _transform_curves(curves, event["ego"], heading_offset_deg)
    # Preserve curve boundaries so the distance calculation cannot create a
    # fictitious segment between the end of one lane and the next.
    reference = np.stack(list(transformed.values()))
    counts = lidar_paint_alignment(points[:, [0, 1, 3]], reference)
    canvas = np.full((600, 320, 3), 18, dtype=np.uint8)
    if len(points):
        pixels = np.rint(_bev_pixel(points[:, :2])).astype(int)
        inside = (
            (pixels[:, 0] >= 0)
            & (pixels[:, 0] < canvas.shape[1])
            & (pixels[:, 1] >= 0)
            & (pixels[:, 1] < canvas.shape[0])
        )
        pixels = pixels[inside]
        intensity = points[inside, 3]
        canvas[pixels[:, 1], pixels[:, 0]] = np.where(
            (intensity >= 40.0)[:, None],
            np.asarray([255, 160, 30], dtype=np.uint8),
            np.asarray([90, 90, 90], dtype=np.uint8),
        )
    style = {
        "yellow_center": (0, 255, 255),
        "white_positive_normal": (255, 80, 255),
        "white_negative_normal": (255, 255, 255),
    }
    for name, value in transformed.items():
        pixels = np.rint(_bev_pixel(value)).astype(np.int32)
        cv2.polylines(canvas, [pixels], False, style[name], 2, cv2.LINE_AA)
    cv2.putText(
        canvas,
        "LiDAR high-I: blue",
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 190, 80),
        1,
        cv2.LINE_AA,
    )
    return canvas, counts


def _contact_sheet(images, count: int, columns: int, tile_size, output_path: Path):
    selected = images[: min(int(count), len(images))]
    if not selected:
        raise ValueError("no images available for contact sheet")
    rows = int(math.ceil(len(selected) / float(columns)))
    sheet = np.full(
        (rows * tile_size[1], columns * tile_size[0], 3), 12, dtype=np.uint8
    )
    for index, image in enumerate(selected):
        tile = cv2.resize(image, tile_size, interpolation=cv2.INTER_AREA)
        row, column = divmod(index, columns)
        y0, x0 = row * tile_size[1], column * tile_size[0]
        sheet[y0 : y0 + tile_size[1], x0 : x0 + tile_size[0]] = tile
    if not cv2.imwrite(
        str(output_path), sheet, [cv2.IMWRITE_JPEG_QUALITY, 92]
    ):
        raise ValueError("failed to write {}".format(output_path))


def _rate(numerator: int, denominator: int) -> float:
    return 0.0 if denominator <= 0 else float(numerator) / float(denominator)


def main():
    args = parse_args()
    root = Path(args.raw_run_root).expanduser().resolve()
    pseudo_path = Path(args.pseudo_gt).expanduser().resolve()
    calibration_path = Path(args.calibration).expanduser().resolve()
    output = Path(args.output_dir).expanduser().resolve()
    if output.exists():
        raise ValueError("output directory already exists: {}".format(output))
    output.mkdir(parents=True)
    pseudo = json.loads(pseudo_path.read_text(encoding="utf-8"))
    heading_offset_deg = float(
        (pseudo.get("pose_alignment") or {}).get("ego_heading_offset_deg", 0.0)
    )
    if not np.isfinite(heading_offset_deg) or abs(heading_offset_deg) > 5.0:
        raise ValueError("pseudo-GT ego heading offset is invalid")
    curves = {
        name: np.asarray(value, dtype=np.float64)
        for name, value in pseudo["boundaries"].items()
    }
    polygon = np.asarray(
        pseudo["shadow_area"]["polygon_xy_m"], dtype=np.float64
    )
    _, calibrations = load_calibration_snapshot(calibration_path)
    front = calibrations["front"]
    camera_samples = _Reservoir(args.sample_count, seed=1301)
    lidar_samples = _Reservoir(args.sample_count, seed=1302)
    latest_ego = None
    lidar_translation = np.asarray([0.8, 0.0, 1.45], dtype=np.float64)
    topic_types = {}
    bag_paths = _bag_paths(root)
    for bag_path in bag_paths:
        with rosbag.Bag(str(bag_path), "r") as bag:
            info = bag.get_type_and_topic_info().topics
            topic_types.update({name: value.msg_type for name, value in info.items()})
            required = {EGO_TOPIC, FRONT_TOPIC, LIDAR_TOPIC}
            if not required.issubset(info):
                raise ValueError(
                    "raw bag is missing topics: {}".format(
                        sorted(required - set(info))
                    )
                )
            for topic, message, bag_stamp in bag.read_messages(
                topics=[EGO_TOPIC, FRONT_TOPIC, LIDAR_TOPIC, TF_STATIC_TOPIC]
            ):
                stamp_ns = _stamp_ns(message, bag_stamp)
                if topic == TF_STATIC_TOPIC:
                    for transform in message.transforms:
                        if transform.child_frame_id == "velodyne":
                            lidar_translation = np.asarray(
                                [
                                    transform.transform.translation.x,
                                    transform.transform.translation.y,
                                    transform.transform.translation.z,
                                ],
                                dtype=np.float64,
                            )
                    continue
                if topic == EGO_TOPIC:
                    latest_ego = _ego_value(message, stamp_ns)
                    continue
                if latest_ego is None or not _inside_convex_polygon(
                    latest_ego["xy"], polygon
                ):
                    continue
                event = {
                    "stamp_ns": stamp_ns,
                    "ego": latest_ego,
                    "ego_age_ms": max(
                        0.0, (stamp_ns - latest_ego["stamp_ns"]) / 1e6
                    ),
                    "message": message,
                }
                if topic == FRONT_TOPIC:
                    camera_samples.add(event)
                elif topic == LIDAR_TOPIC:
                    lidar_samples.add(event)

    camera_events = sorted(camera_samples.values, key=lambda item: item["stamp_ns"])
    lidar_events = sorted(lidar_samples.values, key=lambda item: item["stamp_ns"])
    camera_images = []
    camera_visible = []
    for event in camera_events:
        image, visible = _camera_overlay(
            event, curves, front, heading_offset_deg
        )
        camera_images.append(image)
        camera_visible.append(visible)
    lidar_images = []
    lidar_counts = []
    for event in lidar_events:
        image, counts = _lidar_overlay(
            event, curves, lidar_translation, heading_offset_deg
        )
        lidar_images.append(image)
        lidar_counts.append(counts)
    _contact_sheet(
        camera_images,
        args.contact_sheet_count,
        columns=3,
        tile_size=(480, 270),
        output_path=output / "camera_map_overlay_contact_sheet.jpg",
    )
    _contact_sheet(
        lidar_images,
        args.contact_sheet_count,
        columns=4,
        tile_size=(320, 600),
        output_path=output / "lidar_map_overlay_contact_sheet.jpg",
    )
    totals = {
        key: int(sum(value[key] for value in lidar_counts))
        for key in (
            "near_points",
            "control_points",
            "near_high_points",
            "control_high_points",
        )
    }
    near_rate = _rate(totals["near_high_points"], totals["near_points"])
    control_rate = _rate(
        totals["control_high_points"], totals["control_points"]
    )
    camera_visible_all = sum(
        all(count >= 2 for count in value.values()) for value in camera_visible
    )
    automatic_checks = {
        "camera_sample_count_at_least_30": len(camera_events) >= 30,
        "lidar_sample_count_at_least_30": len(lidar_events) >= 30,
        "all_three_curves_visible_in_at_least_90pct_camera_frames": _rate(
            camera_visible_all, len(camera_events)
        )
        >= 0.90,
        "paint_high_intensity_uplift_at_least_5pct": near_rate - control_rate
        >= 0.05,
        "control_high_intensity_rate_below_5pct": control_rate <= 0.05,
    }
    document = {
        "schema_version": "asmc-shadow-map-alignment-audit-1.0.0",
        "status": "development_alignment_audit",
        "deployment_allowed": False,
        "raw_run": {
            "run_id": root.name,
            "path": str(root),
            "bag_files": [str(value) for value in bag_paths],
        },
        "pseudo_gt": {
            "path": str(pseudo_path),
            "sha256": sha256_file(pseudo_path),
            "ego_heading_offset_deg": heading_offset_deg,
        },
        "calibration": {
            "path": str(calibration_path),
            "sha256": sha256_file(calibration_path),
            "ground_plane": front.ground_plane.to_mapping(),
        },
        "topics": topic_types,
        "sampling": {
            "method": "deterministic_reservoir_inside_shadow_polygon",
            "requested_per_sensor": int(args.sample_count),
            "camera_candidates": int(camera_samples.seen),
            "camera_samples": int(len(camera_events)),
            "lidar_candidates": int(lidar_samples.seen),
            "lidar_samples": int(len(lidar_events)),
            "camera_ego_age_p95_ms": float(
                np.percentile(
                    [value["ego_age_ms"] for value in camera_events], 95
                )
            ),
            "lidar_ego_age_p95_ms": float(
                np.percentile(
                    [value["ego_age_ms"] for value in lidar_events], 95
                )
            ),
        },
        "camera_projection": {
            "frames_all_three_curves_visible": int(camera_visible_all),
            "all_three_curves_visible_rate": _rate(
                camera_visible_all, len(camera_events)
            ),
            "visible_points_median_by_curve": {
                name: float(
                    np.median([value[name] for value in camera_visible])
                )
                for name in curves
            },
        },
        "lidar_paint_alignment": {
            "lidar_translation_m": lidar_translation.tolist(),
            "ground_z_base_link_m": [-0.55, -0.15],
            "forward_x_m": [0.0, 20.0],
            "lateral_abs_y_m": 7.0,
            "near_distance_m": 0.20,
            "control_distance_m": [0.60, 1.20],
            "high_intensity_threshold": 40.0,
            "totals": totals,
            "near_high_intensity_rate": near_rate,
            "control_high_intensity_rate": control_rate,
            "absolute_uplift": near_rate - control_rate,
        },
        "automatic_checks": automatic_checks,
        "automatic_pass": all(automatic_checks.values()),
        "review_status": "pending_human_overlay_review",
        "limitations": [
            "MGeo link width creates pseudo boundaries; this is not surveyed paint GT.",
            "Static ground plane and Ego heading are used; dive/squat compensation is not applied.",
            "LiDAR paint contrast validates alignment statistically, not every boundary point.",
        ],
    }
    document["audit_sha256"] = hashlib.sha256(
        json.dumps(
            document, sort_keys=True, separators=(",", ":")
        ).encode("utf-8")
    ).hexdigest()
    _write_json_atomic(output / "audit.json", document)
    print("shadow map alignment audit: {}".format(output))
    print(json.dumps(automatic_checks, sort_keys=True))
    return 0 if document["automatic_pass"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
