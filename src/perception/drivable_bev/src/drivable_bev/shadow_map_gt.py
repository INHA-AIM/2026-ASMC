"""Build deterministic map-derived lane pseudo-GT for the GPS-shadow ODD."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np


SCHEMA_VERSION = "asmc-shadow-lane-pseudo-gt-1.0.0"


def sha256_file(path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _polyline(points) -> np.ndarray:
    value = np.asarray(points, dtype=np.float64)
    if value.ndim != 2 or value.shape[1] < 2 or len(value) < 2:
        raise ValueError("map polyline must contain at least two XY points")
    value = value[:, :3] if value.shape[1] >= 3 else np.column_stack(
        (value[:, :2], np.zeros(len(value), dtype=np.float64))
    )
    if not np.isfinite(value).all():
        raise ValueError("map polyline contains non-finite values")
    distance = np.linalg.norm(np.diff(value[:, :2], axis=0), axis=1)
    keep = np.concatenate(([True], distance > 1e-9))
    value = value[keep]
    if len(value) < 2:
        raise ValueError("map polyline has no measurable length")
    return value


def polyline_length(points) -> float:
    value = _polyline(points)
    return float(np.sum(np.linalg.norm(np.diff(value[:, :2], axis=0), axis=1)))


def resample_polyline(points, count: int) -> np.ndarray:
    value = _polyline(points)
    if int(count) < 2:
        raise ValueError("resample count must be at least two")
    segment = np.linalg.norm(np.diff(value[:, :2], axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment)))
    targets = np.linspace(0.0, cumulative[-1], int(count))
    return np.column_stack(
        [np.interp(targets, cumulative, value[:, axis]) for axis in range(3)]
    )


def align_opposing_polylines(first, second) -> tuple[np.ndarray, bool]:
    """Orient the second polyline to minimize corresponding end-point error."""
    first_value = _polyline(first)
    second_value = _polyline(second)
    direct = (
        np.linalg.norm(first_value[0, :2] - second_value[0, :2])
        + np.linalg.norm(first_value[-1, :2] - second_value[-1, :2])
    )
    reversed_cost = (
        np.linalg.norm(first_value[0, :2] - second_value[-1, :2])
        + np.linalg.norm(first_value[-1, :2] - second_value[0, :2])
    )
    reversed_second = bool(reversed_cost < direct)
    return (second_value[::-1].copy() if reversed_second else second_value), reversed_second


def rotated_rectangle(
    area: Mapping, margin_m: float = 0.0, yaw_offset_deg: float = 0.0
) -> np.ndarray:
    position = area.get("pos") or {}
    rotation = area.get("rot") or {}
    size = area.get("size") or {}
    center = np.asarray([float(position["x"]), float(position["y"])])
    half = np.asarray(
        [float(size["x"]) / 2.0 - margin_m, float(size["y"]) / 2.0 - margin_m]
    )
    if np.any(half <= 0.0):
        raise ValueError("shadow area becomes empty after applying its margin")
    yaw = math.radians(float(rotation["yaw"]) + float(yaw_offset_deg))
    transform = np.asarray(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]],
        dtype=np.float64,
    )
    local = np.asarray(
        [[-half[0], -half[1]], [half[0], -half[1]], [half[0], half[1]], [-half[0], half[1]]],
        dtype=np.float64,
    )
    return local @ transform.T + center


def points_in_rotated_rectangle(
    points, area: Mapping, margin_m: float = 0.0, yaw_offset_deg: float = 0.0
) -> np.ndarray:
    value = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    position = area.get("pos") or {}
    rotation = area.get("rot") or {}
    size = area.get("size") or {}
    center = np.asarray([float(position["x"]), float(position["y"])])
    half = np.asarray(
        [float(size["x"]) / 2.0 - margin_m, float(size["y"]) / 2.0 - margin_m]
    )
    if np.any(half <= 0.0):
        raise ValueError("shadow area becomes empty after applying its margin")
    yaw = math.radians(float(rotation["yaw"]) + float(yaw_offset_deg))
    inverse = np.asarray(
        [[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]],
        dtype=np.float64,
    )
    local = (value - center) @ inverse.T
    return np.all(np.abs(local) <= half + 1e-9, axis=1)


def map_xy_to_base(points_xy, ego_xy, heading_deg: float) -> np.ndarray:
    """Transform MORAI map XY points into rear-axle ``base_link`` XY.

    EgoVehicleStatus.heading follows the mathematical map convention used by
    the K-City links: zero points along +map-x and positive angles toward
    +map-y. The returned y axis is vehicle-left.
    """
    points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    origin = np.asarray(ego_xy, dtype=np.float64).reshape(2)
    values = np.asarray([*origin, float(heading_deg)], dtype=np.float64)
    if not np.isfinite(values).all():
        raise ValueError("ego map pose must be finite")
    heading = math.radians(float(heading_deg))
    forward = np.asarray([math.cos(heading), math.sin(heading)])
    left = np.asarray([-math.sin(heading), math.cos(heading)])
    delta = points - origin
    return np.column_stack((delta @ forward, delta @ left))


def nearest_polyline_distance(points_xy, reference_xy, chunk_size: int = 256) -> np.ndarray:
    """Return nearest point-to-segment distances with bounded temporary memory.

    ``reference_xy`` may be one ``N×2`` polyline or a ``K×N×2`` family. Curve
    ends are never connected to the next curve in the family.
    """
    points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    reference = np.asarray(reference_xy, dtype=np.float64)
    if reference.ndim == 2 and reference.shape[1:] == (2,):
        reference = reference[None, :, :]
    if reference.ndim != 3 or reference.shape[2] != 2 or reference.shape[1] < 2:
        raise ValueError("reference must contain one or more N-by-2 polylines")
    if int(chunk_size) <= 0:
        raise ValueError("chunk size must be positive")
    segment_start = reference[:, :-1, :].reshape(-1, 2)
    segment_delta = np.diff(reference, axis=1).reshape(-1, 2)
    segment_length_squared = np.sum(segment_delta * segment_delta, axis=1)
    keep = segment_length_squared > 1e-12
    segment_start = segment_start[keep]
    segment_delta = segment_delta[keep]
    segment_length_squared = segment_length_squared[keep]
    if not len(segment_start):
        raise ValueError("reference polylines have no measurable segment")
    result = np.empty(len(points), dtype=np.float64)
    for start in range(0, len(points), int(chunk_size)):
        stop = min(len(points), start + int(chunk_size))
        relative = points[start:stop, None, :] - segment_start[None, :, :]
        parameter = np.clip(
            np.sum(relative * segment_delta[None, :, :], axis=2)
            / segment_length_squared[None, :],
            0.0,
            1.0,
        )
        closest = segment_start[None, :, :] + parameter[:, :, None] * segment_delta[None, :, :]
        squared = np.sum((points[start:stop, None, :] - closest) ** 2, axis=2)
        result[start:stop] = np.sqrt(np.min(squared, axis=1))
    return result


def lidar_paint_alignment(
    points_xyi,
    reference_xy,
    near_distance_m: float = 0.20,
    control_distance_min_m: float = 0.60,
    control_distance_max_m: float = 1.20,
    high_intensity_threshold: float = 40.0,
) -> dict:
    """Compare LiDAR reflectivity near pseudo boundaries with nearby asphalt."""
    points = np.asarray(points_xyi, dtype=np.float64).reshape(-1, 3)
    limits = np.asarray(
        [
            near_distance_m,
            control_distance_min_m,
            control_distance_max_m,
            high_intensity_threshold,
        ],
        dtype=np.float64,
    )
    if not np.isfinite(limits).all():
        raise ValueError("LiDAR alignment thresholds must be finite")
    if not 0.0 < near_distance_m < control_distance_min_m < control_distance_max_m:
        raise ValueError("LiDAR alignment distance bands are invalid")
    if not len(points):
        return {
            "near_points": 0,
            "control_points": 0,
            "near_high_points": 0,
            "control_high_points": 0,
        }
    distance = nearest_polyline_distance(points[:, :2], reference_xy)
    near = distance <= float(near_distance_m)
    control = (distance >= float(control_distance_min_m)) & (
        distance <= float(control_distance_max_m)
    )
    high = points[:, 2] >= float(high_intensity_threshold)
    return {
        "near_points": int(np.count_nonzero(near)),
        "control_points": int(np.count_nonzero(control)),
        "near_high_points": int(np.count_nonzero(near & high)),
        "control_high_points": int(np.count_nonzero(control & high)),
    }


def _find_by_id(values: Sequence[Mapping], key: str, expected) -> Mapping:
    matches = [value for value in values if value.get(key) == expected]
    if len(matches) != 1:
        raise ValueError("expected exactly one {}={!r}, got {}".format(key, expected, len(matches)))
    return matches[0]


def _link_width(link: Mapping) -> float:
    widths = [
        float(value)
        for value in (link.get("width_start"), link.get("width_end"))
        if value is not None
    ]
    if not widths or not np.isfinite(widths).all() or min(widths) <= 0.0:
        raise ValueError("link does not provide a positive nominal width")
    return float(np.mean(widths))


def build_shadow_lane_pseudo_gt(
    links: Sequence[Mapping], scenario: Mapping, config: Mapping
) -> dict:
    if config.get("schema_version") != "asmc-shadow-zone-config-1.0.0":
        raise ValueError("unsupported shadow-zone config schema")
    link_ids = [str(value) for value in config["links"]["ids"]]
    if len(link_ids) != 2 or link_ids[0] == link_ids[1]:
        raise ValueError("exactly two distinct opposing link IDs are required")
    pose_alignment = dict(config.get("pose_alignment") or {})
    ego_heading_offset_deg = float(
        pose_alignment.get("ego_heading_offset_deg", 0.0)
    )
    if not np.isfinite(ego_heading_offset_deg) or abs(ego_heading_offset_deg) > 5.0:
        raise ValueError("ego heading offset must be finite and within 5 degrees")
    selected_links = [_find_by_id(links, "idx", value) for value in link_ids]
    area_id = int(config["shadow_area"]["unique_id"])
    area = _find_by_id(scenario.get("shadedAreaList") or [], "UNIQUEID", area_id)
    yaw_offset_deg = float(
        config["shadow_area"].get("yaw_to_map_offset_deg", 0.0)
    )

    first = _polyline(selected_links[0]["points"])
    second, reversed_second = align_opposing_polylines(
        first, selected_links[1]["points"]
    )
    spacing = float(config["geometry"]["resample_spacing_m"])
    if spacing <= 0.0:
        raise ValueError("resample spacing must be positive")
    common_length = min(polyline_length(first), polyline_length(second))
    sample_count = max(2, int(math.floor(common_length / spacing)) + 1)
    first = resample_polyline(first, sample_count)
    second = resample_polyline(second, sample_count)

    yellow = 0.5 * (first + second)
    tangent = np.gradient(yellow[:, :2], axis=0)
    tangent_norm = np.linalg.norm(tangent, axis=1)
    if np.any(tangent_norm <= 1e-9):
        raise ValueError("derived road centre has a degenerate tangent")
    tangent /= tangent_norm[:, None]
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    separation_vector = first[:, :2] - second[:, :2]
    normal *= np.where(np.sum(normal * separation_vector, axis=1) >= 0.0, 1.0, -1.0)[:, None]

    link_axis = yellow[-1, :2] - yellow[0, :2]
    link_axis_deg = math.degrees(math.atan2(link_axis[1], link_axis[0]))
    area_axis_deg = float(area["rot"]["yaw"]) + yaw_offset_deg
    axis_error_deg = abs((link_axis_deg - area_axis_deg + 90.0) % 180.0 - 90.0)
    max_axis_error_deg = float(
        config["shadow_area"].get("max_link_axis_error_deg", 10.0)
    )
    if axis_error_deg > max_axis_error_deg:
        raise ValueError(
            "shadow-area long axis differs from aligned links by {:.3f} deg".format(
                axis_error_deg
            )
        )

    link_widths = [_link_width(value) for value in selected_links]
    half_road_width = 0.5 * (link_widths[0] + link_widths[1])
    expected = float(config["geometry"]["expected_lane_width_m"])
    tolerance = float(config["geometry"]["lane_width_tolerance_m"])
    if abs(half_road_width - expected) > tolerance:
        raise ValueError("nominal link width differs from the configured lane-width prior")
    center_offset = float(
        config["geometry"].get("center_lateral_offset_m", 0.0)
    )
    positive_width = float(
        config["geometry"].get("positive_lane_width_m", half_road_width)
    )
    negative_width = float(
        config["geometry"].get("negative_lane_width_m", half_road_width)
    )
    calibrated = np.asarray(
        [center_offset, positive_width, negative_width], dtype=np.float64
    )
    if not np.isfinite(calibrated).all() or min(positive_width, negative_width) <= 0.0:
        raise ValueError("calibrated boundary geometry must be finite and positive")
    if abs(center_offset) > tolerance:
        raise ValueError("calibrated road-centre offset exceeds lane-width tolerance")
    if any(
        abs(width - expected) > tolerance
        for width in (positive_width, negative_width)
    ):
        raise ValueError("calibrated lane width differs from the configured prior")

    yellow[:, :2] += center_offset * normal
    positive = yellow.copy()
    negative = yellow.copy()
    positive[:, :2] += positive_width * normal
    negative[:, :2] -= negative_width * normal

    margin = float(config["geometry"].get("clip_margin_m", 0.0))
    # The scenario volume marks where GPS reception is shaded; it is not an
    # surveyed road-edge polygon. Clip the road-centre station to that volume,
    # then retain the corresponding normal-offset boundaries. Requiring both
    # outer paint lines to lie inside the approximate 7.419 m volume would
    # incorrectly collapse the 118 m curved section to only a few metres.
    keep = points_in_rotated_rectangle(
        yellow[:, :2], area, margin, yaw_offset_deg
    )
    indices = np.flatnonzero(keep)
    if len(indices) < 2:
        raise ValueError("derived boundaries do not overlap the configured shadow area")
    # Keep the longest contiguous interval; a rotated volume should intersect
    # this road once and disjoint fragments indicate a bad map/config pairing.
    splits = np.flatnonzero(np.diff(indices) > 1) + 1
    groups = np.split(indices, splits)
    chosen = max(groups, key=len)
    if len(chosen) < 2:
        raise ValueError("shadow-area clipping left no usable boundary segment")
    first = first[chosen]
    second = second[chosen]
    yellow = yellow[chosen]
    positive = positive[chosen]
    negative = negative[chosen]

    separation = np.linalg.norm(first[:, :2] - second[:, :2], axis=1)
    lengths = {
        "yellow_center_m": polyline_length(yellow),
        "white_positive_normal_m": polyline_length(positive),
        "white_negative_normal_m": polyline_length(negative),
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "map_pseudo_gt",
        "deployment_allowed": False,
        "map": str(config["map"]),
        "operating_domain": str(config["operating_domain"]),
        "shadow_area": {
            "unique_id": area_id,
            "center_xy_m": [float(area["pos"]["x"]), float(area["pos"]["y"])],
            "source_yaw_deg": float(area["rot"]["yaw"]),
            "yaw_to_map_offset_deg": yaw_offset_deg,
            "map_long_axis_deg": area_axis_deg,
            "link_axis_deg": link_axis_deg,
            "link_axis_error_deg": axis_error_deg,
            "size_xyz_m": [
                float(area["size"]["x"]),
                float(area["size"]["y"]),
                float(area["size"]["z"]),
            ],
            "polygon_xy_m": rotated_rectangle(
                area, yaw_offset_deg=yaw_offset_deg
            ).tolist(),
        },
        "source_links": [
            {
                "id": link_ids[index],
                "nominal_width_m": link_widths[index],
                "original_length_m": polyline_length(selected_links[index]["points"]),
            }
            for index in range(2)
        ],
        "alignment": {
            "second_link_reversed": reversed_second,
            "resample_spacing_m": spacing,
            "sample_count_before_clip": sample_count,
            "sample_count": int(len(yellow)),
            "center_separation_m": {
                "p05": float(np.percentile(separation, 5)),
                "median": float(np.percentile(separation, 50)),
                "p95": float(np.percentile(separation, 95)),
            },
        },
        "nominal_lane_width_m": half_road_width,
        "boundary_geometry": {
            "center_lateral_offset_m": center_offset,
            "positive_lane_width_m": positive_width,
            "negative_lane_width_m": negative_width,
            "calibration": dict(config["geometry"].get("calibration") or {}),
        },
        "pose_alignment": {
            "ego_heading_offset_deg": ego_heading_offset_deg,
            "calibration": dict(pose_alignment.get("calibration") or {}),
        },
        "lengths": lengths,
        "boundaries": {
            "yellow_center": yellow.tolist(),
            "white_positive_normal": positive.tolist(),
            "white_negative_normal": negative.tolist(),
        },
        "validation": {
            "review_status": "pending_camera_review_after_lidar_calibration"
            if config["geometry"].get("calibration")
            else "pending_camera_lidar_overlay",
            "minimum_review_frames": 30,
            "recommended_review_frames": 60,
        },
        "note": str((config.get("provenance") or {}).get("note", "")),
    }


def with_provenance(document: Mapping, config_path, link_path, scenario_path) -> dict:
    value = dict(document)
    value["provenance"] = {
        "config": {"path": str(Path(config_path)), "sha256": sha256_file(config_path)},
        "link_set": {"path": str(Path(link_path)), "sha256": sha256_file(link_path)},
        "scenario": {"path": str(Path(scenario_path)), "sha256": sha256_file(scenario_path)},
        "generator": "drivable_bev.shadow_map_gt",
    }
    canonical = json.dumps(value, sort_keys=True, separators=(",", ":"))
    value["artifact_sha256"] = hashlib.sha256(canonical.encode("utf-8")).hexdigest()
    return value
