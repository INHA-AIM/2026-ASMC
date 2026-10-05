#!/usr/bin/env python3
"""Build a map-center trace of known background from an obstacle-free lap.

The input is the existing post-tracking `/lidar/tracked_objects_map` topic.
No raw point cloud or additional diagnostic topic is required.  Motion labels
and velocity are deliberately not used: ego motion and changing LiDAR
visibility can contaminate both for long fixtures such as guardrails.  The
obstacle-free-lap contract makes spatially repeated explicit UNKNOWN centers
eligible; known vehicle/person classes are never catalogued.
"""

import argparse
import collections
import hashlib
import math
import os
import statistics
import sys

import rosbag
import yaml


def finite(*values):
    return all(math.isfinite(float(value)) for value in values)


def median(values):
    return float(statistics.median(values))


def normalize_label(value):
    return str(value or "").strip().lower()


def axis_median(angles):
    # An OBB heading is axial: yaw and yaw + pi describe the same box.
    sine = sum(math.sin(2.0 * angle) for angle in angles)
    cosine = sum(math.cos(2.0 * angle) for angle in angles)
    return 0.5 * math.atan2(sine, cosine)


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("output")
    parser.add_argument("--topic", default="/lidar/tracked_objects_map")
    parser.add_argument("--map-id", default="waize_katri_2025-10-23")
    parser.add_argument(
        "--origin", nargs=3, type=float,
        default=[302595.0, 4124145.0, 0.0])
    parser.add_argument("--min-x", type=float, default=-454.5159)
    parser.add_argument("--max-x", type=float, default=162.4841)
    parser.add_argument("--min-y", type=float, default=-825.0458)
    parser.add_argument("--max-y", type=float, default=621.3542)
    parser.add_argument("--min-observations", type=int, default=3)
    parser.add_argument("--minimum-track-span-s", type=float, default=0.10)
    # PlannerObjectAdapter already rejects/clamps geometry below 0.05 m.  Keep
    # that same physical validity floor here: guardrail rails and recurrent
    # ground remnants are often only 0.05--0.15 m tall after segmentation.
    parser.add_argument("--minimum-height-m", type=float, default=0.05)
    parser.add_argument("--center-bin-m", type=float, default=0.50)
    # After a track itself has satisfied the recurrence/time requirement, keep
    # every visited center bin.  Long guardrails make one tracked OBB center
    # slide along their length, so requiring repeated hits in each bin leaves
    # holes even though the physical background track is well established.
    parser.add_argument("--minimum-bin-observations", type=int, default=1)
    parser.add_argument("--minimum-spatial-observations", type=int, default=3)
    return parser.parse_args()


def valid_unknown_object(obj, args):
    classification = normalize_label(obj.classification)
    yolo = normalize_label(obj.yolo_class)
    if classification != "unknown":
        return False
    if yolo not in ("", "unknown"):
        return False
    if int(obj.id) < 0 or int(obj.miss_count) != 0:
        return False
    values = (
        obj.cluster_center.x, obj.cluster_center.y, obj.cluster_center.z,
        obj.length, obj.width, obj.height, obj.obb_yaw)
    if not finite(*values):
        return False
    if not (args.min_x <= obj.cluster_center.x <= args.max_x and
            args.min_y <= obj.cluster_center.y <= args.max_y):
        return False
    if min(obj.length, obj.width) < 0.05 or obj.height < args.minimum_height_m:
        return False
    return True


def representative(observations, track_id):
    return {
        "track_id": int(track_id),
        "center": [
            round(median([item["x"] for item in observations]), 4),
            round(median([item["y"] for item in observations]), 4),
        ],
        "size": [
            round(median([item["length"] for item in observations]), 4),
            round(median([item["width"] for item in observations]), 4),
            round(median([item["height"] for item in observations]), 4),
        ],
        "yaw_rad": round(axis_median([item["yaw"] for item in observations]), 6),
        "support": len(observations),
    }


def append_if_distinct(output, candidate, center_tolerance):
    for reference in output:
        distance = math.hypot(
            candidate["center"][0] - reference["center"][0],
            candidate["center"][1] - reference["center"][1])
        if distance <= center_tolerance:
            return False
    output.append(candidate)
    return True


def main():
    args = parse_args()
    tracks = collections.defaultdict(list)
    eligible_samples = []
    message_count = 0
    rejected_frame_count = 0

    with rosbag.Bag(args.bag, "r") as bag:
        for _, message, _ in bag.read_messages(topics=[args.topic]):
            message_count += 1
            if message.header.frame_id != "map":
                rejected_frame_count += 1
                continue
            stamp = message.header.stamp.to_sec()
            if not math.isfinite(stamp) or stamp <= 0.0:
                rejected_frame_count += 1
                continue
            for obj in message.objects:
                if not valid_unknown_object(obj, args):
                    continue
                sample = {
                    "stamp": stamp,
                    "track_id": int(obj.id),
                    "x": float(obj.cluster_center.x),
                    "y": float(obj.cluster_center.y),
                    "length": float(obj.length),
                    "width": float(obj.width),
                    "height": float(obj.height),
                    "yaw": float(obj.obb_yaw),
                    "speed": float(obj.speed_kph),
                    "dynamic": bool(obj.is_dynamic),
                }
                tracks[int(obj.id)].append(sample)
                eligible_samples.append(sample)

    accepted_tracks = 0
    observations = []
    for track_id, samples in sorted(tracks.items()):
        if len(samples) < max(1, args.min_observations):
            continue
        ordered = sorted(samples, key=lambda item: item["stamp"])
        if ordered[-1]["stamp"] - ordered[0]["stamp"] < \
                args.minimum_track_span_s:
            continue

        bins = collections.defaultdict(list)
        for sample in samples:
            key = (
                int(round(sample["x"] / args.center_bin_m)),
                int(round(sample["y"] / args.center_bin_m)),
            )
            bins[key].append(sample)
        track_references = [
            representative(group, track_id)
            for _, group in sorted(bins.items())
            if len(group) >= args.minimum_bin_observations
        ]
        if not track_references:
            track_references = [representative(samples, track_id)]
        for reference in track_references:
            append_if_distinct(
                observations, reference, 0.25 * args.center_bin_m)
        accepted_tracks += 1

    # Tracking can fragment a thin pole or divider into several short IDs.
    # Recover spatially recurrent clean-lap centers across those IDs. This is
    # still object-level evidence after tracking; it does not alter clustering.
    spatial_bins = collections.defaultdict(list)
    for sample in eligible_samples:
        key = (
            int(round(sample["x"] / args.center_bin_m)),
            int(round(sample["y"] / args.center_bin_m)),
        )
        spatial_bins[key].append(sample)
    accepted_spatial_bins = 0
    for _, samples in sorted(spatial_bins.items()):
        if len(samples) < max(2, args.minimum_spatial_observations):
            continue
        if max(item["stamp"] for item in samples) - \
                min(item["stamp"] for item in samples) < 0.10:
            continue
        track_id = collections.Counter(
            item["track_id"] for item in samples).most_common(1)[0][0]
        if append_if_distinct(
                observations, representative(samples, track_id),
                0.25 * args.center_bin_m):
            accepted_spatial_bins += 1

    if not observations:
        print("No stable UNKNOWN tracks satisfied the catalog policy", file=sys.stderr)
        return 2

    output = {
        "schema": "asmc_known_static_center_trace_v1",
        "map_id": args.map_id,
        "local_origin_in_global": [float(value) for value in args.origin],
        "source": {
            "bag": os.path.basename(args.bag),
            "sha256": sha256(args.bag),
            "topic": args.topic,
            "messages": message_count,
            "rejected_frames": rejected_frame_count,
        },
        "policy": {
            "explicit_unknown_only": True,
            "obstacle_free_lap_required": True,
            "motion_label_used": False,
            "velocity_used": False,
            "min_observations": args.min_observations,
            "minimum_track_span_s": args.minimum_track_span_s,
            "minimum_height_m": args.minimum_height_m,
            "center_bin_m": args.center_bin_m,
            "minimum_bin_observations": args.minimum_bin_observations,
            "minimum_spatial_observations": args.minimum_spatial_observations,
        },
        "track_count": accepted_tracks,
        "fragmented_spatial_bin_count": accepted_spatial_bins,
        "observation_count": len(observations),
        "observations": observations,
    }
    with open(args.output, "w", encoding="utf-8") as stream:
        yaml.safe_dump(output, stream, sort_keys=False, allow_unicode=True)
    print(
        "Wrote {} references from {} stable tracks to {}".format(
            len(observations), accepted_tracks, args.output))
    return 0


if __name__ == "__main__":
    sys.exit(main())
