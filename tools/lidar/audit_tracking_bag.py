#!/usr/bin/env python3
"""Read-only LiDAR timing/frame/handoff audit; no ROS master or playback needed.

This is a contract audit, not an accuracy benchmark. Empty object arrays do not
prove free space, and ID changes without ground truth are not ID-switch metrics.
The ROS import is lazy so the audit core can be tested without ROS installed.
"""

import argparse
import bisect
import collections
import json
import math
from pathlib import Path
import struct


DEFAULT_CLOUDS = (
    "/velodyne_points", "/lidar/debug_raw", "/lidar/debug_roi",
    "/lidar/debug_ransac", "/lidar/debug_clustered",
)
DIAGNOSTIC_TOPICS = ("/lidar/tunnel/diagnostics", "/morai_sensor_bridge/diagnostics", "/morai_udp_bridge/diagnostics")


def distribution(values):
    finite = sorted(float(value) for value in values if math.isfinite(value))
    if not finite:
        return {"count": 0, "min": None, "p50": None, "p95": None, "max": None}

    def percentile(fraction):
        index = (len(finite) - 1) * fraction
        low, high = math.floor(index), math.ceil(index)
        return finite[low] + (finite[high] - finite[low]) * (index - low)

    return {"count": len(finite), "min": finite[0], "p50": percentile(.5),
            "p95": percentile(.95), "max": finite[-1]}


def nearest_index(stamps, value):
    index = bisect.bisect_left(stamps, value)
    candidates = [candidate for candidate in (index - 1, index)
                  if 0 <= candidate < len(stamps)]
    return min(candidates, key=lambda candidate: abs(stamps[candidate] - value))


def cloud_summary(message, max_sample_points):
    """Respect organized row padding, field offsets and byte order."""
    count = int(message.width) * int(message.height)
    schema = sorted((field.name, int(field.offset), int(field.datatype), int(field.count))
                    for field in message.fields)
    result = {"points": count, "schema": schema, "sampled_points": 0,
              "sampled_nonfinite_xyz": 0, "sampled_near_origin_xyz": 0,
              "malformed_layout": False}
    required = int(message.row_step) * int(message.height)
    result["malformed_layout"] = (
        len(message.data) < required or int(message.row_step) < int(message.width) * int(message.point_step)
        or int(message.point_step) <= 0)
    fields = {field.name: field for field in message.fields}
    if count == 0 or max_sample_points == 0 or result["malformed_layout"]:
        return result
    if any(name not in fields or fields[name].datatype not in (7, 8) for name in ("x", "y", "z")):
        result["unsupported_xyz_fields"] = True
        return result
    endian = ">" if message.is_bigendian else "<"
    decoders = [(int(fields[name].offset), struct.Struct(endian + ("f" if fields[name].datatype == 7 else "d")))
                for name in ("x", "y", "z")]
    if any(offset + decoder.size > message.point_step for offset, decoder in decoders):
        result["malformed_layout"] = True
        return result
    stride = max(1, math.ceil(count / max_sample_points))
    for index in range(0, count, stride):
        row, column = divmod(index, int(message.width))
        offset = row * int(message.row_step) + column * int(message.point_step)
        xyz = [decoder.unpack_from(message.data, offset + field_offset)[0]
               for field_offset, decoder in decoders]
        result["sampled_points"] += 1
        if not all(math.isfinite(value) for value in xyz):
            result["sampled_nonfinite_xyz"] += 1
        elif sum(value * value for value in xyz) <= 1.0:
            result["sampled_near_origin_xyz"] += 1
    return result


class TrackingAudit:
    def __init__(self, map_topic="/last_lidar", local_topic="/lidar_nogps",
                 local_frame="velodyne", gps_max_age_sec=.5, pair_tolerance_ms=2.0):
        self.map_topic, self.local_topic = map_topic, local_topic
        self.expected_frames = {map_topic: "map", local_topic: local_frame}
        self.gps_max_age_ns = int(gps_max_age_sec * 1e9)
        self.pair_tolerance_ns = int(pair_tolerance_ms * 1e6)
        self.records = collections.defaultdict(list)

    def add(self, record):
        self.records[record["topic"]].append(record)

    def report(self):
        topics = {}
        all_input_stamps = sorted(record["stamp_ns"] for record in self.records.get("/velodyne_points", [])
                                  if record["stamp_ns"] > 0)
        for topic, records in sorted(self.records.items()):
            arrivals = [record["bag_ns"] for record in records]
            stamps = [record["stamp_ns"] for record in records if record["stamp_ns"] > 0]
            intervals = [(right - left) / 1e6 for left, right in zip(stamps, stamps[1:])]
            duration = (max(arrivals) - min(arrivals)) / 1e9 if arrivals else 0.0
            item = {"messages": len(records), "bag_duration_sec": duration,
                    "bag_rate_hz": (len(records) - 1) / duration if duration > 0 else None,
                    "frames": dict(collections.Counter(record["frame"] for record in records)),
                    "zero_or_missing_stamp_count": len(records) - len(stamps),
                    "stamp_regressions": sum(value < 0 for value in intervals),
                    "duplicate_stamp_count": sum(value == 0 for value in intervals),
                    "header_interval_ms": distribution(intervals),
                    "bag_minus_header_ms": distribution([(record["bag_ns"] - record["stamp_ns"]) / 1e6
                                                          for record in records if record["stamp_ns"] > 0])}
            expected = self.expected_frames.get(topic)
            if expected:
                item["expected_frame"] = expected
                item["unexpected_frame_messages"] = sum(record["frame"] != expected for record in records)
            gps_samples = [record for record in records if "gps_valid" in record]
            if gps_samples:
                item["raw_gps_validity"] = {"samples": len(gps_samples),
                                            "valid_samples": sum(record["gps_valid"] for record in gps_samples)}
            clouds = [record["cloud"] for record in records if "cloud" in record]
            if clouds:
                item["cloud"] = {"point_count": distribution([cloud["points"] for cloud in clouds]),
                                 "schemas": sorted(set(json.dumps(cloud["schema"]) for cloud in clouds)),
                                 **{key: sum(cloud[key] for cloud in clouds) for key in (
                                     "sampled_points", "sampled_nonfinite_xyz", "sampled_near_origin_xyz",
                                     "malformed_layout")}}
            arrays = [record for record in records if "objects" in record]
            if arrays:
                item["tracking"] = self._tracking_summary(arrays)
                if all_input_stamps:
                    offsets = [(stamp - all_input_stamps[nearest_index(all_input_stamps, stamp)]) / 1e6
                               for stamp in stamps]
                    item["output_header_minus_nearest_input_header_ms"] = distribution(offsets)
                    item["output_header_matches_input_count"] = sum(abs(value) <= self.pair_tolerance_ns / 1e6
                                                                     for value in offsets)
                    item["stamp_association_note"] = "Nearest timestamp evidence only; no causal scan association is asserted."
            ego_states = [record["ego"] for record in records if "ego" in record]
            if ego_states:
                finite_positions = [state["position"] for state in ego_states
                                    if all(math.isfinite(value) for value in state["position"])]
                item["ego_state"] = {
                    "speed_mps": distribution([math.sqrt(sum(value * value for value in state["velocity"]))
                                                for state in ego_states]),
                    "position_axis_span_m": [max(position[axis] for position in finite_positions) -
                                              min(position[axis] for position in finite_positions)
                                              for axis in range(3)] if finite_positions else None,
                    "note": "Ego status evidence only; zero positions in GPS-denied scenes may not be ground truth.",
                }
            diagnostic_records = [record["diagnostics"] for record in records if "diagnostics" in record]
            if diagnostic_records:
                statuses = collections.defaultdict(list)
                for diagnostic_array in diagnostic_records:
                    for status in diagnostic_array:
                        statuses[status["name"]].append(status)
                item["diagnostics"] = {}
                for name, values in sorted(statuses.items()):
                    numeric = collections.defaultdict(list)
                    categorical = collections.defaultdict(collections.Counter)
                    for status in values:
                        for key, raw in status["values"].items():
                            try:
                                number = float(raw)
                            except ValueError:
                                categorical[key][raw] += 1
                            else:
                                if math.isfinite(number):
                                    numeric[key].append(number)
                                else:
                                    categorical[key]["nonfinite:" + raw] += 1
                    item["diagnostics"][name] = {
                        "messages": len(values),
                        "levels": dict(collections.Counter(status["level"] for status in values)),
                        "messages_by_text": dict(collections.Counter(status["message"] for status in values)),
                        "numeric": {key: distribution(samples) for key, samples in sorted(numeric.items())},
                        "categorical": {key: dict(sorted(counts.items())) for key, counts in sorted(categorical.items())},
                    }
                    conditions = collections.defaultdict(lambda: collections.defaultdict(list))
                    condition_counts = collections.Counter()
                    for status in values:
                        availability = status["values"].get("protection_available")
                        if availability is None:
                            continue
                        condition_counts[availability] += 1
                        for key in ("kept_points", "clusters", "wall_left_removed", "wall_right_removed",
                                    "time_frame_total_ms", "yolo_offset_ms"):
                            raw = status["values"].get(key)
                            if raw is not None:
                                try:
                                    conditions[availability][key].append(float(raw))
                                except ValueError:
                                    pass
                    if condition_counts:
                        item["diagnostics"][name]["conditioned_on_protection_available"] = {
                            value: {"messages": count, "numeric": {key: distribution(samples)
                                    for key, samples in sorted(conditions[value].items())}}
                            for value, count in sorted(condition_counts.items())}
            topics[topic] = item
        return {"schema_version": "asmc-lidar-tracking-audit-1", "topics": topics,
                "handoff": self._handoff_summary(),
                "interpretation": [
                    "Empty arrays mean no published objects; they do not prove free space or sensor validity.",
                    "ID reappearances are continuity observations, not ground-truth ID switches.",
                    "Bag minus header is end-to-end clock offset; it is latency only after clock provenance is verified.",
                    "GPS validity is inferred from finite nonzero latitude/longitude, not the producer's tunnel hysteresis state.",
                    "velodyne is the sensor frame, not base_link; transform using the calibrated mount before camera/HD-map fusion.",
                ]}

    @staticmethod
    def _tracking_summary(records):
        last = {}
        reappearances = duplicate_ids = bad_geometry = speed_mismatches = 0
        counts = []
        for frame_index, record in enumerate(records):
            ids = [obj["id"] for obj in record["objects"]]
            counts.append(len(ids))
            duplicate_ids += len(ids) - len(set(ids))
            for obj in record["objects"]:
                key = (record["frame"], obj["id"])
                if key in last and last[key] + 1 < frame_index:
                    reappearances += 1
                last[key] = frame_index
                numbers = obj.get("geometry", [])
                bad_geometry += int(not all(math.isfinite(value) for value in numbers))
                velocity = obj.get("velocity_mps", [])
                speed = obj.get("speed_kph")
                if velocity and speed is not None:
                    speed_mismatches += int(not math.isfinite(speed) or
                                            not all(math.isfinite(value) for value in velocity) or
                                            abs(math.sqrt(sum(value * value for value in velocity)) * 3.6 - speed) > .2)
        return {"object_count": distribution(counts), "empty_arrays": sum(value == 0 for value in counts),
                "unique_frame_id_pairs": len(last), "duplicate_ids_within_array": duplicate_ids,
                "id_reappearances_after_absence": reappearances, "nonfinite_geometry_objects": bad_geometry,
                "speed_kph_vs_velocity_norm_mismatch_objects": speed_mismatches}

    def _handoff_summary(self):
        map_records = [record for record in self.records.get(self.map_topic, []) if "objects" in record]
        local_records = [record for record in self.records.get(self.local_topic, []) if "objects" in record]
        # Pair by source/output header, never by bag receipt order across streams.
        local_records = sorted((record for record in local_records if record["stamp_ns"] > 0),
                               key=lambda record: record["stamp_ns"])
        local_stamps = [record["stamp_ns"] for record in local_records]
        pairs = []
        used = set()
        for record in map_records:
            if not local_stamps or record["stamp_ns"] <= 0:
                continue
            index = nearest_index(local_stamps, record["stamp_ns"])
            if index in used or abs(local_stamps[index] - record["stamp_ns"]) > self.pair_tolerance_ns:
                continue
            used.add(index)
            pairs.append((record, local_records[index]))
        both_nonempty = both_empty = 0
        stream_switches = []
        previous = None
        for map_record, local_record in pairs:
            map_ids = {obj["id"] for obj in map_record["objects"]}
            local_ids = {obj["id"] for obj in local_record["objects"]}
            both_nonempty += int(bool(map_ids) and bool(local_ids))
            both_empty += int(not map_ids and not local_ids)
            if bool(map_ids) == bool(local_ids):
                continue
            stream = self.map_topic if map_ids else self.local_topic
            ids = map_ids or local_ids
            if previous and stream != previous[0]:
                stream_switches.append({"from": previous[0], "to": stream,
                                        "stamp_ns": map_record["stamp_ns"],
                                        "elapsed_since_last_nonempty_ms": (map_record["stamp_ns"] - previous[1]) / 1e6,
                                        "shared_ids": sorted(ids & previous[2])})
            previous = (stream, map_record["stamp_ns"], ids)
        gps_records = sorted(self.records.get("/gps", []), key=lambda record: record["bag_ns"])
        gps_times = [record["bag_ns"] for record in gps_records]
        gps_transitions = []
        for left, right in zip(gps_records, gps_records[1:]):
            if left.get("gps_valid") != right.get("gps_valid"):
                gps_transitions.append({"bag_ns": right["bag_ns"], "valid": right.get("gps_valid")})
        observations = collections.Counter()
        for map_record, local_record in pairs:
            time = min(map_record["bag_ns"], local_record["bag_ns"])
            index = bisect.bisect_right(gps_times, time) - 1
            gps_state = "unknown_or_stale"
            if index >= 0 and time - gps_times[index] <= self.gps_max_age_ns:
                gps_state = "raw_valid" if gps_records[index].get("gps_valid") else "raw_invalid"
            state = "both_empty" if not map_record["objects"] and not local_record["objects"] else (
                "both_nonempty" if map_record["objects"] and local_record["objects"] else (
                    "map_nonempty" if map_record["objects"] else "local_nonempty"))
            observations[gps_state + "/" + state] += 1
        return {"map_topic": self.map_topic, "local_topic": self.local_topic,
                "paired_arrays": len(pairs), "unpaired_map_arrays": len(map_records) - len(pairs),
                "unpaired_local_arrays": len(local_records) - len(pairs),
                "both_nonempty_pairs": both_nonempty, "both_empty_pairs": both_empty,
                "observed_nonempty_stream_switches": stream_switches,
                "gps_sample_transitions": gps_transitions,
                "gps_vs_output_observations": dict(sorted(observations.items()))}


def normalize(topic, message, bag_ns, max_sample_points):
    header = getattr(message, "header", None)
    record = {"topic": topic, "bag_ns": bag_ns,
              "stamp_ns": header.stamp.to_nsec() if header is not None else 0,
              "frame": header.frame_id if header is not None else ""}
    if hasattr(message, "point_step"):
        record["cloud"] = cloud_summary(message, max_sample_points)
    if hasattr(message, "objects"):
        record["objects"] = []
        for obj in message.objects:
            geometry = [float(getattr(point, axis)) for point in
                        [obj.cluster_center, obj.obb_center] + list(obj.obb_corners) for axis in ("x", "y", "z")]
            geometry.extend(float(getattr(obj, name)) for name in ("length", "width", "height", "obb_yaw"))
            record["objects"].append({"id": int(obj.id), "geometry": geometry,
                                      "velocity_mps": [float(getattr(obj.velocity_mps, axis)) for axis in ("x", "y", "z")],
                                      "speed_kph": float(obj.speed_kph)})
    if topic == "/gps":
        coordinates = [getattr(message, name, float("nan")) for name in ("latitude", "longitude")]
        record["gps_valid"] = all(math.isfinite(value) and abs(value) > 1e-9 for value in coordinates)
    if topic == "/Ego_topic":
        record["ego"] = {key: [float(getattr(getattr(message, key), axis)) for axis in ("x", "y", "z")]
                         for key in ("position", "velocity")}
    if hasattr(message, "status") and isinstance(message.status, (list, tuple)):
        record["diagnostics"] = [{"name": status.name, "level": int(status.level), "message": status.message,
                                   "values": {value.key: value.value for value in status.values}}
                                  for status in message.status]
    return record


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", type=Path, nargs="+", help="Closed ROS1 .bag files in chronological order")
    parser.add_argument("--output", type=Path, required=True, help="JSON report destination (must not already exist)")
    parser.add_argument("--map-topic", default="/last_lidar")
    parser.add_argument("--local-topic", default="/lidar_nogps")
    parser.add_argument("--expect-local-frame", default="velodyne", help="Current producer uses velodyne, not base_link")
    parser.add_argument("--cloud-topic", action="append", help="Override default cloud topic list; repeatable")
    parser.add_argument("--sample-points-per-cloud", type=int, default=1000)
    parser.add_argument("--gps-max-age-sec", type=float, default=.5)
    parser.add_argument("--pair-tolerance-ms", type=float, default=2.0)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error("output exists; choose a new experiment report path")
    if args.sample_points_per_cloud < 0 or args.gps_max_age_sec <= 0 or args.pair_tolerance_ms < 0:
        parser.error("sample count/tolerance must be nonnegative; GPS age must be positive")
    try:
        import rosbag
    except ImportError:
        parser.error("run in a ROS Noetic container after sourcing /opt/ros/noetic/setup.bash")
    audit = TrackingAudit(args.map_topic, args.local_topic, args.expect_local_frame,
                          args.gps_max_age_sec, args.pair_tolerance_ms)
    topics = list(args.cloud_topic or DEFAULT_CLOUDS) + [args.map_topic, args.local_topic,
                                                       "/lidar/tracked_objects_map", "/gps", "/imu", "/Ego_topic",
                                                       "/YOLO/BBox", "/camera/image/compressed"] + list(DIAGNOSTIC_TOPICS)
    bag_inputs = []
    for path in args.bags:
        with rosbag.Bag(str(path), "r") as bag:
            bag_inputs.append({"path": str(path), "size_bytes": path.stat().st_size})
            for topic, message, time in bag.read_messages(topics=topics):
                audit.add(normalize(topic, message, time.to_nsec(), args.sample_points_per_cloud))
    result = audit.report()
    result["bag_inputs"] = bag_inputs
    result["settings"] = {"expected_local_frame": args.expect_local_frame,
                          "gps_max_age_sec": args.gps_max_age_sec,
                          "pair_tolerance_ms": args.pair_tolerance_ms,
                          "max_sample_points_per_cloud": args.sample_points_per_cloud}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"report": str(args.output), "topics": len(result["topics"]),
                      "handoff": result["handoff"]}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
