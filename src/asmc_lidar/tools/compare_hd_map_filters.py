#!/usr/bin/env python3
"""A1 offline audit only: reads trace/fixture JSONL, never publishes or removes objects.

The v6 vector product owns geometry; its SHA-checked derived grid is used for
bounded raster footprint comparison. Decisions are not an operational filter.
Missing pose accuracy/mission-scope evidence prevents exclusion candidates.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import html
import json
import math
from pathlib import Path
import subprocess
import time

import cv2
import numpy as np
import yaml

SCHEMA = "asmc.lidar.hd_map_filter_audit/v1"
EXCEPTION_TYPES = ("static_obstacle", "dynamic_obstacle", "roundabout")
REPO = Path(__file__).resolve().parents[3]


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=False, separators=(",", ":"), allow_nan=False)


def finite_vector(value, n):
    return isinstance(value, (list, tuple)) and len(value) == n and all(
        isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in value)


def footprint(obj):
    """All eight corners matter for rolled/pitched boxes. Never rebuild from center/yaw."""
    points = obj.get("corners")
    if not isinstance(points, list) or len(points) != 8 or not all(finite_vector(p, 3) for p in points):
        raise ValueError("eight_finite_corners_required")
    xy = np.asarray(points, dtype=np.float64)[:, :2]
    if np.max(np.abs(xy)) > 1e7:
        raise ValueError("coordinate_range_invalid")
    hull = cv2.convexHull(xy.astype(np.float32)).reshape(-1, 2).astype(np.float64)
    if len(hull) < 3 or cv2.contourArea(hull.astype(np.float32)) < 1e-6:
        raise ValueError("degenerate_footprint")
    return hull


class MapAudit:
    def __init__(self, product, grid, metadata, source, legacy_image=None, legacy_metadata=None):
        self.product = json.loads(Path(product).read_text())
        self.metadata = yaml.safe_load(Path(metadata).read_text())
        self.source = yaml.safe_load(Path(source).read_text())
        self.paths = {"product": str(product), "grid": str(grid), "metadata": str(metadata), "source": str(source)}
        self.shas = {k: sha256(v) for k, v in self.paths.items()}
        if self.shas["grid"] != self.product["grid"]["sha256"]:
            raise ValueError("derived_grid_sha_mismatch")
        with np.load(grid, allow_pickle=False) as bundle:
            self.occupancy = bundle["occupancy"].copy()
            self.resolution = float(bundle["resolution_m"])
            self.origin = np.asarray(bundle["origin_xy"], dtype=float)
            frame = str(bundle["frame_id"].reshape(-1)[0])
        spec = self.product["grid"]
        if frame != "map" or self.product["frame_id"] != frame or self.metadata["frame_id"] != frame:
            raise ValueError("map_frame_mismatch")
        if self.occupancy.shape != (spec["height"], spec["width"]) or self.resolution != spec["resolution_m"]:
            raise ValueError("grid_shape_or_resolution_mismatch")
        if not np.allclose(self.origin, [spec["origin_x"], spec["origin_y"]], rtol=0, atol=1e-9):
            raise ValueError("grid_origin_mismatch")
        if not np.isin(self.occupancy, [-1, 0, 100]).all():
            raise ValueError("grid_values_invalid")
        # Protect mission approaches spatially, including reviewed off-route maneuvers.
        self.exception_links = {}
        self.exceptions = np.zeros(self.occupancy.shape, np.uint8)
        self.mission_audit = []
        groups = self.product["selection"]["intersection_groups"]
        for i, kind in enumerate(EXCEPTION_TYPES, 1):
            zones = [z for z in self.metadata["mission_zones"] if z["mission_type"] == kind]
            if not zones:
                raise ValueError("mission_definition_missing:" + kind)
            links = {v for z in zones for v in z["link_ids"]}
            links.update(groups.get(kind + "_maneuver", []))
            if kind == "dynamic_obstacle":
                closure = self.product["selection"].get("intersection_node_pair_closures", {})
                entry = closure.get("dynamic_obstacle_maneuver", {})
                links.update(entry.get("resolved_link_ids", []))
            if kind == "roundabout":
                links.add("A2256W000134")
            self.exception_links[kind] = sorted(links)
            covered = set()
            for area in self.product["drivable_areas"]:
                if links.intersection(area.get("link_ids", [])):
                    poly = np.rint((np.asarray(area["polygon"])[:, :2] - self.origin) / self.resolution).astype(np.int32)
                    cv2.fillPoly(self.exceptions, [poly], int(i))
                    covered.update(links.intersection(area.get("link_ids", [])))
            self.mission_audit.append({"type": kind, "metadata_links": sorted({v for z in zones for v in z["link_ids"]}),
                                       "protected_links": sorted(links), "links_without_area": sorted(links - covered),
                                       "metadata_polygon_empty": any(not z.get("polygon") for z in zones),
                                       "external_approach_review": "unverified"})
        self.legacy = None
        if legacy_image and legacy_metadata:
            lm = yaml.safe_load(Path(legacy_metadata).read_text())
            image = cv2.imread(str(legacy_image), cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise ValueError("legacy_mask_missing")
            if lm["map_id"] != self.product["map_id"]:
                raise ValueError("legacy_map_id_mismatch")
            self.legacy = (image, lm)
            self.paths.update(legacy_image=str(legacy_image), legacy_metadata=str(legacy_metadata))
            self.shas.update({k: sha256(v) for k, v in self.paths.items() if k.startswith("legacy_")})

    def relation(self, hull, expansion_m=0.0):
        """Conservative native-cell relation. Ratios are raster diagnostics, not exact vector area.

        Expanded window covers pose uncertainty, a candidate guard and cell discretization.
        Any road in that window protects the object, deliberately over-preserving corners.
        """
        if not math.isfinite(expansion_m) or not 0 <= expansion_m <= 20:
            return {"available": False, "reason": "uncertainty_window_out_of_range"}
        px = (hull - self.origin) / self.resolution
        lo = np.floor(px.min(axis=0) - 1).astype(int)
        hi = np.ceil(px.max(axis=0) + 1).astype(int) + 1
        padding = int(math.ceil(expansion_m / self.resolution)) + 1
        outer_lo, outer_hi = lo - padding, hi + padding
        h, w = self.occupancy.shape
        if np.any(outer_lo < 0) or outer_hi[0] > w or outer_hi[1] > h:
            return {"available": False, "reason": "map_extent_or_uncertainty_outside"}
        if np.prod(outer_hi - outer_lo) > 1_000_000:
            return {"available": False, "reason": "footprint_window_too_large"}
        patch = self.occupancy[lo[1]:hi[1], lo[0]:hi[0]]
        mask = np.zeros(patch.shape, np.uint8)
        cv2.fillConvexPoly(mask, np.rint(px - lo).astype(np.int32), 1)
        selected = patch[mask != 0]
        if selected.size == 0:
            return {"available": False, "reason": "empty_raster_footprint"}
        outer = self.occupancy[outer_lo[1]:outer_hi[1], outer_lo[0]:outer_hi[0]]
        exception = self.exceptions[outer_lo[1]:outer_hi[1], outer_lo[0]:outer_hi[0]]
        return {"available": True, "road_ratio_grid": float(np.mean(selected == 0)),
                "boundary_protected": bool(np.any(outer == 0)),
                "unknown_cells": bool(np.any(outer == -1)),
                "exception_types": [EXCEPTION_TYPES[i - 1] for i in sorted(set(exception.flat)) if i],
                "raster_resolution_m": self.resolution}

    def evaluate(self, frame, obj, config):
        start = time.perf_counter_ns()
        result = {"decision": "undetermined", "reason": "uninitialized", "spatial": None,
                  "input_state": obj.get("observation_state", "unknown")}
        def finish(decision, reason):
            result.update(decision=decision, reason=reason, comparison_ms=(time.perf_counter_ns() - start) / 1e6)
            return result
        if frame.get("frame_id") != "map" or frame.get("status") not in (1, 2):
            return finish("undetermined", "map_frame_or_array_status_unavailable")
        pose = frame.get("pose", {})
        if pose.get("valid") is not True:
            return finish("undetermined", "pose_invalid_or_missing")
        if frame.get("map_id") != self.product["map_id"]:
            return finish("undetermined", "map_id_unverified")
        if not finite_vector(frame.get("map_origin"), 3) or not np.allclose(frame["map_origin"], config["map_origin"], rtol=0, atol=1e-6):
            return finish("undetermined", "map_origin_unverified")
        stamp = frame.get("stamp_ns")
        if type(stamp) is not int or stamp <= 0 or pose.get("at_stamp_ns") != stamp:
            return finish("undetermined", "pose_scan_stamp_mismatch")
        try:
            hull = footprint(obj)
        except ValueError as error:
            return finish("undetermined", str(error))
        result["footprint_xy"] = hull.tolist()
        bound = pose.get("position_error_bound_m")
        verified = pose.get("position_error_verified") is True and isinstance(bound, (int, float)) and not isinstance(bound, bool) and math.isfinite(bound) and bound >= 0
        expansion = config["candidate_guard_m"] + (bound if verified else 0)
        spatial = self.relation(hull, expansion)
        result["spatial"] = spatial
        if not spatial["available"]:
            return finish("undetermined", spatial["reason"])
        # A generous audit-only exception envelope; not a claim of complete coverage.
        exceptions = self.relation(hull, max(expansion, config["exception_protection_m"]))
        if not exceptions["available"]:
            return finish("undetermined", exceptions["reason"])
        if exceptions["exception_types"]:
            result["exception_types"] = exceptions["exception_types"]
            return finish("keep", "exception_space_protected")
        if spatial["unknown_cells"]:
            return finish("undetermined", "unknown_map_cells")
        if spatial["road_ratio_grid"] > 0:
            return finish("keep", "drivable_overlap")
        if spatial["boundary_protected"]:
            return finish("keep", "boundary_or_uncertainty_protected")
        if not verified:
            return finish("undetermined", "pose_error_bound_unverified")
        age = pose.get("source_age_ms")
        if not isinstance(age, (int, float)) or isinstance(age, bool) or not math.isfinite(age) or not 0 <= age <= config["pose_age_limit_ms"]:
            return finish("undetermined", "pose_source_age_unverified_or_stale")
        if frame.get("mission_scope_verified") is not True:
            return finish("undetermined", "mission_scope_unverified")
        # Candidate only: no object/track mutation and no publishing anywhere.
        return finish("exclude_candidate", "ordinary_space_clearly_outside")

    def compare_maps(self):
        road = self.occupancy == 0
        report = {"v6_shape": list(road.shape), "v6_road_cells": int(road.sum()),
                  "v6_road_area_m2": float(road.sum() * self.resolution ** 2), "resolution_m": self.resolution,
                  "product_build_id": self.product["build_id"], "shas": self.shas,
                  "missions": self.mission_audit, "geometry_authority": "vector; grid ratios are diagnostic approximation"}
        if self.legacy:
            old, meta = self.legacy
            same = old.shape == road.shape and abs(1 / meta["pixels_per_meter"] - self.resolution) < 1e-9 and np.allclose(meta["world_offset"], self.origin)
            report.update(legacy_shape=list(old.shape), aligned_grid=same, legacy_white_cells=int(np.count_nonzero(old)))
            if same:
                before = old > 127
                report.update(both_drivable=int(np.count_nonzero(before & road)),
                              legacy_only=int(np.count_nonzero(before & ~road)), v6_only=int(np.count_nonzero(~before & road)))
        return report


def normalize(record, source_id, line_number, config):
    if record.get("schema") != "asmc_lidar_scan_trace_v1":
        if record.get("schema") != SCHEMA:
            raise ValueError("unsupported_input_schema")
        return record
    typed = record.get("typed", {})
    mapped, local = typed.get("map", {}), typed.get("local", {})
    valid = mapped.get("accepted") is True and mapped.get("status") in (1, 2) and record.get("map", {}).get("valid") is True
    chosen = mapped if valid else local
    stamps = record.get("map", {}).get("gps_stamps", [])
    matrix = record.get("map", {}).get("map_from_base", [])
    stamp = record["stamp_ns"]
    tracks = {t["id"]: t for t in record.get("tracks", [])}
    objects = []
    for original in chosen.get("objects", []):
        obj = dict(original)  # Never put inferred validity into the source object.
        tr = tracks.get(obj["track_id"], {})
        obj["observation_state"] = "predicted" if tr.get("miss", 0) else "observed"
        obj["observation_state_provenance"] = "trace_track_miss_count"
        objects.append(obj)
    return {"schema": SCHEMA, "source_id": source_id, "line_number": line_number,
            "producer_session_id": record.get("producer_session_id"), "generation": record.get("generation"),
            "scan_sequence": record.get("seq"), "stamp_ns": stamp, "arrival_ns": record.get("arrival_ns"),
            "frame_id": "map" if valid else chosen.get("frame", "base_link"), "status": chosen.get("status"),
            # Current replay has no map ID/origin or validated positional uncertainty.
            # Expected map frame comes from explicit audit configuration, NOT the trace itself.
            "map_id": config["expected_map_id"], "map_origin": config["map_origin"],
            "map_identity_provenance": "audit_configuration_not_recorded_in_trace",
            "pose": {"valid": valid, "at_stamp_ns": stamp, "method": record.get("map", {}).get("reason"),
                     "gps_stamps": stamps, "source_age_ms": (stamp - max(stamps)) / 1e6 if stamps else None,
                     "position_error_verified": False, "position_error_bound_m": None},
            "ego": {"x": matrix[3], "y": matrix[7], "speed_mps": record.get("local", {}).get("speed_mps"), "length_m": 4.635} if len(matrix) == 16 else None,
            "mission_scope_verified": False, "objects": objects, "source_kind": "recorded_replay_no_independent_gt"}


class LegacyProbe:
    def __init__(self, binary, repo, main=False):
        self.main = main
        root = repo / "src/planning_inputs/config"
        catalog = repo / "src/lidar+sensorfusion/src/Main/known_static_objects.yaml" if main else root / "validated_0781402/known_static_objects.yaml"
        args = [str(binary), str(root / "validated_0781402/road_mask.png"), str(root / "validated_0781402/road_mask.yaml"),
                str(catalog), str(root / "static_structure_exclusions.yaml"), str(root / "object_filter_params.yaml"), "waize_katri_2025-10-23"]
        self.inputs = {str(p): sha256(p) for p in [*args[1:6], binary]}
        self.process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)

    def evaluate(self, key, frame, obj):
        fields = ("cluster_center", "legacy_velocity_mps", "legacy_is_dynamic", "legacy_motion_state")
        complete = frame.get("frame_id") == "map" and all(k in obj for k in fields)
        try:
            footprint(obj)
        except ValueError:
            complete = False
        message = {"key": key, "frame_key": frame["audit_frame_key"],
                   "object": obj, "baseline_complete": complete,
                   "relevance_complete": complete and bool(frame.get("ego")) and "ground_evidence" in obj,
                   "ego": frame.get("ego")}
        # Missing recorded motion is NEVER assumed to mean static. A separate
        # labeled counterfactual geometry probe makes map differences inspectable.
        scope = "recorded_legacy_fields" if complete else "unavailable"
        if not complete and frame.get("frame_id") == "map":
            try:
                footprint(obj)
                probe = dict(obj, cluster_center=obj["center"], legacy_velocity_mps=[0, 0, 0],
                             legacy_is_dynamic=False, legacy_motion_state=0)
                message.update(object=probe, baseline_complete=True, relevance_complete=False)
                scope = "counterfactual_static_probe_not_recorded_decision"
            except (ValueError, KeyError):
                pass
        self.process.stdin.write(canonical(message) + "\n")
        self.process.stdin.flush()
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError("legacy_probe_failed")
        result = json.loads(line)
        if result["key"] != key:
            raise RuntimeError("legacy_probe_key_mismatch")
        result["scope"] = scope
        return result

    def close(self):
        self.process.stdin.close()
        status = self.process.wait(timeout=30)
        if status:
            raise RuntimeError("legacy_probe_exit:" + str(status))


def quantiles(values):
    return {k: float(np.percentile(values, p)) for k, p in (("p50", 50), ("p95", 95), ("p99", 99))} if values else {}


def write_gallery(directory, cases, audit):
    """Small static SVGs from immutable OBBs; no extra fitting or JPEG processing."""
    cards = []
    colors = {"keep": "#22863a", "exclude_candidate": "#c62828", "undetermined": "#946200"}
    for i, row in enumerate(cases):
        obj, decision = row["object"], row["candidate"]
        try:
            hull = footprint(obj)
        except ValueError:
            continue
        center = hull.mean(axis=0); span = 50.0
        def points(poly):
            return " ".join("%.2f,%.2f" % ((p[0] - center[0] + span / 2) * 10, (center[1] + span / 2 - p[1]) * 10) for p in poly)
        content = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 500 500"><rect width="500" height="500" fill="#f4f4f4"/>']
        if row["frame_id"] == "map":
            for area in audit.product["drivable_areas"]:
                poly = np.asarray(area["polygon"])[:, :2]
                if np.all(poly.max(axis=0) >= center - span / 2) and np.all(poly.min(axis=0) <= center + span / 2):
                    content.append('<polygon points="%s" fill="#cbd8e5" stroke="#687e91" stroke-width="0.7"/>' % points(poly))
        for profile, old in row["legacy"].items():
            if old.get("geometry_clipped") and old.get("output_corners"):
                outline = cv2.convexHull(np.ascontiguousarray(np.asarray(old["output_corners"], dtype=np.float32)[:, :2])).reshape(-1, 2)
                content.append('<polygon points="%s" fill="none" stroke="#2563eb" stroke-width="2" stroke-dasharray="5 4"/>' % points(outline))
        content.append('<polygon points="%s" fill="none" stroke="%s" stroke-width="3"/>' % (points(hull), colors[decision["decision"]]))
        content.append('<text x="10" y="20" font-size="12">map +X right, +Y up; 50m window</text></svg>')
        name = "case-%03d.svg" % i
        (directory / name).write_text("".join(content))
        text = {"source": row["source_id"], "key": row["key"], "stamp_ns": row["stamp_ns"],
                "track_id": obj.get("track_id"), "decision": decision, "legacy": row["legacy"],
                "source_kind": row["source_kind"], "pose": row["pose"],
                "runtime_audit": row.get("runtime_audit"), "runtime_audit_identity": row.get("runtime_audit_identity")}
        cards.append('<article><img src="%s"/><pre>%s</pre></article>' % (name, html.escape(json.dumps(text, ensure_ascii=False, indent=2))))
    (directory / "index.html").write_text('<!doctype html><meta charset="utf-8"><title>A1 HD map filter audit</title><style>body{font:14px sans-serif}article{display:flex;border-top:1px solid #aaa}img{width:500px}pre{white-space:pre-wrap;max-width:850px}</style><h1>A1 OBB filter audit — offline / no deletion</h1><p>Blue: v6 vector. Original projected eight-corner footprint. No independent GT; local-only cases have no map overlay.</p>' + "".join(cards))


def run(args):
    output = Path(args.output)
    if output.exists():
        raise ValueError("output_exists_choose_new_run")
    config = yaml.safe_load(Path(args.config).read_text())
    audit = MapAudit(*(REPO / config[k] for k in ("product", "grid", "metadata", "source")),
                     REPO / config["legacy_image"], REPO / config["legacy_metadata"])
    revision = subprocess.check_output(["git", "-C", str(REPO), "rev-parse", "HEAD"], text=True).strip()
    dirty = bool(subprocess.check_output(["git", "-C", str(REPO), "status", "--porcelain"], text=True).strip())
    if config["expected_map_id"] != audit.product["map_id"]:
        raise ValueError("configured_map_id_mismatch")
    output.mkdir(parents=True)
    probes = {name: LegacyProbe(Path(args.legacy_bin_dir) / ("legacy_" + name + "_filter_probe"), REPO, name == "main") for name in ("main", "planning")} if args.legacy_bin_dir else {}
    counts, strata, reasons, scope, states, discrepancies = Counter(), Counter(), Counter(), Counter(), Counter(), Counter()
    times, scan_times, gallery, inputs = [], [], {}, []
    hashes = hashlib.sha256(); input_hashes = hashlib.sha256()
    failures = []; seen = set()
    try:
        with (output / "decisions.jsonl").open("w") as dest:
            for source in args.input:
                source = Path(source); source_sha = sha256(source)
                inputs.append({"path": str(source), "sha256": source_sha})
                for line_number, line in enumerate(source.open(), 1):
                    original = json.loads(line); frame = normalize(original, source.name, line_number, config)
                    before = canonical(frame["objects"])
                    if frame.get("expected_decision") and len(frame["objects"]) != 1:
                        raise ValueError("expected_decision_requires_one_object_fixture")
                    frame_start = time.perf_counter_ns(); counts["scans"] += 1
                    if not frame["objects"]:
                        states["empty_or_invalid_scan:" + str(frame.get("status"))] += 1
                    identity = [source_sha, line_number, frame.get("producer_session_id"), frame.get("generation"), frame.get("scan_sequence")]
                    frame_key = canonical(identity)
                    frame = dict(frame, audit_frame_key=frame_key)
                    if frame_key in seen:
                        raise ValueError("duplicate_frame_identity")
                    seen.add(frame_key)
                    for index, obj in enumerate(frame["objects"]):
                        key = hashlib.sha256((frame_key + ":" + str(index)).encode()).hexdigest()
                        candidate = audit.evaluate(frame, obj, config)
                        runtime_audit = original.get("hd_map_audit")
                        runtime_decision = None
                        if runtime_audit and runtime_audit.get("available"):
                            matches = [d for d in runtime_audit.get("objects", []) if d["track_id"] == obj["track_id"]]
                            if len(matches) != 1:
                                raise ValueError("runtime_audit_object_identity_mismatch")
                            runtime_decision = matches[0]
                            for field in ("decision", "reason", "spatial"):
                                key_name="obb_reference_"+field if runtime_decision.get("preview") and field!="spatial" else field
                                if runtime_decision[key_name] != candidate[field]:
                                    raise ValueError("runtime_python_audit_mismatch:" + field)
                            if runtime_decision.get("preview"):
                                candidate=dict(runtime_decision)  # effective point/height preview, retains OBB reference

                        legacy = {name: probe.evaluate(key, frame, obj) for name, probe in probes.items()}
                        row = {"schema": SCHEMA, "key": key, "run_id": args.run_id, "source_id": source.name,
                               "source_sha256": source_sha, "source_kind": frame.get("source_kind", "synthetic_fixture"),
                               "frame_identity": identity, "stamp_ns": frame.get("stamp_ns"), "arrival_ns": frame.get("arrival_ns"),
                               "frame_id": frame.get("frame_id"), "pose": frame.get("pose"), "object": obj,
                               "candidate": candidate, "legacy": legacy, "map_build_id": audit.product["build_id"],
                               "required_object": obj.get("required_object"), "review_provenance": obj.get("review_provenance")}
                        if runtime_decision is not None:
                            row["runtime_audit"] = runtime_decision
                            row["runtime_audit_identity"] = {k: runtime_audit[k] for k in
                                ("map_sha256", "bundle_sha256", "policy_sha256")}
                        counts[candidate["decision"]] += 1; reasons[candidate["reason"]] += 1
                        strata[row["source_kind"] + ":" + candidate["decision"]] += 1
                        states[str(obj.get("observation_state", "unknown"))] += 1
                        times.append(candidate["comparison_ms"])
                        for name, old in legacy.items():
                            scope[name + ":" + old["scope"]] += 1
                            if old.get("available"):
                                discrepancies[name + ":" + str(old["keep"]) + ":" + candidate["decision"]] += 1
                        if obj.get("required_object") is True and candidate["decision"] == "exclude_candidate":
                            failures.append({"key": key, "reason": "required_object_exclusion", "source_kind": row["source_kind"]})
                        if frame.get("expected_decision") and frame["expected_decision"] != candidate["decision"]:
                            failures.append({"key": key, "reason": "fixture_expected_decision_mismatch", "case": frame.get("case")})
                        # Timings and the distinct run ID do not define numerical equivalence.
                        stable = json.loads(canonical(row)); stable.pop("run_id", None)
                        stable["candidate"].pop("comparison_ms", None)
                        if "runtime_audit" in stable:
                            stable["runtime_audit"].pop("comparison_ms", None)
                        for old in stable["legacy"].values():
                            old.pop("comparison_ms", None)
                        hashes.update((canonical(stable) + "\n").encode())
                        input_hashes.update((canonical(obj) + "\n").encode())
                        dest.write(canonical(row) + "\n")
                        # Include source, state and reason, not many repeated near-identical boxes.
                        group = (source.name, frame.get("case", candidate["reason"]), obj.get("observation_state"))
                        if group not in gallery and len(gallery) < 48:
                            gallery[group] = row
                    if canonical(frame["objects"]) != before:
                        raise RuntimeError("input_object_mutation")
                    scan_times.append((time.perf_counter_ns() - frame_start) / 1e6)
                if sha256(source) != source_sha:
                    raise RuntimeError("input_file_changed_during_audit")
    finally:
        for probe in probes.values():
            probe.close()
    for name, path in audit.paths.items():
        if sha256(path) != audit.shas[name]:
            raise RuntimeError("map_input_changed_during_audit:" + name)
    stats = {"schema": SCHEMA, "run_id": args.run_id, "source_revision": revision, "source_dirty": dirty,
             "legacy_input_shas": {name: probe.inputs for name, probe in probes.items()}, "counts": counts, "reasons": reasons, "strata": strata,
             "input_states": states, "legacy_scope": scope, "discrepancies": discrepancies,
             "candidate_object_ms": quantiles(times), "scan_total_with_probes_ms": quantiles(scan_times),
             "stable_decisions_sha256": hashes.hexdigest(), "input_objects_sha256": input_hashes.hexdigest(),
             "immutable_inputs_verified": True, "inputs": inputs, "config_sha256": sha256(args.config),
             "map_comparison": audit.compare_maps(), "failures": failures,
             "recommendation": "hold_runtime_adoption_pending_pose_error_and_exception_scope_validation",
             "operational_changes": False, "independent_gt_available": False}
    (output / "summary.json").write_text(json.dumps(stats, ensure_ascii=False, indent=2) + "\n")
    write_gallery(output, list(gallery.values()), audit)
    print(json.dumps({k: stats[k] for k in ("run_id", "counts", "candidate_object_ms", "stable_decisions_sha256", "failures")}, ensure_ascii=False))
    return 2 if failures else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--input", action="append", required=True)
    parser.add_argument("--legacy-bin-dir")
    parser.add_argument("--output", required=True, help="New registered experiment output, never an existing run")
    args = parser.parse_args()
    try:
        return run(args)
    except (ValueError, OSError, RuntimeError, cv2.error) as error:
        parser.exit(1, str(error) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
