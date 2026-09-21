"""Deterministic P2-A audit for lane/topology/drivable-area inputs.

The audit is intentionally read-only.  It records where lane-based clipping is
eligible for a later stage and where the legacy Node/Link-derived road layer
must be preserved.  It never joins paint gaps or writes map geometry.
"""

from __future__ import annotations

from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

import numpy as np
import yaml

from .runtime_bundle import geometry_sha256


class DrivableAreaAuditError(ValueError):
    """Raised when a P2-A input or invariant is not valid."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_yaml(path: Path) -> Mapping[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise DrivableAreaAuditError("YAML root must be a mapping: %s" % path)
    return value


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _source_paths(repo_root: Path, config: Mapping[str, Any]) -> Dict[str, Path]:
    result: Dict[str, Path] = {}
    for name, spec in (config.get("sources") or {}).items():
        path = (repo_root / str(spec["path"])).resolve()
        if not path.is_file():
            raise DrivableAreaAuditError("missing %s: %s" % (name, path))
        actual = sha256_file(path)
        expected = str(spec.get("sha256", ""))
        if actual != expected:
            raise DrivableAreaAuditError(
                "%s SHA mismatch: expected %s, got %s" % (name, expected, actual)
            )
        result[str(name)] = path
    return result


def _xy(raw: Sequence[Sequence[float]]) -> np.ndarray:
    points = np.asarray(raw, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] < 2 or points.shape[1] < 2:
        raise DrivableAreaAuditError("polyline must contain at least two XY points")
    if not np.isfinite(points[:, :2]).all():
        raise DrivableAreaAuditError("polyline contains non-finite coordinates")
    return points[:, :2]


def resample_polyline(points: np.ndarray, step_m: float) -> Tuple[np.ndarray, np.ndarray]:
    pts = _xy(points)
    segment = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    keep = segment > 1e-9
    if not keep.any():
        raise DrivableAreaAuditError("polyline length must be positive")
    pts = np.vstack([pts[0], pts[1:][keep]])
    segment = segment[keep]
    stations = np.concatenate([[0.0], np.cumsum(segment)])
    total = float(stations[-1])
    target = np.arange(0.0, total + 1e-9, float(step_m))
    if target.size == 0 or target[-1] < total - 1e-9:
        target = np.append(target, total)
    sampled = np.column_stack(
        [np.interp(target, stations, pts[:, axis]) for axis in range(2)]
    )
    return sampled, target


def bilateral_boundary_coverage(
    centerline: np.ndarray,
    marking_polylines: Sequence[np.ndarray],
    *,
    sample_step_m: float,
    marking_sample_step_m: float,
    minimum_lateral_distance_m: float,
    maximum_boundary_distance_m: float,
) -> Dict[str, Any]:
    """Measure whether reviewed paint exists on both sides of a link centerline.

    This is evidence for classification, not a lane-polygon builder.  Paint is
    considered only when the stage14 feature explicitly names the current link.
    """

    centers, stations = resample_polyline(centerline, sample_step_m)
    if not marking_polylines:
        return {
            "sample_count": int(len(centers)),
            "left_coverage": 0.0,
            "right_coverage": 0.0,
            "bilateral_coverage": 0.0,
            "left_median_distance_m": None,
            "right_median_distance_m": None,
        }
    samples = [resample_polyline(points, marking_sample_step_m)[0] for points in marking_polylines]
    markings = np.vstack(samples)
    tangent = np.gradient(centers, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    delta = markings[None, :, :] - centers[:, None, :]
    distance = np.linalg.norm(delta, axis=2)
    cross = tangent[:, None, 0] * delta[:, :, 1] - tangent[:, None, 1] * delta[:, :, 0]
    minimum = float(minimum_lateral_distance_m)
    limit = float(maximum_boundary_distance_m)
    left_distance = np.min(np.where(cross > minimum, distance, np.inf), axis=1)
    right_distance = np.min(np.where(cross < -minimum, distance, np.inf), axis=1)
    left = left_distance <= limit
    right = right_distance <= limit

    def median_or_none(values: np.ndarray, valid: np.ndarray) -> float | None:
        if not valid.any():
            return None
        return round(float(np.median(values[valid])), 3)

    return {
        "sample_count": int(len(stations)),
        "left_coverage": round(float(left.mean()), 4),
        "right_coverage": round(float(right.mean()), 4),
        "bilateral_coverage": round(float((left & right).mean()), 4),
        "left_median_distance_m": median_or_none(left_distance, left),
        "right_median_distance_m": median_or_none(right_distance, right),
    }


def merge_intervals(
    intervals: Iterable[Tuple[float, float, str]], length_m: float
) -> List[Dict[str, Any]]:
    """Merge overlapping guard intervals while retaining all reason codes."""

    values = []
    for start, end, reason in intervals:
        lo = max(0.0, min(float(start), float(length_m)))
        hi = max(0.0, min(float(end), float(length_m)))
        if hi > lo + 1e-9:
            values.append((lo, hi, str(reason)))
    values.sort(key=lambda item: (item[0], item[1], item[2]))
    merged: List[Dict[str, Any]] = []
    for start, end, reason in values:
        if not merged or start > float(merged[-1]["end_m"]) + 1e-9:
            merged.append({"start_m": start, "end_m": end, "reasons": [reason]})
            continue
        merged[-1]["end_m"] = max(float(merged[-1]["end_m"]), end)
        if reason not in merged[-1]["reasons"]:
            merged[-1]["reasons"].append(reason)
    for item in merged:
        item["start_m"] = round(float(item["start_m"]), 6)
        item["end_m"] = round(float(item["end_m"]), 6)
        item["reasons"].sort()
    return merged


def complement_intervals(
    guarded: Sequence[Mapping[str, Any]], length_m: float, minimum_length_m: float
) -> Tuple[List[Dict[str, float]], List[Dict[str, Any]]]:
    """Return clip candidates and absorb unsafe short remainders into guards."""

    tolerance = 1e-6
    guard_values = [dict(item) for item in guarded]
    candidates: List[Dict[str, float]] = []
    cursor = 0.0
    for item in guard_values:
        start, end = float(item["start_m"]), float(item["end_m"])
        if start > cursor + tolerance:
            candidates.append({"start_m": cursor, "end_m": start})
        cursor = max(cursor, end)
    if cursor < float(length_m) - tolerance:
        candidates.append({"start_m": cursor, "end_m": float(length_m)})

    short = [
        item
        for item in candidates
        if item["end_m"] - item["start_m"] + tolerance < minimum_length_m
    ]
    if short:
        extra = [
            (item["start_m"], item["end_m"], "clip_interval_below_minimum") for item in short
        ]
        guard_values = merge_intervals(
            [(g["start_m"], g["end_m"], reason) for g in guard_values for reason in g["reasons"]]
            + extra,
            length_m,
        )
        return complement_intervals(guard_values, length_m, 0.0)[0], guard_values
    for item in candidates:
        item["start_m"] = round(float(item["start_m"]), 6)
        item["end_m"] = round(float(item["end_m"]), 6)
    return candidates, guard_values


def _read_h5(path: Path) -> Dict[str, Any]:
    try:
        import h5py
    except ImportError as error:
        raise DrivableAreaAuditError("h5py is required for the P2-A H5 audit") from error
    with h5py.File(str(path), "r") as handle:
        if "road" not in handle:
            raise DrivableAreaAuditError("legacy H5 has no road layer")
        road = np.asarray(handle["road"], dtype=np.uint8)
        attrs = handle.attrs
        return {
            "map_id": str(attrs.get("map_id", "")),
            "local_origin_in_global": [float(value) for value in attrs["local_origin_in_global"]],
            "pixels_per_meter": float(attrs["pixels_per_meter"]),
            "world_offset_in_meters": [float(value) for value in attrs["world_offset_in_meters"]],
            "shape": [int(value) for value in road.shape],
            "road_nonzero_px": int(np.count_nonzero(road)),
            "road_area_m2": round(float(np.count_nonzero(road)) / float(attrs["pixels_per_meter"]) ** 2, 2),
            "road": road,
        }


def _contains(mask_info: Mapping[str, Any], points: np.ndarray) -> np.ndarray:
    road = mask_info["road"]
    ppm = float(mask_info["pixels_per_meter"])
    offset = np.asarray(mask_info["world_offset_in_meters"], dtype=np.float64)
    pixels = np.rint(ppm * (np.asarray(points, dtype=np.float64)[:, :2] - offset)).astype(np.int64)
    valid = (
        (pixels[:, 0] >= 0)
        & (pixels[:, 0] < road.shape[1])
        & (pixels[:, 1] >= 0)
        & (pixels[:, 1] < road.shape[0])
    )
    result = np.zeros(len(pixels), dtype=bool)
    result[valid] = road[pixels[valid, 1], pixels[valid, 0]] > 0
    return result


def _round_interval_stations(items: List[Dict[str, Any]], route_start_m: float) -> None:
    for item in items:
        item["route_start_m"] = round(route_start_m + float(item["start_m"]), 6)
        item["route_end_m"] = round(route_start_m + float(item["end_m"]), 6)


def build_drivable_area_input_audit(repo_root: Path, source_config: Path) -> Dict[str, Any]:
    repo_root = repo_root.resolve()
    config = _load_yaml(source_config)
    paths = _source_paths(repo_root, config)
    expected = config.get("expected") or {}
    policy = config.get("classification") or {}

    links_raw = _load_json(paths["current_link_set"])
    nodes_raw = _load_json(paths["current_node_set"])
    global_info = _load_json(paths["current_global_info"])
    route_config = _load_yaml(paths["reviewed_route_links"])
    reviewed_map = _load_json(paths["reviewed_lane_markings"])
    metadata = _load_yaml(paths["competition_metadata"])
    h5 = _read_h5(paths["legacy_drivable_h5"])

    links = {str(item.get("idx")): item for item in links_raw}
    nodes = {str(item.get("idx")): item for item in nodes_raw}
    route_ids = [str(value) for value in ((route_config.get("route") or {}).get("links") or [])]
    metadata_route = metadata.get("route") or {}
    ranges = metadata_route.get("link_ranges") or []
    range_by_id = {str(item["link_id"]): item for item in ranges}
    if route_ids != [str(value) for value in metadata_route.get("link_ids") or []]:
        raise DrivableAreaAuditError("route link order differs from frozen competition metadata")
    if len(route_ids) != int(expected["route_link_count"]) or len(set(route_ids)) != len(route_ids):
        raise DrivableAreaAuditError("route link count/uniqueness mismatch")
    if any(link_id not in links for link_id in route_ids):
        raise DrivableAreaAuditError("route contains a link absent from current link_set")

    incoming: Counter[str] = Counter(str(item.get("to_node_idx")) for item in links_raw)
    outgoing: Counter[str] = Counter(str(item.get("from_node_idx")) for item in links_raw)
    continuity_errors = []
    pairs = list(zip(route_ids, route_ids[1:] + route_ids[:1]))
    for left_id, right_id in pairs:
        if str(links[left_id].get("to_node_idx")) != str(links[right_id].get("from_node_idx")):
            continuity_errors.append([left_id, right_id])
    if continuity_errors:
        raise DrivableAreaAuditError("route is not topologically continuous: %s" % continuity_errors)

    marking_by_link: Dict[str, List[np.ndarray]] = defaultdict(list)
    marking_ids_by_link: Dict[str, List[str]] = defaultdict(list)
    counts = Counter()
    for feature in reviewed_map.get("features", []):
        feature_type = str(feature.get("feature_type"))
        counts[feature_type] += 1
        if feature_type != "lane_marking":
            continue
        points = _xy((feature.get("geometry") or {}).get("coordinates") or [])
        for link_id in (feature.get("topology") or {}).get("current_link_ids", []):
            link_id = str(link_id)
            if link_id in links:
                marking_by_link[link_id].append(points)
                marking_ids_by_link[link_id].append(str(feature.get("id")))

    actual_geometry_sha = geometry_sha256(reviewed_map)
    if actual_geometry_sha != str(expected["reviewed_geometry_sha256"]):
        raise DrivableAreaAuditError("reviewed geometry hash mismatch")
    if counts["lane_marking"] != int(expected["lane_marking_count"]):
        raise DrivableAreaAuditError("lane marking count mismatch")
    if counts["stopline"] != int(expected["stopline_count"]):
        raise DrivableAreaAuditError("stopline count mismatch")

    origin = [float(value) for value in global_info.get("local_origin_in_global") or []]
    expected_origin = [float(value) for value in expected["local_origin_in_global"]]
    if origin != expected_origin or h5["local_origin_in_global"] != expected_origin:
        raise DrivableAreaAuditError("MGeo/H5 local origins do not match")
    if h5["map_id"] != str(expected["map_id"]):
        raise DrivableAreaAuditError("legacy H5 map_id mismatch")
    if h5["shape"] != [int(value) for value in expected["legacy_road_shape"]]:
        raise DrivableAreaAuditError("legacy H5 road shape mismatch")
    if h5["road_nonzero_px"] != int(expected["legacy_road_nonzero_px"]):
        raise DrivableAreaAuditError("legacy H5 road pixel count mismatch")
    if not math.isclose(h5["pixels_per_meter"], float(expected["legacy_pixels_per_meter"])):
        raise DrivableAreaAuditError("legacy H5 resolution mismatch")

    intersection_types = {str(value) for value in policy["intersection_link_types"]}
    merge_types = {str(value) for value in policy["merge_diverge_link_types"]}
    transition_buffer = float(policy["topology_transition_buffer_m"])
    minimum_coverage = float(policy["minimum_bilateral_coverage"])
    minimum_clip = float(policy["minimum_lane_clip_interval_m"])
    full_guard_reasons = {str(key): str(value) for key, value in (policy.get("full_guard_reasons") or {}).items()}

    records = []
    action_counts: Counter[str] = Counter()
    guarded_length = 0.0
    candidate_length = 0.0
    for order, link_id in enumerate(route_ids):
        link = links[link_id]
        route_range = range_by_id[link_id]
        length = float(route_range["length_m"])
        from_node = str(link.get("from_node_idx"))
        to_node = str(link.get("to_node_idx"))
        link_type = str(link.get("link_type"))
        coverage = bilateral_boundary_coverage(
            _xy(link.get("points") or []),
            marking_by_link.get(link_id, []),
            sample_step_m=float(policy["sample_step_m"]),
            marking_sample_step_m=float(policy["marking_sample_step_m"]),
            minimum_lateral_distance_m=float(policy["minimum_lateral_distance_m"]),
            maximum_boundary_distance_m=float(policy["maximum_boundary_distance_m"]),
        )
        guard_inputs: List[Tuple[float, float, str]] = []
        if link_type in full_guard_reasons:
            guard_inputs.append((0.0, length, full_guard_reasons[link_type]))
        elif link_type in merge_types:
            guard_inputs.append((0.0, length, "merge_diverge_link"))
        if link_type in intersection_types:
            guard_inputs.append(
                (0.0, min(length, float(policy["intersection_clear_from_start_m"])), "intersection_link")
            )
        if float(coverage["bilateral_coverage"]) < minimum_coverage:
            guard_inputs.append((0.0, length, "insufficient_bilateral_marking_coverage"))

        # Branching at a link endpoint is guarded even if paint is visible.
        # Stop-line intersections are already handled by their link type, but
        # the approach-side transition still remains protected by this rule.
        if incoming[from_node] != 1 or outgoing[from_node] != 1:
            guard_inputs.append((0.0, min(length, transition_buffer), "topology_transition_at_start"))
        if incoming[to_node] != 1 or outgoing[to_node] != 1:
            guard_inputs.append((max(0.0, length - transition_buffer), length, "topology_transition_at_end"))

        guards = merge_intervals(guard_inputs, length)
        candidates, guards = complement_intervals(guards, length, minimum_clip)
        route_start = float(route_range["start_station_m"])
        _round_interval_stations(guards, route_start)
        _round_interval_stations(candidates, route_start)
        guard_length = sum(float(item["end_m"]) - float(item["start_m"]) for item in guards)
        clip_length = sum(float(item["end_m"]) - float(item["start_m"]) for item in candidates)
        guarded_length += guard_length
        candidate_length += clip_length
        if not candidates:
            action = "preserve_prior"
        elif not guards:
            action = "lane_clip_candidate"
        else:
            action = "hybrid_guard_and_clip_candidate"
        action_counts[action] += 1
        records.append(
            {
                "link_id": link_id,
                "order": order,
                "route_start_m": round(route_start, 6),
                "route_end_m": round(float(route_range["end_station_m"]), 6),
                "length_m": round(length, 6),
                "link_type": link_type,
                "road_id": str(link.get("road_id") or ""),
                "ego_lane": int(link.get("ego_lane") or 0),
                "from_node": from_node,
                "to_node": to_node,
                "topology": {
                    "from_incoming": int(incoming[from_node]),
                    "from_outgoing": int(outgoing[from_node]),
                    "to_incoming": int(incoming[to_node]),
                    "to_outgoing": int(outgoing[to_node]),
                },
                "reviewed_marking_count": len(marking_ids_by_link.get(link_id, [])),
                "boundary_evidence": coverage,
                "action": action,
                "intersection_guarded": bool(guards),
                "guard_intervals": guards,
                "lane_clip_candidate_intervals": candidates,
                "status": "audited_not_applied",
            }
        )

    checkpoints = metadata.get("checkpoints") or []
    checkpoint_points = np.asarray(
        [[item["pose"]["position"]["x"], item["pose"]["position"]["y"]] for item in checkpoints],
        dtype=np.float64,
    )
    checkpoint_inside = _contains(h5, checkpoint_points)
    if not checkpoint_inside.all():
        missing = [str(checkpoints[index]["id"]) for index, value in enumerate(checkpoint_inside) if not value]
        raise DrivableAreaAuditError("checkpoint centers outside legacy road: %s" % missing)

    route_length = float(metadata_route["length_xy_m"])
    if not math.isclose(route_length, float(expected["route_length_m"]), abs_tol=1e-6):
        raise DrivableAreaAuditError("route length mismatch")
    if len(checkpoints) != int(expected["checkpoint_count"]):
        raise DrivableAreaAuditError("checkpoint count mismatch")
    if not math.isclose(guarded_length + candidate_length, route_length, abs_tol=1e-4):
        raise DrivableAreaAuditError("classification does not cover the route exactly once")

    sources = {}
    for name, path in sorted(paths.items()):
        sources[name] = {
            "path": str(path.relative_to(repo_root)),
            "sha256": sha256_file(path),
            "bytes": int(path.stat().st_size),
        }
    result = {
        "schema_version": "asmc.hd_map.drivable_area_input_audit/v1",
        "audit_id": str(config["audit_id"]),
        "scope": str(config["scope"]),
        "status": "p2a_complete_no_geometry_change",
        "sources": sources,
        "frame": {
            "frame_id": str(expected["frame_id"]),
            "map_id": h5["map_id"],
            "local_origin_in_global": expected_origin,
        },
        "reviewed_hd_map": {
            "geometry_sha256": actual_geometry_sha,
            "lane_marking_count": int(counts["lane_marking"]),
            "stopline_count": int(counts["stopline"]),
        },
        "legacy_drivable_prior": {
            "representation": "node_link_centerline_buffer_raster_with_reviewed_observation_adjustments",
            "true_vector_road_mesh_available": False,
            "dataset": "road",
            "shape": h5["shape"],
            "pixels_per_meter": h5["pixels_per_meter"],
            "world_offset_in_meters": h5["world_offset_in_meters"],
            "road_nonzero_px": h5["road_nonzero_px"],
            "road_area_m2": h5["road_area_m2"],
            "checkpoint_centers_inside": int(checkpoint_inside.sum()),
            "checkpoint_count": len(checkpoints),
        },
        "classification_policy": {
            key: value for key, value in policy.items()
        },
        "route_summary": {
            "closed_loop": True,
            "topologically_continuous": True,
            "link_count": len(route_ids),
            "route_length_m": round(route_length, 9),
            "guarded_length_m": round(guarded_length, 6),
            "lane_clip_candidate_length_m": round(candidate_length, 6),
            "action_counts": dict(sorted(action_counts.items())),
            "full_route_coverage": True,
        },
        "route_links": records,
        "next_gate": {
            "stage": "P2-B_lane_and_drivable_geometry",
            "requirements": [
                "build lane centerlines/polygons without modifying reviewed paint",
                "clip only lane_clip_candidate_intervals",
                "preserve legacy road inside every guard interval",
                "validate checkpoint centers, route connectivity, polygon validity, and geometry SHA parity",
            ],
        },
    }
    return result


def render_report(document: Mapping[str, Any]) -> str:
    summary = document["route_summary"]
    prior = document["legacy_drivable_prior"]
    lines = [
        "# K-City 2025 P2-A lane topology·drivable-area 입력 감사",
        "",
        "> **상태:** P2-A 완료 — 입력과 보호구간만 동결, geometry/publisher 변경 없음",
        "",
        "## 결론",
        "",
        "- 전역경로 39개 링크는 위상적으로 닫혀 있고 총 길이는 `%.3f m`다." % summary["route_length_m"],
        "- 기존 주행가능영역은 진짜 vector road mesh가 아니라 Node/Link 중심선 buffer에 검수 보정을 더한 H5 raster다.",
        "- 기존 road 면적은 `%.2f m²`, 공식 checkpoint 중심은 `%d/%d` 모두 road 안에 있다."
        % (prior["road_area_m2"], prior["checkpoint_centers_inside"], prior["checkpoint_count"]),
        "- 보호구간은 `%.3f m`, 차선 기반 clip 후보는 `%.3f m`이며 합계가 전역경로를 정확히 한 번 덮는다."
        % (summary["guarded_length_m"], summary["lane_clip_candidate_length_m"]),
        "- stage14 geometry hash는 `%s`로 유지됐다." % document["reviewed_hd_map"]["geometry_sha256"],
        "",
        "## 분류 규칙",
        "",
        "- MGeo `link_type=1`은 시작부 최대 30 m를 교차로 보호구간으로 둔다.",
        "- MGeo `link_type=3`인 A153/A451은 합류·분기 및 톨게이트 섬 때문에 링크 전체를 보호한다.",
        "- 노드의 incoming/outgoing 수가 1:1이 아니면 해당 끝점 5 m를 보호한다.",
        "- 좌우 reviewed marking 동시 coverage가 80% 미만이면 링크 전체를 보호한다.",
        "- clip 후보가 5 m보다 짧으면 불안정한 조각을 만들지 않고 보호구간에 흡수한다.",
        "",
        "이 분류는 P2-B에서 적용할 수 있는 후보를 정한 것이지, 아직 road를 잘라낸 결과가 아니다.",
        "물리 도색의 노드 gap도 연결하지 않았으며 `/hd_map/global/drivable_*` 토픽은 계속 비활성 상태다.",
        "",
        "## 링크별 판정",
        "",
        "| 링크 | route s (m) | 양측 coverage | 판정 | 보호 사유 |",
        "|---|---:|---:|---|---|",
    ]
    for item in document["route_links"]:
        reasons = sorted({reason for interval in item["guard_intervals"] for reason in interval["reasons"]})
        lines.append(
            "| `%s` | %.1f–%.1f | %.0f%% | `%s` | %s |"
            % (
                item["link_id"],
                item["route_start_m"],
                item["route_end_m"],
                100.0 * float(item["boundary_evidence"]["bilateral_coverage"]),
                item["action"],
                ", ".join(reasons) if reasons else "—",
            )
        )
    lines.extend(
        [
            "",
            "## 다음 gate",
            "",
            "P2-B에서 Lane centerline/polygon과 vector drivable polygon을 생성한다. clip은 이 manifest의",
            "`lane_clip_candidate_intervals`에만 적용하고 `guard_intervals`에서는 기존 H5 road를 보존한다.",
            "checkpoint 포함, 연결성, polygon self-intersection, lane adjacency와 stage14 geometry hash parity를",
            "모두 통과하기 전에는 `link_node`를 삭제하지 않는다.",
            "",
        ]
    )
    return "\n".join(lines)


def write_audit(document: Mapping[str, Any], manifest_path: Path, report_path: Path) -> None:
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(
        yaml.safe_dump(document, allow_unicode=True, sort_keys=False, width=110),
        encoding="utf-8",
    )
    report_path.write_text(render_report(document), encoding="utf-8")
