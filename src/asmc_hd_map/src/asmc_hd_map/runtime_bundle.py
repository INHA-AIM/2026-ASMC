"""Load and validate the frozen P1-D read-only runtime bundle.

This module deliberately contains no ROS imports.  It verifies that the
reviewed stage14 geometry, P1-C competition metadata, P1-A baseline, and the
official route still agree before a ROS publisher is allowed to expose them.
It never repairs or joins lane-marking geometry at runtime.
"""

from __future__ import annotations

from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import yaml

from .schema import validate_canonical_document


class RuntimeBundleError(ValueError):
    """Raised when a frozen runtime artifact is missing or inconsistent."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def geometry_sha256(document: Mapping[str, Any]) -> str:
    payload = [
        {
            "id": feature.get("id"),
            "feature_type": feature.get("feature_type"),
            "frame_id": feature.get("frame_id"),
            "geometry": feature.get("geometry"),
        }
        for feature in document.get("features", [])
    ]
    payload.sort(key=lambda item: (str(item["feature_type"]), str(item["id"])))
    return canonical_sha256(payload)


def _load_json(path: Path) -> Mapping[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise RuntimeBundleError("cannot load JSON %s: %s" % (path, error))
    if not isinstance(value, dict):
        raise RuntimeBundleError("JSON root must be an object: %s" % path)
    return value


def _load_yaml(path: Path) -> Mapping[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeBundleError("cannot load YAML %s: %s" % (path, error))
    if not isinstance(value, dict):
        raise RuntimeBundleError("YAML root must be a mapping: %s" % path)
    return value


def _require_file(path: Path, label: str) -> Path:
    value = path.expanduser().resolve()
    if not value.is_file():
        raise RuntimeBundleError("missing %s: %s" % (label, value))
    return value


def find_repo_root(path: Path) -> Path:
    """Find the checkout root from any frozen artifact below it."""

    value = path.expanduser().resolve()
    for parent in (value.parent,) + tuple(value.parents):
        if (parent / "config/baselines/runtime_baseline_v1.yaml").is_file():
            return parent
    raise RuntimeBundleError("repository root was not found from %s" % value)


def default_data_root(repo_root: Path) -> Path:
    configured = os.environ.get("ASMC_DATA")
    if configured:
        return Path(configured).expanduser().resolve()
    sibling = (repo_root.parent / "data").resolve()
    return sibling if sibling.is_dir() else Path("/data")


def resolve_source_path(raw_path: str, repo_root: Path, data_root: Path) -> Path:
    prefix = "$ASMC_DATA/"
    if raw_path.startswith(prefix):
        return (data_root / raw_path[len(prefix) :]).resolve()
    expanded = os.path.expandvars(os.path.expanduser(raw_path))
    if "$" in expanded:
        raise RuntimeBundleError("unresolved variable in source path: %s" % raw_path)
    value = Path(expanded)
    return value.resolve() if value.is_absolute() else (repo_root / value).resolve()


def load_route_points(path: Path) -> List[List[float]]:
    points: List[List[float]] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise RuntimeBundleError("cannot read route %s: %s" % (path, error))
    for line_number, line in enumerate(lines, 1):
        values = line.split()
        if not values:
            continue
        if len(values) != 3:
            raise RuntimeBundleError("invalid route line %d in %s" % (line_number, path))
        try:
            point = [float(value) for value in values]
        except ValueError:
            raise RuntimeBundleError("non-numeric route line %d in %s" % (line_number, path))
        if not all(math.isfinite(value) for value in point):
            raise RuntimeBundleError("non-finite route line %d in %s" % (line_number, path))
        points.append(point)
    if len(points) < 2:
        raise RuntimeBundleError("route must contain at least two points: %s" % path)
    return points


def polyline_stations(points: Sequence[Sequence[float]]) -> List[float]:
    if len(points) < 2:
        raise RuntimeBundleError("polyline must contain at least two points")
    stations = [0.0]
    for left, right in zip(points, points[1:]):
        stations.append(
            stations[-1] + math.hypot(float(right[0]) - float(left[0]), float(right[1]) - float(left[1]))
        )
    if stations[-1] <= 0.0:
        raise RuntimeBundleError("polyline length must be positive")
    return stations


def point_at_station(
    points: Sequence[Sequence[float]], stations: Sequence[float], station_m: float
) -> List[float]:
    value = max(0.0, min(float(station_m), float(stations[-1])))
    for index in range(len(stations) - 1):
        if value <= float(stations[index + 1]) + 1e-12:
            length = float(stations[index + 1]) - float(stations[index])
            fraction = 0.0 if length <= 1e-12 else (value - float(stations[index])) / length
            left, right = points[index], points[index + 1]
            z_left = float(left[2]) if len(left) > 2 else 0.0
            z_right = float(right[2]) if len(right) > 2 else 0.0
            return [
                float(left[0]) + fraction * (float(right[0]) - float(left[0])),
                float(left[1]) + fraction * (float(right[1]) - float(left[1])),
                z_left + fraction * (z_right - z_left),
            ]
    last = points[-1]
    return [float(last[0]), float(last[1]), float(last[2]) if len(last) > 2 else 0.0]


def lateral_point_at_station(
    points: Sequence[Sequence[float]],
    stations: Sequence[float],
    station_m: float,
    lateral_offset_m: float,
) -> List[float]:
    """Return a route-relative point using the local left-hand normal.

    This is only a debug-label layout helper.  It never changes the frozen map
    or route geometry and deliberately derives its tangent from the official
    route rather than from lane-marking endpoints.
    """

    if len(points) != len(stations) or len(points) < 2:
        raise RuntimeBundleError("route points/stations must have equal length >= 2")
    route_length = float(stations[-1])
    if route_length <= 0.0:
        raise RuntimeBundleError("route length must be positive")
    station = max(0.0, min(float(station_m), route_length))
    span = min(2.0, max(0.25, route_length * 0.001))
    before = point_at_station(points, stations, max(0.0, station - span))
    after = point_at_station(points, stations, min(route_length, station + span))
    dx = float(after[0]) - float(before[0])
    dy = float(after[1]) - float(before[1])
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        raise RuntimeBundleError("cannot derive route normal at station %.3f" % station)
    center = point_at_station(points, stations, station)
    offset = float(lateral_offset_m)
    return [
        float(center[0]) - dy / length * offset,
        float(center[1]) + dx / length * offset,
        float(center[2]),
    ]


def choose_label_position(
    points: Sequence[Sequence[float]],
    stations: Sequence[float],
    station_m: float,
    lateral_offsets_m: Sequence[float],
    occupied_xy: Sequence[Sequence[float]],
    minimum_spacing_m: float,
) -> List[float]:
    """Choose the first route-relative label position clear of prior labels.

    When every candidate is crowded, the candidate with the greatest nearest
    neighbour distance is used.  Candidate ordering therefore remains the
    deterministic preference order supplied by the caller.
    """

    if not lateral_offsets_m:
        raise RuntimeBundleError("at least one lateral label offset is required")
    candidates = [
        lateral_point_at_station(points, stations, station_m, offset)
        for offset in lateral_offsets_m
    ]
    if not occupied_xy:
        return candidates[0]

    def clearance(candidate: Sequence[float]) -> float:
        return min(
            math.hypot(float(candidate[0]) - float(value[0]), float(candidate[1]) - float(value[1]))
            for value in occupied_xy
        )

    minimum = max(0.0, float(minimum_spacing_m))
    for candidate in candidates:
        if clearance(candidate) >= minimum:
            return candidate
    return max(candidates, key=clearance)


def short_link_id(link_id: str) -> str:
    """Return the competition shorthand (for example A2256W000846 -> A846)."""

    value = str(link_id)
    if len(value) == 12 and value[0].isalpha() and value[1:5].isdigit() and value[5] == "W":
        suffix = value[-3:]
        if suffix.isdigit():
            return value[0] + suffix
    return value


def polyline_slice(
    points: Sequence[Sequence[float]],
    stations: Sequence[float],
    start_station_m: float,
    end_station_m: float,
) -> List[List[float]]:
    start = max(0.0, min(float(start_station_m), float(stations[-1])))
    end = max(0.0, min(float(end_station_m), float(stations[-1])))
    if end < start:
        raise RuntimeBundleError("polyline slice end precedes start")
    result = [point_at_station(points, stations, start)]
    for station, point in zip(stations[1:-1], points[1:-1]):
        if start < float(station) < end:
            result.append([
                float(point[0]),
                float(point[1]),
                float(point[2]) if len(point) > 2 else 0.0,
            ])
    result.append(point_at_station(points, stations, end))
    return result


def marking_role_name(feature: Mapping[str, Any]) -> str:
    if feature.get("feature_type") == "stopline":
        return "stop_line"
    role = str((feature.get("attributes") or {}).get("functional_role", "")).lower()
    if "island" in role or "tollgate" in role:
        return "island_edge"
    if "centerline" in role or "centre" in role:
        return "road_center"
    if "divider" in role or "split" in role:
        return "lane_divider"
    if "guide" in role:
        return "keep_out"
    if "edge" in role or "outer_boundary" in role:
        return "road_edge"
    return "unknown"


def source_type_name(feature: Mapping[str, Any]) -> str:
    display = str((feature.get("attributes") or {}).get("source_display", "")).lower()
    if "legacy" in display:
        return "legacy_map"
    if "preplan" in display:
        return "preplan"
    if "won_tae" in display or "h5" in display:
        return "survey"
    if "current_link" in display or "derived" in display:
        return "derived"
    return "curated"


def load_runtime_bundle(
    map_path: Path,
    metadata_path: Path,
    baseline_path: Optional[Path] = None,
    data_root: Optional[Path] = None,
) -> Dict[str, Any]:
    """Load all P1-D inputs and reject any drift from the frozen baseline."""

    map_path = _require_file(Path(map_path), "reviewed HD map")
    metadata_path = _require_file(Path(metadata_path), "competition metadata")
    repo_root = find_repo_root(map_path)
    baseline_path = _require_file(
        Path(baseline_path) if baseline_path else repo_root / "config/baselines/runtime_baseline_v1.yaml",
        "runtime baseline",
    )
    data_root = Path(data_root).expanduser().resolve() if data_root else default_data_root(repo_root)

    map_document = _load_json(map_path)
    metadata = _load_yaml(metadata_path)
    baseline = _load_yaml(baseline_path)
    try:
        validate_canonical_document(dict(map_document))
    except ValueError as error:
        raise RuntimeBundleError("invalid canonical HD map: %s" % error)

    hd_baseline = baseline.get("hd_map") or {}
    competition_baseline = (baseline.get("competition_inputs") or {}).get("metadata_bundle") or {}
    map_sha = sha256_file(map_path)
    metadata_sha = sha256_file(metadata_path)
    manifest_sha = sha256_file(baseline_path)
    geometry_sha = geometry_sha256(map_document)
    reviewed_source = (metadata.get("sources") or {}).get("reviewed_hd_map") or {}

    expected_pairs = (
        (map_sha, reviewed_source.get("sha256"), "metadata reviewed-HD-map SHA"),
        (map_sha, hd_baseline.get("file_sha256"), "baseline HD-map file SHA"),
        (metadata_sha, competition_baseline.get("sha256"), "baseline metadata SHA"),
        (geometry_sha, hd_baseline.get("geometry_sha256"), "baseline geometry SHA"),
        (map_document.get("map_id"), hd_baseline.get("map_id"), "map ID"),
        ((map_document.get("build") or {}).get("build_id"), hd_baseline.get("build_id"), "build ID"),
    )
    for actual, expected, label in expected_pairs:
        if not expected or actual != expected:
            raise RuntimeBundleError("%s mismatch: %s != %s" % (label, actual, expected))

    features = list(map_document.get("features") or [])
    feature_counts = Counter(str(feature.get("feature_type")) for feature in features)
    checkpoints = list(metadata.get("checkpoints") or [])
    mission_zones = list(metadata.get("mission_zones") or [])
    expected_counts = (
        (feature_counts["lane_marking"], 121, "lane marking count"),
        (feature_counts["stopline"], 7, "stopline count"),
        (len(checkpoints), int(competition_baseline.get("checkpoint_count", -1)), "checkpoint count"),
        (len(mission_zones), int(competition_baseline.get("mission_zone_count", -1)), "mission count"),
    )
    for actual, expected, label in expected_counts:
        if actual != expected:
            raise RuntimeBundleError("%s mismatch: %s != %s" % (label, actual, expected))
    if not bool((metadata.get("validation") or {}).get("passed")):
        raise RuntimeBundleError("competition metadata validation is not passed")

    global_path_source = (metadata.get("sources") or {}).get("official_global_path") or {}
    route_path = _require_file(
        resolve_source_path(str(global_path_source.get("path", "")), repo_root, data_root),
        "official global path",
    )
    if sha256_file(route_path) != global_path_source.get("sha256"):
        raise RuntimeBundleError("official global-path SHA mismatch")
    route_points = load_route_points(route_path)
    route_stations = polyline_stations(route_points)
    route_length = float((metadata.get("route") or {}).get("length_xy_m", -1.0))
    if abs(route_stations[-1] - route_length) > 1e-6:
        raise RuntimeBundleError(
            "route length mismatch: %.9f != %.9f" % (route_stations[-1], route_length)
        )

    return {
        "repo_root": repo_root,
        "data_root": data_root,
        "map_path": map_path,
        "metadata_path": metadata_path,
        "baseline_path": baseline_path,
        "route_path": route_path,
        "map": map_document,
        "metadata": metadata,
        "baseline": baseline,
        "features": features,
        "checkpoints": checkpoints,
        "mission_zones": mission_zones,
        "route_points": route_points,
        "route_stations": route_stations,
        "map_sha256": map_sha,
        "metadata_sha256": metadata_sha,
        "manifest_sha256": manifest_sha,
        "geometry_sha256": geometry_sha,
        "feature_counts": dict(feature_counts),
    }
