"""Build a link-baked competition-route drivable map from reviewed inputs.

The legacy H5 supplies a verified frame/extent and regression reference, not
the road boundary. Selection is driven by the frozen route, same-direction
usable lanes, an explicit intersection-link allowlist, and a reviewed dynamic
junction node-pair closure. Ordinary lane polygons can follow reviewed paint,
adjacent lanes are joined as a corridor, and obstacle-junction gaps use legacy
H5 only inside the selected flat-ended link envelope. No radius/proximity
expansion is allowed to pull unrelated roads into the product.
"""

from __future__ import annotations

from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

import cv2
import numpy as np
import yaml

from .runtime_bundle import geometry_sha256


class CompactDrivableError(ValueError):
    pass


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def load_yaml(path: Path) -> Mapping[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise CompactDrivableError("YAML root must be a mapping: %s" % path)
    return value


def _path(repo_root: Path, spec: Mapping[str, Any]) -> Path:
    value = (repo_root / str(spec["path"])).resolve()
    if not value.is_file():
        raise CompactDrivableError("missing source: %s" % value)
    return value


def _xy(points: Sequence[Sequence[float]]) -> np.ndarray:
    value = np.asarray(points, dtype=np.float64)
    if value.ndim != 2 or value.shape[0] < 2 or value.shape[1] < 2:
        raise CompactDrivableError("polyline requires at least two XY points")
    return value[:, :2]


def _length(points: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1).sum())


def _direction(points: np.ndarray) -> np.ndarray:
    delta = points[-1, :2] - points[0, :2]
    norm = float(np.linalg.norm(delta))
    return delta / max(norm, 1e-9)


def _local_to_pixel(points: np.ndarray, ppm: float, offset: np.ndarray) -> np.ndarray:
    return np.rint((np.asarray(points)[:, :2] - offset[None, :]) * ppm).astype(np.int32)


def _resample_centerline(points: np.ndarray, step_m: float = 0.5) -> np.ndarray:
    segment = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
    keep = segment > 1e-8
    clean = np.vstack([points[0], points[1:][keep]])
    stations = np.concatenate([[0.0], np.cumsum(segment[keep])])
    targets = np.arange(0.0, stations[-1], step_m)
    targets = np.append(targets, stations[-1])
    return np.column_stack([np.interp(targets, stations, clean[:, axis]) for axis in range(2)])


def _parse_h5(repo_root: Path, path: Path) -> Dict[str, Any]:
    try:
        import h5py
    except ImportError as error:
        raise CompactDrivableError("h5py is required to build compact drivable data") from error
    with h5py.File(str(path), "r") as handle:
        road = np.asarray(handle["road"], dtype=np.uint8)
        return {
            "road": np.where(road > 0, 255, 0).astype(np.uint8),
            "ppm": float(handle.attrs["pixels_per_meter"]),
            "offset": np.asarray(handle.attrs["world_offset_in_meters"], dtype=np.float64)[:2],
            "map_id": str(handle.attrs.get("map_id", "")),
            "sha256": sha256_file(path),
            "path": str(path.relative_to(repo_root)),
        }


def _same_direction_siblings(
    links: Mapping[str, Mapping[str, Any]], route_ids: Sequence[str], exclusions: set[str], minimum_dot: float
) -> Tuple[List[str], List[str]]:
    by_road: Dict[str, List[str]] = defaultdict(list)
    for link_id, item in links.items():
        by_road[str(item.get("road_id") or "")].append(link_id)
    route_set = set(route_ids)
    selected = set(route_ids)
    rejected = set()
    for route_id in route_ids:
        route = links[route_id]
        road_id = str(route.get("road_id") or "")
        reference = _direction(_xy(route["points"]))
        for candidate_id in by_road[road_id]:
            if candidate_id in route_set:
                continue
            candidate = links[candidate_id]
            dot = float(np.dot(reference, _direction(_xy(candidate["points"]))))
            if candidate_id in exclusions or dot < minimum_dot:
                rejected.add(candidate_id)
            else:
                selected.add(candidate_id)
    return sorted(selected), sorted(rejected)


def _route_range_by_id(audit: Mapping[str, Any]) -> Dict[str, Mapping[str, Any]]:
    return {str(item["link_id"]): item for item in audit["route_links"]}


def _lane_type(link_type: str) -> str:
    return {"1": "intersection", "2": "merge", "3": "diverge_toll"}.get(link_type, "driving")


def _boundary_ids(link: Mapping[str, Any], side: str) -> List[str]:
    return [str(value) for value in link.get("lane_mark_%s" % side, [])]


def lane_width_polygon(link: Mapping[str, Any], step_m: float) -> List[List[float]]:
    """Return a link-width polygon cut flat on each endpoint-node normal.

    The first and last cross-sections use the local centerline normal, so the
    polygon ends at the link node instead of extending a round half-width cap
    past it. Junction coverage must come from selected topology links rather
    than a cap that paints outside the link boundary.
    """

    center = _resample_centerline(_xy(link["points"]), step_m)
    tangent = np.gradient(center, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    start_width = float(link.get("width_start") or 3.5)
    end_width = float(link.get("width_end") or start_width)
    half_width = 0.5 * np.linspace(start_width, end_width, len(center))
    left = center + normal * half_width[:, None]
    right = center - normal * half_width[:, None]
    ring = np.vstack([left, right[::-1]])
    return [[round(float(x), 4), round(float(y), 4), 0.0] for x, y in ring]


def _smooth_offsets(values: np.ndarray, window_points: int) -> np.ndarray:
    if window_points <= 1:
        return values
    if window_points % 2 == 0:
        window_points += 1
    pad = window_points // 2
    padded = np.pad(values, (pad, pad), mode="edge")
    kernel = np.ones(window_points, dtype=np.float64) / float(window_points)
    return np.convolve(padded, kernel, mode="valid")


def lane_boundary_polygon(
    link: Mapping[str, Any],
    step_m: float,
    boundary_ids: Mapping[str, str],
    feature_by_id: Mapping[str, Mapping[str, Any]],
    policy: Mapping[str, Any],
) -> Tuple[List[List[float]], Dict[str, Any]]:
    """Refine a non-intersection link polygon toward trustworthy paint.

    Boundary points are converted to signed lateral offsets from the link
    centerline. Unsupported samples retain the MGeo width, so a broken marking
    never shortens the drivable polygon or bridges a long observation gap.
    Endpoint samples remain on the node normal and therefore keep flat caps.
    """

    center = _resample_centerline(_xy(link["points"]), step_m)
    tangent = np.gradient(center, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    start_width = float(link.get("width_start") or 3.5)
    end_width = float(link.get("width_end") or start_width)
    nominal = 0.5 * np.linspace(start_width, end_width, len(center))
    maximum = float(policy["maximum_distance_m"])
    minimum_coverage = float(policy["minimum_side_coverage"])
    window_points = max(1, int(round(float(policy["smoothing_window_m"]) / step_m)))
    side_points: Dict[str, np.ndarray] = {}
    details: Dict[str, Any] = {"mode": "reviewed_boundary_local_offset"}
    for side, sign in (("left", 1.0), ("right", -1.0)):
        offsets = nominal.copy()
        feature_id = str(boundary_ids.get(side, ""))
        valid = np.zeros(len(center), dtype=bool)
        if feature_id and feature_id in feature_by_id:
            coordinates = (feature_by_id[feature_id].get("geometry") or {}).get("coordinates", [])
            if len(coordinates) >= 2:
                boundary = _resample_centerline(_xy(coordinates), min(step_m, 0.25))
                delta = boundary[None, :, :] - center[:, None, :]
                signed = np.sum(delta * normal[:, None, :], axis=2) * sign
                distance = np.linalg.norm(delta, axis=2)
                eligible = (signed > 0.25) & (distance <= maximum)
                masked = np.where(eligible, distance, np.inf)
                nearest = np.argmin(masked, axis=1)
                best = masked[np.arange(len(center)), nearest]
                valid = np.isfinite(best)
                coverage = float(np.mean(valid))
                if coverage >= minimum_coverage:
                    measured = signed[np.arange(len(center)), nearest]
                    offsets[valid] = measured[valid]
                    offsets = _smooth_offsets(offsets, window_points)
                    offsets = np.clip(offsets, 0.5, maximum)
                else:
                    valid[:] = False
        coverage = float(np.mean(valid))
        details["%s_boundary_id" % side] = feature_id
        details["%s_refined" % side] = bool(coverage >= minimum_coverage)
        details["%s_coverage" % side] = round(coverage, 6)
        side_points[side] = center + sign * normal * offsets[:, None]
    ring = np.vstack([side_points["left"], side_points["right"][::-1]])
    return (
        [[round(float(x), 4), round(float(y), 4), 0.0] for x, y in ring],
        details,
    )


def corridor_bridge_polygon(
    first: Mapping[str, Any], second: Mapping[str, Any], step_m: float
) -> List[List[float]]:
    """Return the ruled vector strip between two reviewed adjacent links."""

    first_xy = _xy(first["points"])
    second_xy = _xy(second["points"])
    direct = float(
        np.linalg.norm(first_xy[0] - second_xy[0])
        + np.linalg.norm(first_xy[-1] - second_xy[-1])
    )
    reverse = float(
        np.linalg.norm(first_xy[0] - second_xy[-1])
        + np.linalg.norm(first_xy[-1] - second_xy[0])
    )
    if reverse < direct:
        second_xy = second_xy[::-1]
    count = max(2, int(math.ceil(max(_length(first_xy), _length(second_xy)) / step_m)) + 1)

    def normalized(points: np.ndarray) -> np.ndarray:
        segment = np.linalg.norm(np.diff(points, axis=0), axis=1)
        keep = segment > 1e-8
        clean = np.vstack([points[0], points[1:][keep]])
        stations = np.concatenate([[0.0], np.cumsum(segment[keep])])
        targets = np.linspace(0.0, stations[-1], count)
        return np.column_stack(
            [np.interp(targets, stations, clean[:, axis]) for axis in range(2)]
        )

    a = normalized(first_xy)
    b = normalized(second_xy)
    ring = np.vstack([a, b[::-1]])
    return [[round(float(x), 4), round(float(y), 4), 0.0] for x, y in ring]


def explicit_intersection_links(policy: Mapping[str, Any]) -> Tuple[List[str], Dict[str, str]]:
    groups = policy.get("explicit_intersection_groups") or {}
    selected: Dict[str, str] = {}
    for group, values in groups.items():
        for raw in values or []:
            link_id = str(raw)
            if link_id in selected and selected[link_id] != str(group):
                raise CompactDrivableError(
                    "intersection link belongs to multiple groups: %s" % link_id
                )
            selected[link_id] = str(group)
    return sorted(selected), selected


def apply_intersection_node_pair_closures(
    links: Mapping[str, Mapping[str, Any]],
    policy: Mapping[str, Any],
    intersection_types: set[str],
    group_by_link: Mapping[str, str],
) -> Tuple[List[str], Dict[str, str], Dict[str, Dict[str, Any]]]:
    """Expand reviewed anchors to their directed start-node/end-node closure.

    For anchors S_i -> T_i in one group, select only intersection links S -> T
    where S is an anchor start node and T is an anchor end node. This fills the
    reviewed junction's internal link mesh without selecting approach roads or
    any link merely because it is spatially nearby.
    """

    selected = dict(group_by_link)
    resolved: Dict[str, Dict[str, Any]] = {}
    for raw_group, spec in (policy.get("intersection_node_pair_closures") or {}).items():
        group = str(raw_group)
        anchor_ids = sorted({str(value) for value in (spec or {}).get("anchor_link_ids", [])})
        if not anchor_ids:
            raise CompactDrivableError("node-pair closure has no anchors: %s" % group)
        missing = [link_id for link_id in anchor_ids if link_id not in links]
        if missing:
            raise CompactDrivableError("node-pair closure anchor is missing: %s" % missing)
        wrong_group = [link_id for link_id in anchor_ids if selected.get(link_id) != group]
        if wrong_group:
            raise CompactDrivableError(
                "node-pair closure anchors must be explicit in group %s: %s"
                % (group, wrong_group)
            )
        start_nodes = sorted({str(links[link_id].get("from_node_idx")) for link_id in anchor_ids})
        end_nodes = sorted({str(links[link_id].get("to_node_idx")) for link_id in anchor_ids})
        start_set, end_set = set(start_nodes), set(end_nodes)
        closure_ids = sorted(
            link_id
            for link_id, link in links.items()
            if str(link.get("link_type")) in intersection_types
            and str(link.get("from_node_idx")) in start_set
            and str(link.get("to_node_idx")) in end_set
        )
        for link_id in closure_ids:
            existing = selected.get(link_id)
            if existing is not None and existing != group:
                raise CompactDrivableError(
                    "node-pair closure conflicts with group %s: %s is in %s"
                    % (group, link_id, existing)
                )
            selected[link_id] = group
        resolved[group] = {
            "mode": "directed_start_end_node_pair_closure",
            "anchor_link_ids": anchor_ids,
            "start_node_ids": start_nodes,
            "end_node_ids": end_nodes,
            "derived_link_ids": sorted(set(closure_ids) - set(anchor_ids)),
            "resolved_link_ids": closure_ids,
        }
    return sorted(selected), selected, resolved


def explicit_lane_links(policy: Mapping[str, Any]) -> Tuple[List[str], Dict[str, str]]:
    selected: Dict[str, str] = {}
    for group, values in (policy.get("explicit_lane_groups") or {}).items():
        for raw in values or []:
            link_id = str(raw)
            if link_id in selected and selected[link_id] != str(group):
                raise CompactDrivableError("explicit lane belongs to multiple groups: %s" % link_id)
            selected[link_id] = str(group)
    return sorted(selected), selected


def selected_corridor_bridge_pairs(
    links: Mapping[str, Mapping[str, Any]],
    lane_ids: Sequence[str],
    policy: Mapping[str, Any],
) -> List[Dict[str, Any]]:
    """Resolve unique topology and explicitly reviewed lane-surface pairs."""

    lane_set = set(lane_ids)
    pairs: Dict[Tuple[str, str], Dict[str, Any]] = {}
    if bool(policy.get("bridge_selected_adjacent_lanes", False)):
        for link_id in lane_ids:
            link = links[link_id]
            for key in ("left_lane_change_dst_link_idx", "right_lane_change_dst_link_idx"):
                adjacent = str(link.get(key) or "")
                if adjacent not in lane_set or adjacent == link_id:
                    continue
                pair = tuple(sorted((link_id, adjacent)))
                pairs[pair] = {
                    "id": "mgeo_adjacent:%s:%s" % pair,
                    "group": "selected_adjacent_lane_fill",
                    "link_ids": list(pair),
                    "source": "mgeo_lane_change_adjacency",
                }
    for item in policy.get("explicit_bridge_pairs") or []:
        values = [str(value) for value in item.get("link_ids", [])]
        if len(values) != 2 or values[0] == values[1]:
            raise CompactDrivableError("explicit bridge pair must contain two distinct links")
        missing = [value for value in values if value not in lane_set]
        if missing:
            raise CompactDrivableError("explicit bridge pair link is not selected: %s" % missing)
        pair = tuple(sorted(values))
        pairs[pair] = {
            "id": str(item["id"]),
            "group": str(item["group"]),
            "link_ids": list(pair),
            "source": "explicit_reviewed_pair",
        }
    return [pairs[key] for key in sorted(pairs)]


def _polygon_parts(geometry: Any) -> List[Any]:
    if geometry.is_empty:
        return []
    if geometry.geom_type == "Polygon":
        return [geometry]
    if geometry.geom_type in ("MultiPolygon", "GeometryCollection"):
        output = []
        for value in geometry.geoms:
            output.extend(_polygon_parts(value))
        return output
    return []


def _legacy_surface_geometry(
    road: np.ndarray, ppm: float, offset: np.ndarray, bounds: Sequence[float]
) -> Any:
    """Vectorize only the legacy-road crop needed by one reviewed envelope."""

    try:
        from shapely.geometry import Polygon
        from shapely.ops import unary_union
    except ImportError as error:
        raise CompactDrivableError("python3-shapely is required for surface refinement") from error
    minimum = np.floor((np.asarray(bounds[:2]) - offset) * ppm).astype(int) - 2
    maximum = np.ceil((np.asarray(bounds[2:]) - offset) * ppm).astype(int) + 2
    minimum = np.maximum(minimum, [0, 0])
    maximum = np.minimum(maximum, [road.shape[1], road.shape[0]])
    x0, y0 = minimum
    x1, y1 = maximum
    crop = np.where(road[y0:y1, x0:x1] > 0, 255, 0).astype(np.uint8)
    contours, hierarchy = cv2.findContours(crop, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return Polygon()
    hierarchy = hierarchy[0]
    polygons = []
    for index, contour in enumerate(contours):
        if hierarchy[index][3] != -1 or len(contour) < 3:
            continue
        shell_pixels = contour[:, 0, :].astype(np.float64) + np.asarray([x0, y0])
        shell = shell_pixels / ppm + offset[None, :]
        holes = []
        child = int(hierarchy[index][2])
        while child != -1:
            if len(contours[child]) >= 3:
                hole_pixels = contours[child][:, 0, :].astype(np.float64) + np.asarray([x0, y0])
                holes.append(hole_pixels / ppm + offset[None, :])
            child = int(hierarchy[child][0])
        polygon = Polygon(shell, holes).buffer(0)
        if not polygon.is_empty:
            polygons.extend(_polygon_parts(polygon))
    return unary_union(polygons) if polygons else Polygon()


def legacy_intersection_surface_patches(
    lanes: Sequence[Mapping[str, Any]],
    road: np.ndarray,
    ppm: float,
    offset: np.ndarray,
    group_policies: Sequence[Mapping[str, Any]],
    minimum_area_m2: float,
    simplify_tolerance_m: float,
) -> List[Dict[str, Any]]:
    """Fill H5-supported gaps inside explicitly reviewed link envelopes.

    Obstacle junctions intentionally close holes between selected maneuver
    links.  A roundabout is different: its center island must remain
    non-drivable, while the original H5 pavement just outside the nominal link
    width is useful.  Per-group envelope padding and hole preservation keep
    those two policies separate.
    """

    try:
        from shapely.geometry import Polygon
        from shapely.ops import triangulate, unary_union
    except ImportError as error:
        raise CompactDrivableError("python3-shapely is required for surface refinement") from error
    output: List[Dict[str, Any]] = []
    for raw_policy in group_policies:
        group = str(raw_policy["group"])
        envelope_buffer_m = float(raw_policy.get("envelope_buffer_m", 0.0))
        preserve_holes = bool(raw_policy.get("preserve_holes", False))
        if envelope_buffer_m < 0.0:
            raise CompactDrivableError("surface envelope buffer must be non-negative")
        supplemental_link_ids = {
            str(value) for value in raw_policy.get("supplemental_link_ids", [])
        }
        selected = [
            lane for lane in lanes
            if lane.get("drivable_group") == str(group)
            or lane.get("link_id") in supplemental_link_ids
        ]
        if not selected:
            raise CompactDrivableError("surface group has no lanes: %s" % group)
        missing_supplemental = supplemental_link_ids.difference(
            str(lane.get("link_id")) for lane in selected
        )
        if missing_supplemental:
            raise CompactDrivableError(
                "surface supplemental links are not selected lanes: %s"
                % sorted(missing_supplemental)
            )
        lane_geometry = {
            lane["link_id"]: Polygon([point[:2] for point in lane["polygon"]]).buffer(0)
            for lane in selected
        }
        union = unary_union(list(lane_geometry.values())).buffer(0)
        for component_index, component in enumerate(_polygon_parts(union)):
            component_links = sorted(
                link_id for link_id, polygon in lane_geometry.items()
                if polygon.intersects(component)
            )
            envelope = component.convex_hull.buffer(envelope_buffer_m)
            legacy = _legacy_surface_geometry(road, ppm, offset, envelope.bounds)
            target = component.union(legacy.intersection(envelope)).buffer(0)
            patch = target.difference(component).buffer(0)
            for patch_index, polygon in enumerate(_polygon_parts(patch)):
                if float(polygon.area) < minimum_area_m2:
                    continue
                simplified = polygon.simplify(simplify_tolerance_m, preserve_topology=True)
                pieces = _polygon_parts(simplified)
                if preserve_holes:
                    hole_safe_pieces = []
                    for piece in pieces:
                        if not piece.interiors:
                            hole_safe_pieces.append(piece)
                            continue
                        for triangle in triangulate(piece):
                            clipped = triangle.intersection(piece).buffer(0)
                            hole_safe_pieces.extend(_polygon_parts(clipped))
                    pieces = hole_safe_pieces
                for piece_index, piece in enumerate(pieces):
                    if float(piece.area) < minimum_area_m2:
                        continue
                    # A DrivableArea is a simple open ring. Surface patch holes
                    # are emitted as their surrounding H5-supported pavement;
                    # obstacle-junction holes between selected lanes are the
                    # exact gaps this stage is intended to close.
                    candidate_ring = [
                        [round(float(x), 4), round(float(y), 4), 0.0]
                        for x, y in list(piece.exterior.coords)[:-1]
                    ]
                    rounded = Polygon([value[:2] for value in candidate_ring]).buffer(0)
                    for final_index, final_piece in enumerate(_polygon_parts(rounded)):
                        if float(final_piece.area) < minimum_area_m2:
                            continue
                        ring = [
                            [round(float(x), 4), round(float(y), 4), 0.0]
                            for x, y in list(final_piece.exterior.coords)[:-1]
                        ]
                        output.append({
                            "id": "drivable:surface:%s:%02d:%02d:%02d:%02d" % (
                                group, component_index, patch_index, piece_index, final_index
                            ),
                            "area_type": "junction",
                            "validity": "validated",
                            "source_type": "hybrid",
                            "polygon": ring,
                            "lane_ids": ["lane:%s" % value for value in component_links],
                            "link_ids": component_links,
                            "intersection_guarded": True,
                            "source_dataset": "legacy_h5_clipped_to_selected_link_envelope",
                            "source_feature_id": "surface:%s:%02d" % (group, component_index),
                            "surface_group": str(group),
                            "surface_area_m2": round(float(final_piece.area), 4),
                            "surface_envelope_buffer_m": envelope_buffer_m,
                            "surface_preserves_holes": preserve_holes,
                            "surface_supplemental_link_ids": sorted(supplemental_link_ids),
                        })
    return output


def _reviewed_boundaries_by_link(
    reviewed: Mapping[str, Any], links: Mapping[str, Mapping[str, Any]]
) -> Dict[str, Dict[str, str]]:
    """Choose the nearest reviewed marking on each side without bridging gaps."""

    candidates: Dict[str, List[Mapping[str, Any]]] = defaultdict(list)
    for feature in reviewed.get("features", []):
        if str(feature.get("feature_type")) != "lane_marking":
            continue
        for link_id in (feature.get("topology") or {}).get("current_link_ids", []):
            if str(link_id) in links:
                candidates[str(link_id)].append(feature)
    output: Dict[str, Dict[str, str]] = {}
    for link_id, features in candidates.items():
        centers = _resample_centerline(_xy(links[link_id]["points"]), 2.0)
        tangent = np.gradient(centers, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        best: Dict[str, Tuple[float, str]] = {}
        for feature in features:
            marking = _resample_centerline(
                _xy((feature.get("geometry") or {}).get("coordinates", [])), 1.0
            )
            delta = marking[None, :, :] - centers[:, None, :]
            distance = np.linalg.norm(delta, axis=2)
            nearest = np.argmin(distance, axis=1)
            chosen = delta[np.arange(len(centers)), nearest]
            cross = tangent[:, 0] * chosen[:, 1] - tangent[:, 1] * chosen[:, 0]
            side = "left" if float(np.median(cross)) > 0.0 else "right"
            score = float(np.median(distance[np.arange(len(centers)), nearest]))
            if side not in best or score < best[side][0]:
                best[side] = (score, str(feature["id"]))
        output[link_id] = {side: value[1] for side, value in best.items()}
    return output


def _lane_record(
    link: Mapping[str, Any], polygon: List[List[float]], route_range: Mapping[str, Any] | None,
    selected: set[str], reviewed_boundaries: Mapping[str, str], drivable_group: str,
    polygon_refinement: Mapping[str, Any] | None = None,
) -> Dict[str, Any]:
    link_id = str(link["idx"])
    points = _resample_centerline(_xy(link["points"]), 1.0)
    lane = int(link.get("ego_lane") or 0)
    road_id = str(link.get("road_id") or "")
    return {
        "id": "lane:%s" % link_id,
        "link_id": link_id,
        "lane_type": _lane_type(str(link.get("link_type") or "")),
        "direction": "forward",
        "lane_number": lane,
        "lane_number_scope_id": road_id,
        "route_station_start_m": round(float(route_range["route_start_m"]), 6) if route_range else -1.0,
        "route_station_end_m": round(float(route_range["route_end_m"]), 6) if route_range else -1.0,
        "left_boundary_id": str(reviewed_boundaries.get("left", "")),
        "right_boundary_id": str(reviewed_boundaries.get("right", "")),
        # Raw MGeo IDs remain provenance only; runtime boundary IDs above refer
        # exclusively to published reviewed LaneMarking IDs.
        "source_left_boundary_ids": _boundary_ids(link, "left"),
        "source_right_boundary_ids": _boundary_ids(link, "right"),
        "left_adjacent_lane_id": (
            "lane:%s" % link["left_lane_change_dst_link_idx"]
            if link.get("left_lane_change_dst_link_idx") in selected else ""
        ),
        "right_adjacent_lane_id": (
            "lane:%s" % link["right_lane_change_dst_link_idx"]
            if link.get("right_lane_change_dst_link_idx") in selected else ""
        ),
        "predecessor_lane_ids": [],
        "successor_lane_ids": [],
        "centerline": [[round(float(x), 4), round(float(y), 4), 0.0] for x, y in points],
        "polygon": polygon,
        "source_type": "derived",
        "source_dataset": (
            "reviewed_lane_boundary_refined_vector_bake"
            if polygon_refinement and (
                polygon_refinement.get("left_refined")
                or polygon_refinement.get("right_refined")
            )
            else "current_mgeo_link_centerline_width_vector_bake"
        ),
        "source_feature_id": link_id,
        "drivable_group": drivable_group,
        "polygon_refinement": dict(polygon_refinement or {"mode": "mgeo_link_width"}),
    }


def _attach_topology(lanes: List[Dict[str, Any]], links: Mapping[str, Mapping[str, Any]]) -> None:
    lane_by_link = {item["link_id"]: item for item in lanes}
    from_node: Dict[str, List[str]] = defaultdict(list)
    to_node: Dict[str, List[str]] = defaultdict(list)
    for link_id in lane_by_link:
        link = links[link_id]
        from_node[str(link.get("from_node_idx"))].append(link_id)
        to_node[str(link.get("to_node_idx"))].append(link_id)
    for link_id, lane in lane_by_link.items():
        link = links[link_id]
        lane["predecessor_lane_ids"] = [
            "lane:%s" % value for value in sorted(to_node[str(link.get("from_node_idx"))]) if value != link_id
        ]
        lane["successor_lane_ids"] = [
            "lane:%s" % value for value in sorted(from_node[str(link.get("to_node_idx"))]) if value != link_id
        ]


def build_compact_drivable(repo_root: Path, source_config: Path) -> Tuple[Dict[str, Any], Dict[str, np.ndarray]]:
    repo_root = repo_root.resolve()
    config = load_yaml(source_config)
    sources = config["sources"]
    paths = {name: _path(repo_root, spec) for name, spec in sources.items()}
    audit = load_yaml(paths["input_audit"])
    metadata = load_yaml(paths["competition_metadata"])
    links_list = json.loads(paths["current_link_set"].read_text(encoding="utf-8"))
    reviewed = json.loads(paths["reviewed_lane_markings"].read_text(encoding="utf-8"))
    links = {str(item["idx"]): item for item in links_list}
    route_ids = [str(item["link_id"]) for item in audit["route_links"]]
    if route_ids != [str(value) for value in metadata["route"]["link_ids"]]:
        raise CompactDrivableError("audit and competition route differ")
    actual_reviewed_sha = geometry_sha256(reviewed)
    if str(audit["reviewed_hd_map"]["geometry_sha256"]) != actual_reviewed_sha:
        raise CompactDrivableError("reviewed geometry SHA differs from P2-A audit")

    policy = config["selection"]
    if str(policy.get("endpoint_cap_style")) != "node_normal_flat":
        raise CompactDrivableError("endpoint_cap_style must be node_normal_flat")
    exclusions = {str(value) for value in policy.get("excluded_lane_links", [])}
    same_direction_ids, rejected = _same_direction_siblings(
        links, route_ids, exclusions, float(policy["minimum_direction_dot"])
    )
    support_ids, group_by_link = explicit_intersection_links(policy)
    intersection_types = {str(value) for value in policy["intersection_link_types"]}
    support_ids, group_by_link, node_pair_closures = apply_intersection_node_pair_closures(
        links, policy, intersection_types, group_by_link
    )
    missing_support = [link_id for link_id in support_ids if link_id not in links]
    if missing_support:
        raise CompactDrivableError("explicit intersection link is missing: %s" % missing_support)
    overlap = sorted(exclusions.intersection(support_ids))
    if overlap:
        raise CompactDrivableError("explicit intersection link is also excluded: %s" % overlap)
    invalid_support = [
        link_id for link_id in support_ids
        if str(links[link_id].get("link_type")) not in intersection_types
    ]
    if invalid_support:
        raise CompactDrivableError("explicit support is not an intersection link: %s" % invalid_support)
    explicit_lane_ids, explicit_lane_groups = explicit_lane_links(policy)
    missing_explicit_lanes = [link_id for link_id in explicit_lane_ids if link_id not in links]
    if missing_explicit_lanes:
        raise CompactDrivableError("explicit lane link is missing: %s" % missing_explicit_lanes)
    excluded_explicit_lanes = sorted(exclusions.intersection(explicit_lane_ids))
    if excluded_explicit_lanes:
        raise CompactDrivableError(
            "explicit lane link is also excluded: %s" % excluded_explicit_lanes
        )
    group_conflicts = sorted(set(group_by_link).intersection(explicit_lane_groups))
    if group_conflicts:
        raise CompactDrivableError("lane/intersection group conflict: %s" % group_conflicts)
    group_by_link.update(explicit_lane_groups)
    route_set = set(route_ids)
    # Route links are always retained. A non-route intersection can enter only
    # through the reviewed allowlist, never because it happens to be nearby or
    # shares a road_id with the route.
    filtered_same_direction = {
        link_id for link_id in same_direction_ids
        if link_id in route_set
        or str(links[link_id].get("link_type")) not in intersection_types
        or link_id in group_by_link
    }
    rejected = sorted(set(rejected).union(set(same_direction_ids) - filtered_same_direction))
    lane_set = filtered_same_direction.union(support_ids).union(explicit_lane_ids)
    lane_ids = sorted(lane_set)

    h5 = _parse_h5(repo_root, paths["legacy_drivable_h5"])
    road, legacy_ppm, offset = h5["road"], h5["ppm"], h5["offset"]
    grid_resolution = float(config["raster"]["output_resolution_m"])
    grid_ppm = 1.0 / grid_resolution
    extent_m = np.asarray([road.shape[1], road.shape[0]], dtype=np.float64) / legacy_ppm
    grid_shape = (
        int(math.ceil(extent_m[1] * grid_ppm)),
        int(math.ceil(extent_m[0] * grid_ppm)),
    )
    compact = np.zeros(grid_shape, dtype=np.uint8)
    step_m = float(policy["centerline_resample_m"])
    surface_policy = config["surface_refinement"]
    boundary_policy = surface_policy["reviewed_boundary_snap"]
    reviewed_boundaries = _reviewed_boundaries_by_link(reviewed, links)
    feature_by_id = {str(item["id"]): item for item in reviewed.get("features", [])}
    polygons_by_link: Dict[str, List[List[float]]] = {}
    refinement_by_link: Dict[str, Dict[str, Any]] = {}
    for link_id in lane_ids:
        link = links[link_id]
        if (
            bool(boundary_policy.get("enabled", False))
            and str(link.get("link_type")) not in intersection_types
        ):
            polygon, refinement = lane_boundary_polygon(
                link, step_m, reviewed_boundaries.get(link_id, {}),
                feature_by_id, boundary_policy,
            )
        else:
            polygon = lane_width_polygon(link, step_m)
            refinement = {"mode": "mgeo_link_width"}
        polygons_by_link[link_id] = polygon
        refinement_by_link[link_id] = refinement

    range_by_id = _route_range_by_id(audit)
    lanes = []
    for link_id in lane_ids:
        group = group_by_link.get(
            link_id,
            "route" if link_id in route_set else "same_direction_adjacent",
        )
        lanes.append(_lane_record(
            links[link_id], polygons_by_link[link_id], range_by_id.get(link_id), lane_set,
            reviewed_boundaries.get(link_id, {}), group, refinement_by_link[link_id],
        ))
    _attach_topology(lanes, links)

    areas = []
    for lane in lanes:
        link_type = str(links[lane["link_id"]].get("link_type"))
        is_intersection = link_type in intersection_types
        areas.append({
            "id": "drivable:%s" % lane["link_id"],
            "area_type": "junction" if is_intersection else "lane",
            "validity": "validated", "source_type": "lane_clipped", "polygon": lane["polygon"],
            "lane_ids": [lane["id"]], "link_ids": [lane["link_id"]],
            "intersection_guarded": is_intersection,
            "source_dataset": lane["source_dataset"],
            "source_feature_id": lane["link_id"],
            "surface_group": lane["drivable_group"],
        })
    bridge_pairs = selected_corridor_bridge_pairs(
        links, lane_ids, {
            "bridge_selected_adjacent_lanes": surface_policy["bridge_selected_adjacent_lanes"],
            "explicit_bridge_pairs": surface_policy.get("explicit_bridge_pairs", []),
        }
    )
    for index, pair in enumerate(bridge_pairs):
        first, second = pair["link_ids"]
        polygon = corridor_bridge_polygon(links[first], links[second], step_m)
        guarded = any(
            str(links[link_id].get("link_type")) in intersection_types
            for link_id in pair["link_ids"]
        )
        areas.append({
            "id": "drivable:corridor_fill:%03d" % index,
            "area_type": "junction" if guarded else "lane",
            "validity": "validated",
            "source_type": "lane_clipped",
            "polygon": polygon,
            "lane_ids": ["lane:%s" % value for value in pair["link_ids"]],
            "link_ids": pair["link_ids"],
            "intersection_guarded": guarded,
            "source_dataset": "selected_lane_topology_corridor_fill",
            "source_feature_id": pair["id"],
            "surface_group": pair["group"],
        })
    intersection_surface_patches = legacy_intersection_surface_patches(
        lanes,
        road,
        legacy_ppm,
        offset,
        surface_policy["legacy_h5_intersection_surface_groups"],
        float(surface_policy["minimum_patch_area_m2"]),
        float(surface_policy["vector_simplify_tolerance_m"]),
    )
    areas.extend(intersection_surface_patches)
    for area in areas:
        polygon = np.asarray(area["polygon"], dtype=np.float64)
        if len(polygon) < 3:
            raise CompactDrivableError("drivable area polygon has fewer than three points")
        pixels = _local_to_pixel(polygon, grid_ppm, offset)
        cv2.fillPoly(compact, [pixels.reshape((-1, 1, 2))], 255, cv2.LINE_8)
    if not np.any(compact):
        raise CompactDrivableError("compact drivable mask is empty")
    grid = np.where(compact > 0, 0, 100).astype(np.int8)
    missing_checkpoints = []
    for checkpoint in metadata.get("checkpoints", []):
        position = checkpoint["pose"]["position"]
        pixel = _local_to_pixel(
            np.asarray([[position["x"], position["y"]]], dtype=np.float64), grid_ppm, offset
        )[0]
        inside = (
            0 <= pixel[0] < compact.shape[1] and 0 <= pixel[1] < compact.shape[0]
            and compact[pixel[1], pixel[0]] > 0
        )
        if not inside:
            missing_checkpoints.append(str(checkpoint["id"]))
    if missing_checkpoints:
        raise CompactDrivableError("checkpoints outside compact road: %s" % missing_checkpoints)
    product_core = {
        "schema_version": "asmc.hd_map.compact_drivable/v5",
        "build_id": str(config["build_id"]),
        "map_id": str(h5["map_id"]),
        "frame_id": "map",
        "reviewed_geometry_sha256": actual_reviewed_sha,
        "selection": {
            "route_link_ids": route_ids,
            "selected_lane_link_ids": lane_ids,
            "endpoint_cap_style": "node_normal_flat",
            "intersection_support_link_ids": support_ids,
            "intersection_groups": {
                group: sorted(
                    link_id for link_id in support_ids
                    for selected_group in [group_by_link[link_id]]
                    if selected_group == group
                )
                for group in sorted({group_by_link[link_id] for link_id in support_ids})
            },
            "intersection_node_pair_closures": node_pair_closures,
            "explicit_lane_groups": {
                group: sorted(
                    link_id for link_id, selected_group in explicit_lane_groups.items()
                    if selected_group == group
                )
                for group in sorted(set(explicit_lane_groups.values()))
            },
            "surface_bridge_pairs": bridge_pairs,
            "rejected_same_road_link_ids": rejected,
            "explicit_excluded_link_ids": sorted(exclusions),
        },
        "lanes": lanes,
        "drivable_areas": areas,
        "grid": {
            "path": str(config["outputs"]["grid"]),
            "resolution_m": grid_resolution,
            "width": int(grid.shape[1]), "height": int(grid.shape[0]),
            "origin_x": float(offset[0]), "origin_y": float(offset[1]),
            "canonical_representation": False,
            "derived_from_vector_product": True,
            "values": {"drivable": 0, "non_drivable": 100, "unknown": -1},
        },
        "statistics": {
            "route_lane_count": len(route_ids),
            "selected_lane_count": len(lanes),
            "same_direction_extra_lane_count": len(filtered_same_direction - route_set),
            "explicit_non_route_intersection_lane_count": len(set(support_ids) - route_set),
            "intersection_support_link_count": len(support_ids),
            "explicit_additional_lane_count": len(explicit_lane_ids),
            "drivable_area_count": len(areas),
            "base_lane_area_count": len(lanes),
            "corridor_fill_area_count": len(bridge_pairs),
            "intersection_surface_patch_count": len(intersection_surface_patches),
            "boundary_refined_lane_count": sum(
                bool(value.get("left_refined") or value.get("right_refined"))
                for value in refinement_by_link.values()
            ),
            "boundary_refined_side_count": sum(
                bool(value.get("%s_refined" % side))
                for value in refinement_by_link.values()
                for side in ("left", "right")
            ),
            "checkpoint_centers_inside": len(metadata.get("checkpoints", [])),
            "legacy_road_area_m2": round(float(np.count_nonzero(road)) / legacy_ppm ** 2, 2),
            "compact_road_area_m2": round(float(np.count_nonzero(compact)) / grid_ppm ** 2, 2),
            "compact_fraction_of_legacy": round(
                (float(np.count_nonzero(compact)) / grid_ppm ** 2)
                / (float(np.count_nonzero(road)) / legacy_ppm ** 2), 6
            ),
        },
        "sources": {
            name: {"path": str(path.relative_to(repo_root)), "sha256": sha256_file(path)}
            for name, path in sorted(paths.items())
        },
        "limitations": [
            "Lane/DrivableArea vectors are canonical; OccupancyGrid is their 0.20 m derivative.",
            "Every link-width polygon terminates flat on its endpoint-node normal.",
            "The reviewed dynamic junction uses directed node-pair closure links to avoid internal seams.",
            "Ordinary-road polygons locally follow reviewed paint where its signed lateral support is valid.",
            "Selected adjacent lane pairs are joined by canonical vector corridor-fill areas.",
            "Legacy H5 contributes only gap patches inside explicitly reviewed envelopes.",
            "A134 roundabout pavement uses the legacy H5 only inside its 2.0 m padded envelope plus the selected A151 stitch envelope, and preserves the center island.",
            "A131 is available only as the explicitly reviewed opposite lane beside shadow-zone A126.",
            "Explicit obstacle-intersection links describe maneuverable space, not permission to leave mission policy.",
        ],
    }
    product_core["product_sha256"] = canonical_sha256(product_core)
    arrays = {"occupancy": grid, "compact_mask": compact}
    return product_core, arrays


def write_product(repo_root: Path, source_config: Path) -> Dict[str, Any]:
    config = load_yaml(source_config)
    product, arrays = build_compact_drivable(repo_root, source_config)
    product_path = (repo_root / str(config["outputs"]["product"])).resolve()
    grid_path = (repo_root / str(config["outputs"]["grid"])).resolve()
    product_path.parent.mkdir(parents=True, exist_ok=True)
    grid_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        str(grid_path), occupancy=arrays["occupancy"],
        resolution_m=np.float64(product["grid"]["resolution_m"]),
        origin_xy=np.asarray([product["grid"]["origin_x"], product["grid"]["origin_y"]]),
        frame_id=np.asarray([product["frame_id"]]),
    )
    product["grid"]["sha256"] = sha256_file(grid_path)
    product["product_sha256"] = canonical_sha256({k: v for k, v in product.items() if k != "product_sha256"})
    product_path.write_text(json.dumps(product, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return product


def render_report(product: Mapping[str, Any]) -> str:
    stats = product["statistics"]
    return "\n".join([
        "# K-City 2025 P2-B v5 A134 로터리·A151 접속부 주행가능영역",
        "",
        "> **상태:** 생성·검증 가능한 P2-B product. reviewed 차선 geometry는 변경하지 않음.",
        "",
        "## 선택 정책",
        "",
        "- 전역경로 39개 링크와 같은 방향의 사용 가능 동일-road 차로를 기본 Lane으로 채택했다.",
        "- Lane과 추가 surface patch를 합친 vector polygon 집합이 geometry 정본이다.",
        "- polygon 끝은 round buffer가 아니라 endpoint node의 법선에서 flat cap으로 절단한다.",
        "- 일반도로는 신뢰 가능한 reviewed paint의 국소 횡거리로 41개 Lane의 경계를 보정했다.",
        "- 선택된 인접 차로 17쌍과 음영구간 A126/A131 사이를 vector corridor로 채웠다.",
        "- 교차로의 거리/반경 확장을 폐기하고 reviewed link set만 추가했다.",
        "- 동적 장애물 교차로는 검토된 4개 anchor의 directed start/end node-pair closure 8개 link를 사용한다.",
        "- 정적·동적 교차로는 선택 link envelope 안의 legacy H5 road만 gap patch로 사용한다.",
        "- A134 로터리는 2.0 m 확장 link envelope와 A151 접속부 안의 legacy H5 포장면을 사용하고 중앙 섬은 비주행으로 보존한다.",
        "- 일반 교차로는 전역경로 link만, 정적·동적 장애물 교차로는 검토된 회피 link만 포함한다.",
        "- 반대차로는 음영구간 A131만 명시적으로 포함하고, 그 밖의 대회 외 도로는 제거했다.",
        "- `OccupancyGrid`는 vector product를 0.20 m/px로 rasterize한 파생 표현이다.",
        "",
        "## 결과",
        "",
        "- route lanes: `%d`, selected lanes: `%d` (extra same-direction `%d`)" % (
            stats["route_lane_count"], stats["selected_lane_count"], stats["same_direction_extra_lane_count"]),
        "- intersection support links: `%d`, vector areas: `%d`" % (
            stats["intersection_support_link_count"], stats["drivable_area_count"]),
        "- base lane areas: `%d`, adjacent corridor fills: `%d`, intersection gap patches: `%d`" % (
            stats["base_lane_area_count"], stats["corridor_fill_area_count"],
            stats["intersection_surface_patch_count"]),
        "- reviewed-boundary refinement: `%d` lanes / `%d` sides" % (
            stats["boundary_refined_lane_count"], stats["boundary_refined_side_count"]),
        "- explicit shadow opposite lane: `A2256W000131` beside route `A2256W000126`",
        "- reviewed non-route intersection lanes: `%d`" %
        stats["explicit_non_route_intersection_lane_count"],
        "- static allowlist: `%s`" % ", ".join(
            product["selection"]["intersection_groups"]["static_obstacle_maneuver"]
        ),
        "- dynamic allowlist: `%s`" % ", ".join(
            product["selection"]["intersection_groups"]["dynamic_obstacle_maneuver"]
        ),
        "- roundabout extension: `%s`" % ", ".join(
            product["selection"]["intersection_groups"]["roundabout_maneuver"]
        ),
        "- roundabout surface stitch: `A2256W000151` (envelope reference only; not a new lane)",
        "- legacy road: `%.2f m²`, compact road: `%.2f m²` (`%.2f%%`)" % (
            stats["legacy_road_area_m2"], stats["compact_road_area_m2"],
            100.0 * stats["compact_fraction_of_legacy"]),
        "- reviewed geometry SHA: `%s`" % product["reviewed_geometry_sha256"],
        "- product SHA: `%s`" % product["product_sha256"],
        "- grid SHA: `%s`" % product["grid"]["sha256"],
        "",
        "## 소비 규칙",
        "",
        "- 판제의 geometry 정본은 `/hd_map/global/lanes`와 `/hd_map/global/drivable_areas`다.",
        "- `/hd_map/global/drivable_grid`는 빠른 lookup용 0.20 m 파생 raster다.",
        "- `intersection_guarded=true` 영역은 allowlist 기반 공간이며 미션 policy 밖 이탈 허가가 아니다.",
        "",
    ])
