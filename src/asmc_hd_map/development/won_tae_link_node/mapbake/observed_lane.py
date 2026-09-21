"""수집 CSV 차선을 링크 기반 lane prior 위에 선택적으로 덮어쓴다."""
from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple

import cv2
import numpy as np

from .raster import RasterGrid, draw_polyline
from .topology import normals, resample_polyline


@dataclass
class ObservedLaneResult:
    image: np.ndarray
    observed: np.ndarray
    observed_white: np.ndarray
    observed_yellow: np.ndarray
    observed_road: np.ndarray
    erase_mask: np.ndarray
    removed_prior: np.ndarray
    features: List[Dict[str, Any]]
    stats: Dict[str, Any]


@dataclass
class ObservedRoadTrimResult:
    image: np.ndarray
    removed_mask: np.ndarray
    excluded_lane_mask: np.ndarray
    stats: Dict[str, Any]


def restore_trimmed_road_on_links(
    trimmed_road: np.ndarray,
    source_road: np.ndarray,
    grid: RasterGrid,
    links: Iterable[Mapping[str, Any]],
    *,
    width_scale: float = 1.0,
) -> Tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
    """외곽선 오판으로 잘린 특정 유효 링크의 원래 road만 복구한다.

    새 영역을 임의로 칠하지 않고 ``source_road``에 원래 존재했던 픽셀 중 지정한
    링크 폭 안에서 trim 단계가 제거한 픽셀만 되살린다. 따라서 수동 이미지 편집과
    달리 재베이크해도 동일한 월드 좌표에 영구적으로 적용된다.
    """
    if width_scale <= 0:
        raise ValueError('width_scale must be positive')

    trimmed = np.ascontiguousarray(trimmed_road, dtype=np.uint8)
    source = np.ascontiguousarray(source_road, dtype=np.uint8)
    if trimmed.shape != source.shape:
        raise ValueError('trimmed_road and source_road shapes must match')

    corridor = np.zeros_like(trimmed)
    link_ids: List[str] = []
    for link in links:
        points = np.asarray(link['points'], dtype=np.float64).reshape(-1, 2)
        if len(points) < 2:
            continue
        width_m = float(link['width_m']) * float(width_scale)
        draw_polyline(corridor, grid, points,
                      grid.meters_to_px(width_m, minimum=1))
        link_ids.append(str(link['idx']))

    restore_mask = cv2.bitwise_and(source, corridor)
    restore_mask = cv2.bitwise_and(
        restore_mask, cv2.bitwise_not(trimmed))
    restored = cv2.bitwise_or(trimmed, restore_mask)
    restored_px = int((restore_mask > 0).sum())
    return restored, restore_mask, {
        'links': link_ids,
        'width_scale': float(width_scale),
        'restored_px': restored_px,
        'restored_m2': round(
            restored_px / (grid.pixels_per_meter ** 2), 2),
    }


def exclude_road_between_observed_boundaries(
    road: np.ndarray,
    grid: RasterGrid,
    features: Iterable[Mapping[str, Any]],
    boundary_pairs: Iterable[Mapping[str, Any]],
) -> Tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
    """두 실측 경계가 동시에 존재하는 구간 사이를 주행불가로 만든다.

    한 경계가 다른 경계보다 길게 수집됐으면 짧은 경계의 양 끝과 가장 가까운
    지점까지만 긴 경계를 잘라 polygon을 만든다. 따라서 두 선이 함께 없는 구간까지
    삼각형으로 과도하게 잘리는 것을 막는다.
    """
    base = np.ascontiguousarray(road, dtype=np.uint8)
    feature_list = list(features)
    exclusion = np.zeros_like(base)
    pair_stats: List[Dict[str, Any]] = []

    def select(selector: Mapping[str, Any]) -> Mapping[str, Any]:
        wanted_source = str(selector['source'])
        wanted_side = str(selector.get('side', '')).upper()
        wanted_color = str(selector.get('color', '')).upper()
        matches = []
        for feature in feature_list:
            source = str(feature.get('source', ''))
            source_matches = (source == wanted_source
                              or Path(source).name == Path(wanted_source).name)
            if not source_matches:
                continue
            if wanted_side and str(feature.get('side', '')).upper() != wanted_side:
                continue
            if wanted_color and str(feature.get('color', '')).upper() != wanted_color:
                continue
            matches.append(feature)
        if len(matches) != 1:
            raise ValueError(
                f'road exclusion boundary {dict(selector)} matched '
                f'{len(matches)} features; expected exactly one')
        return matches[0]

    def length(points: np.ndarray) -> float:
        return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())

    for pair in boundary_pairs:
        selectors = list(pair.get('boundaries') or ())
        if len(selectors) != 2:
            raise ValueError('road exclusion pair needs exactly two boundaries')
        first_feature, second_feature = map(select, selectors)
        first = np.asarray(first_feature['points'], dtype=np.float64).reshape(-1, 2)
        second = np.asarray(second_feature['points'], dtype=np.float64).reshape(-1, 2)
        if len(first) < 2 or len(second) < 2:
            raise ValueError('road exclusion boundaries need at least two points')
        # 희소한 polyline에서도 짧은 경계 끝의 투영점을 정확히 찾는다.
        sample_step_m = min(0.1, 0.5 / grid.pixels_per_meter)
        first = resample_polyline(first, sample_step_m).astype(np.float64)
        second = resample_polyline(second, sample_step_m).astype(np.float64)

        # 짧은 실측선을 기준으로 긴 선의 동일 구간만 사용한다.
        if length(first) <= length(second):
            short, long = first, second
        else:
            short, long = second, first
        same = (np.linalg.norm(short[0] - long[0])
                + np.linalg.norm(short[-1] - long[-1]))
        reverse = (np.linalg.norm(short[0] - long[-1])
                   + np.linalg.norm(short[-1] - long[0]))
        if reverse < same:
            long = long[::-1]
        i0 = int(np.argmin(np.sum((long - short[0]) ** 2, axis=1)))
        i1 = int(np.argmin(np.sum((long - short[-1]) ** 2, axis=1)))
        if i1 < i0:
            long = long[::-1]
            i0 = int(np.argmin(np.sum((long - short[0]) ** 2, axis=1)))
            i1 = int(np.argmin(np.sum((long - short[-1]) ** 2, axis=1)))
        long = long[i0:i1 + 1]
        if len(long) < 2:
            raise ValueError('road exclusion boundaries have no overlapping span')

        polygon = np.concatenate([short, long[::-1]], axis=0)
        pair_mask = np.zeros_like(base)
        pixels = grid.to_pixel(polygon).astype(np.int32).reshape(-1, 1, 2)
        cv2.fillPoly(pair_mask, [pixels], 255)
        exclusion = cv2.bitwise_or(exclusion, pair_mask)
        pair_stats.append({
            'name': str(pair.get('name', '')),
            'boundaries': [str(s['source']) for s in selectors],
            'polygon_px': int((pair_mask > 0).sum()),
            'polygon_m2': round(
                int((pair_mask > 0).sum()) / (grid.pixels_per_meter ** 2), 2),
        })

    removed = cv2.bitwise_and(base, exclusion)
    result = cv2.bitwise_and(base, cv2.bitwise_not(exclusion))
    removed_px = int((removed > 0).sum())
    return result, exclusion, {
        'pairs': pair_stats,
        'removed_px': removed_px,
        'removed_m2': round(removed_px / (grid.pixels_per_meter ** 2), 2),
    }


def _read_rows(path: Path) -> List[Dict[str, Any]]:
    with open(path, 'r', encoding='utf-8-sig', newline='') as f:
        reader = csv.DictReader(f)
        required = {'timestamp', 'side', 'lane_map_x', 'lane_map_y'}
        missing = required - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f'{path}: 필수 CSV 열 없음: {sorted(missing)}')

        rows: List[Dict[str, Any]] = []
        for row in reader:
            try:
                timestamp = float(row['timestamp'])
                x = float(row['lane_map_x'])
                y = float(row['lane_map_y'])
            except (TypeError, ValueError):
                continue
            if not np.isfinite([timestamp, x, y]).all():
                continue
            side = str(row.get('side', '')).strip().upper()
            if side not in ('LEFT', 'RIGHT'):
                continue
            rows.append({
                'timestamp': timestamp,
                'side': side,
                'x': x,
                'y': y,
                'lane_color': str(row.get('lane_color', 'UNKNOWN')).strip().upper(),
                # 합쳐진 CSV를 원래 수집 단위로 다시 나눌 때 사용한다.
                'source_dataset': str(row.get('source_dataset', '')).strip(),
                'source_file': str(row.get('source_file', '')).strip(),
            })
    return rows


def _split_segments(
    rows: Iterable[Dict[str, Any]],
    max_point_gap_m: float,
    max_time_gap_sec: float,
) -> List[np.ndarray]:
    ordered = sorted(rows, key=lambda r: r['timestamp'])
    if not ordered:
        return []

    segments: List[np.ndarray] = []
    current: List[Tuple[float, float]] = [(ordered[0]['x'], ordered[0]['y'])]
    previous = ordered[0]
    for row in ordered[1:]:
        gap = float(np.hypot(row['x'] - previous['x'], row['y'] - previous['y']))
        dt = row['timestamp'] - previous['timestamp']
        if gap > max_point_gap_m or dt <= 0.0 or dt > max_time_gap_sec:
            if len(current) >= 2:
                segments.append(np.asarray(current, dtype=np.float64))
            current = []
        current.append((row['x'], row['y']))
        previous = row

    if len(current) >= 2:
        segments.append(np.asarray(current, dtype=np.float64))
    return segments


def _bridge_aligned_segments(
    segments: List[np.ndarray], max_gap_m: float, max_heading_deg: float
) -> Tuple[List[np.ndarray], int]:
    """가까우면서 앞·뒤 접선이 같은 방향인 연속 선분만 연결한다."""
    if len(segments) < 2 or max_gap_m <= 0:
        return segments, 0
    cos_limit = float(np.cos(np.deg2rad(max_heading_deg)))
    merged = [segments[0]]
    bridges = 0
    for following in segments[1:]:
        previous = merged[-1]
        connector = following[0] - previous[-1]
        gap = float(np.linalg.norm(connector))
        can_bridge = 1e-9 < gap <= max_gap_m
        if can_bridge:
            connector /= gap
            span_a = min(5, len(previous) - 1)
            span_b = min(5, len(following) - 1)
            tangent_a = previous[-1] - previous[-1 - span_a]
            tangent_b = following[span_b] - following[0]
            norm_a = float(np.linalg.norm(tangent_a))
            norm_b = float(np.linalg.norm(tangent_b))
            can_bridge = norm_a > 1e-9 and norm_b > 1e-9
            if can_bridge:
                tangent_a /= norm_a
                tangent_b /= norm_b
                can_bridge = (
                    float(np.dot(tangent_a, connector)) >= cos_limit
                    and float(np.dot(connector, tangent_b)) >= cos_limit)
        if can_bridge:
            merged[-1] = np.vstack([previous, following])
            bridges += 1
        else:
            merged.append(following)
    return merged, bridges


def _dilate_m(mask: np.ndarray, grid: RasterGrid, radius_m: float) -> np.ndarray:
    if radius_m <= 0.0:
        return mask.copy()
    radius_px = max(1, int(round(radius_m * grid.pixels_per_meter)))
    size = 2 * radius_px + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    return cv2.dilate(mask, kernel)


def _offset_toward_opposite(
    lane_xy: np.ndarray,
    opposite_xy: np.ndarray,
    offset_m: float,
    tangent_half_window: int,
) -> np.ndarray:
    """수집 시각화와 같은 방식으로 반대쪽 경계 방향으로 중심점을 옮긴다."""
    if len(lane_xy) == 0 or len(opposite_xy) == 0 or abs(offset_m) < 1e-12:
        return lane_xy.copy()
    corrected = np.empty_like(lane_xy, dtype=np.float64)
    n = len(lane_xy)
    for i, point in enumerate(lane_xy):
        i0 = max(0, i - tangent_half_window)
        i1 = min(n - 1, i + tangent_half_window)
        tangent = lane_xy[i1] - lane_xy[i0]
        vector = opposite_xy[np.argmin(np.sum((opposite_xy - point) ** 2, axis=1))] - point
        tangent_norm = float(np.linalg.norm(tangent))
        if tangent_norm < 1e-8:
            vector_norm = float(np.linalg.norm(vector))
            corrected[i] = point if vector_norm < 1e-8 else point + offset_m * vector / vector_norm
            continue
        tangent /= tangent_norm
        normal = np.array([-tangent[1], tangent[0]], dtype=np.float64)
        if float(np.dot(normal, vector)) < 0.0:
            normal = -normal
        corrected[i] = point + offset_m * normal
    return corrected


def _offset_single_side(
    lane_xy: np.ndarray,
    side: str,
    offset_m: float,
    tangent_half_window: int,
) -> np.ndarray:
    """반대 경계가 없을 때 timestamp 진행방향 기준으로 안쪽으로 옮긴다."""
    if len(lane_xy) == 0 or abs(offset_m) < 1e-12:
        return lane_xy.copy()
    corrected = np.empty_like(lane_xy, dtype=np.float64)
    n = len(lane_xy)
    for i, point in enumerate(lane_xy):
        i0 = max(0, i - tangent_half_window)
        i1 = min(n - 1, i + tangent_half_window)
        tangent = lane_xy[i1] - lane_xy[i0]
        norm = float(np.linalg.norm(tangent))
        if norm < 1e-8:
            corrected[i] = point
            continue
        tangent /= norm
        normal = (np.array([tangent[1], -tangent[0]], dtype=np.float64)
                  if side == 'LEFT' else
                  np.array([-tangent[1], tangent[0]], dtype=np.float64))
        corrected[i] = point + offset_m * normal
    return corrected


def _apply_inward_offset(
    rows: List[Dict[str, Any]], offset_m: float, tangent_half_window: int
) -> Dict[str, np.ndarray]:
    """행 좌표를 보정하고 보정된 좌/우 polyline을 반환한다."""
    by_side = {
        side: sorted((r for r in rows if r['side'] == side), key=lambda r: r['timestamp'])
        for side in ('LEFT', 'RIGHT')
    }
    raw = {
        side: np.asarray([(r['x'], r['y']) for r in side_rows], dtype=np.float64).reshape(-1, 2)
        for side, side_rows in by_side.items()
    }
    corrected: Dict[str, np.ndarray] = {}
    for side, opposite in (('LEFT', 'RIGHT'), ('RIGHT', 'LEFT')):
        if len(raw[opposite]):
            corrected[side] = _offset_toward_opposite(
                raw[side], raw[opposite], offset_m, tangent_half_window)
        else:
            corrected[side] = _offset_single_side(
                raw[side], side, offset_m, tangent_half_window)
        for row, xy in zip(by_side[side], corrected[side]):
            row['x'], row['y'] = float(xy[0]), float(xy[1])
    return corrected


def _fill_between_sides(
    target: np.ndarray,
    grid: RasterGrid,
    left_xy: np.ndarray,
    right_xy: np.ndarray,
) -> bool:
    """한 수집 파일의 좌·우 실측 경계 사이를 주행가능영역으로 채운다."""
    if len(left_xy) < 2 or len(right_xy) < 2:
        return False
    # 두 경계의 timestamp 진행방향이 반대인 파일도 안전하게 정렬한다.
    same = (np.linalg.norm(left_xy[0] - right_xy[0])
            + np.linalg.norm(left_xy[-1] - right_xy[-1]))
    reverse = (np.linalg.norm(left_xy[0] - right_xy[-1])
               + np.linalg.norm(left_xy[-1] - right_xy[0]))
    if reverse < same:
        right_xy = right_xy[::-1]
    polygon = np.concatenate([left_xy, right_xy[::-1]], axis=0)
    pixels = grid.to_pixel(polygon).astype(np.int32).reshape(-1, 1, 2)
    cv2.fillPoly(target, [pixels], 255)
    return True


def trim_road_outside_observed_lanes(
    road: np.ndarray,
    observed_lane: np.ndarray,
    grid: RasterGrid,
    features: Iterable[Mapping[str, Any]],
    link_centerlines: Iterable[np.ndarray],
    *,
    link_probe_offset_m: float = 1.75,
    link_tolerance_m: float = 1.0,
    boundary_search_m: float = 1.5,
    cut_depth_m: float = 2.0,
    protect_link_core_width_m: float = 1.0,
    sample_step_m: float = 0.25,
    min_run_m: float = 1.0,
) -> ObservedRoadTrimResult:
    """실측선 중 링크 도로의 최외곽과 맞닿은 구간 바깥 road만 제거한다.

    실측선 양쪽에 링크 중심선이 있으면 다차선 사이의 내부선이므로 자르지 않는다.
    한쪽에만 링크 중심선이 있고, 반대쪽 road가 ``boundary_search_m`` 안에서 끝나는
    지점만 외곽선으로 인정한다. 따라서 일부 차선만 수집했어도 그 바깥에 링크로
    표현된 다른 차로가 있으면 주행가능영역이 보존된다.
    """
    base = np.ascontiguousarray(road, dtype=np.uint8)
    center_mask = np.zeros_like(base)
    link_core_mask = np.zeros_like(base)
    center_thickness = grid.meters_to_px(2.0 * link_tolerance_m, minimum=1)
    core_thickness = grid.meters_to_px(protect_link_core_width_m, minimum=1)
    centerline_count = 0
    for points in link_centerlines:
        points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        if len(points) < 2:
            continue
        draw_polyline(center_mask, grid, points, center_thickness)
        draw_polyline(link_core_mask, grid, points, core_thickness)
        centerline_count += 1

    remove_seed = np.zeros_like(base)
    outer_run_corridor = np.zeros_like(base)
    classified_points = 0
    internal_points = 0
    unclassified_points = 0
    runs_drawn = 0

    def sample(mask: np.ndarray, xy: np.ndarray) -> np.ndarray:
        px = grid.to_pixel(xy)
        inside = ((px[:, 0] >= 0) & (px[:, 0] < grid.width_px)
                  & (px[:, 1] >= 0) & (px[:, 1] < grid.height_px))
        out = np.zeros(len(px), dtype=bool)
        out[inside] = mask[px[inside, 1], px[inside, 0]] > 0
        return out

    for feature in features:
        points = resample_polyline(
            np.asarray(feature['points'], dtype=np.float64).reshape(-1, 2),
            sample_step_m,
        ).astype(np.float64)
        if len(points) < 2:
            continue
        unit_normals = normals(points).astype(np.float64)
        plus_link = sample(
            center_mask, points + link_probe_offset_m * unit_normals)
        minus_link = sample(
            center_mask, points - link_probe_offset_m * unit_normals)

        # +1은 좌측 법선 방향이 외곽, -1은 우측 법선 방향이 외곽이다.
        signs = np.zeros(len(points), dtype=np.int8)
        for sign, inner_link, outer_link in (
                (+1, minus_link, plus_link), (-1, plus_link, minus_link)):
            candidates = inner_link & ~outer_link
            if not candidates.any():
                continue
            inner_road = sample(base, points - sign * 0.5 * unit_normals)
            boundary_outside = ~sample(
                base, points + sign * boundary_search_m * unit_normals)
            signs[candidates & inner_road & boundary_outside] = sign

        internal_points += int((plus_link & minus_link).sum())
        classified_points += int((signs != 0).sum())
        unclassified_points += int((~plus_link & ~minus_link).sum())

        # 같은 외곽 방향이 연속되는 구간만 polygon으로 만든다. 한두 점의 링크 매칭
        # 오차가 도로에 구멍을 내지 않도록 최소 길이를 둔다.
        start = 0
        while start < len(points):
            sign = int(signs[start])
            end = start + 1
            while end < len(points) and int(signs[end]) == sign:
                end += 1
            run = points[start:end]
            if sign and len(run) >= 2:
                run_length = float(np.linalg.norm(np.diff(run, axis=0), axis=1).sum())
                if run_length >= min_run_m:
                    outer = run + sign * cut_depth_m * unit_normals[start:end]
                    polygon = np.concatenate([run, outer[::-1]], axis=0)
                    pixels = grid.to_pixel(polygon).astype(np.int32).reshape(-1, 1, 2)
                    cv2.fillPoly(remove_seed, [pixels], 255)
                    # 실제 차선 래스터와 교차시킬 넉넉한 중심선 corridor다. 최종
                    # 제외 폭은 corridor 폭이 아니라 observed_lane 자체의 폭이다.
                    draw_polyline(
                        outer_run_corridor, grid, run,
                        grid.meters_to_px(0.6, minimum=1))
                    runs_drawn += 1
            start = end

    trimmed = cv2.bitwise_and(base, cv2.bitwise_not(remove_seed))
    observed = np.asarray(observed_lane, dtype=np.uint8)
    outer_lane_candidate = cv2.bitwise_and(observed, outer_run_corridor)
    # 유효 링크의 중심부와 겹치는 선은 합류·소멸 과정의 내부 차선이다. 고정 probe가
    # 외곽으로 오판했더라도 여기서는 제외 대상에서 빼 차로 중앙에 구멍을 내지 않는다.
    core_lane_overlap = cv2.bitwise_and(outer_lane_candidate, link_core_mask)
    excluded_lane_mask = cv2.bitwise_and(
        outer_lane_candidate, cv2.bitwise_not(link_core_mask))
    # 내부선과 외곽 여부를 판단할 수 없었던 선은 road에 포함하되, 외곽으로 판정된
    # 선은 도색 폭 자체까지 주행불가로 둔다.
    retained_lane = cv2.bitwise_and(observed, cv2.bitwise_not(excluded_lane_mask))
    trimmed = cv2.bitwise_or(trimmed, retained_lane)
    # 합류·소멸부에서는 고정 1.75 m probe가 휘어지는 인접 링크를 잠깐 놓칠 수 있다.
    # 원래 road에 있던 유효 링크 중심부는 되살려 절단 polygon이 차로 한가운데까지
    # 침범하지 못하게 한다. 없는 road를 새로 만들지는 않는다.
    protected_core = cv2.bitwise_and(base, link_core_mask)
    before_core = trimmed.copy()
    trimmed = cv2.bitwise_or(trimmed, protected_core)
    core_restored_px = int(((trimmed > 0) & (before_core == 0)).sum())
    # 링크 중심부와 겹치지 않는 진짜 외곽 도색만 최종 제외한다.
    trimmed = cv2.bitwise_and(trimmed, cv2.bitwise_not(excluded_lane_mask))
    # 내부 차선을 되살리고 외곽 차선을 제외한 최종 결과와 비교한다.
    removed_mask = cv2.bitwise_and(base, cv2.bitwise_not(trimmed))
    removed_px = int((removed_mask > 0).sum())
    return ObservedRoadTrimResult(
        image=trimmed,
        removed_mask=removed_mask,
        excluded_lane_mask=excluded_lane_mask,
        stats={
            'enabled': True,
            'link_centerlines': centerline_count,
            'link_probe_offset_m': float(link_probe_offset_m),
            'link_tolerance_m': float(link_tolerance_m),
            'boundary_search_m': float(boundary_search_m),
            'cut_depth_m': float(cut_depth_m),
            'protect_link_core_width_m': float(protect_link_core_width_m),
            'link_core_restored_px': core_restored_px,
            'link_core_restored_m2': round(
                core_restored_px / (grid.pixels_per_meter ** 2), 2),
            'sample_step_m': float(sample_step_m),
            'min_run_m': float(min_run_m),
            'outer_points': classified_points,
            'internal_points_preserved': internal_points,
            'unclassified_points_preserved': unclassified_points,
            'outer_runs': runs_drawn,
            'removed_px': removed_px,
            'removed_m2': round(removed_px / (grid.pixels_per_meter ** 2), 2),
            'excluded_lane_px': int((excluded_lane_mask > 0).sum()),
            'excluded_lane_m2': round(
                int((excluded_lane_mask > 0).sum())
                / (grid.pixels_per_meter ** 2), 2),
            'link_core_lane_reclassified_px': int((core_lane_overlap > 0).sum()),
            'link_core_lane_reclassified_m2': round(
                int((core_lane_overlap > 0).sum())
                / (grid.pixels_per_meter ** 2), 2),
        },
    )


def apply_observed_lane_overrides(
    lane_prior: np.ndarray,
    grid: RasterGrid,
    sources: Iterable[Mapping[str, Any]],
    *,
    line_width_m: float = 0.15,
    min_thickness_px: int = 2,
    replace_radius_m: float = 0.8,
    max_point_gap_m: float = 1.5,
    max_time_gap_sec: float = 0.5,
    inward_offset_m: float = 0.0,
    tangent_half_window: int = 3,
    keep_prior: bool = True,
    fill_road_between_sides: bool = False,
    road: Optional[np.ndarray] = None,
) -> ObservedLaneResult:
    """실측선을 래스터화하고 주변 prior 차선만 지운 뒤 합친다.

    링크나 road 자체는 제거하지 않는다. 같은 경계를 공유하는 인접 링크가 있어도
    다차선 도로 구조는 유지되고, 최종 이진 lane에서 중복 선만 하나로 합쳐진다.
    """
    prior = np.ascontiguousarray(lane_prior, dtype=np.uint8)
    observed_white = np.zeros_like(prior)
    observed_yellow = np.zeros_like(prior)
    observed_road = np.zeros_like(prior)
    replace_seed = np.zeros_like(prior)
    thickness = grid.meters_to_px(line_width_m, minimum=min_thickness_px)

    source_stats: List[Dict[str, Any]] = []
    features: List[Dict[str, Any]] = []
    total_rows = 0
    total_segments = 0

    for source in sources:
        path = Path(str(source['path']))
        if not path.is_file():
            raise FileNotFoundError(f'실측 차선 CSV 없음: {path}')
        action = str(source.get('action', 'replace')).strip().lower()
        if action not in ('replace', 'add'):
            raise ValueError(f'{path}: action은 replace 또는 add여야 합니다: {action}')

        rows = _read_rows(path)
        group_column = str(source.get('group_column', '')).strip()
        if group_column and group_column not in ('source_dataset', 'source_file'):
            raise ValueError(
                f'{path}: group_column은 source_dataset 또는 source_file이어야 합니다: '
                f'{group_column}')
        row_groups: Dict[str, List[Dict[str, Any]]] = {}
        if group_column:
            for row in rows:
                row_groups.setdefault(str(row[group_column]), []).append(row)
            if rows and set(row_groups) == {''}:
                raise ValueError(f'{path}: CSV에 {group_column} 값이 없습니다')
        else:
            row_groups[''] = rows

        side_colors = {
            str(k).strip().upper(): str(v).strip().upper()
            for k, v in dict(source.get('side_colors') or {}).items()
        }
        point_gap_by_lane = {
            str(k).strip().upper(): float(v)
            for k, v in dict(source.get('max_point_gap_by_lane') or {}).items()
        }
        invalid_gap_keys = set(point_gap_by_lane) - {
            'LEFT_WHITE', 'LEFT_YELLOW', 'RIGHT_WHITE', 'RIGHT_YELLOW'}
        if invalid_gap_keys:
            raise ValueError(
                f'{path}: max_point_gap_by_lane 키 오류: {sorted(invalid_gap_keys)}')
        bridge_gap_by_lane = {
            str(k).strip().upper(): float(v)
            for k, v in dict(source.get('bridge_segment_gaps_by_lane') or {}).items()
        }
        invalid_bridge_keys = set(bridge_gap_by_lane) - {
            'LEFT_WHITE', 'LEFT_YELLOW', 'RIGHT_WHITE', 'RIGHT_YELLOW'}
        if invalid_bridge_keys:
            raise ValueError(
                f'{path}: bridge_segment_gaps_by_lane 키 오류: '
                f'{sorted(invalid_bridge_keys)}')
        bridge_max_heading_deg = float(source.get('bridge_max_heading_deg', 20.0))
        source_time_gap_sec = float(
            source.get('max_time_gap_sec', max_time_gap_sec))
        if source_time_gap_sec <= 0:
            raise ValueError(f'{path}: max_time_gap_sec은 양수여야 합니다')
        drawn_segments = 0
        bridged_segments = 0
        manual_bridge_count = 0
        road_filled_groups = 0
        color_counts = {'WHITE': 0, 'YELLOW': 0}

        for group_key, group_rows in row_groups.items():
            corrected_by_side = _apply_inward_offset(
                group_rows, inward_offset_m, max(1, int(tangent_half_window)))
            if fill_road_between_sides and _fill_between_sides(
                    observed_road, grid,
                    corrected_by_side['LEFT'], corrected_by_side['RIGHT']):
                road_filled_groups += 1

            for side in ('LEFT', 'RIGHT'):
                side_rows = [r for r in group_rows if r['side'] == side]
                if not side_rows:
                    continue
                forced_color = side_colors.get(side)
                if forced_color:
                    color_groups = {forced_color: side_rows}
                else:
                    color_groups: Dict[str, List[Dict[str, Any]]] = {}
                    for row in side_rows:
                        color_groups.setdefault(row['lane_color'], []).append(row)

                for color, color_rows in color_groups.items():
                    if color not in ('WHITE', 'YELLOW'):
                        raise ValueError(
                            f'{path}: {side} 차선 색을 WHITE/YELLOW로 지정해야 합니다: '
                            f'{color}')
                    target = observed_white if color == 'WHITE' else observed_yellow
                    lane_key = f'{side}_{color}'
                    segments = _split_segments(
                        color_rows,
                        max_point_gap_m=point_gap_by_lane.get(
                            lane_key, max_point_gap_m),
                        max_time_gap_sec=source_time_gap_sec,
                    )
                    segments, bridge_count = _bridge_aligned_segments(
                        segments,
                        bridge_gap_by_lane.get(lane_key, 0.0),
                        bridge_max_heading_deg,
                    )
                    bridged_segments += bridge_count
                    for points in segments:
                        draw_polyline(target, grid, points, thickness)
                        features.append({
                            'points': points.astype(np.float32),
                            'side': side,
                            'color': color,
                            'source': str(path),
                            'source_group': group_key,
                        })
                        if action == 'replace':
                            draw_polyline(replace_seed, grid, points, thickness)
                    drawn_segments += len(segments)
                    color_counts[color] += len(color_rows)

        # 수집 데이터 안에 짧은 역방향 노이즈 선분이 끼어 자동 연결이 불가능한
        # 경우에만 두 원본 끝점을 명시적으로 잇는다. 좌표에는 다른 실측선과 같은
        # 안쪽 오프셋을 적용한 뒤 래스터와 H5 벡터 양쪽에 저장한다.
        for bridge in list(source.get('manual_bridges') or []):
            side = str(bridge.get('side', '')).strip().upper()
            color = str(bridge.get('color', '')).strip().upper()
            if side not in ('LEFT', 'RIGHT') or color not in ('WHITE', 'YELLOW'):
                raise ValueError(f'{path}: manual_bridges의 side/color 오류: {bridge}')
            bridge_points = np.asarray(bridge.get('points'), dtype=np.float64).reshape(-1, 2)
            if len(bridge_points) < 2 or not np.isfinite(bridge_points).all():
                raise ValueError(f'{path}: manual_bridges points 오류: {bridge}')
            bridge_points = _offset_single_side(
                bridge_points, side, inward_offset_m,
                max(1, int(tangent_half_window)))
            target = observed_white if color == 'WHITE' else observed_yellow
            draw_polyline(target, grid, bridge_points, thickness)
            features.append({
                'points': bridge_points.astype(np.float32),
                'side': side,
                'color': color,
                'source': str(path),
                'source_group': 'manual_bridge',
            })
            if action == 'replace':
                draw_polyline(replace_seed, grid, bridge_points, thickness)
            drawn_segments += 1
            manual_bridge_count += 1

        total_rows += len(rows)
        total_segments += drawn_segments
        source_stats.append({
            'path': str(path),
            'action': action,
            'rows': len(rows),
            'segments': drawn_segments,
            'group_column': group_column or None,
            'groups': len(row_groups),
            'road_filled_between_sides': bool(road_filled_groups),
            'road_filled_groups': road_filled_groups,
            'color_rows': color_counts,
            'max_point_gap_by_lane': point_gap_by_lane,
            'bridge_segment_gaps_by_lane': bridge_gap_by_lane,
            'bridge_max_heading_deg': bridge_max_heading_deg,
            'max_time_gap_sec': source_time_gap_sec,
            'bridged_segments': bridged_segments,
            'manual_bridges': manual_bridge_count,
        })

    observed = cv2.bitwise_or(observed_white, observed_yellow)
    if keep_prior:
        erase_corridor = _dilate_m(replace_seed, grid, replace_radius_m)
        # 삭제 마스크에는 실제 prior 차선 픽셀만 남긴다.
        erase_mask = cv2.bitwise_and(erase_corridor, prior)
        remaining = cv2.bitwise_and(prior, cv2.bitwise_not(erase_mask))
    else:
        # 최종 lane에는 실측만 사용하되 prior는 별도 H5 레이어로 보존한다.
        erase_mask = prior.copy()
        remaining = np.zeros_like(prior)
    removed_prior = erase_mask.copy()
    final = cv2.bitwise_or(remaining, observed)

    observed_px = observed > 0
    observed_on_road_ratio = None
    if road is not None and observed_px.any():
        # 선 중심이 road 경계에 놓일 수 있으므로 0.6 m 주변까지 정상으로 본다.
        near_road = _dilate_m(np.asarray(road, dtype=np.uint8), grid, 0.6) > 0
        observed_on_road_ratio = float(near_road[observed_px].mean())

    stats: Dict[str, Any] = {
        'source_count': len(source_stats),
        'rows': total_rows,
        'segments': total_segments,
        'line_width_m': float(line_width_m),
        'thickness_px': int(thickness),
        'effective_width_m': float(thickness / grid.pixels_per_meter),
        'replace_radius_m': float(replace_radius_m),
        'inward_offset_m': float(inward_offset_m),
        'tangent_half_window': int(tangent_half_window),
        'keep_prior': bool(keep_prior),
        'fill_road_between_sides': bool(fill_road_between_sides),
        'observed_road_px': int((observed_road > 0).sum()),
        'observed_px': int(observed_px.sum()),
        'white_px': int((observed_white > 0).sum()),
        'yellow_px': int((observed_yellow > 0).sum()),
        'prior_removed_px': int((removed_prior > 0).sum()),
        'final_lane_px': int((final > 0).sum()),
        'observed_near_road_ratio': (
            None if observed_on_road_ratio is None else round(observed_on_road_ratio, 4)
        ),
        'sources': source_stats,
    }
    return ObservedLaneResult(
        image=final,
        observed=observed,
        observed_white=observed_white,
        observed_yellow=observed_yellow,
        observed_road=observed_road,
        erase_mask=erase_mask,
        removed_prior=removed_prior,
        features=features,
        stats=stats,
    )
