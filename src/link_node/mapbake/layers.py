"""세 정적 레이어의 베이크 로직 (전부 이진 0/255).

소스는 link_set.json 과 node_set.json 뿐이다. 이 맵에는 lane_boundary_set /
lane_marking_set 이 존재하지 않으므로 차선은 링크 위상에서 합성한다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np

from .io_mgeo import MGeoLinkNode
from .raster import (
    RasterGrid,
    close_gaps,
    draw_dashed_polyline,
    draw_polyline,
    fill_thin_gaps,
    shrink_outer,
)
from .topology import (
    LinkTopology,
    average_curve_radius,
    extend_polyline,
    merge_taper_ends,
    stop_line_bar_directions,
    is_intersection,
    offset_polyline,
    outer_side_flags,
    resample_polyline,
    trim_polyline,
)


def resolve_excluded(
    data: MGeoLinkNode,
    exclude_links: Optional[Iterable[str]] = None,
    exclude_roads: Optional[Iterable[str]] = None,
) -> Tuple[set, list]:
    """제외할 링크 집합과, '일부 차로만 빠진 도로' 경고 목록을 만든다.

    제외는 링크(=차로) 단위다. 여러 차로짜리 도로를 주행하며 모으면 지나간 차로만
    빠져서, 남은 차로의 차선만 그려지고 ego 가 차선에 걸친 것처럼 보인다.
    road_id 로 지정하면 그 도로의 모든 차로가 한꺼번에 빠진다.
    """
    dropped = set(exclude_links or ())
    roads = set(str(r) for r in (exclude_roads or ()))
    by_road: Dict[str, list] = {}
    for link in data.valid_links:
        by_road.setdefault(link.road_id, []).append(link)
    for rid in roads:
        for link in by_road.get(rid, ()):
            dropped.add(link.idx)
    partial = []
    for rid, links in by_road.items():
        n = sum(1 for l in links if l.idx in dropped)
        if 0 < n < len(links):
            partial.append((rid, n, len(links)))
    partial.sort()
    return dropped, partial


@dataclass
class LayerResult:
    image: np.ndarray
    stats: Dict[str, Any] = field(default_factory=dict)
    # 래스터만으로는 '가장 가까운 정지선까지 몇 m' 같은 질문에 답할 수 없다.
    # 그린 요소의 벡터 정보를 함께 넘겨 H5 에 같이 저장한다.
    features: List[Dict[str, Any]] = field(default_factory=list)


def bake_drivable(
    data: MGeoLinkNode,
    grid: RasterGrid,
    *,
    close_kernel_px: int = 5,
    fill_gaps_max_width_m: float = 2.0,
    fill_gaps_max_area_m2: float = 20.0,
    merge_taper_m: float = 0.0,
    outer_trim_m: float = 0.0,
    exclude_links: Optional[Iterable[str]] = None,
    exclude_roads: Optional[Iterable[str]] = None,
    centerlines: Optional[Dict[str, np.ndarray]] = None,
) -> LayerResult:
    """링크 중심선을 차로 폭만큼 두꺼운 폴리라인으로 그려 주행가능영역을 만든다.

    노드 핀치 보정(`centerlines`)을 차선에만 적용하면 도로는 노드 근처에서 좁은 채로
    남아, 보정된 차선이 도로 밖으로 삐져나간다. 두 레이어가 같은 중심선을 써야 한다.
    """
    canvas = grid.blank()
    corrected = centerlines or {}
    tapers = merge_taper_ends(data.valid_links) if merge_taper_m > 0 else {}
    dropped, partial = resolve_excluded(data, exclude_links, exclude_roads)
    used = []
    drawn = 0
    tapered = 0
    for link in data.valid_links:
        if link.idx in dropped:
            continue
        used.append(link.idx)
        thickness = grid.meters_to_px(link.width_m)
        pts = corrected.get(link.idx, link.points)
        if link.idx in tapers:
            head, tail = tapers[link.idx]
            pts = extend_polyline(pts, merge_taper_m if head else 0.0,
                                  merge_taper_m if tail else 0.0)
            tapered += 1
        draw_polyline(canvas, grid, pts, thickness)
        drawn += 1
    # 인접 차로 폴리라인 사이에 남는 틈과 코너 계단을 메운다.
    canvas = close_gaps(canvas, close_kernel_px)
    before = int((canvas > 0).sum())
    # 인접 차로 통로 사이에 남는 가느다란 틈만 메운다. 교차로 코너의 인도 섬은
    # 뭉툭해서 (내접반경 2.8 m 이상) 그대로 남는다.
    canvas = fill_thin_gaps(canvas, grid.pixels_per_meter,
                            fill_gaps_max_width_m, fill_gaps_max_area_m2)
    filled = int((canvas > 0).sum())
    # 차선 바깥 절반을 잘라내려면 도로도 같은 양만큼 줄여야 바깥 끝이 맞는다.
    canvas = shrink_outer(canvas, grid.pixels_per_meter, outer_trim_m)
    trimmed = int((canvas > 0).sum())
    return LayerResult(
        image=canvas,
        stats={
            'links_drawn': drawn,
            'outer_trim_m': outer_trim_m,
            'outer_trimmed_m2': round((filled - trimmed) / (grid.pixels_per_meter ** 2), 1),
            'links_excluded': len(dropped & {l.idx for l in data.valid_links}),
            'partially_excluded_roads': partial,
            'close_kernel_px': close_kernel_px,
            'fill_gaps_max_width_m': fill_gaps_max_width_m,
            'fill_gaps_max_area_m2': fill_gaps_max_area_m2,
            'pinch_corrected_links': len(corrected),
            'merge_taper_m': merge_taper_m,
            'merge_tapered_links': tapered,
            'gaps_filled_m2': round((filled - before) / (grid.pixels_per_meter ** 2), 1),
            'filled_px': trimmed,
        },
        features=[{'idx': i} for i in used],
    )


def bake_lane(
    data: MGeoLinkNode,
    grid: RasterGrid,
    *,
    resample_step_m: float = 0.5,
    line_width_m: float = 0.15,
    min_thickness_px: int = 2,
    skip_intersection: bool = True,
    intersection_clear_m: float = 30.0,
    min_avg_radius_m: float = 10.0,
    exclude_links: Optional[Iterable[str]] = None,
    exclude_roads: Optional[Iterable[str]] = None,
    outer_shift_m: float = 0.0,
    clip_to: Optional[np.ndarray] = None,
    centerlines: Optional[Dict[str, np.ndarray]] = None,
    dashed_on_lane_change: bool = False,
    dash_m: float = 3.0,
    gap_m: float = 5.0,
) -> LayerResult:
    """차로 중심선을 ±width/2 오프셋해 차선 경계를 합성한다.

    MGeo 링크는 차로 단위 중심선이고 인접 차로 간격이 곧 차로 폭이므로,
    한 차로의 좌경계와 그 왼쪽 차로의 우경계는 같은 위치에 떨어진다.
    이진 래스터에서는 겹쳐 그려도 결과가 같아 중복 제거가 필요 없다.

    실측 한계: opp_traffic 이 전 링크 False, lane_change_dir 이 전부 None 이라
    중앙선(황색) 판정 근거가 없다. 그래서 이진 단일 클래스로만 굽는다.
    """
    canvas = grid.blank()
    thickness = grid.meters_to_px(line_width_m, minimum=min_thickness_px)
    drawn = 0
    skipped = 0
    trimmed_junction = 0
    skipped_tight = []
    stop_ids = {n.idx for n in data.stop_line_nodes}
    dropped, partial = resolve_excluded(data, exclude_links, exclude_roads)
    used = []
    excluded = 0
    fixed = centerlines or {}
    # 도로 최외곽을 깎았으면 그쪽 차선도 같이 안쪽으로 옮겨야 두께가 살아남는다.
    # 차로 사이 선은 양쪽이 다 도로라 옮기지 않는다.
    outer = ({} if outer_shift_m <= 0 else
             outer_side_flags([l for l in data.valid_links if l.idx not in dropped],
                              fixed))
    shifted = 0
    for link in data.valid_links:
        if link.idx in dropped:
            excluded += 1
            continue
        head_m = tail_m = 0.0
        if skip_intersection and is_intersection(link):
            # link_type=1 은 '교차로 내부'가 아니라 '정지선에서 출발하는 링크'다
            # (256개 전부). 길이가 45~54 m 인 것도 있어 통째로 버리면 교차로를
            # 한참 지난 구간의 차선까지 사라진다. 실측한 교차로 횡단 거리
            # (정지선->정지선 링크 길이, 최대 32.3 m) 만큼만 잘라낸다.
            at_start = link.from_node in stop_ids
            at_end = link.to_node in stop_ids
            if intersection_clear_m <= 0 or (at_start and at_end):
                skipped += 1
                continue
            if at_start:
                head_m = intersection_clear_m
            elif at_end:
                tail_m = intersection_clear_m
            else:
                skipped += 1
                continue
            trimmed_junction += 1
        center = fixed.get(link.idx)
        if center is None:
            center = resample_polyline(link.points, resample_step_m)
        if head_m or tail_m:
            center = trim_polyline(center, head_m, tail_m)
            if center.shape[0] < 4:      # 교차로 안에서 끝나는 링크
                trimmed_junction -= 1
                skipped += 1
                continue
        if center.shape[0] < 2:
            continue
        # U턴·급커넥터 링크는 주행 궤적이지 도색 차선이 아니다. 중심선을 ±width/2
        # 오프셋하면 차 앞에 부채꼴 모양 가짜 차선이 생긴다.
        # 판정은 잘라낸 뒤의 구간으로 한다 — 좌회전 링크는 교차로 안에서만 급하게
        # 휘고 그 너머는 곧게 이어지므로, 전체로 재면 멀쩡한 뒷구간까지 버린다.
        if min_avg_radius_m > 0 and average_curve_radius(center) < min_avg_radius_m:
            skipped_tight.append(link.idx)
            if head_m or tail_m:
                trimmed_junction -= 1
            continue
        half = 0.5 * link.width_m
        is_outer_l, is_outer_r = outer.get(link.idx, (False, False))
        half_l = half - outer_shift_m if is_outer_l else half
        half_r = half - outer_shift_m if is_outer_r else half
        shifted += int(is_outer_l) + int(is_outer_r)
        for offset, dashed in (
            (+half_l, dashed_on_lane_change and link.can_move_left),
            (-half_r, dashed_on_lane_change and link.can_move_right),
        ):
            boundary = offset_polyline(center, offset)
            if dashed:
                draw_dashed_polyline(canvas, grid, boundary, thickness, dash_m, gap_m)
            else:
                draw_polyline(canvas, grid, boundary, thickness)
        drawn += 1
        used.append(link.idx)

    # 도로 바깥으로 나간 차선(=바깥 절반)을 잘라낸다.
    clipped_m2 = 0.0
    if clip_to is not None:
        before_px = int((canvas > 0).sum())
        canvas = np.where(clip_to > 0, canvas, 0).astype(np.uint8)
        clipped_m2 = (before_px - int((canvas > 0).sum())) / (grid.pixels_per_meter ** 2)

    return LayerResult(
        image=canvas,
        stats={
            'links_used': drawn,
            'links_excluded': excluded,
            'partially_excluded_roads': partial,
            'intersection_links_skipped': skipped,
            'intersection_links_trimmed': trimmed_junction,
            'intersection_clear_m': intersection_clear_m,
            'tight_curve_links_skipped': len(skipped_tight),
            'min_avg_radius_m': min_avg_radius_m,
            'pinch_corrected_links': len(fixed),
            'thickness_px': thickness,
            'resample_step_m': resample_step_m,
            'outer_shift_m': outer_shift_m,
            'outer_shifted_sides': shifted,
            'clipped_to_road_m2': round(clipped_m2, 1),
            'filled_px': int((canvas > 0).sum()),
        },
        features=[{'idx': i} for i in used],
    )


def bake_stopline(
    data: MGeoLinkNode,
    grid: RasterGrid,
    *,
    line_width_m: float = 0.4,
    min_thickness_px: int = 2,
    length_scale: float = 1.0,
    align_across_lanes: bool = True,
    group_radius_m: float = 12.0,
    include_nodes: Optional[Iterable[str]] = None,
    exclude_nodes: Optional[Iterable[str]] = None,
    only_with_traffic_light: bool = False,
) -> LayerResult:
    """on_stop_line 노드에 그 노드의 진행 방향과 수직인 선분을 그린다.

    노드는 점 정보뿐이라 기하가 없다. 방향은 노드에 연결된 링크에서 가져오고,
    길이는 그 링크의 차로 폭을 쓴다 (차로마다 노드가 따로 있으므로 여러 차로가
    이어지면 자연스럽게 하나의 정지선으로 붙는다).
    """
    canvas = grid.blank()
    topo = LinkTopology(data)
    thickness = grid.meters_to_px(line_width_m, minimum=min_thickness_px)
    drawn = 0
    unresolved = []

    include = set(include_nodes) if include_nodes is not None else None
    exclude = set(exclude_nodes or ())

    def selected(node) -> bool:
        if include is not None and node.idx not in include:
            return False
        if node.idx in exclude:
            return False
        if only_with_traffic_light and not node.traffic_light_id:
            return False
        return True

    resolved = {}
    for node in data.stop_line_nodes:
        hw = topo.node_heading(node.idx)
        if hw is not None:
            resolved[node.idx] = hw

    # 방향 계산에는 선택되지 않은 이웃 노드도 쓴다. 한 줄에서 하나만 골랐을 때
    # 그 하나가 이웃과 어긋나면 안 되기 때문이다. 거르는 건 그리는 단계에서 한다.
    # 정지선 직전에 휘는 차로는 접선만 쓰면 정지선이 기울어진다. 같은 진입부
    # 노드들이 이루는 직선을 우선 쓰고, 없으면 접선의 법선으로 폴백한다.
    bars = {}
    if align_across_lanes:
        bars = stop_line_bar_directions(
            data.stop_line_nodes,
            {k: v[0] for k, v in resolved.items()},
            group_radius_m=group_radius_m,
        )

    skipped = 0
    features: List[Dict[str, Any]] = []
    for node in data.stop_line_nodes:
        if not selected(node):
            skipped += 1
            continue
        hw = resolved.get(node.idx)
        if hw is None:
            unresolved.append(node.idx)
            continue
        heading, width_m = hw
        # 진행 방향의 좌측 법선: (x, y) -> (-y, x)
        perp = bars.get(node.idx)
        if perp is None:
            perp = np.array([-heading[1], heading[0]], dtype=np.float32)
        half = 0.5 * width_m * length_scale
        seg = np.stack([node.point - half * perp, node.point + half * perp], axis=0)
        draw_polyline(canvas, grid, seg, thickness)
        drawn += 1
        features.append({
            'idx': node.idx,
            'x': float(node.point[0]),
            'y': float(node.point[1]),
            # 정지선 바의 방향 (진행방향에 수직)
            'dir_x': float(perp[0]),
            'dir_y': float(perp[1]),
            # 정지선을 통과하는 주행 방향
            'heading_x': float(heading[0]),
            'heading_y': float(heading[1]),
            'length_m': float(2.0 * half),
            'traffic_light_id': node.traffic_light_id,
            'aligned': node.idx in bars,
        })
    return LayerResult(
        image=canvas,
        stats={
            'stop_line_nodes': len(data.stop_line_nodes),
            'drawn': drawn,
            'filtered_out': skipped,
            'aligned_across_lanes': len(bars),
            'unresolved_heading': unresolved,
            'thickness_px': thickness,
            'filled_px': int((canvas > 0).sum()),
        },
        features=features,
    )
