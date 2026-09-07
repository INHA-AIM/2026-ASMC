#!/usr/bin/env python3
"""링크·노드(MGeo)에서 BEV 정적 레이어를 굽는 CLI.

    python3 src/link_node/bake_map.py
    python3 src/link_node/bake_map.py --config src/link_node/config/waize_katri.yaml
    python3 src/link_node/bake_map.py --ppm 4 --no-overlay

주행가능영역 / 차선 / 정지선 세 레이어를 이진(0/255)으로 굽고 H5 로 저장한 뒤,
검증용 오버레이 PNG 와 정합 지표를 출력한다. ROS/UDP 를 쓰지 않는 오프라인 도구다.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mapbake.io_mgeo import load_link_node
from mapbake.layers import bake_drivable, bake_lane, bake_stopline
from mapbake.raster import RasterGrid, compute_bounds
from mapbake.topology import lane_pinch_corrections
from mapbake.writer import write_h5

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = Path(__file__).resolve().parent / 'config' / 'waize_katri.yaml'


def _resolve(p: str) -> Path:
    path = Path(p)
    return path if path.is_absolute() else REPO_ROOT / path


def colorize(road: np.ndarray, lane: np.ndarray, stopline: np.ndarray) -> np.ndarray:
    """road 회색 / lane 노랑 / stopline 빨강 (BGR). 뒤에 칠한 것이 위로 온다."""
    canvas = np.zeros((*road.shape, 3), dtype=np.uint8)
    canvas[road > 0] = (90, 90, 90)
    canvas[lane > 0] = (40, 230, 240)
    canvas[stopline > 0] = (60, 60, 235)
    return canvas


def _maxpool_to(img: np.ndarray, size: tuple) -> np.ndarray:
    """축소해도 1~2 px 선이 사라지지 않도록 최대값 풀링으로 줄인다.

    팽창 후 INTER_AREA 로 줄이면 얇은 선이 굵어져 도로를 덮어버린다. 팽창 커널을
    축소 배율과 같게 잡고 최근접 샘플링하면 굵기 비율이 보존된다.
    """
    h, w = img.shape[:2]
    r = max(1, int(round(max(h / size[1], w / size[0]))))
    if r > 1:
        img = cv2.dilate(img, cv2.getStructuringElement(cv2.MORPH_RECT, (r, r)))
    return cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)


def link_records(data, centerlines, road, lane) -> list:
    """H5 에 넣을 링크 정보: 실제로 그려진 중심선과 레이어 소속."""
    in_road = {f['idx'] for f in road.features}
    in_lane = {f['idx'] for f in lane.features}
    out = []
    for link in data.valid_links:
        pts = centerlines.get(link.idx, link.points)
        out.append({
            'idx': link.idx,
            'points': np.asarray(pts, dtype=np.float32)[::2],   # 1 m 간격이면 충분
            'in_road': link.idx in in_road,
            'in_lane': link.idx in in_lane,
        })
    return out


def stop_line_nodes_on_route(data, route_yaml: Path) -> list:
    """route yaml 의 링크들이 물고 있는 정지선 노드 id.

    behavior_planner 의 route_links*.yaml 과 같은 형식 (route.links: [링크 id...]).
    """
    with open(route_yaml, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}
    wanted = set((cfg.get('route') or {}).get('links') or [])
    if not wanted:
        raise SystemExit(f'route.links 가 비어 있습니다: {route_yaml}')
    stop_ids = {n.idx for n in data.stop_line_nodes}
    out = []
    for link in data.valid_links:
        if link.idx not in wanted:
            continue
        for node in (link.from_node, link.to_node):
            if node in stop_ids and node not in out:
                out.append(node)
    return out


def print_stop_line_table(data) -> None:
    """정지선 노드 목록. include_nodes 에 넣을 id 를 여기서 고른다."""
    from mapbake.topology import LinkTopology
    topo = LinkTopology(data)
    print(f'{"node idx":<16}{"x":>10}{"y":>10}  {"신호등 id":<16}진입 링크')
    for node in sorted(data.stop_line_nodes, key=lambda n: n.idx):
        links = topo.incoming(node.idx) or topo.outgoing(node.idx)
        names = ','.join(l.idx for l in links[:3]) or '-'
        print(f'{node.idx:<16}{node.point[0]:>10.2f}{node.point[1]:>10.2f}  '
              f'{node.traffic_light_id or "-":<16}{names}')
    print(f'총 {len(data.stop_line_nodes)}개')


def build_overlay(
    road: np.ndarray,
    lane: np.ndarray,
    stopline: np.ndarray,
    max_px: int,
) -> np.ndarray:
    h, w = road.shape
    scale = min(1.0, max_px / max(h, w))
    if scale < 1.0:
        size = (max(1, int(w * scale)), max(1, int(h * scale)))
        road, lane, stopline = (_maxpool_to(x, size) for x in (road, lane, stopline))
    # 이미지 행이 +y 이므로, 위쪽이 북쪽이 되도록 뒤집어 저장한다.
    return cv2.flip(colorize(road, lane, stopline), 0)


def build_zoom(
    road: np.ndarray,
    lane: np.ndarray,
    stopline: np.ndarray,
    grid: RasterGrid,
    center_xy: np.ndarray,
    half_extent_m: float,
) -> np.ndarray:
    """월드 한 지점 주변을 원해상도로 잘라낸다. 차선/정지선 형상 육안 검증용."""
    c, r = grid.to_pixel(np.asarray(center_xy).reshape(1, 2))[0]
    half = int(round(half_extent_m * grid.pixels_per_meter))
    c0, c1 = max(0, c - half), min(grid.width_px, c + half)
    r0, r1 = max(0, r - half), min(grid.height_px, r + half)
    if c1 - c0 < 2 or r1 - r0 < 2:
        return np.zeros((2, 2, 3), dtype=np.uint8)
    crop = colorize(road[r0:r1, c0:c1], lane[r0:r1, c0:c1], stopline[r0:r1, c0:c1])
    return cv2.flip(crop, 0)


def compute_metrics(
    road: np.ndarray,
    lane: np.ndarray,
    stopline: np.ndarray,
    grid: RasterGrid,
) -> Dict[str, Any]:
    """굽고 나서 눈으로만 보면 축 뒤바뀜을 못 잡는다. 수치로 확인한다."""
    metrics: Dict[str, Any] = {}

    road_px = int((road > 0).sum())
    metrics['road_area_m2'] = round(road_px / (grid.pixels_per_meter ** 2), 1)
    metrics['road_fill_ratio'] = round(road_px / road.size, 5)

    # 정지선 노드는 링크 위에 정확히 놓여 있으므로 거의 100% 가 road 위여야 한다.
    sl = stopline > 0
    metrics['stopline_px'] = int(sl.sum())
    if sl.any():
        metrics['stopline_on_road_ratio'] = round(float((road[sl] > 0).mean()), 4)

    # 차선은 차로 경계이므로 road 가장자리 부근에 있어야 한다.
    ln = lane > 0
    metrics['lane_px'] = int(ln.sum())
    if ln.any():
        band = grid.meters_to_px(0.6)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * band + 1, 2 * band + 1))
        near_road = cv2.dilate(road, k) > 0
        metrics['lane_near_road_ratio'] = round(float(near_road[ln].mean()), 4)

    # 좌표 왕복 오차
    probe = np.array(
        [[0.0, 0.0], [-100.0, -300.0], [50.0, 200.0], [-400.0, 500.0]], dtype=np.float64
    )
    back = grid.to_world(grid.to_pixel(probe))
    metrics['world_roundtrip_max_err_m'] = round(
        float(np.abs(back - probe).max()), 6
    )
    metrics['world_roundtrip_tolerance_m'] = round(1.0 / grid.pixels_per_meter, 6)

    return metrics


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    ap.add_argument('--ppm', type=float, default=None, help='설정의 pixels_per_meter 덮어쓰기')
    ap.add_argument('--output', type=Path, default=None, help='출력 H5 경로 덮어쓰기')
    ap.add_argument('--no-overlay', action='store_true', help='검증용 PNG 생성 안 함')
    ap.add_argument('--list-stoplines', action='store_true',
                    help='정지선 노드 목록만 출력하고 끝낸다 (id 고를 때)')
    ap.add_argument('--stopline-route', type=Path, default=None,
                    help='이 route yaml 위의 정지선만 굽는다 '
                         '(behavior_planner 의 route_links*.yaml 형식)')
    args = ap.parse_args()

    with open(args.config, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    src = cfg['source']
    link_json = _resolve(src['link_set'])
    node_json = _resolve(src['node_set'])
    ginfo_json = _resolve(src['global_info']) if src.get('global_info') else None
    for p in (link_json, node_json):
        if not p.is_file():
            raise SystemExit(f'소스 없음: {p}')

    data = load_link_node(link_json, node_json, ginfo_json)
    n_valid = len(data.valid_links)
    n_stop = len(data.stop_line_nodes)
    print(f'[load] links={len(data.links)} (valid {n_valid})  '
          f'nodes={len(data.nodes)}  stop_line_nodes={n_stop}')
    if n_valid == 0:
        raise SystemExit('유효한 링크가 없습니다.')

    if args.list_stoplines:
        print_stop_line_table(data)
        return 0

    ppm = float(args.ppm if args.ppm is not None else cfg['grid']['pixels_per_meter'])
    margin = float(cfg['grid']['margin_m'])

    # 링크와 정지선 노드를 모두 감싸도록 AABB 를 잡는다. 차선은 링크에서 유도되므로
    # 최대 width/2 만큼만 밖으로 나가고, 그건 margin 안에 들어온다.
    arrays = [l.points for l in data.valid_links]
    arrays += [n.point.reshape(1, 2) for n in data.stop_line_nodes]
    min_x, min_y, max_x, max_y = compute_bounds(arrays)
    grid = RasterGrid.from_bounds(min_x, min_y, max_x, max_y, ppm, margin)
    print(f'[grid] bounds x[{min_x:.1f},{max_x:.1f}] y[{min_y:.1f},{max_y:.1f}]  '
          f'ppm={ppm}  size={grid.width_px}x{grid.height_px} px  '
          f'offset=({grid.world_offset[0]:.1f}, {grid.world_offset[1]:.1f})')

    # 노드 핀치 보정은 도로와 차선이 같은 중심선을 써야 하므로 한 번만 계산한다.
    geom = cfg.get('geometry', {})
    centerlines = (lane_pinch_corrections(data.valid_links)
                   if geom.get('fix_node_pinch', True) else {})
    if centerlines:
        print(f'[geom] 노드 핀치 보정: 링크 {len(centerlines)}개')

    road = bake_drivable(data, grid, centerlines=centerlines, **cfg.get('drivable', {}))
    # 차선은 '깎인 뒤의' 도로로 클립해야 바깥 절반이 함께 잘린다.
    # 도로 최외곽을 깎은 만큼 최외곽 차선도 안쪽으로 옮긴다 (두께 보존).
    lane_cfg = dict(cfg.get('lane', {}))
    trim = float(cfg.get('drivable', {}).get('outer_trim_m', 0.0))
    if lane_cfg.pop('follow_outer_trim', True) and trim > 0:
        lane_cfg.setdefault(
            'outer_shift_m',
            trim + 0.5 * grid.meters_to_px(
                lane_cfg.get('line_width_m', 0.15),
                minimum=lane_cfg.get('min_thickness_px', 2)) / grid.pixels_per_meter)
    lane = bake_lane(data, grid, centerlines=centerlines,
                     clip_to=road.image if geom.get('clip_lane_to_road', True) else None,
                     **lane_cfg)
    stop_cfg = dict(cfg.get('stopline', {}))
    route_yaml = args.stopline_route or stop_cfg.pop('route_yaml', None)
    if route_yaml:
        ids = stop_line_nodes_on_route(data, _resolve(str(route_yaml)))
        stop_cfg['include_nodes'] = ids
        print(f'[stopline] route {Path(route_yaml).name} 위의 정지선 {len(ids)}개만 굽습니다')
    stop = bake_stopline(data, grid, **stop_cfg)
    for name, res in (('road', road), ('lane', lane), ('stopline', stop)):
        stats = dict(res.stats)
        partial = stats.pop('partially_excluded_roads', [])
        print(f'[bake] {name:9s} {json.dumps(stats, ensure_ascii=False)}')
        if partial:
            # 여러 차로짜리 도로에서 지나간 차로만 빼면, 남은 차로의 차선만 그려져
            # ego 가 차선에 걸친 것처럼 보인다.
            print(f'[warn] {name}: 일부 차로만 제외된 도로 {len(partial)}개 — '
                  f'{", ".join(f"{r}({a}/{b})" for r, a, b in partial[:6])}'
                  f'{" ..." if len(partial) > 6 else ""}')
            print(f'       도로 전체를 빼려면 exclude_roads 에 road_id 를 넣으세요.')

    metrics = compute_metrics(road.image, lane.image, stop.image, grid)
    print(f'[check] {json.dumps(metrics, ensure_ascii=False)}')

    out_h5 = args.output or _resolve(cfg['output']['h5'])
    write_h5(
        out_h5,
        {'road': road.image, 'lane': lane.image, 'stopline': stop.image},
        grid,
        map_id=cfg['map_id'],
        local_origin_in_global=data.local_origin_in_global,
        source_files=data.source_files,
        layer_params={k: cfg.get(k, {}) for k in ('drivable', 'lane', 'stopline')},
        stats={'road': road.stats, 'lane': lane.stats, 'stopline': stop.stats,
               'metrics': metrics},
        stopline_features=stop.features,
        links=link_records(data, centerlines, road, lane),
        repo_root=REPO_ROOT,
    )
    print(f'[write] {out_h5}  ({out_h5.stat().st_size / 1e6:.1f} MB)')

    if not args.no_overlay and cfg['output'].get('overlay_png'):
        png = _resolve(cfg['output']['overlay_png'])
        png.parent.mkdir(parents=True, exist_ok=True)
        overlay = build_overlay(road.image, lane.image, stop.image,
                                int(cfg['output'].get('overlay_max_px', 2000)))
        cv2.imwrite(str(png), overlay)
        print(f'[write] {png}  {overlay.shape[1]}x{overlay.shape[0]}')

        # 전체 지도만으로는 차선 간격과 정지선 방향을 확인할 수 없다.
        # 정지선 노드가 가장 밀집한 지점을 골라 원해상도로 잘라 둔다.
        stops = np.stack([n.point for n in data.stop_line_nodes], axis=0)
        counts = [(np.linalg.norm(stops - p, axis=1) < 40.0).sum() for p in stops]
        center = stops[int(np.argmax(counts))]
        zoom = build_zoom(road.image, lane.image, stop.image, grid, center, 60.0)
        zpng = png.with_name(png.stem + '_zoom.png')
        cv2.imwrite(str(zpng), zoom)
        print(f'[write] {zpng}  {zoom.shape[1]}x{zoom.shape[0]} '
              f'(center=({center[0]:.1f}, {center[1]:.1f}), +-60 m)')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
