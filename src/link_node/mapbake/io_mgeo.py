"""link_set.json / node_set.json 로더와 스키마 정규화.

MGeo 내보내기는 필드 타입이 판본마다 흔들린다 (좌표가 문자열, lane_type 이 스칼라
또는 리스트 등). 여기서 한 번 정규화해 두고 이후 단계는 정규화된 형태만 다룬다.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

# 링크에 폭 정보가 없거나 비정상일 때 쓰는 기본 차로 폭 (m).
DEFAULT_LANE_WIDTH_M = 3.5
# 이보다 좁은 width 값은 오염된 것으로 보고 기본값으로 대체한다.
MIN_PLAUSIBLE_WIDTH_M = 0.5


def _to_xy(raw: Any) -> np.ndarray:
    """points/point 를 (N, 2) float32 로. 문자열 좌표도 흡수한다."""
    if raw is None:
        return np.zeros((0, 2), dtype=np.float32)
    arr = np.asarray(raw, dtype=object)
    if arr.ndim == 1:  # 단일 점 (node 의 'point')
        arr = arr[None, :]
    if arr.ndim != 2 or arr.shape[1] < 2:
        return np.zeros((0, 2), dtype=np.float32)
    try:
        out = np.array([[float(row[0]), float(row[1])] for row in arr], dtype=np.float32)
    except (TypeError, ValueError):
        return np.zeros((0, 2), dtype=np.float32)
    return out


def _as_bool(v: Any) -> bool:
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        return v.strip().lower() in ('true', '1', 'yes')
    return bool(v)


@dataclass
class Link:
    idx: str
    points: np.ndarray           # (N, 2) float32, 차로 중심선
    from_node: Optional[str]
    to_node: Optional[str]
    width_m: float
    link_type: str
    can_move_left: bool
    can_move_right: bool
    ego_lane: Optional[int]
    road_id: str

    @property
    def valid(self) -> bool:
        return self.points.shape[0] >= 2


@dataclass
class Node:
    idx: str
    point: np.ndarray            # (2,) float32
    node_type: str
    on_stop_line: bool
    traffic_light_id: str


@dataclass
class MGeoLinkNode:
    links: List[Link]
    nodes: Dict[str, Node]
    global_info: Dict[str, Any] = field(default_factory=dict)
    source_files: Dict[str, str] = field(default_factory=dict)

    @property
    def valid_links(self) -> List[Link]:
        return [l for l in self.links if l.valid]

    @property
    def stop_line_nodes(self) -> List[Node]:
        return [n for n in self.nodes.values() if n.on_stop_line]

    @property
    def local_origin_in_global(self) -> List[float]:
        v = self.global_info.get('local_origin_in_global') or []
        return [float(x) for x in v]


def _link_width(entry: Dict[str, Any]) -> float:
    for key in ('width_start', 'width_end'):
        try:
            w = float(entry.get(key))
        except (TypeError, ValueError):
            continue
        if w >= MIN_PLAUSIBLE_WIDTH_M:
            return w
    return DEFAULT_LANE_WIDTH_M


def _parse_link(entry: Dict[str, Any]) -> Link:
    try:
        ego_lane = int(entry.get('ego_lane'))
    except (TypeError, ValueError):
        ego_lane = None
    return Link(
        idx=str(entry.get('idx', '')),
        points=_to_xy(entry.get('points')),
        from_node=entry.get('from_node_idx') or None,
        to_node=entry.get('to_node_idx') or None,
        width_m=_link_width(entry),
        link_type=str(entry.get('link_type')),
        can_move_left=_as_bool(entry.get('can_move_left_lane')),
        can_move_right=_as_bool(entry.get('can_move_right_lane')),
        ego_lane=ego_lane,
        road_id=str(entry.get('road_id') or ''),
    )


def _parse_node(entry: Dict[str, Any]) -> Node:
    return Node(
        idx=str(entry.get('idx', '')),
        point=_to_xy(entry.get('point')).reshape(-1)[:2],
        node_type=str(entry.get('node_type')),
        on_stop_line=_as_bool(entry.get('on_stop_line')),
        traffic_light_id=str(entry.get('traffic_light_id') or ''),
    )


def _read_json(path: Path) -> Any:
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def load_link_node(
    link_json: Path,
    node_json: Path,
    global_info_json: Optional[Path] = None,
) -> MGeoLinkNode:
    """link_set / node_set (+ global_info) 를 읽어 정규화된 구조로 반환."""
    link_raw = _read_json(link_json)
    node_raw = _read_json(node_json)
    if not isinstance(link_raw, list):
        raise ValueError(f'link_set 최상위는 리스트여야 합니다: {link_json}')
    if not isinstance(node_raw, list):
        raise ValueError(f'node_set 최상위는 리스트여야 합니다: {node_json}')

    links = [_parse_link(e) for e in link_raw]
    nodes = {}
    for e in node_raw:
        n = _parse_node(e)
        if n.point.shape[0] == 2:
            nodes[n.idx] = n

    ginfo: Dict[str, Any] = {}
    if global_info_json is not None and Path(global_info_json).is_file():
        raw = _read_json(global_info_json)
        if isinstance(raw, dict):
            # mgeo_file_hash 는 dict 를 문자열로 덤프한 거대한 값이라 제외한다.
            ginfo = {k: v for k, v in raw.items() if k != 'mgeo_file_hash'}

    sources = {
        'link_set': str(link_json),
        'node_set': str(node_json),
    }
    if global_info_json is not None:
        sources['global_info'] = str(global_info_json)

    return MGeoLinkNode(links=links, nodes=nodes, global_info=ginfo, source_files=sources)
