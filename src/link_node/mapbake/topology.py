"""링크 기하 연산과 노드-링크 위상.

세 레이어가 모두 여기에 의존한다:
  - 주행가능영역: 중심선 그대로
  - 차선: 중심선의 법선 오프셋
  - 정지선: 노드에 연결된 링크의 heading
"""
from __future__ import annotations

from collections import defaultdict
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from .io_mgeo import Link, MGeoLinkNode

# MGeo 링크는 차로 중심선이라, 인접 차로 중심선 간격이 곧 차로 폭이다.
# (behavior_planner 맵 실측: 같은 road_id 의 ego_lane 1<->2 간격 중앙값 3.50 m
#  = width_start 3.5 m). 그래서 중심선을 ±width/2 오프셋하면 차선 경계가 된다.
INTERSECTION_LINK_TYPE = '1'


def resample_polyline(points: np.ndarray, step_m: float) -> np.ndarray:
    """등간격 리샘플. 원본 점 간격이 들쭉날쭉하면 법선 오프셋이 코너에서 깨진다."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.shape[0] < 2 or step_m <= 0:
        return pts.astype(np.float32)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    keep = seg > 1e-9
    if not keep.any():
        return pts[:1].astype(np.float32)
    # 중복 점 제거 후 누적 거리
    pts = np.vstack([pts[0], pts[1:][keep]])
    seg = seg[keep]
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    n = max(2, int(np.ceil(total / step_m)) + 1)
    target = np.linspace(0.0, total, n)
    out = np.empty((n, 2), dtype=np.float64)
    out[:, 0] = np.interp(target, s, pts[:, 0])
    out[:, 1] = np.interp(target, s, pts[:, 1])
    return out.astype(np.float32)


def tangents(points: np.ndarray) -> np.ndarray:
    """각 점의 단위 접선. 끝점은 한쪽 차분, 내부는 중앙 차분."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.shape[0] < 2:
        return np.zeros_like(pts, dtype=np.float32)
    t = np.gradient(pts, axis=0)
    norm = np.linalg.norm(t, axis=1, keepdims=True)
    norm[norm < 1e-9] = 1.0
    return (t / norm).astype(np.float32)


def normals(points: np.ndarray) -> np.ndarray:
    """접선을 +90도 회전한 좌측 법선 (x, y) -> (-y, x)."""
    t = tangents(points)
    return np.stack([-t[:, 1], t[:, 0]], axis=1)


def offset_polyline(points: np.ndarray, offset_m: float) -> np.ndarray:
    """법선 방향 평행 오프셋. offset_m > 0 이면 진행방향 기준 좌측."""
    n = normals(points)
    return (np.asarray(points, dtype=np.float32) + offset_m * n).astype(np.float32)


def heading_at(points: np.ndarray, at_end: bool) -> Optional[np.ndarray]:
    """폴리라인 끝(또는 시작)에서의 단위 진행 방향."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.shape[0] < 2:
        return None
    if at_end:
        v = pts[-1] - pts[-2]
    else:
        v = pts[1] - pts[0]
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return None
    return (v / n).astype(np.float32)


class LinkTopology:
    """노드 -> 링크 역인덱스와, 노드 위치에서의 진행 방향."""

    def __init__(self, data: MGeoLinkNode) -> None:
        self._data = data
        self._incoming: Dict[str, List[Link]] = defaultdict(list)  # to_node 가 그 노드
        self._outgoing: Dict[str, List[Link]] = defaultdict(list)  # from_node 가 그 노드
        for link in data.valid_links:
            if link.to_node:
                self._incoming[link.to_node].append(link)
            if link.from_node:
                self._outgoing[link.from_node].append(link)

    def incoming(self, node_idx: str) -> List[Link]:
        return self._incoming.get(node_idx, [])

    def outgoing(self, node_idx: str) -> List[Link]:
        return self._outgoing.get(node_idx, [])

    def node_heading(self, node_idx: str) -> Optional[Tuple[np.ndarray, float]]:
        """노드에서의 (진행방향 단위벡터, 대표 차로폭).

        정지선은 '그 지점에 진입하는' 링크의 방향에 수직하므로 incoming 을 우선한다.
        incoming 이 없으면 (교차로 진입부 등) outgoing 의 시작 방향으로 대체한다.
        """
        for links, at_end in ((self.incoming(node_idx), True), (self.outgoing(node_idx), False)):
            vecs = []
            widths = []
            for link in links:
                h = heading_at(link.points, at_end=at_end)
                if h is not None:
                    vecs.append(h)
                    widths.append(link.width_m)
            if vecs:
                # 여러 차로가 같은 노드를 공유하면 방향을 평균낸다 (거의 평행하다).
                mean = np.mean(np.stack(vecs, axis=0), axis=0)
                n = float(np.linalg.norm(mean))
                if n < 1e-6:
                    mean, n = vecs[0], 1.0
                return (mean / n).astype(np.float32), float(np.mean(widths))
        return None


def stop_line_bar_directions(
    nodes: List,
    headings: Dict[str, np.ndarray],
    *,
    group_radius_m: float = 12.0,
    max_heading_diff_deg: float = 30.0,
    max_longitudinal_m: float = 1.5,
    max_perp_error_deg: float = 20.0,
) -> Dict[str, np.ndarray]:
    """같은 진입부의 정지선 노드들을 지나는 직선(= 실제 도색된 바) 방향을 구한다.

    노드 하나의 링크 접선만 쓰면, 정지선 직전에 휘는 차로(우회전 차로 등)에서
    정지선이 기울어진다. 실측상 같은 진입부의 노드들은 **정확히 한 직선 위**에
    놓여 있으므로(잔차 중앙값 0.000 m, 최악 0.000 m), 그 직선이 정지선 방향이다.

    묶는 조건은 세 가지다. 거리만으로 묶으면 서로 다른 진입부가 섞여 바가 도로를
    따라 눕는다 (수직편차 최악 89도). '진행방향 성분이 작다' = 옆에 나란히 있다는
    조건까지 넣으면 최악 9.4도로 떨어진다.

    Returns: node idx -> 단위 바 방향 (2,). 단일 노드이거나 검증에 실패한 노드는
    포함하지 않는다 (호출부에서 링크 접선으로 폴백).
    """
    out: Dict[str, np.ndarray] = {}
    usable = [n for n in nodes if headings.get(n.idx) is not None]
    used = set()
    cos_lim = math.cos(math.radians(max_heading_diff_deg))
    for node in usable:
        if node.idx in used:
            continue
        group = [node]
        used.add(node.idx)
        h0 = headings[node.idx]
        for other in usable:
            if other.idx in used:
                continue
            off = other.point - node.point
            if float(np.linalg.norm(off)) > group_radius_m:
                continue
            if float(np.dot(h0, headings[other.idx])) < cos_lim:
                continue
            if abs(float(np.dot(off, h0))) > max_longitudinal_m:
                continue
            group.append(other)
            used.add(other.idx)
        if len(group) < 2:
            continue
        pts = np.stack([g.point for g in group]).astype(np.float64)
        _, _, vt = np.linalg.svd(pts - pts.mean(axis=0))
        bar = vt[0] / np.linalg.norm(vt[0])
        mean_h = np.mean(np.stack([headings[g.idx] for g in group]), axis=0)
        nrm = float(np.linalg.norm(mean_h))
        if nrm < 1e-6:
            continue
        mean_h = mean_h / nrm
        # 바는 주행방향과 수직해야 한다. 아니면 엉뚱하게 묶인 것이므로 버린다.
        perp_err = 90.0 - math.degrees(math.acos(min(1.0, abs(float(np.dot(bar, mean_h))))))
        if abs(perp_err) > max_perp_error_deg:
            continue
        # 부호를 통일해 두면 그리는 쪽에서 방향을 신경 쓸 필요가 없다.
        if float(np.dot(np.array([-mean_h[1], mean_h[0]]), bar)) < 0:
            bar = -bar
        for g in group:
            out[g.idx] = bar.astype(np.float32)
    return out


def _local_heading(pts: np.ndarray, i: int, span: int = 6) -> Optional[np.ndarray]:
    """i 주변 구간의 단위 진행 방향."""
    lo = max(0, i - span)
    hi = min(len(pts) - 1, i + span)
    if hi <= lo:
        return None
    v = pts[hi] - pts[lo]
    n = float(np.linalg.norm(v))
    return (v / n).astype(np.float32) if n > 1e-9 else None


def lane_pinch_corrections(
    links: List[Link],
    *,
    resample_step_m: float = 0.5,
    endpoint_tol_m: float = 0.3,
    parallel_deg: float = 15.0,
    min_separation_m: float = 1.4,
    max_separation_m: float = 12.0,
    settle_ratio: float = 0.98,
    max_shift_m: float = 4.0,
) -> Dict[str, np.ndarray]:
    """노드 한 점으로 모이는 구간의 중심선을 이웃 차로와 평행하게 되돌린다.

    이 맵은 같은 도로의 여러 차로가 노드 좌표 한 점에서 출발해 부챗살처럼 벌어진다
    (`A2256W000172` 와 `A2256W000176` 은 시작점·끝점이 모두 동일하고 중간에서만
    3.4 m 벌어진다). 그 구간의 중심선은 실제 차로 위치가 아니라, 오프셋하면 도로를
    가로지르는 대각선 차선이 된다. 지우면 차선이 뚝 끊기므로 제자리로 되돌린다.

    한 노드에 셋 이상 모이면 **쌍마다 따로 고치면 서로 덮어써서** 편차가 남는다
    (3차로 노드에서 2.1~2.3 m 잔차). 그래서 노드마다 기준선 하나를 정하고 나머지를
    전부 그 기준선 기준으로 옮긴다. 기준선은 핀치 구간에서 방향이 가장 덜 휘는
    링크다 — 노드로 끌려간 쪽이 크게 휜다.

    교차로에서 직진/좌회전이 갈라지는 것은 정상이므로 건드리지 않는다. 둘을 가르는
    기준은 '가장 많이 벌어진 지점에서 서로 평행한가'(<15도)다. 끝점에서 재면 안 된다
    — 양끝을 모두 공유하는 쌍은 끝 간격이 0 이라 분기로 오판된다.

    보정량에는 상한(`max_shift_m`)을 둔다. 핀치는 최대 한 차로 폭만큼 어긋나는
    현상이므로, 그보다 크게 옮겨야 한다면 짝을 잘못 잡은 것이다. 상한이 없을 때
    한 링크가 7 m(두 차로 폭)나 밀려나 ego 가 차선에 걸친 것처럼 보였다.

    Returns: link idx -> 보정된 중심선 (등간격 리샘플). 손대지 않은 링크는 없음.
    """
    base: Dict[str, np.ndarray] = {
        l.idx: resample_polyline(l.points, resample_step_m) for l in links
    }
    out: Dict[str, np.ndarray] = {}
    shared: Dict[Tuple[str, str], List[Link]] = defaultdict(list)
    for link in links:
        if link.from_node:
            shared[('f', link.from_node)].append(link)
        if link.to_node:
            shared[('t', link.to_node)].append(link)

    cos_lim = math.cos(math.radians(parallel_deg))

    def oriented(idx: str, at_start: bool) -> np.ndarray:
        c = out.get(idx, base[idx])
        return c if at_start else c[::-1]

    def separation(pa: np.ndarray, pb: np.ndarray):
        n = min(len(pa), len(pb))
        if n < 8 or float(np.linalg.norm(pa[0] - pb[0])) > endpoint_tol_m:
            return None
        d = np.linalg.norm(pa[:n] - pb[:n], axis=1)
        # 한 점 튀는 값에 정착 간격이 좌우되지 않게 살짝 다듬는다.
        if n >= 5:
            d = np.convolve(d, np.ones(5) / 5.0, mode='same')
        return d

    for (tag, _node), group in shared.items():
        if len(group) < 2:
            continue
        at_start = tag == 'f'
        curves = {l.idx: oriented(l.idx, at_start) for l in group}

        # 서로 평행한(= 나란한 차로인) 링크만 한 덩어리로 모은다.
        # **양끝 노드를 모두 공유하는 쌍만** 본다. 핀치 인공물은 두 끝이 같은 점에
        # 묶여 중간에서만 벌어지는(0 -> 3.4 -> 0) 모양이다. 한쪽 끝만 공유하고
        # 단조롭게 모이는 건 실제 합류이므로 건드리면 안 된다 — 그걸 '고치면'
        # 남은 차로가 옆 차로 자리로 밀려가, 제외한 차로에 도로가 되살아난다.
        cluster = []
        for i, a in enumerate(group):
            for b in group[i + 1:]:
                if a.from_node != b.from_node or a.to_node != b.to_node:
                    continue
                pa, pb = curves[a.idx], curves[b.idx]
                d = separation(pa, pb)
                if d is None:
                    continue
                k_max = int(np.argmax(d))
                if not (min_separation_m < float(d[k_max]) < max_separation_m):
                    continue
                ha, hb = _local_heading(pa, k_max), _local_heading(pb, k_max)
                if ha is None or hb is None or float(np.dot(ha, hb)) < cos_lim:
                    continue      # 분기 — 정상
                cluster.extend([a, b])
        cluster = list({l.idx: l for l in cluster}.values())
        if len(cluster) < 2:
            continue

        def swing(idx: str) -> float:
            p = curves[idx]
            h0, h1 = _local_heading(p, 0), _local_heading(p, min(len(p) - 1, 60))
            if h0 is None or h1 is None:
                return 0.0
            return math.acos(max(-1.0, min(1.0, float(np.dot(h0, h1)))))

        ref = min(cluster, key=lambda l: swing(l.idx))
        pr = curves[ref.idx]
        nref = normals(pr)

        for link in cluster:
            if link.idx is ref.idx or link.idx == ref.idx:
                continue
            pd = curves[link.idx]
            d = separation(pr, pd)
            if d is None:
                continue
            sep = float(d.max())
            if not (min_separation_m < sep < max_separation_m):
                continue
            hit = np.nonzero(d >= settle_ratio * sep)[0]
            if not hit.size or hit[0] == 0:
                continue
            k = int(hit[0])
            sign = math.copysign(1.0, float(np.dot(pd[k] - pr[k], nref[k])))
            fixed = pr[:k] + (sep * sign) * nref[:k]
            # 핀치는 최대 한 차로 폭만큼 어긋나는 현상이다. 그보다 크게 옮겨야
            # 한다면 짝을 잘못 잡은 것이므로 버린다. 비교 대상은 '지금까지 보정된
            # 상태'가 아니라 **원본**이어야 한다 — 양끝을 각각 보정하면 조금씩
            # 밀린 게 쌓여 상한을 통과해 버린다.
            orig = base[link.idx] if at_start else base[link.idx][::-1]
            m_ = min(k, len(orig))
            if float(np.linalg.norm(fixed[:m_] - orig[:m_], axis=1).max()) > max_shift_m:
                continue

            target = (out.get(link.idx, base[link.idx])).copy()
            if at_start:
                target[:k] = fixed
            else:
                target[len(target) - k:] = fixed[::-1]
            out[link.idx] = target

    return out


def extend_polyline(points: np.ndarray, head_m: float, tail_m: float) -> np.ndarray:
    """끝 방향을 그대로 이어 폴리라인을 늘린다."""
    pts = np.asarray(points, dtype=np.float32)
    if pts.shape[0] < 2:
        return pts
    parts = []
    if head_m > 0:
        h = heading_at(pts, at_end=False)
        if h is not None:
            parts.append((pts[0] - head_m * h).reshape(1, 2))
    parts.append(pts)
    if tail_m > 0:
        h = heading_at(pts, at_end=True)
        if h is not None:
            parts.append((pts[-1] + tail_m * h).reshape(1, 2))
    return np.vstack(parts).astype(np.float32)


def merge_taper_ends(links: List[Link]) -> Dict[str, Tuple[bool, bool]]:
    """차로 수가 줄어드는(합류) 노드에 닿는 링크 끝을 표시한다.

    주행가능영역은 차로별 통로의 합집합이라, 차로가 끝나면 그 통로도 그 자리에서
    직각으로 잘린다. 실제 도로는 합류부에서 서서히 좁아지므로 그 끝을 조금 늘려
    계단을 완만하게 만든다. 진짜 막다른 끝(이어지는 링크가 없는 노드)은 늘리지
    않는다 — 없는 도로를 만들어내면 안 된다.

    Returns: link idx -> (시작쪽을 늘릴지, 끝쪽을 늘릴지)
    """
    incoming: Dict[str, int] = defaultdict(int)
    outgoing: Dict[str, int] = defaultdict(int)
    for l in links:
        if l.to_node:
            incoming[l.to_node] += 1
        if l.from_node:
            outgoing[l.from_node] += 1
    out: Dict[str, Tuple[bool, bool]] = {}
    for l in links:
        # 끝쪽: 들어오는 게 나가는 것보다 많으면 합류
        tail = bool(l.to_node and outgoing[l.to_node] > 0
                    and incoming[l.to_node] > outgoing[l.to_node])
        # 시작쪽: 나가는 게 들어오는 것보다 많으면 분기 (반대 방향의 합류)
        head = bool(l.from_node and incoming[l.from_node] > 0
                    and outgoing[l.from_node] > incoming[l.from_node])
        if head or tail:
            out[l.idx] = (head, tail)
    return out


def outer_side_flags(
    links: List[Link],
    centerlines: Optional[Dict[str, np.ndarray]] = None,
    *,
    resample_step_m: float = 0.5,
    match_tol_m: float = 0.8,
    min_overlap: float = 0.2,
) -> Dict[str, Tuple[bool, bool]]:
    """링크별로 (좌측이 최외곽인가, 우측이 최외곽인가) 를 판정한다.

    한 차로 폭만큼 옆을 짚어 다른 차로 중심선이 있는지 본다. 없으면 그쪽이 도로의
    바깥 끝이다. 도로 폭을 좁힐 때 최외곽 차선만 안쪽으로 옮기고 차로 사이 선은
    건드리지 않기 위해 필요하다.
    """
    from scipy.spatial import cKDTree
    curves = {l.idx: (centerlines or {}).get(l.idx,
                                             resample_polyline(l.points, resample_step_m))
              for l in links}
    stacked = [c for c in curves.values() if c.shape[0] >= 2]
    if not stacked:
        return {}
    tree = cKDTree(np.vstack(stacked))
    out: Dict[str, Tuple[bool, bool]] = {}
    for link in links:
        c = curves[link.idx]
        if c.shape[0] < 4:
            out[link.idx] = (True, True)
            continue
        n = normals(c)
        flags = []
        for sgn in (+1, -1):
            dist, _ = tree.query(c + sgn * link.width_m * n)
            has_neighbor = float((dist < match_tol_m).mean()) >= min_overlap
            flags.append(not has_neighbor)
        out[link.idx] = (flags[0], flags[1])
    return out


def trim_polyline(points: np.ndarray, head_m: float, tail_m: float) -> np.ndarray:
    """앞/뒤에서 주어진 길이만큼 잘라낸 폴리라인."""
    pts = np.asarray(points, dtype=np.float32)
    if pts.shape[0] < 2:
        return pts
    seg = np.linalg.norm(np.diff(pts.astype(np.float64), axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    lo, hi = max(0.0, head_m), total - max(0.0, tail_m)
    if hi - lo < 1e-6:
        return pts[:0]
    keep = (s >= lo) & (s <= hi)
    return pts[keep]


def average_curve_radius(points: np.ndarray) -> float:
    """폴리라인의 평균 곡률반경 = 길이 / 총 방향변화(rad).

    3점 외접원 반경은 점 간격 노이즈에 크게 흔들린다 (총회전 22.9도인 링크에서
    최소반경 2.55 m 가 나오기도 한다). 길이와 누적 방향변화만 쓰면 그 노이즈가
    상쇄된다.

    '총 회전각' 자체를 기준으로 쓰면 안 된다 — 주행시험장 오벌 링크는 1186 m 에
    걸쳐 334도를 돌지만 반경은 203 m 인 정상 도로다.
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] < 3:
        return float('inf')
    v = np.diff(pts, axis=0)
    seg = np.linalg.norm(v, axis=1)
    keep = seg > 1e-9
    if keep.sum() < 2:
        return float('inf')
    v, seg = v[keep], seg[keep]
    ang = np.arctan2(v[:, 1], v[:, 0])
    d = np.diff(ang)
    d = (d + np.pi) % (2 * np.pi) - np.pi
    turn = abs(float(d.sum()))
    if turn < 1e-6:
        return float('inf')
    return float(seg.sum() / turn)


def is_intersection(link: Link) -> bool:
    """교차로 내부 링크.

    behavior_planner 맵 실측: link_type == '1' 인 256개 링크는 100% 정지선 노드에
    연결되고 related_signal 이 straight/left/right_unprotected 로 채워져 있다.
    실제 도로도 교차로 안에는 차선을 긋지 않으므로 차선 레이어에서 제외한다.
    """
    return link.link_type == INTERSECTION_LINK_TYPE
