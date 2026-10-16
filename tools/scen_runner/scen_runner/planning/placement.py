"""장애물 배치: 경로 좌표로 놓기, 기본 조건 검사, 가능 영역 지도, 랜덤 배치."""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

import numpy as np

from scen_runner.planning.feasibility import AvoidanceChecker, CaseResult
from scen_runner.planning.vehicle import Obstacle

# 가능 영역 지도 칸 상태
INVALID = 0        # 도로 밖이거나 다른 장애물과 겹침
NOT_BLOCKING = 1   # 내 차로를 막지 않음
INFEASIBLE = 2     # 어떤 속도 조건에서 회피 불가
LEFT_ONLY = 3
RIGHT_ONLY = 4
BOTH = 5
MIXED = 6          # 모든 조건에서 가능하지만 속도 조건마다 가능한 방향이 다름
FEASIBLE_CODES = (LEFT_ONLY, RIGHT_ONLY, BOTH, MIXED)


@dataclass
class ModelSize:
    name: str
    length: float
    width: float
    height: float
    center_offset: tuple[float, float] = (0.0, 0.0)
    morai_model: str | None = None

    @classmethod
    def from_catalog(cls, name: str, entry: dict) -> "ModelSize":
        off = entry.get("center_offset") or [0.0, 0.0]
        return cls(name=name, length=float(entry["length"]), width=float(entry["width"]),
                   height=float(entry["height"]), center_offset=(float(off[0]), float(off[1])),
                   morai_model=entry.get("morai_model"))


def catalog_sizes(catalog: dict, measured_only: bool = False) -> dict[str, ModelSize]:
    out = {}
    exclude = set(catalog.get("exclude", []))
    for name, entry in catalog.get("models", {}).items():
        if name in exclude or (entry.get("morai_model") in exclude):
            continue
        if measured_only and not entry.get("measured"):
            continue
        out[name] = ModelSize.from_catalog(name, entry)
    return out


def obstacle_at(checker: AvoidanceChecker, size: ModelSize, s: float, d: float,
                yaw_rel_deg: float = 0.0, source: str = "gui") -> Obstacle:
    """경로 좌표(상자 중심)로 장애물을 만든다. yaw_rel 은 도로 진행방향 기준."""
    route = checker.scene.route
    x, y = route.to_xy(s, d)
    _, _, th, _ = route.sample(s)
    return Obstacle(x=float(x), y=float(y), yaw=float(th) + math.radians(yaw_rel_deg),
                    length=size.length, width=size.width, height=size.height,
                    model=size.name, morai_model=size.morai_model,
                    z=float(route.z_at(s)), source=source)


def spawn_pose(ob: Obstacle, size: ModelSize | None) -> tuple[float, float]:
    """MORAI 스폰 위치. 모델 원점이 상자 중심에서 center_offset 만큼 떨어져 있다."""
    if size is None:
        return ob.x, ob.y
    fx, fy = size.center_offset
    c, s = math.cos(ob.yaw), math.sin(ob.yaw)
    return ob.x - (fx * c - fy * s), ob.y - (fx * s + fy * c)


def _edge_points(ob: Obstacle, per_edge: int = 6) -> np.ndarray:
    cs = ob.corners()
    pts = [cs[i] + (cs[(i + 1) % 4] - cs[i]) * t
           for i in range(4) for t in np.linspace(0.0, 1.0, per_edge, endpoint=False)]
    return np.vstack([pts, [[ob.x, ob.y]]])


def box_gap(a: Obstacle, b: Obstacle) -> float:
    pa, pb = _edge_points(a), _edge_points(b)
    return float(min(b.signed_distance(pa[:, 0], pa[:, 1]).min(),
                     a.signed_distance(pb[:, 0], pb[:, 1]).min()))


def lateral_span(checker: AvoidanceChecker, ob: Obstacle) -> tuple[float, float]:
    pts = _edge_points(ob)
    _, d = checker.scene.route.project(pts[:, 0], pts[:, 1])
    return float(d.min()), float(d.max())


def blocks_lane(checker: AvoidanceChecker, ob: Obstacle) -> bool:
    lo, hi = lateral_span(checker, ob)
    half = 0.5 * checker.vehicle.width
    return lo < half and hi > -half


def basic_check(checker: AvoidanceChecker, ob: Obstacle, others: list[Obstacle],
                min_gap: float) -> str:
    """'' 이면 통과. 아니면 실패 이유."""
    pts = _edge_points(ob)
    if not checker.scene.grid.is_drivable(pts[:, 0], pts[:, 1]).all():
        return "도로(주행가능영역) 밖"
    for other in others:
        if box_gap(ob, other) < min_gap:
            return "다른 장애물과 너무 가까움"
    return ""


def classify(results: list[CaseResult]) -> int:
    if any(r.start_blocked for r in results):
        return INFEASIBLE
    if not all(r.blocked for r in results):
        return NOT_BLOCKING
    if not all(r.feasible for r in results):
        return INFEASIBLE
    left = all(r.sides.get("left") and r.sides["left"].feasible for r in results)
    right = all(r.sides.get("right") and r.sides["right"].feasible for r in results)
    if left and right:
        return BOTH
    if left:
        return LEFT_ONLY
    if right:
        return RIGHT_ONLY
    return MIXED


@dataclass
class RegionMap:
    s: np.ndarray
    d: np.ndarray
    status: np.ndarray
    x: np.ndarray
    y: np.ndarray
    size: ModelSize
    yaw_rel_deg: float

    def counts(self) -> dict:
        names = {INVALID: "도로 밖/겹침", NOT_BLOCKING: "차로 안 막음",
                 INFEASIBLE: "회피 불가", LEFT_ONLY: "왼쪽만", RIGHT_ONLY: "오른쪽만",
                 BOTH: "양쪽 가능", MIXED: "속도별 방향 다름"}
        return {names[k]: int((self.status == k).sum()) for k in names}


def region_map(checker: AvoidanceChecker, size: ModelSize, others: list[Obstacle],
               s_range, d_range, step_s: float, step_d: float, yaw_rel_deg: float = 0.0,
               min_gap: float = 1.0, progress=None, cancelled=None) -> RegionMap:
    ss = np.arange(s_range[0], s_range[1] + 1e-9, step_s)
    dd = np.arange(d_range[0], d_range[1] + 1e-9, step_d)
    status = np.zeros((len(dd), len(ss)), dtype=np.int8)
    xs = np.zeros_like(status, dtype=float)
    ys = np.zeros_like(status, dtype=float)
    total = status.size
    done = 0
    for j, d in enumerate(dd):
        for i, s in enumerate(ss):
            if cancelled is not None and cancelled():
                break
            ob = obstacle_at(checker, size, float(s), float(d), yaw_rel_deg)
            xs[j, i], ys[j, i] = ob.x, ob.y
            if basic_check(checker, ob, others, min_gap):
                status[j, i] = INVALID
            elif not blocks_lane(checker, ob):
                status[j, i] = NOT_BLOCKING
            else:
                status[j, i] = classify(checker.evaluate(others + [ob]))
            done += 1
            if progress is not None:
                progress(done, total)
    return RegionMap(s=ss, d=dd, status=status, x=xs, y=ys, size=size,
                     yaw_rel_deg=yaw_rel_deg)


@dataclass
class RandomRequest:
    sizes: list[ModelSize]
    count: int
    s_range: tuple[float, float]
    d_range: tuple[float, float]
    yaw_range_deg: tuple[float, float]
    min_gap: float
    require_lane_block: bool
    require_all_cases: bool
    max_tries: int
    seed: int


@dataclass
class RandomLayout:
    obstacles: list[Obstacle]
    seed: int
    tries: int
    results: list[CaseResult] = field(default_factory=list)
    message: str = ""
    picks: list[dict] = field(default_factory=list)


def random_layout(checker: AvoidanceChecker, req: RandomRequest, fixed: list[Obstacle] | None = None,
                  progress=None, cancelled=None) -> RandomLayout:
    """조건을 만족할 때까지 하나씩 뽑는다. 같은 seed 면 같은 배치가 나온다."""
    rng = random.Random(req.seed)
    fixed = list(fixed or [])
    placed: list[Obstacle] = []
    picks: list[dict] = []
    tries = 0
    last_results: list[CaseResult] = []
    for k in range(req.count):
        ok = False
        while tries < req.max_tries and not ok:
            if cancelled is not None and cancelled():
                return RandomLayout(placed, req.seed, tries, last_results, "취소됨", picks)
            tries += 1
            size = rng.choice(req.sizes)
            s = rng.uniform(*req.s_range)
            d = rng.uniform(*req.d_range)
            yaw = rng.uniform(*req.yaw_range_deg)
            ob = obstacle_at(checker, size, s, d, yaw, source="random")
            ob.tag = f"seed{req.seed}-{k + 1}"
            if progress is not None:
                progress(tries, req.max_tries)
            if basic_check(checker, ob, fixed + placed, req.min_gap):
                continue
            if req.require_lane_block and not blocks_lane(checker, ob):
                continue
            results = checker.evaluate(fixed + placed + [ob])
            if req.require_all_cases and not all(r.feasible for r in results):
                continue
            if not req.require_all_cases and not any(r.feasible for r in results):
                continue
            placed.append(ob)
            picks.append({"model": size.name, "s": s, "d": d, "yaw_rel_deg": yaw})
            last_results = results
            ok = True
        if not ok:
            msg = f"{req.max_tries}번 시도 안에 {k + 1}번째 장애물 위치를 찾지 못했습니다."
            return RandomLayout(placed, req.seed, tries, last_results, msg, picks)
    return RandomLayout(placed, req.seed, tries, last_results,
                        f"{len(placed)}개 배치 완료 ({tries}번 시도)", picks)
