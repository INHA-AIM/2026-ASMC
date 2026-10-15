"""정적 장애물 회피 가능 여부와 회피 경로 계산.

회피 경로는 Frenet 경로 생성기와 같은 형태로 만든다:
차로 중심(d=0) -> 부드러운 5차 곡선으로 옆 이동(d=D) -> 유지 -> 5차 곡선으로 복귀(d=0).
경로는 뒷바퀴 축이 지나는 선이며 다음을 모두 지켜야 "회피 가능"이다.

- 곡률 <= max_curvature, 옆 가속도 v^2*k <= max_lateral_accel, 바퀴 조향 속도 <= max_steer_rate
- 차체가 장애물과 clearance.obstacle_m 이상 떨어짐
- 차체가 실선(노란선, 흰 실선)에 닿지 않음, 주행가능영역 경계와 clearance.road_edge_m 이상
- 체크포인트 지점에서 경로 중심과의 거리 <= clearance.checkpoint_m
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from scen_runner.planning.map_scene import StaticZoneScene
from scen_runner.planning.vehicle import Obstacle, VehicleSpec


def smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t))


def smoothstep_d1(t):
    t = np.clip(t, 0.0, 1.0)
    return 30.0 * t * t * (1.0 - t) ** 2


def smoothstep_d2(t):
    t = np.clip(t, 0.0, 1.0)
    return 60.0 * t * (1.0 - t) * (1.0 - 2.0 * t)


@dataclass(frozen=True)
class SpeedCase:
    case_id: str
    label: str
    s_start: float
    v0: float
    vmax: float
    accel: float

    def speed(self, s):
        travel = np.maximum(np.asarray(s, dtype=float) - self.s_start, 0.0)
        return np.minimum(self.vmax, np.sqrt(self.v0 ** 2 + 2.0 * self.accel * travel))


def build_speed_cases(cfg: dict, scene: StaticZoneScene, vehicle: VehicleSpec):
    cases = []
    for c in cfg["speed_cases"]:
        if "start_station_m" in c:
            s_start = float(c["start_station_m"])
        elif c["start"] == "stopline":
            s_start = scene.stopline_s - vehicle.front_extent - float(c.get("stop_gap_m", 1.0))
        else:
            s_start = scene.stopline_s - float(c["start_distance_m"])
        cases.append(SpeedCase(case_id=c["id"], label=c["label"], s_start=s_start,
                               v0=float(c["initial_speed_kmh"]) / 3.6,
                               vmax=float(c["max_speed_kmh"]) / 3.6,
                               accel=float(cfg["limits"]["max_accel"])))
    return cases


@dataclass
class Maneuver:
    offset: float
    s1: float
    x1: float
    s2: float
    x2: float

    def lateral(self, s):
        s = np.asarray(s, dtype=float)
        t1 = (s - self.s1) / self.x1
        t2 = (s - self.s2) / self.x2
        d = self.offset * (smoothstep(t1) - smoothstep(t2))
        dp = self.offset * (smoothstep_d1(t1) * ((t1 > 0) & (t1 < 1)) / self.x1
                            - smoothstep_d1(t2) * ((t2 > 0) & (t2 < 1)) / self.x2)
        dpp = self.offset * (smoothstep_d2(t1) * ((t1 > 0) & (t1 < 1)) / self.x1 ** 2
                             - smoothstep_d2(t2) * ((t2 > 0) & (t2 < 1)) / self.x2 ** 2)
        return d, dp, dpp


@dataclass
class PathSamples:
    s: np.ndarray
    d: np.ndarray
    x: np.ndarray
    y: np.ndarray
    heading: np.ndarray
    kappa: np.ndarray
    speed: np.ndarray
    obstacle_gap: np.ndarray
    edge_gap: np.ndarray
    line_gap: np.ndarray


@dataclass
class SideResult:
    side: str
    feasible: bool
    reason: str = ""
    maneuver: Maneuver | None = None
    path: PathSamples | None = None

    @property
    def label(self) -> str:
        return "왼쪽" if self.side == "left" else "오른쪽"


@dataclass
class CaseResult:
    case: SpeedCase
    blocked: bool
    start_blocked: bool = False
    note: str = ""
    sides: dict = field(default_factory=dict)
    straight: PathSamples | None = None

    @property
    def feasible(self) -> bool:
        if self.start_blocked:
            return False
        return (not self.blocked) or any(r.feasible for r in self.sides.values())

    def feasible_sides(self):
        return [r for r in self.sides.values() if r.feasible]


@dataclass
class _Segments:
    """같은 (옆 이동량, 길이) 의 전이 구간 묶음. 행 = 시작 위치."""

    start_idx: np.ndarray
    steps: int
    s: np.ndarray
    x: np.ndarray
    y: np.ndarray
    th: np.ndarray
    lateral: np.ndarray


class AvoidanceChecker:
    def __init__(self, scene: StaticZoneScene, cfg: dict):
        self.scene = scene
        self.cfg = cfg
        self.vehicle = VehicleSpec.from_config(cfg)
        lim, gap, srch = cfg["limits"], cfg["clearance"], cfg["search"]
        self.kmax = min(float(lim["max_curvature"]), self.vehicle.kinematic_max_curvature)
        self.alat = float(lim["max_lateral_accel"])
        self.steer_rate = float(lim["max_steer_rate"])
        self.gap_obstacle = float(gap["obstacle_m"])
        self.gap_edge = float(gap["road_edge_m"])
        self.gap_line = float(gap["solid_line_m"])
        self.cp_tol = float(gap["checkpoint_m"])
        self.pose_step = float(srch["pose_step_m"])
        self.station_stride = max(1, int(round(float(srch["station_step_m"]) / self.pose_step)))
        self.transitions = [float(x) for x in srch["transition_lengths_m"]]
        self.per_offset = int(srch["transitions_per_offset"])
        self.s_goal = scene.checkpoint_s + float(srch["goal_after_checkpoint_m"])
        self.cases = build_speed_cases(cfg, scene, self.vehicle)
        self.offsets = {"left": self._offset_list(srch, "left"),
                        "right": self._offset_list(srch, "right")}
        self.r = self.vehicle.circle_radius
        self._cache: dict = {}

    # ------------------------------------------------------------ geometry
    @staticmethod
    def _offset_list(srch: dict, side: str):
        fine = srch["lateral_offsets_m"]["fine"]
        coarse = srch["lateral_offsets_m"]["coarse"]
        far = float(coarse["max_left" if side == "left" else "max_right"])
        vals = list(np.arange(fine["step"], fine["max"] + 1e-9, fine["step"]))
        vals += list(np.arange(fine["max"] + coarse["step"], far + 1e-9, coarse["step"]))
        sign = 1.0 if side == "left" else -1.0
        return [sign * round(float(v), 3) for v in vals]

    def _poses(self, s, d, dp, dpp):
        x_r, y_r, th_r, k_r = self.scene.route.sample(s)
        one = 1.0 - k_r * d
        x = x_r - d * np.sin(th_r)
        y = y_r + d * np.cos(th_r)
        dth = np.arctan2(dp, one)
        cos = np.cos(dth)
        kappa = ((dpp + k_r * dp * np.tan(dth)) * cos * cos / one + k_r) * cos / one
        ds_path = np.sqrt(one * one + dp * dp)
        return x, y, th_r + dth, kappa, ds_path

    def _dynamics_ok(self, kappa, ds_path, speed):
        """행마다(마지막 축) 곡률·옆 가속도·조향 속도 검사."""
        ak = np.abs(kappa)
        ok = np.all(ak <= self.kmax + 1e-9, axis=-1)
        ok &= np.all(speed * speed * ak <= self.alat + 1e-9, axis=-1)
        delta = np.arctan(self.vehicle.wheelbase * kappa)
        ddelta = np.gradient(delta, self.pose_step, axis=-1)
        rate = np.abs(ddelta) * speed / ds_path
        ok &= np.all(rate <= self.steer_rate + 1e-9, axis=-1)
        return ok

    def _map_gaps(self, x, y, th):
        cx, cy = self.vehicle.circle_centers(np.ravel(x), np.ravel(y), np.ravel(th))
        g = self.scene.grid
        edge_distance, line_distance = g.distances(cx, cy)
        edge = (edge_distance - self.r).min(axis=1)
        line = (line_distance - self.r).min(axis=1)
        return edge.reshape(np.shape(x)), line.reshape(np.shape(x))

    def _obstacle_gap(self, x, y, th, obstacles):
        shape = np.shape(x)
        gap = np.full(int(np.prod(shape)), np.inf)
        if not obstacles:
            return gap.reshape(shape)
        xf, yf, tf = np.ravel(x), np.ravel(y), np.ravel(th)
        reach = self.vehicle.length + self.r + 1.0
        for ob in obstacles:
            ob_reach = 0.5 * math.hypot(ob.length, ob.width) + reach + self.gap_obstacle
            near = np.hypot(xf - ob.x, yf - ob.y) < ob_reach
            if not near.any():
                continue
            cx, cy = self.vehicle.circle_centers(xf[near], yf[near], tf[near])
            g = (ob.signed_distance(cx, cy) - self.r).min(axis=1)
            gap[near] = np.minimum(gap[near], g)
        return gap.reshape(shape)

    def _pose_ok(self, edge, line, obs):
        return (edge > self.gap_edge) & (line > self.gap_line) & (obs > self.gap_obstacle)

    # ------------------------------------------------------------ cached map part
    def _case_cache(self, case: SpeedCase):
        key = case.case_id
        if key in self._cache:
            return self._cache[key]
        n = int(math.floor((self.s_goal - case.s_start) / self.pose_step)) + 1
        stations = case.s_start + self.pose_step * np.arange(n)
        cache = {"stations": stations, "segments": {}, "hold_map": {}}
        self._cache[key] = cache
        return cache

    def _hold_map(self, case: SpeedCase, offset: float):
        cache = self._case_cache(case)
        if offset not in cache["hold_map"]:
            st = cache["stations"]
            d = np.full_like(st, offset)
            x, y, th, _, _ = self._poses(st, d, np.zeros_like(st), np.zeros_like(st))
            edge, line = self._map_gaps(x, y, th)
            cache["hold_map"][offset] = (x, y, th, edge, line)
        return cache["hold_map"][offset]

    def _segments(self, case: SpeedCase, offset: float, length: float, kind: str):
        """kind='out' 은 0 -> offset, 'ret' 은 offset -> 0 전이."""
        cache = self._case_cache(case)
        key = (kind, offset, length)
        if key in cache["segments"]:
            return cache["segments"][key]
        st = cache["stations"]
        steps = int(round(length / self.pose_step))
        rows = np.arange(0, len(st) - steps, self.station_stride)
        if len(rows) == 0:
            cache["segments"][key] = None
            return None
        tau = np.arange(steps + 1) / steps
        prof, prof1, prof2 = smoothstep(tau), smoothstep_d1(tau), smoothstep_d2(tau)
        if kind == "ret":
            prof, prof1, prof2 = 1.0 - prof, -prof1, -prof2
        s = st[rows][:, None] + self.pose_step * np.arange(steps + 1)[None, :]
        d = np.broadcast_to(offset * prof, s.shape)
        dp = np.broadcast_to(offset * prof1 / length, s.shape)
        dpp = np.broadcast_to(offset * prof2 / length ** 2, s.shape)
        x, y, th, kappa, ds_path = self._poses(s, d, dp, dpp)
        ok = self._dynamics_ok(kappa, ds_path, case.speed(s))
        if ok.any():
            edge, line = self._map_gaps(x[ok], y[ok], th[ok])
            ok[ok] = np.all((edge > self.gap_edge) & (line > self.gap_line), axis=1)
        seg = _Segments(start_idx=rows[ok], steps=steps, s=s[ok], x=x[ok], y=y[ok],
                        th=th[ok], lateral=offset * prof)
        cache["segments"][key] = seg
        return seg

    def _lengths_for(self, case: SpeedCase, offset: float):
        """동역학상 가능한 전이 길이 후보(최고 속도 기준 가장 짧은 것부터 per_offset 개)."""
        key = ("lengths", case.case_id, offset)
        cache = self._case_cache(case)
        if key in cache["segments"]:
            return cache["segments"][key]
        chosen = []
        feasible_at_vmax = 0
        for length in self.transitions:
            tau = np.linspace(0.0, 1.0, int(round(length / self.pose_step)) + 1)
            kappa = offset * smoothstep_d2(tau) / length ** 2
            dp = offset * smoothstep_d1(tau) / length
            ds_path = np.sqrt(1.0 + dp * dp)
            fast = self._dynamics_ok(kappa, ds_path, np.full_like(tau, case.vmax))
            slow = np.all(np.abs(kappa) <= self.kmax + 1e-9)
            if fast:
                feasible_at_vmax += 1
            if slow and (case.v0 < case.vmax or fast):
                chosen.append(length)
            if feasible_at_vmax >= self.per_offset:
                break
        cache["segments"][key] = chosen
        return chosen

    # ------------------------------------------------------------ evaluation
    def evaluate(self, obstacles: list[Obstacle]) -> list[CaseResult]:
        return [self.evaluate_case(case, obstacles) for case in self.cases]

    def evaluate_case(self, case: SpeedCase, obstacles: list[Obstacle]) -> CaseResult:
        x0, y0, th0, e0, l0 = self._hold_map(case, 0.0)
        o0 = self._obstacle_gap(x0, y0, th0, obstacles)
        free0 = self._pose_ok(e0, l0, o0)
        straight = self._samples_for(case, None, obstacles)
        result = CaseResult(case=case, blocked=not bool(free0.all()), straight=straight)
        if not free0[0]:
            result.start_blocked = True
            result.note = "출발 위치에서 이미 장애물·실선·연석과 겹칩니다."
            return result
        if not result.blocked:
            result.note = "장애물이 차로를 막지 않아 그대로 지나갈 수 있습니다."
            return result
        if np.all(o0 > self.gap_obstacle):
            result.note = "장애물이 없는데도 차로 중심 주행이 막힙니다(맵/설정 확인)."
        blocked_idx = np.flatnonzero(~free0)
        for side in ("left", "right"):
            result.sides[side] = self._search_side(case, side, obstacles, free0, blocked_idx)
        return result

    def _search_side(self, case, side, obstacles, free0, blocked_idx) -> SideResult:
        st = self._case_cache(case)["stations"]
        n = len(st)
        p0 = np.r_[0, np.cumsum(~free0)]
        cp_idx = int(round((self.scene.checkpoint_s - case.s_start) / self.pose_step))
        cp_idx = min(max(cp_idx, 0), n - 1)
        first_block, last_block = int(blocked_idx[0]), int(blocked_idx[-1])
        stage = {"hold": False, "out": False, "ret": False}

        for offset in self.offsets[side]:
            xh, yh, thh, eh, lh = self._hold_map(case, offset)
            oh = self._obstacle_gap(xh, yh, thh, obstacles)
            free_d = self._pose_ok(eh, lh, oh)
            # 막힌 구간 어딘가에서 옆으로 비킨 채 지나갈 수 있어야 의미가 있다.
            if not free_d[first_block:last_block + 1].any():
                continue
            stage["hold"] = True
            pd = np.r_[0, np.cumsum(~free_d)]

            outs, rets = [], []
            for length in self._lengths_for(case, offset):
                for kind, bucket in (("out", outs), ("ret", rets)):
                    seg = self._segments(case, offset, length, kind)
                    if seg is None or len(seg.start_idx) == 0:
                        continue
                    a = seg.start_idx
                    if kind == "out":
                        keep = (p0[a + 1] - p0[0] == 0) & (a <= first_block)
                    else:
                        f = a + seg.steps
                        keep = (p0[n] - p0[f] == 0) & (f >= last_block)
                    if not keep.any():
                        continue
                    gap = self._obstacle_gap(seg.x[keep], seg.y[keep], seg.th[keep], obstacles)
                    good = np.all(gap > self.gap_obstacle, axis=1)
                    if not good.any():
                        continue
                    idx = np.flatnonzero(keep)[good]
                    bucket.append((a[idx], seg.steps, length, gap[good].min(axis=1), seg.lateral))
            if outs:
                stage["out"] = True
            if rets:
                stage["ret"] = True
            if not outs or not rets:
                continue

            best = self._connect(outs, rets, pd, cp_idx, offset)
            if best is None:
                continue
            a, n1, x1, b, n2, x2 = best
            man = Maneuver(offset=offset, s1=float(st[a]), x1=x1, s2=float(st[b]), x2=x2)
            return SideResult(side=side, feasible=True, maneuver=man,
                              path=self._samples_for(case, man, obstacles))

        if not stage["hold"]:
            reason = "이쪽으로는 장애물 옆을 지나갈 공간이 없습니다(연석·실선·다른 장애물)."
        elif not stage["out"]:
            reason = "옆으로 비킬 거리가 부족합니다(출발점·실선에서 장애물까지 너무 가까움)."
        elif not stage["ret"]:
            reason = "장애물을 지난 뒤 차로로 돌아올 거리가 부족합니다(노란 실선·흰 실선)."
        else:
            constraint = "체크포인트 4 또는 중간 막힘" if self.scene.checkpoint_radius > 0 else "복귀 조건 또는 중간 막힘"
            reason = f"탐색한 후보에서 비키는 경로와 돌아오는 경로가 이어지지 않습니다({constraint})."
        return SideResult(side=side, feasible=False, reason=reason)

    def _connect(self, outs, rets, pd, cp_idx, offset):
        a = np.concatenate([o[0] for o in outs])
        n1 = np.concatenate([np.full(len(o[0]), o[1]) for o in outs])
        x1 = np.concatenate([np.full(len(o[0]), o[2]) for o in outs])
        g1 = np.concatenate([o[3] for o in outs])
        lat1 = [o[4] for o in outs for _ in range(len(o[0]))]
        b = np.concatenate([r[0] for r in rets])
        n2 = np.concatenate([np.full(len(r[0]), r[1]) for r in rets])
        x2 = np.concatenate([np.full(len(r[0]), r[2]) for r in rets])
        g2 = np.concatenate([r[3] for r in rets])
        lat2 = [r[4] for r in rets for _ in range(len(r[0]))]

        e = a + n1
        f = b + n2
        E, B = e[:, None], b[None, :]
        ok = E <= B
        ok &= (pd[np.minimum(B + 1, len(pd) - 1)] - pd[np.minimum(E, len(pd) - 1)]) == 0
        if not ok.any():
            return None
        # 체크포인트 지점의 옆 거리
        d_out = np.array([lat1[i][cp_idx - a[i]] if a[i] <= cp_idx <= e[i] else np.nan
                          for i in range(len(a))])
        d_ret = np.array([lat2[j][cp_idx - b[j]] if b[j] <= cp_idx <= f[j] else np.nan
                          for j in range(len(b))])
        d_cp = np.zeros(ok.shape)
        in_hold = (E < cp_idx) & (cp_idx < B)
        d_cp = np.where(in_hold, offset, d_cp)
        d_cp = np.where(~np.isnan(d_ret)[None, :], d_ret[None, :], d_cp)
        d_cp = np.where(~np.isnan(d_out)[:, None], d_out[:, None], d_cp)
        ok &= np.abs(d_cp) <= self.cp_tol
        if not ok.any():
            return None
        clearance = np.minimum(np.minimum(g1[:, None], g2[None, :]), 3.0)
        hold_len = (B - E) * self.pose_step
        cost = -clearance + 0.02 * hold_len
        cost = np.where(ok, cost, np.inf)
        i, j = np.unravel_index(int(np.argmin(cost)), cost.shape)
        return int(a[i]), int(n1[i]), float(x1[i]), int(b[j]), int(n2[j]), float(x2[j])

    def _samples_for(self, case: SpeedCase, man: Maneuver | None, obstacles) -> PathSamples:
        st = self._case_cache(case)["stations"]
        if man is None:
            d = dp = dpp = np.zeros_like(st)
        else:
            d, dp, dpp = man.lateral(st)
        x, y, th, kappa, _ = self._poses(st, d, dp, dpp)
        edge, line = self._map_gaps(x, y, th)
        obs = self._obstacle_gap(x, y, th, obstacles)
        return PathSamples(s=st, d=d, x=x, y=y, heading=th, kappa=kappa,
                           speed=case.speed(st), obstacle_gap=obs, edge_gap=edge,
                           line_gap=line)
