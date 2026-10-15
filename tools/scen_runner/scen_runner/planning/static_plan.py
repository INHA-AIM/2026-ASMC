"""직진을 방해하되 회피 가능한 객체를 접근도로 1개, 교차로 1~2개 배치한다.

검증은 GUI의 운동학/동역학 제한 모델이다. 실제 MPC 주행 성공 판정은 아니다.
잉여도로 배치 띠와 차량 주행가능영역은 별개다.
"""
from __future__ import annotations

import math
import random

import numpy as np
from matplotlib.path import Path as MplPath

from scen_runner.config import TOOL_ROOT, merged, section_config
from scen_runner.planning.feasibility import AvoidanceChecker
from scen_runner.planning.map_scene import reusable_scene as load_scene
from scen_runner.planning.placement import ModelSize, basic_check, box_gap, obstacle_at, _edge_points

POLICY_PATH = TOOL_ROOT / "config" / "one_scene.yaml"


class SceneCancelled(RuntimeError):
    pass


def overlaps_intersection_region(checker, ob, policy):
    """실제 XY 후보 띠와 객체 사각형의 겹침/경계 접촉을 검사한다."""
    region = policy["intersection"]
    lo, hi = region["s_from_stopline_m"]
    d0, d1 = region["d_range_m"]
    stations = np.linspace(checker.scene.stopline_s + lo, checker.scene.stopline_s + hi, 100)
    x0, y0 = checker.scene.route.to_xy(stations, d0)
    x1, y1 = checker.scene.route.to_xy(stations[::-1], d1)
    band = np.c_[np.r_[x0, x1], np.r_[y0, y1]]
    box = ob.corners()
    if MplPath(band).contains_points(box).any() or MplPath(box).contains_points(band).any():
        return True
    a, b = band[:, None, :], np.roll(band, -1, axis=0)[:, None, :]
    c, d = box[None, :, :], np.roll(box, -1, axis=0)[None, :, :]
    def cross(u, v):
        return u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]
    ab, cd = b - a, d - c
    eps = 1e-9
    crossing = (cross(ab, c-a) * cross(ab, d-a) <= eps) & (cross(cd, a-c) * cross(cd, b-c) <= eps)
    bbox = np.all(np.maximum(np.minimum(a, b), np.minimum(c, d)) <=
                  np.minimum(np.maximum(a, b), np.maximum(c, d)) + eps, axis=-1)
    return bool(np.any(crossing & bbox))


def runtime_config(base, policy):
    cfg = merged(base, {})
    handoff = float(policy["phases"]["handoff_before_stopline_m"])
    cfg["single_scene_runtime"] = {"handoff_before_stopline_m": handoff}
    for case in cfg["speed_cases"]:
        if case["start"] == "before_stopline":
            case["start_distance_m"] = handoff
    return cfg


def white_d(scene, stations):
    samples = []
    for marking in scene.markings:
        if marking.marking_id == "a213_white":
            s, d = scene.route.project(marking.points[:, 0], marking.points[:, 1])
            samples.extend(zip(s.tolist(), d.tolist()))
    if not samples:
        raise ValueError("A213 오른쪽 흰 실선 데이터가 없습니다.")
    samples.sort()
    s, d = np.asarray(samples).T
    requested = np.asarray(stations)
    if np.any(requested < s[0]) or np.any(requested > s[-1]):
        raise ValueError("객체가 흰 실선 데이터 범위를 벗어납니다.")
    return np.interp(requested, s, d)


def placement_reason(section, checker, ob, others, policy):
    if any(box_gap(ob, other) < policy["min_object_gap_m"] for other in others):
        return "다른 객체와 간격 부족"
    if section == "intersection":
        if not overlaps_intersection_region(checker, ob, policy):
            return "객체 외형이 교차로 후보 범위와 겹치지 않음"
        return basic_check(checker, ob, others, policy["min_object_gap_m"])
    pts = _edge_points(ob)
    s, d = checker.scene.route.project(pts[:, 0], pts[:, 1])
    region = policy["approach"]
    start = checker.scene.zone_s[0]
    lo, hi = region["s_from_section_start_m"]
    if s.min() < start + lo or s.max() > start + hi:
        return "객체 외형이 앞뒤 배치 범위를 벗어남"
    line = white_d(checker.scene, s)
    if np.any(d < line - region["placement_band_outer_m"]) or np.any(d > line + region["placement_band_inner_m"]):
        return "객체 외형이 잉여도로 시험 배치 띠를 벗어남"
    center_s, center_d = checker.scene.route.project(ob.x, ob.y)
    if center_d > white_d(checker.scene, center_s) + 1e-6:
        return "객체 중심이 흰 실선 안쪽"
    return ""


def blocks_straight(checker, ob):
    # 다른 객체나 맵 오류가 아니라 새 객체 자신 때문에 회피가 필요해야 한다.
    return all(np.min(checker._samples_for(case, None, [ob]).obstacle_gap)
               <= checker.gap_obstacle for case in checker.cases)


def summarize(checker, objects):
    out = []
    for result in checker.evaluate(objects):
        sides = []
        for side in result.feasible_sides():
            p = side.path
            sides.append({"side": side.side, "offset_m": side.maneuver.offset,
                          "obstacle_gap_m": float(p.obstacle_gap.min()),
                          "line_gap_m": float(p.line_gap.min()),
                          "edge_gap_m": float(p.edge_gap.min())})
        out.append({"case": result.case.case_id, "blocked": result.blocked,
                    "feasible": result.feasible, "sides": sides})
    return out


def make_checkers(cfg):
    result = {}
    for section in ("A2256W000213", "intersection"):
        local = section_config(cfg, section)
        result[section] = AvoidanceChecker(load_scene(local), local)
    return result


def candidate_models(catalog, region, excluded=()):
    names = catalog.get("available_models", []) if region["models"] == "all_available" else region["models"]
    if not names:
        raise RuntimeError("MORAI 모델 목록이 없습니다.")
    result = []
    for name in names:
        if name in excluded:
            continue
        entry = catalog["models"].get(name, {})
        if not entry.get("measured") or not entry.get("valid_box") or not entry.get("morai_model"):
            raise RuntimeError(f"{name}: 모델 실측 데이터가 없습니다.")
        # 단일 씬 정책의 사용자 지정 제외 목록만 적용한다.
        result.append(ModelSize.from_catalog(name, entry))
    return result


def generate_scene(cfg, catalog, policy, seed, fixed=(), progress=print, cancelled=lambda: False,
                   intersection_count=None, checkers=None):
    rng = random.Random(seed)
    drawn_count = rng.choice(policy["intersection"].get("count_choices", [1, 2]))
    count = drawn_count if intersection_count is None else intersection_count
    if count not in (1, 2):
        raise ValueError("교차로 객체 수는 1 또는 2여야 합니다.")
    checkers = make_checkers(cfg) if checkers is None else checkers
    pools = {sid: candidate_models(catalog, policy[key], policy.get("excluded_models", []))
             for sid, key in (("A2256W000213", "approach"), ("intersection", "intersection"))}
    if not all(pools.values()):
        raise RuntimeError("제외 목록 적용 후 사용 가능 모델이 없습니다.")
    progress(f"객체 수: 접근도로 1개 / 교차로 {count}개")
    batch = 0
    while True:
        if cancelled():
            raise SceneCancelled("씬 생성 취소")
        batch += 1
        chosen = []
        schedule = [("A2256W000213", "approach")] + [("intersection", "intersection")] * count
        for section, key in schedule:
            checker, region = checkers[section], policy[key]
            models, order = pools[section], []
            others = list(fixed) + chosen
            for attempt in range(1, policy["max_tries_per_region"] + 1):
                if cancelled():
                    raise SceneCancelled("씬 생성 취소")
                if not order:
                    order = list(models)
                    rng.shuffle(order)
                size = order.pop()
                yaw = rng.uniform(*region["yaw_range_deg"])
                angle = math.radians(yaw)
                half_width = .5 * (size.length * abs(math.sin(angle)) + size.width * abs(math.cos(angle)))
                half_length = .5 * (size.length * abs(math.cos(angle)) + size.width * abs(math.sin(angle)))
                if section == "intersection":
                    lo, hi = region["s_from_stopline_m"]
                    d0, d1 = region["d_range_m"]
                    # 중심이 범위 밖이어도 외형이 걸칠 수 있도록 샘플 범위를 확장한다.
                    s = checker.scene.stopline_s + rng.uniform(lo - half_length, hi + half_length)
                    d = rng.uniform(d0 - half_width, d1 + half_width)
                else:
                    s = checker.scene.zone_s[0] + rng.uniform(*region["s_from_section_start_m"])
                    lo, hi = region["center_outside_white_m"]
                    lo = max(lo, half_width - region["placement_band_inner_m"])
                    hi = min(hi, region["placement_band_outer_m"] - half_width)
                    if lo > hi:
                        continue
                    d = float(white_d(checker.scene, s)) - rng.uniform(lo, hi)
                ob = obstacle_at(checker, size, s, d, yaw, source="one_scene")
                ob.extra = {"section": section, "object_type": 3}
                if placement_reason(section, checker, ob, others, policy) or not blocks_straight(checker, ob):
                    continue
                if not all(r.feasible for r in checker.evaluate(others + [ob])):
                    continue
                chosen.append(ob)
                progress(f"{section}: {size.name}, s={s:.2f}, d={d:+.2f}, 후보 {attempt}회 → 회피 가능")
                break
            else:
                progress(f"배치 탐색 {batch}차: {section} 후보 {attempt}회 소진, 전체 조합을 다시 탐색합니다.")
                break
        else:
            try:
                validation = validate_scene(checkers, chosen, fixed, policy)
            except ValueError as exc:
                progress(f"조합 재탐색: {exc}")
                continue
            return chosen, validation, checkers


def validate_scene(checkers, objects, fixed, policy):
    approach = [o for o in objects if o.extra.get("section") == "A2256W000213"]
    intersection = [o for o in objects if o.extra.get("section") == "intersection"]
    if len(approach) != 1 or len(intersection) not in (1, 2) or len(objects) != len(approach) + len(intersection):
        raise ValueError("접근도로 1개, 교차로 1~2개가 필요합니다.")
    result = {}
    for section, checker in checkers.items():
        for own in (o for o in objects if o.extra["section"] == section):
            reason = placement_reason(section, checker, own, list(fixed) + [o for o in objects if o is not own], policy)
            if reason or not blocks_straight(checker, own):
                raise ValueError(f"{section}: 주행경로 방해/배치 조건 위반: {reason}")
        summary = summarize(checker, list(fixed) + list(objects))
        if not all(r["feasible"] and r["blocked"] for r in summary):
            raise ValueError(f"{section}: 두 속도 조건의 회피 검증 실패")
        result[section] = summary
    return result
