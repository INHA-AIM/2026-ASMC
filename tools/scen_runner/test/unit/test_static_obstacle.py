#!/usr/bin/env python3
"""정적 장애물 구간 판정 회귀 테스트 (실제 HD맵·전역경로 사용, 시뮬레이터 불필요)."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

TOOL_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TOOL_ROOT))

from scen_runner.planning import placement as pl  # noqa: E402
from scen_runner.config import load_catalog, load_config, merged, section_config  # noqa: E402
from scen_runner.planning.feasibility import AvoidanceChecker  # noqa: E402
from scen_runner.planning.map_scene import load_scene  # noqa: E402
from scen_runner.planning.vehicle import Obstacle  # noqa: E402


class StaticObstacleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = load_config()
        cls.scene = load_scene(cls.cfg)
        cls.checker = AvoidanceChecker(cls.scene, cls.cfg)
        cls.barricade = pl.catalog_sizes(load_catalog())["barricade_measured"]

    def at(self, ahead, d, yaw=0.0):
        return pl.obstacle_at(self.checker, self.barricade, self.scene.stopline_s + ahead, d, yaw)

    def test_zone_matches_competition_metadata(self):
        self.assertAlmostEqual(self.scene.stopline_s, 315.906, places=2)
        self.assertAlmostEqual(self.scene.checkpoint_s, 346.742, places=2)
        x, y = self.scene.route.to_xy(self.scene.checkpoint_s, 0.0)
        self.assertLess(math.hypot(x - self.scene.checkpoint_xy[0], y - self.scene.checkpoint_xy[1]), 0.1)

    def test_lane_lines_are_classified(self):
        kinds = {m.marking_id: m.forbidden for m in self.scene.markings}
        self.assertTrue(kinds["a213_yellow_after_physical_gap"])
        self.assertTrue(kinds["a213_white"])
        self.assertTrue(kinds["a220_yellow"])
        self.assertFalse(kinds["a222_centerline_guide"])  # 교차로 흰 점선은 넘어도 됨

    def test_empty_zone_is_clear_in_every_case(self):
        for res in self.checker.evaluate([]):
            self.assertFalse(res.blocked, res.case.label)
            self.assertTrue(res.feasible, res.case.label)
            self.assertGreater(res.straight.line_gap.min(), 0.0)

    def test_sample_scenario_obstacle_is_avoidable_both_sides(self):
        # 주최 샘플 시나리오 장애물: 정지선 +13.2 m, 왼쪽 0.75 m
        sample = Obstacle(x=-60.61, y=-142.18, yaw=math.radians(-83.2),
                          length=self.barricade.length, width=self.barricade.width, height=1.0)
        for res in self.checker.evaluate([sample]):
            self.assertTrue(res.blocked)
            self.assertTrue(res.sides["left"].feasible, res.case.label)
            self.assertTrue(res.sides["right"].feasible, res.case.label)
            for side in res.sides.values():
                path = side.path
                self.assertGreater(path.obstacle_gap.min(), self.checker.gap_obstacle)
                self.assertGreater(path.line_gap.min(), self.checker.gap_line)
                self.assertLessEqual(abs(path.kappa).max(), self.checker.kmax + 1e-6)

    def test_obstacle_right_after_stopline_is_not_avoidable_from_stop(self):
        res = {r.case.case_id: r for r in self.checker.evaluate([self.at(3.0, 0.0)])}
        self.assertFalse(res["stop_start"].feasible)

    def test_obstacle_at_zone_end_blocks_return(self):
        res = self.checker.evaluate([self.at(29.0, 0.0)])
        self.assertTrue(all(not r.feasible for r in res))

    def test_random_layout_is_reproducible(self):
        s0 = self.scene.stopline_s
        req = pl.RandomRequest(sizes=[self.barricade], count=2, s_range=(s0 + 2, s0 + 28),
                               d_range=(-2.5, 2.5), yaw_range_deg=(-10, 10), min_gap=1.0,
                               require_lane_block=True, require_all_cases=True,
                               max_tries=200, seed=7)
        a = pl.random_layout(self.checker, req)
        b = pl.random_layout(self.checker, req)
        self.assertEqual(len(a.obstacles), 2)
        self.assertEqual([p["s"] for p in a.picks], [p["s"] for p in b.picks])
        self.assertTrue(all(r.feasible for r in self.checker.evaluate(a.obstacles)))


class ApproachLinkTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base = load_config()
        cls.cfg = section_config(cls.base, "A2256W000213")
        cls.scene = load_scene(cls.cfg)
        cls.checker = AvoidanceChecker(cls.scene, cls.cfg)

    def test_link_midpoint_is_used_for_both_speed_cases(self):
        self.assertAlmostEqual(self.scene.zone_s[0], (237.42279 + 315.906098) / 2, places=3)
        self.assertAlmostEqual(self.scene.zone_s[1], 315.906098, places=3)
        for case in self.checker.cases:
            self.assertAlmostEqual(case.s_start, self.scene.zone_s[0])
        self.assertEqual(self.scene.checkpoint_radius, 0.0)
        self.assertEqual(self.scene.reference_label, "구간 시작")
        self.assertAlmostEqual(self.checker.s_goal, self.scene.zone_s[1])
        self.assertNotIn("start_station_m", self.base["speed_cases"][0])

    def test_empty_approach_link_is_drivable(self):
        self.assertTrue(all(r.feasible and not r.blocked for r in self.checker.evaluate([])))

    def test_box_blocking_narrow_lane_cannot_cross_solid_lines(self):
        size = pl.ModelSize("test_box", 0.6, 0.6, 0.6)
        ob = pl.obstacle_at(self.checker, size, 300.0, -0.6)
        for result in self.checker.evaluate([ob]):
            self.assertTrue(result.blocked)
            self.assertFalse(result.feasible)

    def test_intersection_obstacle_does_not_block_approach_only(self):
        size = pl.ModelSize("test_box", 0.6, 0.6, 0.6)
        ob = pl.obstacle_at(self.checker, size, self.scene.zone_s[1] + 13.2, 0.0)
        self.assertTrue(all(r.feasible and not r.blocked for r in self.checker.evaluate([ob])))

    def test_live_small_obstacle_requires_offset_between_coarse_samples(self):
        # MORAI에서 읽은 A213의 소형 장애물. 0.25 m는 간격 부족,
        # 0.50 m는 경계 침범이지만 0.30 m 왼쪽 회피는 가능하다.
        ob = Obstacle(x=-68.3324508667, y=-208.9381866455,
                      yaw=math.radians(-88.6529541016),
                      length=0.3663700223, width=0.3663699925, height=0.8589373231)
        # 이전 실측 배치는 현재 후반부 검증 범위 앞에 있으므로,
        # 이 회귀 검사만 원래의 전체 링크 범위를 사용한다.
        full_cfg = section_config(self.base, "A2256W000213")
        full_cfg["zone"]["evaluation_s_range"][0] = 237.42279
        full_cfg["zone"]["view_s_range"][0] = 229.42279
        for case in full_cfg["speed_cases"]:
            case["start_station_m"] = 237.42279
        full_scene = load_scene(full_cfg)
        fine = AvoidanceChecker(full_scene, full_cfg)
        coarse_cfg = merged(full_cfg, {})
        coarse_cfg["search"]["lateral_offsets_m"]["fine"]["step"] = 0.25
        coarse = AvoidanceChecker(full_scene, coarse_cfg)
        self.assertTrue(all(not r.feasible for r in coarse.evaluate([ob])))
        for result in fine.evaluate([ob]):
            self.assertTrue(result.blocked)
            self.assertTrue(result.sides["left"].feasible)
            self.assertFalse(result.sides["right"].feasible)
            path = result.sides["left"].path
            self.assertGreater(path.obstacle_gap.min(), self.checker.gap_obstacle)
            self.assertGreater(path.line_gap.min(), self.checker.gap_line)
            self.assertGreater(path.edge_gap.min(), self.checker.gap_edge)
            self.assertLessEqual(abs(path.kappa).max(), self.checker.kmax + 1e-9)
            self.assertAlmostEqual(path.d[-1], 0.0)

    def test_obstacle_in_first_half_is_outside_evaluation(self):
        size = pl.ModelSize("test_box", 0.6, 0.6, 0.6)
        ob = pl.obstacle_at(self.checker, size, 250.0, 0.0)
        self.assertTrue(all(r.feasible and not r.blocked for r in self.checker.evaluate([ob])))


if __name__ == "__main__":
    unittest.main()
