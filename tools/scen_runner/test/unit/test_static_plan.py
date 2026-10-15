"""통합 정적 배치의 방해/회피 계약과 재현성 검증."""
import copy
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scen_runner.config import load_config, load_catalog, load_yaml
from scen_runner.planning.static_plan import (POLICY_PATH, runtime_config, generate_scene,
                                   validate_scene, blocks_straight, placement_reason, candidate_models, overlaps_intersection_region, SceneCancelled)
from scen_runner.planning.placement import obstacle_at
from scen_runner.morai.morai_link import MoraiLink


class OneSceneTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.policy = load_yaml(POLICY_PATH)
        cls.base = load_config()
        cls.cfg = runtime_config(cls.base, cls.policy)
        cls.catalog = load_catalog()
        cls.objects, cls.summary, cls.checkers = generate_scene(
            cls.cfg, cls.catalog, cls.policy, 42, progress=lambda *_: None)

    def test_each_object_blocks_straight_but_both_speeds_can_avoid(self):
        self.assertEqual(len(self.objects), 2)
        for ob in self.objects:
            checker = self.checkers[ob.extra["section"]]
            self.assertTrue(blocks_straight(checker, ob))
            for case in self.summary[ob.extra["section"]]:
                self.assertTrue(case["blocked"] and case["feasible"])
                self.assertTrue(case["sides"])
                for side in case["sides"]:
                    self.assertGreater(side["obstacle_gap_m"], .5)
                    self.assertGreater(side["line_gap_m"], 0)
                    self.assertGreater(side["edge_gap_m"], .1)

    def test_seed_reproduces_models_and_positions(self):
        objects, _, _ = generate_scene(self.cfg, self.catalog, self.policy, 42, progress=lambda *_: None)
        self.assertEqual([o.to_dict() for o in self.objects], [o.to_dict() for o in objects])

    def test_both_regions_apply_only_requested_exclusions(self):
        excluded = {"Road_Sign_Mark", "Speed_Bump", "fr-streetlight-2", "hole-cover", "EFFECT_Fire01"}
        excluded.update(f"Tree{i:02d}" for i in range(1, 10))
        excluded.update({"OBJ_Hyundai_Universe_High", "OBJ_Hyundai_Xcient_Mixer",
                         "OBJ_Hyundai_Xcient_P520", "OBJ_Hyundai_Xcient_P540"})
        self.assertEqual(set(self.policy["excluded_models"]), excluded)
        for region in ("approach", "intersection"):
            candidates = candidate_models(self.catalog, self.policy[region], excluded)
            self.assertEqual(len(candidates), 52)
            self.assertEqual({m.morai_model for m in candidates}, set(self.catalog["available_models"]) - excluded)

    def test_nonblocking_or_outside_band_is_rejected(self):
        approach = self.checkers["A2256W000213"]
        own = next(o for o in self.objects if o.extra["section"] == "A2256W000213")
        nonblocking = copy.deepcopy(own)
        s, _ = approach.scene.route.project(own.x, own.y)
        nonblocking.x, nonblocking.y = map(float, approach.scene.route.to_xy(s, -4.0))
        self.assertFalse(blocks_straight(approach, nonblocking))
        self.assertTrue(placement_reason("A2256W000213", approach, nonblocking, [], self.policy))
        objects = [nonblocking if o is own else o for o in self.objects]
        with self.assertRaises(ValueError):
            validate_scene(self.checkers, objects, [], self.policy)


    def test_center_outside_region_but_footprint_overlaps_is_accepted(self):
        checker = self.checkers["intersection"]
        from scen_runner.planning.placement import ModelSize
        size = ModelSize("test", 1.0, 1.0, 1.0)
        lo, _ = self.policy["intersection"]["s_from_stopline_m"]
        edge = checker.scene.stopline_s + lo
        partial = obstacle_at(checker, size, edge - .25, 0)
        outside = obstacle_at(checker, size, edge - 2, 0)
        self.assertTrue(overlaps_intersection_region(checker, partial, self.policy))
        self.assertFalse(overlaps_intersection_region(checker, outside, self.policy))
        self.assertEqual(placement_reason("intersection", checker, partial, [], self.policy), "")

    def test_exhausted_search_retries_until_cancelled(self):
        policy = copy.deepcopy(self.policy)
        policy["max_tries_per_region"] = 1
        logs = []
        def cancelled(): return sum("배치 탐색" in line for line in logs) >= 3
        with patch("scen_runner.planning.static_plan.placement_reason", return_value="reject candidate"):
            with self.assertRaises(SceneCancelled):
                generate_scene(self.cfg, self.catalog, policy, 42, progress=logs.append,
                               cancelled=cancelled, checkers=self.checkers)
        self.assertEqual(sum("배치 탐색" in line for line in logs), 3)

    def test_two_intersection_objects_are_jointly_avoidable(self):
        objects, summary, _ = generate_scene(self.cfg, self.catalog, self.policy, 7,
                                              intersection_count=2, checkers=self.checkers,
                                              progress=lambda *_: None)
        self.assertEqual(len(objects), 3)
        self.assertEqual(sum(o.extra["section"] == "intersection" for o in objects), 2)
        self.assertTrue(all(c["blocked"] and c["feasible"] for cases in summary.values() for c in cases))


    def test_live_list_requests_aliases_matching_spawn_and_destroy_ids(self):
        link = MoraiLink(self.base)
        link.client_key = "scen_runner_one_scene"
        def get_states(request, **kwargs):
            self.assertEqual(request.client_key, link.client_key)
            return SimpleNamespace(states=[])
        link._sa = SimpleNamespace(_actor_stub=SimpleNamespace(GetAllActorsState=get_states))
        self.assertEqual(link.read_actors(), (None, []))



if __name__ == "__main__":
    unittest.main()
