import unittest
from scen_runner.records.local_gallery import select_cases


class LocalGalleryTest(unittest.TestCase):
    def test_rejected_family_keeps_exact_selected_source_for_visual_review(self):
        selected=select_cases([dict(plan_sequence=1,scene_stamp_ns=0,
            input_join=dict(selected_stamp_ns=123),reason='family_plane_invalid',
            valid=False,path=[])])
        self.assertEqual(selected[123][0]['plan_sequence'],1)
        self.assertNotIn(0,selected)

    def test_failure_matches_exact_plan_not_nearest_stamp(self):
        rows = [
            dict(
                plan_sequence=i,
                scene_stamp_ns=i * 100,
                reason="ok",
                valid=True,
                selected_length_m=15,
                path=[[0, 0, 0]],
            )
            for i in range(1, 100)
        ]
        rows[24]["valid"] = False
        rows[24]["reason"] = "insufficient_stopping_space"
        rows[24]["selected_length_m"] = 0.3
        selected = select_cases(
            rows, [dict(plan_sequence=25, reason="local_source_stale")]
        )
        assert selected[2500][0]["plan_sequence"] == 25
        assert selected[2500][1] == "제어: local_source_stale"
        assert len(selected) <= 24
        assert 0 not in selected


class LocalPathStabilityTest(unittest.TestCase):
    def test_common_position_removes_ego_translation_and_yaw(self):
        from scen_runner.records.analyze_local_path_stability import align, lateral
        import math
        old = (2, 3, math.pi / 2)
        current = (2, 4, math.pi / 2)
        path = [[0, 1], [10, 1]]
        moved = align(path, old, current)
        self.assertAlmostEqual(moved[0][0], -1)
        self.assertAlmostEqual(lateral(moved, 5), 1)
        self.assertIsNone(lateral(moved, 10))

    def test_nonmonotonic_path_is_not_bisected_as_forward_path(self):
        from scen_runner.records.analyze_local_path_stability import lateral
        self.assertIsNone(lateral([[0, 0], [5, 1], [4, 2], [10, 3]], 3))
