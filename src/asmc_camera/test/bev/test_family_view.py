import unittest
from types import SimpleNamespace as NS
import numpy as np
from asmc_camera.bev.family_view import family_segments
from asmc_camera.bev.grid import BevGridSpec
from asmc_camera.bev.visualization import draw_metric_grid, draw_stateful_lane_family


class FamilyViewTest(unittest.TestCase):
    def family(self, states):
        return NS(valid=True,boundaries=[NS(role=1,color=1,support_state=states,
            polyline=[NS(x=i,y=1) for i in range(len(states))])])

    def test_published_states_are_preserved_and_invalid_intervals_are_not_observed(self):
        segments = family_segments(self.family([1,1,0,0,2,2,1,1]))
        self.assertEqual([s[1] for s in segments],
            ['observed','invalid','structure_inferred','observed'])
        self.assertEqual([s[3][:,0].tolist() for s in segments],[[0,1],[2,3],[4,5],[6,7]])

    def test_invalid_family_is_empty_and_malformed_support_is_rejected(self):
        message = self.family([1,1]); message.valid = False
        self.assertEqual(family_segments(message),[])
        message.valid=True; message.boundaries[0].support_state=[1]
        with self.assertRaises(ValueError): family_segments(message)
        with self.assertRaises(ValueError): family_segments(self.family([1,9]))

    def test_family_contract_can_render_without_a_semantic_bev_or_second_fit(self):
        grid=BevGridSpec(-2,10,-4,4,.1)
        canvas=np.zeros((*grid.shape,3),np.uint8)
        draw_metric_grid(canvas,grid)
        self.assertIs(draw_stateful_lane_family(canvas,family_segments(self.family([1]*5)),grid),canvas)
        self.assertTrue(canvas.any())


if __name__=='__main__': unittest.main()
