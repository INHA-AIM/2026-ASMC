"""Finish-line passes, side offsets, overshoot, reverse motion and teleport exclusion."""
from pathlib import Path
import math
import sys
from types import SimpleNamespace as S
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.planning.motion_curve import Curve
from scen_runner.runtime.section_completion import SectionCompletion


def ego(x,y=0.,speed=25.):
    return S(transform=S(location=S(x=x,y=y)),velocity=S(x=speed,y=0.,z=0.))


class SectionCompletionTest(unittest.TestCase):
    def tracker(self,width=10.):
        curve=Curve([[0.,0.,0.],[100.,0.,0.]])
        tracker=SectionCompletion(dict(curve=curve,end_xy=(100.,0.),end_heading=0.),half_width_m=width)
        tracker.reset((0.,0.),0.)
        return tracker

    def test_lateral_offset_pass_outside_old_three_metre_circle(self):
        tracker=self.tracker()
        self.assertFalse(tracker.update(ego(90.,5.),10.))
        self.assertTrue(tracker.update(ego(108.,5.),12.))
        self.assertFalse(tracker.update(ego(110.,5.),13.))

    def test_overshoot_between_two_outside_samples_is_detected(self):
        tracker=self.tracker()
        tracker.reset((90.,0.),0.)
        self.assertTrue(tracker.update(ego(112.,speed=80.),1.))

    def test_original_close_arrival_before_line_remains_supported(self):
        tracker=self.tracker()
        tracker.update(ego(95.),10.)
        self.assertTrue(tracker.update(ego(98.),11.))

    def test_reverse_pass_does_not_finish(self):
        tracker=self.tracker()
        tracker.reset((108.,5.),0.)
        self.assertFalse(tracker.update(ego(90.,5.),2.))

    def test_crossing_too_far_from_road_does_not_finish(self):
        tracker=self.tracker()
        tracker.reset((90.,20.),0.)
        self.assertFalse(tracker.update(ego(108.,20.),2.))

    def test_width_can_be_configured(self):
        tracker=self.tracker(width=4.)
        tracker.reset((90.,5.),0.)
        self.assertFalse(tracker.update(ego(108.,5.),2.))

    def test_teleport_to_end_or_beyond_then_staying_is_not_a_pass(self):
        for x in (100.,108.):
            with self.subTest(x=x):
                tracker=self.tracker()
                self.assertFalse(tracker.update(ego(x,speed=0.),.2))
                self.assertFalse(tracker.update(ego(x,speed=0.),1.))
                self.assertFalse(tracker.completed)

    def test_early_nearby_route_branch_cannot_jump_to_finish_station(self):
        curve=Curve([[0.,0.,0.],[10.,0.,0.],[10.,20.,0.],[0.,20.,0.],[0.,1.,0.],[10.,1.,0.]])
        tracker=SectionCompletion(dict(curve=curve,end_xy=(10.,1.),end_heading=0.),approach_m=25.)
        tracker.reset((5.,0.),0.)
        self.assertFalse(tracker.update(ego(11.,1.),.5))
        self.assertFalse(tracker.update(ego(12.,1.),1.))

    def test_reset_clears_previous_finish(self):
        tracker=self.tracker()
        tracker.reset((90.,5.),0.)
        self.assertTrue(tracker.update(ego(108.,5.),2.))
        tracker.reset((0.,0.),3.)
        self.assertFalse(tracker.completed)
        self.assertFalse(tracker.update(ego(1.),3.2))

    def test_invalid_positions_do_not_count(self):
        tracker=self.tracker()
        self.assertFalse(tracker.update(ego(math.nan),1.))
        self.assertFalse(tracker.update(ego(100.),2.))

    def test_all_actual_sections_accept_five_metre_offset_and_overshoot(self):
        from scen_runner.runtime.validation_episode import EpisodeRoute
        route=EpisodeRoute()
        for name in route.cfg['sections']:
            with self.subTest(section=name):
                definition=route.section(name)
                heading=definition['end_heading'];x,y=definition['end_xy']
                tangent=(math.cos(heading),math.sin(heading));normal=(-tangent[1],tangent[0])
                before=(x-8*tangent[0]+5*normal[0],y-8*tangent[1]+5*normal[1])
                after=(x+8*tangent[0]+5*normal[0],y+8*tangent[1]+5*normal[1])
                tracker=route.prepare_section_end(name);tracker.reset(before,0.)
                self.assertTrue(route.finished(name,ego(*after,speed=80.),.8))


if __name__=='__main__':unittest.main()
