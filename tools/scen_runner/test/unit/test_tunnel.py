import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock,patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.morai.episode_link import SceneLink
from scen_runner.planning.tunnel_plan import TunnelPlan,tunnel_static_models,stopped_path_gap,ego_box,static_lateral_bounds
from scen_runner.planning.motion_curve import Curve
from scen_runner.planning.vehicle import VehicleSpec
from scen_runner.config import load_config


class TunnelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.combined=TunnelPlan(42,'both');cls.combined.prepare()

    def test_scene_catalogue_direction_and_seed(self):
        models=tunnel_static_models()
        self.assertTrue(models)
        self.assertFalse(any(m.name.startswith('OBJ_Hyundai_Xcient') for m in models))
        p=TunnelPlan(123,'dynamic');p.prepare();q=TunnelPlan(123,'dynamic');q.prepare()
        self.assertEqual(p.cfg['ego_links'],['A2256W000448','A2256W000126','A2256W000128'])
        self.assertEqual(p.dynamic.start,q.dynamic.start)
        self.assertEqual(p.dynamic.cfg,q.dynamic.cfg)

    def test_crossing_braking_and_static_path_gap(self):
        p=self.combined;d=p.dynamic;t=d.timing
        self.assertIn(d.models[0][1],['Man1','Woman1','Boy1','Girl1','Deer1','Dog1','Ped_Pet1'])
        self.assertIn(d.cfg['walk_speed_kmh'],[15,16,17,18])
        self.assertTrue(t['stop_path_safe'])
        self.assertGreaterEqual(t['stopped_path_gap_m'],t['npc_radius_m']+.3+1.)
        self.assertGreaterEqual(t['braking_path_gap_m'],t['npc_radius_m']+.3)
        self.assertGreaterEqual(t['front_gap_m'],t['required_gap_m'])
        points=np.linspace(d.start[:2],d.end[:2],100)
        gap=p.static.signed_distance(points[:,0],points[:,1]).min()
        self.assertGreaterEqual(gap,t['npc_radius_m']+p.cfg['static_dynamic_gap_m'])
        self.assertTrue(p.static_avoidance)

    def test_tunnel_allows_lane_marking_crossing_but_preserves_road_and_obstacle_checks(self):
        p=self.combined;checker=p._static_context[1]
        self.assertTrue(p.cfg['allow_lane_marking_crossing'])
        self.assertEqual(checker.cfg['forbidden_lines'],[])
        self.assertTrue(np.isinf(p.scene.grid.line_dist).all())
        self.assertTrue((~p.scene.grid.drivable).any())
        self.assertFalse(checker._pose_ok(np.array([0.]),np.array([np.inf]),np.array([100.]))[0])
        self.assertFalse(checker._pose_ok(np.array([100.]),np.array([np.inf]),np.array([0.]))[0])
        self.assertTrue(load_config()['forbidden_lines'])  # Other static sections retain their rules.

    def test_horizontal_and_forward_diagonal_are_generated_and_backwards_is_excluded(self):
        patterns=set()
        for seed in range(8):
            p=TunnelPlan(seed,'dynamic');p.prepare();d=p.dynamic
            patterns.add(d.timing['cross_pattern'])
            _,_,_,h=p.curve.pose(d.timing['cross_station_m'])
            direction=np.array(d.end[:2])-d.start[:2];direction/=np.linalg.norm(direction)
            forward=float(np.dot(direction,[np.cos(h),np.sin(h)]))
            self.assertGreaterEqual(forward,-1e-6)
            self.assertNotIn('후방',d.direction_label)
            if d.timing['cross_pattern']==0:self.assertAlmostEqual(forward,0.,places=5)
        self.assertEqual(patterns,{0,1})

    def test_whole_npc_path_crossing_stopped_car_is_rejected_by_geometry(self):
        p=self.combined;v=VehicleSpec.from_config(load_config())
        box=ego_box(p.curve,p.prefix+50,v)
        # Endpoints alone are safe but the line runs straight through the car.
        direction=np.array([np.cos(box.yaw),np.sin(box.yaw)])
        a=np.array([box.x,box.y])-direction*15;b=np.array([box.x,box.y])+direction*15
        self.assertGreater(float(box.signed_distance(*a)),1.)
        self.assertGreater(float(box.signed_distance(*b)),1.)
        self.assertLess(stopped_path_gap(p.curve,p.prefix+50,v,a,b),0.)

    def test_explicit_horizontal_selection_survives_candidate_search(self):
        p=TunnelPlan(0,'dynamic','horizontal');p.prepare()
        self.assertEqual(p.dynamic.timing['cross_pattern'],0)
        self.assertIn('가로',p.dynamic.direction_label)




    def test_static_id_lookup_waits_for_list_cache_instead_of_querying_missing_actor(self):
        link=SceneLink(load_config());stub=Mock();link._sa=S(_actor_stub=stub)
        stub.GetAllActorsState.return_value=S(states=[])
        self.assertIsNone(link.static_native_id('alias'))
        stub.GetActorState.assert_not_called()
        pose=S(x=1.,y=2.,z=3.)
        scoped=S(actor_info=S(id=S(value='alias'),object_type=3),transform=S(location=pose))
        native=S(actor_info=S(id=S(value='7'),object_type=3),transform=S(location=pose))
        stub.GetAllActorsState.side_effect=[S(states=[scoped]),S(states=[native])]
        self.assertEqual(link.static_native_id('alias'),'7')

    def test_candidate_exhaustion_yields_and_searches_another_position(self):
        p=TunnelPlan(42,'static');p.cfg.update(static_max_tries=2,static_tick_candidates=2)
        checker=Mock();checker.vehicle=VehicleSpec.from_config(load_config())
        result=S(blocked=True,feasible=True,start_blocked=False,feasible_sides=lambda:[])
        checker.evaluate.return_value=[result]
        context=(1877.744799,checker,[self.combined.static_size]);p._static_context=context
        with patch('scen_runner.planning.tunnel_plan.obstacle_at',return_value=self.combined.static) as place, \
             patch('scen_runner.planning.tunnel_plan.basic_check',side_effect=['outside','outside','']), \
             patch('scen_runner.planning.tunnel_plan.blocks_straight',return_value=True), \
             patch('scen_runner.planning.tunnel_plan.load_scene') as loader:
            self.assertFalse(p.prepare(incremental=True))
            self.assertFalse(p.prepared);self.assertEqual(p.search_rounds,1)
            self.assertIn('재선택',p.search_status)
            self.assertTrue(p.prepare(incremental=True))
            loader.assert_not_called()
        self.assertIs(p._static_context,context)
        self.assertEqual(p.search_attempts,3)
        self.assertEqual(len({call.args[2] for call in place.call_args_list}),3)

    def test_both_scene_reselects_crossing_before_spawning_when_batch_fails(self):
        p=TunnelPlan(42,'both');old=self.combined.dynamic;p.dynamic=old
        new=Mock();new.timing=old.timing
        def search(cancelled,limit):
            if p.search_rounds==0:p._round_attempts=p.cfg['static_max_tries'];return False
            return True
        with patch.object(p,'prepare_static',side_effect=search), \
             patch('scen_runner.planning.tunnel_plan.stopping_crossing',return_value=new) as crossing:
            self.assertFalse(p.prepare(incremental=True))
            self.assertIsNone(p.dynamic);self.assertEqual(p.scene_type,'both')
            self.assertTrue(p.prepare(incremental=True))
            crossing.assert_called_once()
        self.assertIs(p.dynamic,new)


    def test_combined_station_sampling_uses_entire_tunnel_without_after_cross_restriction(self):
        p=TunnelPlan(42,'both')
        p.dynamic=S(timing={'cross_station_m':p.prefix+74.9,'npc_radius_m':.5},start=(0.,0.,0.),end=(1.,1.,0.))
        checker=Mock();checker.vehicle=VehicleSpec.from_config(load_config())
        checker.evaluate.return_value=[S(blocked=True,feasible=True,start_blocked=False,feasible_sides=lambda:[])]
        p._static_context=(1877.,checker,[self.combined.static_size])
        ob=Mock();ob.signed_distance.return_value=np.full(100,100.)
        with patch('scen_runner.planning.tunnel_plan.obstacle_at',return_value=ob) as place, \
             patch('scen_runner.planning.tunnel_plan.basic_check',return_value=''),patch('scen_runner.planning.tunnel_plan.blocks_straight',return_value=True):
            self.assertTrue(p.prepare_static(lambda:False,1))
        station=place.call_args.args[2]-1877.
        self.assertGreaterEqual(station,0.)
        self.assertLessEqual(station,p.cfg['static_station_m'][1])
        self.assertLess(station,94.9)  # Previously clipped to 94.9..95 for this crossing.

    def test_full_link_extent_is_resolved_from_map_for_both_scene_types(self):
        for kind in ['static','both']:
            p=TunnelPlan(42,kind)
            self.assertEqual(p.cfg['static_station_m'][0],0.)
            self.assertAlmostEqual(p.cfg['static_station_m'][1],Curve(p.links[p.cfg['target_link']]['points']).length)

    def test_actual_candidate_sampler_mirrors_same_lateral_interval(self):
        offsets=[];size=self.combined.static_size
        for side in [-1,1]:
            p=TunnelPlan(42,'static');checker=Mock();checker.vehicle=VehicleSpec.from_config(load_config())
            checker.evaluate.return_value=[S(blocked=True,feasible=True,start_blocked=False,feasible_sides=lambda:[])]
            p._static_context=(1877.,checker,[size])
            low,high=static_lateral_bounds(checker.vehicle.width,size.width,p.cfg['static_lateral_jitter_m'])
            p.rng=Mock();p.rng.choice.side_effect=[size,side]
            p.rng.uniform.side_effect=[60.,(low+high)/2,0.]
            with patch('scen_runner.planning.tunnel_plan.obstacle_at',return_value=self.combined.static) as place, \
                 patch('scen_runner.planning.tunnel_plan.basic_check',return_value=''),patch('scen_runner.planning.tunnel_plan.blocks_straight',return_value=True):
                self.assertTrue(p.prepare_static(lambda:False,1))
            self.assertEqual(p.rng.uniform.call_args_list[1].args,(low,high))
            offsets.append(place.call_args.args[3])
        self.assertEqual(offsets[0],-offsets[1])



if __name__=='__main__':unittest.main()
