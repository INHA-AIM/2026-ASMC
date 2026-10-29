"""Preparation pause/release and activation diagnostics preserve native movement."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.config import load_config
from scen_runner.morai.episode_link import SceneLink
from scen_runner.runtime.episode_sections import DynamicSection,TunnelSection
from scen_runner.records.episode_telemetry import crossing_records,CrossingConsole


class EpisodeCrossingTest(unittest.TestCase):
    def test_crossing_registers_native_route_after_body_state_without_extra_pause(self):
        link=Mock(actor_scopes={});link.spawn_crossing.return_value='7'
        link.states.return_value=[]
        with patch('scen_runner.runtime.episode_actors.SceneLink',return_value=link):
            section=DynamicSection('dynamic',5,'scope',load_config(),lambda:False,lambda *_:None)
        self.assertFalse(section.poll_prepare())
        self.assertFalse(section.poll_prepare());link.spawn_crossing.assert_called_once()
        link.register_crossing.assert_not_called()
        link.states.return_value=[S(actor_info=S(id=S(value='7'),object_type=2))]
        self.assertTrue(section.poll_prepare())
        link.register_crossing.assert_called_once_with('7',section.plan)
        self.assertTrue(section.crossing_details()['7']['route_registered'])
        section.resume_schedule(0.)
        section.resume_schedule(1.)
        link.pause_actor.assert_not_called()
        self.assertFalse(section.crossing_details()['7']['preparation_paused'])
        link.set_speed_limit.assert_not_called()

    def test_pause_rpc_uses_pedestrian_kind_and_own_scope(self):
        link=SceneLink(load_config());link.client_key='crossing_scope';stub=Mock()
        stub.SetPause.return_value=S(status=1);link._sa=S(_actor_stub=stub)
        link.pause_actor('7',2,False)
        request=stub.SetPause.call_args.args[0]
        self.assertEqual(request.actor_info.object_type,2)
        self.assertEqual(request.actor_info.client_key,'crossing_scope')
        self.assertEqual(request.actor_info.id.value,'7');self.assertFalse(request.enable)

    def test_failed_route_registration_retains_actor_for_cleanup(self):
        link=Mock(actor_scopes={});link.spawn_crossing.return_value='7'
        link.states.return_value=[S(actor_info=S(id=S(value='7'),object_type=2))]
        link.register_crossing.side_effect=RuntimeError('route failed')
        with patch('scen_runner.runtime.episode_actors.SceneLink',return_value=link):
            section=DynamicSection('dynamic',5,'scope',load_config(),lambda:False,lambda *_:None)
        section.poll_prepare()
        with self.assertRaisesRegex(RuntimeError,'route failed'):section.poll_prepare()
        self.assertIn('7',section.owned())

    def test_tunnel_dynamic_uses_same_deferred_route_lifecycle(self):
        link=Mock(actor_scopes={});link.spawn_crossing.return_value='7'
        from scen_runner.config import load_catalog
        catalog=load_catalog()
        link.available_obstacle_models.return_value=[
            catalog['models'][name].get('morai_model',name)
            for name in catalog['available_models']]
        with patch('scen_runner.runtime.episode_actors.SceneLink',return_value=link):
            section=TunnelSection('tunnel',44,'scope',load_config(),lambda:False,lambda *_:None)
        from scen_runner.planning.random_crossing import CrossingGenerator
        crossing=CrossingGenerator(5).next_plan()
        section.plan=S(static=None,dynamic=crossing,prepare=lambda *args,**kwargs:True,scene_type='dynamic')
        self.assertFalse(section.poll_prepare());link.register_crossing.assert_not_called()
        link.states.return_value=[S(actor_info=S(id=S(value='7'),object_type=2))]
        self.assertTrue(section.poll_prepare())
        link.register_crossing.assert_called_once_with('7',crossing)
        section.resume_schedule(0.);link.pause_actor.assert_not_called()

    def test_spawn_and_later_route_preserve_activation_speed_and_native_endpoints(self):
        from scen_runner.planning.random_crossing import CrossingGenerator
        plan=CrossingGenerator(1181241943).next_plan();link=SceneLink(load_config());stub=Mock()
        stub.SpawnPedestrian.return_value=S(status=1,custom_message='7')
        stub.SetPedestrianWaypoint.return_value=S(status=1)
        link._sa=S(_actor_stub=stub)
        aid=link.spawn_crossing(plan,with_route=False);link.register_crossing(aid,plan)
        spawn=stub.SpawnPedestrian.call_args.args[0]
        self.assertFalse(spawn.start_action);self.assertEqual(len(spawn.waypoints.waypoints),0)
        self.assertAlmostEqual(spawn.active_dist,plan.cfg['activation_distance_m'],places=4)
        route=stub.SetPedestrianWaypoint.call_args.args[0]
        self.assertEqual(route.actor_info.id.value,'7');self.assertEqual(route.actor_info.object_type,2)
        self.assertEqual(len(route.waypoints.waypoints),3)
        self.assertAlmostEqual(route.waypoints.waypoints[-1].waypoint.x,plan.end[0],places=4)
        self.assertEqual(route.waypoints.waypoints[-1].speed,18)
        stub.SetPause.assert_not_called()

    def test_records_distinguish_start_waiting_movement_endpoint_and_missing(self):
        def state(aid,x,y,speed=0):
            return S(actor_info=S(id=S(value=aid)),transform=S(location=S(x=x,y=y,z=0)),velocity=S(x=speed,y=0,z=0))
        details={'7':dict(id='7',model='Deer1',section='dynamic',start=[0,0,0],end=[10,0,0],
                         activation_distance_m=15,preparation_paused=False)}
        statuses=[]
        for x,speed in [(0,0),(2,10),(10,0)]:
            records=crossing_records([state('Ego',0,12),state('7',x,0,speed)],details)
            statuses.append(records[0]['status'])
            self.assertTrue(records[0]['inside_activation'])
        self.assertEqual(statuses,['거리 조건 충족 · 출발점 대기','이동 관찰','끝점 도착'])
        self.assertEqual(crossing_records([state('Ego',0,12),state('7',0,0,17)],details)[0]['status'],
                         '거리 조건 충족 · 출발점 대기')
        self.assertEqual(crossing_records([state('Ego',0,20),state('7',0,0)],details)[0]['status'],
                         '거리 조건 밖 · 출발점 대기')
        self.assertEqual(crossing_records([state('Ego',0,12)],details)[0]['status'],'상태 미수신')

    def test_stalled_diagnostic_printed_once_and_cleared_for_new_episode(self):
        console=CrossingConsole()
        record=dict(id='7',section='dynamic',model='Deer1',status='거리 조건 충족 · 출발점 대기',
                    ego_to_start_m=12,activation_distance_m=15,moved_from_start_m=0,remaining_to_end_m=10)
        with patch('builtins.print') as log:
            for t in [0,1,2,3,4,5,6]:console.sample([record],1,'running',t)
            self.assertEqual(log.call_count,2)
            self.assertIn('3초 이상 이동 미관찰',log.call_args.args[0])
            console.sample([record],2,'running',7)
            self.assertEqual(log.call_count,3)


if __name__=='__main__':unittest.main()
