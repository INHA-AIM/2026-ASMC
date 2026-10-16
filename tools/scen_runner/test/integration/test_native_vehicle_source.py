import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.config import load_config
from scen_runner.morai.episode_link import SceneLink
from scen_runner.runtime.episode_traffic import HighSpeedSection
from scen_runner.morai.native_vehicle_source import create_ticket,SourcePlacementDeferred,waiting_details


class NativeSourceTest(unittest.TestCase):
    @staticmethod
    def source():
        with patch('scen_runner.runtime.episode_actors.SceneLink',return_value=Mock(actor_scopes={})):
            return HighSpeedSection('high_speed',42,'test_source',load_config(),
                                    lambda:False,lambda *_:None,clock=lambda:0.).sources[0]

    def test_actual_custom_target_and_required_empty_id_are_sent(self):
        link=SceneLink(load_config());stub=Mock();actors=Mock()
        stub.CreateVehicleSpawnPoint.return_value=SimpleNamespace(status=1,custom_message='7')
        actors.GetOptionName.return_value=SimpleNamespace(value='SpawnPoint')
        link._sa=SimpleNamespace(_scenario_stub=stub,_actor_stub=actors)
        lane=self.source()
        ticket=create_ticket(link,lane,'2015_Kia_K5',78)
        r=stub.CreateVehicleSpawnPoint.call_args.args[0]
        wire=type(r).FromString(r.SerializeToString())
        self.assertTrue(wire.spawn_point_info.object_info.HasField('id'))
        self.assertTrue(wire.spawn_point_info.pause)
        self.assertEqual(wire.config.desired_velocity_type,2)
        self.assertEqual(wire.config.min_desired_velocity_custom,78)
        self.assertEqual(wire.config.max_desired_velocity_custom,78)
        self.assertEqual(wire.config.parameter_type,1)
        self.assertEqual(wire.config.destination_obj_unique_id,-1)
        self.assertEqual(wire.config.start_link_info.waypoint_idx,78)
        self.assertEqual(wire.config.end_link_info.waypoint_idx,83)
        self.assertEqual(wire.config.min_spawn_period,lane.plan.period)
        self.assertEqual(ticket['target'],78)
        actors.SpawnVehicle.assert_not_called()

    def test_false_response_is_deferred_but_unknown_errors_are_not(self):
        link=SceneLink(load_config());stub=Mock()
        link._sa=SimpleNamespace(_scenario_stub=stub)
        lane=self.source()
        stub.CreateVehicleSpawnPoint.return_value=SimpleNamespace(status=0,description='False')
        with self.assertRaises(SourcePlacementDeferred):create_ticket(link,lane,'2015_Kia_K5',78)
        stub.CreateVehicleSpawnPoint.return_value=SimpleNamespace(status=0,description='Unknown error')
        with self.assertRaisesRegex(RuntimeError,'Unknown error'):create_ticket(link,lane,'2015_Kia_K5',78)

    def test_wait_diagnostic_exposes_id_and_link_rejections(self):
        lane=self.source();pose=lane.plan.cfg['spawn_pose']
        state=SimpleNamespace(actor_info=SimpleNamespace(object_type=1,id=SimpleNamespace(value='7')),
              transform=SimpleNamespace(location=SimpleNamespace(x=pose['x'],y=pose['y'])),
              vehicle_state=SimpleNamespace(current_link_info=SimpleNamespace(id=SimpleNamespace(value='other'))))
        diagnostic=waiting_details(dict(lane=lane,baseline={'7'}),[state],{'7'})
        self.assertIn('기존/관리 ID',diagnostic)
        self.assertIn('요청 전 ID',diagnostic)
        self.assertIn('시작 링크 불일치',diagnostic)
