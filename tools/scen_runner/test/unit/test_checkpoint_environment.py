"""Six-case exclusion, scene checkpoint detection and section-repeat scheduling."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.runtime.checkpoint_environment import CheckpointEnvironment
from scen_runner.morai.episode_link import SceneLink
from scen_runner.config import load_config


CFG=dict(weathers=['Sunny','Foggy'],hours=[11,13,15],instantly=True)


def point(aid,x,y=0):
    return dict(id=aid,sequence=int(aid) if aid.isdigit() else 0,
                pose=dict(position=dict(x=x,y=y,z=0)),pass_radius_m=3.,
                is_start=aid=='START_END',is_finish=aid=='START_END')


def ego(x,y=0,speed=0):
    return S(transform=S(location=S(x=x,y=y,z=0)),velocity=S(x=speed,y=0,z=0))


class EnvironmentTest(unittest.TestCase):
    def scheduler(self,points=None):
        link=Mock()
        scheduler=CheckpointEnvironment(link,CFG,points or [point('1',0),point('2',30)])
        return scheduler,link

    def test_excludes_only_previous_pair_and_restores_older_pair_as_candidate(self):
        e,l=self.scheduler();e.begin(42,'scene')
        e.current=('Sunny',11)
        def choose(choices):
            self.assertEqual(set(choices),{(w,h) for w in ['Sunny','Foggy'] for h in [11,13,15]}-{('Sunny',11)})
            return ('Sunny',13)
        with patch.object(e.rng,'choice',side_effect=choose):e.change('check')
        self.assertEqual(e.current,('Sunny',13))
        with patch.object(e.rng,'choice',side_effect=lambda choices: ('Sunny',11) if ('Sunny',11) in choices else self.fail('older pair excluded')):
            e.change('check2')
        self.assertEqual(e.current,('Sunny',11))

    def test_long_random_sequence_never_repeats_adjacent_pair(self):
        e,l=self.scheduler();e.begin(1,'scene')
        pairs=[e.current]
        with patch('builtins.print'):
            for _ in range(300):e.change('check');pairs.append(e.current)
        self.assertEqual(len(set(pairs)),6)
        self.assertTrue(all(a!=b for a,b in zip(pairs,pairs[1:])))

    def test_scene_point_radius_changes_once_while_ego_remains_there(self):
        e,l=self.scheduler();e.begin(42,'scene');initial=e.current
        self.assertEqual(e.update(ego(-2),0),['1'])
        self.assertNotEqual(e.current,initial)
        for t in range(1,6):self.assertEqual(e.update(ego(0),t),[])
        self.assertEqual(l.set_environment.call_count,2)

    def test_segment_crossing_detects_high_speed_pass_with_both_samples_outside(self):
        e,l=self.scheduler();e.begin(42,'scene')
        self.assertEqual(e.update(ego(-5,speed=80),0),[])
        self.assertEqual(e.update(ego(5,speed=80),.5),['1'])

    def test_reset_teleport_does_not_trigger_points_along_jump(self):
        e,l=self.scheduler();e.begin(42,'scene')
        e.update(ego(-100),0)
        self.assertEqual(e.update(ego(100),.2),[])
        self.assertEqual(l.set_environment.call_count,1)

    def test_section_environment_is_fixed_until_new_repeat_and_pair_exclusion_spans_repeats(self):
        e,l=self.scheduler();e.begin(42,'section');previous=e.current
        for t in range(3):self.assertEqual(e.update(ego(0),t),[])
        self.assertEqual(l.set_environment.call_count,1)
        e.begin(43,'section')
        self.assertNotEqual(e.current,previous);self.assertEqual(l.set_environment.call_count,2)
        self.assertFalse(e.visited)

    def test_failure_does_not_commit_new_case_or_mark_checkpoint_passed(self):
        e,l=self.scheduler();e.begin(42,'scene');previous=e.current
        l.set_environment.side_effect=RuntimeError('weather RPC failed')
        with self.assertRaisesRegex(RuntimeError,'weather RPC failed'):e.update(ego(0),0)
        self.assertEqual(e.current,previous);self.assertFalse(e.visited)
        self.assertIn('RPC failed',e.snapshot()['error'])

    def test_return_to_start_after_cp14_rearms_same_points_next_lap(self):
        e,l=self.scheduler([point('START_END',0),point('1',10),point('14',30)])
        e.begin(42,'scene')
        e.update(ego(10,speed=20),0);e.update(ego(30,speed=20),5)
        self.assertEqual(e.visited,{'1','14'})
        count=l.set_environment.call_count
        e.update(ego(0,speed=20),10)
        self.assertEqual(l.set_environment.call_count,count)
        self.assertNotIn('1',e.visited)
        self.assertEqual(e.update(ego(10,speed=20),15),['1'])

    def test_user_image_cp11_override_without_modifying_metadata(self):
        e=CheckpointEnvironment(Mock())
        self.assertEqual(len(e.checkpoints),14)
        c=next(p for p in e.checkpoints if p['id']=='11')
        self.assertAlmostEqual(c['xy'][0],73.55110931396484)
        self.assertAlmostEqual(c['xy'][1],-238.7718963623047)

    def test_full_current_route_replay_hits_all_fourteen_checkpoints_at_eighty_kmh(self):
        from scen_runner.runtime.validation_episode import EpisodeRoute
        route=EpisodeRoute();e=CheckpointEnvironment(Mock());e.begin(42,'scene')
        passed=[];station=0.;now=0.
        with patch('builtins.print'):
            while station<route.curve.length:
                x,y,z,_=route.curve.pose(station)
                passed.extend(e.update(ego(x,y,80),now))
                station+=80/3.6*.2;now+=.2
        self.assertEqual(passed,[str(i) for i in range(1,15)])
        self.assertEqual(e.switch_count,15)

    def test_rpc_checks_responses_and_waits_for_readback_without_resending(self):
        link=SceneLink(load_config());stub=Mock();link._sa=S(_environment_stub=stub)
        stub.SetWeather.return_value=S(status=1);stub.SetTime.return_value=S(status=1)
        stub.GetWeather.return_value=S(type=2)
        stub.GetTime.side_effect=[S(hour=11),S(hour=13)]
        with patch('scen_runner.morai.episode_link.time.sleep'):link.set_environment('Foggy',13)
        stub.SetWeather.assert_called_once();stub.SetTime.assert_called_once()
        weather=stub.SetWeather.call_args.args[0];time=stub.SetTime.call_args.args[0]
        self.assertEqual(weather.type,2);self.assertTrue(weather.instantly)
        self.assertEqual(time.hour,13);self.assertFalse(time.data_only);self.assertTrue(time.instantly)
        self.assertEqual({f.name for f,_ in weather.ListFields()},{'type','instantly'})

    def test_failed_weather_does_not_send_time(self):
        link=SceneLink(load_config());stub=Mock();link._sa=S(_environment_stub=stub)
        stub.SetWeather.return_value=S(status=0,description='bad weather')
        with self.assertRaisesRegex(RuntimeError,'날씨 변경 실패'):link.set_environment('Foggy',13)
        stub.SetTime.assert_not_called()


if __name__=='__main__':unittest.main()
