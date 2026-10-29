import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.runtime.validation_episode import EpisodeController,EpisodeLink,EpisodeRoute,PREPARE_ORDER
from scen_runner.config import load_config


class FakeSection:
    serial=0
    def __init__(self,name,*args):
        FakeSection.serial+=1;self.name=name;self.id=str(FakeSection.serial);self.prepared=False;self.known_ids=set()
        self.values={};self.cleanup=Mock();self.resume_schedule=Mock();self.tick=Mock()
    def poll_prepare(self):self.prepared=True;self.values={self.id:(self.name,15)};return True
    def owned(self):return dict(self.values)
    def ready_live(self,states):return self.prepared and any(s.actor_info.id.value==self.id for s in states)
    def forget_reused_id(self,aid):self.values.pop(aid,None);self.known_ids.discard(aid)


class EpisodeTest(unittest.TestCase):
    def test_tunnel_finish_waits_for_selected_normal_mode(self):
        e,l,r=self.engine('section','tunnel');e.planning=Mock();e.planning.ready.return_value=True
        e.planning.recovery_ready.return_value=False;r.planning_recovery_tail=20.
        e.begin(1);self.prepare(e);self.now=2.;r.finished.return_value=True
        l.hold.reset_mock()
        self.assertIsNone(e.step());self.assertTrue(e.section_finish_pending)
        l.hold.assert_called_once()
        r.finished.return_value=False;e.planning.recovery_ready.return_value=True
        self.assertEqual(e.step(),'section_end')

    def test_episode_end_records_stagnation_separately_from_completion(self):
        e,l,r=self.engine('section','tunnel')
        events=[];e.on_lifecycle_observation=events.append
        e.begin(1);self.prepare(e)
        self.now=160.
        self.assertEqual(e.end_run('stagnation'),'stagnation')
        ended=[event for event in events if event['event']=='episode_end']
        self.assertEqual(len(ended),1)
        self.assertEqual(ended[0]['reason'],'stagnation')
        self.assertEqual(ended[0]['episode'],1)
        self.assertGreater(ended[0]['elapsed_s'],150.)

    def test_background_signal_failure_holds_ego_during_driving(self):
        e,l,r=self.engine('section','dynamic')
        e.begin(1);self.prepare(e)
        l.hold.reset_mock()
        e.signals.error='native red state persisted'
        with self.assertRaisesRegex(RuntimeError,'native red state persisted'):e.step()
        l.hold.assert_called_once()
        self.assertEqual(e.current_operation,dict(section='dynamic',action='section_signals'))

    def test_section_green_before_planning_and_restored_when_gui_switches_to_scene(self):
        e,l,r=self.engine('section','static')
        r.cfg={'section_traffic_lights':dict(enabled=True,color=48,ids=dict(static=['A']),controllers=dict(static={'J':0}))}
        l.intersection_state.return_value=dict(phase=1,colors={'A':1})
        l.traffic_light_color.return_value=1
        e.planning=Mock();e.planning.ready.return_value=True
        events=[]
        l.set_intersection_phase.side_effect=lambda aid,phase:events.append(('phase',phase))
        e.planning.ready.side_effect=lambda *_:events.append(('planner',)) or True
        l.release.side_effect=lambda:events.append(('release',))
        e.begin(1);self.prepare(e)
        self.assertLess(events.index(('phase',0)),events.index(('planner',)))
        self.assertLess(events.index(('planner',)),events.index(('release',)))
        e.begin(2,selection=('scene','static'),reset_budget=True)
        self.prepare(e)
        self.assertEqual([x for x in events if x[0]=='phase'],[('phase',0),('phase',1)])
        self.assertEqual(e.signals.snapshot()['policy'],'unchanged')

    def test_signal_failure_blocks_release_and_stop_still_cleans_objects(self):
        e,l,r=self.engine('section')
        r.cfg={'section_traffic_lights':dict(enabled=True,color=48,ids=dict(static=['A']),controllers=dict(static={'J':0}))}
        l.intersection_state.return_value=dict(phase=1,colors={'A':1})
        l.traffic_light_color.return_value=1
        e.begin(1);e.step()
        l.verify_traffic_light_colors.side_effect=RuntimeError('signal mismatch')
        with self.assertRaisesRegex(RuntimeError,'signal mismatch'):e.step()
        l.release.assert_not_called()
        l.set_intersection_phase.side_effect=RuntimeError('restore timeout')
        actor=e.adapters[0]
        with self.assertRaisesRegex(RuntimeError,'restore timeout'):e.stop()
        actor.cleanup.assert_called_once()
        l.release.assert_not_called()
        l.set_intersection_phase.side_effect=None
        e.stop()
        self.assertFalse(e.signals.original)

    def test_gui_section_switch_stops_planning_before_old_cleanup_and_applies_new_budget(self):
        from scen_runner.runtime.run_counts import RunBudget
        e,l,r=self.engine('section','static'); e.planning=Mock();e.planning.ready.return_value=True
        e.run_budget=RunBudget(dict(scene_runs=3,section_runs=dict(static=1,dynamic=2,roundabout=3,high_speed=4,tunnel=5)),
                               'section','static')
        e.begin(1);self.prepare(e)
        self.assertFalse(e.run_budget.can_start)
        events=[]
        e.planning.stop.side_effect=lambda:events.append(('stop',e.section))
        e.adapters[0].cleanup.side_effect=lambda:events.append(('cleanup',e.section))
        l.reset.side_effect=lambda pose:events.append(('respawn',e.section))
        e.begin(2,selection=('section','dynamic'),reset_budget=True)
        self.assertEqual(events,[('stop','static'),('cleanup','static'),('respawn','dynamic')])
        self.assertEqual(e.phase,'preparing');self.assertEqual(e.run_budget.limit,2)
        self.assertEqual(e.run_budget.started,0);self.assertEqual(e.episode,2)
        self.prepare(e);self.assertEqual(e.run_budget.started,1)

    def test_linked_planning_stops_before_teleport_and_readiness_precedes_release(self):
        e,l,r=self.engine('section'); e.planning=Mock(status='objects 미수신')
        events=[]
        l.hold.side_effect=lambda:events.append('hold')
        e.planning.stop.side_effect=lambda:events.append('planning_stop')
        l.reset.side_effect=lambda pose:events.append('teleport')
        l.release.side_effect=lambda:events.append('release')
        e.planning.ready.return_value=False
        e.begin(1)
        self.assertEqual(events[:3],['hold','planning_stop','teleport'])
        e.step(); e.step()
        self.assertEqual(e.phase,'preparing'); l.release.assert_not_called()
        e.planning.ready.return_value=True; e.step()
        self.assertEqual(e.phase,'running'); self.assertEqual(events[-1],'release')
        e.end_run('collision'); e.begin(2)
        self.assertEqual(events[-3:],['hold','planning_stop','teleport'])
        self.assertEqual(e.planning.stop.call_count,2)

    def test_linked_planning_failure_holds_ego_and_last_run_stops_planning_before_final_return(self):
        e,l,r=self.engine('section'); e.planning=Mock(); e.planning.ready.return_value=True
        e.run_budget=self.budget(1); e.begin(1); self.prepare(e)
        e.planning.check.side_effect=RuntimeError('supervisor died')
        l.hold.reset_mock()
        with self.assertRaisesRegex(RuntimeError,'supervisor died'):e.step()
        l.hold.assert_called_once()
        e.planning.check.side_effect=None
        self.now=2.;r.finished.return_value=True
        events=[]
        e.planning.stop.side_effect=lambda:events.append('stop')
        l.reset.side_effect=lambda pose:events.append('return')
        e.step()
        self.assertEqual(events,['stop','return'])
        self.assertEqual(e.phase,'runs_done')

    @staticmethod
    def budget(limit):
        from scen_runner.runtime.run_counts import RunBudget,SECTIONS
        return RunBudget(dict(scene_runs=limit,section_runs={name:limit for name in SECTIONS}))

    @staticmethod
    def body(aid,x=0.,kind=1):
        return S(actor_info=S(id=S(value=aid),object_type=kind),
                 bounding_box=S(location=S(x=x,y=0.,z=1.),extent=S(x=1.,y=2.,z=1.)),
                 velocity=S(x=0.,y=0.,z=0.))

    def contact_sample(self,e,aid,x=3.,started=None):
        return dict(epoch=e.contact_epoch,error=None,contacts=[aid],owned_ids=[aid],
                    query_started_at=e.run_started if started is None else started,query_s=.01,
                    actor_states=[self.body('Ego'),self.body(aid,x)])

    def engine(self,mode='scene',section='static'):
        link=Mock();link.held_at.return_value=True
        link.ego_id='Ego'
        link.ego.return_value=S(transform=S(location=S(x=1.,y=2.,z=3.)),velocity=S(x=0.,y=0.,z=0.))
        route=Mock();route.start.return_value=dict(x=1,y=2,z=3,roll=0,pitch=0,yaw=0);route.finished.return_value=False
        route.cfg={}
        self.now=0.
        lease=Mock(held=True)
        engine=EpisodeController(link,route,mode,section,42,factory=FakeSection,clock=lambda:self.now,lease=lease)
        engine.signals._start_monitor=Mock()
        link.states.side_effect=lambda:[S(actor_info=S(id=S(value=aid))) for aid in engine.owned]
        return engine,link,route

    def prepare(self,e):
        for _ in range(10):
            e.step()
            if e.phase=='running':return
        self.fail('episode did not become ready')

    def test_scene_prepares_all_sections_and_releases_only_after_barrier(self):
        e,l,r=self.engine();e.begin(1)
        for _ in PREPARE_ORDER:
            e.step();l.release.assert_not_called()
        self.assertEqual([a.name for a in e.adapters],PREPARE_ORDER)
        e.step();l.release.assert_called_once();self.assertEqual(e.phase,'running')
        l.hold.assert_called_once();l.reset.assert_called_once_with(r.start.return_value)

    def test_replay_recorded_63kmh_ego_starts_at_zero_on_first_run_and_repeat(self):
        from scen_runner.records.collision_replay import configure_replay
        from scen_runner.config import load_yaml,TOOL_ROOT
        e,l,r=self.engine('section');r.section.return_value={}
        definition=dict(section='static',seed=77,ego_at_entry=dict(speed_kmh=63.),
            route_config=load_yaml(TOOL_ROOT/'config/validation.yaml'),
            start_pose=dict(r.start.return_value),environment=dict(weather='Sunny',hour=15))
        configure_replay(e,definition)
        e.factory=FakeSection  # Offline actors; exercise the real replay ready/release flow.
        wire_link=EpisodeLink(load_config());stub=Mock()
        for name in ['SetPause','SetVelocity','SetVehicleGear']:
            getattr(stub,name).return_value=S(status=1)
        wire_link._sa=S(_actor_stub=stub);l.release.side_effect=wire_link.release
        for epoch in (1,2):
            e.begin(epoch);self.prepare(e)
        self.assertEqual(e.phase,'running');self.assertEqual(stub.SetVelocity.call_count,2)
        for call in stub.SetVelocity.call_args_list:
            self.assertEqual(call.args[0].actor_info.id.value,'Ego')
            self.assertEqual(call.args[0].velocity,0.)
        self.assertEqual(stub.SetVehicleGear.call_args.args[0].gear,4)
        self.assertFalse(stub.SetPause.call_args.args[0].enable)
        self.assertEqual(definition['ego_at_entry']['speed_kmh'],63.)

    def test_environment_does_not_change_object_seeds_and_applies_before_release(self):
        e,l,r=self.engine('scene');seeds=[]
        def factory(name,seed,*args):
            seeds.append(seed);return FakeSection(name,seed,*args)
        e.factory=factory;e.begin(1)
        l.set_environment.assert_called_once();l.release.assert_not_called()
        self.prepare(e)
        import random
        expected=random.Random(42)
        self.assertEqual(seeds,[expected.randrange(2**32) for _ in PREPARE_ORDER])

    def test_missing_actor_state_or_ego_movement_blocks_release(self):
        e,l,r=self.engine('section');e.begin(1);e.step()
        l.states.side_effect=None;l.states.return_value=[];e.step();l.release.assert_not_called()
        l.states.side_effect=lambda:[S(actor_info=S(id=S(value=aid))) for aid in e.owned]
        l.held_at.return_value=False
        with self.assertRaisesRegex(RuntimeError,'정지 조건'):e.step()
        l.release.assert_not_called()

    def test_preparation_does_not_consume_a_driving_attempt(self):
        e,l,r=self.engine('section');e.run_budget=self.budget(1);e.begin(1)
        e.step();self.assertEqual(e.run_budget.started,0)
        l.states.side_effect=None;l.states.return_value=[]
        e.step();self.assertEqual(e.run_budget.started,0)
        l.states.side_effect=lambda:[S(actor_info=S(id=S(value=aid))) for aid in e.owned]
        self.prepare(e)
        self.assertEqual(e.run_budget.started,1);l.release.assert_called_once()

    @staticmethod
    def stagnant_monitor():
        from scen_runner.runtime.stagnation import StagnationMonitor
        return StagnationMonitor(dict(enabled=True,duration_s=30.,position_radius_m=1.,max_sample_gap_s=5.))

    def test_stagnation_saves_before_hold_and_uses_normal_repeat_flow(self):
        e,l,r=self.engine('section');e.stagnation=self.stagnant_monitor()
        e.begin(1);self.prepare(e);saved=[]
        def save(engine,evidence,states):
            self.assertEqual(engine.phase,'running');self.assertTrue(engine.adapters)
            self.assertEqual(l.reset.call_count,1);saved.append(evidence)
        e.on_stagnation=save
        for t in range(1,30):self.now=float(t);self.assertIsNone(e.step())
        self.now=30.;self.assertEqual(e.step(),'stagnation')
        self.assertEqual(len(saved),1);self.assertEqual(saved[0]['duration_s'],30.)
        e.begin(2);self.assertEqual(e.episode,2);self.assertEqual(e.pending,['static'])
        self.assertFalse(e.stagnation.detected)

    def test_stagnation_on_last_drive_returns_and_completes_without_new_drive(self):
        e,l,r=self.engine('section');e.stagnation=self.stagnant_monitor();e.run_budget=self.budget(1)
        e.on_stagnation=Mock();e.begin(1);self.prepare(e)
        for t in range(1,30):self.now=float(t);e.step()
        self.now=30.;l.release.reset_mock()
        self.assertIsNone(e.step());self.assertEqual(e.phase,'runs_done')
        e.on_stagnation.assert_called_once();self.assertEqual(e.run_budget.started,1)
        self.assertEqual(l.reset.call_count,2);l.release.assert_not_called()

    def test_preparation_is_not_stagnation_and_save_failure_keeps_scene(self):
        e,l,r=self.engine('section');e.stagnation=self.stagnant_monitor();e.on_stagnation=Mock()
        e.begin(1);l.states.side_effect=None;l.states.return_value=[]
        for t in range(10,101,10):self.now=float(t);e.step()
        e.on_stagnation.assert_not_called()
        l.states.side_effect=lambda:[S(actor_info=S(id=S(value=aid))) for aid in e.owned]
        self.prepare(e)
        e.on_stagnation.side_effect=RuntimeError('save failed')
        for t in range(101,130):self.now=float(t);e.step()
        self.now=130.
        with self.assertRaisesRegex(RuntimeError,'save failed'):e.step()
        self.assertEqual(e.phase,'running');self.assertTrue(e.adapters)
        self.assertEqual(l.reset.call_count,1)

    def test_collision_during_stagnation_snapshot_takes_priority(self):
        e,l,r=self.engine('section');e.stagnation=self.stagnant_monitor();e.on_stagnation=Mock()
        e.begin(1);self.prepare(e);aid=next(iter(e.owned))
        for t in range(1,30):self.now=float(t);e.step()
        e.on_state_observation=lambda *_:e.contact(self.contact_sample(e,aid))
        self.now=30.;self.assertEqual(e.step(),'collision')
        e.on_stagnation.assert_not_called()

    def test_last_collision_saves_cleans_and_returns_held_without_another_drive(self):
        e,l,r=self.engine('scene');e.run_budget=self.budget(1);e.begin(1);self.prepare(e)
        aid=next(iter(e.owned));saved=[];e.on_collision=lambda *args:saved.append(args)
        old=list(e.adapters);before=l.reset.call_count;l.release.reset_mock()
        self.assertTrue(e.contact(self.contact_sample(e,aid)))
        self.assertIsNone(e.step())
        self.assertEqual(len(saved),1);self.assertEqual(e.phase,'runs_done')
        self.assertEqual(e.run_budget.started,1);self.assertEqual(l.reset.call_count,before+1)
        l.reset.assert_called_with(r.start.return_value)
        r.start.assert_called_with('scene','static')
        for adapter in old:adapter.cleanup.assert_called_once()
        self.assertFalse(e.adapters);self.assertFalse(e.owned)
        l.release.assert_not_called();self.assertTrue(e.cancelled.is_set())
        e.step();l.release.assert_not_called()

    def test_section_total_two_drives_and_stops_at_second_end(self):
        e,l,r=self.engine('section','dynamic');e.run_budget=self.budget(2)
        e.run_budget.reset('section','dynamic')
        e.begin(1);self.prepare(e);self.now=2.;r.finished.return_value=True
        self.assertEqual(e.step(),'section_end');self.assertEqual(e.run_budget.started,1)
        e.begin(2);r.finished.return_value=False;self.prepare(e)
        self.assertEqual(e.run_budget.started,2)
        self.now=4.;r.finished.return_value=True;l.release.reset_mock()
        self.assertIsNone(e.step());self.assertEqual(e.phase,'runs_done')
        self.assertEqual(l.reset.call_count,3);l.release.assert_not_called()
        r.start.assert_called_with('section','dynamic')
        self.assertEqual(e.run_budget.started,2)

    def test_full_scene_limit_returns_to_start_after_final_finish_without_new_drive(self):
        from scen_runner.runtime.scene_completion import SceneCompletion
        e,l,r=self.engine('scene');e.run_budget=self.budget(1)
        e.completion=SceneCompletion([dict(id='14',xy=(30.,0.),radius=3.)],dict(xy=(0.,0.),radius=3.))
        e.begin(1);self.prepare(e)
        self.now=10.;l.ego.return_value=S(transform=S(location=S(x=30.,y=0.)),velocity=S(x=20.,y=0.,z=0.))
        e.step()
        self.now=20.;l.ego.return_value=S(transform=S(location=S(x=0.,y=0.)),velocity=S(x=20.,y=0.,z=0.))
        l.release.reset_mock()
        self.assertIsNone(e.step());self.assertEqual(e.phase,'runs_done')
        self.assertEqual(e.episode,1);self.assertEqual(e.run_budget.started,1)
        self.assertEqual(l.reset.call_count,2)
        r.start.assert_called_with('scene','static')
        l.release.assert_not_called()

    def test_final_cleanup_precedes_return_and_return_does_not_prepare_new_objects(self):
        e,l,r=self.engine('section','tunnel');e.run_budget=self.budget(1)
        e.begin(1);self.prepare(e)
        events=[]
        e.adapters[0].cleanup.side_effect=lambda:events.append('cleanup')
        l.reset.side_effect=lambda pose:events.append('return')
        l.held_at.side_effect=lambda pose:events.append('verify') or True
        self.now=2.;r.finished.return_value=True;l.release.reset_mock()
        e.step()
        self.assertEqual(events,['cleanup','return','verify'])
        self.assertFalse(e.pending);self.assertFalse(e.adapters)
        self.assertEqual(e.episode,1);l.release.assert_not_called()

    def test_failed_final_return_or_verification_prevents_automatic_exit(self):
        for failure in ('reset','verification'):
            with self.subTest(failure=failure):
                e,l,r=self.engine('section');e.run_budget=self.budget(1)
                e.begin(1);self.prepare(e);self.now=2.;r.finished.return_value=True
                if failure=='reset':l.reset.side_effect=RuntimeError('reset failed')
                else:l.held_at.return_value=False
                l.release.reset_mock()
                with patch('scen_runner.runtime.validation_episode.time.sleep'),self.assertRaises(RuntimeError):e.step()
                self.assertNotEqual(e.phase,'runs_done')
                self.assertEqual(e.current_operation,dict(section=None,action='final_reset'))
                l.release.assert_not_called()

    def test_final_pose_readback_waits_for_updated_cache(self):
        e,l,r=self.engine('section');e.run_budget=self.budget(1)
        e.begin(1);self.prepare(e);self.now=2.;r.finished.return_value=True
        l.held_at.side_effect=[False,False,True]
        with patch('scen_runner.runtime.validation_episode.time.sleep') as sleep:e.step()
        self.assertEqual(e.phase,'runs_done');self.assertEqual(sleep.call_count,2)

    def test_failed_final_cleanup_does_not_label_batch_complete_or_release_ego(self):
        e,l,r=self.engine('section');e.run_budget=self.budget(1);e.begin(1);self.prepare(e)
        e.adapters[0].cleanup.side_effect=RuntimeError('cleanup failed')
        self.now=2.;r.finished.return_value=True;l.release.reset_mock()
        with self.assertRaisesRegex(RuntimeError,'cleanup failed'):e.step()
        self.assertNotEqual(e.phase,'runs_done');l.release.assert_not_called()

    def test_ego_collision_causes_one_transition_and_old_epoch_is_ignored(self):
        e,l,r=self.engine('section');e.begin(1);self.prepare(e);aid=next(iter(e.owned))
        sample=self.contact_sample(e,aid)
        self.assertTrue(e.contact(sample));self.assertFalse(e.contact(sample))
        self.assertEqual(e.step(),'collision');self.assertEqual(e.phase,'transition')
        old=e.adapters[0];e.begin(2);old.cleanup.assert_called_once();self.prepare(e)
        self.assertEqual(e.episode,2);self.assertFalse(e.contact(sample))

    def test_preparation_query_delivered_after_release_is_ignored(self):
        e,l,r=self.engine('section');e.begin(1);e.step();aid=next(iter(e.owned))
        old=self.contact_sample(e,aid,started=0.)
        self.now=10.;self.prepare(e)
        self.assertFalse(e.contact(old));self.assertFalse(e.collision_requested)
        cleared=self.contact_sample(e,aid);cleared['contacts']=[]
        e.contact(cleared)
        self.assertTrue(e.contact(self.contact_sample(e,aid)))

    def test_same_id_in_native_contacts_with_body_258m_away_does_not_reset(self):
        e,l,r=self.engine('section');e.begin(1);self.prepare(e);aid=next(iter(e.owned))
        self.assertFalse(e.contact(self.contact_sample(e,aid,x=258.)))
        self.assertFalse(e.collision_requested)
        self.assertEqual(e.last_contact_evidence['records'][0]['verdict'],'rejected')
        self.assertIsNone(e.step());self.assertEqual(e.phase,'running')
        # Getting nearby alone is not a new collision.
        self.assertFalse(e.contact(self.contact_sample(e,aid,x=3.)))
        self.assertEqual(e.last_contact_evidence['records'][0]['verdict'],'stale')
        # A successful native release followed by a new entry re-arms this ID.
        cleared=self.contact_sample(e,aid);cleared['contacts']=[]
        e.contact(cleared)
        self.assertTrue(e.contact(self.contact_sample(e,aid,x=3.)))

    def test_preparation_contact_cannot_turn_into_collision_when_car_only_approaches(self):
        e,l,r=self.engine('section');e.begin(1);e.step();aid=next(iter(e.owned))
        sample=self.contact_sample(e,aid,x=258.,started=0.)
        self.assertFalse(e.contact(sample))
        self.now=10.;self.prepare(e)
        for x in (14.,8.,6.,3.,0.):
            self.assertFalse(e.contact(self.contact_sample(e,aid,x=x)))
        self.assertFalse(e.collision_requested)
        self.assertEqual(e.last_contact_evidence['records'][0]['verdict'],'stale')

    def test_contact_event_saved_before_reset_request_and_deduplicated(self):
        e,l,r=self.engine('section');e.begin(1);self.prepare(e);aid=next(iter(e.owned));saved=[]
        def save(evidence,sample):
            self.assertFalse(e.collision_requested)
            saved.append(evidence)
        e.on_contact_observation=save
        self.assertTrue(e.contact(self.contact_sample(e,aid)))
        self.assertEqual(saved[0]['records'][0]['verdict'],'accepted')
        self.assertEqual(saved[0]['episode'],1)
        e.on_contact_observation=Mock()
        e.contact(self.contact_sample(e,aid))
        e.on_contact_observation.reset_mock()
        e.contact(self.contact_sample(e,aid))
        e.on_contact_observation.assert_not_called()

    def test_missing_body_state_cannot_turn_native_id_into_confirmed_collision(self):
        e,l,r=self.engine('section');e.begin(1);self.prepare(e);aid=next(iter(e.owned))
        sample=self.contact_sample(e,aid);sample['actor_states']=[self.body('Ego')]
        self.assertFalse(e.contact(sample));self.assertFalse(e.collision_requested)
        self.assertEqual(e.last_contact_evidence['records'][0]['verdict'],'unknown')

    def test_query_timestamp_is_taken_after_slow_resume_and_ego_release(self):
        e,l,r=self.engine('section');e.begin(1);e.step()
        def release():self.now=5.
        l.release.side_effect=release
        self.prepare(e);self.assertEqual(e.run_started,5.)
        aid=next(iter(e.owned))
        self.assertFalse(e.contact(self.contact_sample(e,aid,started=4.9)))

    def test_foreign_contacts_do_not_restart_scene(self):
        e,l,r=self.engine('section');e.begin(1);self.prepare(e)
        self.assertFalse(e.contact(dict(epoch=1,error=None,contacts=['Ego','-100','foreign'],owned_ids=list(e.owned))))
        self.assertFalse(e.contact(dict(epoch=1,error='unavailable',contacts=list(e.owned),owned_ids=list(e.owned))))
        self.assertIsNone(e.step());self.assertEqual(e.phase,'running')

    def test_section_end_restarts_same_section_only(self):
        e,l,r=self.engine('section','tunnel');e.begin(1);self.prepare(e)
        self.assertEqual([a.name for a in e.adapters],['tunnel'])
        self.now=2.;r.finished.return_value=True
        self.assertEqual(e.step(),'section_end');e.begin(2)
        self.assertEqual(e.section,'tunnel');self.assertEqual(e.pending,['tunnel'])

    def test_real_dynamic_end_offset_pass_enters_same_repeat_flow(self):
        e,l,_=self.engine('section','dynamic');route=EpisodeRoute();e.route=route
        l.intersection_state.return_value=dict(phase=0,colors={'C1256W000016':1,'C1256W000018':1})
        e.begin(1);self.prepare(e)
        definition=route.section('dynamic');x,y=definition['end_xy'];h=definition['end_heading']
        import math
        tangent=(math.cos(h),math.sin(h));normal=(-tangent[1],tangent[0])
        def at(forward):
            return S(transform=S(location=S(x=x+forward*tangent[0]+5*normal[0],
                                            y=y+forward*tangent[1]+5*normal[1])),
                     velocity=S(x=25.,y=0.,z=0.))
        self.now=10.;l.ego.return_value=at(-10.)
        self.assertIsNone(e.step())
        self.now=12.;l.ego.return_value=at(8.)
        self.assertEqual(e.step(),'section_end')
        e.begin(2)
        self.assertEqual(e.pending,['dynamic']);self.assertEqual(e.section,'dynamic')

    def test_scene_finish_rebuilds_all_sections_with_new_seed_and_prepare_barrier(self):
        from scen_runner.runtime.scene_completion import SceneCompletion
        e,l,r=self.engine('scene')
        e.completion=SceneCompletion([dict(id='14',xy=(30.,0.),radius=3.)],dict(xy=(0.,0.),radius=3.))
        e.begin(1);self.prepare(e)
        previous_seed=e.seed;previous_environment=e.environment.current
        old=list(e.adapters);old_ids=set(e.owned)
        self.now=10.;l.ego.return_value=S(transform=S(location=S(x=30.,y=0.)),velocity=S(x=20.,y=0.,z=0.))
        self.assertIsNone(e.step());self.assertTrue(e.completion.checkpoint14_passed)
        self.now=20.;l.ego.return_value=S(transform=S(location=S(x=0.,y=0.)),velocity=S(x=20.,y=0.,z=0.))
        self.assertEqual(e.step(),'scene_end');self.assertEqual(e.phase,'transition')
        self.assertIsNone(e.step())
        e.begin(2)
        for adapter in old:adapter.cleanup.assert_called_once()
        self.assertEqual(e.episode,2);self.assertNotEqual(e.seed,previous_seed)
        self.assertNotEqual(e.environment.current,previous_environment)
        self.assertEqual(e.pending,PREPARE_ORDER)
        self.assertFalse(e.completion.checkpoint14_passed)
        self.assertEqual(l.release.call_count,1)
        for _ in PREPARE_ORDER:
            e.step();self.assertEqual(l.release.call_count,1)
        self.prepare(e)
        self.assertEqual(l.release.call_count,2)
        self.assertFalse(set(e.owned)&old_ids)
        self.assertEqual(l.reset.call_count,2)

    def test_collision_arriving_during_finish_query_takes_priority(self):
        from scen_runner.runtime.scene_completion import SceneCompletion
        e,l,r=self.engine('scene')
        e.completion=SceneCompletion([dict(id='14',xy=(30.,0.),radius=3.)],dict(xy=(0.,0.),radius=3.))
        e.begin(1);self.prepare(e)
        self.now=10.;l.ego.return_value=S(transform=S(location=S(x=30.,y=0.)),velocity=S(x=20.,y=0.,z=0.))
        e.step();aid=next(iter(e.owned));e.on_collision=Mock()
        def finish_state():
            self.assertTrue(e.contact(self.contact_sample(e,aid)))
            return S(transform=S(location=S(x=0.,y=0.)),velocity=S(x=20.,y=0.,z=0.))
        self.now=20.;l.ego.side_effect=finish_state
        self.assertEqual(e.step(),'collision');e.on_collision.assert_called_once()

    def test_pending_close_during_finish_hold_does_not_start_another_scene(self):
        from scen_runner.runtime.scene_completion import SceneCompletion
        e,l,r=self.engine('scene')
        e.completion=SceneCompletion([dict(id='14',xy=(30.,0.),radius=3.)],dict(xy=(0.,0.),radius=3.))
        e.begin(1);self.prepare(e)
        self.now=10.;l.ego.return_value=S(transform=S(location=S(x=30.,y=0.)),velocity=S(x=20.,y=0.,z=0.))
        e.step()
        l.hold.side_effect=e.cancelled.set
        self.now=20.;l.ego.return_value=S(transform=S(location=S(x=0.,y=0.)),velocity=S(x=20.,y=0.,z=0.))
        self.assertIsNone(e.step());self.assertNotEqual(e.phase,'transition')

    def test_failed_cleanup_prevents_new_ego_reset(self):
        e,l,r=self.engine();a=FakeSection('static');a.cleanup.side_effect=RuntimeError('delete failed');e.adapters=[a]
        with self.assertRaisesRegex(RuntimeError,'delete failed'):e.begin(1)
        l.hold.assert_called_once();l.reset.assert_not_called();self.assertIn(a,e.adapters)
        self.assertEqual(e.current_operation,dict(section='static',action='cleanup'))

    def test_prepare_failure_records_section_before_high_speed_can_be_reached(self):
        e,l,r=self.engine()
        e.begin(1)
        for _ in range(3):e.step()
        def factory(name,*args):
            adapter=FakeSection(name,*args)
            adapter.poll_prepare=Mock(side_effect=RuntimeError('vehicle failed'))
            return adapter
        e.factory=factory
        with self.assertRaisesRegex(RuntimeError,'vehicle failed'):e.step()
        self.assertEqual(e.current_operation,dict(section='roundabout',action='prepare'))
        self.assertEqual(e.pending,['roundabout','high_speed'])
        self.assertNotIn('high_speed',[a.name for a in e.adapters])
        l.release.assert_not_called()

    def test_stop_releases_ego_only_after_all_episode_objects_are_cleaned(self):
        e,l,r=self.engine('section');e.begin(1);self.prepare(e)
        calls=[]
        l.hold.side_effect=lambda:calls.append('hold')
        adapter=e.adapters[0];adapter.cleanup.side_effect=lambda:calls.append('cleanup')
        l.release.side_effect=lambda:calls.append('release')
        e.stop()
        self.assertEqual(calls,['hold','cleanup','release'])
        self.assertEqual(e.phase,'stopped');self.assertFalse(e.adapters)
        self.assertTrue(e.cancelled.is_set())

    def test_failed_stop_cleanup_keeps_ego_held_and_can_be_retried(self):
        e,l,r=self.engine('section');e.begin(1);self.prepare(e)
        l.release.reset_mock()
        adapter=e.adapters[0];adapter.cleanup.side_effect=RuntimeError('cleanup failed')
        with self.assertRaisesRegex(RuntimeError,'cleanup failed'):e.stop()
        l.release.assert_not_called();self.assertIn(adapter,e.adapters)
        adapter.cleanup.side_effect=None
        e.stop();l.release.assert_called_once();self.assertEqual(e.phase,'stopped')

    def test_planning_unavailable_during_stop_still_cleans_objects_and_keeps_ego_held(self):
        from scen_runner.orchestration.control import PlanningUnavailable
        e,l,r=self.engine('section');e.begin(1);self.prepare(e)
        adapter=e.adapters[0];e.planning=Mock()
        e.planning.stop.side_effect=PlanningUnavailable('supervisor exited first')
        l.release.reset_mock()
        e.stop()
        adapter.cleanup.assert_called_once();self.assertFalse(e.adapters)
        l.release.assert_not_called();e.lease.release.assert_called_once()
        self.assertEqual(e.phase,'stopped')

    def test_lost_planning_does_not_hide_a_cleanup_failure_or_start_a_new_run(self):
        from scen_runner.orchestration.control import PlanningUnavailable
        e,l,r=self.engine('section');e.begin(1);self.prepare(e)
        e.planning=Mock();e.planning.stop.side_effect=PlanningUnavailable('supervisor exited first')
        e.adapters[0].cleanup.side_effect=RuntimeError('alias still exists')
        l.release.reset_mock();l.reset.reset_mock()
        with self.assertRaisesRegex(RuntimeError,'alias still exists'):e.stop()
        self.assertTrue(e.adapters);l.release.assert_not_called();l.reset.assert_not_called()

    def test_cross_section_id_reuse_forgets_old_owner_without_deleting_new_actor(self):
        e,l,r=self.engine();a=FakeSection('roundabout');b=FakeSection('high_speed')
        a.values={'3':('old',15)};e.adapters=[a];e.refresh_owned()
        b.values={'3':('new',80)};e.adapters.append(b);e.refresh_owned()
        self.assertFalse(a.owned());self.assertEqual(e.owned['3'],('new',80));l.destroy.assert_not_called()

    def test_ego_reset_rpc_order_and_pose_are_explicit(self):
        link=EpisodeLink(load_config());stub=Mock()
        for name in ['SetPause','SetVelocity','SetVehicleGear','SetTransform']:
            getattr(stub,name).return_value=S(status=1)
        link._sa=S(_actor_stub=stub)
        pose=dict(x=1,y=2,z=3,roll=.1,pitch=.2,yaw=62.515)
        stub.GetActorState.return_value=S(actor_info=S(id=S(value='Ego')),transform=S(rotation=S(x=.1,y=.2,z=62.515)))
        link.reset(pose)
        names=[call[0] for call in stub.method_calls]
        self.assertEqual(names,['SetPause','SetVelocity','SetVehicleGear','SetTransform','SetPause','SetVelocity','SetVehicleGear','GetActorState'])
        for call in stub.SetPause.call_args_list:self.assertTrue(call.args[0].enable)
        request=stub.SetTransform.call_args.args[0]
        self.assertEqual(request.actor_info.id.value,'Ego');self.assertAlmostEqual(request.transform.rotation.z,62.515)
        link.release();self.assertFalse(stub.SetPause.call_args.args[0].enable)
        self.assertEqual(stub.SetVehicleGear.call_args.args[0].gear,4)

    def test_reset_corrects_reported_chassis_rotation_offset(self):
        link=EpisodeLink(load_config());stub=Mock()
        for name in ['SetPause','SetVelocity','SetVehicleGear','SetTransform']:getattr(stub,name).return_value=S(status=1)
        def state(yaw):return S(actor_info=S(id=S(value='Ego')),transform=S(rotation=S(x=0.,y=0.,z=yaw)))
        stub.GetActorState.side_effect=[state(63.862),state(62.515)]
        link._sa=S(_actor_stub=stub)
        link.reset(dict(x=1,y=2,z=3,roll=0.,pitch=0.,yaw=62.515))
        self.assertEqual(stub.SetTransform.call_count,2)
        self.assertAlmostEqual(stub.SetTransform.call_args.args[0].transform.rotation.z,61.168,places=3)

    def test_portable_sample_start_and_section_end_detection(self):
        route=EpisodeRoute();self.assertAlmostEqual(route.scene_start['x'],-131.48599243164062)
        self.assertAlmostEqual(route.scene_start['yaw'],62.515)
        for section in route.cfg['sections']:
            d=route.section(section)
            x,y=d['end_xy'];state=S(transform=S(location=S(x=x,y=y)))
            self.assertTrue(route.finished(section,state))
            pose=d['pose'];state=S(transform=S(location=S(x=pose['x'],y=pose['y'])))
            self.assertFalse(route.finished(section,state))


if __name__=='__main__':unittest.main()
