"""Save-before-reset, generation reuse and deterministic native replay contracts."""
import copy
import csv
import hashlib
import json
import os
import sys
import tempfile
import unittest
import numpy as np
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.records.collision_archive import CollisionArchive,load_collision,actor_state,provenance,SCHEMA,STAGNATION_SCHEMA,VERSION
from scen_runner.records.collision_replay import ReplaySection,ReplayEnvironment,crossing_plan,configure_replay
from scen_runner.runtime.episode_actors import Section
from scen_runner.config import load_yaml,TOOL_ROOT,repo_path
from scen_runner.planning.traffic_geometry import load_links


def body(aid,x=0.,kind=3,speed=0.,link='a'):
    return S(actor_info=S(id=S(value=aid),object_type=kind),
        transform=S(location=S(x=x,y=0.,z=1.),rotation=S(x=0.,y=0.,z=90.)),
        velocity=S(x=speed,y=0.,z=0.),bounding_box=S(location=S(x=x,y=0.,z=1.),extent=S(x=1.,y=2.,z=1.)),
        vehicle_state=S(throttle=.2,brake=0.,steer=0.,current_link_info=S(id=S(value=link)),
                        remaining_distance=20.,collision_objects=[]))


def static_definition():
    return dict(type='static',model='Bollard01',pose=dict(x=20.,y=0.,z=1.,yaw=90.,roll=0.,pitch=0.),object={})


def crossing_definition():
    return dict(type='crossing',model='Deer1',group='Animal',cfg=dict(walk_speed_kmh=17,
        activation_distance_m=16.,spawn_height_offset_m=0.),start=[-6.,0.,1.],end=[6.,0.,1.],
        heading_deg=0.,move_distance=12.,direction='가로',timing={})


class ArchiveTest(unittest.TestCase):
    def test_trial_checkpoint_is_replayable_and_does_not_consume_later_collision(self):
        self.now[0]=2.
        self.assertTrue(self.archive.checkpoint(self.engine,'ready'))
        saved=self.finish()
        self.assertEqual(saved['schema'],'asmc.section_trial')
        self.assertNotIn('collision',saved)
        self.assertEqual(saved['trial']['reason'],'ready')
        selected,_=load_collision(self.archive.last_file,check_maps=False)
        self.assertEqual(selected['event_type'],'trial')
        self.assertEqual(selected['section'],'static')
        self.assertFalse(self.archive.completed)
        self.archive.collision(self.engine,{},self.sample(),{'1'})
        self.assertEqual(self.finish()['schema'],SCHEMA)
        self.assertEqual(self.archive.saved,2)

    def test_section_signal_policy_is_saved_and_restored_in_replay(self):
        settings=dict(policy='green',colors={'C1256W000091':48})
        self.engine.signals=S(snapshot=lambda:copy.deepcopy(settings))
        self.archive.entries.clear()
        self.archive.observe(self.engine,self.states)
        self.archive.collision(self.engine,{},self.sample(),{'1'})
        settings['colors'].clear()
        self.finish()
        selected,_=load_collision(self.archive.last_file,check_maps=False)
        self.assertEqual(selected['traffic_lights'],dict(policy='green',colors={'C1256W000091':48}))
        replay_engine=Mock()
        replay_engine.route.section.return_value={}
        configure_replay(replay_engine,selected)
        self.assertEqual(replay_engine.signal_replay,selected['traffic_lights'])
        selected.pop('traffic_lights')
        configure_replay(replay_engine,selected)
        self.assertEqual(replay_engine.signal_replay,dict(policy='unchanged',colors={}))

    def test_legacy_link_map_record_resolves_new_file_and_still_checks_geometry_hash(self):
        self.archive.collision(self.engine,{},self.sample(),{'1'})
        saved=self.finish()
        legacy='src/behavior_planner/src/config/link_set.json'
        current='src/asmc_planner/config/behavior/link_set.json'
        asset_root=Path(self.temp.name)/'map_repo'
        moved=asset_root/current;moved.parent.mkdir(parents=True);moved.write_bytes(b'original-geometry')
        saved['provenance']=dict(sha256={legacy:hashlib.sha256(moved.read_bytes()).hexdigest()})
        record_path=Path(self.temp.name)/'legacy_replay.json'
        record_path.write_text(json.dumps(saved))
        with patch('scen_runner.config.REPO_ROOT',asset_root):
            self.assertEqual(repo_path(legacy),moved)
            self.assertEqual(repo_path(str(asset_root/legacy)),moved)
            selected,_=load_collision(record_path)
            self.assertEqual(selected['section'],'static')
            moved.write_bytes(b'changed-geometry')
            with self.assertRaisesRegex(ValueError,'맵 파일'):
                load_collision(record_path)

    def test_external_absolute_map_path_keeps_its_own_repository(self):
        external=Path(self.temp.name)/'external/src/behavior_planner/src/config/link_set.json'
        self.assertEqual(repo_path(str(external)),external)

    def test_provenance_includes_nested_runtime_and_dll_sources(self):
        with patch('scen_runner.records.collision_archive.subprocess.check_output',side_effect=['fixture\n','']):
            metadata=provenance()
        hashes=metadata['sha256']
        for relative in ('scen_runner/ui/validation_gui.py','scen_runner/runtime/validation_episode.py',
                         'scen_runner/planning/static_plan.py','dll/src/PathCleanupPatch.cs'):
            self.assertIn('tools/scen_runner/'+relative,hashes)
        self.assertEqual(metadata['commit'],'fixture')

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.now=[0.];self.env=dict(weather='Sunny',hour=13,reason='씬 시작',error=None)
        link=Mock(actor_scopes={});link.ego_id='Ego'
        with patch('scen_runner.runtime.episode_actors.SceneLink',return_value=link):
            self.section=Section('static',77,'scope',{},lambda:False,lambda *_:None,clock=lambda:self.now[0])
        self.actor=self.section.register('alias1',3,'Bollard01',native='1',replay=static_definition())
        route=S(curve=S(project=lambda x,y:(x,0.)),cfg=load_yaml(TOOL_ROOT/'config/validation.yaml'),
            section=lambda name:dict(start=0.,end=100.,pose=dict(x=0.,y=0.,z=1.,roll=0.,pitch=0.,yaw=90.)))
        self.engine=S(phase='running',episode=1,mode='section',section='static',seed=42,run_started=0.,
            clock=lambda:self.now[0],adapters=[self.section],link=link,route=route,id_owners={'1':self.section},
            environment=S(snapshot=lambda:dict(self.env),snapshot_at=lambda now:dict(self.env)))
        self.archive=CollisionArchive(self.temp.name,notify=lambda *_:None,metadata={})
        self.archive.start();self.addCleanup(self.archive.close)
        self.states=[body('Ego',kind=1,speed=25.),body('1',20.)]
        self.archive.begin(self.engine,self.states)

    def sample(self,now=4.):
        return dict(received_at=now,received_at_utc='2026-10-11T06:00:00+00:00',query_s=.002,
                    actor_states=self.states,native_timestamp=123,native_tick_count=10)

    def test_lifecycle_and_ignored_contacts_are_saved_without_changing_collision_sequence(self):
        self.archive.event(dict(event='episode_begin',episode=2,owned_ids=['2']))
        self.archive.event(dict(event='contact_changed',raw_contacts=['1'],
                               records=[dict(id='1',verdict='ignored',unowned=True)]))
        self.archive.jobs.join()
        records=[json.loads(line) for line in (self.archive.folder/'events.jsonl').read_text().splitlines()]
        self.assertEqual([r['event'] for r in records],['episode_begin','contact_changed'])
        self.assertEqual(records[1]['records'][0]['verdict'],'ignored')
        self.assertIn('kst',records[0]['time']);self.assertEqual(self.archive.saved,0)
        self.assertEqual(self.archive.sequence,0)

    def finish(self):
        self.archive.jobs.join()
        return json.loads(Path(self.archive.last_file).read_text())

    def test_run_folder_atomic_collision_file_and_summary_survive_reset_and_deduplicate(self):
        self.archive.collision(self.engine,dict(raw_contacts=['1']),self.sample(),{'1'})
        self.archive.collision(self.engine,dict(raw_contacts=['1']),self.sample(),{'1'})
        self.section.replay_history[0]['definition']['pose']['x']=999.
        self.states[0].transform.location.x=999.
        self.engine.episode=2;self.archive.begin(self.engine,[])
        saved=self.finish()
        self.assertRegex(self.archive.folder.name,r'^\d{8}_\d{6}(?:_\d+)?$')
        self.assertEqual(Path(self.archive.last_file).name,'001_static.json')
        self.assertEqual(saved['episode'],1);self.assertEqual(saved['collision']['actors']['Ego']['pose']['x'],0.)
        self.assertEqual(saved['replay']['static']['baseline'][0]['record']['definition']['pose']['x'],20.)
        self.assertEqual(saved['collision']['time']['kst'],'2026-10-11T15:00:00+09:00')
        self.assertEqual(self.archive.saved,1)
        files=list(self.archive.folder.glob('*.json'));self.assertEqual(len(files),2)  # run + one collision
        self.assertFalse(list(self.archive.folder.glob('*.tmp')))
        with (self.archive.folder/'summary.csv').open(encoding='utf-8-sig',newline='') as stream:
            self.assertEqual(len(list(csv.reader(stream))),2)
        selected,_=load_collision(self.archive.last_file,check_maps=False)
        self.assertEqual(selected['section'],'static');self.assertEqual(selected['ego_at_entry']['speed_kmh'],25.)

    def test_stagnation_is_distinct_replayable_and_saved_before_reset(self):
        self.now[0]=30.
        evidence=dict(detected_at=30.,duration_s=30.,position_radius_m=1.,maximum_displacement_m=.1)
        self.assertTrue(self.archive.stagnation(self.engine,evidence,self.states))
        self.assertFalse(self.archive.stagnation(self.engine,evidence,self.states))
        self.section.replay_history[0]['definition']['pose']['x']=999.
        self.states[0].transform.location.x=999.
        saved=self.finish()
        self.assertEqual(saved['schema'],STAGNATION_SCHEMA);self.assertNotIn('collision',saved)
        self.assertEqual(Path(self.archive.last_file).name,'001_stagnation_static.json')
        self.assertEqual(saved['stagnation']['actors']['Ego']['pose']['x'],0.)
        self.assertEqual(saved['replay']['static']['baseline'][0]['record']['definition']['pose']['x'],20.)
        selected,_=load_collision(self.archive.last_file,check_maps=False)
        self.assertEqual(selected['event_type'],'stagnation');self.assertEqual(selected['section'],'static')
        with (self.archive.folder/'summary.csv').open(encoding='utf-8-sig') as stream:
            rows=list(csv.DictReader(stream))
        self.assertEqual(rows[0]['event_type'],'stagnation')

    def test_stagnation_before_section_records_observed_snapshot_with_explicit_resolution(self):
        self.engine.mode='scene';self.archive.entries.clear()
        self.engine.route.curve.project=lambda x,y:(-20.,0.)
        self.states[0].transform.location.x=-20.;self.now[0]=30.
        self.archive.stagnation(self.engine,dict(detected_at=30.,duration_s=30.),self.states)
        saved=self.finish()
        self.assertEqual(saved['stagnation']['section_resolution'],'outside_section_upcoming')
        self.assertTrue(saved['replay_available'])
        self.assertEqual(saved['replay']['static']['start_pose']['x'],-20.)

    def test_stagnation_missing_object_state_saves_settings_without_inventing_replay(self):
        self.archive.entries.clear();self.now[0]=30.
        self.archive.stagnation(self.engine,dict(detected_at=30.,duration_s=30.),self.states[:1])
        saved=self.finish();self.assertFalse(saved['replay_available'])
        self.assertEqual(saved['all_section_settings'][0]['model'],'Bollard01')

    def test_same_second_runs_get_short_suffix_and_collision_files_are_sequential(self):
        stamp=dict(utc='2026-10-11T08:00:00+00:00',kst='2026-10-11T17:00:00+09:00')
        with patch('scen_runner.records.collision_archive.wall_times',return_value=stamp):
            first=CollisionArchive(self.temp.name,notify=lambda *_:None,metadata={})
            second=CollisionArchive(self.temp.name,notify=lambda *_:None,metadata={})
            first.start();self.addCleanup(first.close)
            second.start();self.addCleanup(second.close)
        self.assertEqual(first.folder.name,'20261011_170000')
        self.assertEqual(second.folder.name,'20261011_170000_2')
        self.assertNotEqual(first.run_id,second.run_id)
        self.archive.collision(self.engine,{},self.sample(),{'1'});self.finish()
        self.engine.episode=2;self.archive.begin(self.engine,self.states)
        self.archive.collision(self.engine,{},self.sample(),{'1'});self.finish()
        self.assertEqual(Path(self.archive.last_file).name,'002_static.json')
        self.assertTrue((self.archive.folder/'001_static.json').is_file())

    def test_late_visible_birth_and_owned_generation_removal_are_saved(self):
        self.now[0]=2.
        new=self.section.register('alias2',3,'Bollard01',native='2',replay=static_definition())
        self.engine.id_owners['2']=self.section
        self.archive.observe(self.engine,self.states)
        self.assertEqual(self.archive.entries['static']['events'][0]['time_s'],2.)
        self.now[0]=3.;self.archive.observe(self.engine,self.states+[body('2',40.)])
        self.assertEqual(self.archive.entries['static']['events'][0]['action'],'spawn')
        # Missing one state is not evidence that an owned actor was deleted.
        self.now[0]=4.;self.archive.observe(self.engine,self.states)
        self.assertEqual(len(self.archive.entries['static']['events']),1)
        self.section.actors.remove(new);self.engine.id_owners.pop('2')
        self.now[0]=5.;self.archive.observe(self.engine,self.states)
        events=self.archive.entries['static']['events']
        self.assertEqual([e['action'] for e in events],['spawn','remove'])
        self.assertEqual(events[0]['record']['key'],events[1]['key'])

    def test_callback_delay_does_not_save_future_weather_or_reused_id_definition(self):
        old_key=self.actor.replay_key
        self.section.actors=[];self.now[0]=6.
        newer=self.section.register('new1',3,'new-model',native='1',replay=dict(static_definition(),model='new-model'))
        self.env=dict(weather='Foggy',hour=15,reason='변경',error=None)
        self.engine.environment.snapshot_at=lambda now:dict(weather='Sunny',hour=13,reason='이전',error=None)
        self.archive.observe(self.engine,self.states)
        self.archive.collision(self.engine,{},self.sample(4.),{'1'});saved=self.finish()
        self.assertEqual(saved['collision']['environment']['weather'],'Sunny')
        self.assertEqual([r['key'] for r in saved['all_section_settings']],[old_key])
        self.assertFalse(saved['replay']['static']['weather_events'])
        self.assertFalse(saved['replay']['static']['events'])

    def test_writer_failure_is_reported_and_does_not_claim_success(self):
        with patch('scen_runner.records.collision_archive.json.dumps',side_effect=OSError('disk full')):
            self.archive.collision(self.engine,{},self.sample(),{'1'});self.archive.jobs.join()
        self.assertEqual(self.archive.saved,0);self.assertIn('disk full',self.archive.error)
        self.assertFalse(list(self.archive.folder.glob('*.tmp')))

    def test_numpy_tunnel_timing_in_other_section_cannot_break_collision_save(self):
        self.section.replay_history[0]['definition']['timing']=dict(
            stop_timing_feasible=np.bool_(False),nominal_conflict=np.bool_(True),
            pattern=np.int64(1),gap=np.float32(2.5),points=np.array([1.,2.]))
        self.archive.records.clear();self.archive.observe(self.engine,self.states)
        self.archive.collision(self.engine,{},self.sample(),{'1'});saved=self.finish()
        timing=saved['all_section_settings'][0]['definition']['timing']
        self.assertIs(timing['stop_timing_feasible'],False);self.assertIs(timing['nominal_conflict'],True)
        self.assertEqual(timing['pattern'],1);self.assertEqual(timing['points'],[1.,2.])

    def test_missing_section_entry_is_not_replaced_with_an_overlapping_impact_pose(self):
        self.archive.entries.clear()
        self.archive.collision(self.engine,{},self.sample(),{'1'});saved=self.finish()
        self.assertFalse(saved['replay_available'])
        with self.assertRaises(ValueError):load_collision(self.archive.last_file,check_maps=False)

    def test_same_numeric_id_has_distinct_generation_keys(self):
        key=self.actor.replay_key
        self.section.forget_reused_id('1');self.now[0]=2.
        replacement=self.section.register('new',3,'replacement',native='1',replay=static_definition())
        self.archive.observe(self.engine,self.states)
        self.assertNotEqual(key,replacement.replay_key)
        self.assertEqual(len(self.archive.records),2)

    def test_history_keeps_independent_values_when_native_state_objects_are_reused(self):
        key=self.actor.replay_key
        for index in range(1,31):
            self.now[0]=index/5
            self.states[1].transform.location.x=20.+index
            self.states[1].velocity.x=float(index)
            self.states[1].bounding_box.location.x=20.+index
            self.states[1].vehicle_state.collision_objects[:]=[str(index)]
            self.archive.observe(self.engine,self.states)
        self.assertEqual(len(self.archive.history),25)
        self.archive.collision(self.engine,{},self.sample(6.),{'1'})
        # Both native states and a new observation can change before file I/O.
        self.states[1].transform.location.x=999.
        self.states[1].vehicle_state.collision_objects.clear()
        self.now[0]=7.;self.archive.observe(self.engine,self.states)
        saved=self.finish()
        history=saved['history']
        self.assertEqual(len(history),25)
        for index,row in zip(range(6,31),history):
            state=row['actors'][key]
            self.assertEqual(state['pose']['x'],20.+index)
            self.assertEqual(state['box_center'][0],20.+index)
            self.assertEqual(state['velocity'],[float(index),0.,0.])
            self.assertEqual(state['native_contacts'],[str(index)])
        self.assertEqual(saved['replay']['static']['baseline'][0]['state']['pose']['x'],20.)

    def test_missing_entry_is_retried_then_retained_without_reprojecting(self):
        self.engine.mode='scene';self.archive.entries.clear()
        project=Mock(return_value=(200.,0.));self.engine.route.curve.project=project
        self.archive.observe(self.engine,self.states)
        self.assertFalse(self.archive.entries)
        project.return_value=(30.,0.)
        self.archive.observe(self.engine,self.states[:1])  # Managed object absent from this sample.
        self.assertFalse(self.archive.entries)
        self.now[0]=1.;self.archive.observe(self.engine,self.states)
        self.assertEqual(self.archive.entries['static']['entry_elapsed_s'],1.)
        project.reset_mock();project.return_value=(200.,0.)
        self.env['weather']='Foggy';self.now[0]=2.
        self.archive.observe(self.engine,self.states)
        project.assert_not_called()
        self.assertEqual(self.archive.entries['static']['entry_elapsed_s'],1.)
        self.assertEqual(self.archive.entries['static']['weather_events'][0]['time_s'],1.)


@unittest.skipUnless(os.geteuid()==0,'real Docker-root ownership check')
class ArchiveOwnershipTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.shared=Path(self.temp.name)/'shared';self.shared.mkdir()
        self.owner=(30001,30001)
        os.chown(self.shared,*self.owner)
        self.unrelated=self.shared/'existing-other-log';self.unrelated.write_text('keep')

    def save(self,root=None):
        archive=CollisionArchive(root,notify=lambda *_:None,metadata={});archive.start()
        try:
            archive.jobs.put(dict(episode=1,seed=42,collision=dict(
                time=dict(kst='2026-10-11T17:00:00+09:00'),sections=['static'],
                counterpart_ids=['1'],environment=dict(weather='Sunny',hour=11))))
            archive.jobs.join()
            self.assertIsNone(archive.error);self.assertEqual(archive.saved,1)
            for path in [archive.folder,*archive.folder.iterdir()]:
                self.assertEqual((path.stat().st_uid,path.stat().st_gid),self.owner,str(path))
                self.assertFalse(path.stat().st_mode & 0o002)  # No world-write permission workaround.
            self.assertEqual(self.unrelated.read_text(),'keep')
            self.assertEqual(self.unrelated.stat().st_uid,0)  # Other existing files are untouched.
            return archive
        finally:archive.close()

    def test_nested_custom_save_directory_and_all_files_inherit_shared_parent_owner(self):
        root=self.shared/'new-parent'/'logs'
        self.save(root)
        for path in [root,root.parent]:
            self.assertEqual((path.stat().st_uid,path.stat().st_gid),self.owner)

    def test_default_save_root_previously_created_by_root_uses_workspace_owner(self):
        root=self.shared/'save_collision_log';root.mkdir()
        self.assertEqual(root.stat().st_uid,0)
        with patch('scen_runner.records.collision_archive.TOOL_ROOT',self.shared),patch(
                'scen_runner.records.collision_archive.SAVE_ROOT',root):self.save()
        self.assertEqual((root.stat().st_uid,root.stat().st_gid),self.owner)


class ReplayTest(unittest.TestCase):
    def section(self,items,events=()):
        self.now=[0.];link=Mock(actor_scopes={});link.ego_id='Ego';link.states.return_value=[]
        definition=dict(section='static',seed=77,baseline=items,events=list(events),environment=dict(weather='Sunny',hour=13))
        with patch('scen_runner.runtime.episode_actors.SceneLink',return_value=link):
            section=ReplaySection('static',77,'new-scope',{},lambda:False,lambda *_:None,
                                 clock=lambda:self.now[0],definition=definition)
        return section,link

    def item(self,key='old',kind=3,definition=None,state=None):
        return dict(record=dict(key=key,native_id='1',kind=kind,model='model',target_kmh=17.,source='',
                                definition=definition or static_definition()),**({'state':state} if state else {}))

    def test_saved_static_values_used_without_rng_and_fresh_id_owned(self):
        item=self.item(state=actor_state(body('1',20.)));section,link=self.section([item])
        link.spawn_obstacle.return_value='fresh-alias';link.static_native_id.return_value='91'
        with patch('random.Random.choice',side_effect=AssertionError('no random replay')):
            section.poll_prepare()
            link.states.return_value=[body('91',20.)];self.assertTrue(section.poll_prepare())
        link.spawn_obstacle.assert_called_once_with('Bollard01',20.,0.,1.,90.,'collision_replay')
        self.assertEqual(set(section.owned()),{'91'});self.assertNotIn('1',section.owned())

    def test_baseline_and_future_crossing_routes_registered_once_after_live_state(self):
        first=self.item('baseline',2,crossing_definition())
        future=self.item('future',2,crossing_definition())['record']
        section,link=self.section([first],[dict(action='spawn',time_s=2.,record=future)])
        link.spawn_crossing.side_effect=['9','10']
        section.poll_prepare();link.states.return_value=[body('9',kind=2)]
        self.assertTrue(section.poll_prepare());link.register_crossing.assert_not_called()
        section.resume_schedule(0.);self.assertEqual(link.register_crossing.call_count,1)
        self.now[0]=2.;section.tick();self.assertEqual(link.spawn_crossing.call_count,2)
        link.states.return_value=[body('9',kind=2),body('10',kind=2)]
        section.tick();section.tick();self.assertEqual(link.register_crossing.call_count,2)

    def test_saved_crossing_progress_and_completed_endpoint(self):
        d=crossing_definition();p=crossing_plan(d,actor_state(body('1',0.,kind=2)))
        self.assertEqual(p.start,[0.,0.,1.]);self.assertTrue(p.replay_started)
        self.assertEqual(p.cfg['activation_distance_m'],16.)
        p=crossing_plan(d,actor_state(body('1',6.,kind=2)))
        self.assertTrue(p.replay_finished);self.assertFalse(p.replay_started)
        self.assertEqual(d['start'],[-6.,0.,1.])

    def test_native_ticket_custom_target_and_actual_entry_speed_are_preserved(self):
        cfg=load_yaml(TOOL_ROOT/'config/roundabout.yaml');links=load_links(repo_path(cfg['link_map']))
        current=cfg['npc_links'][1];p=links[current]['points'][2]
        state=actor_state(body('1',kind=1,speed=58.,link=current));state['pose'].update(x=p[0],y=p[1],z=p[2])
        d=dict(type='vehicle',method='native',model='2015_Kia_K5',target_kmh=78.,cfg=cfg,source='6',period_s=8.)
        item=self.item('native',1,d,state);section,link=self.section([item]);ticket=dict(info=S(client_key='fresh-source'),actor=None)
        with patch('scen_runner.records.collision_replay.create_ticket',return_value=ticket) as create,patch(
             'scen_runner.records.collision_replay.enable_ticket'),patch('scen_runner.records.collision_replay.ticket_candidates',return_value=[body('99',kind=1)]):
            section.poll_prepare();self.assertEqual(create.call_args.args[3],78.)
            plan=create.call_args.args[1].plan
            self.assertEqual(plan.cfg['initial_speed_kmh'],58.);self.assertEqual(plan.cfg['npc_links'][0],current)
            section.poll_prepare()
        link.set_speed_limit.assert_called_once_with('99',78.)
        link.restore_actor.assert_called_once_with('99',1,state)
        link.pause_actor.assert_called_with('99',1,True)

    def test_reused_live_id_cannot_be_deleted_by_old_generation_event(self):
        section,link=self.section([])
        d=dict(type='vehicle',method='direct',target_kmh=17.,cfg={})
        old=section.register_replay(self.item('old',1,d)['record'],'5')
        new=section.register_replay(self.item('new',1,d)['record'],'5')
        self.assertNotIn('old',section.by_key);self.assertEqual(section.actors,[new])
        self.assertIsNot(old,new)

    def test_environment_timeline_repeats_saved_choices(self):
        clock=[0.];link=Mock()
        d=dict(environment=dict(weather='Sunny',hour=13),weather_events=[dict(time_s=2.,environment=dict(weather='Foggy',hour=15))])
        e=ReplayEnvironment(link,d,lambda:clock[0]);e.begin();e.update(None,0.)
        clock[0]=3.;e.update(None,3.)
        self.assertEqual(e.snapshot_at(1.)['weather'],'Sunny')
        self.assertEqual(e.snapshot()['weather'],'Foggy');e.begin();self.assertEqual(e.snapshot()['weather'],'Sunny')


if __name__=='__main__':unittest.main()
