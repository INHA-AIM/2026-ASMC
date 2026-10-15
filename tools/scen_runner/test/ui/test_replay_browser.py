"""Saved run selection, chronological navigation and safe worker transitions."""
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.records.collision_archive import SCHEMA,STAGNATION_SCHEMA,VERSION
from scen_runner.config import TOOL_ROOT,load_yaml
from scen_runner.records.replay_browser import saved_runs
from scen_runner.ui.validation_gui import ValidationRunner,ValidationWindow


def document(section='static',time='2026-10-11T17:01:00+09:00',start='2026-10-11T17:00:00+09:00'):
    pose=dict(x=0.,y=0.,z=1.,roll=0.,pitch=0.,yaw=90.)
    model='Bollard01' if section=='static' else 'Dog1'
    actor=dict(key='old',native_id='1',section=section,model=model,
               definition=dict(type='static',model=model,pose=pose))
    entry=dict(section=section,seed=42,start_pose=pose,
        route_config=load_yaml(TOOL_ROOT/'config/validation.yaml'),ego_at_entry=dict(speed_kmh=63.),
        baseline=[dict(record=actor)],environment=dict(weather='Sunny',hour=15),events=[])
    return dict(schema=SCHEMA,version=VERSION,run_started=dict(kst=start),episode=1,
        replay_available=True,replay={section:entry},all_section_settings=[actor],
        collision=dict(time=dict(kst=time),counterpart_ids=['1'],environment=dict(weather='Sunny',hour=15)))


class ReplayBrowserTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.run=self.root/'20261011_170000_old_uuid';self.run.mkdir()
        self.write(self.run/'z_old_episode0001_static.json',document())
        self.write(self.run/'001_dynamic.json',document('dynamic','2026-10-11T17:02:00+09:00'))

    @staticmethod
    def write(path,data):path.write_text(json.dumps(data),encoding='utf-8')

    def test_old_and_short_names_sort_by_collision_time_and_empty_newer_runs_are_skipped(self):
        empty=self.root/'20261011_180000';empty.mkdir();self.write(empty/'run.json',{})
        runs,warnings=saved_runs(self.root)
        self.assertEqual(len(runs),1);self.assertFalse(warnings)
        self.assertEqual([c.section for c in runs[0].cases],['static','dynamic'])
        self.assertEqual(runs[0].cases[0].models,'Bollard01')
        self.assertEqual(runs[0].cases[0].load()['ego_at_entry']['speed_kmh'],63.)
        self.assertEqual(runs[0].label,'2026-10-11 17:00:00 · 충돌 2건')

    def test_stagnation_records_mix_with_collisions_and_are_labelled_separately(self):
        saved=document(time='2026-10-11T17:03:00+09:00')
        saved['schema']=STAGNATION_SCHEMA;saved['event_type']='stagnation'
        saved['stagnation']=saved.pop('collision');saved['stagnation'].pop('counterpart_ids')
        self.write(self.run/'003_stagnation_static.json',saved)
        runs,warnings=saved_runs(self.root)
        self.assertFalse(warnings);self.assertEqual(len(runs[0].cases),3)
        case=runs[0].cases[-1]
        self.assertEqual(case.event_type,'stagnation');self.assertEqual(case.models,'Bollard01')
        self.assertEqual(case.load()['event_type'],'stagnation')
        self.assertEqual(runs[0].label,'2026-10-11 17:00:00 · 충돌 2건 · 정체 1건')

    def test_default_run_uses_saved_start_time_rather_than_folder_spelling(self):
        newer=self.root/'another_name';newer.mkdir()
        self.write(newer/'001_static.json',document(start='2026-10-11T18:00:00+09:00'))
        runs,_=saved_runs(self.root)
        self.assertEqual(runs[0].folder,newer)

    def test_invalid_and_unavailable_records_do_not_hide_valid_saved_collisions(self):
        (self.run/'broken.json').write_text('{broken',encoding='utf-8')
        (self.run/'unexpected.json').write_text('[]',encoding='utf-8')
        unavailable=document();unavailable['replay_available']=False
        self.write(self.run/'unavailable.json',unavailable)
        runs,warnings=saved_runs(self.root)
        self.assertEqual(len(runs[0].cases),2);self.assertEqual(len(warnings),2)
        with self.assertRaisesRegex(ValueError,'충돌 기록'):
            saved_runs(self.root/'absent')

    def runner(self):
        runs,_=saved_runs(self.root)
        runner=ValidationRunner('section','static',42,save_collisions=False,replay_runs=runs)
        self.assertEqual(runner.replay['section'],'static')
        runner.engine.lease=Mock(held=True)
        runner.link.hold=Mock();runner.engine.cleanup=Mock();runner.engine.begin=Mock()
        runner.monitor.reset_context=Mock(return_value=2)
        return runner

    def test_worker_cleans_old_actors_before_selecting_next_and_uses_saved_definition(self):
        runner=self.runner();order=[]
        runner.link.hold.side_effect=lambda:order.append('hold')
        runner.engine.cleanup.side_effect=lambda:order.append('cleanup')
        runner.engine.begin.side_effect=lambda epoch:order.append('begin')
        original_configure=__import__('scen_runner.records.collision_replay',fromlist=['configure_replay']).configure_replay
        def configure(engine,definition):
            order.append('configure');original_configure(engine,definition)
        with patch('scen_runner.ui.validation_gui.configure_replay',side_effect=configure):runner.select_replay(0,1)
        self.assertEqual(order,['hold','cleanup','configure','begin'])
        self.assertEqual(runner.engine.section,'dynamic');self.assertEqual(runner.replay_case_index,1)
        self.assertEqual(runner.replay_case_status()['models'],'Dog1')
        self.assertEqual(runner.engine.replay_seed,42)
        runner.engine.begin.assert_called_once_with(2)

    def test_bad_map_is_rejected_before_holding_or_cleaning_active_scene(self):
        runner=self.runner();doc=document('dynamic')
        map_file=load_yaml(TOOL_ROOT/'config/static_obstacle.yaml')['map']['drivable_map']
        doc['provenance']=dict(sha256={map_file:'bad hash'})
        self.write(runner.replay_runs[0].cases[1].path,doc)
        with self.assertRaisesRegex(ValueError,'맵 파일'):runner.select_replay(0,1)
        runner.link.hold.assert_not_called();runner.engine.cleanup.assert_not_called()
        self.assertEqual(runner.replay_case_index,0)

    def test_collision_or_end_waits_for_button_instead_of_automatically_repeating(self):
        for reason in ('collision','section_end'):
            with self.subTest(reason=reason):
                runner=self.runner();runner.link._sa=Mock()  # Connected fake; no simulator access.
                runner.engine.begin.side_effect=lambda epoch:setattr(runner.engine,'phase','running')
                runner.engine.read_states=Mock();runner.engine.step=Mock(return_value=reason)
                runner.engine.stop=Mock();runner.monitor.start=Mock();runner.monitor.wait=Mock()
                snapshots=[]
                def snapshot():
                    snapshots.append(runner.engine.phase)
                    if len(snapshots)==3:runner.commands.put(('close',None,None))
                runner.snapshot=Mock(side_effect=snapshot)
                runner.run()
                runner.engine.begin.assert_called_once()
                runner.engine.step.assert_called_once()
                self.assertEqual(runner.engine.phase,'replay_done')
                self.assertEqual(runner.replay_done_reason,reason)

    def test_gui_next_previous_and_run_selector_send_worker_commands_only(self):
        from PyQt5 import QtWidgets
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        other=self.root/'20261011_160000';other.mkdir()
        self.write(other/'001_static.json',document(start='2026-10-11T16:00:00+09:00'))
        runs,_=saved_runs(self.root)
        with patch.object(ValidationRunner,'start'):
            window=ValidationWindow(save_collisions=False,replay_runs=runs)
        try:
            self.assertFalse(window.replay_previous.isEnabled());self.assertTrue(window.replay_next.isEnabled())
            self.assertFalse(window.runner.link.connected)
            window.replay_next.click()
            self.assertEqual(window.runner.commands.get_nowait(),('replay',0,1))
            window.runner.replay_case_index=1;window.runner.engine.section='dynamic'
            window.runner.engine.pose=window.runner.route.start('section','dynamic')
            window.runner.engine.names=['dynamic'];window.runner.snapshot();app.processEvents();window.flush()
            self.assertIn('2/2',window.header.text());self.assertIn('Dog1',window.replay_info.text())
            self.assertEqual(window.section.currentData(),'dynamic')
            self.assertFalse(window.replay_next.isEnabled());self.assertTrue(window.replay_previous.isEnabled())
            window.replay_previous.click()
            self.assertEqual(window.runner.commands.get_nowait(),('replay',0,0))
            window.replay_selector.setCurrentIndex(1)
            self.assertEqual(window.runner.commands.get_nowait(),('replay',1,0))
            self.assertFalse(window.runner.link.connected)
        finally:
            window.allow_close=True;window.timer.stop();window.close()

    def test_gui_stagnation_case_shows_event_type_and_disables_stagnation_auto_restart(self):
        from PyQt5 import QtWidgets
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        saved=document(time='2026-10-11T18:01:00+09:00',start='2026-10-11T18:00:00+09:00')
        saved['schema']=STAGNATION_SCHEMA;saved['event_type']='stagnation'
        saved['stagnation']=saved.pop('collision');saved['stagnation'].pop('counterpart_ids')
        only=self.root/'20261011_180000';only.mkdir()
        self.write(only/'001_stagnation_static.json',saved)
        runs,_=saved_runs(self.root)
        with patch.object(ValidationRunner,'start'):
            window=ValidationWindow(save_collisions=False,replay_runs=runs)
        try:
            window.runner.snapshot();app.processEvents();window.flush()
            self.assertIn('정체 재현',window.header.text())
            self.assertIn('원래 정체',window.replay_info.text())
            self.assertIsNone(window.runner.engine.stagnation)
            self.assertIsNone(window.runner.engine.run_budget)
            self.assertEqual(window.replay_next.text(),'다음 기록')
            self.assertFalse(window.runner.link.connected)
        finally:
            window.allow_close=True;window.timer.stop();window.close()


if __name__=='__main__':unittest.main()
