import sys
import unittest
import time
import tempfile
from pathlib import Path
from unittest.mock import patch,Mock
from types import SimpleNamespace as S
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.ui.validation_gui import ValidationWindow,ValidationRunner


class ValidationGuiTest(unittest.TestCase):
    def test_both_gui_start_buttons_submit_the_selected_section(self):
        from PyQt5 import QtWidgets
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with patch.object(ValidationRunner,'start'):
            window=ValidationWindow('section','static',42,save_collisions=False)
        try:
            window.mode.setCurrentIndex(window.mode.findData('section'))
            window.section.setCurrentIndex(window.section.findData('dynamic'))
            buttons={button.text():button for button in window.findChildren(QtWidgets.QPushButton)}
            buttons['다음 반복 · 다시 준비'].click()
            self.assertEqual(window.runner.commands.get_nowait(),('reset','section','dynamic'))
            window.section.setCurrentIndex(window.section.findData('roundabout'))
            buttons['선택 모드로 시작'].click()
            self.assertEqual(window.runner.commands.get_nowait(),('apply','section','roundabout'))
            self.assertIn('꺼짐',window.planning_status.text())
        finally:
            window.allow_close=True;window.timer.stop();window.close()

    def test_gui_apply_uses_common_restart_instead_of_cleaning_before_planning_stop(self):
        runner,link=self.stopped_runner()
        runner.engine.mode='section';runner.engine.section='static'
        def begin(*args,**kwargs):
            runner.engine.phase='running'
            runner.engine.cancelled.clear()
        runner.engine.begin.side_effect=begin;runner.engine.step=Mock(return_value=None)
        runner.engine.cleanup=Mock()
        runner.request('apply','section','dynamic')
        runner.snapshot.side_effect=lambda:runner.request('close')
        runner.run()
        self.assertEqual(runner.engine.begin.call_args.kwargs,
                         dict(selection=('section','dynamic'),reset_budget=True))
        runner.engine.cleanup.assert_not_called()
        link.hold.assert_not_called()

    def stopped_runner(self,connected=True):
        runner=ValidationRunner('scene','static',42,save_collisions=False)
        link=Mock(connected=connected);runner.link=runner.engine.link=link
        runner.engine.lease=Mock(held=False)
        runner.monitor.start=Mock();runner.monitor.wait=Mock()
        runner.snapshot=Mock();runner.engine.begin=Mock();runner.engine.stop=Mock()
        return runner,link

    def test_close_has_priority_over_reconnection_and_queued_mode_changes(self):
        runner,link=self.stopped_runner(False)
        runner.commands.put(('apply','section','dynamic'))
        runner.request('close');closed=[];runner.finished_clean.connect(lambda:closed.append(True))
        runner.run()
        link.connect.assert_not_called();runner.engine.begin.assert_not_called()
        runner.engine.stop.assert_called_once();self.assertEqual(closed,[True])

    def test_missing_planning_supervisor_does_not_trap_close_in_error_loop(self):
        from scen_runner.orchestration.control import PlanningUnavailable
        runner,link=self.stopped_runner()
        runner.engine.stop.side_effect=PlanningUnavailable('supervisor exited')
        closed=[];runner.finished_clean.connect(lambda:closed.append(True))
        runner.request('close');runner.run()
        self.assertEqual(closed,[True]);self.assertTrue(runner.closing)
        self.assertEqual(runner.engine.phase,'stopped')

    def test_close_arriving_inside_failed_connect_is_not_lost_or_retried(self):
        runner,link=self.stopped_runner(False)
        def connect():
            runner.request('close');raise ConnectionError('simulator exited during connection')
        link.connect.side_effect=connect;runner.archive=Mock()
        closed=[];runner.finished_clean.connect(lambda:closed.append(True))
        runner.run()
        link.connect.assert_called_once();runner.engine.stop.assert_not_called()
        link.hold.assert_not_called();link.release.assert_not_called()
        runner.archive.close.assert_called_once();self.assertEqual(closed,[True])
        self.assertEqual(runner.engine.phase,'stopped')

    def test_dead_simulator_cleanup_failure_still_flushes_logs_and_closes_all_links(self):
        import grpc
        class Unavailable(grpc.RpcError):
            def code(self):return grpc.StatusCode.UNAVAILABLE
        runner,link=self.stopped_runner()
        runner.engine.stop.side_effect=Unavailable('MORAI exited')
        section_link=Mock();runner.engine.adapters=[S(link=section_link)]
        runner.archive=Mock();runner.speed_trace=Mock();order=[]
        runner.archive.close.side_effect=lambda:order.append('saved')
        runner.finished_clean.connect(lambda:order.append('closed'))
        runner.request('close');runner.run()
        link.disconnect.assert_called_once();section_link.disconnect.assert_called_once()
        runner.engine.lease.release.assert_called_once();runner.speed_trace.close.assert_called_once()
        self.assertEqual(order,['saved','closed']);self.assertTrue(runner.closing)

    def test_semantic_cleanup_failure_remains_an_error_and_can_be_retried(self):
        runner,link=self.stopped_runner()
        runner.engine.stop.side_effect=RuntimeError('object deletion refused')
        phases=[]
        def snapshot():
            phases.append(runner.engine.phase)
            runner.engine.stop.side_effect=None;runner.request('close')
        runner.snapshot.side_effect=snapshot
        runner.request('close');runner.run()
        self.assertEqual(phases,['error']);self.assertEqual(runner.engine.stop.call_count,2)
        self.assertTrue(runner.closing)

    @staticmethod
    def count_file(folder,scene=1):
        path=Path(folder)/'counts.yaml'
        path.write_text(f'scene_runs: {scene}\nsection_runs: {{static: 1, dynamic: 2, roundabout: 3, high_speed: 4, tunnel: 5}}\n')
        return path

    def test_worker_done_batch_closes_automatically_without_releasing_returned_ego(self):
        from unittest.mock import Mock
        with tempfile.TemporaryDirectory() as folder:
            runner=ValidationRunner('scene','static',42,save_collisions=False,run_counts=self.count_file(folder))
        link=Mock(connected=True);link.states.return_value=[]
        runner.link=runner.engine.link=link
        runner.monitor.start=Mock();runner.monitor.wait=Mock()
        runner.engine.begin=Mock(side_effect=lambda *_:setattr(runner.engine,'phase','running'))
        def step():
            runner.engine.run_budget.start_run();runner.engine.phase='runs_done'
        runner.engine.step=Mock(side_effect=step);runner.engine.stop=Mock()
        runner.archive=Mock()
        order=[];runner.archive.close.side_effect=lambda:order.append('archive_flushed')
        closed=[];runner.finished_clean.connect(lambda:closed.append('closed'))
        runner.finished_clean.connect(lambda:order.append('window_close_requested'))
        frames=[]
        def snapshot():
            frames.append(runner.engine.phase)
        runner.snapshot=Mock(side_effect=snapshot);runner.run()
        runner.engine.begin.assert_called_once();runner.engine.step.assert_called_once()
        self.assertEqual(frames[-1],'runs_done');self.assertEqual(runner.engine.run_budget.started,1)
        runner.engine.stop.assert_not_called();link.release.assert_not_called()
        self.assertTrue(runner.closing);self.assertEqual(closed,['closed'])
        runner.archive.close.assert_called_once()
        self.assertEqual(order,['archive_flushed','window_close_requested'])

    def test_completion_signal_closes_gui_without_manual_stop_request(self):
        from PyQt5 import QtWidgets
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with patch.object(ValidationRunner,'start'):window=ValidationWindow('scene','static',42,save_collisions=False)
        try:
            window.show();app.processEvents()
            self.assertTrue(window.isVisible())
            window.runner.finished_clean.emit();app.processEvents()
            self.assertTrue(window.allow_close);self.assertFalse(window.isVisible())
            self.assertTrue(window.runner.commands.empty())
        finally:
            window.allow_close=True;window.timer.stop();window.close()

    def test_manual_next_within_batch_counts_toward_same_total(self):
        from unittest.mock import Mock
        with tempfile.TemporaryDirectory() as folder:
            runner=ValidationRunner('scene','static',42,save_collisions=False,run_counts=self.count_file(folder,2))
        link=Mock(connected=True);link.states.return_value=[]
        runner.link=runner.engine.link=link;runner.engine.lease=Mock(held=True)
        runner.monitor.start=Mock();runner.monitor.wait=Mock()
        def begin(*_):
            runner.engine.run_budget.start_run();runner.engine.phase='running';runner.engine.cancelled.clear()
        runner.engine.begin=Mock(side_effect=begin);runner.engine.step=Mock(return_value=None);runner.engine.stop=Mock()
        frames=[]
        def snapshot():
            frames.append(runner.engine.run_budget.started)
            if len(frames)==1:runner.request('reset')
            elif runner.engine.begin.call_count==2:runner.commands.put(('close',None,None))
        runner.snapshot=Mock(side_effect=snapshot);runner.run()
        self.assertEqual(runner.engine.begin.call_count,2)
        self.assertEqual(runner.engine.run_budget.started,2)

    def test_gui_displays_loaded_total_and_final_held_status(self):
        from PyQt5 import QtWidgets
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with tempfile.TemporaryDirectory() as folder,patch.object(ValidationRunner,'start'):
            window=ValidationWindow('section','dynamic',42,save_collisions=False,run_counts=self.count_file(folder))
        try:
            engine=window.runner.engine
            engine.run_budget.start_run();engine.phase='running'
            window.runner.snapshot();app.processEvents();window.flush()
            self.assertIn('주행 1/2',window.header.text())
            self.assertIn(f'정체 감시 0/{engine.stagnation.duration:g}초',window.status.text())
            engine.signals.settings=dict(policy='green',colors={'C1256W000016':48})
            window.runner.snapshot();app.processEvents();window.flush()
            self.assertIn('구간 신호: 직진·좌회전 초록 고정',window.environment_status.text())
            engine.run_budget.start_run();engine.phase='runs_done';engine.status='총 주행 2/2회 완료 · ego 정지'
            window.runner.snapshot();app.processEvents();window.flush()
            self.assertIn('주행 2/2',window.header.text());self.assertIn('ego 정지',window.status.text())
            self.assertNotIn('정체 감시',window.status.text())
        finally:
            window.allow_close=True;window.timer.stop();window.close()

    def test_worker_scene_finish_starts_next_random_episode_automatically(self):
        from unittest.mock import Mock
        runner=ValidationRunner('scene','static',42,save_collisions=False)
        link=Mock(connected=True);link.states.return_value=[]
        runner.link=runner.engine.link=link
        runner.monitor.start=Mock();runner.monitor.wait=Mock()
        runner.engine.begin=Mock(side_effect=lambda *_:setattr(runner.engine,'phase','running'))
        runner.engine.step=Mock(return_value='scene_end')
        runner.engine.stop=Mock()
        def snapshot():
            if runner.engine.begin.call_count==2:
                runner.commands.put(('close',None,None))
        runner.snapshot=Mock(side_effect=snapshot)
        runner.run()
        self.assertEqual(runner.engine.begin.call_count,2)
        runner.engine.step.assert_called_once()
        runner.engine.stop.assert_called_once()

    def test_scene_completion_qualification_visible_in_gui(self):
        from PyQt5 import QtWidgets
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with patch.object(ValidationRunner,'start'):window=ValidationWindow('scene','static',42)
        try:
            engine=window.runner.engine;engine.phase='running'
            window.runner.snapshot();app.processEvents();window.flush()
            self.assertIn('CP14 통과 대기',window.status.text())
            engine.completion.checkpoint14_passed=True
            window.runner.snapshot();app.processEvents();window.flush()
            self.assertIn('START/END 복귀 대기',window.status.text())
            engine.phase='preparing';engine.completion.reset()
            window.runner.snapshot();app.processEvents();window.flush()
            self.assertNotIn('START/END 복귀 대기',window.status.text())
        finally:
            window.allow_close=True;window.timer.stop();window.close()

    def test_worker_does_not_fetch_all_actors_again_after_step_already_refreshed_them(self):
        from unittest.mock import Mock
        runner=ValidationRunner('scene','static',42,save_collisions=False)
        link=Mock(connected=True);link.states.return_value=[]
        runner.link=runner.engine.link=link
        runner.monitor.start=Mock();runner.monitor.wait=Mock()
        runner.engine.begin=Mock(side_effect=lambda *_:setattr(runner.engine,'phase','running'))
        runner.engine.step=Mock(side_effect=lambda:(runner.engine.read_states(),None)[1])
        runner.engine.stop=Mock()
        calls=[0]
        def snapshot():
            calls[0]+=1
            if calls[0]==2:runner.commands.put(('close',None,None))
        runner.snapshot=Mock(side_effect=snapshot)
        runner.run()
        self.assertEqual(link.states.call_count,2)  # Initial frame + step frame.

    def test_plot_and_table_objects_are_reused_and_old_labels_are_removed(self):
        from PyQt5 import QtWidgets
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with patch.object(ValidationRunner,'start'):window=ValidationWindow(seed=42)
        try:
            engine=window.runner.engine;points=window.runner.route.curve.xyz
            engine.phase='running';engine.pose=window.runner.route.start('scene','static')
            details={};states=[]
            for index in range(5):
                aid=str(index+1);p=points[index*len(points)//5]
                details[aid]=dict(section='high_speed',model='2015_Kia_K5',target=70,
                                  born_at=time.monotonic()-15,source='6',kind=1)
                states.append(S(actor_info=S(id=S(value=aid),object_type=1),
                    transform=S(location=S(x=float(p[0]),y=float(p[1]),z=float(p[2])),rotation=S(z=90.)),
                    velocity=S(x=55.,y=0.,z=0.),vehicle_state=S(throttle=.4,brake=0.,gear_mode=4,
                        current_link_info=S(id=S(value='A2256W000406')),remaining_distance=100.,tl_id=S(value='none'),tl_color=-2)))
            engine.last_states=states;engine.actor_details=Mock(return_value=details)
            engine.owned={aid:(detail['model'],70) for aid,detail in details.items()}
            frames=[];window.runner.frame.connect(frames.append);window.runner.snapshot()
            frame=frames[-1];window.view='all'
            window.enqueue(frame);window.flush()
            artist_ids=[id(a) for a in window.map_artists]
            cell=window.vehicle_table.item(0,0)
            window.enqueue(frame);window.flush()
            self.assertEqual([id(a) for a in window.map_artists],artist_ids)
            self.assertIs(window.vehicle_table.item(0,0),cell)
            self.assertEqual(len(window.ax.collections),3)
            self.assertEqual(len(window.map_labels),5)
            frame['states']=[s for s in frame['states'] if s['id']!='1']
            window.enqueue(frame);window.flush()
            self.assertNotIn(('1',0),window.map_labels)
            self.assertEqual(len(window.map_labels),4)
        finally:
            window.allow_close=True;window.timer.stop();window.close()

    def test_busy_validator_does_not_pause_or_clean_the_other_validator(self):
        from scen_runner.runtime.validation_lease import ValidationBusy
        from unittest.mock import Mock
        runner=ValidationRunner('scene','static',42,save_collisions=False)
        link=Mock(connected=True);runner.link=link
        runner.engine.lease=Mock(held=False)
        runner.engine.lease.acquire.side_effect=ValidationBusy('already running')
        runner.monitor.start=Mock();runner.monitor.wait=Mock()
        runner.snapshot=Mock(side_effect=lambda:runner.commands.put(('close',None,None)))
        runner.run()
        link.hold.assert_not_called();link.reset.assert_not_called()
        link.release.assert_not_called();link.destroy.assert_not_called()
        self.assertEqual(runner.engine.phase,'stopped')
        self.assertFalse(runner.engine.adapters)

    def test_lock_permission_failure_also_leaves_ego_unmodified(self):
        from unittest.mock import Mock
        runner=ValidationRunner('scene','static',42,save_collisions=False)
        link=Mock(connected=True);runner.link=link
        runner.engine.lease=Mock(held=False)
        runner.engine.lease.acquire.side_effect=PermissionError('lock file')
        runner.monitor.start=Mock();runner.monitor.wait=Mock()
        runner.snapshot=Mock(side_effect=lambda:runner.commands.put(('close',None,None)))
        runner.run()
        link.hold.assert_not_called();link.release.assert_not_called()
        link.destroy.assert_not_called()

    def test_raw_far_contact_is_not_shown_as_red_and_old_preparation_query_is_ignored(self):
        from PyQt5 import QtWidgets
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with patch.object(ValidationRunner,'start'):window=ValidationWindow('scene','static',42)
        def body(aid,x):
            return S(actor_info=S(id=S(value=aid),object_type=1),
                     bounding_box=S(location=S(x=x,y=0,z=1),extent=S(x=1,y=2,z=1)),
                     velocity=S(x=0,y=0,z=0))
        try:
            e=window.runner.engine;e.phase='running';e.run_started=1.;e.owned={'1':('Bollard01',0)}
            sample=dict(epoch=0,error=None,contacts=['1'],owned_ids={'1'},query_started_at=2.,query_s=.01,
                        actor_states=[body('Ego',0),body('1',258)])
            window.runner.receive_contact(sample);window.contacts(sample)
            self.assertIn('먼 객체',window.contact.text());self.assertNotIn('#d32f2f',window.contact.styleSheet())
            sample['actor_states']=[body('Ego',0),body('1',3)];sample['query_started_at']=.5
            window.runner.receive_contact(sample);window.contacts(sample);self.assertIn('주행 시작 전',window.contact.text())
            sample['query_started_at']=3.
            window.runner.receive_contact(sample);window.contacts(sample)
            self.assertIn('기존 접촉 기록 잔류',window.contact.text())
            self.assertNotIn('#d32f2f',window.contact.styleSheet())
            clear=dict(sample,contacts=[],query_started_at=4.)
            window.runner.receive_contact(clear);window.contacts(clear)
            sample['query_started_at']=5.
            window.runner.receive_contact(sample);window.contacts(sample)
            self.assertIn('#d32f2f',window.contact.styleSheet())
        finally:
            window.allow_close=True;window.timer.stop();window.close()

    def test_vehicle_panel_and_map_show_target_actual_and_native_braking(self):
        from PyQt5 import QtWidgets
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with patch.object(ValidationRunner,'start'):window=ValidationWindow('section','high_speed',42)
        try:
            engine=window.runner.engine;engine.names=['high_speed'];engine.pose=window.runner.route.start('section','high_speed')
            detail=dict(section='high_speed',model='2015_Kia_K5',target=80,born_at=time.monotonic()-15,source='6',kind=1)
            owned={'3':('2015_Kia_K5',80)}
            section=S(name='high_speed',prepared=True,status='생성 완료',owned=lambda:owned,
                      actor_details=lambda:{'3':detail},crossing_details=lambda:{})
            engine.adapters=[section];engine.owned=owned;engine.id_owners={'3':section}
            engine.last_states=[S(actor_info=S(id=S(value='3'),object_type=1),
                transform=S(location=S(x=66.,y=200.,z=28.),rotation=S(z=-90.)),
                velocity=S(x=0.,y=-65.,z=0.),vehicle_state=S(throttle=.4,brake=.2,gear_mode=4,
                    current_link_info=S(id=S(value='A2256W000409')),remaining_distance=-1.,tl_id=S(value='none'),tl_color=-2))]
            window.runner.snapshot();app.processEvents();window.flush()
            self.assertEqual(window.vehicle_table.rowCount(),1)
            self.assertEqual(window.vehicle_table.item(0,2).text(),'80')
            self.assertEqual(window.vehicle_table.item(0,3).text(),'65.0')
            self.assertEqual(window.vehicle_table.item(0,5).text(),'0.20')
            self.assertIn('A2256W000409',window.vehicle_table.item(0,0).toolTip())
        finally:
            window.allow_close=True;window.timer.stop();window.close()

    def test_mode_selection_and_close_requests_cancel_worker_without_gui_rpc(self):
        from PyQt5 import QtWidgets
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with patch.object(ValidationRunner,'start'):window=ValidationWindow('section','tunnel',42)
        try:
            engine=window.runner.engine;engine.names=['tunnel'];engine.pose=window.runner.route.start('section','tunnel')
            window.runner.snapshot()
            app.processEvents();window.flush()
            self.assertIn('구간별',window.header.text())
            window.mode.setCurrentIndex(0);window.apply_mode()
            self.assertEqual(window.runner.commands.get_nowait(),('apply','scene','tunnel'))
            self.assertTrue(engine.cancelled.is_set())
            window.runner.request('close')
            self.assertEqual(window.runner.commands.get_nowait(),('close',None,None))
            self.assertFalse(window.runner.link.connected)
        finally:
            window.allow_close=True;window.timer.stop();window.close()

    def test_current_confirmed_environment_and_section_timing_visible(self):
        from PyQt5 import QtWidgets
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with patch.object(ValidationRunner,'start'):window=ValidationWindow('section','dynamic',42)
        try:
            e=window.runner.engine;e.names=['dynamic'];e.pose=window.runner.route.start('section','dynamic')
            e.environment.current=('Sunny',13);e.environment.reason='구간 반복 시작'
            window.runner.snapshot();app.processEvents();window.flush()
            self.assertIn('Sunny',window.environment_status.text())
            self.assertIn('13:00',window.environment_status.text())
            self.assertIn('구간 반복 시작 시 변경',window.environment_status.text())
            e.environment.error='시간 조회 실패'
            window.runner.snapshot();app.processEvents();window.flush()
            self.assertIn('실제 환경 미확인',window.environment_status.text())
        finally:
            window.allow_close=True;window.timer.stop();window.close()
