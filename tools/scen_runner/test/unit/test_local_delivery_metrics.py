import json
from pathlib import Path
import tempfile
import unittest
from scen_runner.records.analyze_local import analyze


def check_delivery_clock_and_marked_drive_window(tmp_path):
    def rows(name, values):
        (tmp_path/name).write_text(''.join(json.dumps(v)+'\n' for v in values))
    rows('decisions.jsonl', [])
    rows('deliveries.jsonl', [dict(valid=v, reason='ok', received_steady=t)
                            for t,v in [(1,False),(2,True),(2.1,True),(2.4,True)]])
    rows('events.jsonl', [dict(event='driving_started',monotonic=2),
                         dict(event='mode',value=1,monotonic=2.1),
                         dict(event='mode',value=0,monotonic=2.8),
                         dict(event='driving_ended',monotonic=3)])
    rows('wire.jsonl', [dict(received_steady=t) for t in [1,2,2.05,2.1,4]])
    rows('resources.jsonl', [dict(processes=[dict(name='planner',rss_kib=x)]) for x in [10,12,11]])
    result=analyze(tmp_path)
    assert result['plans']==4 and result['valid']==3
    assert abs(result['valid_plan_gap_ms']['max']-300)<1e-6
    assert result['wire_command_receive_gap_ms']['max']>1000
    assert abs(result['driving_wire_command_receive_gap_ms']['max']-50)<1e-6
    assert abs(result['driving_valid_plan_max_silence_ms']-600)<1e-6
    assert abs(result['active_local_valid_plan_max_silence_ms']-400)<1e-6
    assert result['rss_kib']['planner']==dict(first=10,last=11,maximum=12,samples=3)


class DeliveryMetricsTest(unittest.TestCase):
    def test_trial_readiness_waits_for_selected_fresh_wire_command(self):
        from types import SimpleNamespace
        from run_local_trial import selected_command_ready
        m=SimpleNamespace(longlCmdType=1,steering=0.,accel=.1,brake=0.)
        self.assertFalse(selected_command_ready(dict(wire=(10.,m)),10.))
        self.assertFalse(selected_command_ready(dict(mode=0,wire=(10.,m)),10.2))
        m.brake=1.;self.assertFalse(selected_command_ready(dict(mode=0,wire=(10.,m)),10.01))
        m.brake=0.;self.assertTrue(selected_command_ready(dict(mode=0,wire=(10.,m)),10.01))

    def test_collision_sentinel_is_not_contact_but_brief_real_entries_survive(self):
        from types import SimpleNamespace
        from run_local_trial import collision_objects
        def message(values):return SimpleNamespace(collision_object=[
            SimpleNamespace(type=kind,unique_id=identity) for kind,identity in values])
        self.assertEqual(collision_objects(message([(-1,0)])),[])
        self.assertEqual(collision_objects(message([(-1,0),(3,42),(0,7)])),
                         [dict(type=3,id=42),dict(type=0,id=7)])

    def test_full_delivery_clock_and_marked_drive_window(self):
        with tempfile.TemporaryDirectory() as root:
            check_delivery_clock_and_marked_drive_window(Path(root))

    def test_reference_source_and_unissued_brake_are_not_scored_as_family_control(self):
        from scen_runner.records.analyze_local_following import analyze as follow
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            def rows(name,values):
                (root/name).write_text(''.join(json.dumps(v)+'\n' for v in values))
            rows('events.jsonl',[dict(event='native_sample',ros_time=t,
                ego=dict(speed_kmh=10,native_contacts=['Ego'])) for t in [100,102]])
            rows('decisions.jsonl',[dict(valid=True,decision_stamp=100+i,phase='active',
                scene=dict(home=[[-.5,e],[.5,e]]),input_join=dict(reference_source=source))
                for i,e,source in [(0,.1,'family'),(1,2.,'semantic_right_offset_trial')]])
            rows('control.jsonl',[dict(decision_stamp=101,reason='inactive_normal_mode',
                command_published=False,brake=1.)])
            result=follow(root)
            self.assertAlmostEqual(result['family_reference_error_abs_m']['p95'],.1)
            self.assertEqual(result['semantic_reference_error_abs_m']['p95'],2.)
            self.assertEqual(result['hard_brake_ticks'],0)


if __name__=='__main__': unittest.main()
