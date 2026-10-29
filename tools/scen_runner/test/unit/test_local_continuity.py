import json
from pathlib import Path
import tempfile
import unittest
from scen_runner.records.local_continuity import summarize


class ContinuityTest(unittest.TestCase):
    def test_warmup_and_invalid_plan_are_distinct_from_physical_stop(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            def put(name,records):
                (root/name).write_text(''.join(json.dumps(x)+'\n' for x in records))
            put('events.jsonl',[dict(event='native_sample',ros_time=1,ego=dict(speed_kmh=0)),
                dict(event='ego_released',ros_time=10)]+[
                dict(event='native_sample',ros_time=t,ego=dict(speed_kmh=v))
                for t,v in [(10,0),(11,5),(13,0),(13.8,0),(14,5),(15,5)]])
            put('decisions.jsonl',[dict(decision_stamp=1,valid=False,reason='motion_unavailable'),
                dict(decision_stamp=11,valid=False,scene_valid=True,reason='no_verified_prefix',processing_ms=30),
                dict(decision_stamp=13.5,valid=True,scene_valid=True,reason='ok',processing_ms=25)])
            put('control.jsonl',[dict(decision_stamp=11,valid=False,command_published=True,reason='no_verified_prefix'),
                dict(decision_stamp=13,valid=False,command_published=False,reason='plan_replacement_pending')])
            put('deliveries.jsonl',[dict(received=11,received_steady=100,valid=True),
                dict(received=11.5,received_steady=100.5,valid=False),
                dict(received=12,received_steady=101,valid=True)])
            (root/'trial-result.json').write_text(json.dumps(dict(reason='distance_limit',travel_m=20)))
            r=summarize(root)
            self.assertAlmostEqual(r['stationary_seconds'],.8)
            self.assertEqual(r['input_invalid_plans'],0)
            self.assertEqual(r['stop_command_reasons'],{'no_verified_prefix':1})
            self.assertEqual(r['valid_path_gap_ms']['max'],1000)
            self.assertNotIn('motion_unavailable',r['plan_reasons'])

    def test_preparation_failure_has_no_driving_score(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);(root/'events.jsonl').write_text('')
            self.assertFalse(summarize(root)['driving'])


if __name__=='__main__':unittest.main()
