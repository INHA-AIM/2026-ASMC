import sys
import unittest
import json
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scen_runner.records.analyze_shadow import TopicStats, ScalarStats, native_contact_summary, scenario_outcome_summary, parse_selector, completed_bags


class AnalysisTest(unittest.TestCase):
    def test_receipt_gap_and_source_regression_are_separate(self):
        stat=TopicStats()
        for receipt,source in [(10.,9.),(10.1,9.),(10.5,8.),(10.6,10.)]:
            stat.observe(receipt,source)
        value=stat.summary()
        self.assertAlmostEqual(value['receipt_hz'],5.)
        self.assertAlmostEqual(value['maximum_receipt_gap_s'],.4)
        self.assertEqual(value['duplicate_source_stamps'],1)
        self.assertEqual(value['regressing_source_stamps'],1)

    def test_three_views_and_restarts_do_not_invent_source_regressions(self):
        stat=TopicStats()
        for receipt,source,stream in [(10.,9.9,'front'),(10.1,9.8,'left'),(10.2,9.9,'left'),(10.3,9.85,'front')]:
            stat.observe(receipt,source,stream)
        self.assertEqual(stat.summary()['regressing_source_stamps'],1)
        self.assertEqual(stat.summary()['duplicate_source_stamps'],0)
        for i in range(150):stat.observe(11+i,10+i,'session'+str(i))
        self.assertEqual(stat.summary()['source_stream_count'],128)
        self.assertGreater(stat.summary()['untracked_source_stream_samples'],0)

    def test_active_or_incomplete_split_inventory_is_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            r=Path(root);(r/'ros').mkdir();(r/'ros/a.bag').write_bytes(b'one')
            m=r/'ros/manifest.json'
            m.write_text(json.dumps(dict(status='recording')))
            with self.assertRaises(ValueError):completed_bags(r)
            m.write_text(json.dumps(dict(status='complete',files=[dict(file='a.bag',bytes=3)])))
            self.assertEqual(completed_bags(r),[r/'ros/a.bag'])
            m.write_text(json.dumps(dict(status='incomplete',ended_unix=20,missing_required_topics=['/ctrl_cmd/jamming'],files=[dict(file='a.bag',bytes=3)])))
            with self.assertRaises(ValueError):completed_bags(r)
            self.assertEqual(completed_bags(r,True),[r/'ros/a.bag'])
            m.write_text(json.dumps(dict(status='complete',files=[dict(file='a.bag',bytes=3)])))
            (r/'ros/b.bag.active').touch()
            with self.assertRaises(ValueError):completed_bags(r)
            (r/'ros/b.bag.active').rename(r/'ros/b.bag')
            with self.assertRaises(ValueError):completed_bags(r)

    def test_scenario_lifecycle_is_distinct_from_recording_success(self):
        with tempfile.TemporaryDirectory() as root:
            p=Path(root)/'scenes/run';p.mkdir(parents=True)
            (p/'events.jsonl').write_text('\n'.join(json.dumps(x) for x in [
                dict(event='driving_ready'),dict(event='run_ended',reason='section_end',elapsed_s=100),
                dict(event='runtime_error',error='preparation_timeout'),dict(event='cleanup_confirmed')]))
            result=scenario_outcome_summary(root)
            self.assertEqual(result['counts']['runtime_error'],1)
            self.assertEqual(result['terminal_events'][0]['reason'],'section_end')
            self.assertNotIn('success',result)

    def test_bounded_cost_and_native_query_errors_are_explicit(self):
        stat=ScalarStats(3)
        for n in range(5):stat.observe(n)
        self.assertEqual(stat.summary()['dropped_samples'],2)
        self.assertEqual(stat.summary()['p50'],3.)
        with tempfile.TemporaryDirectory() as root:
            p=Path(root)/'scenes/run';p.mkdir(parents=True)
            (p/'events.jsonl').write_text('\n'.join(json.dumps(x) for x in [
                dict(event='native_contact_sample',contacts=['29'],error=None),
                dict(event='native_contact_sample',contacts=[],error='Channel closed!')]))
            summary=native_contact_summary(root)
            self.assertEqual(summary['counts']['samples_with_contact'],1)
            self.assertEqual(summary['contact_ids'],{'29':1})
            self.assertEqual(summary['query_errors'],{'Channel closed!':1})

    def test_selector_sample_does_not_infer_unknown_source(self):
        self.assertEqual(parse_selector('mode=JAMMING selected_source=/ctrl_cmd/jamming command_fresh=false'),
                         dict(mode='JAMMING',selected_source='/ctrl_cmd/jamming',command_fresh='false'))
        self.assertEqual(parse_selector('unavailable'),{})


if __name__=='__main__':
    unittest.main()
