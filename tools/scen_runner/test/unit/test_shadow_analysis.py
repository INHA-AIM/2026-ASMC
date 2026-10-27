import sys
import unittest
import json
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scen_runner.records.analyze_shadow import TopicStats, parse_selector, completed_bags


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

    def test_active_or_incomplete_split_inventory_is_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            r=Path(root);(r/'ros').mkdir();(r/'ros/a.bag').write_bytes(b'one')
            m=r/'ros/manifest.json'
            m.write_text(json.dumps(dict(status='recording')))
            with self.assertRaises(ValueError):completed_bags(r)
            m.write_text(json.dumps(dict(status='complete',files=[dict(file='a.bag',bytes=3)])))
            self.assertEqual(completed_bags(r),[r/'ros/a.bag'])
            (r/'ros/b.bag.active').touch()
            with self.assertRaises(ValueError):completed_bags(r)
            (r/'ros/b.bag.active').rename(r/'ros/b.bag')
            with self.assertRaises(ValueError):completed_bags(r)

    def test_selector_sample_does_not_infer_unknown_source(self):
        self.assertEqual(parse_selector('mode=JAMMING selected_source=/ctrl_cmd/jamming command_fresh=false'),
                         dict(mode='JAMMING',selected_source='/ctrl_cmd/jamming',command_fresh='false'))
        self.assertEqual(parse_selector('unavailable'),{})


if __name__=='__main__':
    unittest.main()
