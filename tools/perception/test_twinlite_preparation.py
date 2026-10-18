import json
from pathlib import Path
import tempfile
import unittest

from finalize_twinlite_captures import partition, report_conditions
from prepare_twinlite_captures import source_path


class PreparationTests(unittest.TestCase):
    def test_collection_session_stays_together_in_all_conditions(self):
        groups = set()
        for run in ('20260904_165112_kcity_comp_sample_b_seg01_sunny_sim11am_wheel',
                    '20260904_170026_kcity_comp_sample_b_seg06_foggy_sim3pm_wheel'):
            groups.add(partition(dict(run_id=run,split_group='run:'+run)))
        self.assertEqual(groups,{('val','capture_session:20260904_comp_sample_b')})
        self.assertEqual(partition(dict(run_id='route_pp_30_test',split_group='layout120')),
                         ('train','layout120'))
        self.assertEqual(partition(dict(run_id='20260903_kcity_comp_sample_foggy_sim11am_wheel',split_group='run')),
                         ('test','capture_session:20260903_20260904_comp_sample'))

    def test_weather_is_reported_not_assumed_and_conflicts_fail(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root/'a.json').write_text(json.dumps(dict(runs=[
                dict(run_id='r',weather='sunny',hour=13),
                dict(run_id='unknown',weather='cloudy',hour=13)])))
            result = report_conditions(root)
            self.assertEqual((result['r']['weather'],result['r']['hour']),('SUNNY',13))
            self.assertNotIn('unknown',result)
            (root/'b.json').write_text(json.dumps(dict(runs=[dict(run_id='r',weather='foggy',hour=13)])))
            with self.assertRaisesRegex(ValueError,'conflicting collection'):
                report_conditions(root)

    def test_source_root_rejects_parent_and_absolute_escape(self):
        with tempfile.TemporaryDirectory() as temp:
            roots = {'raw':Path(temp)}
            self.assertEqual(source_path(roots,dict(root='raw',path='camera/a.png')),Path(temp)/'camera/a.png')
            for path in ('../a.png','/outside/a.png'):
                with self.assertRaises(ValueError):source_path(roots,dict(root='raw',path=path))


if __name__ == '__main__':
    unittest.main()
